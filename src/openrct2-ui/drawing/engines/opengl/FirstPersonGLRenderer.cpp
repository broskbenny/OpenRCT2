/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#ifndef DISABLE_OPENGL
#include "FirstPersonGLRenderer.h"
#include "FirstPersonGpuBatching.h"
#include <openrct2/paint/FirstPersonAsync.h>
#include "SwapFramebuffer.h"
#include "OpenGLFramebuffer.h"
#include "TextureCache.h"
#include <openrct2/drawing/Drawing.Sprite.h>
#include <openrct2/profiling/Profiling.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <limits>

namespace OpenRCT2::Ui
{
    namespace
    {
        constexpr char kVertexShader[] = R"GLSL(#version 330 core
layout(location=0) in vec3 aWorld;
layout(location=1) in vec2 aUV;
layout(location=2) in vec4 aAtlas;
layout(location=3) in vec2 aSize;
layout(location=4) in int aAtlasLayer;
layout(location=5) in ivec4 aPalettes;
layout(location=6) in int aFlags;
layout(location=7) in vec4 aMaskAtlas;
layout(location=8) in vec2 aMaskSize;
layout(location=9) in int aMaskLayer;
layout(location=10) in uint aPaintOrdinal;
uniform vec3 uEye;
uniform vec3 uForward;
uniform vec3 uRight;
uniform vec3 uUp;
uniform vec2 uFocal;
uniform vec2 uNearFar;
out vec2 fUV;
out vec3 fWorld;
flat out vec4 fAtlas;
flat out vec2 fSize;
flat out int fAtlasLayer;
flat out ivec4 fPalettes;
flat out int fFlags;
flat out vec4 fMaskAtlas;
flat out vec2 fMaskSize;
flat out int fMaskLayer;
flat out uint fPaintOrdinal;
void main() {
    vec3 delta = aWorld - uEye;
    float x = dot(delta, uRight);
    float y = dot(delta, uUp);
    float z = dot(delta, uForward);
    float n = uNearFar.x;
    float f = uNearFar.y;
    gl_Position = vec4(x*uFocal.x, y*uFocal.y,
                       z*(f+n)/(f-n) - 2.0*f*n/(f-n), z);
    fUV = aUV;
    fWorld = aWorld;
    fAtlas = aAtlas;
    fSize = aSize;
    fAtlasLayer = aAtlasLayer;
    fPalettes = aPalettes;
    fFlags = aFlags;
    fMaskAtlas = aMaskAtlas;
    fMaskSize = aMaskSize;
    fMaskLayer = aMaskLayer;
    fPaintOrdinal = aPaintOrdinal;
}
)GLSL";
        constexpr char kFragmentShader[] = R"GLSL(#version 330 core
uniform usampler2DArray uSprites;
uniform usampler2D uPalettes;
uniform sampler2D uOpaqueDepth;
uniform sampler2D uPreviousDepth;
uniform usampler2D uPreviousLayer;
uniform sampler2D uSelectedPhysicalDepth;
uniform bool uPeeling;
uniform int uPeelStage;
uniform vec3 uEye;
uniform vec3 uForward;
uniform vec2 uNearFar;
in vec2 fUV;
in vec3 fWorld;
flat in vec4 fAtlas;
flat in vec2 fSize;
flat in int fAtlasLayer;
flat in ivec4 fPalettes;
flat in int fFlags;
flat in vec4 fMaskAtlas;
flat in vec2 fMaskSize;
flat in int fMaskLayer;
flat in uint fPaintOrdinal;
layout(location=0) out uint oIndex;

float CoplanarDepthTolerance(float physicalDepth) {
    // Convert a tiny world-space equality tolerance through the same
    // logarithmic depth transform used for gl_FragDepth. This keeps
    // "same physical plane" stable with distance without moving geometry.
    const float worldTolerance = 0.125;
    const float fixedDepthUlpAllowance = 0.00000048;
    float denominator = log2(1.0 + max(uNearFar.y, 1.0));
    float here = log2(1.0 + physicalDepth) / denominator;
    float nearby =
        log2(1.0 + physicalDepth + worldTolerance)
        / denominator;
    return max(
        fixedDepthUlpAllowance,
        nearby - here);
}

void main() {
    // First-person wall semantic quads are authored clockwise from their
    // physical visible side. Keep global culling disabled, but reject the
    // opposite (OpenGL front-facing) side for explicitly single-sided planes.
    if ((fFlags & 64) != 0 && gl_FrontFacing) discard;

    uint col = 0u;
    if ((fFlags & 16) != 0) {
        // Geometry reconstructed from simulation semantics can carry a direct
        // indexed palette colour without fabricating a sprite texture.
        col = uint(fPalettes.w);
    } else {
        bool physicalCoverage = (fFlags & 8) != 0;
        bool materialFallbackOnly = (fFlags & 32) != 0;
        uint materialFallback = uint((fFlags >> 8) & 255);
        bool outside =
            any(lessThan(fUV, vec2(0.0)))
            || any(greaterThanEqual(fUV, fSize));
        if (outside && !physicalCoverage) discard;

        // Never turn an out-of-range projective sample into stretched border
        // artwork. Geometry remains solid, but unowned regions use a stable
        // material index derived from the source sprite.
        if (physicalCoverage && (outside || materialFallbackOnly)) {
            col = materialFallback;
        } else {
            ivec2 p = ivec2(floor(fUV));
            vec2 uv = (fAtlas.xy + vec2(p) + vec2(0.5)) / fAtlas.zw;
            col = texture(uSprites, vec3(uv, float(fAtlasLayer))).r;
            if (col == 0u && physicalCoverage) {
                ivec2 limit = ivec2(fSize);
                for (int radius = 1; radius <= 2 && col == 0u; ++radius) {
                    for (int dy = -radius; dy <= radius && col == 0u; ++dy) {
                        for (int dx = -radius; dx <= radius; ++dx) {
                            if (abs(dx) != radius && abs(dy) != radius) continue;
                            ivec2 q = p + ivec2(dx, dy);
                            if (q.x < 0 || q.y < 0 || q.x >= limit.x || q.y >= limit.y) continue;
                            vec2 neighbourUv =
                                (fAtlas.xy + vec2(q) + vec2(0.5)) / fAtlas.zw;
                            col = texture(
                                uSprites,
                                vec3(neighbourUv, float(fAtlasLayer))).r;
                            if (col != 0u) break;
                        }
                    }
                }
                if (col == 0u)
                    col = materialFallback;
            }
        }
        if (col == 0u) discard;
        if (fMaskLayer >= 0) {
            if (any(lessThan(fUV,vec2(0.0))) || any(greaterThanEqual(fUV,fMaskSize))) discard;
            vec2 m = (fMaskAtlas.xy + floor(fUV) + vec2(0.5)) / fMaskAtlas.zw;
            if (texture(uSprites, vec3(m, float(fMaskLayer))).r == 0u) discard;
        }
    }
    if (col == 0u) discard;
    // Logarithmic physical depth leaves usable precision at the far end of
    // a complete RCT2 park while allowing the passenger near geometry.
    float depth = max(0.0, dot(fWorld - uEye, uForward));
    float logarithmic = log2(1.0 + depth)/log2(1.0 + uNearFar.y);
    gl_FragDepth = logarithmic;
    if ((fFlags & 4) != 0 && (fFlags & 1) == 0) {
        // Coplanar ownership is a logical paint relation, not a geometry
        // offset. Compare against completed ordinary physical depth and snap
        // only the numerically-equal layer to that exact stored depth.
        float opaqueDepthEps =
            CoplanarDepthTolerance(depth);
        float opaqueDepth = texelFetch(
            uOpaqueDepth, ivec2(gl_FragCoord.xy), 0).r;
        if (logarithmic > opaqueDepth + opaqueDepthEps) discard;
        if (abs(logarithmic - opaqueDepth) <= opaqueDepthEps)
        {
            logarithmic = opaqueDepth;
            gl_FragDepth = opaqueDepth;
        }
    }
    if ((fFlags & 1) != 0) {
        float opaqueDepth = texelFetch(
            uOpaqueDepth,ivec2(gl_FragCoord.xy),0).r;
        bool coplanarOwner = (fFlags & 4) != 0;
        if (coplanarOwner) {
            // Ownership permits only the same physical layer. Use the same
            // world-space equality test as opaque semantic owners and snap the
            // numerical key when it is genuinely coplanar.
            float opaqueDepthEps =
                CoplanarDepthTolerance(depth);
            if (logarithmic > opaqueDepth + opaqueDepthEps)
                discard;
            if (abs(logarithmic - opaqueDepth)
                <= opaqueDepthEps)
                logarithmic = opaqueDepth;
        } else {
            // Non-owners remain strict: this is occlusion, not ownership.
            const float opaqueDepthEps = 0.00000002;
            if (logarithmic
                >= opaqueDepth - opaqueDepthEps)
                discard;
        }

        uint row = uint(fPalettes.y);
        // Native water mask changes its palette row with the mask texel.
        if ((fFlags & 2) != 0) row += col - 1u;
        if (row > 254u) discard;

        uint ordinal = min(fPaintOrdinal, 0x00ffffffu);
        if (uPeeling) {
            float previousDepth = texelFetch(uPreviousDepth,ivec2(gl_FragCoord.xy),0).r;
            uint previousToken = texelFetch(uPreviousLayer,ivec2(gl_FragCoord.xy),0).r;
            uint previousOrdinal = previousToken >> 8u;
            // Far-to-near lexicographic remainder:
            // lower physical depth remains after the previous layer; at exactly
            // equal physical depth only a later native paint ordinal remains.
            if (logarithmic > previousDepth) discard;
            if (logarithmic == previousDepth && ordinal <= previousOrdinal) discard;
        }

        if (uPeelStage == 1) {
            // Stage one chooses the farthest REMAINING physical layer only.
            gl_FragDepth = logarithmic;
            oIndex = 1u;
            return;
        }

        if (uPeelStage == 2) {
            // Stage two chooses native paint order only among fragments that
            // occupy the selected physical layer. Geometry/depth is unchanged.
            float selectedDepth = texelFetch(
                uSelectedPhysicalDepth,ivec2(gl_FragCoord.xy),0).r;
            // Physical peel depth is stored as DEPTH_COMPONENT32F. Compare
            // the exact float key selected by stage one, rather than comparing
            // a freshly calculated value with a quantized fixed-point sample.
            if (floatBitsToUint(logarithmic) != floatBitsToUint(selectedDepth)) discard;
            gl_FragDepth = (float(ordinal) + 0.5) / 16777216.0;
            // Low byte stores filter row+1, high 24 bits store paint ordinal.
            oIndex = (ordinal << 8u) | (row + 1u);
            return;
        }

        // Defensive fallback; transparent rendering normally uses the two-stage path.
        oIndex = row + 1u;
        return;
    }
    int count = fPalettes.x;
    if (count >= 3 && col >= 0x2eu && col < 0x3au)
        col = texture(uPalettes, (vec2(float(col + 0xc5u), float(fPalettes.w)) + 0.5)/256.0).r;
    else if (count >= 2 && col >= 0xcau && col < 0xd6u)
        col = texture(uPalettes, (vec2(float(col + 0x29u), float(fPalettes.z)) + 0.5)/256.0).r;
    else if (count >= 1)
        col = texture(uPalettes, (vec2(float(col), float(fPalettes.y)) + 0.5)/256.0).r;
    if (col == 0u) discard;
    oIndex = col;
}
)GLSL";
        // Fullscreen triangle: compose the next deeper-to-nearer palette
        // transformation into the accumulated indexed-colour image.
        constexpr char kComposeVertexShader[] = R"GLSL(#version 330 core
void main() {
    vec2 p=vec2(float((gl_VertexID << 1) & 2),float(gl_VertexID & 2));
    gl_Position=vec4(p*2.0-1.0,0.0,1.0);
}
)GLSL";
        constexpr char kComposeFragmentShader[] = R"GLSL(#version 330 core
uniform usampler2D uAccumulated;
uniform usampler2D uLayer;
uniform usampler2D uPalettes;
layout(location=0) out uint oIndex;
void main() {
    ivec2 xy=ivec2(gl_FragCoord.xy);
    uint background=texelFetch(uAccumulated,xy,0).r;
    uint encoded=texelFetch(uLayer,xy,0).r;
    uint encodedRow=encoded & 0xffu;
    if(encodedRow==0u){oIndex=background;return;}
    uint row=encodedRow-1u;
    oIndex=texelFetch(uPalettes,ivec2(int(background),int(row)),0).r;
}
)GLSL";
        struct GPUVertex
        {
            float world[3];
            float uv[2];
            float atlas[4];
            float size[2];
            int32_t atlasLayer;
            int32_t palettes[4];
            int32_t flags;
            float maskAtlas[4];
            float maskSize[2];
            int32_t maskLayer;
            uint32_t paintOrdinal;
        };
        void ConfigureVertexInput(GLuint vao, GLuint vbo)
        {
        glCall(glBindVertexArray, vao);
        glCall(glBindBuffer, GL_ARRAY_BUFFER, vbo);
        glCall(glEnableVertexAttribArray, 0);
        glCall(glVertexAttribPointer, 0, 3, GL_FLOAT, GL_FALSE, sizeof(GPUVertex), reinterpret_cast<void*>(offsetof(GPUVertex, world)));
        glCall(glEnableVertexAttribArray, 1);
        glCall(glVertexAttribPointer, 1, 2, GL_FLOAT, GL_FALSE, sizeof(GPUVertex), reinterpret_cast<void*>(offsetof(GPUVertex, uv)));
        glCall(glEnableVertexAttribArray, 2);
        glCall(glVertexAttribPointer, 2, 4, GL_FLOAT, GL_FALSE, sizeof(GPUVertex), reinterpret_cast<void*>(offsetof(GPUVertex, atlas)));
        glCall(glEnableVertexAttribArray, 3);
        glCall(glVertexAttribPointer, 3, 2, GL_FLOAT, GL_FALSE, sizeof(GPUVertex), reinterpret_cast<void*>(offsetof(GPUVertex, size)));
        glCall(glEnableVertexAttribArray, 4);
        glCall(glVertexAttribIPointer, 4, 1, GL_INT, sizeof(GPUVertex), reinterpret_cast<void*>(offsetof(GPUVertex, atlasLayer)));
        glCall(glEnableVertexAttribArray, 5);
        glCall(glVertexAttribIPointer, 5, 4, GL_INT, sizeof(GPUVertex), reinterpret_cast<void*>(offsetof(GPUVertex, palettes)));
        glCall(glEnableVertexAttribArray, 6);
        glCall(glVertexAttribIPointer, 6, 1, GL_INT, sizeof(GPUVertex), reinterpret_cast<void*>(offsetof(GPUVertex, flags)));
        glCall(glEnableVertexAttribArray, 7);
        glCall(glVertexAttribPointer, 7, 4, GL_FLOAT, GL_FALSE, sizeof(GPUVertex), reinterpret_cast<void*>(offsetof(GPUVertex, maskAtlas)));
        glCall(glEnableVertexAttribArray, 8);
        glCall(glVertexAttribPointer, 8, 2, GL_FLOAT, GL_FALSE, sizeof(GPUVertex), reinterpret_cast<void*>(offsetof(GPUVertex, maskSize)));
        glCall(glEnableVertexAttribArray, 9);
        glCall(glVertexAttribIPointer, 9, 1, GL_INT, sizeof(GPUVertex), reinterpret_cast<void*>(offsetof(GPUVertex, maskLayer)));
        glCall(glEnableVertexAttribArray, 10);
        glCall(glVertexAttribIPointer, 10, 1, GL_UNSIGNED_INT, sizeof(GPUVertex), reinterpret_cast<void*>(offsetof(GPUVertex, paintOrdinal)));
        }
        GLuint Compile(GLenum type, const char* source)
        {
            const GLuint shader = glCall(glCreateShader, type);
            glCall(glShaderSource, shader, 1, &source, nullptr);
            glCall(glCompileShader, shader);
            GLint status{};
            glCall(glGetShaderiv, shader, GL_COMPILE_STATUS, &status);
            if (status == GL_TRUE) return shader;
            GLint length{};
            glCall(glGetShaderiv, shader, GL_INFO_LOG_LENGTH, &length);
            std::string log(size_t(std::max(length, 1)), '\0');
            glCall(glGetShaderInfoLog, shader, length, nullptr, log.data());
            glCall(glDeleteShader, shader);
            throw std::runtime_error("First-person shader compilation failed: " + log);
        }
        GLint Uniform(GLuint shader, const char* name)
        {
            return glCall(glGetUniformLocation, shader, name);
        }
        uint8_t DominantOpaqueSpritePixel(const G1Element& g1)
        {
            if (g1.offset == nullptr || g1.width <= 0 || g1.height <= 0
                || g1.flags.has(G1Flag::isPalette))
                return 1;

            std::array<uint32_t, 256> counts{};
            const auto countPixel = [&](uint8_t pixel) {
                if (pixel != 0)
                    ++counts[pixel];
            };
            if (g1.flags.has(G1Flag::hasRLECompression))
            {
                for (int32_t y = 0; y < g1.height; ++y)
                {
                    const uint16_t lineOffset =
                        uint16_t(g1.offset[y * 2])
                        | (uint16_t(g1.offset[y * 2 + 1]) << 8);
                    const uint8_t* run = g1.offset + lineOffset;
                    bool end = false;
                    size_t guard = 0;
                    while (!end && guard++ < 256)
                    {
                        uint8_t length = *run++;
                        run++;
                        end = (length & 0x80u) != 0;
                        length &= 0x7fu;
                        for (uint8_t i = 0; i < length; ++i)
                            countPixel(run[i]);
                        run += length;
                    }
                }
            }
            else
            {
                const size_t count =
                    size_t(g1.width) * size_t(g1.height);
                for (size_t i = 0; i < count; ++i)
                    countPixel(g1.offset[i]);
            }

            uint8_t best = 1;
            uint32_t bestCount = 0;
            for (uint16_t pixel = 1; pixel < counts.size(); ++pixel)
            {
                if (counts[pixel] > bestCount)
                {
                    bestCount = counts[pixel];
                    best = uint8_t(pixel);
                }
            }
            return best;
        }

        int32_t PaletteY(Drawing::FilterPaletteID id)
        {
            return TextureCache::PaletteToY(id);
        }
    } // namespace

    FirstPersonGLRenderer::FirstPersonGLRenderer()
    {
        const GLuint vs = Compile(GL_VERTEX_SHADER, kVertexShader);
        const GLuint fs = Compile(GL_FRAGMENT_SHADER, kFragmentShader);
        _program = glCall(glCreateProgram);
        glCall(glAttachShader, _program, vs);
        glCall(glAttachShader, _program, fs);
        glCall(glLinkProgram, _program);
        GLint status{};
        glCall(glGetProgramiv, _program, GL_LINK_STATUS, &status);
        glCall(glDeleteShader, vs);
        glCall(glDeleteShader, fs);
        if (status != GL_TRUE) throw std::runtime_error("First-person shader linking failed");
        const GLuint cv = Compile(GL_VERTEX_SHADER, kComposeVertexShader);
        const GLuint cf = Compile(GL_FRAGMENT_SHADER, kComposeFragmentShader);
        _composeProgram = glCall(glCreateProgram);
        glCall(glAttachShader, _composeProgram, cv);
        glCall(glAttachShader, _composeProgram, cf);
        glCall(glLinkProgram, _composeProgram);
        glCall(glGetProgramiv, _composeProgram, GL_LINK_STATUS, &status);
        glCall(glDeleteShader, cv);
        glCall(glDeleteShader, cf);
        if (status != GL_TRUE) throw std::runtime_error("First-person compositor linking failed");
        glCall(glGenQueries, GLsizei(_peelCoverageQueries.size()), _peelCoverageQueries.data());
        glCall(glGenVertexArrays, 1, &_vao);
        glCall(glGenBuffers, 1, &_vbo);
        glCall(glGenBuffers, 1, &_ebo);
        ConfigureVertexInput(_vao,_vbo);
    }
    void FirstPersonGLRenderer::DiscardRegionBuffer(uint64_t key)
    {
        auto it=_staticOpaqueRegions.find(key);
        if(it==_staticOpaqueRegions.end()) return;
        glCall(glDeleteBuffers,1,&it->second.vbo);
        glCall(glDeleteVertexArrays,1,&it->second.vao);
        _staticRegionBytes-=it->second.bytes;
        _staticOpaqueRegions.erase(it);
    }
    void FirstPersonGLRenderer::DiscardPendingRegion()
    {
        if (!_pendingRegion) return;
        glCall(glDeleteBuffers, 1, &_pendingRegion->buffer.vbo);
        glCall(glDeleteVertexArrays, 1, &_pendingRegion->buffer.vao);
        _pendingRegion.reset();
    }
    void FirstPersonGLRenderer::DiscardAllRegionBuffers()
    {
        DiscardPendingRegion();
        while(!_staticOpaqueRegions.empty())
            DiscardRegionBuffer(_staticOpaqueRegions.begin()->first);
    }
    FirstPersonGLRenderer::~FirstPersonGLRenderer()
    {
        DiscardAllRegionBuffers();
        glCall(glDeleteBuffers, 1, &_ebo);
        glCall(glDeleteBuffers, 1, &_vbo);
        glCall(glDeleteVertexArrays, 1, &_vao);
        glCall(glDeleteProgram, _program);
        glCall(glDeleteProgram, _composeProgram);
        glCall(glDeleteQueries,GLsizei(_peelCoverageQueries.size()),_peelCoverageQueries.data());
    }
    void FirstPersonGLRenderer::Draw(
        const Paint::FirstPersonScene& scene, TextureCache& textures,
        SwapFramebuffer& output, int32_t screenWidth, int32_t screenHeight,
        const ScreenRect& dirtyClip)
    {
        const int32_t left = scene.screenOrigin.x;
        const int32_t top = scene.screenOrigin.y;
        const int32_t width = scene.dimensions.width;
        const int32_t height = scene.dimensions.height;
        if (width <= 0 || height <= 0)
            return;

        const auto clip = Paint::IntersectFirstPersonScreenRects(
            ScreenRect{ left, top, left + width, top + height },
            dirtyClip);
        if (clip.getWidth() <= 0 || clip.getHeight() <= 0)
            return;

        // Outside an explicit presentation frame, preserve the old immediate
        // behaviour. The OpenGL window path supplies a nonzero serial and can
        // safely reuse one fully rendered viewport for all dirty clips.
        if (scene.presentationFrameSerial == 0)
        {
            RenderScene(
                scene, textures, output,
                screenWidth, screenHeight, dirtyClip);
            return;
        }

        const bool rebuild =
            _preparedOutput == nullptr
            || _preparedSceneSerial
                != scene.presentationFrameSerial
            || _preparedScreenWidth != screenWidth
            || _preparedScreenHeight != screenHeight
            || _preparedViewportLeft != left
            || _preparedViewportTop != top
            || _preparedViewportWidth != width
            || _preparedViewportHeight != height;
        if (rebuild)
        {
            if (_preparedOutput == nullptr
                || _preparedScreenWidth != screenWidth
                || _preparedScreenHeight != screenHeight)
            {
                _preparedOutput =
                    std::make_unique<SwapFramebuffer>(
                        screenWidth, screenHeight);
            }
            _preparedOutput->Clear();
            RenderScene(
                scene, textures, *_preparedOutput,
                screenWidth, screenHeight,
                ScreenRect{
                    left, top, left + width, top + height
                });
            _preparedSceneSerial =
                scene.presentationFrameSerial;
            _preparedScreenWidth = screenWidth;
            _preparedScreenHeight = screenHeight;
            _preparedViewportLeft = left;
            _preparedViewportTop = top;
            _preparedViewportWidth = width;
            _preparedViewportHeight = height;
        }

        auto& destination = output.GetFinalFramebuffer();
        auto& source =
            _preparedOutput->GetFinalFramebuffer();
        destination.BindDraw();
        source.BindRead();

        const int32_t x0 = clip.getLeft();
        const int32_t x1 = clip.getRight();
        const int32_t y0 =
            screenHeight - clip.getBottom();
        const int32_t y1 =
            screenHeight - clip.getTop();
        glCall(
            glBlitFramebuffer,
            x0, y0, x1, y1,
            x0, y0, x1, y1,
            GL_COLOR_BUFFER_BIT, GL_NEAREST);

        // Match RenderScene's final state in the real target: first-person
        // physical depth must never leak into later 2-D/window drawing, and
        // pixels outside this dirty clip remain untouched.
        destination.Bind();
        glCall(glEnable, GL_SCISSOR_TEST);
        glCall(
            glScissor,
            x0, y0,
            clip.getWidth(), clip.getHeight());
        glCall(glDepthMask, GL_TRUE);
        glCall(glClear, GL_DEPTH_BUFFER_BIT);
        glCall(glDisable, GL_SCISSOR_TEST);
        glCall(glViewport, 0, 0, screenWidth, screenHeight);
        glCall(glBindVertexArray, 0);
        OpenGLState::Reset();
    }

    void FirstPersonGLRenderer::RenderScene(
        const Paint::FirstPersonScene& scene, TextureCache& textures,
        SwapFramebuffer& output, int32_t screenWidth, int32_t screenHeight,
        const ScreenRect& dirtyClip)
    {
        PROFILED_FUNCTION();
        const int32_t left = scene.screenOrigin.x;
        const int32_t top = scene.screenOrigin.y;
        const int32_t width = scene.dimensions.width;
        const int32_t height = scene.dimensions.height;
        if (width <= 0 || height <= 0) return;
        if (_sceneEpoch != scene.sceneEpoch)
        {
            DiscardAllRegionBuffers();
            textures.ClearFirstPersonPersistentBitmaps();
            _sceneEpoch = scene.sceneEpoch;
        }
        textures.ClearFirstPersonTransientBitmaps();
        const auto clip = Paint::IntersectFirstPersonScreenRects(
            ScreenRect{ left, top, left + width, top + height },
            dirtyClip);
        if (clip.getWidth() <= 0 || clip.getHeight() <= 0)
            return;
        const int32_t clipLeft = clip.getLeft();
        const int32_t clipTop = clip.getTop();
        const int32_t clipRight = clip.getRight();
        const int32_t clipBottom = clip.getBottom();
        const int32_t clipWidth = clip.getWidth();
        const int32_t clipHeight = clip.getHeight();
        const auto pollPeelCoverageQueries =
            [&](uint64_t token) {
                bool exhausted = false;
                for (size_t i = 0;
                     i < _peelCoverageQueries.size(); ++i)
                {
                    if (!_peelCoveragePending[i])
                        continue;
                    GLuint ready = 0;
                    glCall(
                        glGetQueryObjectuiv,
                        _peelCoverageQueries[i],
                        GL_QUERY_RESULT_AVAILABLE, &ready);
                    if (ready == 0)
                        continue;
                    GLuint anySamples = 1;
                    glCall(
                        glGetQueryObjectuiv,
                        _peelCoverageQueries[i],
                        GL_QUERY_RESULT, &anySamples);
                    if (_peelCoverageTokens[i] == token
                        && anySamples == 0)
                        exhausted = true;
                    _peelCoveragePending[i] = false;
                }
                return exhausted;
            };
        const auto findFreePeelCoverageQuery =
            [&]() {
                for (size_t i = 0;
                     i < _peelCoverageQueries.size(); ++i)
                {
                    if (!_peelCoveragePending[i])
                        return i;
                }
                return _peelCoverageQueries.size();
            };
        const auto waitForPeelCoverageQuery =
            [&](uint64_t token) {
                size_t slot = _peelCoverageQueries.size();
                // Prefer a probe from this tile: a zero result then proves
                // that every later exact peel layer is empty.
                for (size_t i = 0;
                     i < _peelCoverageQueries.size(); ++i)
                {
                    if (_peelCoveragePending[i]
                        && _peelCoverageTokens[i] == token)
                    {
                        slot = i;
                        break;
                    }
                }
                if (slot == _peelCoverageQueries.size())
                {
                    for (size_t i = 0;
                         i < _peelCoverageQueries.size(); ++i)
                    {
                        if (_peelCoveragePending[i])
                        {
                            slot = i;
                            break;
                        }
                    }
                }
                if (slot == _peelCoverageQueries.size())
                    return false;

                GLuint anySamples = 1;
                glCall(
                    glGetQueryObjectuiv,
                    _peelCoverageQueries[slot],
                    GL_QUERY_RESULT, &anySamples);
                const bool exhausted =
                    _peelCoverageTokens[slot] == token
                    && anySamples == 0;
                _peelCoveragePending[slot] = false;
                return exhausted;
            };
        // Retire completed probes from earlier tiles/frames without waiting.
        (void)pollPeelCoverageQueries(0);

        // We are inside an OpenRCT2 onDraw callback. Earlier 2-D commands have been flushed.
        output.BindOpaque();
        glCall(glEnable, GL_SCISSOR_TEST);
        glCall(glScissor, clipLeft, screenHeight - clipBottom, clipWidth, clipHeight);
        // The existing 2-D transparency pass may have changed depth state.
        // Depth clears obey the depth write mask: enable it BEFORE clearing.
        glCall(glDepthMask, GL_TRUE);
        const GLuint sky[4] = { 136u, 0u, 0u, 0u };
        glCall(glClearBufferuiv, GL_COLOR, 0, sky);
        glCall(glClear, GL_DEPTH_BUFFER_BIT);
        glCall(glViewport, left, screenHeight - top - height, width, height);
        glCall(glEnable, GL_DEPTH_TEST);
        glCall(glDepthFunc, GL_LEQUAL);
        glCall(glDisable, GL_BLEND);
        glCall(glDisable, GL_CULL_FACE); // terrain triangles and near-camera sprites are double-sided
        glCall(glUseProgram, _program);
        const auto& view = scene.resolvedView;
        const auto b = Paint::GetFirstPersonBasis(view.camera);
        const auto& eye = view.camera.position;
        glCall(glUniform3f, Uniform(_program, "uEye"), eye.x, eye.y, eye.z);
        glCall(glUniform3f, Uniform(_program, "uForward"), b.forward.x, b.forward.y, b.forward.z);
        glCall(glUniform3f, Uniform(_program, "uRight"), b.right.x, b.right.y, b.right.z);
        glCall(glUniform3f, Uniform(_program, "uUp"), b.up.x, b.up.y, b.up.z);
        const float halfFov = std::clamp(view.fieldOfViewDegrees, 30.0f, 120.0f)
            * 3.14159265358979323846f / 360.0f;
        const float foc = 1.0f / std::tan(halfFov);
        glCall(glUniform2f, Uniform(_program, "uFocal"), foc, foc * float(width) / float(height));
        glCall(glUniform2f, Uniform(_program, "uNearFar"), view.nearClip, view.farClip);
        OpenGLAPI::SetTexture(0, GL_TEXTURE_2D_ARRAY, textures.GetAtlasesTexture());
        OpenGLAPI::SetTexture(1, GL_TEXTURE_2D, textures.GetPaletteTexture());
        glCall(glUniform1i, Uniform(_program, "uSprites"), 0);
        glCall(glUniform1i, Uniform(_program, "uPalettes"), 1);
        glCall(glUniform1i, Uniform(_program, "uOpaqueDepth"), 2);
        glCall(glUniform1i, Uniform(_program, "uPreviousDepth"), 3);
        glCall(glUniform1i, Uniform(_program, "uPreviousLayer"), 4);
        glCall(glUniform1i, Uniform(_program, "uSelectedPhysicalDepth"), 5);
        glCall(glUniform1i, Uniform(_program, "uPeeling"), 0);
        glCall(glUniform1i, Uniform(_program, "uPeelStage"), 0);

        // Stable fixed world geometry belongs to resident GPU regions. Camera-
        // facing impostors, guests, cars and transparency remain streamed.
        // Unlike a single global cache, turning across a region boundary need
        // not upload every unchanged terrain tile and ordinary wall again.
        ++_frame;
        constexpr size_t kRegionGpuLimit = 96u * 1024u * 1024u;
        std::vector<const Paint::FirstPersonSurface*> streamedOpaque;
        std::vector<const Paint::FirstPersonSurface*> streamedTransparent;
        for (const auto& surface:scene.surfaces)
        {
            if (surface.solidColour != 0)
            {
                streamedOpaque.push_back(&surface);
                continue;
            }
            if (!surface.image.HasValue()) continue;
            if (surface.image.IsBlended()) streamedTransparent.push_back(&surface);
            else streamedOpaque.push_back(&surface);
        }
        // Native texture-cache atlas handles can change on sprite loads or
        // game-art reload. Preload unique image/mask IDs BEFORE constructing
        // any resident buffer; never retain stale atlas UV/layer metadata.
        const uint64_t textureRevisionSerial = textures.GetImageTextureRevisionSerial();
        Paint::FirstPersonFrameBudget uploadBudget(512 * 1024, std::chrono::microseconds(2000));
        const GLuint currentAtlas = textures.GetAtlasesTexture();
        if (_atlasHandle != currentAtlas)
        {
            DiscardAllRegionBuffers();
            _coverageFallbackCache.clear();
            _atlasHandle = currentAtlas;
        }
        std::unordered_map<uint64_t, BasicTextureInfo> immutableTextures;
        immutableTextures.reserve(scene.surfaces.size() / 16 + 1);
        auto prepareMaterial = [&](const Paint::FirstPersonSurface& surface) {
            const auto image = surface.image;
            if (surface.hasImmutablePixelData() && (surface.immutableWidth <= 0 || surface.immutableHeight <= 0)) return true;
            const auto prepareImage = [&](ImageId id) {
                if (!id.HasValue() || textures.HasImageTexture(id)) return true;
                const auto* source = GfxGetG1Element(id);
                if (source == nullptr || source->width <= 0 || source->height <= 0) return true;
                if (!uploadBudget.take(size_t(source->width) * size_t(source->height))) return false;
                textures.GetOrLoadImageTexture(id);
                return true;
            };
            if (surface.solidColour == 0)
            {
                if (surface.hasImmutablePixelData())
                {
                    const bool loaded = surface.persistentBitmap
                        ? textures.HasFirstPersonPersistentBitmap(surface.immutableFingerprint)
                        : immutableTextures.contains(surface.immutableFingerprint);
                    if (!loaded)
                    {
                        if (!uploadBudget.take(size_t(surface.immutableWidth) * size_t(surface.immutableHeight))) return false;
                        const auto& pixels = surface.immutablePixelData();
                        if (surface.persistentBitmap)
                            textures.GetOrLoadFirstPersonPersistentBitmap(surface.immutableFingerprint, pixels.data(),
                                size_t(surface.immutableWidth), size_t(surface.immutableHeight));
                        else
                            immutableTextures.emplace(surface.immutableFingerprint,
                                textures.LoadFirstPersonTransientBitmap(pixels.data(),
                                    size_t(surface.immutableWidth), size_t(surface.immutableHeight)));
                    }
                }
                else if (!prepareImage(image)) return false;
                if (!prepareImage(surface.mask)) return false;
            }
            return true;
        };
        // Admit streamed materials before any draw calls; transparent scenery
        // must not starve merely because opaque drawing used the time slice.
        static size_t materialCursor = 0;
        if (!scene.surfaces.empty())
        {
            const size_t start = materialCursor % scene.surfaces.size();
            size_t examined = 0;
            while (examined < scene.surfaces.size() && uploadBudget.available())
                prepareMaterial(scene.surfaces[(start + examined++) % scene.surfaces.size()]);
            materialCursor = start + examined;
        }
        auto appendVertices = [&](
            std::vector<GPUVertex>& vertices,
            const Paint::FirstPersonSurface& surface,
            bool forceCoplanarOwner = false,
            bool suppressCoplanarOwner = false) {
            const auto image=surface.image;
            if (!prepareMaterial(surface)) return false;
            const auto* g1 =
                surface.solidColour == 0
                    && (!surface.hasImmutablePixelData()
                        || surface.physicalCoverage)
                ? GfxGetG1Element(image)
                : nullptr;
            BasicTextureInfo tex{};
            float imageWidth = 0.0f;
            float imageHeight = 0.0f;
            if (surface.solidColour != 0)
            {
                // The shader ignores atlas/UV fields for direct palette geometry.
                imageWidth = imageHeight = 1.0f;
            }
            else if (surface.hasImmutablePixelData())
            {
                if (surface.immutableWidth <= 0 || surface.immutableHeight <= 0)
                    return true;
                const auto& pixelData =
                    surface.immutablePixelData();
                if (surface.persistentBitmap)
                {
                    tex = textures.GetOrLoadFirstPersonPersistentBitmap(
                        surface.immutableFingerprint,
                        pixelData.data(),
                        size_t(surface.immutableWidth),
                        size_t(surface.immutableHeight));
                }
                else
                {
                    auto it = immutableTextures.find(surface.immutableFingerprint);
                    if (it == immutableTextures.end())
                    {
                        const auto loaded = textures.LoadFirstPersonTransientBitmap(
                            pixelData.data(),
                            size_t(surface.immutableWidth), size_t(surface.immutableHeight));
                        it = immutableTextures.emplace(
                            surface.immutableFingerprint, loaded).first;
                    }
                    tex = it->second;
                }
                imageWidth = float(surface.immutableWidth);
                imageHeight = float(surface.immutableHeight);
            }
            else
            {
                if(g1==nullptr || g1->width<=0 || g1->height<=0) return true;
                tex=textures.GetOrLoadImageTexture(image);
                imageWidth = float(g1->width);
                imageHeight = float(g1->height);
            }
            int32_t coverageFallback = 0;
            if (surface.physicalCoverage && g1 != nullptr)
            {
                const auto imageIndex = image.GetIndex();
                const uint64_t imageRevision =
                    textures.GetImageTextureRevision(imageIndex);
                auto [it, inserted] =
                    _coverageFallbackCache.try_emplace(
                        imageIndex);
                if (inserted
                    || it->second.imageRevision
                        != imageRevision)
                {
                    it->second.imageRevision =
                        imageRevision;
                    it->second.paletteIndex =
                        DominantOpaqueSpritePixel(*g1);
                }
                coverageFallback =
                    int32_t(it->second.paletteIndex);
            }
            BasicTextureInfo maskTex{};
            const auto* maskG1=surface.mask.HasValue()?GfxGetG1Element(surface.mask):nullptr;
            if(maskG1!=nullptr) maskTex=textures.GetOrLoadImageTexture(surface.mask);
            int32_t count=0,palettes[3]={};
            if(surface.solidColour == 0 && image.HasSecondary())
            {
                count=image.HasTertiary()?3:2;
                palettes[0]=PaletteY(static_cast<Drawing::FilterPaletteID>(image.GetPrimary()));
                palettes[1]=PaletteY(static_cast<Drawing::FilterPaletteID>(image.GetSecondary()));
                palettes[2]=PaletteY(static_cast<Drawing::FilterPaletteID>(image.GetTertiary()));
            }
            else if(surface.solidColour == 0 && (image.IsRemap() || image.IsBlended()))
            {
                count=1;
                palettes[0]=PaletteY(static_cast<Drawing::FilterPaletteID>(image.GetRemap()));
            }
            for(const auto& p:surface.triangles)
                vertices.push_back({
                    {p.world.x,p.world.y,p.world.z},{p.u,p.v},
                    {tex.coords.x,tex.coords.y,tex.coords.z,tex.coords.w},
                    {imageWidth,imageHeight},int32_t(tex.index),
                    {count,palettes[0],palettes[1],
                     surface.solidColour != 0 ? int32_t(surface.solidColour) : palettes[2]},
                    (surface.solidColour == 0 && image.IsBlended()
                        ? (image.GetRemap()==static_cast<uint8_t>(Drawing::FilterPaletteID::paletteWater)?3:1)
                        : 0)
                        | ((forceCoplanarOwner
                                || (!suppressCoplanarOwner
                                    && surface.coplanarOwner))
                            ? 4 : 0)
                        | (surface.physicalCoverage ? 8 : 0)
                        | (surface.solidColour != 0 ? 16 : 0)
                        | (surface.textureFallbackOnly ? 32 : 0)
                        | (surface.singleSided ? 64 : 0)
                        | (coverageFallback << 8),
                    {maskTex.coords.x,maskTex.coords.y,maskTex.coords.z,maskTex.coords.w},
                    {maskG1?float(maskG1->width):0.0f,maskG1?float(maskG1->height):0.0f},
                    maskG1?int32_t(maskTex.index):-1,
                    uint32_t(std::min<uint64_t>(
                        surface.nativePaintOrdinal,
                        0x00ffffffull))});
            return true;
        };
        std::vector<GPUVertex> opaqueOrdinaryVertices;
        std::vector<GPUVertex> opaqueOwnerVertices;
        std::vector<const Paint::FirstPersonSurface*>
            streamedOwners;
        for (const auto* surface : streamedOpaque)
        {
            appendVertices(
                opaqueOrdinaryVertices, *surface,
                false, true);
            if (surface->coplanarOwner)
                streamedOwners.push_back(surface);
        }
        std::stable_sort(
            streamedOwners.begin(), streamedOwners.end(),
            [](const auto* a, const auto* second) {
                const uint64_t ao =
                    a->nativePaintOrdinal == 0
                    ? std::numeric_limits<uint64_t>::max()
                    : a->nativePaintOrdinal;
                const uint64_t bo =
                    second->nativePaintOrdinal == 0
                    ? std::numeric_limits<uint64_t>::max()
                    : second->nativePaintOrdinal;
                return ao < bo;
            });
        for (const auto* surface : streamedOwners)
            appendVertices(
                opaqueOwnerVertices, *surface,
                true, false);
        std::vector<uint64_t> regionDraws;
        regionDraws.reserve(scene.staticRegions.size());
        // CPU packing/drawing cached content does not consume the next upload
        // phase. Both phases share one 512 KiB byte allowance.
        uploadBudget.beginPhase(std::chrono::microseconds(2000));
        // The cache target controls unseen residents. The visible working set
        // plus one staged replacement must fit: the previous unbounded stream
        // fallback stalled the frame, while a hard cap here would leave dense
        // visible regions permanently absent.
        size_t visibleBytes = 0, largestPacketBytes = 0;
        for (const auto& packet : scene.staticRegions)
        {
            const auto bytes = packet.vertexCount * sizeof(GPUVertex);
            visibleBytes += bytes;
            largestPacketBytes = std::max(largestPacketBytes, bytes);
        }
        const size_t regionBudget = std::max(kRegionGpuLimit, visibleBytes + largestPacketBytes);
        // Protect every visible resident before considering LRU eviction.
        for (const auto& packet : scene.staticRegions)
            if (const auto it = _staticOpaqueRegions.find(packet.key); it != _staticOpaqueRegions.end())
                it->second.lastSeen = _frame;
        if (_pendingRegion)
        {
            const auto packet = std::find_if(scene.staticRegions.begin(), scene.staticRegions.end(),
                [&](const auto& p) { return p.key == _pendingRegion->key; });
            if (packet == scene.staticRegions.end() || packet->sourceRevision != _pendingRegion->buffer.sourceRevision
                || _pendingRegion->buffer.dependencyRevisionSerial != textureRevisionSerial)
                DiscardPendingRegion();
        }
        for (const auto& packet : scene.staticRegions)
        {
            auto found = _staticOpaqueRegions.find(packet.key);
            const bool ready = found != _staticOpaqueRegions.end()
                && found->second.sceneEpoch == packet.sceneEpoch
                && found->second.generation == packet.generation
                && found->second.dependencyRevisionSerial == textureRevisionSerial;
            if (!ready && !_pendingRegion && packet.surfaceStorage && uploadBudget.available())
            {
                const auto bytes = packet.vertexCount * sizeof(GPUVertex);
                // Replacement must be possible even when there is no room for
                // both versions. Drop the obsolete resident and let it pop in
                // again after the bounded upload, instead of stalling forever.
                if (bytes <= regionBudget && _staticRegionBytes + bytes > regionBudget
                    && found != _staticOpaqueRegions.end())
                {
                    DiscardRegionBuffer(packet.key);
                    found = _staticOpaqueRegions.end();
                }
                while (_staticRegionBytes + bytes > regionBudget)
                {
                    auto victim = _staticOpaqueRegions.end();
                    for (auto it = _staticOpaqueRegions.begin(); it != _staticOpaqueRegions.end(); ++it)
                        if (it->second.lastSeen != _frame && (victim == _staticOpaqueRegions.end()
                            || it->second.lastSeen < victim->second.lastSeen)) victim = it;
                    if (victim == _staticOpaqueRegions.end()) break;
                    DiscardRegionBuffer(victim->first);
                }
                if (bytes > 0 && _staticRegionBytes + bytes <= regionBudget
                    && packet.surfaces->size() <= size_t(std::numeric_limits<GLsizei>::max()) / 12)
                {
                    _pendingRegion = std::make_unique<PendingRegion>();
                    auto& pending = *_pendingRegion;
                    pending.key = packet.key;
                    pending.surfaces = packet.surfaceStorage;
                    auto& buffer = pending.buffer;
                    buffer.sceneEpoch = packet.sceneEpoch;
                    buffer.generation = packet.generation;
                    buffer.sourceRevision = packet.sourceRevision;
                    buffer.dependencyRevisionSerial = textureRevisionSerial;
                    buffer.bytes = bytes;
                    glCall(glGenVertexArrays, 1, &buffer.vao);
                    glCall(glGenBuffers, 1, &buffer.vbo);
                    ConfigureVertexInput(buffer.vao, buffer.vbo);
                    glCall(glBindBuffer, GL_ARRAY_BUFFER, buffer.vbo);
                    glCall(glBufferData, GL_ARRAY_BUFFER, GLsizeiptr(bytes), nullptr, GL_STATIC_DRAW);
                }
            }
            if (_pendingRegion && _pendingRegion->key == packet.key)
            {
                auto& pending = *_pendingRegion;
                std::vector<GPUVertex> packed;
                packed.reserve(384);
                const auto flush = [&] {
                    if (packed.empty()) return;
                    glCall(glBindBuffer, GL_ARRAY_BUFFER, pending.buffer.vbo);
                    glCall(glBufferSubData, GL_ARRAY_BUFFER,
                        GLintptr(pending.uploadedVertices * sizeof(GPUVertex)),
                        GLsizeiptr(packed.size() * sizeof(GPUVertex)), packed.data());
                    pending.uploadedVertices += packed.size();
                    packed.clear();
                };
                while (uploadBudget.available())
                {
                    if (!pending.ownersPhase && pending.nextSurface == pending.surfaces->size())
                    {
                        flush();
                        pending.buffer.ordinaryCount = GLsizei(pending.uploadedVertices);
                        pending.ownersPhase = true;
                    }
                    if (pending.ownersPhase && pending.owners.empty()) break;
                    const size_t index = pending.ownersPhase
                        ? pending.owners.begin()->second[pending.nextOwner] : pending.nextSurface;
                    const auto& surface = (*pending.surfaces)[index];
                    if (!uploadBudget.take(6 * sizeof(GPUVertex))) break;
                    if (!appendVertices(packed, surface, pending.ownersPhase, !pending.ownersPhase)) break;
                    if (!pending.ownersPhase)
                    {
                        if (surface.coplanarOwner)
                            pending.owners[surface.nativePaintOrdinal == 0
                                ? std::numeric_limits<uint64_t>::max() : surface.nativePaintOrdinal].push_back(index);
                        ++pending.nextSurface;
                    }
                    else if (++pending.nextOwner == pending.owners.begin()->second.size())
                    {
                        pending.owners.erase(pending.owners.begin());
                        pending.nextOwner = 0;
                    }
                    if (packed.size() >= 384) flush();
                }
                flush();
                if (pending.ownersPhase && pending.owners.empty())
                {
                    pending.buffer.ownerCount = GLsizei(pending.uploadedVertices) - pending.buffer.ordinaryCount;
                    DiscardRegionBuffer(packet.key);
                    pending.buffer.lastSeen = _frame;
                    _staticRegionBytes += pending.buffer.bytes;
                    _staticOpaqueRegions.emplace(packet.key, pending.buffer);
                    _pendingRegion.reset();
                    found = _staticOpaqueRegions.find(packet.key);
                }
            }
            if (found != _staticOpaqueRegions.end())
            {
                // Keep ready geometry during an incremental replacement, but
                // never show a region after authoritative game-state invalidation.
                if (found->second.sourceRevision == packet.sourceRevision)
                    regionDraws.push_back(packet.key);
            }
        }
        // Do not evict fixed world geometry merely because the camera looked
        // away. The memory-budget path above performs LRU eviction only when a
        // new region actually needs space, so ordinary turning does not cause
        // avoidable GPU re-uploads.
        // Bind sprites AFTER all texture loads; the atlas array may have grown.
        OpenGLAPI::SetTexture(0,GL_TEXTURE_2D_ARRAY,textures.GetAtlasesTexture());
        OpenGLAPI::SetTexture(1,GL_TEXTURE_2D,textures.GetPaletteTexture());
        for(const auto key:regionDraws)
        {
            const auto& region=_staticOpaqueRegions.at(key);
            if (region.ordinaryCount <= 0)
                continue;
            glCall(glBindVertexArray,region.vao);
            glCall(
                glDrawArrays,GL_TRIANGLES,0,
                region.ordinaryCount);
        }
        glCall(glBindVertexArray,_vao);
        glCall(glBindBuffer,GL_ARRAY_BUFFER,_vbo);
        glCall(
            glBufferData,GL_ARRAY_BUFFER,
            GLsizeiptr(
                opaqueOrdinaryVertices.size()
                    * sizeof(GPUVertex)),
            opaqueOrdinaryVertices.data(),GL_STREAM_DRAW);
        if(!opaqueOrdinaryVertices.empty())
            glCall(
                glDrawArrays,GL_TRIANGLES,0,
                GLsizei(opaqueOrdinaryVertices.size()));

        bool haveOpaqueOwners =
            !opaqueOwnerVertices.empty();
        if (!haveOpaqueOwners)
        {
            for (const auto key : regionDraws)
            {
                if (_staticOpaqueRegions.at(key)
                        .ownerCount > 0)
                {
                    haveOpaqueOwners = true;
                    break;
                }
            }
        }

        if (haveOpaqueOwners)
        {
            auto& front = output.GetFinalFramebuffer();
            if (!_opaqueSnapshot
                || _opaqueSnapshot->GetWidth()
                    != GLuint(screenWidth)
                || _opaqueSnapshot->GetHeight()
                    != GLuint(screenHeight))
            {
                _opaqueSnapshot =
                    std::make_unique<OpenGLFramebuffer>(
                        screenWidth, screenHeight,
                        true, true, false);
            }

            // Freeze ordinary physical depth once. Owners may replace only
            // that same physical layer (within depth-buffer quantisation) or
            // genuinely nearer space; they never receive a world-space bias.
            _opaqueSnapshot->BindDraw();
            front.BindRead();
            glCall(
                glBlitFramebuffer,
                clipLeft, screenHeight - clipBottom,
                clipRight, screenHeight - clipTop,
                clipLeft, screenHeight - clipBottom,
                clipRight, screenHeight - clipTop,
                GL_DEPTH_BUFFER_BIT,GL_NEAREST);
            front.Bind();
            glCall(
                glViewport,left,
                screenHeight-top-height,
                width,height);
            glCall(
                glScissor,clipLeft,
                screenHeight-clipBottom,
                clipWidth,clipHeight);
            glCall(glEnable,GL_DEPTH_TEST);
            glCall(glDepthFunc,GL_LEQUAL);
            glCall(glDepthMask,GL_TRUE);
            glCall(glUseProgram,_program);
            OpenGLAPI::SetTexture(
                2,GL_TEXTURE_2D,
                _opaqueSnapshot->GetDepthTexture());

            for (const auto key : regionDraws)
            {
                const auto& region =
                    _staticOpaqueRegions.at(key);
                if (region.ownerCount <= 0)
                    continue;
                glCall(
                    glBindVertexArray,region.vao);
                glCall(
                    glDrawArrays,GL_TRIANGLES,
                    region.ordinaryCount,
                    region.ownerCount);
            }

            glCall(glBindVertexArray,_vao);
            glCall(
                glBindBuffer,GL_ARRAY_BUFFER,_vbo);
            glCall(
                glBufferData,GL_ARRAY_BUFFER,
                GLsizeiptr(
                    opaqueOwnerVertices.size()
                        * sizeof(GPUVertex)),
                opaqueOwnerVertices.data(),
                GL_STREAM_DRAW);
            if (!opaqueOwnerVertices.empty())
            {
                glCall(
                    glDrawArrays,GL_TRIANGLES,0,
                    GLsizei(
                        opaqueOwnerVertices.size()));
            }
        }

        if (!streamedTransparent.empty())
        {
            // Material/atlas resolution is independent of screen-tile binning.
            // Pack every transparent surface once; a surface crossing several
            // 128x128 tiles should not redo G1 lookup, remap resolution and
            // immutable-bitmap lookup once per tile.
            std::vector<GPUVertex> packedTransparentVertices;
            packedTransparentVertices.reserve(
                streamedTransparent.size() * 6);
            std::vector<std::pair<size_t, size_t>>
                transparentVertexRanges;
            transparentVertexRanges.reserve(
                streamedTransparent.size());
            for (const auto* surface : streamedTransparent)
            {
                const size_t first =
                    packedTransparentVertices.size();
                appendVertices(
                    packedTransparentVertices, *surface);
                transparentVertexRanges.emplace_back(
                    first,
                    packedTransparentVertices.size()
                        - first);
            }

            struct TransparentTileCandidate
            {
                size_t vertexOffset{};
                size_t vertexCount{};
                int32_t x0{}, y0{}, x1{}, y1{};
            };
            struct TransparentScreenTile
            {
                int32_t x0{}, y0{}, x1{}, y1{};
                std::vector<TransparentTileCandidate> candidates;
            };

            constexpr int32_t kTransparencyTileSize = 128;
            const int32_t tileColumns =
                (width + kTransparencyTileSize - 1)
                / kTransparencyTileSize;
            const int32_t tileRows =
                (height + kTransparencyTileSize - 1)
                / kTransparencyTileSize;
            std::vector<TransparentScreenTile> transparencyTiles(
                size_t(tileColumns) * size_t(tileRows));
            for (int32_t ty = 0; ty < tileRows; ++ty)
            for (int32_t tx = 0; tx < tileColumns; ++tx)
            {
                auto& tile =
                    transparencyTiles[
                        size_t(ty) * size_t(tileColumns)
                        + size_t(tx)];
                tile.x0 = tx * kTransparencyTileSize;
                tile.y0 = ty * kTransparencyTileSize;
                tile.x1 =
                    std::min(
                        width,
                        tile.x0 + kTransparencyTileSize);
                tile.y1 =
                    std::min(
                        height,
                        tile.y0 + kTransparencyTileSize);
            }

            struct TransparentCameraPoint
            {
                float x{}, y{}, z{};
            };
            const float clipZ =
                std::max(view.nearClip, 0.001f);
            const float focalY =
                foc * float(width) / float(height);
            const auto toCamera =
                [&](const Paint::FirstPersonVec3& world) {
                    const Paint::FirstPersonVec3 delta{
                        world.x - eye.x,
                        world.y - eye.y,
                        world.z - eye.z,
                    };
                    return TransparentCameraPoint{
                        Paint::FpDot(delta, b.right),
                        Paint::FpDot(delta, b.up),
                        Paint::FpDot(delta, b.forward),
                    };
                };
            const auto projectCameraPoint =
                [&](const TransparentCameraPoint& p) {
                    const float ndcX =
                        p.x * foc / p.z;
                    const float ndcY =
                        p.y * focalY / p.z;
                    return std::array<float, 2>{
                        (0.5f + 0.5f * ndcX)
                            * float(width),
                        (0.5f - 0.5f * ndcY)
                            * float(height),
                    };
                };

            // Clip the two native quad triangles against the real near plane
            // before screen binning. A crossing surface therefore occupies
            // only tiles reached by its clipped polygon, never the full
            // viewport merely because one original vertex was behind the eye.
            for (size_t surfaceIndex = 0;
                 surfaceIndex < streamedTransparent.size();
                 ++surfaceIndex)
            {
                const auto* surface =
                    streamedTransparent[surfaceIndex];
                const auto packedRange =
                    transparentVertexRanges[
                        surfaceIndex];
                if (packedRange.second == 0)
                    continue;

                float minX =
                    std::numeric_limits<float>::infinity();
                float minY =
                    std::numeric_limits<float>::infinity();
                float maxX =
                    -std::numeric_limits<float>::infinity();
                float maxY =
                    -std::numeric_limits<float>::infinity();
                bool haveProjection = false;

                for (size_t triangleStart :
                     { size_t{ 0 }, size_t{ 3 } })
                {
                    std::array<TransparentCameraPoint, 3>
                        input{};
                    for (size_t i = 0; i < input.size(); ++i)
                    {
                        input[i] = toCamera(
                            surface->triangles[
                                triangleStart + i].world);
                    }
                    std::array<TransparentCameraPoint, 4>
                        clipped{};
                    size_t clippedCount = 0;
                    for (size_t i = 0; i < input.size(); ++i)
                    {
                        const auto& current = input[i];
                        const auto& next =
                            input[(i + 1) % input.size()];
                        const bool currentInside =
                            current.z >= clipZ;
                        const bool nextInside =
                            next.z >= clipZ;
                        if (currentInside)
                            clipped[clippedCount++] = current;
                        if (currentInside != nextInside)
                        {
                            const float t =
                                (clipZ - current.z)
                                / (next.z - current.z);
                            clipped[clippedCount++] = {
                                current.x
                                    + (next.x - current.x) * t,
                                current.y
                                    + (next.y - current.y) * t,
                                clipZ,
                            };
                        }
                    }

                    for (size_t i = 0;
                         i < clippedCount; ++i)
                    {
                        const auto projected =
                            projectCameraPoint(clipped[i]);
                        haveProjection = true;
                        minX = std::min(
                            minX, projected[0]);
                        minY = std::min(
                            minY, projected[1]);
                        maxX = std::max(
                            maxX, projected[0]);
                        maxY = std::max(
                            maxY, projected[1]);
                    }
                }

                if (!haveProjection
                    || maxX < 0.0f || maxY < 0.0f
                    || minX >= float(width)
                    || minY >= float(height))
                    continue;

                const int32_t x0 =
                    std::clamp(
                        int32_t(std::floor(minX)) - 1,
                        0, width);
                const int32_t y0 =
                    std::clamp(
                        int32_t(std::floor(minY)) - 1,
                        0, height);
                const int32_t x1 =
                    std::clamp(
                        int32_t(std::ceil(maxX)) + 1,
                        0, width);
                const int32_t y1 =
                    std::clamp(
                        int32_t(std::ceil(maxY)) + 1,
                        0, height);
                if (x1 <= x0 || y1 <= y0)
                    continue;

                const int32_t firstColumn =
                    std::clamp(
                        x0 / kTransparencyTileSize,
                        0, tileColumns - 1);
                const int32_t lastColumn =
                    std::clamp(
                        (x1 - 1) / kTransparencyTileSize,
                        0, tileColumns - 1);
                const int32_t firstRow =
                    std::clamp(
                        y0 / kTransparencyTileSize,
                        0, tileRows - 1);
                const int32_t lastRow =
                    std::clamp(
                        (y1 - 1) / kTransparencyTileSize,
                        0, tileRows - 1);
                for (int32_t ty = firstRow;
                     ty <= lastRow; ++ty)
                for (int32_t tx = firstColumn;
                     tx <= lastColumn; ++tx)
                {
                    auto& tile =
                        transparencyTiles[
                            size_t(ty) * size_t(tileColumns)
                            + size_t(tx)];
                    tile.candidates.push_back({
                        packedRange.first,
                        packedRange.second,
                        std::max(x0, tile.x0),
                        std::max(y0, tile.y0),
                        std::min(x1, tile.x1),
                        std::min(y1, tile.y1),
                    });
                }
            }

            // Rectangle coverage deliberately overestimates real sprite
            // coverage. Its maximum is therefore a safe upper bound on the
            // number of logical transparent layers any pixel can contain.
            // Coverage changes only at candidate edges, so do not rebuild a
            // 128x128 per-pixel prefix grid for every tile and frame.
            const auto conservativeLayerLimit =
                [](const TransparentScreenTile& tile) {
                    return FirstPersonMaximumRectangleOverlap(
                        tile.candidates);
                };

            auto& front = output.GetFinalFramebuffer();
            if (!_background || _background->GetWidth()!=GLuint(screenWidth) || _background->GetHeight()!=GLuint(screenHeight))
            {
                _background = std::make_unique<OpenGLFramebuffer>(screenWidth,screenHeight,false,true,false);
                _opaqueSnapshot = std::make_unique<OpenGLFramebuffer>(screenWidth,screenHeight,true,true,false);
                for(auto& layer:_peelLayers)
                    layer = std::make_unique<OpenGLFramebuffer>(
                        screenWidth,screenHeight,true,true,false,true);
                for(auto& layer:_peelDepthLayers)
                    layer = std::make_unique<OpenGLFramebuffer>(
                        screenWidth, screenHeight, true, true, false, false,
                        GL_DEPTH_COMPONENT32F);
            }

            // Keep one immutable opaque colour/depth snapshot for all screen
            // tiles. Transparent tiles are disjoint in pixel space, so their
            // palette compositions may be committed independently.
            _opaqueSnapshot->BindDraw();
            front.BindRead();
            glCall(
                glBlitFramebuffer,
                clipLeft, screenHeight - clipBottom,
                clipRight, screenHeight - clipTop,
                clipLeft, screenHeight - clipBottom,
                clipRight, screenHeight - clipTop,
                GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT,
                GL_NEAREST);

            const int32_t viewportBottom = screenHeight - top - height;
            const auto tileClip =
                [&](const TransparentScreenTile& tile) {
                    return ScreenRect{
                        std::max(clipLeft, left + tile.x0),
                        std::max(clipTop, top + tile.y0),
                        std::min(clipRight, left + tile.x1),
                        std::min(clipBottom, top + tile.y1)
                    };
                };
            auto setTileScissor = [&](const TransparentScreenTile& tile) {
                const auto tileRect = tileClip(tile);
                glCall(glScissor, tileRect.getLeft(), screenHeight - tileRect.getBottom(), tileRect.getWidth(), tileRect.getHeight());
            };
            auto composeLayer = [&](OpenGLFramebuffer& layer, const TransparentScreenTile& tile) {
                _background->Bind();
                glCall(glViewport,0,0,screenWidth,screenHeight);
                setTileScissor(tile);
                glCall(glDisable,GL_DEPTH_TEST);
                glCall(glUseProgram,_composeProgram);
                OpenGLAPI::SetTexture(0,GL_TEXTURE_2D,front.GetTexture());
                OpenGLAPI::SetTexture(1,GL_TEXTURE_2D,layer.GetTexture());
                OpenGLAPI::SetTexture(2,GL_TEXTURE_2D,textures.GetPaletteTexture());
                glCall(glUniform1i,Uniform(_composeProgram,"uAccumulated"),0);
                glCall(glUniform1i,Uniform(_composeProgram,"uLayer"),1);
                glCall(glUniform1i,Uniform(_composeProgram,"uPalettes"),2);
                glCall(glDrawArrays,GL_TRIANGLES,0,3);

                front.BindDraw();
                _background->BindRead();
                const auto tileRect = tileClip(tile);
                const int32_t x0 = tileRect.getLeft();
                const int32_t x1 = tileRect.getRight();
                const int32_t y0 =
                    screenHeight - tileRect.getBottom();
                const int32_t y1 =
                    screenHeight - tileRect.getTop();
                glCall(glBlitFramebuffer,x0,y0,x1,y1,x0,y0,x1,y1,GL_COLOR_BUFFER_BIT,GL_NEAREST);
            };

            glCall(glBindVertexArray,_vao);
            glCall(glBindBuffer,GL_ARRAY_BUFFER,_vbo);
            glCall(
                glBufferData,GL_ARRAY_BUFFER,
                GLsizeiptr(
                    packedTransparentVertices.size()
                    * sizeof(GPUVertex)),
                packedTransparentVertices.data(),
                GL_STREAM_DRAW);
            glCall(
                glBindBuffer,
                GL_ELEMENT_ARRAY_BUFFER,_ebo);
            std::vector<GLuint> tileIndices;
            for (const auto& tile : transparencyTiles)
            {
                const auto dirtyTileClip = tileClip(tile);
                if (tile.candidates.empty()
                    || dirtyTileClip.getWidth() <= 0
                    || dirtyTileClip.getHeight() <= 0)
                    continue;

                // Every transparent surface was packed once above. A screen
                // tile now uploads only compact indices into that shared
                // vertex buffer rather than recopying full GPUVertex structs.
                size_t tileVertexCount = 0;
                for (const auto& candidate :
                     tile.candidates)
                {
                    tileVertexCount +=
                        candidate.vertexCount;
                }
                tileIndices.clear();
                tileIndices.reserve(tileVertexCount);
                for (const auto& candidate :
                     tile.candidates)
                {
                    for (size_t i = 0;
                         i < candidate.vertexCount; ++i)
                    {
                        tileIndices.push_back(
                            GLuint(
                                candidate.vertexOffset
                                + i));
                    }
                }
                if (tileIndices.empty())
                    continue;

                glCall(
                    glBufferData,
                    GL_ELEMENT_ARRAY_BUFFER,
                    GLsizeiptr(
                        tileIndices.size()
                        * sizeof(GLuint)),
                    tileIndices.data(),GL_STREAM_DRAW);

                // The rectangle-overlap maximum is conservative: transparent
                // pixels can only reduce it. Exact physical-depth/native-order
                // peeling is retained for every layer that can exist.
                const size_t logicalPasses =
                    std::min(
                        tile.candidates.size(),
                        conservativeLayerLimit(tile));
                constexpr size_t kCoverageProbeThreshold = 8;
                const bool probeExhaustion =
                    logicalPasses > kCoverageProbeThreshold;
                uint64_t coverageToken = 0;
                if (probeExhaustion)
                {
                    coverageToken = ++_nextPeelCoverageToken;
                    if (coverageToken == 0)
                        coverageToken = ++_nextPeelCoverageToken;
                }

                for(size_t pass=0; pass<logicalPasses; ++pass)
                {
                    size_t coverageSlot =
                        _peelCoverageQueries.size();
                    if (probeExhaustion)
                    {
                        if (pollPeelCoverageQueries(
                                coverageToken))
                            break;

                        coverageSlot =
                            findFreePeelCoverageQuery();
                        if (coverageSlot
                            == _peelCoverageQueries.size())
                        {
                            // Do not blindly queue the rest of a pathological
                            // exact stack after all probes are in flight. A
                            // small bounded pipeline keeps GPU work ahead of
                            // the CPU, then waits for one real coverage result.
                            // This changes scheduling only, never layer output.
                            if (waitForPeelCoverageQuery(
                                    coverageToken))
                                break;
                            coverageSlot =
                                findFreePeelCoverageQuery();
                        }
                    }

                    auto& physicalLayer = *_peelDepthLayers[size_t(pass&1)];
                    auto& logicalLayer = *_peelLayers[size_t(pass&1)];
                    const bool peeling = pass != 0;

                    // Stage 1: select the farthest remaining PHYSICAL depth.
                    physicalLayer.Bind();
                    glCall(glViewport,left,viewportBottom,width,height);
                    setTileScissor(tile);
                    glCall(glDepthMask,GL_TRUE);
                    const GLuint empty[4]={0,0,0,0};
                    const GLfloat farthest[1]={0.0f};
                    glCall(glClearBufferuiv,GL_COLOR,0,empty);
                    glCall(glClearBufferfv,GL_DEPTH,0,farthest);
                    glCall(glEnable,GL_DEPTH_TEST);
                    glCall(glDepthFunc,GL_GREATER);
                    glCall(glUseProgram,_program);
                    glCall(glUniform1i,Uniform(_program,"uPeeling"),peeling);
                    glCall(glUniform1i,Uniform(_program,"uPeelStage"),1);
                    OpenGLAPI::SetTexture(0,GL_TEXTURE_2D_ARRAY,textures.GetAtlasesTexture());
                    OpenGLAPI::SetTexture(1,GL_TEXTURE_2D,textures.GetPaletteTexture());
                    OpenGLAPI::SetTexture(2,GL_TEXTURE_2D,_opaqueSnapshot->GetDepthTexture());
                    OpenGLAPI::SetTexture(3,GL_TEXTURE_2D,
                        peeling?_peelDepthLayers[size_t((pass+1)&1)]->GetDepthTexture()
                               :_opaqueSnapshot->GetDepthTexture());
                    OpenGLAPI::SetTexture(4,GL_TEXTURE_2D,
                        peeling?_peelLayers[size_t((pass+1)&1)]->GetTexture()
                               :_peelLayers[size_t(pass&1)]->GetTexture());
                    // Unit 5 is unused in stage one; bind a non-attached depth
                    // texture anyway to avoid framebuffer/texture feedback.
                    OpenGLAPI::SetTexture(5,GL_TEXTURE_2D,_opaqueSnapshot->GetDepthTexture());

                    if (coverageSlot
                        < _peelCoverageQueries.size())
                    {
                        glCall(
                            glBeginQuery,
                            GL_ANY_SAMPLES_PASSED,
                            _peelCoverageQueries[
                                coverageSlot]);
                    }
                    glCall(
                        glDrawElements,GL_TRIANGLES,
                        GLsizei(tileIndices.size()),
                        GL_UNSIGNED_INT,nullptr);
                    if (coverageSlot
                        < _peelCoverageQueries.size())
                    {
                        glCall(
                            glEndQuery,
                            GL_ANY_SAMPLES_PASSED);
                        _peelCoverageTokens[
                            coverageSlot] =
                            coverageToken;
                        _peelCoveragePending[
                            coverageSlot] = true;
                    }

                    // A completed zero-sample result is definitive: every
                    // subsequent logical layer is empty too. Normal progress
                    // remains nonblocking; saturation backpressure above is the
                    // only place that waits for a coverage result.
                    if (probeExhaustion
                        && pollPeelCoverageQueries(
                            coverageToken))
                        break;

                    // Stage 2: at that exact physical depth, select the earliest
                    // remaining native paint ordinal. gl_FragDepth is an ordinal
                    // selector here only; world geometry and physical depth are
                    // unchanged.
                    logicalLayer.Bind();
                    glCall(glViewport,left,viewportBottom,width,height);
                    setTileScissor(tile);
                    const GLfloat latestOrdinal[1]={1.0f};
                    glCall(glClearBufferuiv,GL_COLOR,0,empty);
                    glCall(glClearBufferfv,GL_DEPTH,0,latestOrdinal);
                    glCall(glEnable,GL_DEPTH_TEST);
                    glCall(glDepthFunc,GL_LESS);
                    glCall(glUseProgram,_program);
                    glCall(glUniform1i,Uniform(_program,"uPeeling"),peeling);
                    glCall(glUniform1i,Uniform(_program,"uPeelStage"),2);
                    OpenGLAPI::SetTexture(2,GL_TEXTURE_2D,_opaqueSnapshot->GetDepthTexture());
                    OpenGLAPI::SetTexture(3,GL_TEXTURE_2D,
                        peeling?_peelDepthLayers[size_t((pass+1)&1)]->GetDepthTexture()
                               :_opaqueSnapshot->GetDepthTexture());
                    OpenGLAPI::SetTexture(4,GL_TEXTURE_2D,
                        peeling?_peelLayers[size_t((pass+1)&1)]->GetTexture()
                               :_opaqueSnapshot->GetTexture());
                    OpenGLAPI::SetTexture(5,GL_TEXTURE_2D,physicalLayer.GetDepthTexture());
                    glCall(
                        glDrawElements,GL_TRIANGLES,
                        GLsizei(tileIndices.size()),
                        GL_UNSIGNED_INT,nullptr);
                    composeLayer(logicalLayer,tile);

                    if (probeExhaustion
                        && pollPeelCoverageQueries(
                            coverageToken))
                        break;
                }
            }

            // Restore the physical viewport/scissor for the final depth clear.
            front.Bind();
            glCall(glViewport,left,viewportBottom,width,height);
            glCall(glScissor, clipLeft, screenHeight - clipBottom, clipWidth, clipHeight);
            glCall(glUseProgram,_program);
            glCall(glUniform1i,Uniform(_program,"uPeelStage"),0);
            glCall(glUniform1i,Uniform(_program,"uPeeling"),0);
        }
        // Separate physical depth from the game's draw-order depth, without
        // destroying depth belonging to ALREADY drawn windows outside this
        // viewport. Keep the scissor active until the depth clear finishes.
        glCall(glDepthMask, GL_TRUE);
        glCall(glClear, GL_DEPTH_BUFFER_BIT);
        glCall(glDisable, GL_SCISSOR_TEST);
        glCall(glViewport, 0, 0, screenWidth, screenHeight);
        glCall(glBindVertexArray, 0);
        OpenGLState::Reset();
    }
} // namespace OpenRCT2::Ui
#endif

