/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#ifndef DISABLE_OPENGL
#include "FirstPersonGLRenderer.h"
#include "FirstPersonGpuBatching.h"
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
void main() {
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
    if ((fFlags & 1) != 0) {
        // Opaque depth still comes from the renderer's fixed-point depth
        // buffer; this tolerance belongs only to that opaque comparison.
        const float opaqueDepthEps = 0.00000002;
        float opaqueDepth = texelFetch(uOpaqueDepth,ivec2(gl_FragCoord.xy),0).r;
        if (logarithmic >= opaqueDepth - opaqueDepthEps) discard;

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
        glCall(glGenQueries, GLsizei(_timerQueries.size()), _timerQueries.data());
        glCall(glGenQueries, GLsizei(_peelCoverageQueries.size()), _peelCoverageQueries.data());
        glCall(glGenVertexArrays, 1, &_vao);
        glCall(glGenBuffers, 1, &_vbo);
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
    void FirstPersonGLRenderer::DiscardAllRegionBuffers()
    {
        while(!_staticOpaqueRegions.empty())
            DiscardRegionBuffer(_staticOpaqueRegions.begin()->first);
    }
    FirstPersonGLRenderer::~FirstPersonGLRenderer()
    {
        DiscardAllRegionBuffers();
        glCall(glDeleteBuffers, 1, &_vbo);
        glCall(glDeleteVertexArrays, 1, &_vao);
        glCall(glDeleteProgram, _program);
        glCall(glDeleteProgram, _composeProgram);
        glCall(glDeleteQueries,GLsizei(_timerQueries.size()),_timerQueries.data());
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
        // GPU timing remains diagnostic only. Queries are polled, never waited
        // on, so diagnostics cannot force a CPU/GPU synchronisation every frame.
        for (size_t i=0;i<_timerQueries.size();++i)
        {
            if (!_timerPending[i]) continue;
            GLuint ready=0;
            glCall(glGetQueryObjectuiv,_timerQueries[i],GL_QUERY_RESULT_AVAILABLE,&ready);
            if (ready)
            {
                GLuint64 nanos=0;
                glCall(glGetQueryObjectui64v,_timerQueries[i],GL_QUERY_RESULT,&nanos);
                _lastGpuTimeMs=float(double(nanos)*1e-6);
                _timerPending[i]=false;
            }
        }
        auto timerSlot=_timerQueries.size();
        for(size_t n=0;n<_timerQueries.size();++n)
        {
            const size_t i=(_nextTimer+n)%_timerQueries.size();
            if(!_timerPending[i]) {timerSlot=i;_nextTimer=uint32_t((i+1)%_timerQueries.size());break;}
        }
        if(timerSlot<_timerQueries.size())
            glCall(glBeginQuery,GL_TIME_ELAPSED,_timerQueries[timerSlot]);

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
        std::unordered_set<uint64_t> seenTextures;
        seenTextures.reserve(scene.surfaces.size()/2+1);
        for(const auto& surface:scene.surfaces)
        {
            if(surface.image.HasValue() && surface.immutablePixels.empty() &&
               seenTextures.insert(FirstPersonMaterialFingerprint(surface.image)).second)
                textures.GetOrLoadImageTexture(surface.image);
            if(surface.mask.HasValue() &&
               seenTextures.insert(FirstPersonMaterialFingerprint(surface.mask)).second)
                textures.GetOrLoadImageTexture(surface.mask);
        }
        for (const auto& region : scene.staticRegions)
        {
            if (region.textureDependencies == nullptr)
                continue;
            for (const auto image : *region.textureDependencies)
            {
                const ImageId id(image);
                if (seenTextures.insert(FirstPersonMaterialFingerprint(id)).second)
                    textures.GetOrLoadImageTexture(id);
            }
        }
        const GLuint currentAtlas=textures.GetAtlasesTexture();
        if(_atlasHandle!=currentAtlas)
        {
            DiscardAllRegionBuffers();
            _atlasHandle=currentAtlas;
        }
        std::unordered_map<uint64_t, BasicTextureInfo> immutableTextures;
        immutableTextures.reserve(scene.surfaces.size() / 16 + 1);
        std::unordered_map<uint32_t, uint8_t> coverageFallbacks;
        coverageFallbacks.reserve(64);
        auto appendVertices = [&](std::vector<GPUVertex>& vertices, const Paint::FirstPersonSurface& surface) {
            const auto image=surface.image;
            const auto* g1=surface.solidColour == 0 ? GfxGetG1Element(image) : nullptr;
            BasicTextureInfo tex{};
            float imageWidth = 0.0f;
            float imageHeight = 0.0f;
            if (surface.solidColour != 0)
            {
                // The shader ignores atlas/UV fields for direct palette geometry.
                imageWidth = imageHeight = 1.0f;
            }
            else if (!surface.immutablePixels.empty())
            {
                if (surface.immutableWidth <= 0 || surface.immutableHeight <= 0)
                    return;
                auto it = immutableTextures.find(surface.immutableFingerprint);
                if (it == immutableTextures.end())
                {
                    const auto loaded = textures.LoadFirstPersonTransientBitmap(
                        surface.immutablePixels.data(),
                        size_t(surface.immutableWidth), size_t(surface.immutableHeight));
                    it = immutableTextures.emplace(surface.immutableFingerprint, loaded).first;
                }
                tex = it->second;
                imageWidth = float(surface.immutableWidth);
                imageHeight = float(surface.immutableHeight);
            }
            else
            {
                if(g1==nullptr || g1->width<=0 || g1->height<=0) return;
                tex=textures.GetOrLoadImageTexture(image);
                imageWidth = float(g1->width);
                imageHeight = float(g1->height);
            }
            int32_t coverageFallback = 0;
            if (surface.physicalCoverage && g1 != nullptr)
            {
                auto [it, inserted] =
                    coverageFallbacks.try_emplace(image.GetIndex(), 0);
                if (inserted)
                    it->second = DominantOpaqueSpritePixel(*g1);
                coverageFallback = int32_t(it->second);
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
                        | (surface.physicalCoverage ? 8 : 0)
                        | (surface.solidColour != 0 ? 16 : 0)
                        | (surface.textureFallbackOnly ? 32 : 0)
                        | (coverageFallback << 8),
                    {maskTex.coords.x,maskTex.coords.y,maskTex.coords.z,maskTex.coords.w},
                    {maskG1?float(maskG1->width):0.0f,maskG1?float(maskG1->height):0.0f},
                    maskG1?int32_t(maskTex.index):-1,
                    std::min<uint32_t>(surface.nativePaintOrdinal,0x00ffffffu)});
        };
        std::vector<GPUVertex> opaqueVertices;
        // Equal-depth ownership is resolved without changing physical depth:
        // draw ordinary opaque geometry first, then semantic owners. GL_LEQUAL
        // lets the later owner replace exactly coplanar terrain pixels.
        for(const auto* surface:streamedOpaque)
            if(!surface->coplanarOwner) appendVertices(opaqueVertices,*surface);
        for(const auto* surface:streamedOpaque)
            if(surface->coplanarOwner) appendVertices(opaqueVertices,*surface);
        std::vector<uint64_t> regionDraws;
        regionDraws.reserve(scene.staticRegions.size());
        for (const auto& packet : scene.staticRegions)
        {
            const uint64_t key = packet.key;
            uint64_t dependencyStamp = 14695981039346656037ull;
            if (packet.textureDependencies != nullptr)
            {
                for (const auto image : *packet.textureDependencies)
                {
                    ExtendFirstPersonFingerprint(dependencyStamp, image);
                    ExtendFirstPersonFingerprint(
                        dependencyStamp, textures.GetImageTextureRevision(image));
                }
            }

            auto found=_staticOpaqueRegions.find(key);
            if(found!=_staticOpaqueRegions.end()
                && found->second.sceneEpoch==packet.sceneEpoch
                && found->second.generation==packet.generation
                && found->second.dependencyStamp==dependencyStamp)
            {
                found->second.lastSeen=_frame;
                regionDraws.push_back(key);
                continue;
            }

            if (packet.surfaces == nullptr)
                continue;
            std::vector<GPUVertex> packed;
            packed.reserve(packet.surfaces->size()*6);
            for(const auto& surface:*packet.surfaces)
                if(!surface.coplanarOwner) appendVertices(packed,surface);
            for(const auto& surface:*packet.surfaces)
                if(surface.coplanarOwner) appendVertices(packed,surface);
            const size_t bytes=packed.size()*sizeof(GPUVertex);
            if(bytes==0) continue;
            const size_t oldBytes=found!=_staticOpaqueRegions.end()?found->second.bytes:0;
            // Evict only regions not needed THIS frame. If a very dense park
            // exceeds the cache budget, stream the extra region: preserve park
            // coverage rather than thrashing or silently dropping geometry.
            while(_staticRegionBytes-oldBytes+bytes>kRegionGpuLimit)
            {
                auto victim=_staticOpaqueRegions.end();
                for(auto it=_staticOpaqueRegions.begin();it!=_staticOpaqueRegions.end();++it)
                    if(it->first!=key && it->second.lastSeen!=_frame &&
                       (victim==_staticOpaqueRegions.end() ||
                        it->second.lastSeen<victim->second.lastSeen)) victim=it;
                if(victim==_staticOpaqueRegions.end()) break;
                DiscardRegionBuffer(victim->first);
            }
            if(_staticRegionBytes-oldBytes+bytes>kRegionGpuLimit ||
               packed.size()>size_t(std::numeric_limits<GLsizei>::max()))
            {
                if(found!=_staticOpaqueRegions.end()) DiscardRegionBuffer(key);
                opaqueVertices.insert(opaqueVertices.end(),packed.begin(),packed.end());
                continue;
            }
            if(found==_staticOpaqueRegions.end())
            {
                RegionBuffer buffer{};
                glCall(glGenVertexArrays,1,&buffer.vao);
                glCall(glGenBuffers,1,&buffer.vbo);
                ConfigureVertexInput(buffer.vao,buffer.vbo);
                found=_staticOpaqueRegions.emplace(key,buffer).first;
            }
            auto& region=found->second;
            glCall(glBindVertexArray,region.vao);
            glCall(glBindBuffer,GL_ARRAY_BUFFER,region.vbo);
            glCall(glBufferData,GL_ARRAY_BUFFER,GLsizeiptr(bytes),packed.data(),GL_STATIC_DRAW);
            _staticRegionBytes=_staticRegionBytes-region.bytes+bytes;
            region.bytes=bytes;
            region.count=GLsizei(packed.size());
            region.sceneEpoch=packet.sceneEpoch;
            region.generation=packet.generation;
            region.dependencyStamp=dependencyStamp;
            region.lastSeen=_frame;
            regionDraws.push_back(key);
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
            glCall(glBindVertexArray,region.vao);
            glCall(glDrawArrays,GL_TRIANGLES,0,region.count);
        }
        glCall(glBindVertexArray,_vao);
        glCall(glBindBuffer,GL_ARRAY_BUFFER,_vbo);
        glCall(glBufferData,GL_ARRAY_BUFFER,GLsizeiptr(opaqueVertices.size()*sizeof(GPUVertex)),
               opaqueVertices.data(),GL_STREAM_DRAW);
        if(!opaqueVertices.empty())
            glCall(glDrawArrays,GL_TRIANGLES,0,GLsizei(opaqueVertices.size()));

        if (!streamedTransparent.empty())
        {
            struct TransparentTileCandidate
            {
                const Paint::FirstPersonSurface* surface{};
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
            for (const auto* surface : streamedTransparent)
            {
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
                        surface,
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
            const auto conservativeLayerLimit =
                [](const TransparentScreenTile& tile) {
                    const int32_t tileWidth =
                        tile.x1 - tile.x0;
                    const int32_t tileHeight =
                        tile.y1 - tile.y0;
                    if (tileWidth <= 0 || tileHeight <= 0
                        || tile.candidates.empty())
                        return size_t{ 0 };

                    const int32_t stride = tileWidth + 1;
                    std::vector<int32_t> overlap(
                        size_t(tileWidth + 1)
                            * size_t(tileHeight + 1),
                        0);
                    const auto at =
                        [&](int32_t x, int32_t y)
                            -> int32_t& {
                            return overlap[
                                size_t(y) * size_t(stride)
                                + size_t(x)];
                        };
                    for (const auto& candidate :
                         tile.candidates)
                    {
                        const int32_t x0 =
                            candidate.x0 - tile.x0;
                        const int32_t y0 =
                            candidate.y0 - tile.y0;
                        const int32_t x1 =
                            candidate.x1 - tile.x0;
                        const int32_t y1 =
                            candidate.y1 - tile.y0;
                        if (x1 <= x0 || y1 <= y0)
                            continue;
                        ++at(x0, y0);
                        --at(x1, y0);
                        --at(x0, y1);
                        ++at(x1, y1);
                    }

                    for (int32_t y = 0;
                         y <= tileHeight; ++y)
                    for (int32_t x = 1;
                         x <= tileWidth; ++x)
                    {
                        at(x, y) += at(x - 1, y);
                    }
                    for (int32_t x = 0;
                         x <= tileWidth; ++x)
                    for (int32_t y = 1;
                         y <= tileHeight; ++y)
                    {
                        at(x, y) += at(x, y - 1);
                    }

                    int32_t maximum = 0;
                    for (int32_t y = 0;
                         y < tileHeight; ++y)
                    for (int32_t x = 0;
                         x < tileWidth; ++x)
                    {
                        maximum =
                            std::max(maximum, at(x, y));
                    }
                    return size_t(
                        std::max(maximum, 0));
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
            glCall(glBlitFramebuffer,0,0,screenWidth,screenHeight,
                   0,0,screenWidth,screenHeight,GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT,GL_NEAREST);

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
                const auto clip = tileClip(tile);
                glCall(glScissor, clip.getLeft(), screenHeight - clip.getBottom(), clip.getWidth(), clip.getHeight());
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
                const auto clip = tileClip(tile);
                const int32_t x0 = clip.getLeft();
                const int32_t x1 = clip.getRight();
                const int32_t y0 =
                    screenHeight - clip.getBottom();
                const int32_t y1 =
                    screenHeight - clip.getTop();
                glCall(glBlitFramebuffer,x0,y0,x1,y1,x0,y0,x1,y1,GL_COLOR_BUFFER_BIT,GL_NEAREST);
            };

            glCall(glBindVertexArray,_vao);
            glCall(glBindBuffer,GL_ARRAY_BUFFER,_vbo);
            for (const auto& tile : transparencyTiles)
            {
                const auto dirtyTileClip = tileClip(tile);
                if (tile.candidates.empty()
                    || dirtyTileClip.getWidth() <= 0
                    || dirtyTileClip.getHeight() <= 0)
                    continue;

                std::vector<GPUVertex> tileVertices;
                tileVertices.reserve(
                    tile.candidates.size() * 6);
                for (const auto& candidate :
                     tile.candidates)
                {
                    appendVertices(
                        tileVertices, *candidate.surface);
                }
                if (tileVertices.empty())
                    continue;

                glCall(glBindVertexArray,_vao);
                glCall(glBindBuffer,GL_ARRAY_BUFFER,_vbo);
                glCall(glBufferData,GL_ARRAY_BUFFER,
                    GLsizeiptr(tileVertices.size()*sizeof(GPUVertex)),
                    tileVertices.data(),GL_STREAM_DRAW);

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
                    if (probeExhaustion
                        && pollPeelCoverageQueries(
                            coverageToken))
                        break;

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

                    size_t coverageSlot =
                        _peelCoverageQueries.size();
                    if (probeExhaustion)
                    {
                        for (size_t i = 0;
                             i < _peelCoverageQueries.size(); ++i)
                        {
                            if (!_peelCoveragePending[i])
                            {
                                coverageSlot = i;
                                break;
                            }
                        }
                    }
                    if (coverageSlot
                        < _peelCoverageQueries.size())
                    {
                        glCall(
                            glBeginQuery,
                            GL_ANY_SAMPLES_PASSED,
                            _peelCoverageQueries[
                                coverageSlot]);
                    }
                    glCall(glDrawArrays,GL_TRIANGLES,0,GLsizei(tileVertices.size()));
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
                    // subsequent logical layer is empty too. Never wait for
                    // the result; if the driver is still working, continue the
                    // exact peel path and poll again after composition.
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
                    glCall(glDrawArrays,GL_TRIANGLES,0,GLsizei(tileVertices.size()));
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
        if(timerSlot<_timerQueries.size())
        {
            glCall(glEndQuery,GL_TIME_ELAPSED);
            _timerPending[timerSlot]=true;
        }
    }
} // namespace OpenRCT2::Ui
#endif

