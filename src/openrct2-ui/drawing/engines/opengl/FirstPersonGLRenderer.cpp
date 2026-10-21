/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#ifndef DISABLE_OPENGL
#include "FirstPersonGLRenderer.h"
#include "FirstPersonGpuBatching.h"
#include "FirstPersonRegionResidencyBudget.h"
#include <openrct2/paint/FirstPersonAsync.h>
#include "SwapFramebuffer.h"
#include "OpenGLFramebuffer.h"
#include "TextureCache.h"
#include <openrct2/drawing/Drawing.Sprite.h>
#include <openrct2/profiling/Profiling.h>
#include <openrct2/core/Console.hpp>

#include <algorithm>
#include <array>
#include <chrono>
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
layout(location=11) in vec3 aOutwardNormal;
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
flat out vec3 fOutwardNormal;
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
    fOutwardNormal = aOutwardNormal;
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
uniform int uOwnerSidePass;
uniform vec3 uEye;
uniform vec3 uForward;
uniform vec3 uRight;
uniform vec3 uUp;
uniform vec2 uFocal;
uniform vec4 uViewport;
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
flat in vec3 fOutwardNormal;
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
    // Closed reconstructed hull faces are physical boundary surfaces.
    // Use their authoritative world-space outward normal instead of
    // OpenGL winding so shared internal boundaries cannot both render.
    if ((fFlags & 64) != 0
        && dot(fOutwardNormal, uEye - fWorld) <= 0.0)
        discard;

    // Opposite authored wall sides can occupy the exact same plane. Native
    // paint ordinal does not encode which side of the wall the camera sees.
    // Replay rear-facing material first and front-facing material second.
    // A lone wall still renders from either side, and physical depth testing
    // continues to reject genuinely farther geometry. Other owner surfaces
    // need only the second pass.
    if (uOwnerSidePass != 0) {
        bool orientedOwner =
            (fFlags & 4) != 0 && (fFlags & 64) == 0
            && fOutwardNormal.z == 18.0;
        if (!orientedOwner) {
            if (uOwnerSidePass == 1) discard;
        } else {
            float axis = abs(fOutwardNormal.x);
            vec3 normal =
                axis == 1.0 ? vec3(1.0, 0.0, 0.0)
                : axis == 2.0 ? vec3(0.0, 1.0, 0.0)
                : vec3(0.0, 0.0, 1.0);
            normal *= sign(fOutwardNormal.x);
            bool cameraFacing = dot(normal, uEye - fWorld) > 0.0;
            if ((uOwnerSidePass == 1 && cameraFacing)
                || (uOwnerSidePass == 2 && !cameraFacing))
                discard;
        }
    }

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
    // Only exact world-axis-aligned, world-fixed native artwork owners have
    // a canonical plane. Two triangulations of the same plane can interpolate
    // fWorld differently at one pixel, causing a lower native ordinal to
    // win a GL_LEQUAL depth fight. Intersect the current pixel's camera ray
    // with their SHARED plane instead. Non-owners and nonplanar surfaces keep
    // their original physical depth, with no positional offsets or biases.
    // The native-owner-only plane is tagged in the unused outward-normal
    // attribute: (axis 1..3, coordinate, marker 17). Physical exterior faces
    // retain their true normal and never use this path. Thus the GPU vertex
    // remains its original size, with no new attribute or bandwidth penalty.
    if ((fFlags & 4) != 0 && (fFlags & 64) == 0
        && (fOutwardNormal.z == 17.0
            || fOutwardNormal.z == 18.0)) {
        vec3 axis =
            abs(fOutwardNormal.x) == 1.0 ? vec3(1.0, 0.0, 0.0)
            : abs(fOutwardNormal.x) == 2.0 ? vec3(0.0, 1.0, 0.0)
            : vec3(0.0, 0.0, 1.0);
        vec2 ndc = 2.0 * (gl_FragCoord.xy - uViewport.xy)
            / uViewport.zw - vec2(1.0);
        vec3 ray = uForward
            + (ndc.x / uFocal.x) * uRight
            + (ndc.y / uFocal.y) * uUp;
        float denominator = dot(axis, ray);
        if (abs(denominator) > 0.000001) {
            float planeDepth =
                (fOutwardNormal.y - dot(axis, uEye)) / denominator;
            if (planeDepth > 0.0 && planeDepth <= uNearFar.y)
                depth = planeDepth;
        }
    }
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

        if (uPeelStage == 3) {
            // Provably one transparent layer per pixel: physical depth is
            // the only selector. Emit the identical palette/ordinal token
            // as stage two without a separate depth-selection render pass.
            gl_FragDepth = logarithmic;
            oIndex = (ordinal << 8u) | (row + 1u);
            return;
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
            float outwardNormal[3];
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
        glCall(glEnableVertexAttribArray, 11);
        glCall(glVertexAttribPointer, 11, 3, GL_FLOAT, GL_FALSE, sizeof(GPUVertex), reinterpret_cast<void*>(offsetof(GPUVertex, outwardNormal)));
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
        using FrameClock = std::chrono::steady_clock;
        const auto renderStarted = FrameClock::now();
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
        int64_t coverageWaitUs = 0;
        size_t executedPeels = 0;
        size_t singleLayerTiles = 0;
        size_t depthOrderedTiles = 0;
        size_t activeTransparencyTiles = 0;
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
        glCall(glUniform4f, Uniform(_program, "uViewport"),
            float(left), float(screenHeight - top - height),
            float(width), float(height));
        OpenGLAPI::SetTexture(0, GL_TEXTURE_2D_ARRAY, textures.GetAtlasesTexture());
        OpenGLAPI::SetTexture(1, GL_TEXTURE_2D, textures.GetPaletteTexture());
        glCall(glUniform1i, Uniform(_program, "uSprites"), 0);
        glCall(glUniform1i, Uniform(_program, "uPalettes"), 1);
        glCall(glUniform1i, Uniform(_program, "uOpaqueDepth"), 2);
        glCall(glUniform1i, Uniform(_program, "uPreviousDepth"), 3);
        glCall(glUniform1i, Uniform(_program, "uPreviousLayer"), 4);
        glCall(glUniform1i, Uniform(_program, "uSelectedPhysicalDepth"), 5);
        glCall(glUniform1i, Uniform(_program, "uPeeling"), 0);
        glCall(glUniform1i, Uniform(_program, "uOwnerSidePass"), 0);
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
        const auto imageRevision = [&](auto image) { return textures.GetImageTextureRevision(image); };
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
                if (!uploadBudget.takeOversized(size_t(source->width) * size_t(source->height))) return false;
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
                        if (!uploadBudget.takeOversized(size_t(surface.immutableWidth) * size_t(surface.immutableHeight))) return false;
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
            // Restrict the canonical-depth path to exact axis-aligned quads.
            // For a native wall the constant world coordinate is authoritative
            // across adjacent tiles, unlike perspective-interpolated fWorld.
            // Slopes, curved scenery, terrain, ordinary geometry and sprites
            // retain their existing shader depth without approximation.
            std::array<float, 3> encodedOutwardNormal{
                surface.outwardNormal.x,
                surface.outwardNormal.y,
                surface.outwardNormal.z,
            };
            // Only native, non-exterior owners can reuse this field. Exterior
            // occupancy boundaries still need their world outward normals for
            // true front/back visibility; all other faces retain their data.
            if (!surface.exteriorOnly
                && (forceCoplanarOwner
                    || (!suppressCoplanarOwner && surface.coplanarOwner)))
            {
                const auto coordinate = [](const Paint::FirstPersonVec3& point, int axis) {
                    return axis == 0 ? point.x : axis == 1 ? point.y : point.z;
                };
                for (int axis = 0; axis < 3; ++axis)
                {
                    const float position = coordinate(surface.triangles[0].world, axis);
                    if (!std::isfinite(position))
                        continue;
                    if (std::all_of(surface.triangles.begin(), surface.triangles.end(),
                        [&](const auto& vertex) {
                            return coordinate(vertex.world, axis) == position;
                        }))
                    {
                        // Record winding as well as the canonical plane.
                        // Its sign is semantic wall sidedness, NOT a depth
                        // offset: reverse faces from adjacent tiles carry
                        // opposite signs even with identical coordinates.
                        const auto& a = surface.triangles[0].world;
                        const auto& b = surface.triangles[1].world;
                        const auto& c = surface.triangles[2].world;
                        const float u[3]{ b.x - a.x, b.y - a.y, b.z - a.z };
                        const float v[3]{ c.x - a.x, c.y - a.y, c.z - a.z };
                        const float cross[3]{
                            u[1] * v[2] - u[2] * v[1],
                            u[2] * v[0] - u[0] * v[2],
                            u[0] * v[1] - u[1] * v[0],
                        };
                        if (std::abs(cross[axis]) > 0.000001f)
                        {
                            // Directional replay only applies to authored
                            // wall planes. Generic coplanar owners (floors,
                            // roofs, decals, entrances) retain their prior
                            // native paint-order semantics. Both kinds still
                            // share the exact analytic physical depth.
                            const bool authoredWall =
                                surface.diagnosticSemanticRole
                                    == static_cast<uint8_t>(
                                        FirstPersonPaintSemanticRole::wall);
                            encodedOutwardNormal = {
                                std::copysign(float(axis + 1), cross[axis]),
                                position, authoredWall ? 18.0f : 17.0f
                            };
                        }
                        break;
                    }
                }
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
                        | (surface.exteriorOnly ? 64 : 0)
                        | (coverageFallback << 8),
                    {maskTex.coords.x,maskTex.coords.y,maskTex.coords.z,maskTex.coords.w},
                    {maskG1?float(maskG1->width):0.0f,maskG1?float(maskG1->height):0.0f},
                    maskG1?int32_t(maskTex.index):-1,
                    uint32_t(std::min<uint64_t>(
                        surface.nativePaintOrdinal,
                        0x00ffffffull)),
                    {encodedOutwardNormal[0],
                     encodedOutwardNormal[1],
                     encodedOutwardNormal[2]}});
            return true;
        };
        std::vector<GPUVertex> opaqueOrdinaryVertices;
        std::vector<GPUVertex> opaqueOwnerVertices;
        std::vector<const Paint::FirstPersonSurface*>
            streamedOwners;
        for (const auto* surface : streamedOpaque)
        {
            if (surface->coplanarOwner)
            {
                // Owners are appearance/order layers, not contributors to the
                // ordinary physical-depth snapshot. Render them only once,
                // later, in authoritative owner order.
                streamedOwners.push_back(surface);
            }
            else
            {
                appendVertices(
                    opaqueOrdinaryVertices, *surface);
            }
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
        bool streamedDirectionalWallOwner = false;
        for (const auto* surface : streamedOwners)
        {
            if (appendVertices(
                    opaqueOwnerVertices, *surface,
                    true, false)
                && !surface->exteriorOnly
                && surface->diagnosticSemanticRole
                    == static_cast<uint8_t>(
                        FirstPersonPaintSemanticRole::wall))
                streamedDirectionalWallOwner = true;
        }
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
        size_t residentVisibleBytes = 0;
        // Keep last-known-good geometry drawable while a different-sized
        // replacement is prepared. A shrink can temporarily need OLD + NEW
        // bytes, not just desired visible bytes. Unseen VBOs remain evictable.
        for (const auto& packet : scene.staticRegions)
        {
            const auto it = _staticOpaqueRegions.find(packet.key);
            if (it == _staticOpaqueRegions.end())
                continue;
            if (it->second.sceneEpoch != packet.sceneEpoch)
            {
                DiscardRegionBuffer(packet.key);
                continue;
            }
            residentVisibleBytes += it->second.bytes;
            it->second.lastSeen = _frame;
        }
        const size_t regionBudget = FirstPersonVisibleRegionGpuBudget(
            kRegionGpuLimit, visibleBytes, residentVisibleBytes,
            largestPacketBytes);
        if (_pendingRegion)
        {
            const auto packet = std::find_if(scene.staticRegions.begin(), scene.staticRegions.end(),
                [&](const auto& p) { return p.key == _pendingRegion->key; });
            if (packet == scene.staticRegions.end()
                || !Paint::FirstPersonStaticRegionUploadIsCurrent(
                    *packet, _pendingRegion->buffer.sceneEpoch,
                    _pendingRegion->buffer.generation, _pendingRegion->buffer.sourceRevision)
                || !_pendingRegion->buffer.imageRevisions.current(
                    *packet->textureDependencies, textureRevisionSerial, imageRevision))
                DiscardPendingRegion();
        }
        // A region's stable draw order is not a good upload priority:
        // spatial key order may load distant scenery before the nearest
        // missing wall. Prioritise new VBO work by camera distance while
        // retaining the original submission order for coplanar owner replay.
        std::vector<const Paint::FirstPersonStaticRegion*> uploadOrder;
        uploadOrder.reserve(scene.staticRegions.size());
        for (const auto& packet : scene.staticRegions)
        {
            // Fully resident, texture-current regions have no upload work.
            // Do not sort them every frame after a park has settled.
            const auto found = _staticOpaqueRegions.find(packet.key);
            if (found != _staticOpaqueRegions.end()
                && found->second.sceneEpoch == packet.sceneEpoch
                && found->second.generation == packet.generation
                && found->second.imageRevisions.current(
                    *packet.textureDependencies, textureRevisionSerial,
                    imageRevision))
                continue;
            uploadOrder.push_back(&packet);
        }
        const auto eye = scene.options.camera.position;
        const auto distanceSquared = [&](const auto* packet) {
            const auto dx = packet->center.x - eye.x;
            const auto dy = packet->center.y - eye.y;
            const auto dz = packet->center.z - eye.z;
            return dx * dx + dy * dy + dz * dz;
        };
        std::sort(uploadOrder.begin(), uploadOrder.end(),
            [&](const auto* a, const auto* b) {
                const auto ad = distanceSquared(a);
                const auto bd = distanceSquared(b);
                return ad != bd ? ad < bd : a->key < b->key;
            });
        for (const auto* selected : uploadOrder)
        {
            const auto& packet = *selected;
            auto found = _staticOpaqueRegions.find(packet.key);
            const bool ready = found != _staticOpaqueRegions.end()
                && found->second.sceneEpoch == packet.sceneEpoch
                && found->second.generation == packet.generation
                && found->second.imageRevisions.current(
                    *packet.textureDependencies, textureRevisionSerial, imageRevision);
            if (!ready && !_pendingRegion && packet.surfaceStorage && uploadBudget.available())
            {
                const auto bytes = packet.vertexCount * sizeof(GPUVertex);
                // A visible resident is the last-known-good world. Make room
                // only by evicting invisible LRU regions; if that is still not
                // enough, postpone this replacement rather than blanking the
                // visible region while the bounded upload catches up.
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
                    buffer.imageRevisions.capture(
                        *packet.textureDependencies, textureRevisionSerial, imageRevision);
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
                    if (!pending.ownersPhase
                        && pending.nextSurface == pending.surfaces->size())
                    {
                        flush();
                        pending.buffer.ordinaryCount =
                            GLsizei(pending.uploadedVertices);
                        pending.ownersPhase = true;
                    }
                    if (pending.ownersPhase && pending.owners.empty())
                        break;

                    if (!pending.ownersPhase)
                    {
                        const size_t index = pending.nextSurface++;
                        const auto& surface =
                            (*pending.surfaces)[index];
                        if (surface.coplanarOwner)
                        {
                            pending.owners[
                                surface.nativePaintOrdinal == 0
                                    ? std::numeric_limits<uint64_t>::max()
                                    : surface.nativePaintOrdinal]
                                .push_back(index);
                            continue;
                        }

                        // Admit source pixels before charging vertices, so
                        // an oversized bitmap can use an untouched frame.
                        if (!prepareMaterial(surface))
                        {
                            --pending.nextSurface;
                            break;
                        }
                        if (!uploadBudget.take(
                                6 * sizeof(GPUVertex)))
                        {
                            --pending.nextSurface;
                            break;
                        }
                        if (!appendVertices(
                                packed, surface))
                        {
                            --pending.nextSurface;
                            break;
                        }
                    }
                    else
                    {
                        const size_t index =
                            pending.owners.begin()
                                ->second[pending.nextOwner];
                        const auto& surface =
                            (*pending.surfaces)[index];
                        if (!prepareMaterial(surface))
                            break;
                        if (!uploadBudget.take(
                                6 * sizeof(GPUVertex)))
                            break;
                        if (!appendVertices(
                                packed, surface, true, false))
                            break;
                        if (!surface.exteriorOnly
                            && surface.diagnosticSemanticRole
                                == static_cast<uint8_t>(
                                    FirstPersonPaintSemanticRole::wall))
                            pending.buffer.hasDirectionalWallOwner = true;
                        if (++pending.nextOwner
                            == pending.owners.begin()
                                   ->second.size())
                        {
                            pending.owners.erase(
                                pending.owners.begin());
                            pending.nextOwner = 0;
                        }
                    }

                    if (packed.size() >= 384)
                        flush();
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
        }
        // Preserve the deterministic CPU region order for all draw and owner
        // replay passes. Uploading near-first must never change which of two
        // coincident native or map-owned wall faces wins the depth tie.
        for (const auto& packet : scene.staticRegions)
        {
            const auto found = _staticOpaqueRegions.find(packet.key);
            // Keep last-known-good buffers while their replacements upload;
            // only the scene epoch denotes a different park altogether.
            if (found != _staticOpaqueRegions.end()
                && found->second.sceneEpoch == packet.sceneEpoch)
                regionDraws.push_back(packet.key);
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

        bool hasDirectionalWallOwner = streamedDirectionalWallOwner;
        if (!hasDirectionalWallOwner)
        {
            for (const auto key : regionDraws)
            {
                if (_staticOpaqueRegions.at(key).hasDirectionalWallOwner)
                {
                    hasDirectionalWallOwner = true;
                    break;
                }
            }
        }
        size_t opaqueOwnerDrawCalls = 0;
        size_t opaqueOwnerSubmittedVertices = 0;
        const auto ownerReplayBegin = FrameClock::now();
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

            // Freeze only non-owner physical depth. Coplanar owners were
            // deliberately omitted from the ordinary pass, so they cannot
            // self-authorise by winning a depth fight before ordered replay.
            // Owners may replace only the same physical layer (within
            // depth-buffer quantisation) or genuinely nearer space; they never
            // receive a world-space bias.
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

            // Draw every region's rear-facing owners before any front-facing
            // owners, including across region boundaries. Drawing each region
            // once in isolation would reintroduce inter-region paint fighting.
            // The side test is performed per fragment and does not require
            // view-dependent re-upload of resident geometry.
            glCall(glBindVertexArray,_vao);
            glCall(glBindBuffer,GL_ARRAY_BUFFER,_vbo);
            glCall(
                glBufferData,GL_ARRAY_BUFFER,
                GLsizeiptr(
                    opaqueOwnerVertices.size()
                        * sizeof(GPUVertex)),
                opaqueOwnerVertices.data(),
                GL_STREAM_DRAW);
            // A park viewport without any relevant authored walls uses
            // its original single owner pass. Avoid doubling unrelated
            // roof/floor/decal submissions in such scenes.
            const int firstSidePass = hasDirectionalWallOwner ? 1 : 0;
            const int lastSidePass = hasDirectionalWallOwner ? 2 : 0;
            for (int sidePass = firstSidePass; sidePass <= lastSidePass; ++sidePass)
            {
                glCall(glUniform1i, Uniform(_program, "uOwnerSidePass"), sidePass);
                for (const auto key : regionDraws)
                {
                    const auto& region = _staticOpaqueRegions.at(key);
                    if (region.ownerCount <= 0)
                        continue;
                    glCall(glBindVertexArray, region.vao);
                    glCall(glDrawArrays, GL_TRIANGLES,
                        region.ordinaryCount, region.ownerCount);
                    ++opaqueOwnerDrawCalls;
                    opaqueOwnerSubmittedVertices += size_t(region.ownerCount);
                }
                if (!opaqueOwnerVertices.empty())
                {
                    glCall(glBindVertexArray, _vao);
                    glCall(glDrawArrays, GL_TRIANGLES, 0,
                        GLsizei(opaqueOwnerVertices.size()));
                    ++opaqueOwnerDrawCalls;
                    opaqueOwnerSubmittedVertices += opaqueOwnerVertices.size();
                }
            }
            glCall(glUniform1i, Uniform(_program, "uOwnerSidePass"), 0);

            // Owner replay deliberately used the pre-owner snapshot so equal
            // depth could be resolved without owners competing with
            // themselves. Transparency, however, must be occluded by the
            // completed opaque scene. Refresh only the depth snapshot now that
            // all owners have written their final physical depth.
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
        }

        const auto ownerReplayEnd = FrameClock::now();
        const auto opaqueDone = FrameClock::now();
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
                // Camera-space bounds after near clipping: provably disjoint
                // intervals have a fixed far-to-near palette composition order.
                float depthMin{}, depthMax{};
                bool fixedDepthOrder = false;
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
                float depthMin = std::numeric_limits<float>::infinity();
                float depthMax = -std::numeric_limits<float>::infinity();

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
                        depthMin = std::min(depthMin, clipped[i].z);
                        depthMax = std::max(depthMax, clipped[i].z);
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
                        depthMin, depthMax,
                        !surface->coplanarOwner,
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
            // A single index upload services every screen tile. Separate
            // indexed draws preserve exact clipping and the order of all
            // physical-depth/native-ordinal peel layers.
            struct TransparentIndexRange
            {
                size_t start{};
                GLsizei count{};
            };
            std::vector<TransparentIndexRange> tileRanges(
                transparencyTiles.size());
            std::vector<GLuint> tileIndices;
            for (size_t tileIndex = 0;
                 tileIndex < transparencyTiles.size(); ++tileIndex)
            {
                const auto& tile = transparencyTiles[tileIndex];
                const auto dirtyTileClip = tileClip(tile);
                if (tile.candidates.empty()
                    || dirtyTileClip.getWidth() <= 0
                    || dirtyTileClip.getHeight() <= 0)
                    continue;
                size_t tileVertexCount = 0;
                for (const auto& candidate : tile.candidates)
                    tileVertexCount += candidate.vertexCount;
                if (tileVertexCount > size_t(std::numeric_limits<GLsizei>::max()))
                    continue;
                const size_t first = tileIndices.size();
                tileIndices.reserve(first + tileVertexCount);
                for (const auto& candidate : tile.candidates)
                    for (size_t i = 0; i < candidate.vertexCount; ++i)
                        tileIndices.push_back(GLuint(candidate.vertexOffset + i));
                tileRanges[tileIndex] = {
                    first, GLsizei(tileIndices.size() - first)
                };
            }
            glCall(glBufferData,GL_ELEMENT_ARRAY_BUFFER,
                GLsizeiptr(tileIndices.size() * sizeof(GLuint)),
                tileIndices.empty() ? nullptr : tileIndices.data(),
                GL_STREAM_DRAW);
            for (size_t tileIndex = 0;
                 tileIndex < transparencyTiles.size(); ++tileIndex)
            {
                const auto& tile = transparencyTiles[tileIndex];
                const auto range = tileRanges[tileIndex];
                if (range.count == 0)
                    continue;
                ++activeTransparencyTiles;
                const void* indexOffset = reinterpret_cast<const void*>(
                    range.start * sizeof(GLuint));

                // The rectangle-overlap maximum is conservative: transparent
                // pixels can only reduce it. Exact physical-depth/native-order
                // peeling is retained for every layer that can exist.
                const size_t logicalPasses =
                    std::min(
                        tile.candidates.size(),
                        conservativeLayerLimit(tile));

                if (logicalPasses == 1)
                {
                    // The conservative rectangle bound proves that no pixel
                    // in this tile can contain two DIFFERENT source surfaces.
                    // The direct pass uses physical depth in the same way as
                    // stage one and emits the exact encoded palette token
                    // from stage two; no second geometry pass is necessary.
                    auto& logicalLayer = *_peelLayers[0];
                    logicalLayer.Bind();
                    glCall(glViewport, left, viewportBottom, width, height);
                    setTileScissor(tile);
                    const GLuint emptyToken[4] = { 0, 0, 0, 0 };
                    const GLfloat initialDepth[1] = { 0.0f };
                    glCall(glClearBufferuiv, GL_COLOR, 0, emptyToken);
                    glCall(glClearBufferfv, GL_DEPTH, 0, initialDepth);
                    glCall(glDepthMask, GL_TRUE);
                    glCall(glEnable, GL_DEPTH_TEST);
                    glCall(glDepthFunc, GL_GREATER);
                    glCall(glUseProgram, _program);
                    glCall(glUniform1i, Uniform(_program, "uPeeling"), 0);
                    glCall(glUniform1i, Uniform(_program, "uPeelStage"), 3);
                    OpenGLAPI::SetTexture(0, GL_TEXTURE_2D_ARRAY,
                        textures.GetAtlasesTexture());
                    OpenGLAPI::SetTexture(1, GL_TEXTURE_2D,
                        textures.GetPaletteTexture());
                    OpenGLAPI::SetTexture(2, GL_TEXTURE_2D,
                        _opaqueSnapshot->GetDepthTexture());
                    OpenGLAPI::SetTexture(3, GL_TEXTURE_2D,
                        _opaqueSnapshot->GetDepthTexture());
                    OpenGLAPI::SetTexture(4, GL_TEXTURE_2D,
                        _opaqueSnapshot->GetTexture());
                    OpenGLAPI::SetTexture(5, GL_TEXTURE_2D,
                        _opaqueSnapshot->GetDepthTexture());
                    glCall(glDrawElements, GL_TRIANGLES, range.count,
                        GL_UNSIGNED_INT, indexOffset);
                    composeLayer(logicalLayer, tile);
                    ++executedPeels;
                    ++singleLayerTiles;
                    continue;
                }

                // Exact depth-separated fast path. Camera-space z is the
                // same monotonic physical-depth key used by the fragment
                // shader for non-owner surfaces. Strictly disjoint intervals
                // prove that every pixel sees the same far-to-near ordering.
                // Ties, canonical coplanar owners and crossing surfaces
                // always use the established exact two-stage peeler.
                std::array<size_t, 64> depthOrder{};
                size_t depthOrderCount = tile.candidates.size();
                if (depthOrderCount < 2 || depthOrderCount > depthOrder.size())
                    depthOrderCount = 0;
                if (depthOrderCount != 0)
                {
                    // Reject owners and malformed ranges BEFORE sorting.
                    // In particular NaN would violate std::sort's strict
                    // weak-order contract if admitted to the comparator.
                    for (const auto& candidate : tile.candidates)
                    {
                        if (!candidate.fixedDepthOrder
                            || !std::isfinite(candidate.depthMin)
                            || !std::isfinite(candidate.depthMax)
                            || candidate.depthMin <= 0.0f
                            || candidate.depthMin > candidate.depthMax)
                        {
                            depthOrderCount = 0;
                            break;
                        }
                    }
                }
                if (depthOrderCount != 0)
                {
                    for (size_t i = 0; i < depthOrderCount; ++i)
                        depthOrder[i] = i;
                    std::sort(depthOrder.begin(),
                        depthOrder.begin() + depthOrderCount,
                        [&](size_t a, size_t b) {
                            const auto& left = tile.candidates[a];
                            const auto& right = tile.candidates[b];
                            return left.depthMin > right.depthMin
                                || (left.depthMin == right.depthMin && a < b);
                        });
                    constexpr float kSeparationMargin = 0.01f;
                    for (size_t i = 1; i < depthOrderCount; ++i)
                    {
                        if (tile.candidates[depthOrder[i - 1]].depthMin
                            <= tile.candidates[depthOrder[i]].depthMax
                                + kSeparationMargin)
                        {
                            depthOrderCount = 0;
                            break;
                        }
                    }
                }
                if (depthOrderCount != 0)
                {
                    auto& logicalLayer = *_peelLayers[0];
                    const GLuint emptyToken[4] = { 0, 0, 0, 0 };
                    const GLfloat initialDepth[1] = { 0.0f };
                    // Existing tile element indices are contiguous for each
                    // candidate; reordering draw ranges needs no new EBO.
                    std::array<size_t, 64> offsets{};
                    size_t indexAt = range.start;
                    for (size_t i = 0; i < depthOrderCount; ++i)
                    {
                        offsets[i] = indexAt;
                        indexAt += tile.candidates[i].vertexCount;
                    }
                    for (size_t i = 0; i < depthOrderCount; ++i)
                    {
                        const auto candidateIndex = depthOrder[i];
                        const auto& candidate = tile.candidates[candidateIndex];
                        logicalLayer.Bind();
                        glCall(glViewport, left, viewportBottom, width, height);
                        setTileScissor(tile);
                        glCall(glClearBufferuiv, GL_COLOR, 0, emptyToken);
                        glCall(glClearBufferfv, GL_DEPTH, 0, initialDepth);
                        glCall(glDepthMask, GL_TRUE);
                        glCall(glEnable, GL_DEPTH_TEST);
                        glCall(glDepthFunc, GL_GREATER);
                        glCall(glUseProgram, _program);
                        glCall(glUniform1i, Uniform(_program, "uPeeling"), 0);
                        glCall(glUniform1i, Uniform(_program, "uPeelStage"), 3);
                        OpenGLAPI::SetTexture(0, GL_TEXTURE_2D_ARRAY,
                            textures.GetAtlasesTexture());
                        OpenGLAPI::SetTexture(1, GL_TEXTURE_2D,
                            textures.GetPaletteTexture());
                        OpenGLAPI::SetTexture(2, GL_TEXTURE_2D,
                            _opaqueSnapshot->GetDepthTexture());
                        OpenGLAPI::SetTexture(3, GL_TEXTURE_2D,
                            _opaqueSnapshot->GetDepthTexture());
                        OpenGLAPI::SetTexture(4, GL_TEXTURE_2D,
                            _opaqueSnapshot->GetTexture());
                        OpenGLAPI::SetTexture(5, GL_TEXTURE_2D,
                            _opaqueSnapshot->GetDepthTexture());
                        const void* offset = reinterpret_cast<const void*>(
                            offsets[candidateIndex] * sizeof(GLuint));
                        glCall(glDrawElements, GL_TRIANGLES,
                            GLsizei(candidate.vertexCount),
                            GL_UNSIGNED_INT, offset);
                        composeLayer(logicalLayer, tile);
                        ++executedPeels;
                    }
                    ++depthOrderedTiles;
                    continue;
                }

                constexpr size_t kCoverageProbeThreshold = 8;
                // GL_QUERY_RESULT_AVAILABLE can still require a driver
                // round-trip. Poll a bounded number of times per tile rather
                // than after every physical and ordinal draw; any extra
                // conservative layers remain exact, only more expensive.
                constexpr size_t kCoveragePollInterval = 4;
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
                        if (pass % kCoveragePollInterval == 0
                            && pollPeelCoverageQueries(coverageToken))
                            break;

                        coverageSlot =
                            findFreePeelCoverageQuery();
                        // Saturated probes are never a reason to stall
                        // the render thread. The remaining conservative
                        // layers must still execute to preserve every palette
                        // filter when no query slot is available.
                    }

                    auto& physicalLayer = *_peelDepthLayers[size_t(pass&1)];
                    auto& logicalLayer = *_peelLayers[size_t(pass&1)];
                    const bool peeling = pass != 0;
                    ++executedPeels;

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
                        range.count,
                        GL_UNSIGNED_INT,indexOffset);
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

                    // Do not synchronously query completion between the
                    // physical and logical selection draws. The next
                    // scheduled poll may elide remaining empty layers.

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
                        range.count,
                        GL_UNSIGNED_INT,indexOffset);
                    composeLayer(logicalLayer,tile);
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
        const auto transparencyDone = FrameClock::now();
        // Separate physical depth from the game's draw-order depth, without
        // destroying depth belonging to ALREADY drawn windows outside this
        // viewport. Keep the scissor active until the depth clear finishes.
        glCall(glDepthMask, GL_TRUE);
        glCall(glClear, GL_DEPTH_BUFFER_BIT);
        glCall(glDisable, GL_SCISSOR_TEST);
        glCall(glViewport, 0, 0, screenWidth, screenHeight);
        glCall(glBindVertexArray, 0);
        OpenGLState::Reset();
        const auto renderDone = FrameClock::now();
        static auto lastDetailedSlowReport = FrameClock::time_point{};
        if (renderDone - renderStarted >= std::chrono::microseconds(16667)
            && renderDone - lastDetailedSlowReport >= std::chrono::seconds(2))
        {
            lastDetailedSlowReport = renderDone;
            const auto elapsed = [](auto before, auto after) {
                return std::chrono::duration_cast<std::chrono::microseconds>(
                    after - before).count();
            };
            Console::WriteLine(
                "FP_GL_RENDER_STAGES totalUs=%lld opaqueUs=%lld transparencyUs=%lld"
                " finishUs=%lld transparencySurfaces=%zu tiles=%zu peels=%zu singleLayerTiles=%zu depthOrderedTiles=%zu coverageWaitUs=%lld"
                " streamedOpaque=%zu staticRegions=%zu"
                " ownerReplayCpuUs=%lld ownerDraws=%zu ownerVertices=%zu sideWallReplay=%d",
                static_cast<long long>(elapsed(renderStarted, renderDone)),
                static_cast<long long>(elapsed(renderStarted, opaqueDone)),
                static_cast<long long>(elapsed(opaqueDone, transparencyDone)),
                static_cast<long long>(elapsed(transparencyDone, renderDone)),
                streamedTransparent.size(), activeTransparencyTiles, executedPeels,
                singleLayerTiles, depthOrderedTiles, static_cast<long long>(coverageWaitUs),
                streamedOpaque.size(), scene.staticRegions.size(),
                static_cast<long long>(elapsed(ownerReplayBegin, ownerReplayEnd)),
                opaqueOwnerDrawCalls, opaqueOwnerSubmittedVertices,
                hasDirectionalWallOwner ? 1 : 0);
        }
    }
} // namespace OpenRCT2::Ui
#endif

