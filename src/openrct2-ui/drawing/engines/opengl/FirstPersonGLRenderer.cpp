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
}
)GLSL";
        constexpr char kFragmentShader[] = R"GLSL(#version 330 core
uniform usampler2DArray uSprites;
uniform usampler2D uPalettes;
uniform sampler2D uOpaqueDepth;
uniform sampler2D uPreviousDepth;
uniform bool uPeeling;
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
layout(location=0) out uint oIndex;
void main() {
    // A texture sample outside the original sprite is EMPTY, not a stretched
    // border pixel. This matters when an isometric sprite is mapped to a
    // physical wall: the artwork often covers less than the entire wall plane.
    if (any(lessThan(fUV,vec2(0.0))) || any(greaterThanEqual(fUV,fSize))) discard;
    vec2 pixel = floor(fUV);
    vec2 uv = (fAtlas.xy + pixel + vec2(0.5)) / fAtlas.zw;
    uint col = texture(uSprites, vec3(uv, float(fAtlasLayer))).r;
    if (col == 0u && (fFlags & 8) != 0) {
        const ivec2 neighbours[4] = ivec2[4](
            ivec2(-1, 0), ivec2(1, 0), ivec2(0, -1), ivec2(0, 1));
        ivec2 p = ivec2(pixel);
        ivec2 limit = ivec2(fSize);
        for (int i = 0; i < 4 && col == 0u; ++i) {
            ivec2 q = p + neighbours[i];
            if (q.x < 0 || q.y < 0 || q.x >= limit.x || q.y >= limit.y) continue;
            vec2 neighbourUv = (fAtlas.xy + vec2(q) + vec2(0.5)) / fAtlas.zw;
            col = texture(uSprites, vec3(neighbourUv, float(fAtlasLayer))).r;
        }
    }
    if (col == 0u) discard;
    if (fMaskLayer >= 0) {
        if (any(lessThan(fUV,vec2(0.0))) || any(greaterThanEqual(fUV,fMaskSize))) discard;
        vec2 m = (fMaskAtlas.xy + floor(fUV) + vec2(0.5)) / fMaskAtlas.zw;
        if (texture(uSprites, vec3(m, float(fMaskLayer))).r == 0u) discard;
    }
    // Logarithmic physical depth leaves usable precision at the far end of
    // a complete RCT2 park while allowing the passenger near geometry.
    float depth = max(0.0, dot(fWorld - uEye, uForward));
    if ((fFlags & 4) != 0)
        depth = max(0.0, depth - 0.125);
    float logarithmic = log2(1.0 + depth)/log2(1.0 + uNearFar.y);
    gl_FragDepth = logarithmic;
    if ((fFlags & 1) != 0) {
        // A separate R16UI peel target stores the original palette FILTER,
        // not an already-composited colour. Zero denotes an empty fragment.
        float opaqueDepth = texelFetch(uOpaqueDepth,ivec2(gl_FragCoord.xy),0).r;
        if (logarithmic >= opaqueDepth - 0.00000002) discard;
        if (uPeeling) {
            float previous = texelFetch(uPreviousDepth,ivec2(gl_FragCoord.xy),0).r;
            if (logarithmic >= previous - 0.00000002) discard;
        }
        uint row = uint(fPalettes.y);
        // Native water mask changes its palette row with the mask texel.
        if ((fFlags & 2) != 0) row += col - 1u;
        if (row > 254u) discard;
        oIndex = (row+1u) << 8u;
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
    if(encoded==0u){oIndex=background;return;}
    uint row=(encoded>>8u)-1u;
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
    }
    void FirstPersonGLRenderer::Draw(
        const Paint::FirstPersonScene& scene, TextureCache& textures, SwapFramebuffer& output,
        int32_t screenWidth, int32_t screenHeight, int32_t left, int32_t top)
    {
        PROFILED_FUNCTION();
        if (scene.dimensions.width <= 0 || scene.dimensions.height <= 0) return;
        textures.ClearFirstPersonTransientBitmaps();
        // GPU timing queries are polled, never waited on, so the budget
        // controller cannot force a CPU/GPU synchronisation every frame.
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
        // We are inside an OpenRCT2 onDraw callback. Earlier 2-D commands have been flushed.
        output.BindOpaque();
        const int32_t width = scene.dimensions.width;
        const int32_t height = scene.dimensions.height;
        glCall(glEnable, GL_SCISSOR_TEST);
        glCall(glScissor, left, screenHeight - top - height, width, height);
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
        const auto b = Paint::GetFirstPersonBasis(scene.options.camera);
        const auto& eye = scene.options.camera.position;
        glCall(glUniform3f, Uniform(_program, "uEye"), eye.x, eye.y, eye.z);
        glCall(glUniform3f, Uniform(_program, "uForward"), b.forward.x, b.forward.y, b.forward.z);
        glCall(glUniform3f, Uniform(_program, "uRight"), b.right.x, b.right.y, b.right.z);
        glCall(glUniform3f, Uniform(_program, "uUp"), b.up.x, b.up.y, b.up.z);
        const float halfFov = std::clamp(scene.options.fieldOfViewDegrees, 30.0f, 120.0f)
            * 3.14159265358979323846f / 360.0f;
        const float foc = 1.0f / std::tan(halfFov);
        glCall(glUniform2f, Uniform(_program, "uFocal"), foc, foc * float(width) / float(height));
        glCall(glUniform2f, Uniform(_program, "uNearFar"), scene.options.nearClip, scene.options.farClip);
        OpenGLAPI::SetTexture(0, GL_TEXTURE_2D_ARRAY, textures.GetAtlasesTexture());
        OpenGLAPI::SetTexture(1, GL_TEXTURE_2D, textures.GetPaletteTexture());
        glCall(glUniform1i, Uniform(_program, "uSprites"), 0);
        glCall(glUniform1i, Uniform(_program, "uPalettes"), 1);
        glCall(glUniform1i, Uniform(_program, "uOpaqueDepth"), 2);
        glCall(glUniform1i, Uniform(_program, "uPreviousDepth"), 3);
        glCall(glUniform1i, Uniform(_program, "uPeeling"), 0);

        // Stable fixed world geometry belongs to resident GPU regions. Camera-
        // facing impostors, guests, cars and transparency remain streamed.
        // Unlike a single global cache, turning across a region boundary need
        // not upload every unchanged terrain tile and ordinary wall again.
        ++_frame;
        constexpr size_t kRegionGpuLimit = 96u * 1024u * 1024u;
        std::map<uint64_t,std::vector<const Paint::FirstPersonSurface*>> staticGroups;
        std::vector<const Paint::FirstPersonSurface*> streamedOpaque;
        std::vector<const Paint::FirstPersonSurface*> streamedTransparent;
        for (const auto& surface:scene.surfaces)
        {
            if (!surface.image.HasValue()) continue;
            if (surface.image.IsBlended()) streamedTransparent.push_back(&surface);
            else if (surface.gpuRegion!=0 && !surface.viewFacing)
                staticGroups[surface.gpuRegion].push_back(&surface);
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
        const GLuint currentAtlas=textures.GetAtlasesTexture();
        if(_atlasHandle!=currentAtlas)
        {
            DiscardAllRegionBuffers();
            _atlasHandle=currentAtlas;
        }
        std::unordered_map<uint64_t, BasicTextureInfo> immutableTextures;
        immutableTextures.reserve(scene.surfaces.size() / 16 + 1);
        auto appendVertices = [&](std::vector<GPUVertex>& vertices, const Paint::FirstPersonSurface& surface) {
            const auto image=surface.image;
            const auto* g1=GfxGetG1Element(image);
            BasicTextureInfo tex{};
            float imageWidth = 0.0f;
            float imageHeight = 0.0f;
            if (!surface.immutablePixels.empty())
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
            BasicTextureInfo maskTex{};
            const auto* maskG1=surface.mask.HasValue()?GfxGetG1Element(surface.mask):nullptr;
            if(maskG1!=nullptr) maskTex=textures.GetOrLoadImageTexture(surface.mask);
            int32_t count=0,palettes[3]={};
            if(image.HasSecondary())
            {
                count=image.HasTertiary()?3:2;
                palettes[0]=PaletteY(static_cast<Drawing::FilterPaletteID>(image.GetPrimary()));
                palettes[1]=PaletteY(static_cast<Drawing::FilterPaletteID>(image.GetSecondary()));
                palettes[2]=PaletteY(static_cast<Drawing::FilterPaletteID>(image.GetTertiary()));
            }
            else if(image.IsRemap() || image.IsBlended())
            {
                count=1;
                palettes[0]=PaletteY(static_cast<Drawing::FilterPaletteID>(image.GetRemap()));
            }
            for(const auto& p:surface.triangles)
                vertices.push_back({
                    {p.world.x,p.world.y,p.world.z},{p.u,p.v},
                    {tex.coords.x,tex.coords.y,tex.coords.z,tex.coords.w},
                    {imageWidth,imageHeight},int32_t(tex.index),
                    {count,palettes[0],palettes[1],palettes[2]},
                    (image.IsBlended()
                        ? (image.GetRemap()==static_cast<uint8_t>(Drawing::FilterPaletteID::paletteWater)?3:1)
                        : 0)
                        | (surface.depthBias ? 4 : 0)
                        | (surface.edgeCoverage ? 8 : 0),
                    {maskTex.coords.x,maskTex.coords.y,maskTex.coords.z,maskTex.coords.w},
                    {maskG1?float(maskG1->width):0.0f,maskG1?float(maskG1->height):0.0f},
                    maskG1?int32_t(maskTex.index):-1});
        };
        std::vector<GPUVertex> opaqueVertices;
        std::vector<GPUVertex> transparentVertices;
        for(const auto* surface:streamedOpaque) appendVertices(opaqueVertices,*surface);
        for(const auto* surface:streamedTransparent) appendVertices(transparentVertices,*surface);
        std::vector<uint64_t> regionDraws;
        regionDraws.reserve(staticGroups.size());
        for(const auto& [key,surfaces]:staticGroups)
        {
            uint64_t fingerprint=FirstPersonRegionFingerprint(surfaces);
            // Native OpenRCT2 can invalidate and re-pack one sprite inside
            // the SAME GL texture name. Refresh only regions USING that
            // particular original sprite; scrolling text elsewhere must not
            // force the entire park geometry back over the GPU bus.
            for(const auto* surface:surfaces)
            {
                ExtendFirstPersonFingerprint(fingerprint,
                    textures.GetImageTextureRevision(surface->image.GetIndex()));
                if(surface->mask.HasValue())
                    ExtendFirstPersonFingerprint(fingerprint,
                        textures.GetImageTextureRevision(surface->mask.GetIndex()));
            }
            auto found=_staticOpaqueRegions.find(key);
            if(found!=_staticOpaqueRegions.end() && found->second.fingerprint==fingerprint)
            {
                found->second.lastSeen=_frame;
                regionDraws.push_back(key);
                continue;
            }
            std::vector<GPUVertex> packed;
            packed.reserve(surfaces.size()*6);
            for(const auto* surface:surfaces) appendVertices(packed,*surface);
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
            region.fingerprint=fingerprint;
            region.lastSeen=_frame;
            regionDraws.push_back(key);
        }
        // Retain regions briefly during quick turns, but not every region ever
        // explored on a large map. GPU objects belong to this GL context only.
        if(_frame%120==0)
        {
            std::vector<uint64_t> old;
            for(const auto& [key,region]:_staticOpaqueRegions)
                if(_frame-region.lastSeen>240) old.push_back(key);
            for(const auto key:old) DiscardRegionBuffer(key);
        }
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

        if (!transparentVertices.empty())
        {
            auto& front = output.GetFinalFramebuffer();
            if (!_background || _background->GetWidth()!=GLuint(screenWidth) || _background->GetHeight()!=GLuint(screenHeight))
            {
                _background = std::make_unique<OpenGLFramebuffer>(screenWidth,screenHeight,false,true,false);
                _opaqueSnapshot = std::make_unique<OpenGLFramebuffer>(screenWidth,screenHeight,true,true,false);
                for(auto& layer:_peelLayers)
                    layer = std::make_unique<OpenGLFramebuffer>(screenWidth,screenHeight,true,true,true);
            }
            // Keep immutable original opaque colour and depth; palette effects
            // read the PREVIOUS composed layer, never the unmodified image.
            _opaqueSnapshot->BindDraw();
            front.BindRead();
            glCall(glBlitFramebuffer,0,0,screenWidth,screenHeight,
                   0,0,screenWidth,screenHeight,GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT,GL_NEAREST);
            glCall(glBindVertexArray,_vao);
            glCall(glBindBuffer,GL_ARRAY_BUFFER,_vbo);
            glCall(glBufferData,GL_ARRAY_BUFFER,GLsizeiptr(transparentVertices.size()*sizeof(GPUVertex)),
                   transparentVertices.data(),GL_STREAM_DRAW);
            // Palette-filter transparency is order dependent, so peel a bounded
            // number of exact far-to-near layers. Never make scene complexity
            // the loop bound and never synchronously wait for an occlusion query.
            // If a pixel is deeper than the exact budget, preserve its nearest
            // remaining transparent surface as a deterministic fallback.
            constexpr size_t kMaxExactPeelPasses = 6;
            const size_t maximumPossibleDepth = transparentVertices.size() / 6;
            const size_t exactPasses = std::min(kMaxExactPeelPasses, maximumPossibleDepth);
            auto composeLayer = [&](OpenGLFramebuffer& layer) {
                _background->Bind();
                glCall(glViewport,0,0,screenWidth,screenHeight);
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
                const int32_t y=screenHeight-top-height;
                glCall(glBlitFramebuffer,left,y,left+width,y+height,
                       left,y,left+width,y+height,GL_COLOR_BUFFER_BIT,GL_NEAREST);
            };
            for(size_t pass=0;pass<exactPasses;++pass)
            {
                auto& layer = *_peelLayers[size_t(pass&1)];
                layer.Bind();
                glCall(glViewport,left,screenHeight-top-height,width,height);
                glCall(glDepthMask,GL_TRUE);
                const GLuint empty[4]={0,0,0,0};
                const GLfloat farthest[1]={0.0f};
                glCall(glClearBufferuiv,GL_COLOR,0,empty);
                glCall(glClearBufferfv,GL_DEPTH,0,farthest);
                glCall(glEnable,GL_DEPTH_TEST);
                glCall(glDepthFunc,GL_GREATER);
                glCall(glUseProgram,_program);
                glCall(glUniform1i,Uniform(_program,"uPeeling"),pass!=0);
                OpenGLAPI::SetTexture(0,GL_TEXTURE_2D_ARRAY,textures.GetAtlasesTexture());
                OpenGLAPI::SetTexture(1,GL_TEXTURE_2D,textures.GetPaletteTexture());
                OpenGLAPI::SetTexture(2,GL_TEXTURE_2D,_opaqueSnapshot->GetDepthTexture());
                OpenGLAPI::SetTexture(3,GL_TEXTURE_2D,
                    pass==0?_opaqueSnapshot->GetDepthTexture():_peelLayers[size_t((pass+1)&1)]->GetDepthTexture());
                glCall(glDrawArrays,GL_TRIANGLES,0,GLsizei(transparentVertices.size()));
                composeLayer(layer);
            }
            if(maximumPossibleDepth>kMaxExactPeelPasses)
            {
                // Approximate only the overflow: select the NEAREST fragment
                // still in front of the last exact peel. This retains the glass
                // or water surface closest to the passenger instead of silently
                // discarding all layers beyond the budget.
                const size_t pass = exactPasses;
                auto& layer = *_peelLayers[size_t(pass&1)];
                layer.Bind();
                glCall(glViewport,left,screenHeight-top-height,width,height);
                glCall(glDepthMask,GL_TRUE);
                const GLuint empty[4]={0,0,0,0};
                const GLfloat nearest[1]={1.0f};
                glCall(glClearBufferuiv,GL_COLOR,0,empty);
                glCall(glClearBufferfv,GL_DEPTH,0,nearest);
                glCall(glEnable,GL_DEPTH_TEST);
                glCall(glDepthFunc,GL_LESS);
                glCall(glUseProgram,_program);
                glCall(glUniform1i,Uniform(_program,"uPeeling"),GL_TRUE);
                OpenGLAPI::SetTexture(0,GL_TEXTURE_2D_ARRAY,textures.GetAtlasesTexture());
                OpenGLAPI::SetTexture(1,GL_TEXTURE_2D,textures.GetPaletteTexture());
                OpenGLAPI::SetTexture(2,GL_TEXTURE_2D,_opaqueSnapshot->GetDepthTexture());
                OpenGLAPI::SetTexture(3,GL_TEXTURE_2D,_peelLayers[size_t((pass+1)&1)]->GetDepthTexture());
                glCall(glDrawArrays,GL_TRIANGLES,0,GLsizei(transparentVertices.size()));
                composeLayer(layer);
            }
            // Restore the physical viewport depth attachment before clearing.
            front.Bind();
            glCall(glViewport,left,screenHeight-top-height,width,height);
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

