/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once
#ifndef DISABLE_OPENGL
#include "OpenGLAPI.h"
#include <openrct2/paint/FirstPersonRenderer.h>
#include <memory>
#include <array>
#include <vector>
#include <cstdint>
#include <unordered_map>
namespace OpenRCT2::Ui
{
    class TextureCache;
    class SwapFramebuffer;
    class OpenGLFramebuffer;
    // Owns only GL resources. It never owns or mutates the park simulation.
    class FirstPersonGLRenderer final
    {
    public:
        FirstPersonGLRenderer();
        ~FirstPersonGLRenderer();
        FirstPersonGLRenderer(const FirstPersonGLRenderer&) = delete;
        FirstPersonGLRenderer& operator=(const FirstPersonGLRenderer&) = delete;
        void Draw(
            const Paint::FirstPersonScene& scene, TextureCache& textures,
            SwapFramebuffer& output, int32_t screenWidth, int32_t screenHeight,
            const ScreenRect& dirtyClip);
    private:
        GLuint _program{}, _composeProgram{}, _vao{}, _vbo{}, _ebo{};
        // Boolean occlusion queries retire empty exact transparency layers.
        // Normal polling is nonblocking. If all probes are in flight on a
        // pathological stack, bounded backpressure waits for one completed
        // probe instead of queuing unbounded extra peel passes.
        std::array<GLuint,8> _peelCoverageQueries{};
        std::array<bool,8> _peelCoveragePending{};
        std::array<uint64_t,8> _peelCoverageTokens{};
        uint64_t _nextPeelCoverageToken{};
        std::unique_ptr<OpenGLFramebuffer> _background; // indexed composite scratch
        std::unique_ptr<OpenGLFramebuffer> _opaqueSnapshot; // indexed pixels AND physical depth
        std::array<std::unique_ptr<OpenGLFramebuffer>,2> _peelLayers; // R32UI filter row + native ordinal
        std::array<std::unique_ptr<OpenGLFramebuffer>,2> _peelDepthLayers; // selected physical depth per logical layer
        struct RegionBuffer
        {
            GLuint vao{}, vbo{};
            uint64_t sceneEpoch{}, generation{}, dependencyStamp{}, lastSeen{};
            uint64_t dependencyRevisionSerial{};
            size_t bytes{};
            GLsizei ordinaryCount{};
            GLsizei ownerCount{};
        };
        std::unordered_map<uint64_t,RegionBuffer> _staticOpaqueRegions;
        struct CoverageFallbackCacheEntry
        {
            uint64_t imageRevision{};
            uint8_t paletteIndex{};
        };
        std::unordered_map<uint32_t, CoverageFallbackCacheEntry>
            _coverageFallbackCache;
        size_t _staticRegionBytes{};
        uint64_t _frame{};
        uint64_t _sceneEpoch{};
        GLuint _atlasHandle{};
        std::unique_ptr<SwapFramebuffer> _preparedOutput;
        uint64_t _preparedSceneSerial{};
        int32_t _preparedScreenWidth{};
        int32_t _preparedScreenHeight{};
        int32_t _preparedViewportLeft{};
        int32_t _preparedViewportTop{};
        int32_t _preparedViewportWidth{};
        int32_t _preparedViewportHeight{};

        void RenderScene(
            const Paint::FirstPersonScene& scene, TextureCache& textures,
            SwapFramebuffer& output, int32_t screenWidth,
            int32_t screenHeight, const ScreenRect& dirtyClip);
        void DiscardRegionBuffer(uint64_t key);
        void DiscardAllRegionBuffers();
    };
}
#endif

