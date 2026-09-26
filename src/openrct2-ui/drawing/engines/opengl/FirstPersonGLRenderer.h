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
        void Draw(const Paint::FirstPersonScene& scene, TextureCache& textures, SwapFramebuffer& output,
                  int32_t screenWidth, int32_t screenHeight, int32_t left, int32_t top);
        [[nodiscard]] float LastGpuTimeMs() const { return _lastGpuTimeMs; }
    private:
        GLuint _program{}, _composeProgram{}, _vao{}, _vbo{};
        std::array<GLuint,3> _timerQueries{};
        std::array<bool,3> _timerPending{};
        uint32_t _nextTimer{};
        float _lastGpuTimeMs{};
        std::unique_ptr<OpenGLFramebuffer> _background; // indexed composite scratch
        std::unique_ptr<OpenGLFramebuffer> _opaqueSnapshot; // indexed pixels AND physical depth
        std::array<std::unique_ptr<OpenGLFramebuffer>,2> _peelLayers; // R16UI encoded filters + depth
        struct RegionBuffer
        {
            GLuint vao{}, vbo{};
            uint64_t generation{}, dependencyStamp{}, lastSeen{};
            size_t bytes{};
            GLsizei count{};
        };
        std::unordered_map<uint64_t,RegionBuffer> _staticOpaqueRegions;
        size_t _staticRegionBytes{};
        uint64_t _frame{};
        GLuint _atlasHandle{};
        void DiscardRegionBuffer(uint64_t key);
        void DiscardAllRegionBuffers();
    };
}
#endif

