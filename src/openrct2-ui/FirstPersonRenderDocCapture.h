/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once
#if defined(_WIN32) && !defined(DISABLE_OPENGL)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include "renderdoc_app.h"
#include <openrct2/core/Console.hpp>
#include <filesystem>
#include <string>
#include <system_error>

namespace OpenRCT2::Ui::FirstPersonRenderDocCapture
{
struct CaptureState
{
    RENDERDOC_API_1_1_2* api = nullptr;
    bool pending = false;
    bool capturing = false;
    bool announced = false;
    uint32_t captureCountBefore = 0;
    std::string failure = "not-initialised";
};
inline CaptureState& State()
{
    static CaptureState state;
    return state;
}
inline std::string ToUtf8(const std::wstring& wide)
{
    if (wide.empty()) return {};
    const int len = WideCharToMultiByte(CP_UTF8, 0, wide.data(),
        int(wide.size()), nullptr, 0, nullptr, nullptr);
    if (len <= 0) return {};
    std::string result(size_t(len), '\0');
    WideCharToMultiByte(CP_UTF8, 0, wide.data(), int(wide.size()),
        result.data(), len, nullptr, nullptr);
    return result;
}
// Called from wmain BEFORE the first graphics API or SDL context creation.
inline void InitialiseAtStartup()
{
    auto& state = State();
    state.failure = "renderdoc.dll-missing";
    HMODULE module = GetModuleHandleW(L"renderdoc.dll");
    if (!module)
    {
        wchar_t executable[32768]{};
        const DWORD n = GetModuleFileNameW(nullptr, executable, 32768);
        if (n > 0 && n < 32768)
        {
            auto dll = std::filesystem::path(std::wstring(executable, n))
                .parent_path() / L"renderdoc.dll";
            // LOAD_LIBRARY_SEARCH_* requires KB2533623 on Windows 7.
            // LOAD_WITH_ALTERED_SEARCH_PATH works on unpatched Windows 7
            // and resolves dependencies relative to this absolute DLL path.
            module = LoadLibraryExW(dll.c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
            if (!module)
                state.failure = "renderdoc.dll-load-failed-win32-" + std::to_string(GetLastError());
        }
    }
    if (!module) return;
    auto getApi = reinterpret_cast<pRENDERDOC_GetAPI>(
        GetProcAddress(module, "RENDERDOC_GetAPI"));
    if (!getApi)
    {
        state.failure = "renderdoc-api-export-missing";
        return;
    }
    // RenderDoc installs graphics/API hooks. Keep their owning module resident
    // for the entire process; otherwise those hooks may target unloaded code,
    // especially on Windows 7 during process shutdown. This function and both
    // flags are available on Windows 7 without KB2533623.
    HMODULE pinnedModule = nullptr;
    if (!GetModuleHandleExW(
            GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN,
            reinterpret_cast<LPCWSTR>(getApi), &pinnedModule))
    {
        state.failure = "renderdoc.dll-pin-failed-win32-" + std::to_string(GetLastError());
        return;
    }
    if (getApi(eRENDERDOC_API_Version_1_1_2,
            reinterpret_cast<void**>(&state.api)) != 1 || !state.api)
    {
        state.failure = "renderdoc-api-unavailable";
        return;
    }
    wchar_t profile[32768]{};
    const DWORD n = GetEnvironmentVariableW(L"USERPROFILE", profile, 32768);
    if (n > 0 && n < 32768)
    {
        auto folder = std::filesystem::path(std::wstring(profile,n))
            / L"Documents" / L"OpenRCT2" / L"captures";
        std::error_code ec;
        std::filesystem::create_directories(folder, ec);
        if (!ec)
        {
            auto prefix = folder /
                (L"fp-inspector-" + std::to_wstring(GetCurrentProcessId()));
            auto utf8 = ToUtf8(prefix.wstring());
            if (!utf8.empty()) state.api->SetCaptureFilePathTemplate(utf8.c_str());
        }
    }
    state.failure.clear();
}
inline void RequestOnInspectorClick()
{
    auto& state = State();
    if (!state.capturing) state.pending = true;
}
inline void BeginFrame()
{
    auto& s = State();
    if (!s.announced)
    {
        s.announced = true;
        Console::WriteLine("FP_RENDERDOC_BUILD protocol=click-capture-v1 ready=%d reason=%s",
            s.api ? 1 : 0, s.api ? "ok" : s.failure.c_str());
    }
    if (!s.pending || s.capturing) return;
    s.pending = false;
    if (!s.api)
    {
        Console::WriteLine("FP_RENDERDOC_CAPTURE status=unavailable reason=%s",
            s.failure.c_str());
        return;
    }
    s.captureCountBefore = s.api->GetNumCaptures();
    s.api->StartFrameCapture(nullptr, nullptr);
    s.capturing = s.api->IsFrameCapturing() != 0;
    Console::WriteLine("FP_RENDERDOC_CAPTURE status=%s previous=%u",
        s.capturing ? "started" : "start-failed", s.captureCountBefore);
}
inline void EndFrame()
{
    auto& s = State();
    if (!s.capturing || !s.api) return;
    s.capturing = false;
    if (!s.api->EndFrameCapture(nullptr, nullptr))
    {
        Console::WriteLine("FP_RENDERDOC_CAPTURE status=end-failed");
        return;
    }
    uint32_t count = s.api->GetNumCaptures();
    char filename[8192]{};
    uint32_t capacity = sizeof(filename);
    if (count > s.captureCountBefore
        && s.api->GetCapture(count-1,filename,&capacity,nullptr))
        Console::WriteLine("FP_RENDERDOC_CAPTURE status=saved captures=%u file=%s",
            count,filename);
    else
        Console::WriteLine("FP_RENDERDOC_CAPTURE status=completed file=pending captures=%u",
            count);
}
}
#else
namespace OpenRCT2::Ui::FirstPersonRenderDocCapture
{
inline void InitialiseAtStartup() {}
inline void RequestOnInspectorClick() {}
inline void BeginFrame() {}
inline void EndFrame() {}
}
#endif
