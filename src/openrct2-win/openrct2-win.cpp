/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

// Windows.h needs to be included first
#ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

// The MSVC static C++20 filesystem/runtime can import these Win8 APIs even
// when the application itself targets Windows 7. Resolve its import-address
// symbols locally instead of importing unavailable KERNEL32 entry points.
// These bridges are limited to the actual calls made by that runtime.
#if defined(_MSC_VER) && defined(_M_X64)
namespace
{
    struct Win7CreateFile2Parameters
    {
        DWORD size;
        DWORD attributes;
        DWORD flags;
        DWORD securityQosFlags;
        LPSECURITY_ATTRIBUTES securityAttributes;
        HANDLE templateFile;
    };

    HANDLE WINAPI Win7CreateFile2(
        LPCWSTR file, DWORD desiredAccess, DWORD shareMode,
        DWORD disposition, const void* extended)
    {
        const auto* params = static_cast<const Win7CreateFile2Parameters*>(extended);
        if (params != nullptr && params->size != sizeof(*params))
        {
            SetLastError(ERROR_INVALID_PARAMETER);
            return INVALID_HANDLE_VALUE;
        }
        return CreateFileW(
            file, desiredAccess, shareMode,
            params != nullptr ? params->securityAttributes : nullptr,
            disposition,
            params != nullptr
                ? params->attributes | params->flags | params->securityQosFlags
                : FILE_ATTRIBUTE_NORMAL,
            params != nullptr ? params->templateFile : nullptr);
    }

    struct Win7CopyFile2Parameters
    {
        DWORD size;
        DWORD flags;
        BOOL* cancelled;
        void* progressCallback;
        void* callbackContext;
    };

    HRESULT WINAPI Win7CopyFile2(
        LPCWSTR source, LPCWSTR destination, const void* extended)
    {
        const auto* params = static_cast<const Win7CopyFile2Parameters*>(extended);
        if (params != nullptr)
        {
            if (params->size != sizeof(*params))
                return HRESULT_FROM_WIN32(ERROR_INVALID_PARAMETER);
            // std::filesystem uses only COPY_FILE_FAIL_IF_EXISTS. Do not
            // silently ignore other CopyFile2 options or callbacks.
            if ((params->flags & ~DWORD(COPY_FILE_FAIL_IF_EXISTS)) != 0
                || params->cancelled != nullptr
                || params->progressCallback != nullptr)
                return HRESULT_FROM_WIN32(ERROR_NOT_SUPPORTED);
        }
        if (CopyFileW(
                source, destination,
                params != nullptr && (params->flags & COPY_FILE_FAIL_IF_EXISTS) != 0))
            return S_OK;
        return HRESULT_FROM_WIN32(GetLastError());
    }

    void WINAPI Win7GetSystemTimePreciseAsFileTime(LPFILETIME result)
    {
        GetSystemTimeAsFileTime(result);
    }
}

// Define the exact x64 IAT symbols before the Windows import library is
// searched. No KERNEL32 replacement DLL or changes to the user's OS.
extern "C"
{
    HANDLE(WINAPI* __imp_CreateFile2)(
        LPCWSTR, DWORD, DWORD, DWORD, const void*) = &Win7CreateFile2;
    HRESULT(WINAPI* __imp_CopyFile2)(
        LPCWSTR, LPCWSTR, const void*) = &Win7CopyFile2;
    void(WINAPI* __imp_GetSystemTimePreciseAsFileTime)(
        LPFILETIME) = &Win7GetSystemTimePreciseAsFileTime;
}
#endif

// Enable visual styles
#pragma comment(                                                                                                               \
    linker,                                                                                                                    \
    "\"/manifestdependency:type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")

// Then the rest
#include <algorithm>
#include <iterator>
#include <openrct2-ui/Ui.h>
#include <openrct2-ui/FirstPersonRenderDocCapture.h>
#include <openrct2/core/EnumUtils.hpp>
#include <openrct2/core/String.hpp>
#include <string>
#include <vector>

static std::vector<std::string> GetCommandLineArgs(int argc, wchar_t** argvW);

/**
 * Windows entry point to OpenRCT2 with a console window using a traditional C main function.
 */
int wmain(int argc, wchar_t** argvW, [[maybe_unused]] wchar_t* envp)
{
    OpenRCT2::Ui::FirstPersonRenderDocCapture::InitialiseAtStartup();
    auto argvStrings = GetCommandLineArgs(argc, argvW);

    SetConsoleCP(EnumValue(OpenRCT2::CodePage::utf8));
    SetConsoleOutputCP(EnumValue(OpenRCT2::CodePage::utf8));

    std::vector<const char*> argv;
    std::transform(
        argvStrings.begin(), argvStrings.end(), std::back_inserter(argv), [](const auto& string) { return string.c_str(); });

    // Ensure that argv[argc] == nullptr, as mandated by the standard
    argv.push_back(nullptr);
    return NormalisedMain(argc, argv.data());
}

static std::vector<std::string> GetCommandLineArgs(int argc, wchar_t** argvW)
{
    // Allocate UTF-8 strings
    std::vector<std::string> argv;
    for (int i = 0; i < argc; i++)
    {
        argv.push_back(OpenRCT2::String::toUtf8(argvW[i]));
    }
    return argv;
}
