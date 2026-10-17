// Standalone RenderDoc DLL initialization diagnostic for Windows 7 x64.
// Compiled in CI as one native executable; no dependencies beyond Win7 APIs.
// Never launches or modifies OpenRCT2. Run separately from PowerShell.
#ifndef WINVER
#define WINVER 0x0601
#endif
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0601
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winver.h>
#include <cstdio>
#include <cstdint>

static volatile LONG g_stackObserved = 0;
static volatile LONG g_accessObserved = 0;
static void* g_stackIp = nullptr;
static void* g_accessIp = nullptr;
static void* g_stackSp = nullptr;
static LONG WINAPI OnException(EXCEPTION_POINTERS* exception)
{
    if (exception == nullptr || exception->ExceptionRecord == nullptr)
        return EXCEPTION_CONTINUE_SEARCH;
    const DWORD code = exception->ExceptionRecord->ExceptionCode;
    if (code == EXCEPTION_STACK_OVERFLOW &&
        InterlockedCompareExchange(&g_stackObserved, 1, 0) == 0)
    {
        g_stackIp = exception->ExceptionRecord->ExceptionAddress;
#ifdef _M_X64
        g_stackSp = reinterpret_cast<void*>(exception->ContextRecord->Rsp);
#endif
    }
    if (code == EXCEPTION_ACCESS_VIOLATION &&
        InterlockedCompareExchange(&g_accessObserved, 1, 0) == 0)
        g_accessIp = exception->ExceptionRecord->ExceptionAddress;
    return EXCEPTION_CONTINUE_SEARCH;
}

struct NativeOsVersion {
    ULONG size;
    ULONG major, minor, build, platform;
    WCHAR csd[128];
};
typedef LONG(WINAPI* RtlGetVersionFn)(NativeOsVersion*);
static void PrintSystemIdentity()
{
    OSVERSIONINFOW reported = {};
    reported.dwOSVersionInfoSize = sizeof(reported);
#pragma warning(push)
#pragma warning(disable:4996)
    if (GetVersionExW(&reported))
        std::printf("FP_RDLDR reported_os=%lu.%lu.%lu\n",
          reported.dwMajorVersion, reported.dwMinorVersion, reported.dwBuildNumber);
#pragma warning(pop)
    HMODULE ntdll = GetModuleHandleW(L"ntdll.dll");
    if (ntdll != nullptr)
    {
        RtlGetVersionFn rtl = reinterpret_cast<RtlGetVersionFn>(
          GetProcAddress(ntdll, "RtlGetVersion"));
        if (rtl != nullptr)
        {
            NativeOsVersion os = {};
            os.size = sizeof(os);
            if (rtl(&os) >= 0)
                std::printf("FP_RDLDR rtl_os=%lu.%lu.%lu\n", os.major, os.minor, os.build);
        }
        WCHAR ntdllPath[MAX_PATH] = {};
        if (GetModuleFileNameW(ntdll, ntdllPath, MAX_PATH))
        {
            DWORD ignored = 0;
            DWORD len = GetFileVersionInfoSizeW(ntdllPath, &ignored);
            if (len != 0 && len < 1048576)
            {
                // Avoid allocations and use version resource APIs on Windows 7.
                BYTE* buffer = reinterpret_cast<BYTE*>(HeapAlloc(GetProcessHeap(), 0, len));
                if (buffer)
                {
                    VS_FIXEDFILEINFO* info = nullptr;
                    UINT cb = 0;
                    if (GetFileVersionInfoW(ntdllPath, 0, len, buffer) &&
                        VerQueryValueW(buffer, L"\\", reinterpret_cast<void**>(&info), &cb) &&
                        cb >= sizeof(VS_FIXEDFILEINFO) && info != nullptr)
                        std::printf("FP_RDLDR ntdll_file_version=%u.%u.%u.%u\n",
                          HIWORD(info->dwFileVersionMS), LOWORD(info->dwFileVersionMS),
                          HIWORD(info->dwFileVersionLS), LOWORD(info->dwFileVersionLS));
                    HeapFree(GetProcessHeap(), 0, buffer);
                }
            }
        }
    }
    std::fflush(stdout);
}

int wmain(int argc, wchar_t** argv)
{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    if (argc != 2)
    {
        std::printf("FP_RDLDR usage=win7-loader-probe.exe <absolute-path-to-renderdoc.dll>\n");
        return 2;
    }
    DWORD attrs = GetFileAttributesW(argv[1]);
    if (attrs == INVALID_FILE_ATTRIBUTES || (attrs & FILE_ATTRIBUTE_DIRECTORY))
    {
        std::printf("FP_RDLDR missing_dll=1 win32=%lu\n", GetLastError());
        return 3;
    }
    PrintSystemIdentity();
    std::printf("FP_RDLDR pre_load=1\n");
    PVOID veh = AddVectoredExceptionHandler(1, &OnException);
    if (veh == nullptr)
    {
        std::printf("FP_RDLDR veh_install_failed=%lu\n", GetLastError());
        return 4;
    }
    SetLastError(0);
    HMODULE dll = LoadLibraryExW(argv[1], nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
    const DWORD loadError = GetLastError();
    std::printf("FP_RDLDR post_load=%d win32=%lu stack_exception=%ld stack_ip=%p stack_sp=%p access_exception=%ld access_ip=%p\n",
      dll != nullptr ? 1 : 0, loadError, g_stackObserved, g_stackIp,
      g_stackSp, g_accessObserved, g_accessIp);
    if (dll)
    {
        FARPROC entry = GetProcAddress(dll, "RENDERDOC_GetAPI");
        std::printf("FP_RDLDR export_present=%d\n", entry != nullptr ? 1 : 0);
    }
    RemoveVectoredExceptionHandler(veh);
    std::fflush(stdout);
    // Do NOT run CRT/FLS shutdown after a DLL initialization failure.
    // That secondary AV obscures the primary loader exception (Microsoft KB2754614).
    // This is an isolated disposable probe process; it owns no application data.
    TerminateProcess(GetCurrentProcess(), dll ? 0 : 10);
    return dll ? 0 : 10;
}
