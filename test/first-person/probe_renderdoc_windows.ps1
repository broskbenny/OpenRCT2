# Standalone Windows 7-safe RenderDoc API smoke probe.
# No application build, installation, download, or GPU context is required.
# Launch as a SEPARATE powershell.exe process: failures never crash OpenRCT2.
param(
    [Parameter(Mandatory=$true)][string]$DllPath,
    [switch]$SkipApi,
    [switch]$NoPin
)
$ErrorActionPreference = 'Stop'
$ExpectedSha256 = '809da38e3867d9fd09cc5c30dd5310500dee75e999166d7a14ad1ef6e9eca65d'

function ProbeMarker([string]$Message) {
    [Console]::Out.WriteLine("FP_RENDERDOC_PROBE " + $Message)
    [Console]::Out.Flush()
}

try {
    $file = [System.IO.Path]::GetFullPath($DllPath)
    ProbeMarker ("stage=begin path=" + $file)
    if (-not [System.IO.File]::Exists($file)) {
        ProbeMarker "stage=missing-dll"
        [Environment]::Exit(2)
    }
    $sha = [System.Security.Cryptography.SHA256]::Create()
    $stream = [System.IO.File]::OpenRead($file)
    try {
        $digest = [System.BitConverter]::ToString($sha.ComputeHash($stream)).Replace("-", "").ToLowerInvariant()
    }
    finally {
        $stream.Close()
        # SHA256Managed implements Clear() on .NET 2.0; Dispose() is not callable
        # from this PowerShell 2.0 host.
        $sha.Clear()
    }
    ProbeMarker ("stage=hash sha256=" + $digest)
    if ($digest -ne $ExpectedSha256) {
        ProbeMarker ("stage=hash-mismatch expected=" + $ExpectedSha256)
        [Environment]::Exit(3)
    }

    # Windows 7 PowerShell 2.0 / .NET 2.0-compatible P/Invoke declarations.
    $source = @'
using System;
using System.Runtime.InteropServices;
public static class NativeRenderDocProbe {
    [DllImport("kernel32.dll", CharSet=CharSet.Unicode, SetLastError=true, EntryPoint="LoadLibraryExW")]
    public static extern IntPtr LoadLibraryEx(string file, IntPtr ignored, uint flags);
    [DllImport("kernel32.dll", CharSet=CharSet.Ansi, SetLastError=true)]
    public static extern IntPtr GetProcAddress(IntPtr module, string exportName);
    [DllImport("kernel32.dll", SetLastError=true, EntryPoint="GetModuleHandleExW")]
    public static extern bool PinModule(uint flags, IntPtr address, out IntPtr pinnedModule);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    public delegate int GetApiDelegate(int apiVersion, out IntPtr api);
}
'@
    Add-Type -TypeDefinition $source
    ProbeMarker "stage=load-enter"
    # LOAD_WITH_ALTERED_SEARCH_PATH = 0x8, available without KB2533623.
    $module = [NativeRenderDocProbe]::LoadLibraryEx($file, [IntPtr]::Zero, [uint32]8)
    if ($module -eq [IntPtr]::Zero) {
        $win32error = [Runtime.InteropServices.Marshal]::GetLastWin32Error()
        ProbeMarker ("stage=load-failed win32=" + $win32error)
        [Environment]::Exit(4)
    }
    ProbeMarker "stage=load-ok"
    $proc = [NativeRenderDocProbe]::GetProcAddress($module, "RENDERDOC_GetAPI")
    if ($proc -eq [IntPtr]::Zero) {
        ProbeMarker ("stage=getproc-failed win32=" + [Runtime.InteropServices.Marshal]::GetLastWin32Error())
        [Environment]::Exit(5)
    }
    ProbeMarker "stage=getproc-ok"
    if (-not $NoPin) {
        # PIN | FROM_ADDRESS = 0x5; Windows XP SP1+.
        # Prevent import-hook thunks from targeting an unloaded module.
        $pinnedModule = [IntPtr]::Zero
        ProbeMarker "stage=pin-enter flags=5"
        $pinOk = [NativeRenderDocProbe]::PinModule([uint32]5, $proc, [ref]$pinnedModule)
        if (-not $pinOk -or $pinnedModule -eq [IntPtr]::Zero) {
            ProbeMarker ("stage=pin-failed win32=" + [Runtime.InteropServices.Marshal]::GetLastWin32Error())
            [Environment]::Exit(8)
        }
        ProbeMarker "stage=pin-ok"
    }
    else {
        ProbeMarker "stage=pin-skipped"
    }
    if ($SkipApi) {
        ProbeMarker "stage=api-skipped"
        ProbeMarker "stage=success-before-shutdown"
        [Environment]::Exit(0)
    }
    $delegate = [Runtime.InteropServices.Marshal]::GetDelegateForFunctionPointer($proc, [type][NativeRenderDocProbe+GetApiDelegate])
    $api = [IntPtr]::Zero
    ProbeMarker "stage=getapi-enter version=10102"
    $result = $delegate.Invoke(10102, [ref]$api)
    ProbeMarker ("stage=getapi-return code=" + $result + " nonnull=" + ($api -ne [IntPtr]::Zero))
    if ($result -ne 1 -or $api -eq [IntPtr]::Zero) {
        [Environment]::Exit(6)
    }
    # Do NOT FreeLibrary: RenderDoc's hooks can outlive the DLL image.
    ProbeMarker "stage=success-before-shutdown"
    [Environment]::Exit(0)
}
catch {
    ProbeMarker ("stage=managed-error exception=" + $_.Exception.ToString())
    [Environment]::Exit(7)
}
