# Windows 7 / PowerShell 2 compatible: stage committed AMD64 DLL offline.
param([Parameter(Mandatory=$true)][string]$OutputDirectory)
$ErrorActionPreference = 'Stop'
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$vendorDir = Join-Path $scriptDir '..\third_party\renderdoc\windows-x64'
$sourceDll = Join-Path $vendorDir 'renderdoc.dll'
$sourceLicense = Join-Path $vendorDir 'renderdoc-LICENSE.md'
if (-not (Test-Path -LiteralPath $sourceDll -PathType Leaf)) {
    throw "Pinned RenderDoc DLL missing from checkout: $sourceDll. Fetch latest first-person/renderdoc-click-capture."
}
if (-not (Test-Path -LiteralPath $sourceLicense -PathType Leaf)) {
    throw "Pinned RenderDoc license missing from checkout: $sourceLicense."
}
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$targetDll = Join-Path $OutputDirectory 'renderdoc.dll'
$targetLicense = Join-Path $OutputDirectory 'renderdoc-LICENSE.md'
# Re-stage every build: stale binaries must not silently pass validation.
Copy-Item -LiteralPath $sourceDll -Destination $targetDll -Force
Copy-Item -LiteralPath $sourceLicense -Destination $targetLicense -Force
if (-not (Test-Path -LiteralPath $targetDll -PathType Leaf)) {
    throw "RenderDoc staging failed: $targetDll"
}
Write-Host "RenderDoc bundled AMD64 DLL staged: $targetDll"
