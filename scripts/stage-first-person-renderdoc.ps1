# Automatic build-time staging of portable RenderDoc v1.46. No installation.
param([Parameter(Mandatory=$true)][string]$OutputDirectory)
$ErrorActionPreference = 'Stop'
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$target = Join-Path $OutputDirectory 'renderdoc.dll'
$licenseTarget = Join-Path $OutputDirectory 'renderdoc-LICENSE.md'
if ((Test-Path $target) -and (Test-Path $licenseTarget)) {
    Write-Host "RenderDoc runtime already staged: $target"
    exit 0
}
$uri = 'https://renderdoc.org/stable/1.46/RenderDoc_1.46_64.zip'
$zip = Join-Path ([IO.Path]::GetTempPath()) ([guid]::NewGuid().ToString() + '.zip')
try {
    [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
    Write-Host "Downloading official portable RenderDoc 1.46 at build time"
    Invoke-WebRequest -UseBasicParsing -Uri $uri -OutFile $zip
    Add-Type -AssemblyName System.IO.Compression
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    $archive = [IO.Compression.ZipFile]::OpenRead($zip)
    try {
        $dlls = @($archive.Entries | Where-Object { $_.Name -ieq 'renderdoc.dll' })
        $licenses = @($archive.Entries | Where-Object { $_.Name -ieq 'LICENSE.md' })
        if ($dlls.Count -ne 1) { throw "Portable ZIP has $($dlls.Count) renderdoc DLL entries" }
        if ($licenses.Count -lt 1) { throw "Portable ZIP contains no license" }
        [IO.Compression.ZipFileExtensions]::ExtractToFile($dlls[0], "$target.partial", $true)
        [IO.Compression.ZipFileExtensions]::ExtractToFile($licenses[0], $licenseTarget, $true)
        Move-Item "$target.partial" $target -Force
        Write-Host "RenderDoc staged: $target"
    }
    finally { $archive.Dispose() }
}
catch {
    Write-Error "Cannot produce self-contained inspector capture build: $($_.Exception.Message)"
    exit 1
}
finally { Remove-Item $zip -Force -ErrorAction SilentlyContinue }
