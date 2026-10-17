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
        if ($dlls.Count -lt 1) { throw "Portable ZIP has no renderdoc.dll" }
        if ($licenses.Count -lt 1) { throw "Portable ZIP contains no license" }
        # The 64-bit bundle includes an additional 32-bit renderdoc.dll.
        # Identify the correct binary by the actual PE/COFF architecture.
        $found64 = 0
        foreach ($entry in $dlls) {
            $probe = Join-Path $OutputDirectory ("rdoc-" + [guid]::NewGuid().ToString() + ".probe")
            try {
                [IO.Compression.ZipFileExtensions]::ExtractToFile($entry, $probe, $true)
                $stream = [IO.File]::OpenRead($probe)
                try {
                    $reader = [IO.BinaryReader]::new($stream)
                    $stream.Position = 0x3c
                    $peOffset = $reader.ReadInt32()
                    if ($peOffset -le 0 -or $peOffset -gt ($stream.Length - 6)) {
                        throw "Invalid PE header: $($entry.FullName)"
                    }
                    $stream.Position = $peOffset
                    if ($reader.ReadUInt32() -ne 0x4550) {
                        throw "Invalid PE signature: $($entry.FullName)"
                    }
                    $machine = $reader.ReadUInt16()
                    Write-Host ("RenderDoc candidate {0}: PE machine {1:X4}" -f $entry.FullName, $machine)
                    if ($machine -eq 0x8664) {
                        $found64++
                        Copy-Item $probe "$target.partial" -Force
                    }
                }
                finally { $stream.Dispose() }
            }
            finally { Remove-Item $probe -Force -ErrorAction SilentlyContinue }
        }
        if ($found64 -ne 1) { throw "Expected exactly one AMD64 DLL, found $found64" }
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
