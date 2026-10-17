# Windows 7 / PowerShell 2.0-compatible compatibility sweep.
# Never launches or rebuilds OpenRCT2, mutates its bin/renderdoc.dll, or installs software.
# Each RenderDoc DLL is tested in an independent, time-limited child process.
$ErrorActionPreference = 'Stop'
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$root = [System.IO.Path]::GetFullPath((Join-Path $scriptDir '..\..'))
$probe = Join-Path $scriptDir 'probe_renderdoc_windows.ps1'
$base = Join-Path $root 'third_party\renderdoc\compatibility\windows-x64'
$report = Join-Path $root 'bin\renderdoc-win7-matrix.log'
$results = New-Object System.Collections.ArrayList
$powershellExe = Join-Path $env:SystemRoot 'System32\WindowsPowerShell\v1.0\powershell.exe'

function Emit([string]$line) {
    [Console]::Out.WriteLine($line)
    [Console]::Out.Flush()
    [void]$results.Add($line)
}

$candidates = @(
    @{ Name='v1.45'; Hash='313f450bfa1f7f7d5f50cc7ee80d9f99b68d3fe873e25eb865ff0433e359f7b3' },
    @{ Name='v1.32'; Hash='04c52cf6363f1379162a91ff81a8ac7fa98e649b84e470ee197e4608da4fe745' },
    @{ Name='v1.20'; Hash='fd1c55418f327029583d2fb6890a7ff616f3112203c8cf929308360c5a19baa1' }
)
Emit ('FP_RENDERDOC_MATRIX begin host=' + [Environment]::OSVersion.VersionString)
foreach ($candidate in $candidates) {
    $version = $candidate.Name
    $file = Join-Path (Join-Path $base $version) 'renderdoc.dll'
    if (-not [System.IO.File]::Exists($file)) {
        Emit ('FP_RENDERDOC_MATRIX version=' + $version + ' status=MISSING file=' + $file)
        continue
    }
    $child = $null
    try {
        $start = New-Object System.Diagnostics.ProcessStartInfo
        $start.FileName = $powershellExe
        $start.Arguments = '-NoProfile -NonInteractive -ExecutionPolicy Bypass -File "' +
            $probe + '" -DllPath "' + $file + '" -ExpectedSha256 ' + $candidate.Hash
        $start.UseShellExecute = $false
        $start.CreateNoWindow = $true
        $start.RedirectStandardOutput = $true
        $start.RedirectStandardError = $true
        $child = New-Object System.Diagnostics.Process
        $child.StartInfo = $start
        Emit ('FP_RENDERDOC_MATRIX version=' + $version + ' status=START')
        if (-not $child.Start()) {
            Emit ('FP_RENDERDOC_MATRIX version=' + $version + ' status=START_FAILED')
            continue
        }
        $finished = $child.WaitForExit(12000)
        $status = 'EXIT'
        if (-not $finished) {
            $status = 'TIMEOUT_KILLED'
            $child.Kill()
            [void]$child.WaitForExit(3000)
        }
        if ($child.HasExited) {
            $exitCode = $child.ExitCode
            Emit ('FP_RENDERDOC_MATRIX version=' + $version + ' status=' + $status + ' exit=' + $exitCode)
            # The probe emits only a few lines, well below pipe capacity.
            $stdout = $child.StandardOutput.ReadToEnd()
            $stderr = $child.StandardError.ReadToEnd()
            foreach ($line in ($stdout -split '[\r\n]+')) {
                if ($line.Length -gt 0) { Emit ('[' + $version + '] ' + $line) }
            }
            foreach ($line in ($stderr -split '[\r\n]+')) {
                if ($line.Length -gt 0) { Emit ('[' + $version + '] STDERR ' + $line) }
            }
        }
        else {
            Emit ('FP_RENDERDOC_MATRIX version=' + $version + ' status=STILL_RUNNING_AFTER_KILL')
        }
    }
    catch {
        Emit ('FP_RENDERDOC_MATRIX version=' + $version + ' status=ERROR message=' + $_.Exception.Message)
    }
    finally {
        if ($child -ne $null) { $child.Close() }
    }
}
[System.IO.Directory]::CreateDirectory([System.IO.Path]::GetDirectoryName($report)) | Out-Null
[System.IO.File]::WriteAllText($report, [string]::Join([Environment]::NewLine, [string[]]$results.ToArray([string])) + [Environment]::NewLine)
Emit ('FP_RENDERDOC_MATRIX report=' + $report)
