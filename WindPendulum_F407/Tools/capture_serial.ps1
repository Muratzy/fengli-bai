param(
    [string]$Port = "COM9",
    [double]$Duration = 0,
    [string]$Label = "",
    [string]$Output = ""
)

$ErrorActionPreference = "Stop"
if ([double]::IsNaN($Duration) -or [double]::IsInfinity($Duration) -or $Duration -lt 0) {
    throw "Duration must be finite and >= 0 (0 means continuous capture)."
}
$scriptPath = Join-Path $PSScriptRoot "serial_capture.py"
$python = Get-Command python -ErrorAction SilentlyContinue
if ($null -eq $python) {
    throw "Python was not found. Install Python 3.9+ and pyserial first."
}

$arguments = @("-X", "utf8", $scriptPath, "--port", $Port)
if ($Duration -gt 0) { $arguments += @("--duration", $Duration.ToString([Globalization.CultureInfo]::InvariantCulture)) }
if ($Label) { $arguments += @("--label", $Label) }
if ($Output) { $arguments += @("--output", $Output) }

Write-Host "Read-only capture on $Port. The target will not receive serial bytes. Press Ctrl+C to stop."
& $python.Source @arguments
exit $LASTEXITCODE
