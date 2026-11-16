[CmdletBinding()]
param(
  [ValidateSet('Build', 'BuildFlash', 'Inspect')]
  [string]$Action = 'Build',
  [string]$CubeIde = 'C:\cubeIDE\STM32CubeIDE_1.19.0\STM32CubeIDE\stm32cubeidec.exe',
  [string]$JLink = 'C:\Users\Muratsy\SEGGER\JLink_V962\JLink.exe',
  [string]$ProbeSerial = '602712599',
  [ValidateRange(100, 4000)]
  [int]$SpeedKHz = 4000,
  [string]$HeadlessWorkspace = 'C:\Temp\WindPendulumCubeIDE',
  [string]$StagingDirectory = 'C:\Temp\WindPendulumTools'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$logDirectory = Join-Path $projectRoot 'build\workflow'
$elfPath = Join-Path $projectRoot 'Debug\WindPendulum_F407.elf'

function Invoke-LoggedTool {
  param([string]$Executable, [string[]]$Arguments, [string]$LogPath)
  if (-not (Test-Path -LiteralPath $Executable -PathType Leaf)) {
    throw "Tool not found: $Executable"
  }
  # Native stderr is part of the log; decide success from the exit code.
  $previousPreference = $ErrorActionPreference
  $ErrorActionPreference = 'Continue'
  try {
    & $Executable @Arguments *> $LogPath
    $toolExit = $LASTEXITCODE
  }
  finally {
    $ErrorActionPreference = $previousPreference
  }
  if ($toolExit -ne 0) {
    Get-Content -LiteralPath $LogPath -Tail 25 | Write-Host
    throw "Tool failed (exit $toolExit). Log: $LogPath"
  }
  return [IO.File]::ReadAllText($LogPath)
}

New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null
Push-Location -LiteralPath $projectRoot
try {
  if ($Action -ne 'Inspect') {
    $buildStart = [DateTime]::UtcNow
    $buildLog = Join-Path $logDirectory 'cubeide-build.log'
    Write-Host 'Building WindPendulum_F407/Debug with STM32CubeIDE...'
    # Backslashes are required: CDT treats C:/... in -import as a URI scheme.
    $builderArguments = @(
      '--launcher.suppressErrors', '-nosplash',
      '-application', 'org.eclipse.cdt.managedbuilder.core.headlessbuild',
      '-data', $HeadlessWorkspace, '-import', $projectRoot,
      '-cleanBuild', 'WindPendulum_F407/Debug', '-printErrorMarkers'
    )
    $buildOutput = Invoke-LoggedTool $CubeIde $builderArguments $buildLog
    if ($buildOutput -notmatch 'Headless build completed successfully') {
      throw "Missing CubeIDE success marker. Log: $buildLog"
    }
    $elfInfo = Get-Item -LiteralPath $elfPath
    if ($elfInfo.LastWriteTimeUtc -lt $buildStart.AddSeconds(-2)) {
      throw 'The build did not create a fresh ELF; refusing to flash an old image.'
    }
    $manifest = [ordered]@{
      builtAtUtc = [DateTime]::UtcNow.ToString('o')
      project = $projectRoot
      configuration = 'Debug'
      elf = $elfPath
      sha256 = (Get-FileHash -LiteralPath $elfPath -Algorithm SHA256).Hash
      cubeIde = $CubeIde
      buildLog = $buildLog
      flashVerified = $false
    }
    $manifestPath = Join-Path $logDirectory 'last-run.json'
    $manifest | ConvertTo-Json | Set-Content -LiteralPath $manifestPath -Encoding UTF8
    $buildOutput -split '\r?\n' | Where-Object {
      $_ -match 'Build Finished|Headless build completed'
    } | ForEach-Object { Write-Host $_ }
    Write-Host "ELF: $elfPath"
    Write-Host "SHA256: $($manifest.sha256)"
  }

  if ($Action -eq 'BuildFlash') {
    # An ASCII path avoids J-Link Commander ELF path encoding failures.
    if ($StagingDirectory -match '[^\x20-\x7E]') {
      throw 'StagingDirectory must use an ASCII path for J-Link.'
    }
    New-Item -ItemType Directory -Path $StagingDirectory -Force | Out-Null
    $stagedElf = Join-Path $StagingDirectory 'WindPendulum_F407.elf'
    Copy-Item -LiteralPath $elfPath -Destination $stagedElf -Force
    if ((Get-FileHash -LiteralPath $stagedElf -Algorithm SHA256).Hash -ne $manifest.sha256) {
      throw 'The staged ELF hash differs from the new build.'
    }
    $flashCommands = Join-Path $StagingDirectory 'flash.jlink'
    @('r', ('loadfile "{0}"' -f $stagedElf), 'r', 'g', 'exit') |
      Set-Content -LiteralPath $flashCommands -Encoding ASCII
    $flashLog = Join-Path $logDirectory 'jlink-flash.log'
    $probeArguments = @(
      '-device', 'STM32F407VE', '-if', 'SWD', '-speed', "$SpeedKHz",
      '-SelectEmuBySN', $ProbeSerial, '-autoconnect', '1', '-NoGui', '1',
      '-ExitOnError', '1', '-CommanderScript', $flashCommands
    )
    Write-Host "Flashing through J-Link $ProbeSerial (SWD $SpeedKHz kHz)..."
    $flashOutput = Invoke-LoggedTool $JLink $probeArguments $flashLog
    if (($flashOutput -match '(?im)Error:|Cannot connect|Could not|FAILED') -or
        ($flashOutput -notmatch 'Verifying flash|Flash contents already match|Flash download skipped') -or
        ($flashOutput -notmatch 'Script processing completed')) {
      throw "Flash verification was not confirmed. Log: $flashLog"
    }
    $manifest['flashVerified'] = $true
    $manifest['flashedAtUtc'] = [DateTime]::UtcNow.ToString('o')
    $manifest['probeSerial'] = $ProbeSerial
    $manifest['speedKHz'] = $SpeedKHz
    $manifest['flashLog'] = $flashLog
    $manifest | ConvertTo-Json | Set-Content -LiteralPath $manifestPath -Encoding UTF8
    Write-Host 'Flash verified; MCU reset and running. No button or PWM commands sent.'
  }

  if ($Action -eq 'Inspect') {
    New-Item -ItemType Directory -Path $StagingDirectory -Force | Out-Null
    $inspectCommands = Join-Path $StagingDirectory 'inspect.jlink'
    # Non-halting reads only. J-Link mem32 counts are hexadecimal.
    # PSC/ARR/RCR/CCR1..4; GPIO D IDR/ODR; GPIO C IDR; IWDG PR/RLR/SR.
    @(
      'mem32 0x40010028 0x7', 'mem32 0x40020C10 0x2',
      'mem32 0x40020810 0x1', 'mem32 0x40003004 0x3', 'exit'
    ) | Set-Content -LiteralPath $inspectCommands -Encoding ASCII
    $inspectArguments = @(
      '-device', 'STM32F407VE', '-if', 'SWD', '-speed', "$SpeedKHz",
      '-SelectEmuBySN', $ProbeSerial, '-autoconnect', '1', '-NoGui', '1',
      '-ExitOnError', '1', '-CommanderScript', $inspectCommands
    )
    $inspectLog = Join-Path $logDirectory 'jlink-inspect.log'
    $inspectOutput = Invoke-LoggedTool $JLink $inspectArguments $inspectLog
    if ($inspectOutput -match '(?im)Error:|Cannot connect|Could not|FAILED') {
      throw "Register read failed. Log: $inspectLog"
    }
    $inspectOutput -split '\r?\n' | Where-Object {
      $_ -match '^400[0-9A-Fa-f]+ = '
    } | ForEach-Object { Write-Host $_ }
  }
  Write-Host "Logs: $logDirectory"
}
finally {
  Pop-Location
}
