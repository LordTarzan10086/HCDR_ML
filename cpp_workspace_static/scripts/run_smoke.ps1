param(
    [string]$BuildDir = "build",
    [string]$Configuration = "Release",
    [string]$CondaEnv = "C:\Users\A\miniconda3\envs\hcdr_pin"
)

$ErrorActionPreference = "Stop"

$WorkspaceRoot = Split-Path -Parent $PSScriptRoot
$RunnerExe = Join-Path $WorkspaceRoot "$BuildDir\$Configuration\workspace_static_runner.exe"
$SmokeConfig = Join-Path $WorkspaceRoot "config\workspace_static_smoke.json"

if (-not (Test-Path $RunnerExe)) {
    throw "workspace_static_runner.exe not found. Build first with scripts\build_vs2019.ps1"
}

$env:PATH = "$CondaEnv\Library\bin;$env:PATH"
& $RunnerExe --config $SmokeConfig
