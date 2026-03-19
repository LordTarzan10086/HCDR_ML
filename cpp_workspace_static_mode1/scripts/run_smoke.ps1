param(
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"

$WorkspaceRoot = Split-Path -Parent $PSScriptRoot
$BuildScript = Join-Path $PSScriptRoot "build_vs2019.ps1"
$Runner = Join-Path $WorkspaceRoot "build\$Configuration\mode1_workspace_runner.exe"
$Config = Join-Path $WorkspaceRoot "config\workspace_static_mode1_smoke.json"
$CondaBin = "C:\Users\A\miniconda3\envs\hcdr_pin\Library\bin"

& powershell -ExecutionPolicy Bypass -File $BuildScript -Configuration $Configuration
$env:PATH = "$CondaBin;$env:PATH"
& $Runner --config $Config
