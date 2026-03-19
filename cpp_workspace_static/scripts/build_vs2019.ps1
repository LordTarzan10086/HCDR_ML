param(
    [string]$BuildDir = "build",
    [string]$CondaEnv = "C:\Users\A\miniconda3\envs\hcdr_pin",
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"

$WorkspaceRoot = Split-Path -Parent $PSScriptRoot
$CmakeExe = "C:\Program Files\MATLAB\R2024b\bin\win64\cmake\bin\cmake.exe"

if (-not (Test-Path $CmakeExe)) {
    throw "cmake.exe not found at $CmakeExe"
}

& $CmakeExe `
    -S $WorkspaceRoot `
    -B (Join-Path $WorkspaceRoot $BuildDir) `
    -G "Visual Studio 16 2019" `
    -A x64 `
    -DHCDR_PIN_ENV="$CondaEnv" `
    -DCMAKE_PREFIX_PATH="$CondaEnv;$CondaEnv\Library"

& $CmakeExe --build (Join-Path $WorkspaceRoot $BuildDir) --config $Configuration
