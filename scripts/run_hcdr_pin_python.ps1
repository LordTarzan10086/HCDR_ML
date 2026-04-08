param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]] $PythonArgs
)

$pythonExe = "C:\Users\A\miniconda3\envs\hcdr_pin\python.exe"
if (-not (Test-Path -LiteralPath $pythonExe)) {
    Write-Error "hcdr_pin python.exe not found: $pythonExe"
    exit 1
}

# Set native runtime knobs before Python loads NumPy/Pinocchio/MuJoCo.
if (-not $env:OMP_NUM_THREADS) { $env:OMP_NUM_THREADS = "1" }
if (-not $env:MKL_NUM_THREADS) { $env:MKL_NUM_THREADS = "1" }
if (-not $env:NUMEXPR_NUM_THREADS) { $env:NUMEXPR_NUM_THREADS = "1" }
if (-not $env:MKL_THREADING_LAYER) { $env:MKL_THREADING_LAYER = "SEQUENTIAL" }
if (-not $env:KMP_DUPLICATE_LIB_OK) { $env:KMP_DUPLICATE_LIB_OK = "TRUE" }

& $pythonExe @PythonArgs
exit $LASTEXITCODE
