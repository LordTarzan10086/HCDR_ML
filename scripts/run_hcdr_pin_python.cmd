@echo off
setlocal

set "PYTHON_EXE=C:\Users\A\miniconda3\envs\hcdr_pin\python.exe"
if not exist "%PYTHON_EXE%" (
  echo hcdr_pin python.exe not found: %PYTHON_EXE% 1>&2
  exit /b 1
)

if "%OMP_NUM_THREADS%"=="" set "OMP_NUM_THREADS=1"
if "%MKL_NUM_THREADS%"=="" set "MKL_NUM_THREADS=1"
if "%NUMEXPR_NUM_THREADS%"=="" set "NUMEXPR_NUM_THREADS=1"
if "%MKL_THREADING_LAYER%"=="" set "MKL_THREADING_LAYER=SEQUENTIAL"
if "%KMP_DUPLICATE_LIB_OK%"=="" set "KMP_DUPLICATE_LIB_OK=TRUE"

"%PYTHON_EXE%" %*
exit /b %ERRORLEVEL%
