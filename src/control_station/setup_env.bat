@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
py -3.13 -c "import sys; print(sys.version)"
if errorlevel 1 (
    echo Python 3.13 is required for pygame. Install Python 3.13 x64, then run this script again.
    exit /b 1
)

if exist "%SCRIPT_DIR%.venv" (
    echo The existing .venv must be removed before recreating it with Python 3.13.
    exit /b 1
)

py -3.13 -m venv "%SCRIPT_DIR%.venv"
"%SCRIPT_DIR%.venv\Scripts\python.exe" -m pip install --upgrade pip
"%SCRIPT_DIR%.venv\Scripts\python.exe" -m pip install -r "%SCRIPT_DIR%requirements.txt"

endlocal
