@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
py -3 -m venv "%SCRIPT_DIR%.venv"
"%SCRIPT_DIR%.venv\Scripts\python.exe" -m pip install --upgrade pip
"%SCRIPT_DIR%.venv\Scripts\python.exe" -m pip install -r "%SCRIPT_DIR%requirements.txt"

endlocal
