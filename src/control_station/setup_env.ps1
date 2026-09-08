$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$venvPath = Join-Path $scriptDir ".venv"

if (-not (Get-Command py -ErrorAction SilentlyContinue)) {
    throw "The Python Launcher for Windows (py.exe) is required. Install Python 3.13 and enable the launcher."
}

py -3.13 -c "import sys; print(sys.version)"
if ($LASTEXITCODE -ne 0) {
    throw "Python 3.13 is required for pygame. Install Python 3.13 x64, then run this script again."
}

if (Test-Path -LiteralPath $venvPath) {
    throw "The existing environment uses an incompatible or unknown Python version. Remove '$venvPath' and rerun this script."
}

py -3.13 -m venv $venvPath
& "$venvPath/Scripts/python.exe" -m pip install --upgrade pip
& "$venvPath/Scripts/python.exe" -m pip install -r "$scriptDir/requirements.txt"
