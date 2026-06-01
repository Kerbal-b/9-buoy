$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$venvPath = Join-Path $scriptDir ".venv"

py -3 -m venv $venvPath
& "$venvPath/Scripts/python.exe" -m pip install --upgrade pip
& "$venvPath/Scripts/python.exe" -m pip install -r "$scriptDir/requirements.txt"
