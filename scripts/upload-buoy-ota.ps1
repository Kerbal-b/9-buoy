param(
    [Parameter(Mandatory = $true)]
    [string]$BuoyAddress
)

$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent $PSScriptRoot
$sketchPath = Join-Path $projectRoot "src\firmware\esp32\esp32-buoy-firmware"
$buildPath = Join-Path $projectRoot ".build\buoy-firmware"
$fqbn = "esp32:esp32:esp32"

$cliCommand = Get-Command arduino-cli -ErrorAction SilentlyContinue
if ($cliCommand) {
    $arduinoCli = $cliCommand.Source
} else {
    $arduinoCli = Join-Path $env:LOCALAPPDATA "Arduino15\arduino-cli.exe"
    if (-not (Test-Path -LiteralPath $arduinoCli)) {
        $arduinoCli = "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
    }
}
if (-not (Test-Path -LiteralPath $arduinoCli)) {
    throw "Could not find arduino-cli. Install Arduino CLI or Arduino IDE."
}

Write-Host "Building buoy firmware..."
& $arduinoCli compile --fqbn $fqbn --build-path $buildPath $sketchPath
if ($LASTEXITCODE -ne 0) {
    throw "Firmware compile failed with exit code $LASTEXITCODE."
}

$securePassword = Read-Host "Buoy OTA password (same as buoy Wi-Fi password)" -AsSecureString
$passwordPointer = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($securePassword)
try {
    $password = [Runtime.InteropServices.Marshal]::PtrToStringBSTR($passwordPointer)
    Write-Host "Uploading to $BuoyAddress over Wi-Fi. Keep the buoy powered and disarmed."
    & $arduinoCli upload --fqbn $fqbn --port $BuoyAddress --protocol network --discovery-timeout 15s --input-dir $buildPath --upload-field "password=$password"
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "Arduino CLI upload failed. Trying the ESP32 OTA uploader directly by IP."
        $coreRoot = Join-Path $env:LOCALAPPDATA "Arduino15\packages\esp32\hardware\esp32"
        $espotaScript = Get-ChildItem -LiteralPath $coreRoot -Directory -ErrorAction Stop |
            Sort-Object LastWriteTime -Descending |
            ForEach-Object { Join-Path $_.FullName "tools\espota.py" } |
            Where-Object { Test-Path -LiteralPath $_ } |
            Select-Object -First 1
        $pythonPath = Join-Path $projectRoot "src\control_station\.venv\Scripts\python.exe"
        $firmwareImage = Join-Path $buildPath "esp32-buoy-firmware.ino.bin"
        if (-not $espotaScript -or -not (Test-Path -LiteralPath $pythonPath) -or
            -not (Test-Path -LiteralPath $firmwareImage)) {
            throw "Direct OTA fallback needs espota.py, the control-station Python environment, and the compiled firmware image."
        }
        & $pythonPath $espotaScript -i $BuoyAddress -p 3232 -a $password -f $firmwareImage
        if ($LASTEXITCODE -ne 0) {
            throw "Direct OTA upload failed with exit code $LASTEXITCODE."
        }
    }
} finally {
    if ($passwordPointer -ne [IntPtr]::Zero) {
        [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($passwordPointer)
    }
    $password = $null
}

Write-Host "OTA upload completed. The buoy should restart into the updated firmware."
