# Flash the puck's updater (firmware/PuckUpdater) into its second program slot over USB - once, or when the updater
# itself changes. The normal firmware is flashed as usual (./flash.sh Puck, 16 MB layout) and is not touched here.
#   powershell -ExecutionPolicy Bypass -File firmware/puck-updater-usb.ps1 -Port COM3
param([string]$Port = "COM3")
$cli = "C:\Program Files\Arduino CLI\arduino-cli.exe"
$fqbn = "esp32:esp32:esp32s3:CDCOnBoot=default,CPUFreq=240,PSRAM=disabled,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB"
$sk = Join-Path $PSScriptRoot "PuckUpdater"
$bp = Join-Path $env:TEMP "puck-updater-build"
if (-not (Test-Path (Join-Path $sk "secrets.h"))) { "firmware/PuckUpdater/secrets.h missing - copy secrets.example.h and fill in the WiFi"; exit 1 }
$out = & $cli compile --fqbn $fqbn --build-path $bp $sk 2>&1
$out | Select-String -Pattern "error|Sketch uses|Global variables" | ForEach-Object { $_.Line }
if ($LASTEXITCODE -ne 0) { "COMPILE FAILED"; exit 1 }
$et = Get-ChildItem "$env:LOCALAPPDATA\Arduino15\packages\esp32\tools\esptool_py" -Recurse -Filter esptool.exe | Select-Object -First 1 -ExpandProperty FullName
& $et --chip esp32s3 --port $Port write-flash 0x310000 (Join-Path $bp "PuckUpdater.ino.bin") 2>&1 |
  Select-String -Pattern "Wrote|Hash of data verified|error|A fatal" | ForEach-Object { $_.Line }
if ($LASTEXITCODE -ne 0) { "WRITE FAILED"; exit 1 }
"updater written to the second slot (ota_1, 0x310000)"
