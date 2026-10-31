# Update the puck's normal firmware over WiFi. First put the puck in update mode: hold a finger on its screen while
# powering it up (~3 s, until it says UPDATE MODE), or type "update" on its USB console. Its screen then shows its
# address; puck.local works where mDNS does.
#   powershell -ExecutionPolicy Bypass -File firmware/puck-ota.ps1 -Address 192.168.1.60
param([string]$Address = "puck.local", [switch]$NoCompile)
$cli = "C:\Program Files\Arduino CLI\arduino-cli.exe"
$fqbn = "esp32:esp32:esp32s3:CDCOnBoot=default,CPUFreq=240,PSRAM=disabled,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB"
$sk = Join-Path $PSScriptRoot "Puck"
$bp = Join-Path $env:TEMP "puck-build"
if (-not $NoCompile) {
  $out = & $cli compile --fqbn $fqbn --build-path $bp $sk 2>&1
  $out | Select-String -Pattern "error|Sketch uses|Global variables" | ForEach-Object { $_.Line }
  if ($LASTEXITCODE -ne 0) { "COMPILE FAILED"; exit 1 }
}
$bin = Join-Path $bp "Puck.ino.bin"
"pushing $([int]((Get-Item $bin).Length / 1024)) KB to http://$Address/update ..."
curl.exe -sS --max-time 180 -F "image=@$bin" "http://$Address/update"
if ($LASTEXITCODE -ne 0) { "PUSH FAILED (curl exit $LASTEXITCODE)"; exit 1 }
