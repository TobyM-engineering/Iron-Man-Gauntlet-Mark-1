#!/usr/bin/env bash
# Console to the board over WiFi (default) or USB.
#   ./monitor.sh                    ironman.local
#   ./monitor.sh ironman            a bare name gets .local -> ironman.local
#   ./monitor.sh 192.168.1.50       by IP
#   ./monitor.sh COM11              serial, cable required
# One-shot peek without the console:  curl ironman.local/log
cd "$(dirname "$0")"
T="${1:-ironman.local}"
if [[ "$T" =~ ^COM[0-9]+$ ]]; then
  exec "C:/Program Files/Arduino CLI/arduino-cli.exe" monitor -p "$T" -c baudrate=115200
fi
[[ "$T" == *.* ]] || T="$T.local"
exec powershell.exe -NoProfile -ExecutionPolicy Bypass -File monitor.ps1 -Target "$T"
