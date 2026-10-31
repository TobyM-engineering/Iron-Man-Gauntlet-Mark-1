# Console to the board over WiFi. Keys you type go straight to the menu.
#   powershell -ExecutionPolicy Bypass -File monitor.ps1 [-Target ironman.local]
# Ctrl+C to exit.
#
# Polls GET /log every 250 ms and sends keystrokes with GET /key. Plain
# request/response - nothing the board has to push on its own.
param([string]$Target = "ironman.local")
$ProgressPreference = "SilentlyContinue"
$base = "http://$Target"
$cursor = 0
$down = $false
Write-Host "console @ $base   (Ctrl+C to quit)" -ForegroundColor DarkGray

while ($true) {
  try {
    $r = Invoke-WebRequest -Uri "$base/log?from=$cursor" -UseBasicParsing -TimeoutSec 15
    $head = [uint32]$r.Headers["X-Log-Head"]
    if ($down) { Write-Host "`n[board back]" -ForegroundColor DarkGray; $down = $false }
    if ($head -lt $cursor) { Write-Host "`n[board rebooted]" -ForegroundColor DarkGray }
    if ($r.Content.Length) { [Console]::Out.Write($r.Content) }
    $cursor = $head
  } catch {
    if (-not $down) { Write-Host "`n[waiting for board at $base]" -ForegroundColor DarkGray; $down = $true }
    Start-Sleep -Milliseconds 1000
  }
  # KeyAvailable throws without an interactive console (piped/redirected);
  # in that case just keep streaming the log read-only.
  try {
    while ([Console]::KeyAvailable) {
      $k = [Console]::ReadKey($true)
      if ($k.KeyChar) {
        $c = [Uri]::EscapeDataString([string]$k.KeyChar)
        try { Invoke-WebRequest -Uri "$base/key?c=$c" -UseBasicParsing -TimeoutSec 15 | Out-Null } catch {}
      }
    }
  } catch {}
  Start-Sleep -Milliseconds 250
}
