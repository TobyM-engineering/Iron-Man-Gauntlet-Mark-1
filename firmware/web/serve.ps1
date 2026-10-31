param([int]$Port = 8000, [string]$Root = $PSScriptRoot)   # -Root app serves the Gauntlet app (app/Gauntlet.html + img/) for a preview
# Tiny static server for developing sim/arm.html and sim/hud.html without reflashing the board:
#   powershell -ExecutionPolicy Bypass -File sim/serve.ps1
#   -> http://localhost:8000/arm.html?host=ironman.local   (or ?demo=1 with no board)
#   -> http://localhost:8000/hud.html?host=ironman.local   the telemetry page, same switches
# The board serves the same page itself at http://ironman.local:8080/ once flashed.
$root = $Root
$listener = New-Object System.Net.HttpListener
$listener.Prefixes.Add("http://localhost:$Port/")
$listener.Start()
Write-Host "serving $root at http://localhost:$Port/arm.html  (Ctrl+C to stop)"
$types = @{ ".html" = "text/html; charset=utf-8"; ".js" = "text/javascript"; ".css" = "text/css"; ".json" = "application/json"; ".png" = "image/png"; ".svg" = "image/svg+xml"; ".jpg" = "image/jpeg"; ".jpeg" = "image/jpeg" }
while ($listener.IsListening) {
  $ctx = $listener.GetContext()
  $res = $ctx.Response
  try {
    $path = [Uri]::UnescapeDataString($ctx.Request.Url.AbsolutePath)
    if ($path -eq "/") { $path = "/arm.html" }
    $file = Join-Path $root $path.TrimStart("/")
    if (Test-Path $file -PathType Leaf) {
      $bytes = [IO.File]::ReadAllBytes($file)
      $ext = [IO.Path]::GetExtension($file).ToLower()
      if ($types.ContainsKey($ext)) { $res.ContentType = $types[$ext] } else { $res.ContentType = "application/octet-stream" }
      $res.Headers.Add("Cache-Control", "no-store")
      $res.ContentLength64 = $bytes.Length
      $res.OutputStream.Write($bytes, 0, $bytes.Length)
    } else {
      $res.StatusCode = 404
      $b = [Text.Encoding]::UTF8.GetBytes("not found: $path")
      $res.OutputStream.Write($b, 0, $b.Length)
    }
  } catch { $res.StatusCode = 500 }
  $res.Close()
}
