# Compiles and runs the GauntletLink host test (host_test.cpp) with the PC's g++ (WinLibs MinGW here).
#   powershell -ExecutionPolicy Bypass -File lib/GauntletLink/test/run.ps1
# Exit code: 0 = every test passed, 1 = a test failed, 2 = no compiler / the compile failed.
$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$gpp = Get-Command g++ -ErrorAction SilentlyContinue
if (-not $gpp) { Write-Output 'glink host test: no g++ on PATH - install a host compiler (winget install BrechtSanders.WinLibs.POSIX.UCRT)'; exit 2 }
$exe = Join-Path $env:TEMP 'glink-host-test.exe'
& $gpp.Source -std=c++17 -O1 -Wall -Wextra -Werror -Wno-format-truncation -o $exe (Join-Path $here 'host_test.cpp')   # format-truncation: the encoder clamps its fields, which the warning cannot see
if ($LASTEXITCODE -ne 0) { Write-Output 'glink host test: the compile failed'; exit 2 }
& $exe
exit $LASTEXITCODE
