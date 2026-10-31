#!/usr/bin/env bash
# Run by flash.sh before compiling Ironman: gzips the browser pages and embeds the bytes in C
# headers, so the board serves them itself (Bench.livePageGz, Content-Encoding: gzip). Plain text
# they were 137 KB and had the app slot at 98 %; gzipped they are about a third of that:
#   web/arm.html -> Ironman/armpage.h  ARM_GZ / ARM_GZ_LEN   the arm simulator, GET :8080/
#   web/hud.html -> Ironman/hudpage.h  HUD_GZ / HUD_GZ_LEN   the telemetry page,  GET :8080/hud
# Edit the html, never the generated headers (both are gitignored). Needs gzip and xxd (Git Bash has both).
set -euo pipefail
cd "$(dirname "$0")/.."
embed() {   # $1 html, $2 C symbol, $3 header
  local gz; gz="$(mktemp)"
  gzip -9 -n -c "$1" > "$gz"
  {
    echo "// GENERATED from $1 by Ironman/prebuild.sh (gzip -9) - edit the html, not this file."
    echo "#pragma once"
    echo "#include <stdint.h>"
    echo "#include <stddef.h>"
    echo "static const uint8_t $2[] = {"
    xxd -i < "$gz"
    echo "};"
    echo "static const size_t ${2}_LEN = sizeof($2);"
  } > "$3"
  echo "  $(basename "$3"): $1 $(wc -c < "$1") bytes -> $(wc -c < "$gz") gzipped"
  rm -f "$gz"
}
embed web/arm.html ARM_GZ Ironman/armpage.h
embed web/hud.html HUD_GZ Ironman/hudpage.h
