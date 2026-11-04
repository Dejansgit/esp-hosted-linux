# xWave: build one of the two firmware variants of network_adapter for the Edge.
#
#   .\xwave-build.ps1 hosted     the ESP-Hosted service (cfg80211 on wlan0):
#                                Espressif's patched Wi-Fi libraries (lib\esp32)
#   .\xwave-build.ps1 mesh       ESP-WIFI-MESH node/root with the ESP's own
#                                TCP/IP stack (xwave_mesh.c): ESP-IDF's stock
#                                Wi-Fi libraries, the only ones whose station
#                                and AP run authentication inside the ESP (the
#                                patched ones defer it to the host: "auth
#                                timeout" against any router, 2026-11-04)
#
# Both variants are one source tree and one sdkconfig; they differ in the
# libraries under esp-idf\components\esp_wifi\lib (a git submodule: the stock
# set is restored with `git submodule update --force`, the patched set copied
# over it as setup.ps1 does) and in the define XWAVE_MESH_IMAGE, which the
# firmware reports in its mesh status so Linux knows which image runs.
# Outputs: network_adapter\out\network_adapter-hosted.bin and -mesh.bin.
# Linux switches between them over SPI (xwave-wifi apply, 35 s), no hands.
#
# Run in PowerShell after esp-idf\export.ps1.
param([Parameter(Mandatory = $true)][ValidateSet("hosted", "mesh")][string]$Variant)

# "Continue": Windows PowerShell 5.1 would otherwise turn any stderr line of a
# native command (a CMake warning) into a terminating error; exit codes decide.
$ErrorActionPreference = "Continue"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root
$libDir = Join-Path $root "esp-idf\components\esp_wifi\lib"

Set-Location (Join-Path $root "network_adapter")
if ($Variant -eq "mesh") {
    Write-Host "xwave-build: stock Wi-Fi libraries (git submodule)"
    git -C ..\esp-idf submodule update --init --force components/esp_wifi/lib
    if ($LASTEXITCODE -ne 0) { throw "submodule update failed" }
    git -C $libDir checkout -- .
    git -C $libDir clean -fdq
    # the one configuration difference: CONFIG_XWAVE_MESH_IMAGE, in a copy of
    # sdkconfig so the hosted build keeps the file as it is
    $cfg = "sdkconfig.mesh"
    (Get-Content sdkconfig) | Where-Object { $_ -notmatch "^(# )?CONFIG_XWAVE_MESH_IMAGE\b" } | Set-Content $cfg -Encoding ascii
    Add-Content $cfg "CONFIG_XWAVE_MESH_IMAGE=y" -Encoding ascii
} else {
    Write-Host "xwave-build: Espressif's patched Wi-Fi libraries (lib\esp*)"
    Copy-Item "..\lib\esp*" $libDir -Recurse -Force
    $cfg = "sdkconfig"
}

idf.py -B "build-$Variant" "-DSDKCONFIG=$cfg" build
if ($LASTEXITCODE -ne 0) { throw "idf.py build failed" }
New-Item -ItemType Directory -Force -Path out | Out-Null
Copy-Item "build-$Variant\network_adapter.bin" "out\network_adapter-$Variant.bin" -Force
$f = Get-Item "out\network_adapter-$Variant.bin"
Write-Host ("xwave-build: {0} {1} bytes, sha256 {2}" -f $f.Name, $f.Length, (Get-FileHash $f.FullName -Algorithm SHA256).Hash.Substring(0, 12).ToLower())
