# Copyright (c) 2026 Jim Schreckengast
# SPDX-License-Identifier: MIT OR Apache-2.0
#
# Package an already-built (and, in CI, already-signed) intersection-lab.exe as a
# portable zip and an MSI installer. Requires the WiX v5 .NET tool with the UI
# extension:
#   dotnet tool install --global wix --version 5.0.2
#   wix extension add --global WixToolset.UI.wixext/5.0.2
#
# usage: packaging/windows/package.ps1 -Arch x64|arm64 [-OutDir dist]
param(
    [Parameter(Mandatory = $true)][ValidateSet('x64', 'arm64')][string]$Arch,
    [string]$OutDir = 'dist'
)
$ErrorActionPreference = 'Stop'
Set-Location (Join-Path $PSScriptRoot '..\..')

$target = @{ x64 = 'x86_64-pc-windows-msvc'; arm64 = 'aarch64-pc-windows-msvc' }[$Arch]
$version = (Select-String -Path Cargo.toml -Pattern '^version = "(.*)"' | Select-Object -First 1).Matches[0].Groups[1].Value
$exe = "target\$target\release\intersection-lab.exe"
if (-not (Test-Path $exe)) { throw "$exe not found: run cargo build --release --locked --target $target first" }

$stage = "target\package-windows-$Arch\stage"
$work = "target\package-windows-$Arch"
Remove-Item -Recurse -Force $work -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force $stage, $OutDir | Out-Null

Copy-Item $exe $stage
Copy-Item LICENSE-MIT "$stage\LICENSE-MIT.txt"
Copy-Item LICENSE-APACHE "$stage\LICENSE-APACHE.txt"
Copy-Item COPYRIGHT "$stage\COPYRIGHT.txt"
Copy-Item THIRD_PARTY_LICENSES.md "$stage\THIRD_PARTY_LICENSES.txt"

# License page for the installer (RTF): copyright notice and the MIT text.
$text = (Get-Content COPYRIGHT -Raw) + "`n`n" + (Get-Content LICENSE-MIT -Raw)
$body = $text.Replace('\', '\\').Replace('{', '\{').Replace('}', '\}') -replace "`r?`n", "\par`n"
$rtf = "$work\License.rtf"
"{\rtf1\ansi\deff0{\fonttbl{\f0 Segoe UI;}}\f0\fs18 $body}" | Set-Content -Encoding ascii $rtf

$base = "$OutDir\IntersectionLab-$version-windows-$Arch"
Compress-Archive -Path "$stage\*" -DestinationPath "$base.zip" -Force
Write-Host "==> $base.zip"

wix build packaging\windows\main.wxs -arch $Arch -ext WixToolset.UI.wixext `
    -d "Version=$version" -d "StageDir=$stage" -d "IconPath=packaging\windows\app.ico" -d "LicenseRtf=$rtf" `
    -o "$base.msi"
if ($LASTEXITCODE -ne 0) { throw "wix build failed" }
Write-Host "==> $base.msi"
