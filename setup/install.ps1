<#
.SYNOPSIS
  One-click installer: WSL2 + Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic (+ Nav2 / SLAM / TurtleBot3).

.DESCRIPTION
  1. checks Windows / WSL / disk space
  2. downloads the official Ubuntu 24.04 WSL image (SHA256 verified) and imports it to -InstallDir
  3. writes/merges %USERPROFILE%\.wslconfig (memory / CPU limits; existing values are never overwritten)
  4. runs linux/setup.sh inside the distro (mirror auto-selection, ROS 2, Gazebo, GPU env)
  5. runs linux/verify.sh

  Safe to re-run: it refuses to touch an existing distro with the same name.
  Run:  powershell -ExecutionPolicy Bypass -File .\install.ps1
  Works on Windows PowerShell 5.1 and PowerShell 7. Messages are ASCII on purpose.

.EXAMPLE
  .\install.ps1
.EXAMPLE
  .\install.ps1 -InstallDir E:\WSL\ros -Mirror aliyun -SkipRobots -SetDefault
#>
[CmdletBinding()]
param(
  [string]$DistroName = 'Ubuntu-24.04',
  [string]$InstallDir = '',                 # default: <drive with most free space>:\WSL\<DistroName>
  [string]$UserName   = 'ros',
  [ValidateSet('auto','huawei','aliyun','tuna','ustc','official')]
  [string]$Mirror     = 'auto',
  [string]$RootfsPath = '',                 # use a local .wsl/.tar.gz instead of downloading (offline)
  [int]$MemoryGB      = 0,                  # 0 = auto (half of RAM, 4..16 GB)
  [int]$Processors    = 0,                  # 0 = auto (3/4 of logical CPUs)
  [int]$SwapGB        = 8,
  [string]$RosKeyFile = '',                 # local ros.key for offline / blocked networks
  [switch]$SkipRobots,                      # skip Nav2 / SLAM Toolbox / TurtleBot3
  [switch]$SkipWslConfig,                   # do not touch .wslconfig
  [switch]$SkipVerify,
  [switch]$SetDefault,                      # make this distro the default for plain `wsl`
  [switch]$Backup,                          # export a .tar backup when done
  [string]$BackupDir  = '',
  [switch]$SimulatePackages                 # apt-get -s: test the whole flow without installing packages
)

$ErrorActionPreference = 'Stop'
$ProgressPreference    = 'SilentlyContinue'
$env:WSL_UTF8 = '1'
try { [Console]::OutputEncoding = [Text.Encoding]::UTF8 } catch {}
try { [Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12 } catch {}

function Step($m) { Write-Host ""; Write-Host "==> $m" -ForegroundColor Cyan }
function Info($m) { Write-Host "    $m" }
function Warn2($m){ Write-Host "[WARN] $m" -ForegroundColor Yellow }
function Fail($m) { Write-Host "[ERROR] $m" -ForegroundColor Red; exit 1 }

function Get-WebText($url) {
  $r = Invoke-WebRequest -Uri $url -UseBasicParsing -TimeoutSec 60
  if ($r.Content -is [byte[]]) { return [Text.Encoding]::UTF8.GetString($r.Content) }
  return [string]$r.Content
}
# wsl.exe wrapper: returns output lines, strips NULs (older wsl.exe prints UTF-16)
function Invoke-Wsl {
  $old = $ErrorActionPreference; $ErrorActionPreference = 'Continue'
  try { $out = & wsl.exe @args 2>&1; $script:WslRc = $LASTEXITCODE } finally { $ErrorActionPreference = $old }
  return ($out | ForEach-Object { ("$_") -replace "`0", '' })
}

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$linuxSrc = Join-Path $here 'linux'
if (-not (Test-Path (Join-Path $linuxSrc 'setup.sh'))) { Fail "linux\setup.sh not found next to install.ps1 ($linuxSrc)" }

# ------------------------------------------------------------------ 1. preflight
Step '1/6 Preflight checks'
$build = [Environment]::OSVersion.Version.Build
Info "Windows build: $build"
if ($build -lt 19041) { Fail 'WSL2 needs Windows 10 2004 (build 19041) or newer.' }
if ($build -lt 19044) { Warn2 'WSLg (Linux GUI windows) needs Windows 10 21H2+ or Windows 11. Gazebo/RViz windows will not show.' }

if (-not (Get-Command wsl.exe -ErrorAction SilentlyContinue)) {
  Fail 'wsl.exe not found. Enable WSL first (admin PowerShell): wsl --install --no-distribution   then reboot, and re-run this script.'
}
$null = Invoke-Wsl --status
if ($script:WslRc -ne 0) {
  Fail 'WSL is not ready. In an ADMIN PowerShell run: wsl --install --no-distribution ; reboot ; then re-run this script. (Also check virtualization is enabled in BIOS.)'
}
$ver = (Invoke-Wsl --version) -join ' '
if ($ver -match '(\d+\.\d+\.\d+)') { Info "WSL version: $($Matches[1])" } else { Warn2 'Old inbox WSL detected. Recommended: wsl --update (admin).' }

if (-not $InstallDir) {
  $vol = Get-Volume | Where-Object { $_.DriveLetter -and $_.DriveType -eq 'Fixed' } | Sort-Object SizeRemaining -Descending | Select-Object -First 1
  if (-not $vol) { Fail 'No fixed drive found; pass -InstallDir.' }
  $InstallDir = '{0}:\WSL\{1}' -f $vol.DriveLetter, $DistroName
}
$InstallDir = [IO.Path]::GetFullPath($InstallDir)
$drive = (Split-Path -Qualifier $InstallDir).TrimEnd(':')
$freeGB = [math]::Round((Get-Volume -DriveLetter $drive).SizeRemaining / 1GB, 1)
Info "Install dir: $InstallDir  (drive ${drive}: free ${freeGB} GB)"
if ($freeGB -lt 40) { Fail "Need at least 40 GB free on ${drive}: (have $freeGB GB). Use -InstallDir on another drive." }

$existing = @(Invoke-Wsl -l -q | ForEach-Object { $_.Trim() } | Where-Object { $_ })
if ($existing -contains $DistroName) {
  Fail "A WSL distro named '$DistroName' already exists. Choose another -DistroName, or remove it yourself first (wsl --unregister $DistroName deletes ALL its data)."
}
if ((Test-Path $InstallDir) -and (Get-ChildItem $InstallDir -Force -ErrorAction SilentlyContinue)) {
  Fail "$InstallDir exists and is not empty. Choose an empty -InstallDir."
}

# ------------------------------------------------------------------ 2. rootfs
Step '2/6 Ubuntu 24.04 rootfs'
$rootfs = $RootfsPath
if ($rootfs) {
  if (-not (Test-Path $rootfs)) { Fail "RootfsPath not found: $rootfs" }
  Info "Using local rootfs: $rootfs"
} else {
  $base = 'https://releases.ubuntu.com/noble/'
  $name = 'ubuntu-24.04.5-wsl-amd64.wsl'; $sha = 'bb415d824822c4b878125729af451a5d18fb13d1cf5cbed9a7393ad64ac6039e'   # fallback pin
  try {
    $page = Get-WebText $base
    $cands = [regex]::Matches($page, 'ubuntu-24\.04\.(\d+)-wsl-amd64\.wsl') | ForEach-Object { [pscustomobject]@{ N = $_.Value; P = [int]$_.Groups[1].Value } } | Sort-Object P -Descending -Unique
    if ($cands) { $name = ($cands | Select-Object -First 1).N }
    $sums = Get-WebText ($base + 'SHA256SUMS')
    $m = [regex]::Match($sums, '(?m)^([0-9a-f]{64}) \*?' + [regex]::Escape($name) + '\s*$')
    if ($m.Success) { $sha = $m.Groups[1].Value } else { Warn2 "No checksum for $name in SHA256SUMS; falling back to pinned $name" ; $name = 'ubuntu-24.04.5-wsl-amd64.wsl'; $sha = 'bb415d824822c4b878125729af451a5d18fb13d1cf5cbed9a7393ad64ac6039e' }
  } catch { Warn2 "Could not read $base ($($_.Exception.Message)); using pinned $name" }
  $dl = Join-Path (Split-Path -Parent $InstallDir) 'downloads'
  New-Item -ItemType Directory -Force $dl | Out-Null
  $rootfs = Join-Path $dl $name
  for ($try = 1; $try -le 2; $try++) {
    if (-not (Test-Path $rootfs) -or ((Get-FileHash $rootfs -Algorithm SHA256).Hash.ToLower() -ne $sha)) {
      Info "Downloading $name (~370 MB) ..."
      & curl.exe -L --fail --retry 5 --retry-delay 3 -C - -o $rootfs ($base + $name)
      if ($LASTEXITCODE -ne 0) { Remove-Item $rootfs -ErrorAction SilentlyContinue; if ($try -eq 2) { Fail 'Download failed. Download the file manually and pass -RootfsPath.' } }
    }
    $actual = (Get-FileHash $rootfs -Algorithm SHA256).Hash.ToLower()
    if ($actual -eq $sha) { Info "SHA256 OK ($sha)"; break }
    Remove-Item $rootfs -Force
    if ($try -eq 2) { Fail "SHA256 mismatch (expected $sha, got $actual). Aborting." }
  }
}

# ------------------------------------------------------------------ 3. .wslconfig
Step '3/6 .wslconfig'
if ($SkipWslConfig) {
  Info 'skipped (-SkipWslConfig)'
} else {
  $ramGB = [math]::Round((Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory / 1GB)
  $cpus  = (Get-CimInstance Win32_ComputerSystem).NumberOfLogicalProcessors
  if ($MemoryGB   -le 0) { $MemoryGB   = [int][math]::Min(16, [math]::Max(4, [math]::Floor($ramGB / 2))) }
  if ($Processors -le 0) { $Processors = [int][math]::Max(2, [math]::Floor($cpus * 3 / 4)) }
  $want = [ordered]@{ 'memory' = "${MemoryGB}GB"; 'processors' = "$Processors"; 'swap' = "${SwapGB}GB"; 'autoProxy' = 'true' }
  if ($build -ge 22621) { $want['networkingMode'] = 'mirrored' }   # Win11 22H2+

  $cfg = Join-Path $env:USERPROFILE '.wslconfig'
  $lines = New-Object System.Collections.Generic.List[string]
  if (Test-Path $cfg) {
    $bak = "$cfg.bak-" + (Get-Date -Format 'yyyyMMdd-HHmmss'); Copy-Item $cfg $bak
    Info "existing config backed up to $bak"
    Get-Content $cfg | ForEach-Object { $lines.Add($_) }
  }
  $h = -1; for ($i = 0; $i -lt $lines.Count; $i++) { if ($lines[$i].Trim() -ieq '[wsl2]') { $h = $i; break } }
  if ($h -lt 0) { if ($lines.Count -gt 0) { $lines.Add('') }; $lines.Add('[wsl2]'); $h = $lines.Count - 1 }
  $end = $lines.Count; for ($i = $h + 1; $i -lt $lines.Count; $i++) { if ($lines[$i].Trim().StartsWith('[')) { $end = $i; break } }
  foreach ($k in $want.Keys) {
    $present = $false
    for ($i = $h + 1; $i -lt $end; $i++) { if ($lines[$i] -match ('^\s*' + [regex]::Escape($k) + '\s*=')) { $present = $true; break } }
    if ($present) { Info "keep existing $k" } else { $lines.Insert($h + 1, "$k=$($want[$k])"); $end++; Info "add  $k=$($want[$k])" }
  }
  [IO.File]::WriteAllLines($cfg, $lines.ToArray(), (New-Object Text.UTF8Encoding($false)))
  Info 'Note: limits take effect after the WSL VM restarts (wsl --shutdown).'
}

# ------------------------------------------------------------------ 4. import
Step "4/6 Import distro '$DistroName'"
New-Item -ItemType Directory -Force $InstallDir | Out-Null
$null = Invoke-Wsl --import $DistroName $InstallDir $rootfs --version 2
if ($script:WslRc -ne 0) { Fail "wsl --import failed (exit $script:WslRc)." }
if ($SetDefault) { $null = Invoke-Wsl --set-default $DistroName; Info "default distro -> $DistroName" }

# ------------------------------------------------------------------ 5. setup inside distro
Step '5/6 Install ROS 2 / Gazebo inside the distro (20-60 min depending on network)'
$stage = Join-Path ([IO.Path]::GetTempPath()) ('ros-wsl-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Force $stage | Out-Null
foreach ($f in Get-ChildItem $linuxSrc -Filter '*.sh') {      # normalise CRLF -> LF
  $txt = [IO.File]::ReadAllText($f.FullName) -replace "`r`n", "`n"
  [IO.File]::WriteAllText((Join-Path $stage $f.Name), $txt, (New-Object Text.UTF8Encoding($false)))
}
if ($RosKeyFile) { Copy-Item $RosKeyFile (Join-Path $stage 'ros.key') }
$stageWsl = (Invoke-Wsl -d $DistroName -u root --exec wslpath -a ($stage -replace '\\', '/') | Select-Object -First 1).Trim()
$null = Invoke-Wsl -d $DistroName -u root --exec mkdir -p /opt/ros-installer
$null = Invoke-Wsl -d $DistroName -u root --exec cp -r "$stageWsl/." /opt/ros-installer/
Remove-Item $stage -Recurse -Force -ErrorAction SilentlyContinue

$logDir = Join-Path (Split-Path -Parent $InstallDir) 'logs'
New-Item -ItemType Directory -Force $logDir | Out-Null
$log = Join-Path $logDir ("install-{0}-{1}.log" -f $DistroName, (Get-Date -Format 'yyyyMMdd-HHmmss'))
Info "log: $log"

$setupArgs = @('--user', $UserName, '--mirror', $Mirror)
if ($SkipRobots)       { $setupArgs += '--no-robots' }
if ($SimulatePackages) { $setupArgs += '--simulate' }
if ($RosKeyFile)       { $setupArgs += @('--ros-key-file', '/opt/ros-installer/ros.key') }

$old = $ErrorActionPreference; $ErrorActionPreference = 'Continue'
& wsl.exe -d $DistroName -u root --exec bash /opt/ros-installer/setup.sh @setupArgs 2>&1 | ForEach-Object { ("$_") -replace "`0", '' } | Tee-Object -FilePath $log
$rc = $LASTEXITCODE
$ErrorActionPreference = $old
if ($rc -ne 0) { Fail "setup.sh failed (exit $rc). See $log . Fix the cause and re-run (setup.sh is idempotent: wsl -d $DistroName -u root --exec bash /opt/ros-installer/setup.sh --user $UserName)." }

# restart the distro so systemd + default user from /etc/wsl.conf take effect
$null = Invoke-Wsl --terminate $DistroName

# ------------------------------------------------------------------ 6. verify / backup
if ($SkipVerify -or $SimulatePackages) {
  Step '6/6 Verification skipped'
} else {
  Step '6/6 Verification'
  $old = $ErrorActionPreference; $ErrorActionPreference = 'Continue'
  & wsl.exe -d $DistroName -u $UserName --exec bash /opt/ros-installer/verify.sh 2>&1 | ForEach-Object { ("$_") -replace "`0", '' }
  $vrc = $LASTEXITCODE
  $ErrorActionPreference = $old
  if ($vrc -ne 0) { Warn2 "verify.sh reported $vrc failed check(s). The install may still be usable; see output above." }
}

if ($Backup) {
  if (-not $BackupDir) { $BackupDir = Join-Path (Split-Path -Parent $InstallDir) 'backup' }
  New-Item -ItemType Directory -Force $BackupDir | Out-Null
  $tar = Join-Path $BackupDir ("{0}-ros-jazzy-{1}.tar" -f $DistroName, (Get-Date -Format 'yyyyMMdd'))
  Info "exporting backup to $tar ..."
  $null = Invoke-Wsl --export $DistroName $tar
}

Write-Host ""
Write-Host "DONE. Distro '$DistroName' installed at $InstallDir" -ForegroundColor Green
Write-Host "  Enter it :  wsl -d $DistroName"
if (-not $SetDefault) { Write-Host "  (make it the default: wsl --set-default $DistroName)" }
Write-Host "  Try it   :  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
Write-Host "  Apply .wslconfig limits: wsl --shutdown   (then start the distro again)"
Write-Host "  Full-sim check: wsl -d $DistroName -- bash /opt/ros-installer/verify.sh --tb3"
