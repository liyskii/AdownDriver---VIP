<#
  AdownDriver - one-click demo connection (Windows laptop).

  Easiest: double-click AdownDriver.bat in the repo folder.

  Usage (PowerShell, in the repo folder):
    .\demo.ps1               everything: hotspot on -> find both Pis -> SSH keys (first time only, password 12345678)
                             -> copy changed code to both Pis -> check/start services -> open dashboard
    .\demo.ps1 -Setup        only redo the SSH keys
    .\demo.ps1 -CockpitOff   disconnect test: stop the cockpit (robot must EMERGENCY_STOP)
    .\demo.ps1 -CockpitOn    bring the cockpit back
    .\demo.ps1 -Restart      restart every AdownDriver service on both Pis
    .\demo.ps1 -Shutdown     power both Pis off safely (then switch off the OpenCR)

  If Windows blocks the script:  powershell -ExecutionPolicy Bypass -File .\demo.ps1
#>
param([switch]$Setup, [switch]$CockpitOff, [switch]$CockpitOn, [switch]$Restart, [switch]$Shutdown, [switch]$NoSync, [switch]$Check)

$SSID      = "adown-net"
$PASS      = "12345678"
$ROBOT     = "192.168.137.220"   # Node 2 (TurtleBot) - fixed address on the hotspot
$COCKPIT   = "192.168.137.210"   # Node 1 (cockpit)
$ROBOT_U   = "ubuntu"
$COCKPIT_U = "pi"
$PI_PASS   = "12345678"          # same password on both Pis (team setup)
$DASH      = "http://${ROBOT}:8000"
$REPO      = $PSScriptRoot
$SYNC_DIRS = @("adowndriver", "config", "sounds", "tools")      # what gets copied to both Pis

function Say($msg, $color = "Gray") { Write-Host $msg -ForegroundColor $color }
function Ok($msg)   { Write-Host "  [ OK ] $msg" -ForegroundColor Green }
function Bad($msg)  { Write-Host "  [FAIL] $msg" -ForegroundColor Red }
function Wait1($msg){ Write-Host "  [ .. ] $msg" -ForegroundColor Yellow }

function Remote($user, $ip, $cmd) {
    # BatchMode: never stop and ask for a password in the middle of the demo
    # ConnectTimeout also bounds the SSH handshake; ServerAlive kills a session whose Pi stopped answering
    $out = & ssh -o BatchMode=yes -o ConnectTimeout=6 -o ServerAliveInterval=3 -o ServerAliveCountMax=3 `
                 -o StrictHostKeyChecking=accept-new "$user@$ip" $cmd 2>&1
    return @{ ok = ($LASTEXITCODE -eq 0); out = ($out -join "`n") }
}

function Test-Up($ip) { return (Test-Connection -ComputerName $ip -Count 1 -Quiet -ErrorAction SilentlyContinue) }

# ------------------------------------------------------------------ hotspot
function Start-Hotspot {
    Say "1) Mobile hotspot '$SSID'" Cyan
    try {
        Add-Type -AssemblyName System.Runtime.WindowsRuntime
        $asTask = ([System.WindowsRuntimeSystemExtensions].GetMethods() | Where-Object {
            $_.Name -eq 'AsTask' -and $_.GetParameters().Count -eq 1 -and
            $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncOperation`1' })[0]
        function Await($op, $type) { $t = $asTask.MakeGenericMethod($type).Invoke($null, @($op)); $t.Wait(-1) | Out-Null; $t.Result }
        function AwaitAction($op) {
            $m = ([System.WindowsRuntimeSystemExtensions].GetMethods() | Where-Object {
                $_.Name -eq 'AsTask' -and $_.GetParameters().Count -eq 1 -and
                $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncAction' })[0]
            $m.Invoke($null, @($op)).Wait(-1) | Out-Null }

        [Windows.Networking.Connectivity.NetworkInformation, Windows.Networking.Connectivity, ContentType = WindowsRuntime] | Out-Null
        [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager, Windows.Networking.NetworkOperators, ContentType = WindowsRuntime] | Out-Null
        $connProfile = [Windows.Networking.Connectivity.NetworkInformation]::GetInternetConnectionProfile()
        if ($null -eq $connProfile) { throw "the laptop has no internet connection to share" }
        $tm = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager]::CreateFromConnectionProfile($connProfile)

        $cfg = $tm.GetCurrentAccessPointConfiguration()
        if ($cfg.Ssid -ne $SSID -or $cfg.Passphrase -ne $PASS) {
            $cfg.Ssid = $SSID; $cfg.Passphrase = $PASS
            AwaitAction ($tm.ConfigureAccessPointAsync($cfg))
            Ok "hotspot renamed to $SSID"
        }
        if ($tm.TetheringOperationalState -eq 1) { Ok "hotspot already on" }
        else {
            $r = Await ($tm.StartTetheringAsync()) ([Windows.Networking.NetworkOperators.NetworkOperatorTetheringOperationResult])
            if ($r.Status -eq 0) { Ok "hotspot switched on" } else { throw "Windows refused: $($r.Status) $($r.AdditionalErrorMessage)" }
        }
    } catch {
        Bad "could not switch the hotspot on automatically ($($_.Exception.Message))"
        Say "     Turn it on by hand: Settings -> Network & internet -> Mobile hotspot (name $SSID, password $PASS)." Yellow
        Say "     No internet here? Use a phone hotspot with the same name and password - both Pis join it by themselves." Yellow
        Start-Process "ms-settings:network-mobilehotspot"
        Read-Host "     Press Enter when the hotspot is on"
    }
}

# ------------------------------------------------------------------ wait for the Pis
function Wait-Nodes($timeoutS = 150) {
    Say "2) Waiting for the robot and the cockpit (power them on now)" Cyan
    $t0 = Get-Date; $r = $false; $c = $false
    while (((Get-Date) - $t0).TotalSeconds -lt $timeoutS) {
        if (-not $r) { $r = Test-Up $ROBOT;   if ($r) { Ok "robot   $ROBOT" } }
        if (-not $c) { $c = Test-Up $COCKPIT; if ($c) { Ok "cockpit $COCKPIT" } }
        if ($r -and $c) { return $true }
        $left = [int]($timeoutS - ((Get-Date) - $t0).TotalSeconds)
        Write-Host ("`r  [ .. ] robot {0}  cockpit {1}  ({2} s left)   " -f ($(if($r){"up"}else{"--"}), $(if($c){"up"}else{"--"}), $left)) -NoNewline -ForegroundColor Yellow
        Start-Sleep -Seconds 2
    }
    Write-Host ""
    if (-not $r) { Bad "robot not found: is it on, is the battery charged, is the hotspot on 2.4 GHz?" }
    if (-not $c) { Bad "cockpit not found: is its USB-C power plugged in?" }
    return ($r -and $c)
}

# ------------------------------------------------------------------ services
function Ensure-Services {
    Say "4) Checking the programs on both Pis" Cyan
    $q = Remote $ROBOT_U $ROBOT "for s in turtlebot3-bringup adown-node2 adown-dashboard; do echo `$s=`$(systemctl is-active `$s); done"
    if (-not $q.ok) { Bad "robot: SSH without password not set up - run .\demo.ps1 -Setup once"; return $false }
    $down = @(); foreach ($l in $q.out -split "`n") { if ($l -match "^(.+)=(.+)$") { if ($Matches[2] -eq "active") { Ok "robot   $($Matches[1])" } else { $down += $Matches[1] } } }
    if ($script:changed -contains $ROBOT) { $down = @("adown-node2", "adown-dashboard") + $down | Select-Object -Unique }
    if ($down.Count) { Wait1 "robot: (re)starting $($down -join ', ')"; Remote $ROBOT_U $ROBOT "sudo systemctl restart $($down -join ' ')" | Out-Null }

    $q = Remote $COCKPIT_U $COCKPIT "systemctl is-active adown-node1"
    if (-not $q.ok -and $q.out -notmatch "inactive|failed|activating") { Bad "cockpit: SSH without password not set up - run .\demo.ps1 -Setup once"; return $false }
    if ($q.out.Trim() -eq "active" -and -not ($script:changed -contains $COCKPIT)) { Ok "cockpit adown-node1" } else { Wait1 "cockpit: starting adown-node1"; Remote $COCKPIT_U $COCKPIT "sudo systemctl restart adown-node1" | Out-Null }
    return $true
}

function Wait-Dashboard($timeoutS = 60) {
    Say "5) Opening the dashboard" Cyan
    $t0 = Get-Date
    while (((Get-Date) - $t0).TotalSeconds -lt $timeoutS) {
        try {
            $st = Invoke-RestMethod -Uri "$DASH/api/state" -TimeoutSec 3 -ErrorAction Stop
            $n1 = $st.online.node1; $n2 = $st.online.node2
            if ($n1 -and $n2) { Ok "dashboard: robot and cockpit online, state $($st.node2.state)"; Start-Process $DASH; return $true }
            Wait1 ("dashboard up, robot {0}, cockpit {1} - waiting" -f $(if($n2){"online"}else{"offline"}), $(if($n1){"online"}else{"offline"}))
        } catch {
            try { Invoke-WebRequest -Uri $DASH -UseBasicParsing -TimeoutSec 3 -ErrorAction Stop | Out-Null; Start-Process $DASH; Ok "dashboard open (status check unavailable)"; return $true } catch { }
        }
        Start-Sleep -Seconds 3
    }
    Bad "dashboard did not come up at $DASH"; Start-Process $DASH; return $false
}

# ------------------------------------------------------------------ lidar (obstacle sensor) check + self-heal
function Get-LidarCm {
    try { $st = Invoke-RestMethod -Uri "$DASH/api/state" -TimeoutSec 3 -ErrorAction Stop; return $st.node2.tof.d_cm } catch { return $null }
}
function Wait-Lidar($timeoutS) {
    $t0 = Get-Date
    while (((Get-Date) - $t0).TotalSeconds -lt $timeoutS) { if ($null -ne (Get-LidarCm)) { return $true }; Start-Sleep -Seconds 2 }
    return $false
}
function Check-Lidar {
    Say "6) Checking the lidar (obstacle sensor)" Cyan
    if (Wait-Lidar 10) { Ok ("lidar OK - {0} cm free ahead" -f [math]::Round((Get-LidarCm), 0)); return $true }
    Wait1 "lidar sends no data - restarting the robot base (motors + lidar), ~30 s"
    $diag = Join-Path $REPO "lidar_diag.txt"
    $d = Remote $ROBOT_U $ROBOT "echo '== usb'; ls -l /dev/ttyUSB* /dev/ttyACM* 2>&1; echo '== bringup'; systemctl is-active turtlebot3-bringup; journalctl -u turtlebot3-bringup -n 40 --no-pager 2>&1 | tail -40; echo '== node2'; journalctl -u adown-node2 -n 20 --no-pager 2>&1 | tail -20"
    $d.out | Out-File -Encoding utf8 $diag
    Remote $ROBOT_U $ROBOT "sudo systemctl restart turtlebot3-bringup; sleep 12; sudo systemctl restart adown-node2" | Out-Null
    if (Wait-Lidar 40) { Ok ("lidar OK after restart - {0} cm free ahead" -f [math]::Round((Get-LidarCm), 0)); return $true }
    $d2 = Remote $ROBOT_U $ROBOT "echo '== after restart'; ls -l /dev/ttyUSB* /dev/ttyACM* 2>&1; journalctl -u turtlebot3-bringup -n 40 --no-pager 2>&1 | tail -40"
    $d2.out | Out-File -Encoding utf8 -Append $diag
    Run-Check
    Bad "lidar still silent. Check: lidar USB cable into the robot Pi, lidar spinning, battery charged, OpenCR switch ON. Details: lidar_diag.txt"
    return $false
}

# ------------------------------------------------------------------ full health check -> check_report.txt
function Run-Check {
    Say "Full check of robot + cockpit (about 40 s)" Cyan
    $rep = Join-Path $REPO "check_report.txt"
    "AdownDriver check $(Get-Date -Format s)" | Out-File -Encoding utf8 $rep
    "`n######## ROBOT ########" | Out-File -Encoding utf8 -Append $rep
    (Remote $ROBOT_U $ROBOT "bash ~/AdownDriver---VIP/tools/robot_check.sh").out | Out-File -Encoding utf8 -Append $rep
    "`n######## COCKPIT ########" | Out-File -Encoding utf8 -Append $rep
    (Remote $COCKPIT_U $COCKPIT "bash ~/AdownDriver---VIP/tools/cockpit_check.sh").out | Out-File -Encoding utf8 -Append $rep
    "`n######## DASHBOARD /api/state ########" | Out-File -Encoding utf8 -Append $rep
    try { (Invoke-WebRequest -Uri "$DASH/api/state" -UseBasicParsing -TimeoutSec 4).Content | Out-File -Encoding utf8 -Append $rep } catch { "dashboard not reachable: $_" | Out-File -Encoding utf8 -Append $rep }
    Ok "report saved: $rep"
}

# ------------------------------------------------------------------ one-time SSH key setup
function Setup-Keys {
    Say "Setting up passwordless SSH (you type each Pi's password once)" Cyan
    $key = Join-Path $env:USERPROFILE ".ssh\id_ed25519"
    if (-not (Test-Path $key)) {
        New-Item -ItemType Directory -Force -Path (Split-Path $key) | Out-Null
        # Start-Process passes '""' through as a real empty passphrase on both PowerShell 5 and 7
        Start-Process ssh-keygen -ArgumentList '-q','-t','ed25519','-f',"`"$key`"",'-N','""' -Wait -NoNewWindow
        if (Test-Path $key) { Ok "key created" } else { Bad "ssh-keygen failed"; return }
    }
    $pub = Get-Content "$key.pub" -Raw
    # The Pis use the team password; feed it to ssh once through SSH_ASKPASS so nobody has to type it.
    $askpass = Join-Path $env:TEMP "adown_askpass.cmd"
    Set-Content -Path $askpass -Value "@echo $PI_PASS" -Encoding ASCII
    $env:SSH_ASKPASS = $askpass; $env:SSH_ASKPASS_REQUIRE = "force"; if (-not $env:DISPLAY) { $env:DISPLAY = "localhost:0" }
    try {
        foreach ($h in @(@($ROBOT_U, $ROBOT), @($COCKPIT_U, $COCKPIT))) {
            Say "  $($h[0])@$($h[1]) - installing the key (if it asks for a password, type $PI_PASS)" Yellow
            $pub | & ssh -o StrictHostKeyChecking=accept-new -o PreferredAuthentications=publickey,password,keyboard-interactive "$($h[0])@$($h[1])" "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && sort -u -o ~/.ssh/authorized_keys ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys && echo KEY_OK"
        }
    } finally {
        Remove-Item $askpass -ErrorAction SilentlyContinue
        Remove-Item Env:SSH_ASKPASS, Env:SSH_ASKPASS_REQUIRE -ErrorAction SilentlyContinue
    }
    foreach ($h in @(@($ROBOT_U, $ROBOT), @($COCKPIT_U, $COCKPIT))) {
        if ((Remote $h[0] $h[1] "true").ok) { Ok "$($h[0])@$($h[1]) works without a password" } else { Bad "$($h[0])@$($h[1]) still asks for a password" }
    }
}


# ------------------------------------------------------------------ keys on demand
function Ensure-Keys {
    # a Pi that has just booted answers ping before SSH is ready - retry for up to 90 s
    Wait1 "connecting to both Pis over SSH"
    $t0 = Get-Date; $ok = $false
    while (((Get-Date) - $t0).TotalSeconds -lt 90) {
        $r = (Remote $ROBOT_U $ROBOT "true").ok; $c = (Remote $COCKPIT_U $COCKPIT "true").ok
        if ($r -and $c) { $ok = $true; break }
        Wait1 ("SSH not ready yet: robot {0}, cockpit {1} - retrying" -f $(if($r){"ok"}else{"waiting"}), $(if($c){"ok"}else{"waiting"}))
        Start-Sleep -Seconds 4
    }
    if ($ok) { Ok "SSH without password works"; return $true }
    Say "   First run on this laptop: saving SSH keys on both Pis." Yellow
    Setup-Keys
    return ((Remote $ROBOT_U $ROBOT "true").ok -and (Remote $COCKPIT_U $COCKPIT "true").ok)
}

# ------------------------------------------------------------------ copy changed code to both Pis
function Sync-Code {
    Say "3) Copying the latest project code to the Pis" Cyan
    $dirs = $SYNC_DIRS | Where-Object { Test-Path (Join-Path $REPO $_) }
    $newest = (Get-ChildItem -Path ($dirs | ForEach-Object { Join-Path $REPO $_ }) -Recurse -File |
               Where-Object { $_.FullName -notmatch "__pycache__" -and $_.Name -ne "local.yaml" } |
               Sort-Object LastWriteTimeUtc -Descending | Select-Object -First 1).LastWriteTimeUtc.ToString("yyyyMMddHHmmss")
    $tgz = Join-Path $env:TEMP "adown_sync.tgz"
    Push-Location $REPO
    # config/local.yaml stays per machine (each Pi keeps its own addresses and demo thresholds)
    & tar -czf $tgz --exclude "__pycache__" --exclude "*.pyc" --exclude "config/local.yaml" $dirs 2>$null
    Pop-Location
    if (-not (Test-Path $tgz)) { Bad "could not pack the code (tar)"; return }
    foreach ($h in @(@($ROBOT_U, $ROBOT, "robot  "), @($COCKPIT_U, $COCKPIT, "cockpit"))) {
        $stamp = (Remote $h[0] $h[1] "cat ~/AdownDriver---VIP/.synced 2>/dev/null").out.Trim()
        if ($stamp -eq $newest) { Ok "$($h[2]) code is up to date"; continue }
        & scp -q -o BatchMode=yes -o ConnectTimeout=6 -o ServerAliveInterval=3 -o ServerAliveCountMax=3 $tgz "$($h[0])@$($h[1]):/tmp/adown_sync.tgz"
        $r = Remote $h[0] $h[1] "cd ~/AdownDriver---VIP && tar xzf /tmp/adown_sync.tgz && echo $newest > .synced"
        if ($r.ok) { Ok "$($h[2]) code updated"; $script:changed += $h[1] } else { Bad "$($h[2]) update failed: $($r.out)" }
    }
}

# ------------------------------------------------------------------ main
if ($Setup) { Setup-Keys; exit }
if ($Check) {
    if (Ensure-Keys) { Sync-Code; Ensure-Services | Out-Null; Wait-Dashboard | Out-Null; Start-Sleep -Seconds 8; Run-Check; Check-Lidar | Out-Null }
    exit
}

if ($CockpitOff) {
    $r = Remote $COCKPIT_U $COCKPIT "sudo systemctl stop adown-node1"
    if ($r.ok) { Ok "cockpit stopped - the robot should be in EMERGENCY STOP (lost connection)" } else { Bad $r.out }
    Say "Bring it back with:  .\demo.ps1 -CockpitOn" Yellow; exit
}
if ($CockpitOn) {
    $r = Remote $COCKPIT_U $COCKPIT "sudo systemctl start adown-node1"
    if ($r.ok) { Ok "cockpit started - link returns in ~5 s; if the robot was driving it resumes by itself after 1 s" } else { Bad $r.out }
    exit
}
if ($Restart) {
    Remote $ROBOT_U $ROBOT "sudo systemctl restart turtlebot3-bringup adown-node2 adown-dashboard" | Out-Null; Ok "robot services restarted"
    Remote $COCKPIT_U $COCKPIT "sudo systemctl restart adown-node1" | Out-Null; Ok "cockpit restarted"
    if (Wait-Dashboard) { Check-Lidar | Out-Null }; exit
}
if ($Shutdown) {
    Remote $COCKPIT_U $COCKPIT "sudo poweroff" | Out-Null; Ok "cockpit powering off"
    Remote $ROBOT_U $ROBOT "sudo poweroff" | Out-Null;     Ok "robot powering off"
    Say "Wait 20 s until the green LEDs stop, then switch off the OpenCR and unplug the battery." Yellow; exit
}

$script:changed = @()
Say "AdownDriver - starting everything" White
Start-Hotspot
if (Wait-Nodes) {
    if (Ensure-Keys) {
        if (-not $NoSync) { Sync-Code }
        if (Ensure-Services) { if (Wait-Dashboard) { Check-Lidar | Out-Null } }
    } else { Bad "SSH keys are not working - run .\demo.ps1 -Setup and type 12345678" }
}
Say "`nReady. Shortcuts:  .\demo.ps1 -CockpitOff | -CockpitOn | -Restart | -Shutdown" White
