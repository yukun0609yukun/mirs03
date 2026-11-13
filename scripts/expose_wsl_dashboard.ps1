# Expose the WSL2-hosted ROS 2 dashboards (port 8080 = food-count input,
# port 8090 = operator dashboard) to other devices on the same Wi-Fi/LAN.
#
# WSL2's internal IP address changes every time WSL2 restarts, so re-run
# this script (as Administrator) after every WSL2 restart.
#
# Usage (in an elevated/Administrator PowerShell):
#   powershell -ExecutionPolicy Bypass -File scripts\expose_wsl_dashboard.ps1

$ErrorActionPreference = "Stop"

$wslIp = (wsl hostname -I).Trim().Split(" ")[0]
if (-not $wslIp) {
    Write-Error "Could not get the WSL2 IP address. Make sure WSL2 (Ubuntu-22.04) is running."
    exit 1
}
Write-Host "WSL2 IP: $wslIp"

$ports = @(8080, 8090)

foreach ($port in $ports) {
    netsh interface portproxy delete v4tov4 listenport=$port listenaddress=0.0.0.0 2>$null | Out-Null
    netsh interface portproxy add v4tov4 listenport=$port listenaddress=0.0.0.0 connectport=$port connectaddress=$wslIp
    Write-Host "port $port -> $wslIp forwarded"

    $ruleName = "Nokosahen Dashboard $port"
    if (-not (Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue)) {
        New-NetFirewallRule -DisplayName $ruleName -Direction Inbound -LocalPort $port -Protocol TCP -Action Allow | Out-Null
        Write-Host "Firewall rule '$ruleName' added"
    }
}

$lanIp = (Get-NetIPAddress -AddressFamily IPv4 |
    Where-Object { $_.InterfaceAlias -notmatch 'Loopback|WSL|vEthernet' -and $_.IPAddress -notmatch '^169\.254\.' } |
    Select-Object -First 1 -ExpandProperty IPAddress)

Write-Host ""
Write-Host "Other devices on the same Wi-Fi/LAN can now open:"
Write-Host "  Food count input:      http://${lanIp}:8080"
Write-Host "  Operator dashboard:    http://${lanIp}:8090"
