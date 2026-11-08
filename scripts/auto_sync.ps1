# auto_sync.ps1 - Background Auto-Sync Daemon for GitHub Remote Agent & Local Unity Workflow
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$Host.UI.RawUI.WindowTitle = "F1 Sim - GitHub Auto-Sync Daemon"

Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host "  F1 SIMULATION RACING - GITHUB AUTO-SYNC DAEMON (REMOTE AGENT)" -ForegroundColor Yellow
Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host "[INFO] Dang theo doi remote branch origin/main..." -ForegroundColor Gray
Write-Host "[INFO] Chu ky kiem tra: moi 5 giay. Nhan Ctrl+C de dung." -ForegroundColor Gray
Write-Host ""

$intervalSeconds = 5

while ($true) {
    try {
        git fetch origin main --quiet 2>$null
        
        $localHash = (git rev-parse HEAD 2>$null).Trim()
        $remoteHash = (git rev-parse origin/main 2>$null).Trim()

        if ($localHash -and $remoteHash -and ($localHash -ne $remoteHash)) {
            $baseHash = (git merge-base HEAD origin/main 2>$null).Trim()
            
            if ($baseHash -eq $localHash) {
                # Local is strictly behind remote (can fast-forward)
                $timeStr = (Get-Date).ToString("HH:mm:ss")
                Write-Host "[$timeStr] [AUTO-SYNC] Phat hien commit moi tu Remote Agent ($remoteHash)!" -ForegroundColor Green
                Write-Host "[$timeStr] [AUTO-SYNC] Dang cap nhat code ve may local..." -ForegroundColor Yellow
                
                # Check if local working tree is dirty
                $dirty = (git status --porcelain 2>$null)
                if ($dirty) {
                    Write-Host "[$timeStr] [CANH BAO] May local co file thay doi chua commit, dang stash tam thoi..." -ForegroundColor DarkYellow
                    git stash --quiet
                    git pull --ff-only origin main
                    git stash pop --quiet
                } else {
                    git pull --ff-only origin main
                }

                $newLocal = (git rev-parse --short HEAD 2>$null).Trim()
                $commitMsg = (git log -1 --pretty=format:"%s" 2>$null)
                
                Write-Host "[$timeStr] [THANH CONG] Da keo commit moi nhat: [$newLocal] $commitMsg" -ForegroundColor Cyan
                Write-Host "[$timeStr] [THANH CONG] Unity se tu dong reload/recompile de ban test ngay!" -ForegroundColor Green
                
                # Audio chime notification
                try {
                    [Console]::Beep(1000, 250)
                    Start-Sleep -Milliseconds 50
                    [Console]::Beep(1400, 350)
                } catch {}
            } elseif ($baseHash -eq $remoteHash) {
                # Local is ahead of remote - nothing to pull
            } else {
                $timeStr = (Get-Date).ToString("HH:mm:ss")
                Write-Host "[$timeStr] [CANH BAO] Local va Remote da bi phan nhanh (diverged). Khong the fast-forward!" -ForegroundColor Red
            }
        }
    } catch {
        # Silent ignore temporary network glitches
    }

    Start-Sleep -Seconds $intervalSeconds
}
