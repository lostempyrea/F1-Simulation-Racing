@echo off
title F1 Simulation Racing - GitHub Auto-Sync Daemon
cd /d "%~dp0"

echo =================================================================
echo   F1 SIMULATION RACING - GITHUB AUTO-SYNC DAEMON
echo =================================================================
echo   Dong bo tu dong commit tu Remote Agent tren GitHub ve may local
echo   Nhan Ctrl+C bat cu luc nao de dung.
echo =================================================================
echo.

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\auto_sync.ps1"

pause
