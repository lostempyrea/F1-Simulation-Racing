@echo off
title Formula 1 Grand Prix Simulation (Unity 6 URP)
cd /d "c:\Users\nguye\PROJECT\F1"

echo ========================================================
echo   FORMULA 1 GRAND PRIX SIMULATION - UNITY 6 URP
echo ========================================================
echo [NOTICE] Python gameplay runner has been retired.
echo Launching 100%% Native Unity 6 Standalone Game...
echo ========================================================

if exist "c:\Users\nguye\PROJECT\F1\Build\F1_GrandPrix.exe" (
    start "" "c:\Users\nguye\PROJECT\F1\Build\F1_GrandPrix.exe" -screen-width 1600 -screen-height 900 -screen-fullscreen 0
    exit /b 0
) else (
    echo [ERROR] Build\F1_GrandPrix.exe not found!
    echo Please launch Unity Editor or run Play_F1_Unity.bat.
    pause
    exit /b 1
)
