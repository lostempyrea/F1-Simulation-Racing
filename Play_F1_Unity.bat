@echo off
title Formula 1 Grand Prix Simulation (Unity 6 URP)
echo ========================================================
echo   FORMULA 1 GRAND PRIX SIMULATION - UNITY 6 URP
echo ========================================================

set GAME_EXE=c:\Users\nguye\PROJECT\F1\Build\F1_GrandPrix.exe
set UNITY_EXE=E:\AppsTools\Unity\Hub\Editor\6000.3.21f1\Editor\Unity.exe

if "%1"=="--editor" (
    echo Dang mo du an UnityF1 trong Unity Editor 6000.3.21f1...
    start "" "%UNITY_EXE%" -projectPath "c:\Users\nguye\PROJECT\F1\UnityF1"
    exit /b 0
)

if exist "%GAME_EXE%" (
    echo Dang khoi chay Formula 1 Grand Prix Simulation (Unity 6 URP)...
    start "" "%GAME_EXE%" -screen-width 1600 -screen-height 900 -screen-fullscreen 0
    echo [OK] Game da duoc khoi chay thanh cong!
) else if exist "%UNITY_EXE%" (
    echo Dang mo du an UnityF1 trong Unity Editor 6000.3.21f1...
    start "" "%UNITY_EXE%" -projectPath "c:\Users\nguye\PROJECT\F1\UnityF1"
    echo [OK] Unity Editor da duoc khoi chay!
) else (
    echo [ERROR] Khong tim thay F1_GrandPrix.exe hoac Unity.exe
    pause
)
