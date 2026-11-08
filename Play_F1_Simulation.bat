@echo off
title Formula 1 Grand Prix Simulation (Unity 6 URP)
cd /d "c:\Users\nguye\PROJECT\F1"

if "%1"=="--python" (
    shift
    python "c:\Users\nguye\PROJECT\F1\main.py" %*
    exit /b %ERRORLEVEL%
)

if exist "c:\Users\nguye\PROJECT\F1\Build\F1_GrandPrix.exe" (
    start "" "c:\Users\nguye\PROJECT\F1\Build\F1_GrandPrix.exe" -screen-width 1600 -screen-height 900 -screen-fullscreen 0
    exit /b 0
)

if exist "C:\Program Files\Python312\python.exe" (
    "C:\Program Files\Python312\python.exe" "c:\Users\nguye\PROJECT\F1\main.py" %*
) else (
    python "c:\Users\nguye\PROJECT\F1\main.py" %*
)
