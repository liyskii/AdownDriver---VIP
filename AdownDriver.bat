@echo off
rem AdownDriver - double-click to start the whole demo (hotspot, robot, cockpit, dashboard)
title AdownDriver
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0demo.ps1" %*
echo.
pause
