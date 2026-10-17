@echo off
chcp 65001 >nul
title TJC screen text sender
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0tjc-from-file.ps1"
echo.
echo -------- sent. Now look at the screen. Press any key to close this window. --------
echo.
pause >nul
