@echo off
cd /d "%~dp0"
where py.exe >nul 2>&1
if errorlevel 1 (
    python.exe -B -X utf8 server.py --open
) else (
    py.exe -3 -B -X utf8 server.py --open
)
if errorlevel 1 pause
