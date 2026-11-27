@echo off
setlocal
cd /d "%~dp0.."

echo ==========================================
echo MaixCAM D/E direct deployment
echo Host: 100.118.92.10
echo STM32 will remain running.
echo ==========================================

python tools\deploy_j_silent.py --host 100.118.92.10
set "RESULT=%ERRORLEVEL%"

if "%RESULT%"=="0" (
  echo.
  echo Deployment succeeded.
) else (
  echo.
  echo Deployment failed. Check logs above.
)
echo.
pause
exit /b %RESULT%
