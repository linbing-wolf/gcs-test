@echo off
setlocal
call "%~dp0vision-maixcam\tools\DEPLOY_MAIXCAM_DISC.cmd"
exit /b %ERRORLEVEL%
