@echo off
cd /d "%~dp0.."
echo Activando entorno virtual (.venv)...
call .venv\Scripts\activate.bat
cmd /k

