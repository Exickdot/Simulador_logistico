@echo off
setlocal
echo ========================================================
echo   COMPILANDO FIRMWARE ARDUINO UNO (ESTACION LOGISTICA)
echo ========================================================

set ARDUINO_CLI="C:\Users\alexn\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"

if not exist %ARDUINO_CLI% (
    echo [ERROR] No se encontro arduino-cli en la ruta esperada.
    if /i not "%1"=="--no-pause" pause
    exit /b 1
)

%ARDUINO_CLI% compile --fqbn arduino:avr:uno "EstacionLogistica" --output-dir "EstacionLogistica\build"

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ========================================================
    echo   [OK] Compilacion exitosa. Binarios ELF/HEX listos.
    echo   Wokwi Simulator cargara los nuevos binarios en VS Code.
    echo ========================================================
) else (
    echo.
    echo   [ERROR] Fallo la compilacion. Revisa los mensajes arriba.
)

if /i not "%1"=="--no-pause" pause

