@echo off
title Estación Logística Industrial - Simulador Digital Twin
echo ======================================================================
echo   INICIANDO SIMULADOR DIGITAL TWIN (ENTORNO VIRTUAL .venv)
echo ======================================================================
echo.

if not exist ".venv\Scripts\python.exe" (
    echo [ERROR] No se encontro el entorno virtual .venv.
    echo Creando entorno virtual e instalando librerias...
    python -m venv .venv
    .venv\Scripts\python.exe -m pip install -r requirements.txt
)

echo Ejecutando aplicacion SCADA...
.venv\Scripts\python.exe simulador_digital_twin.py

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [AVISO] La aplicacion finalizo con codigo %ERRORLEVEL%.
    pause
)
