@echo off
title Estación Logística Industrial - Gemelo Digital 3D Nativo (OpenGL)
echo ======================================================================
echo   INICIANDO MOTOR GRAFICO 3D NATIVO (OPENGL / PYGAME-CE)
echo   Entorno Virtual: .venv (Sin dependencias externas ni web)
echo ======================================================================
echo.

if not exist ".venv\Scripts\python.exe" (
    echo [ERROR] No se encontro el entorno virtual .venv.
    echo Creando entorno virtual e instalando librerias...
    python -m venv .venv
    .venv\Scripts\python.exe -m pip install -r requirements.txt
)

echo Ejecutando aplicacion grafica 3D de alta precision...
.venv\Scripts\python.exe simulador_3d_desktop.py %*

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [AVISO] La aplicacion finalizo con codigo %ERRORLEVEL%.
    pause
)
