@echo off
title Visor 3D - Faja Transportadora Ajustable de Laboratorio (GrabCAD Modificado)
cd /d "%~dp0"

echo =====================================================================
echo  VISOR 3D FAJA TRANSPORTADORA AJUSTABLE CON PORTICO DE PASO LIBRE
echo  Modelo CAD modular con sensor HC-SR04 y aceleracion GPU OpenGL
echo =====================================================================
echo.
echo Iniciando entorno virtual de Python...

if exist "..\.venv\Scripts\python.exe" (
    "..\.venv\Scripts\python.exe" ver_faja_3d.py
) else (
    python ver_faja_3d.py
)

if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Ocurrio un problema al ejecutar el visor 3D.
    pause
)
