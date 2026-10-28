@echo off
setlocal
cd /d "%~dp0"
title Visor CAD 3D - Sensor Ultrasonico HC-SR04
echo ======================================================================
echo   VISOR CAD 3D - SENSOR ULTRASONICO HC-SR04 (GrabCAD High-Fidelity)
echo ======================================================================
echo.

if not exist "..\.venv\Scripts\python.exe" (
    echo [ERROR] No se encontro el entorno virtual en ..\.venv
    pause
    exit /b 1
)

echo Iniciando visor 3D en tiempo real (OpenGL / Pygame-CE)...
echo Controles:
echo   - Clic Izquierdo + Arrastre: Rotar
echo   - Clic Derecho + Arrastre:   Mover (Pan)
echo   - Rueda de Raton:            Zoom
echo   - Tecla 'W':                 Alternar Modo Alambrico (Wireframe)
echo   - Tecla '1','2','3','4':     Vistas Rapidas (Isometria GrabCAD)
echo.

..\.venv\Scripts\python.exe ver_sensor_3d.py %*
endlocal
