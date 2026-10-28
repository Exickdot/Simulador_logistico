@echo off
setlocal
cd /d "%~dp0cpp_digital_twin"
title Estación Logística - Gemelo Digital 3D Nativo C++

if not exist "simulador_3d_cpp.exe" (
    echo [INFO] El ejecutable no existe. Compilando primero...
    call compilar.bat
)

echo Iniciando Gemelo Digital Industrial en C++ Nativo (485 KB)...
start "" "simulador_3d_cpp.exe"
endlocal
