@echo off
setlocal
cd /d "%~dp0"
title Compilador C++ Nativo - Gemelo Digital Industrial 3D

echo ======================================================================
echo   COMPILANDO GEMELO DIGITAL INDUSTRIAL EN C++ NATIVO (OPENGL / WIN32)
echo ======================================================================
echo.

set GXX="%~dp0w64devkit\bin\g++.exe"

if not exist %GXX% (
    echo [ERROR] No se encontro el compilador en %GXX%
    pause
    exit /b 1
)

echo Compilando codigo fuente C++ optimizado (-O2 -Wall -Wextra)...
%GXX% -O2 -Wall -Wextra -std=c++17 -mwindows src\main.cpp -lopengl32 -lglu32 -lgdi32 -luser32 -o simulador_3d_cpp.exe

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ======================================================================
    echo   [EXITO] Compilacion completada satisfactoriamente.
    echo   Archivo generado: simulador_3d_cpp.exe
    echo ======================================================================
    dir simulador_3d_cpp.exe
) else (
    echo.
    echo [ERROR] Hubo un error durante la compilacion de C++.
    pause
    exit /b %ERRORLEVEL%
)

endlocal
