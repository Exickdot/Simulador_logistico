@echo off
title Pruebas Unitarias - Lazos de Control y Firmware
cd /d "%~dp0.."
echo ======================================================================
echo   EJECUTANDO SUITE DE PRUEBAS UNITARIAS EN .venv
echo ======================================================================
echo.

.venv\Scripts\python.exe emulator\test_emulator.py
echo.
pause

