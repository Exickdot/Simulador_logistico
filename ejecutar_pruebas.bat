@echo off
title Pruebas Unitarias - Lazos de Control y Firmware
echo ======================================================================
echo   EJECUTANDO SUITE DE PRUEBAS UNITARIAS EN .venv
echo ======================================================================
echo.

.venv\Scripts\python.exe emulator\test_emulator.py
echo.
pause
