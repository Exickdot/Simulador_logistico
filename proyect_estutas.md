# REGISTRO DE ESTADO Y CONTROL DEL PROYECTO (`proyect_estutas.md`)
**Proyecto:** Estación Logística de Selección, Clasificación y Enrutamiento Industrial  
**Plataforma Objetivo:** Arduino Uno R3 (ATmega328P) + Gemelos Digitales 2D/3D  
**Última Actualización:** 2026-10-29  
**Propósito:** Documento maestro de persistencia técnica para optimizar tokens, guiar el desarrollo y mantener alineación entre sesiones sin reprocesar archivos completos.

---

## 1. RESUMEN EJECUTIVO Y ARQUITECTURA GENERAL

El sistema automatiza una estación logística industrial con tres lazos de control cerrados simultáneos, arquitectura multitarea cooperativa no bloqueante (`millis()`) y una máquina de estados finitos (FSM):

```
                                  [CÁMARA / LDR A0 + FOCO LED PIN 6]
                                               │ (Lazo 3: SP=70%, Kp=2.5)
     [ENTRADA]                     ▼           ▼         [SERVO SG90 PIN 5]
   [Alimentador]  ═════════════════════════════════════════════► [3 TOLVAS]
   (Cajas 3D)           ▲                                          │ ├─ Retrabajo (45°)
                        │ (Lazo 1: Kp=10 PWM/cm)                   │ ├─ Conforme (90°)
             [SENSOR ULTRASONIDO HC-SR04]                          │ └─ Rechazo (135°)
             (Pines D8 Trig / D9 Echo)                             ▼
                                                         [Gavetas de Almacén]
```

### Los 3 Lazos de Control Cerrados:
1. **Lazo 1 (Tracción y Frenado Proporcional):**
   - Sensor: HC-SR04 (Pines 8 Trig, 9 Echo). Tag ISA: `ZT-101`.
   - Actuador: Motor DC + Driver L298N (Pin 3 PWM ENA, Pines 4/7 DIR). Tag ISA: `M-101`, `SC-101`.
   - Algoritmo: $d \ge 25\text{ cm} \to \text{PWM } 200$ (Nominal); $5\text{ cm} \le d < 25\text{ cm} \to \text{Rampa Proporcional } K_p = 10\text{ PWM/cm}$; $d < 5\text{ cm} \to \text{PWM } 0$ (Parada total anticolisión).
2. **Lazo 2 (Servocontrol de Enrutamiento Angular):**
   - Actuador: Servomotor SG90 (Pin 5 PWM). Tag ISA: `XV-102`, `ZY-102`.
   - Posiciones: $45^\circ$ (Retrabajo / Tolva 1), $90^\circ$ (Conforme / Tolva 2 central), $135^\circ$ (Rechazo / Tolva 3). Retorno temporizado a neutro en 2.0 segundos.
3. **Lazo 3 (Estabilización Fotométrica de Inspección):**
   - Sensor: Fotorresistencia LDR en divisor 10 kΩ (Pin A0). Tag ISA: `QT-103`.
   - Actuador: Transistor TIP120 NPN + Foco LED (Pin 6 PWM). Tag ISA: `QL-103`, `QY-103`.
   - Algoritmo: Proporcional $SP = 70\%$, $K_p = 2.5$. Mantiene iluminación constante compensando la luz ambiental externa.
4. **Seguridad / Interrupción Externa (ISR INT0):**
   - Pulsador de Parada de Emergencia tipo Seta (Pin 2, `FALLING`, `INPUT_PULLUP`). Tag ISA: `HS-100`, `I-100`.
   - Corte atómico de actuadores en $< 5\ \mu\text{s}$.

---

## 2. MAPA DE ASIGNACIÓN ESTRICTA DE HARDWARE (ARDUINO UNO R3)

| Pin | Función | Periférico Conectado | Tag ISA-5.1 |
| :--- | :--- | :--- | :--- |
| **D0 / D1** | UART RX/TX | Telemetría Serie / Bluetooth HC-05 (9600 bps) | `ST-100` |
| **D2** | INT0 | Pulsador Seta Parada de Emergencia (`FALLING`) | `HS-100` / `I-100` |
| **D3** | PWM (OC2B) | ENA Driver L298N (Velocidad Motor DC Faja) | `SC-101` |
| **D4 / D7** | GPIO Out | IN1 / IN2 Driver L298N (Sentido de giro faja) | `SC-101` |
| **D5** | PWM (OC0B) | Señal Servomotor SG90 (Compuerta deflectora) | `XV-102` / `ZY-102` |
| **D6** | PWM (OC0A) | Base Transistor NPN (Foco LED de inspección) | `QL-103` / `QY-103` |
| **D8 / D9** | GPIO / PCINT | Trigger (D8) y Echo (D9) Sensor HC-SR04 | `ZT-101` |
| **D10** | GPIO Out | Buzzer Activo de Alarma Acústica | `XA-100` |
| **D11** | GPIO Out | LED Verde Indicador de Marcha / RUN | `XL-101` |
| **D12** | GPIO Out | LED Amarillo Indicador de Espera / Inspección | `XL-102` |
| **D13** | GPIO Out | LED Rojo Indicador de Falla / Emergencia | `XL-103` |
| **A0** | ADC0 | Fotorresistencia LDR (Divisor 10 kΩ a GND) | `QT-103` |
| **A1** | ADC1 | Potenciómetro de calibración analógica de Setpoint | `HIC-103` |
| **A4 / A5** | I2C (SDA/SCL)| Pantalla LCD 16x2 I2C (Dirección `0x27`) | `HMI-100` |

---

## 3. ESTADO DE LOS COMPONENTES DEL PROYECTO

### 3.1 Firmware Embebido C++ (`EstacionLogistica/EstacionLogistica.ino`)
- **Estado:** ✅ 100% Completo y Verificado.
- **Detalle:** Tareas cooperativas con `millis()`, sin funciones `delay()`, ISR atómica con variables `volatile`, parser de comandos seriales JSON/texto (`START`, `STOP`, `RESET`, `CLASSIFY`, `SIM_DIST`, `SIM_LDR`). No requiere cámara obligatoria para clasificar (clasifica mediante reflectancia fotométrica LDR/LED o por puerto UART).

### 3.2 Emulador de Física Continua (`emulator/plant_emulator.py`)
- **Estado:** ✅ 100% Funcional.
- **Detalle:** Modela dinámica de motor de 1er orden ($\tau = 120\text{ ms}$), ruido acústico gaussiano en HC-SR04, respuesta de LDR ($\tau = 60\text{ ms}$) y soporte para conexión física Hardware-In-The-Loop (`--hil COMx`).
- **Pruebas Unitarias:** `emulator/test_emulator.py` (`100% OK`).

### 3.3 Suite SCADA Digital Twin Multi-Pestaña (`simulador_digital_twin.py`)
- **Estado:** ✅ 100% Funcional (4 Pestañas Integradas). Lanzador: `iniciar_simulador.bat`.
- **Detalle:** Construido en Tkinter con diseño industrial claro (`#ECEFF1` / `#FAFAFA`).
  - **Pestaña 1:** Mímico SCADA animado 2D de la faja, HMI local (LCD 16x2 + LEDs), panel de mando y gráficas Matplotlib en vivo de Lazos 1 y 3.
  - **Pestaña 2:** Plano esquemático P&ID formal bajo norma ANSI / ISA-5.1.
  - **Pestaña 3:** Mapa de arquitectura de hardware, componentes y tabla de pines.
  - **Pestaña 4 (Opción A):** Gemelo Digital 3D CAD acelerado por GPU (`pyopengltk` + `PyOpenGL`). Carga el ensamble `modelsd/faja_transportadora_ensamble.obj` en Display Lists, cámara orbital interactiva con presets técnicos, banda móvil, cajas dinámicas, haz sonar HC-SR04, foco LED dinámico y panel de control/telemetría sincronizado en vivo.

### 3.4 Gemelo Digital 3D Nativo Python (`simulador_3d_desktop.py`)
- **Estado:** ✅ 100% Funcional y Optimizado con Ensamble CAD GrabCAD e Integración HIL Wokwi. Lanzador: `iniciar_simulador_3d.bat`.
- **Detalle:** Construido en Pygame-ce + PyOpenGL. Renderizado 3D de alta fidelidad acelerado en GPU mediante Display Lists (`faja_modular_estacion_3d.obj`). Flujo unidireccional físico estricto: Entrada por gravedad -> Faja CAD -> Zona de Inspección Fotométrica (Lazo 3) -> Sensor Sonar HC-SR04 con incidencia normal a 90° -> Caída libre por rampa -> Carro móvil indexador M-102 con 3 gavetas (Retrabajo 45°, Conforme 90°, Rechazo 135°). Consola retro industrial CRT inferior de 195 px unificada con botón y atajo de teclado (`W`) para enlace Hardware-In-The-Loop con Wokwi (puerto 4000). Telemetría en vivo sincronizada con los ciclos del ATmega328P.

### 3.5 Gemelo Digital 3D Nativo C++ (`cpp_digital_twin/`)
- **Estado:** ✅ 100% Compilado y Operativo. Lanzador: `iniciar_simulador_cpp.bat`.
- **Detalle:** C++20 con OpenGL puro y GLFW. Código en `cpp_digital_twin/src/`. Toolchain `w64devkit` mantenido localmente y excluido de Git para no saturar repositorios.

### 3.6 Catálogo y Modelos CAD 3D (`modelsd/`)
- **Estado:** ✅ Piezas modeladas en `.obj` y `.stl`.
- **Detalle:** Perfiles T-Slot 20x20 mm, rodillos con rodamientos 608ZZ, patas laterales y extendidas, soporte de sensor, sensor HC-SR04, motor NEMA 17, banda transportadora y rampa de descarte. Visor de inspección individual: `modelsd/ver_faja_3d.bat`.

### 3.7 Simulación Circuital e Integración HIL Wokwi (`diagram.json`, `wokwi.toml`, `WOKWI_GUIA.md`)
- **Estado:** ✅ 100% Configurado, Verificado y Calibrado. Guía paso a paso: `WOKWI_GUIA.md`.
- **Detalle:** Circuito esquemático completo de Arduino Uno R3 en Wokwi con puerto RFC2217 (`localhost:4000`). Cliente HIL bidireccional integrado tanto en `simulador_3d_desktop.py` (visor 3D nativo preferido) como en `simulador_digital_twin.py` (suite SCADA). 
- **Protocolo de Comunicación:** Trama atómica combinada `SIM:<dist>,<lux>\n` espaciada a 10 Hz (100 ms) con control de concurrencia (`threading.Lock` y `.flush()`), eliminando desbordes del búfer UART de 64 bytes del ATmega328P. El firmware responde silenciosamente en modo simulación para mantener holgura total en 9600 baudios y garantizar lecturas fotométricas exactas (Conforme >= 65%, Retrabajo >= 45%, Rechazo < 45%). Conmutación en caliente segura hacia emulación interna Python si Wokwi no está en ejecución.
- **Herramientas de Compilación:** `compilar_arduino.bat` con detección automática de `arduino-cli` y generación de binarios en `EstacionLogistica/build/`. Tag git de respaldo: `002_diseño_estable`.

---

## 4. REGLAS TÉCNICAS Y RESTRICCIONES CRÍTICAS DEL USUARIO

1. **Sin Tecnologías Web para Simuladores de Escritorio:**
   - La simulación principal debe ejecutarse en ventana nativa de escritorio (Python/Tkinter/OpenGL o C++), no en navegadores. Wokwi actúa como simulador de laboratorio circuital / HIL.
2. **Cero Saturación de Almacenamiento / Protocolos Limpios:**
   - Dependencias instaladas estrictamente dentro del entorno virtual local [`.venv`](file:///c:/PY/Auto_tif/.venv).
   - `.gitignore` configurado estrictamente para ignorar `.venv/`, `**/w64devkit/` (740 MB), `*.exe` compilados y cachés `__pycache__/`.
3. **Estética Industrial Técnica:**
   - Fondos claros/blancos de ingeniería (`#ECEFF1`, `#FAFAFA`, `#FFFFFF`), líneas oscuras técnicas (`#111111`), sin temas oscuros "gaming" ni luces neón.
4. **Cámara NO Obligatoria:**
   - El Arduino Uno (2 KB RAM) no procesa visión. La estación opera de forma 100% autónoma con el par LDR (A0) + Foco LED (Pin 6) por reflectancia. La "cámara" en diagramas representa el puesto de inspección cualitativa (`QE-103`) ampliable por UART.

---

## 5. TAREAS ACTIVAS Y PRÓXIMOS PASOS

- [x] **Integración del Modelo 3D de `modelsd/` en `iniciar_simulador.bat` (Opción A):**
  - Implementada con éxito la **Pestaña 4 (`[4] GEMELO DIGITAL 3D (CAD / GPU)`)** en `simulador_digital_twin.py` con `pyopengltk`.
  - Display Lists compiladas en VRAM de la GPU, rotación orbital 360°, paneo, zoom, 5 presets de cámara, renderizado Color/Clay/Alámbrico, banda y rodillos animados, cajas dinámicas 3D, haz ultrasónico y panel lateral de telemetría y mandos en vivo.
- [x] **Estandarización e Incorporación de Wokwi (`diagram.json`, `wokwi.toml`):**
  - Creado el esquemático de simulación con resolución de inconsistencias de hardware (agregado LED IN2 en D7 para monitoreo completo del L298N).
  - Verificada la compatibilidad total de los ángulos de clasificación del Servomotor SG90 con la cinemática de tolvas del Gemelo Digital.
  - Documentada la guía completa de pruebas para capturas de informe en `WOKWI_GUIA.md`.
- [ ] **Validación Matemática en MATLAB (Cuando el usuario lo requiera):**
  - Script `.m` para funciones de transferencia de Lazos 1 y 3, respuesta al escalón, márgenes de fase y estabilidad.
