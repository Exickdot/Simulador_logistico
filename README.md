# Estación Logística de Selección, Clasificación y Enrutamiento (Auto-TIF)

Sistema integral de automatización industrial basado en **Arduino Uno R3 (ATmega328P)** con arquitectura multitarea cooperativa no bloqueante, Máquina de Estados Finitos (FSM), 3 lazos de control cerrados continuos e interrupción externa de seguridad (ISR INT0). 

Incluye **Gemelo Digital 3D fotorrealista dual (C++ nativo de 496 KB y Python PyOpenGL)** con **Consola Retro Industrial de Supervisión**, optimización de rendimiento GPU VRAM y un **subsistema completo de modelado CAD 3D paramétrico** (`modelsd/`) con despiece de piezas y ensamble acoplado (OBJ / STL binario).

---

## 📁 Estructura Integral del Proyecto

```tree
c:\PY\Auto_tif\
├── EstacionLogistica\
│   └── EstacionLogistica.ino          # Firmware maestro C++/Arduino (ATmega328P)
├── cpp_digital_twin\
│   ├── src\
│   │   └── main.cpp                   # Gemelo Digital 3D en C++ nativo (Win32 + OpenGL)
│   ├── bin\
│   │   └── simulador_3d_cpp.exe       # Ejecutable standalone ultraligero de 496 KB (120+ FPS)
│   ├── compilar.bat                   # Script de compilación automática con MinGW-w64
│   └── w64devkit\                     # Toolchain portátil GCC/G++ incluido
├── modelsd\                           # Subsistema de Modelado CAD 3D de Componentes
│   ├── perfil_aluminio_2020.obj/.stl  # Perfil extruido de aluminio T-Slot 20x20 mm (360 mm)
│   ├── rodillo_conveyor.obj/.stl      # Rodillo Ø 26 mm con eje de acero y rodamientos 608ZZ
│   ├── pata_soporte_lateral.obj/.stl  # Pata de soporte estándar trapezoidal negra
│   ├── pata_soporte_extendida.obj/.stl# Pata lateral con brazo de extensión longitudinal (72 mm)
│   ├── torre_roja_ajustable.obj/.stl  # Torre vertical regulable con corredera para sensores
│   ├── placa_soporte_sensor.obj/.stl  # Placa frontal ranurada GrabCAD (108x67 mm)
│   ├── sensor_hcsr04_compacto.obj/.stl# Módulo ultrasónico HC-SR04 de alta precisión
│   ├── banda_transportadora.obj/.stl  # Banda continua plana de caucho azul-gris (70 mm)
│   ├── motor_nema17_drive.obj/.stl    # Motor NEMA 17 con acople flexible y escuadra
│   ├── rampa_descarga.obj/.stl        # Rampa de evacuación de cajas con labios laterales
│   ├── faja_transportadora_ensamble.obj/.stl # Ensamble Maestro Acoplado Integral (2,722 v / 4,956 f)
│   ├── HC-SR04-UltrasonicSensor.obj/.stl     # Modelo CAD 1:1 independiente del sensor HC-SR04
│   ├── piezas_faja.mtl                # Librería maestra de materiales PBR (aluminio, caucho, PLA)
│   ├── model_builder.py               # Motor matemático de geometría analítica y exportador OBJ/STL
│   ├── generate_parts.py              # Generador paramétrico de piezas individuales y ensamble
│   ├── ver_faja_3d.py                 # Visor 3D interactivo con selector de piezas (0..9, TAB)
│   ├── ver_faja_3d.bat                # Lanzador directo con doble clic del visor 3D de la faja
│   └── ver_sensor_3d.py/.bat          # Visor 3D dedicado para el sensor HC-SR04
├── emulator\
│   ├── plant_emulator.py              # Emulador de física de planta y modo HIL
│   ├── test_emulator.py               # Suite de pruebas unitarias de lazos de control
│   └── requirements.txt               # Dependencias Python (Pygame, PyOpenGL, PySerial)
├── simulador_3d_desktop.py            # Gemelo Digital 3D interactivo en Python (Pygame + OpenGL)
├── simulador_digital_twin.py          # Simulador SCADA 2D en Python (Canvas 60 FPS)
├── simulador_web.html                 # Simulador SCADA Web en HTML5 Canvas (sin dependencias)
├── iniciar_simulador_cpp.bat          # Lanzador del Gemelo Digital 3D nativo C++
├── iniciar_simulador_3d.bat           # Lanzador del Gemelo Digital 3D en Python
├── iniciar_simulador.bat              # Lanzador del Simulador 2D SCADA
├── MEMORIA_TECNICA.md                 # Documentación técnica formal, ecuaciones y tabla de FSM
└── README.md                          # Guía de inicio rápido, especificaciones y manual de uso
```

---

## ⚡ 1. Asignación de Pines de Hardware (Arduino Uno R3)

| Pin | Función | Descripción Técnica y Circuital |
| :--- | :--- | :--- |
| **D0 / D1** | UART RX/TX | Telemetría Serie y Bluetooth HC-05 (9600 baudios, tramas CSV/JSON) |
| **D2** | INT0 | Pulsador de Parada de Emergencia (`FALLING` con Pull-Up interno) |
| **D3** | PWM (OC2B) | ENA Driver L298N (Lazo 1: Velocidad faja transportadora) |
| **D4 / D7** | GPIO | IN1 / IN2 Driver L298N (Control de sentido y frenado motor DC) |
| **D5** | PWM / PPM | Servomotor SG90 (Lazo 2: Enrutamiento angular a tolvas / gavetas) |
| **D6** | PWM (OC0A) | Base Transistor NPN TIP122 (Lazo 3: Iluminación LED regulada) |
| **D8 / D9** | GPIO / PCINT | Trigger (Pulso $10\,\mu\text{s}$) y Echo Sensor Ultrasónico HC-SR04 |
| **D10** | GPIO | Buzzer activo de alarma acústica de falla |
| **D11** | GPIO | LED Verde (Estado: Marcha / En Proceso) |
| **D12** | GPIO | LED Amarillo (Estado: Inspección / Espera) |
| **D13** | GPIO | LED Rojo (Estado: Alarma / Parada de Emergencia) |
| **A0** | ADC0 | Fotorresistencia LDR (Divisor de tensión $10\text{ k}\Omega$ a GND) |
| **A1** | ADC1 | Potenciómetro de calibración analógica de Setpoint de luminosidad |
| **A4 / A5** | I2C (SDA/SCL)| Pantalla LCD 16x2 I2C (Dirección `0x27`, driver PCF8574T) |

---

## 🎛️ 2. Consola Retro Industrial de Supervisión y Control (SCADA HMI)

Tanto en el ejecutable C++ como en Python, se unificó la supervisión en una **consola retro industrial de 195 px** en la franja inferior:

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                        ZONA 3D ABIERTA: CAMARA ORBITAL / ESTACION                      │
├────────────────────────────────────────────────────────────────────────────────────────┤
│ [ CONSOLA DE SUPERVISION INDUSTRIAL // AUTO-TIF v2.4 ]         ATmega328P • FPS: 60.0  │
├──────────────────────────┬────────────────────────────┬────────────────────────────────┤
│ CRT-1: TELEMETRIA FSM    │ CRT-2: MATRIZ DIGITAL OEE  │ TABLERO DE ACCIONAMIENTO       │
│ >> Fósforo Verde <<      │ >> Lámparas & Producción <<│ >> Botones Táctiles Biselados <<│
│ • [FSM]: TRANSP          │ • TOTAL:    128 unidades   │ [ INICIAR ]  [ DETENER ] [ESTOP]
│ • ZT-101: 24.5 cm        │ • CONFORME: 110 ( 86.0%)   │ [+ CAJA ]    [ RESET ]   [LUZ] │
│ • PWM Motor: 82%         │ • RETRABAJO: 12 (  9.4%)   │                                │
│ • PID LED:   65%         │ • RECHAZO:    6 (  4.6%)   │ SELECTOR DE CAMARAS:           │
│ • Servo Compuerta: 90°   │ • L1..L4: Hardware Status  │ [V1:ISO] [V2:CAM] [V3..V5]     │
└──────────────────────────┴────────────────────────────┴────────────────────────────────┘
```

1. **Módulo CRT-1 (Fósforo Verde):** Emulación de pantalla analógica con barrido *scanlines*, reportando estado de la FSM (`REPOSO`, `TRANSP`, `INSPEC`, `DESVIO`, `EMERG!`), telemetría en tiempo real del sonar ultrasónico y lazos de control de actuadores.
2. **Módulo CRT-2 (Matriz LED OEE y Diagnóstico):** Matriz 2x2 para contabilizar unidades conformes, retrabajos y rechazos, junto con 4 lámparas LED (`L1: SISTEMA OK`, `L2: MOTOR RUN`, `L3: SONAR ACTIVE`, `L4: CAM/LED PID`).
3. **Módulo de Mando:** Botones con bisel 3D para arranque/parada, inyección de perturbaciones y seta de emergencia (`! E-STOP !`) con marco de seguridad a rayas amarillas y negras.

---

## ⚡ 3. Optimización de Rendimiento Gráfico GPU (PyOpenGL)

* **Diagnóstico de lentitud previa:** La función `_draw_text` realizaba *Texture Thrashing*, asignando y destruyendo más de **2,100 texturas de texto por segundo** a 60 FPS, bloqueando el bus PCIe y el driver OpenGL.
* **Solución VRAM Cache:** Implementación de `TextTextureCache` en [`simulador_3d_desktop.py`](file:///c:/PY/Auto_tif/simulador_3d_desktop.py). Las texturas fijas se crean 1 sola vez y los valores dinámicos se reutilizan, reduciendo la carga del driver a cero.
* **Display Lists de Hardware:** En los visores y modelos 3D ([`ver_faja_3d.py`](file:///c:/PY/Auto_tif/modelsd/ver_faja_3d.py)), se compila la geometría con `glGenLists` y `glCallList`, logrando **60 a 120+ FPS constantes** en Python.

---

## 📐 4. Subsistema de Modelado CAD 3D de Componentes (`modelsd/`)

Diseñado a escala métrica 1:1 en milímetros según referencias normalizadas de **GrabCAD**:

### A. Ingeniería de la Faja Transportadora (GrabCAD: Gabriele Gregorini)
* **Principio Físico Acústico (HC-SR04 a 40 kHz):** Las ondas ultrasónicas sufren reflexión especular. Para asegurar un eco de retorno con máxima potencia en la cápsula receptora "R", el sensor **debe permanecer a la altura de la caja ($Y = 26.0\text{ mm}$), incidiendo a $90^\circ$ perpendicular sobre su cara frontal**.
* **Extensión Longitudinal de Evacuación:** Se extendieron dos brazos de soporte de $72\text{ mm}$ en las placas laterales hacia $X = -232.0\text{ mm}$, creando una **boca de caída libre de $58\text{ mm}$ de largo $\times 78\text{ mm}$ de ancho** con rampa inclinada a $35^\circ$. Las cajas son detectadas con precisión y caen libremente hacia la gaveta inferior sin rozar el sensor.

```
                    ◄───── 58 mm (Espacio Libre) ─────►
                 ┌───────────────────────┐
  [FAJA]         │ CAJA 1 (EN DETECCION) │            (T)  [ SENSOR HC-SR04 ]
═════════════════╡                       ╞═════════◄═══════ (Emisor 40 kHz)
Y = +14.6 mm     │ Cara frontal a 90°    │   Haz Acústico   Alineado a Y = 26 mm
                 └───────────────────────┘═════════►═══════ (R) (Receptor Eco)
                                   │
                            [Fin de Faja]
                                   │
                                   ▼
                       ┌───────────────────────┐
                       │ CAJA 2 (EVACUACION)   │
                       │ Cae libremente por la │  [PLACA ORIGINAL GrabCAD]
                       │ rampa hacia el acopio │  Montada en X = -232 mm
                       └───────────────────────┘  (Sobre brazos de extensión)
```

### B. Inventario de Componentes 3D Individuales (OBJ y STL Binario)

| # | Archivo 3D OBJ / STL | Componente / Descripción Técnica | Vértices / Caras |
| :-: | :--- | :--- | :-: |
| **1** | [perfil_aluminio_2020.obj/.stl](file:///c:/PY/Auto_tif/modelsd/perfil_aluminio_2020.obj) | Perfil T-Slot 20x20 mm (Longitud 360 mm, 4 ranuras T de 6 mm) | 8 v / 12 f |
| **2** | [rodillo_conveyor.obj/.stl](file:///c:/PY/Auto_tif/modelsd/rodillo_conveyor.obj) | Rodillo $\varnothing 26\text{ mm}$, coronas de guiado $\varnothing 30\text{ mm}$, eje $\varnothing 8\text{ mm}$, 2x 608ZZ | 448 v / 864 f |
| **3** | [pata_soporte_lateral.obj/.stl](file:///c:/PY/Auto_tif/modelsd/pata_soporte_lateral.obj) | Pata de apoyo estándar trapezoidal en acrílico/acero negro satinado (6 mm) | 84 v / 152 f |
| **4** | [pata_soporte_extendida.obj/.stl](file:///c:/PY/Auto_tif/modelsd/pata_soporte_extendida.obj) | Pata lateral con brazo de extensión longitudinal integrado de 72 mm para sensor | 160 v / 292 f |
| **5** | [torre_roja_ajustable.obj/.stl](file:///c:/PY/Auto_tif/modelsd/torre_roja_ajustable.obj) | Torre vertical regulable en PLA rojo con corredera colisa de 38 mm | 108 v / 188 f |
| **6** | [placa_soporte_sensor.obj/.stl](file:///c:/PY/Auto_tif/modelsd/placa_soporte_sensor.obj) | Placa frontal ranurada GrabCAD (108x67x4 mm) en aleación gris grafito | 348 v / 652 f |
| **7** | [sensor_hcsr04_compacto.obj/.stl](file:///c:/PY/Auto_tif/modelsd/sensor_hcsr04_compacto.obj) | Módulo ultrasónico HC-SR04 (PCB azul, transductores $\varnothing 16\text{ mm}$, cristal HC-49) | 320 v / 600 f |
| **8** | [banda_transportadora.obj/.stl](file:///c:/PY/Auto_tif/modelsd/banda_transportadora.obj) | Banda plana continua de caucho azul-grisáceo mate (ancho 70 mm) | 272 v / 408 f |
| **9** | [motor_nema17_drive.obj/.stl](file:///c:/PY/Auto_tif/modelsd/motor_nema17_drive.obj) | Motor NEMA 17 ($42\times 42\times 34\text{ mm}$) con acople elástico azul $\varnothing 19\text{ mm}$ y escuadra | 116 v / 208 f |
| **10** | [rampa_descarga.obj/.stl](file:///c:/PY/Auto_tif/modelsd/rampa_descarga.obj) | Rampa de evacuación inclinada a $35^\circ$ con labios laterales de guiado | 24 v / 36 f |
| **T** | [faja_transportadora_ensamble.obj/.stl](file:///c:/PY/Auto_tif/modelsd/faja_transportadora_ensamble.obj) | **Ensamble Maestro Acoplado Integral con todas las piezas en posición final** | **2,722 v / 4,956 f** |

---

## 🚀 5. Ejecución Rápida y Guía de Uso

### 5.1 Visor 3D Interactivo de la Faja y Catálogo de Piezas
Permite inspeccionar cada componente 3D de forma individual o verlos acoplados en el ensamble completo:
```bash
# Doble clic en:
modelsd\ver_faja_3d.bat
# o por consola:
.venv\Scripts\python.exe modelsd\ver_faja_3d.py
```
* **Controles de Cámara:** Arrastre con clic izquierdo (órbita 360°), clic derecho (paneo), rueda del ratón (zoom).
* **Selector de Piezas en Tiempo Real:**
  * `0`: Ensamble Completo Acoplado
  * `1`: Perfil de Aluminio T-Slot 20x20
  * `2`: Rodillo con Eje y Rodamientos 608ZZ
  * `3`: Pata de Soporte Estándar
  * `4`: Pata Lateral con Brazo para Sensor
  * `5`: Torre Roja Regulable con Corredera
  * `6`: Placa de Montaje del Sensor (GrabCAD)
  * `7`: Sensor Ultrasónico HC-SR04
  * `8`: Banda Transportadora Continua
  * `9`: Motor NEMA 17 con Acople y Escuadra
  * `TAB`: Ciclar secuencialmente entre todas las piezas
  * `C`: Alternar acabado PBR realista / monocromático *Clay CAD*
  * `W`: Alternar visualización alámbrica (*wireframe*)
  * `S`: Guardar captura PNG en alta resolución

### 5.2 Gemelo Digital 3D Nativo en C++ (Standalone de 496 KB)
```bash
iniciar_simulador_cpp.bat
# o directamente:
cpp_digital_twin\bin\simulador_3d_cpp.exe
```
* **Rendimiento:** 120–240+ FPS constantes, consumo de RAM < 18 MB, sin requerir Python ni dependencias externas.
* **Recompilación:** Si modifica el código fuente C++, ejecute `cpp_digital_twin\compilar.bat`.

### 5.3 Gemelo Digital 3D en Python Desktop
```bash
iniciar_simulador_3d.bat
# o directamente:
.venv\Scripts\python.exe simulador_3d_desktop.py
```
* Incluye física continua de cajas cayendo de la tolva, avance sobre la faja, lazo cerrado PID para iluminación de cámara y clasificación hacia el carro lanzadera.

### 5.4 Regeneración Paramétrica de Modelos 3D
Si desea modificar cotas milimétricas o generar nuevos modelos OBJ/STL:
```bash
.venv\Scripts\python.exe modelsd\generate_parts.py
```

---

## 📊 6. Documentación Adicional

* [`MEMORIA_TECNICA.md`](file:///c:/PY/Auto_tif/MEMORIA_TECNICA.md): Modelado matemático de los 3 lazos continuos, ecuaciones diferenciales de fricción y tablas de transición de la FSM.
* [`faja_transportadora_3d.md`](file:///c:/Users/alexn/.gemini/antigravity/brain/09a9911c-7590-417d-9edb-523386af58c7/faja_transportadora_3d.md): Documento interactivo con carruseles de renders fotorrealistas y comparativas visuales.
