# GUÍA DE OPERACIÓN E INTEGRACIÓN DE WOKWI (`WOKWI_GUIA.md`)
**Proyecto:** Estación Logística de Selección, Clasificación y Enrutamiento Industrial (Auto-TIF)  
**Microcontrolador:** Arduino Uno R3 (ATmega328P)  
**Entorno de Simulación:** Wokwi Simulator (Web & VS Code Extension)

---

## 1. Arquitectura de Archivos en el Repositorio

El proyecto incorpora Wokwi manteniendo compatibilidad total con el firmware nativo y los gemelos digitales:

```tree
c:\PY\Auto_tif\
├── EstacionLogistica\
│   ├── EstacionLogistica.ino   # Firmware maestro oficial C++
│   └── libraries.txt           # Declaración de librerías para Wokwi Web
├── diagram.json                # Circuito esquemático industrial en Wokwi
├── wokwi.toml                  # Descriptor de compilación para VS Code Wokwi
├── libraries.txt               # Librerías globales (LiquidCrystal I2C, Servo)
└── WOKWI_GUIA.md               # Este documento de referencia
```

---

## 2. Mapa de Conexiones y Equivalencias en Wokwi

El circuito en [`diagram.json`](file:///c:/PY/Auto_tif/diagram.json) replica los 3 lazos de control cerrados y el sistema de seguridad. Para solventar la ausencia del bloque físico del driver L298N en Wokwi, se utilizan LEDs de monitoreo de señales de control de potencia:

| Periférico Industrial | Pin Arduino | Componente en Wokwi | Indicador / Equivalencia |
| :--- | :--- | :--- | :--- |
| **Parada de Emergencia** | **Pin D2 (INT0)** | `btn_estop` (Pulsador rojo) | Interrupción atómica por flanco de bajada (`FALLING`). |
| **Motor Faja PWM (Lazo 1)** | **Pin D3** | `led_motor_pwm` (LED Cian) | Brillo proporcional al PWM ($0 \dots 200$). |
| **Motor Sentido FWD** | **Pin D4** | `led_motor_in1` (LED Azul) | Señal `IN1` del Driver L298N (Marcha adelante). |
| **Motor Sentido REV / Freno** | **Pin D7** | `led_motor_in2` (LED Naranja)| Señal `IN2` del Driver L298N (Sentido inverso / freno pasivo). |
| **Compuerta Deflectora (Lazo 2)**| **Pin D5** | `servo1` (Servomotor SG90)| Movimiento angular ($45^\circ$, $90^\circ$, $135^\circ$). |
| **Luz Cabina Óptica (Lazo 3)** | **Pin D6** | `led_light_pwm` (LED Blanco)| Brillo proporcional de la lámpara de inspección. |
| **Sensor Sonar Ultrasonido** | **D8 (Trig) / D9 (Echo)** | `ultrasonic1` (HC-SR04) | Barra interactiva de distancia en centímetros. |
| **Buzzer Alarma Sonora** | **Pin D10** | `bz1` (Buzzer piezoeléctrico)| Alarma acústica intermitente durante `EMERG!`. |
| **Baliza RUN (Marcha)** | **Pin D11** | `led_run` (LED Verde) | Estado activo de transporte de piezas. |
| **Baliza INSP / REPOSO** | **Pin D12** | `led_insp` (LED Amarillo) | Modo de espera o pieza en inspección. |
| **Baliza ALARM (Falla)** | **Pin D13** | `led_alarm` (LED Rojo) | Destello continuo ante parada de emergencia. |
| **Sensor Fotométrico LDR** | **Pin A0** | `ldr1` (Módulo LDR) | Slider de lux (reflectancia de la pieza). |
| **Potenciómetro de Setpoint**| **Pin A1** | `pot1` (Potenciómetro 10k) | Ajuste manual analógico del Setpoint de lux (20% a 95%). |
| **Pantalla LCD 16x2 I2C** | **A4 (SDA) / A5 (SCL)** | `lcd1` (LCD1602 I2C `0x27`) | Telemetría local HMI en vivo. |

---

## 3. Compatibilidad: Servomotor en Wokwi vs. Cajas en el Gemelo Digital

En el circuito de Wokwi, el servomotor SG90 actúa físicamente sobre la compuerta deflectora. En los Gemelos Digitales (2D y 3D), la compuerta mecánica desvía la caja hacia su contenedor:

```
                          [ÁNGULO SERVO SG90 (PIN 5)]
                                      │
         ┌────────────────────────────┼────────────────────────────┐
         ▼                            ▼                            ▼
   45° (Retrabajo)              90° (Conforme)               135° (Rechazo)
         │                            │                            │
         ▼                            ▼                            ▼
 [Tolva 1 / Gaveta Ámbar]     [Tolva 2 / Gaveta Verde]     [Tolva 3 / Gaveta Roja]
 (Offset Z = -0.16 m)         (Centro Z = 0.00 m)          (Offset Z = +0.16 m)
```

Ambas representaciones son **100% coherentes y sincronizadas**: la cinemática de las cajas en 3D responde de manera determinista al ángulo que emite el firmware en Wokwi.

---

## 4. Enlace Hardware-In-The-Loop (HIL) con el Gemelo Digital

El Gemelo Digital ([`simulador_digital_twin.py`](file:///c:/PY/Auto_tif/simulador_digital_twin.py)) cuenta con un cliente de puente automático para conectarse a Wokwi en tiempo real mediante el puerto virtual RFC2217 (`localhost:4000`):

1. **Paso 1:** En VS Code, presiona `F1` y ejecuta `Wokwi: Start Simulator`.
2. **Paso 2:** Inicia el Gemelo Digital con [`iniciar_simulador.bat`](file:///c:/PY/Auto_tif/iniciar_simulador.bat).
3. **Paso 3:** En el panel de control (Pestaña 1 o Pestaña 4), haz clic en el botón **`[⚡ Conectar Wokwi (Port 4000)]`**.
4. **Resultado:**
   * El indicador cambiará a **`[✓ Wokwi Online]`** y la insignia mostrará **`● HIL Activo (C++)`**.
   * Las cajas físicas que avanzan en la faja 3D envían distancias y reflectancias continuas (`SIM_DIST` y `SIM_LDR`) al microcontrolador en Wokwi.
   * El ATmega328P de Wokwi procesa los lazos de control cerrados y gobierna la velocidad del motor y la compuerta de desvío 3D en tiempo real.
   * Si detienes Wokwi o pulsas desconectar, la aplicación conmuta suavemente al emulador local en Python sin interrupciones.

---

## 4. Guía Operativa para Pruebas y Capturas de Informe

### Prueba 1: Modo REPOSO e Inicio de Marcha
1. Iniciar la simulación en Wokwi.
2. Observar la pantalla LCD: `REPOSO | D: 30cm` con el **LED Amarillo (D12)** encendido y el motor apagado (`M: 0`).
3. En el Monitor Serial inferior de Wokwi, escribir `START` y presionar Enter.
4. El sistema pasa a `TRANSP`, enciende el **LED Verde (D11)** y el motor arranca a velocidad nominal (`M: 200`, LEDs Cian y Azul encendidos).

### Prueba 2: Lazo 1 — Desaceleración Anticolisión Proporcional
1. Con la faja en `TRANSP`, hacer clic sobre el sensor `HC-SR04`.
2. Mover la barra de distancia desde 30 cm hacia 12 cm.
3. Observar cómo en la segunda fila del LCD el valor `M:` decrece en tiempo real siguiendo la rampa proporcional:
   $$\text{PWM} = 10 \cdot (d - 5)$$
   El LED Cian (`Lazo1 ENA`) atenúa su brillo hasta llegar a `M: 0` si la distancia baja de 5 cm.

### Prueba 3: Lazos 2 y 3 — Inspección Fotométrica y Clasificación Angular
1. Ajustar la distancia del `HC-SR04` a **8 cm** (rango de detección: 6 cm a 10 cm).
2. El sistema se detiene automáticamente en `INSPEC` durante 1.5 segundos (LED Amarillo activo).
3. Durante ese tiempo, variar el deslizador del módulo LDR:
   * **Luz $\ge 65\%$:** Pieza **CONFORME** $\to$ Servo SG90 se mantiene a **$90^\circ$**.
   * **Luz entre $40\%$ y $64\%$:** Pieza **RETRABAJO** $\to$ Servo SG90 rota a **$45^\circ$**.
   * **Luz $< 40\%$:** Pieza **RECHAZO** $\to$ Servo SG90 rota a **$135^\circ$**.
4. Al cumplirse 2.0 segundos de desvío, la compuerta retorna a $90^\circ$ y el sistema vuelve a `TRANSP`.

### Prueba 4: Seguridad — Interrupción de Parada de Emergencia (ISR INT0)
1. Presionar el botón rojo `E-STOP (D2)`.
2. Inmediatamente en $< 5\ \mu\text{s}$, el motor y la luz caen a 0, el LCD indica `EMERG!`, el **Buzzer (D10)** emite tono intermitente y el **LED Rojo (D13)** parpadea.
3. Para rearmar el sistema, escribir `RESET` en el Monitor Serial. Si el botón no está presionado, la estación retorna a `REPOSO`.

---

## 5. Tabla de Comandos Seriales Disponibles

| Comando Serial | Efecto en la Estación | Respuesta Serial JSON |
| :--- | :--- | :--- |
| `START` | Pone la faja en marcha desde `REPOSO` a `TRANSP` | `{"RESP":"CMD_OK","ACCION":"INICIAR_TRANSPORTE"}` |
| `STOP` | Detiene la faja de forma segura en `REPOSO` | `{"RESP":"CMD_OK","ACCION":"DETENER_EN_REPOSO"}` |
| `RESET` | Desarma la parada de emergencia si el botón D2 fue liberado | `{"RESP":"REARME_EXITOSO","NUEVO_ESTADO":"REPOSO"}` |
| `CLASSIFY:CONFORME` | Fuerza la clasificación de la siguiente pieza como Conforme | `{"RESP":"CMD_OK","CLASIFICACION_ASIGNADA":"CONFORME"}` |
| `CLASSIFY:RETRABAJO`| Fuerza la clasificación de la siguiente pieza como Retrabajo | `{"RESP":"CMD_OK","CLASIFICACION_ASIGNADA":"RETRABAJO"}` |
| `CLASSIFY:RECHAZO`  | Fuerza la clasificación de la siguiente pieza como Rechazo | `{"RESP":"CMD_OK","CLASIFICACION_ASIGNADA":"RECHAZO"}` |
| `SIM_DIST:15.5`     | Inyecta una distancia virtual de 15.5 cm al algoritmo de control | `{"RESP":"SIM_OK","DIST_FORZADA":15.5}` |
| `SIM_LDR:75.0`      | Inyecta un valor de lux virtual de 75.0% al Lazo 3 | `{"RESP":"SIM_OK","LUX_FORZADO":75.0}` |
| `SIM_OFF`           | Desactiva la inyección virtual y vuelve a leer sensores físicos | `{"RESP":"SIM_OFF","MODO":"HARDWARE_REAL"}` |

