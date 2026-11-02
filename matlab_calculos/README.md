# Suite de Cálculos Matemáticos y Simulación Dinámica: Auto-TIF (MATLAB)

Este directorio contiene la memoria formal de cálculo, modelado dinámico, análisis de estabilidad en lazo cerrado y validación cinemática del proyecto **Estación Logística de Selección, Clasificación y Enrutamiento Industrial (Auto-TIF)**, gobernado por un microcontrolador **ATmega328P (Arduino Uno R3)** e integrado con gemelos digitales 2D/3D y simulación circuital HIL (Wokwi).

---

## 📁 Estructura del Módulo de Cálculos

```tree
c:\PY\Auto_tif\matlab_calculos\
├── MEMORIA_CALCULOS_DINAMICOS_AUTOTIF.m # ⭐ ARCHIVO DINAMICO UNIFICADO: Formulas, variables y deducciones
├── main_analisis_autotif.m            # Script Maestro: Ejecuta todas las simulaciones y compila el reporte
├── lazo1_velocidad_frenado.m          # Lazo 1: Modelo motor DC, sensor ultrasónico, polos y parada a 10.3 cm
├── lazo2_servomotor_enrutamiento.m    # Lazo 2: Dinámica SG90 (2do orden), enrutamiento (45°/90°/135°) y Carro M-102
├── lazo3_fotometria_iluminacion.m     # Lazo 3: Balance fotométrico LDR/LED, rechazo de perturbaciones (P vs PI)
├── mecanica_traccion_rampa.m         # Mecánica: Dimensionamiento faja, par NEMA 17, acústica HC-SR04 y Smoothstep C1
├── simulacion_fsm_cooperativa.m       # FSM y RTOS: Planificador millis(), estados, latencia ISR INT0 y OEE
└── README.md                          # Memoria técnica formal, deducciones matemáticas y guía de uso
```

---

## ⚡ 1. Mapeo de Instrumentación y Lazos de Control (Norma ANSI/ISA-5.1)

| Tag ISA-5.1 | Lazo | Instrumento / Periférico | Pin Arduino | Variable de Proceso (PV) | Variable Manipulada (CV) |
| :--- | :---: | :--- | :--- | :--- | :--- |
| **`ZIC-101`** / **`SC-101`** | **Lazo 1** | Sonar HC-SR04 + Driver L298N + Motor DC | D8/D9 (Sonar), D3 (PWM) | Distancia ultrasónica $d(t)$ [cm] | PWM Motor $\in [0, 255]$ |
| **`ZIC-102`** / **`XV-102`** | **Lazo 2** | Servomotor SG90 + Compuerta Deflectora | D5 (PWM / PPM) | Clasificación de pieza | Posición angular $\theta \in [45^\circ, 135^\circ]$ |
| **`QIC-103`** / **`QY-103`** | **Lazo 3** | Sensor LDR + Transistor NPN + Banco LED | A0 (ADC0), D6 (PWM) | Iluminancia relativa $L(t)$ [%] | PWM LED $\in [0, 255]$ |
| **`HS-100`** / **`I-100`**   | **Seguridad** | Seta Parada Emergencia (Interrupción) | D2 (INT0 Pull-up) | Flanco `FALLING` | Corte atómico actuadores ($< 5\ \mu\text{s}$) |

---

## 📐 2. Fundamentos Matemáticos y Ecuaciones por Módulo

### 2.1 Lazo 1: Regulación Proporcional de Velocidad y Frenado por Inercia (`lazo1_velocidad_frenado.m`)
* **Modelo del Motor DC y Faja Transportadora:**
  $$G_v(s) = \frac{V(s)}{\text{PWM}(s)} = \frac{K_m}{\tau_m s + 1}$$
  Donde:
  - $K_m = \frac{v_{max}}{\text{PWM}_{max}} = \frac{25.0\text{ cm/s}}{255\text{ PWM}} = 0.09804\text{ cm/(s}\cdot\text{PWM)}$ ($9.8039 \times 10^{-4}\text{ m/(s}\cdot\text{PWM)}$).
  - $\tau_m = 0.150\text{ s}$ ($150\text{ ms}$, inercia mecánica de rotor y faja).
* **Función de Transferencia de Posición:**
  $$G_x(s) = \frac{X(s)}{\text{PWM}(s)} = \frac{K_m}{s(\tau_m s + 1)}$$
* **Controlador Proporcional Calibrado en Firmware:**
  $$\text{PWM}(d) = \begin{cases} 
  0 & d \le 11.5\text{ cm} \quad (\text{Corte anticolisión}) \\ 
  80 + \frac{d - 11.5}{20.0 - 11.5} \times (200 - 80) & 11.5 < d < 20.0\text{ cm} \quad (\text{Rampa proporcional}) \\ 
  200 & d \ge 20.0\text{ cm} \quad (\text{Transporte nominal})
  \end{cases}$$
  Ganancia efectiva: $K_{p1} = \frac{120}{8.5} \approx 14.118\text{ PWM/cm}$.
* **Polinomio Característico y Estabilidad:**
  $$s^2 + \frac{1}{\tau_m} s + \frac{K_{p1} K_m}{\tau_m} = 0 \implies \omega_n = 3.037\text{ rad/s}, \quad \zeta = 1.097 \quad (\text{Sobreamortiguado})$$
  Al ser $\zeta > 1$, la respuesta temporal no presenta sobreelongación ($M_p = 0\%$), eliminando el riesgo de colisión de las cajas contra la pared del túnel.
* **Demostración Analítica de la Parada en $10.3\text{ cm}$:**
  Al alcanzar el umbral de corte $d = 11.5\text{ cm}$, el motor entra a $80\text{ PWM}$ ($v_{entry} = 80 \times 0.09804 = 7.843\text{ cm/s}$). La ecuación de desaceleración por rozamiento e inercia es:
  $$\frac{dv}{dt} = -\frac{1}{\tau_m} v \implies v(t) = v_{entry} e^{-t/\tau_m}$$
  La distancia de arrastre residual (coastdown) es:
  $$\Delta x = \int_0^\infty v(t)\, dt = v_{entry} \cdot \tau_m = 7.843\text{ cm/s} \times 0.150\text{ s} = 1.176\text{ cm}$$
  Posición de reposo final:
  $$d_{final} = d_{stop} - \Delta x = 11.500\text{ cm} - 1.176\text{ cm} = 10.324\text{ cm} \approx \mathbf{10.3\text{ cm}}$$
  *Queda matemáticamente demostrado que la inercia ubica la caja exactamente en $10.3\text{ cm}$ bajo la cámara de inspección.*

---

### 2.2 Lazo 2: Servomotor SG90 y Carro Indexador M-102 (`lazo2_servomotor_enrutamiento.m`)
* **Modelo Dinámico del Servomotor:**
  $$G_{servo}(s) = \frac{\theta(s)}{\theta_{ref}(s)} = \frac{\omega_n^2}{s^2 + 2\zeta \omega_n s + \omega_n^2}$$
  Con $\omega_n = 30.0\text{ rad/s}$, $\zeta = 0.707$, y saturación física de velocidad angular $\dot{\theta}_{max} = 500^\circ/\text{s}$ ($8.726\text{ rad/s}$).
* **Mapeo de Rutas y Pulsos PPM:**
  - **Retrabajo:** $\theta = 45^\circ$, $T_{pulso} = 1.250\text{ ms}$, Destino: Tolva Izquierda / Carro $Z = +0.075\text{ m}$.
  - **Conforme:** $\theta = 90^\circ$, $T_{pulso} = 1.500\text{ ms}$, Destino: Tolva Central / Carro $Z = 0.000\text{ m}$.
  - **Rechazo:** $\theta = 135^\circ$, $T_{pulso} = 1.750\text{ ms}$, Destino: Tolva Derecha / Carro $Z = -0.075\text{ m}$.
* **Fuerza de Deflexión y Factor de Seguridad:**
  Impacto de caja ($m = 0.050\text{ kg}$, $v = 0.18\text{ m/s}$, deflexión $45^\circ$ en $40\text{ ms}$):
  $$F_{deflexion} = \frac{m v \sin(45^\circ)}{\Delta t} = 0.159\text{ N} \implies \tau_{deflexion} = F \cdot (0.7 L) = 0.0072\text{ N}\cdot\text{m} = 0.074\text{ kg}\cdot\text{cm}$$
  Par disponible SG90: $\tau_{stall} = 1.80\text{ kg}\cdot\text{cm}$.
  $$\text{Factor de Seguridad (SF)} = \frac{1.80}{0.074} = \mathbf{24.3} \gg 5.0$$
* **Cinemática del Carro Móvil Indexador:**
  Velocidad $v_{cart} = 0.38\text{ m/s}$. Carrera máxima $\Delta z = 75\text{ mm}$.
  $$t_{transito} = \frac{0.075\text{ m}}{0.38\text{ m/s}} = 0.197\text{ s} < T_{desvio} = 1.000\text{ s} \quad (\text{Margen de seguridad: } 803\text{ ms})$$

---

### 2.3 Lazo 3: Balance Fotométrico y Regulación de Iluminación (`lazo3_fotometria_iluminacion.m`)
* **Modelo de la Planta Fotométrica:**
  $$\tau_{ldr} \frac{d(PV)}{dt} + PV(t) = L_{amb}(t) + K_{led} \cdot \text{PWM}(t)$$
  Con $\tau_{ldr} = 0.080\text{ s}$ ($80\text{ ms}$) y ganancia $K_{led} = \frac{60\%}{255\text{ PWM}} = 0.23529\%/\text{PWM}$.
* **Controlador Proporcional del Firmware ($SP = 70.0\%$):**
  $$\text{PWM}(t) = \text{sat}_{[0, 255]}\Big( \text{PWM}_{base} + K_{p3} \cdot (SP - PV) \cdot 2.55 \Big)$$
  Ganancia dimensional: $K_c = 2.5 \times 2.55 = 6.375\text{ PWM}/\%$.
  Ganancia de lazo: $K_{loop} = K_c \cdot K_{led} = 1.500$.
* **Dinámica en Lazo Cerrado:**
  $$\tau_{eff} = \frac{\tau_{ldr}}{1 + K_{loop}} = \frac{0.080}{1 + 1.500} = 0.032\text{ s} = \mathbf{32\text{ ms}} \implies t_s(2\%) = 4 \tau_{eff} = \mathbf{128\text{ ms}} < 150\text{ ms}$$
* **Atenuación de Perturbaciones de Luz Ambiental:**
  - Modo P: Atenuación en DC $= \frac{1}{1 + K_{loop}} = \frac{1}{2.50} = 0.40$ ($60\%$ de compensación).
  - Modo PI propuesto ($T_i = \tau_{ldr} = 0.08\text{ s}$): Error estático nulo ($e_{ss} = 0.00\%$, $100\%$ de rechazo).

---

### 2.4 Mecánica, Tracción y Acústica de Rampa (`mecanica_traccion_rampa.m`)
* **Tracción y Par Motor:**
  - Banda: $L_{faja} = 360\text{ mm}$, $W = 70\text{ mm}$, rodillos $\varnothing 26\text{ mm}$ ($r = 13\text{ mm}$).
  - Carga: $m_{banda} = 0.101\text{ kg}$, $m_{cajas} = 0.200\text{ kg}$, $\mu = 0.28$.
  - Par torsor estático en régimen: $\tau_{est} = 1.139\text{ N}\cdot\text{cm}$.
  - Par torsor pico dinámico ($a = 1.2\text{ m/s}^2$): $\tau_{pico} = 1.814\text{ N}\cdot\text{cm}$.
  - Motor paso a paso NEMA 17: Par retención $= 26.0\text{ N}\cdot\text{cm} \implies \text{SF} = 14.3$.
* **Física Acústica del HC-SR04 ($40\text{ kHz}$):**
  - Velocidad del sonido ($22^\circ\text{C}$): $c = 344.4\text{ m/s} \implies \lambda = \frac{344.4}{40000} = 8.61\text{ mm}$.
  - Cono de dispersión útil: $\pm 15^\circ$. Incidencia obligatoria a $90^\circ$ sobre la cara frontal para reflexión especular máxima.
* **Cinemática de Rampa con Transición Smoothstep $C^1$:**
  $$s(t) = 3t^2 - 2t^3, \quad \theta_z(x) = -52.0^\circ \cdot s(x)$$
  Boca de caída libre de $58.0\text{ mm} \times 78.0\text{ mm}$, permitiendo el descenso ininterrumpido sin colisiones con los brazos de extensión del sensor ultrasónico ($X = -232\text{ mm}$).

---

### 2.5 Arquitectura Multitarea y FSM (`simulacion_fsm_cooperativa.m`)
* **Planificador Cooperativo No Bloqueante (`millis()`):**
  - Lazos de Control (D3, D6): $50\text{ ms}$ ($20\text{ Hz}$).
  - Transmisor Ultrasónico (D8, D9): $60\text{ ms}$ ($16.7\text{ Hz}$).
  - Telemetría Serie / HIL Wokwi: $30\text{ ms}$ ($33\text{ Hz}$).
  - Interfaz LCD 16x2 I2C: $250\text{ ms}$ ($4\text{ Hz}$).
* **Interrupción de Seguridad por Hardware (ISR INT0):**
  - Desconexión atómica de actuadores en $< 5\ \mu\text{s}$ ante flanco de bajada (`FALLING`) en Pin D2.
* **Métricas de Rendimiento Industrial:**
  - Throughput medio: $15 - 20\text{ piezas/minuto}$.
  - Tiempo de ciclo medio: $T_{cycle} \approx 3.5\text{ s/pieza}$.
  - OEE Global Estimado: $> 85\%$.

---

## 🚀 3. Instrucciones de Ejecución

### En MATLAB (Escritorio o Terminal)
Abra MATLAB en la carpeta del proyecto y ejecute:
```matlab
% Desde la consola de MATLAB:
cd('C:\PY\Auto_tif\matlab_calculos');
main_analisis_autotif;
```
O ejecute cada módulo individualmente:
```matlab
lazo1_velocidad_frenado;          % Análisis detallado Lazo 1
lazo2_servomotor_enrutamiento;    % Análisis detallado Lazo 2
lazo3_fotometria_iluminacion;     % Análisis detallado Lazo 3
mecanica_traccion_rampa;         % Análisis mecánico y acústico
simulacion_fsm_cooperativa;       % Simulación de lote y FSM
```

### En Modo Batch (Línea de Comandos de Windows / PowerShell)
```powershell
& "C:\Program Files\MATLAB\R2024a\bin\matlab.exe" -batch "cd('C:/PY/Auto_tif/matlab_calculos'); main_analisis_autotif"
```

### En GNU Octave
Todos los scripts fueron diseñados con sintaxis matemática universal compatible con GNU Octave 6.0+:
```bash
octave --persist main_analisis_autotif.m
```

---

## 📊 4. Correlación con el Firmware y Gemelo Digital 3D

| Parámetro Teórico | Archivo MATLAB | Archivo Firmware C++ | Script Python 3D |
| :--- | :--- | :--- | :--- |
| Parada anticolisión ($10.3\text{ cm}$) | `lazo1_velocidad_frenado.m` | `DIST_MIN_STOP_CM` ($11.5\text{ cm}$) | `simulador_3d_desktop.py` (corte $11.5\text{ cm}$, parada $10.3\text{ cm}$) |
| Ángulos servo ($45^\circ / 90^\circ / 135^\circ$) | `lazo2_servomotor_enrutamiento.m`| `SERVO_ANG_RETRABAJO`, `CONFORME`, `RECHAZO` | Mapeo cinemático tolvas y carro $Z$ |
| Ganancia Lazo Luz ($K_{p3} = 2.5$) | `lazo3_fotometria_iluminacion.m` | `KP_LIGHT = 2.5f`, `SP_LUX_DEFAULT = 70.0f` | `sp_lux_pct = 70.0`, `kp_light = 2.5` |
| Rampa Smoothstep ($-52^\circ$) | `mecanica_traccion_rampa.m` | N/A (Geometría física) | `tan52 = 1.27994`, interpolación $s(t)$ |
| Periodos de Tareas ($50, 60, 30, 250\text{ ms}$) | `simulacion_fsm_cooperativa.m` | `PERIOD_CONTROL_MS`, `PERIOD_ULTRASONIC_MS` | `sync_sensors()` a $33\text{ Hz}$ |
