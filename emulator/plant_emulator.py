"""
=====================================================================================
EMULADOR DE PLANTA FÍSICA Y ENTORNO HARDWARE-IN-THE-LOOP (HIL)
SISTEMA: Estación Logística de Selección, Clasificación y Enrutamiento
PLATAFORMA OBJETIVO: Arduino Uno R3 / Emulación Autónoma Python
=====================================================================================

Este emulador modela la física continua y discreta del proceso:
1. Dinámica de la Faja Transportadora y sensor ultrasónico (Lazo 1).
2. Cinemática del Servomotor SG90 y enrutamiento a tolvas (Lazo 2).
3. Balance fotométrico: Luz ambiental + Iluminancia LED controlada por LDR (Lazo 3).
4. Máquina de Estados Finitos (FSM) interna y procesamiento de Parada de Emergencia.
5. Modo Dual:
   - Modo Autónomo (Simulación completa Física + Firmware).
   - Modo HIL (Hardware-In-The-Loop) conectado vía Serial/Bluetooth al Arduino real.
"""

import sys
import time
import math
import random
import threading
from dataclasses import dataclass
from typing import Optional, List, Tuple

try:
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError:
    print("[ADVERTENCIA] Matplotlib/NumPy no instalados. Se ejecutará en modo CLI interactivo.")
    np = None
    plt = None

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


@dataclass
class PlantState:
    # Cinemática de la faja
    box_position_m: float = 0.0          # Posición del paquete (0.0 a 1.2 metros)
    belt_speed_mps: float = 0.0          # Velocidad actual (m/s)
    motor_pwm: int = 0                   # Entrada de control (0 a 255)
    ultrasonic_dist_cm: float = 50.0     # Distancia medida al obstáculo/sensor (cm)
    
    # Servocontrol de Enrutamiento / Carro Móvil Indexador (M-102)
    servo_angle_deg: float = 90.0        # Ángulo del servo (45: Retrabajo, 90: Conforme, 135: Rechazo)
    routed_chute: str = "NINGUNA"        # Tolva / Contenedor de destino
    cart_z: float = 0.0                  # Posición transversal del carro móvil (m)
    target_cart_z: float = 0.0           # Posición objetivo del carro móvil (m)
    cart_speed_mps: float = 0.0          # Velocidad de traslación del carro móvil (m/s)
    
    # Fotometría e Iluminación
    ambient_lux_pct: float = 40.0        # Perturbación de luz ambiental (0 - 100%)
    led_pwm: int = 0                     # Entrada PWM de potencia luminaria (0 - 255)
    total_lux_pct: float = 40.0          # Luz resultante en zona de inspección (PV Lazo 3)
    sp_lux_pct: float = 70.0             # Setpoint de control
    
    # Estados de supervisión
    fsm_state: str = "REPOSO"
    emergency_stop: bool = False
    piece_quality: str = "CONFORME"      # "CONFORME", "RETRABAJO", "RECHAZO"


class PlantPhysicsModel:
    """
    Simulador de las ecuaciones diferenciales y cinemáticas de la planta física.
    """
    def __init__(self, dt: float = 0.05):
        self.dt = dt
        self.state = PlantState()
        
        # Parámetros físicos de la planta
        self.BELT_LENGTH_M = 1.0           # Longitud total de la faja (100 cm)
        self.INSPECTION_ZONE_M = 0.85      # Zona donde el HC-SR04 y la LDR monitorean (85 cm)
        self.MAX_BELT_SPEED = 0.25         # Velocidad máx a 255 PWM = 0.25 m/s
        self.MOTOR_TIME_CONST = 0.15       # Inercia del motor DC (Tau = 150 ms)
        self.LDR_TIME_CONST = 0.08         # Respuesta temporal del sensor LDR
        
        # Ruido y perturbaciones
        self.ambient_light_noise = 0.0
        self.sensor_noise_cm = 0.0
        
        # Generador de piezas
        self.piece_active = True
        self.state.box_position_m = 0.1

    def inject_disturbance(self, ambient_lux_drop: float):
        """Inyecta una caída o incremento en la luz ambiental."""
        self.state.ambient_lux_pct = max(0.0, min(100.0, ambient_lux_drop))

    def trigger_emergency(self, activate: bool):
        """Activa o desactiva la señal física de parada de emergencia."""
        self.state.emergency_stop = activate
        if activate:
            self.state.motor_pwm = 0
            self.state.belt_speed_mps = 0.0
            self.state.led_pwm = 0

    def reset_piece(self, quality: str = "CONFORME"):
        """Coloca una nueva pieza en el alimentador de entrada."""
        self.state.box_position_m = 0.05
        self.piece_active = True
        self.state.piece_quality = quality
        self.state.routed_chute = "EN_PROCESO"

    def step(self, motor_pwm_cmd: int, led_pwm_cmd: int, servo_angle_cmd: float, target_cart_z: Optional[float] = None):
        """
        Avanza un paso de integración temporal en la física de la planta.
        """
        if self.state.emergency_stop:
            motor_pwm_cmd = 0
            led_pwm_cmd = 0
            self.state.belt_speed_mps = 0.0

        self.state.motor_pwm = motor_pwm_cmd
        self.state.led_pwm = led_pwm_cmd
        self.state.servo_angle_deg = servo_angle_cmd

        # Dinámica del Carro Móvil Indexador M-102
        if target_cart_z is not None:
            self.state.target_cart_z = target_cart_z
        dz = self.state.target_cart_z - self.state.cart_z
        max_step = 0.38 * self.dt
        if abs(dz) <= max_step:
            self.state.cart_z = self.state.target_cart_z
            self.state.cart_speed_mps = 0.0
        else:
            dir_z = 1.0 if dz > 0.0 else -1.0
            self.state.cart_z += dir_z * max_step
            self.state.cart_speed_mps = dir_z * 0.38

        # 1. Dinámica del Motor y Faja (Modelo de 1er Orden)
        target_speed = (motor_pwm_cmd / 255.0) * self.MAX_BELT_SPEED
        alpha_m = math.exp(-self.dt / self.MOTOR_TIME_CONST)
        self.state.belt_speed_mps = alpha_m * self.state.belt_speed_mps + (1 - alpha_m) * target_speed

        # 2. Desplazamiento de la pieza
        if self.piece_active:
            self.state.box_position_m += self.state.belt_speed_mps * self.dt
            
            # Distancia física hacia el sensor ultrasónico ubicado al final de la zona
            # Distancia en cm desde el sensor hacia el borde frontal de la pieza
            dist_exacta_cm = (self.INSPECTION_ZONE_M - self.state.box_position_m) * 100.0
            
            if dist_exacta_cm < 2.0:
                dist_exacta_cm = 2.0
            elif dist_exacta_cm > 100.0:
                dist_exacta_cm = 100.0

            # Añadir ruido Gaussiano de medición ultrasónica
            noise = random.gauss(0, 0.4)
            self.state.ultrasonic_dist_cm = max(2.0, dist_exacta_cm + noise)

            # Si la pieza superó el punto de enrutamiento
            if self.state.box_position_m >= self.BELT_LENGTH_M:
                # Determinar en qué tolva cayó según el ángulo del servo
                ang = self.state.servo_angle_deg
                if abs(ang - 45.0) < 15.0:
                    self.state.routed_chute = "TOLVA_RETRABAJO (45°)"
                elif abs(ang - 135.0) < 15.0:
                    self.state.routed_chute = "TOLVA_RECHAZO (135°)"
                else:
                    self.state.routed_chute = "TOLVA_CONFORME (90°)"
                
                self.piece_active = False # Pieza evacuada
        else:
            # Faja libre
            self.state.ultrasonic_dist_cm = 100.0 + random.gauss(0, 0.5)

        # 3. Balance Fotométrico (Lazo 3)
        # La iluminancia total es la suma de la luz ambiental + la iluminación LED
        # Ganancia del foco LED: a 255 PWM aporta hasta +60% de lux en la cámara
        led_lux_contrib = (led_pwm_cmd / 255.0) * 60.0
        target_lux = self.state.ambient_lux_pct + led_lux_contrib + random.gauss(0, 0.3)
        target_lux = max(0.0, min(100.0, target_lux))

        # Respuesta de la fotorresistencia LDR (filtro de 1er orden)
        alpha_ldr = math.exp(-self.dt / self.LDR_TIME_CONST)
        self.state.total_lux_pct = alpha_ldr * self.state.total_lux_pct + (1 - alpha_ldr) * target_lux


class VirtualArduinoFirmware:
    """
    Emulación idéntica del Firmware Arduino C++ para ejecución autónoma sin hardware.
    """
    def __init__(self):
        self.fsm_state = "REPOSO"
        self.sp_dist_cm = 25.0
        self.sp_lux_pct = 70.0
        self.kp_light = 2.5
        self.light_base_pwm = 128
        
        self.motor_pwm = 0
        self.led_pwm = 25
        self.servo_angle = 90
        self.target_cart_z = 0.0
        
        self.inspect_timer = 0.0
        self.divert_timer = 0.0
        self.piece_classification = "NINGUNA"

    def execute_cycle(self, dt: float, pv_dist: float, pv_lux: float, e_stop: bool):
        if e_stop:
            self.fsm_state = "EMERG!"
            self.motor_pwm = 0
            self.led_pwm = 0
            return self.motor_pwm, self.led_pwm, self.servo_angle

        # 1. MÁQUINA DE ESTADOS FINITOS (FSM)
        if self.fsm_state == "REPOSO":
            self.servo_angle = 90
            # Auto-inicio fluido al detectar piezas en línea
            if pv_dist < 95.0 or pv_dist > 15.0:
                self.fsm_state = "TRANSP"

        elif self.fsm_state == "TRANSP":
            if pv_dist <= 11.5:
                self.fsm_state = "INSPEC"
                self.inspect_timer = 0.0

        elif self.fsm_state == "INSPEC":
            self.inspect_timer += dt
            if self.inspect_timer >= 0.8:
                # Clasificación por reflectancia/iluminancia
                if pv_lux >= 65.0:
                    self.piece_classification = "CONFORME"
                    self.servo_angle = 90
                    self.target_cart_z = 0.0
                elif pv_lux >= 40.0:
                    self.piece_classification = "RETRABAJO"
                    self.servo_angle = 45
                    self.target_cart_z = 0.16
                else:
                    self.piece_classification = "RECHAZO"
                    self.servo_angle = 135
                    self.target_cart_z = -0.16
                
                self.fsm_state = "DESVIO"
                self.divert_timer = 0.0

        elif self.fsm_state == "DESVIO":
            self.divert_timer += dt
            if self.divert_timer >= 1.0:
                self.piece_classification = "NINGUNA"
                self.fsm_state = "TRANSP"

        # 2. LAZO 1: Control de Velocidad y Anticolisión
        if self.fsm_state == "TRANSP":
            if pv_dist <= 11.5:
                self.motor_pwm = 0
            elif pv_dist >= 20.0:
                self.motor_pwm = 200
            else:
                factor = (pv_dist - 11.5) / (20.0 - 11.5)
                self.motor_pwm = int(80 + factor * 120)
                self.motor_pwm = max(80, min(200, self.motor_pwm))
        elif self.fsm_state == "DESVIO":
            # Pulso controlado de expulsión hacia la rampa
            self.motor_pwm = 180
        else:
            self.motor_pwm = 0

        # 3. LAZO 3: Iluminación
        if self.fsm_state in ["TRANSP", "INSPEC"]:
            err_lux = self.sp_lux_pct - pv_lux
            calc_pwm = self.light_base_pwm + (self.kp_light * err_lux * 2.55)
            self.led_pwm = int(max(0, min(255, calc_pwm)))
        else:
            self.led_pwm = 25

        return self.motor_pwm, self.led_pwm, self.servo_angle


class IndustrialDashboard:
    """
    Panel gráfico en tiempo real con 4 cuadrantes de telemetría y control.
    """
    def __init__(self, physics: PlantPhysicsModel, firmware: VirtualArduinoFirmware):
        self.physics = physics
        self.firmware = firmware
        self.time_history = []
        self.dist_history = []
        self.motor_pwm_history = []
        self.ambient_lux_history = []
        self.total_lux_history = []
        self.led_pwm_history = []
        self.servo_history = []
        self.fsm_history = []
        
        self.sim_time = 0.0
        self.max_samples = 150

    def start_gui(self):
        if plt is None:
            self.start_cli_loop()
            return

        plt.style.use('dark_background')
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(13, 8))
        fig.suptitle("ESTACIÓN LOGÍSTICA DE CLASIFICACIÓN - EMULADOR DE PLANTA HIL", fontsize=14, color='#00FFAA', weight='bold')

        # Subplot 1: Lazo 1 (Distancia vs PWM Motor)
        line_dist, = ax1.plot([], [], '#00E5FF', label="Distancia PV (cm)", lw=2)
        line_pwm_m, = ax1.plot([], [], '#FF9100', label="Motor PWM (0-255)", lw=1.8, linestyle="--")
        ax1.axhline(25.0, color='#00E5FF', linestyle=":", alpha=0.6, label="SP Dist (25cm)")
        ax1.axhline(5.0, color='#FF1744', linestyle=":", alpha=0.8, label="Stop (5cm)")
        ax1.set_title("Lazo 1: Control de Velocidad y Frenado Anticolisión", color='white', fontsize=10)
        ax1.set_ylabel("cm / PWM")
        ax1.set_ylim(-5, 265)
        ax1.legend(loc="upper right", fontsize=8)
        ax1.grid(True, alpha=0.2)

        # Subplot 2: Lazo 3 (Iluminancia vs Compensación LED)
        line_lux_tot, = ax2.plot([], [], '#76FF03', label="Lux Inspección PV (%)", lw=2)
        line_lux_amb, = ax2.plot([], [], '#78909C', label="Luz Ambiental Perturbación (%)", lw=1.5, linestyle=":")
        line_led_pwm, = ax2.plot([], [], '#FFEA00', label="Actuador LED PWM (0-255)", lw=1.8, linestyle="--")
        ax2.axhline(70.0, color='#76FF03', linestyle=":", alpha=0.7, label="SP Lux (70%)")
        ax2.set_title("Lazo 3: Estabilización Proporcional de Iluminación", color='white', fontsize=10)
        ax2.set_ylabel("% / PWM")
        ax2.set_ylim(-5, 265)
        ax2.legend(loc="upper right", fontsize=8)
        ax2.grid(True, alpha=0.2)

        # Subplot 3: Lazo 2 (Ángulo Servomotor SG90)
        line_servo, = ax3.plot([], [], '#D500F9', label="Ángulo Compuerta (°)", lw=2.2)
        ax3.axhline(90, color='#00E676', linestyle="--", label="90° Conforme (Paso Directo)")
        ax3.axhline(45, color='#FFD600', linestyle="--", label="45° Retrabajo")
        ax3.axhline(135, color='#FF1744', linestyle="--", label="135° Rechazo")
        ax3.set_title("Lazo 2: Servocontrol de Enrutamiento Angular", color='white', fontsize=10)
        ax3.set_ylabel("Ángulo (°)")
        ax3.set_ylim(20, 160)
        ax3.legend(loc="upper right", fontsize=8)
        ax3.grid(True, alpha=0.2)

        # Subplot 4: Posición de Pieza en Faja y Estado FSM
        ax4.set_title("Supervisión de Planta: FSM y Posición de Transporte", color='white', fontsize=10)
        ax4.set_xlim(0, 1.1)
        ax4.set_ylim(-0.5, 1.5)
        ax4.set_xlabel("Recorrido de Faja (Metros)")
        ax4.set_yticks([])
        ax4.grid(True, alpha=0.1)

        # Elementos gráficos interactivos de la faja
        belt_rect = plt.Rectangle((0, 0.2), 1.0, 0.3, color='#37474F', alpha=0.8)
        ax4.add_patch(belt_rect)
        box_patch = plt.Rectangle((0.1, 0.25), 0.08, 0.2, color='#00E5FF')
        ax4.add_patch(box_patch)
        sensor_line = ax4.axvline(0.85, color='#FF1744', linestyle="--", lw=2, label="Zona Sensor/Cámara")
        
        status_text = ax4.text(0.05, 0.9, "", color='#00FFAA', fontsize=10, weight='bold')
        chute_text = ax4.text(0.05, -0.3, "", color='#FFD600', fontsize=10)

        # Control interactivo de teclado
        def on_key(event):
            if event.key == 'e':
                self.physics.trigger_emergency(not self.physics.state.emergency_stop)
                print(f"[EVENTO] Parada de Emergencia: {'ACTIVADA' if self.physics.state.emergency_stop else 'DESACTIVADA'}")
            elif event.key == 'n':
                qual = random.choice(["CONFORME", "RETRABAJO", "RECHAZO"])
                self.physics.reset_piece(qual)
                print(f"[EVENTO] Nueva pieza generada en faja -> Tipo: {qual}")
            elif event.key == 'l':
                nueva_luz = 15.0 if self.physics.state.ambient_lux_pct > 30.0 else 60.0
                self.physics.inject_disturbance(nueva_luz)
                print(f"[PERTURBACIÓN] Luz ambiental forzada a: {nueva_luz}%")

        fig.canvas.mpl_connect('key_press_event', on_key)

        print("\n" + "="*70)
        print(" [CONTROL INTERACTIVO DE SIMULACIÓN - TECLAS ACTIVAS]")
        print("   [N] : Generar Nueva Pieza en la Faja")
        print("   [L] : Inyectar Perturbación de Luz Ambiental (Oscuridad/Luz)")
        print("   [E] : Pulsador Parada de Emergencia (Interrupción Externa D2)")
        print("="*70 + "\n")

        def update(frame):
            dt = 0.05
            self.sim_time += dt

            # Ejecutar ciclo de control firmware
            m_pwm, l_pwm, s_ang = self.firmware.execute_cycle(
                dt,
                self.physics.state.ultrasonic_dist_cm,
                self.physics.state.total_lux_pct,
                self.physics.state.emergency_stop
            )

            # Avanzar física
            self.physics.step(m_pwm, l_pwm, s_ang)
            self.physics.state.fsm_state = self.firmware.fsm_state

            # Auto reposición de piezas si concluyó el ciclo
            if not self.physics.piece_active and self.firmware.fsm_state == "REPOSO":
                if random.random() < 0.05:
                    self.physics.reset_piece(random.choice(["CONFORME", "RETRABAJO", "RECHAZO"]))

            # Registrar series temporales
            self.time_history.append(self.sim_time)
            self.dist_history.append(self.physics.state.ultrasonic_dist_cm)
            self.motor_pwm_history.append(self.physics.state.motor_pwm)
            self.ambient_lux_history.append(self.physics.state.ambient_lux_pct)
            self.total_lux_history.append(self.physics.state.total_lux_pct)
            self.led_pwm_history.append(self.physics.state.led_pwm)
            self.servo_history.append(self.physics.state.servo_angle_deg)

            if len(self.time_history) > self.max_samples:
                self.time_history.pop(0)
                self.dist_history.pop(0)
                self.motor_pwm_history.pop(0)
                self.ambient_lux_history.pop(0)
                self.total_lux_history.pop(0)
                self.led_pwm_history.pop(0)
                self.servo_history.pop(0)

            # Actualizar curvas
            t_data = self.time_history
            line_dist.set_data(t_data, self.dist_history)
            line_pwm_m.set_data(t_data, self.motor_pwm_history)
            ax1.set_xlim(max(0, t_data[0]), t_data[-1] + 0.5)

            line_lux_tot.set_data(t_data, self.total_lux_history)
            line_lux_amb.set_data(t_data, self.ambient_lux_history)
            line_led_pwm.set_data(t_data, self.led_pwm_history)
            ax2.set_xlim(max(0, t_data[0]), t_data[-1] + 0.5)

            line_servo.set_data(t_data, self.servo_history)
            ax3.set_xlim(max(0, t_data[0]), t_data[-1] + 0.5)

            # Actualizar faja gráfica
            box_patch.set_x(self.physics.state.box_position_m)
            box_patch.set_visible(self.physics.piece_active)
            
            # Color según calidad
            if self.physics.state.piece_quality == "CONFORME":
                box_patch.set_color('#00E676')
            elif self.physics.state.piece_quality == "RETRABAJO":
                box_patch.set_color('#FFD600')
            else:
                box_patch.set_color('#FF1744')

            status_text.set_text(
                f"FSM: [{self.physics.state.fsm_state}] | Dist: {self.physics.state.ultrasonic_dist_cm:.1f} cm | "
                f"PWM: {self.physics.state.motor_pwm} | Lux: {self.physics.state.total_lux_pct:.1f}%"
            )
            chute_text.set_text(f"Último Destino de Pieza: {self.physics.state.routed_chute}")

            return line_dist, line_pwm_m, line_lux_tot, line_led_pwm, line_servo, box_patch, status_text

        ani = FuncAnimation(fig, update, interval=50, blit=False)
        plt.tight_layout()
        plt.show()

    def start_cli_loop(self):
        """Bucle de ejecución interactiva en terminal si no hay entorno gráfico."""
        print("\n--- INICIANDO SIMULADOR EN MODO TERMINAL (CLI) ---")
        dt = 0.05
        step_count = 0
        while True:
            step_count += 1
            m_pwm, l_pwm, s_ang = self.firmware.execute_cycle(
                dt,
                self.physics.state.ultrasonic_dist_cm,
                self.physics.state.total_lux_pct,
                self.physics.state.emergency_stop
            )
            self.physics.step(m_pwm, l_pwm, s_ang)
            
            if step_count % 10 == 0:
                print(f"[{time.strftime('%X')}] FSM:{self.firmware.fsm_state:<7} | "
                      f"DIST:{self.physics.state.ultrasonic_dist_cm:5.1f}cm | "
                      f"PWM_MOT:{self.physics.state.motor_pwm:3d} | "
                      f"LUX:{self.physics.state.total_lux_pct:5.1f}% | "
                      f"PWM_LED:{self.physics.state.led_pwm:3d} | "
                      f"SERVO:{self.physics.state.servo_angle_deg:3.0f}°")
            time.sleep(dt)


def run_hil_mode(com_port: str, baud_rate: int = 9600):
    """
    Modo Hardware-In-The-Loop: Se conecta al puerto COM del Arduino físico.
    """
    if serial is None:
        print("[ERROR] pyserial no está instalado. Ejecute 'pip install pyserial'.")
        return

    print(f"[HIL] Conectando a {com_port} a {baud_rate} baudios...")
    try:
        ser = serial.Serial(com_port, baud_rate, timeout=0.1)
        time.sleep(2.0) # Esperar reset de Arduino
        print("[HIL] Conexión establecida con éxito. Transmitiendo variables de planta...")
        
        physics = PlantPhysicsModel(dt=0.05)
        
        while True:
            # 1. Enviar variables físicas emuladas al Arduino (SIM_DIST y SIM_LDR)
            cmd_dist = f"SIM_DIST:{physics.state.ultrasonic_dist_cm:.1f}\n"
            cmd_ldr = f"SIM_LDR:{physics.state.total_lux_pct:.1f}\n"
            ser.write(cmd_dist.encode('utf-8'))
            ser.write(cmd_ldr.encode('utf-8'))

            # 2. Leer telemetría de respuesta del Arduino
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("TLM,"):
                    # Formato: TLM,STATUS,DIST,PWM_MOTOR,LUX,SERVO_DEG,EMERGENCY
                    parts = line.split(',')
                    if len(parts) >= 7:
                        fsm_st = parts[1]
                        pwm_m = int(parts[3])
                        pwm_led = int(float(parts[4]) * 2.55) # aprox
                        servo_deg = float(parts[5])
                        
                        # Actualizar la física con los actuadores reales del microcontrolador
                        physics.step(pwm_m, pwm_led, servo_deg)
                        print(f"[HIL TELEMETRÍA] Arduino FSM: {fsm_st} | PWM Motor: {pwm_m} | Servo: {servo_deg}°")

            time.sleep(0.05)
    except Exception as e:
        print(f"[HIL ERROR] Error en la comunicación serial: {e}")


if __name__ == "__main__":
    print("=====================================================================")
    print("  ESTACIÓN LOGÍSTICA INDUSTRIAL - EMULADOR DE PLANTA FÍSICA Y HIL   ")
    print("=====================================================================")
    print("Seleccione modo de operación:")
    print("1. Simulación Autónoma Integrada (GUI con Gráficos en Tiempo Real)")
    print("2. Hardware-In-The-Loop (Conexión a Arduino Uno R3 Físico por Serial)")
    
    if len(sys.argv) > 1 and sys.argv[1] == "--hil":
        port = sys.argv[2] if len(sys.argv) > 2 else "COM3"
        run_hil_mode(port)
    else:
        physics_sim = PlantPhysicsModel()
        firmware_sim = VirtualArduinoFirmware()
        dashboard = IndustrialDashboard(physics_sim, firmware_sim)
        dashboard.start_gui()
