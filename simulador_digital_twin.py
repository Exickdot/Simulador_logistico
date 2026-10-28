"""
=====================================================================================
SISTEMA DIGITAL TWIN & SUITE SCADA INDUSTRIAL - TEMA CLARO / INGENIERÍA
ESTACIÓN LOGÍSTICA DE SELECCIÓN, CLASIFICACIÓN Y ENRUTAMIENTO
NORMATIVA DE INSTRUMENTACIÓN: ANSI / ISA-5.1 (P&ID Industrial Plano Blanco)
PLATAFORMA: Arduino Uno R3 (ATmega328P) / Entorno Virtual (.venv)
=====================================================================================
"""

import sys
import os
import time
import math
import random
from dataclasses import dataclass
from typing import List, Optional

import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# =====================================================================================
# MODELOS DE FÍSICA Y FIRMWARE VIRTUAL ARDUINO
# =====================================================================================

@dataclass
class SimulatedPackage:
    id_num: int
    pos_x_m: float = 0.0          # Posición sobre la faja (0.0 a 1.2 m)
    quality: str = "CONFORME"     # "CONFORME", "RETRABAJO", "RECHAZO"
    color: str = "#2E7D32"        # Verde ingeniería
    reflectance: float = 75.0     # % de reflectancia para sensor óptico
    width_m: float = 0.07         # 7 cm de ancho
    is_inspected: bool = False
    is_diverting: bool = False
    is_routed: bool = False
    routed_chute: int = 2         # 1: Retrabajo (45°), 2: Conforme (90°), 3: Rechazo (135°)
    chute_anim_progress: float = 0.0


class VirtualIndustrialPlant:
    """
    Simulador de física continua de planta y emulación de firmware Arduino Uno R3.
    """
    def __init__(self):
        self.BELT_LENGTH_M = 1.0
        self.SENSOR_X_M = 0.82
        self.GATE_X_M = 0.90
        self.MAX_SPEED_MPS = 0.28
        self.MOTOR_TAU = 0.12
        self.LDR_TAU = 0.06

        self.belt_speed_mps = 0.0
        self.motor_pwm = 0
        self.servo_angle_deg = 90.0
        self.target_servo_angle = 90.0
        
        # Lazo 3: Fotometría
        self.ambient_lux_pct = 40.0
        self.led_pwm = 25
        self.total_lux_pct = 40.0
        self.sp_lux_pct = 70.0
        self.kp_light = 2.5
        self.base_light_pwm = 128
        self.error_lux = 0.0

        # Lazo 1: Ultrasonido
        self.ultrasonic_dist_cm = 100.0
        self.sp_dist_nominal = 25.0
        self.dist_critica_stop = 5.0
        
        # FSM y Seguridad
        self.fsm_state = "REPOSO"
        self.emergency_stop_latched = False
        
        self.inspect_timer = 0.0
        self.divert_timer = 0.0
        
        self.packages: List[SimulatedPackage] = []
        self.package_counter = 0
        self.auto_feed_active = True
        self.last_feed_time = time.time()
        
        self.count_conforme = 0
        self.count_retrabajo = 0
        self.count_rechazo = 0
        self.total_processed = 0
        self.current_piece_under_inspection: Optional[SimulatedPackage] = None

    def spawn_package(self, quality_type: Optional[str] = None):
        for p in self.packages:
            if p.pos_x_m < 0.18 and not p.is_routed:
                return None
                
        self.package_counter += 1
        if quality_type is None:
            r = random.random()
            if r < 0.65:
                quality_type = "CONFORME"
            elif r < 0.85:
                quality_type = "RETRABAJO"
            else:
                quality_type = "RECHAZO"

        if quality_type == "CONFORME":
            col = "#2E7D32" # Verde ingeniería claro
            ref = random.uniform(68.0, 85.0)
        elif quality_type == "RETRABAJO":
            col = "#E65100" # Naranja / Ámbar ingeniería
            ref = random.uniform(45.0, 62.0)
        else:
            col = "#C62828" # Rojo ladrillo
            ref = random.uniform(15.0, 38.0)

        pkg = SimulatedPackage(
            id_num=self.package_counter,
            pos_x_m=0.01,
            quality=quality_type,
            color=col,
            reflectance=ref
        )
        self.packages.append(pkg)
        
        if self.fsm_state == "REPOSO" and not self.emergency_stop_latched:
            self.fsm_state = "TRANSPORTE"
            
        return pkg

    def trigger_emergency(self, active: bool):
        self.emergency_stop_latched = active
        if active:
            self.fsm_state = "EMERGENCIA"
            self.motor_pwm = 0
            self.belt_speed_mps = 0.0
            self.led_pwm = 0

    def reset_emergency(self) -> bool:
        if not self.emergency_stop_latched:
            self.fsm_state = "REPOSO"
            self.target_servo_angle = 90.0
            self.servo_angle_deg = 90.0
            if any(not p.is_routed for p in self.packages):
                self.fsm_state = "TRANSPORTE"
            return True
        return False

    def update_physics_and_firmware(self, dt: float):
        if self.emergency_stop_latched:
            self.fsm_state = "EMERGENCIA"
            self.motor_pwm = 0
            self.led_pwm = 0
            self.belt_speed_mps = 0.0
            return

        # FSM
        if self.fsm_state == "INIT":
            self.fsm_state = "REPOSO"

        elif self.fsm_state == "REPOSO":
            self.target_servo_angle = 90.0
            if any(not p.is_routed for p in self.packages):
                self.fsm_state = "TRANSPORTE"

        elif self.fsm_state == "TRANSPORTE":
            piece_in_zone = None
            for p in self.packages:
                if not p.is_inspected and (self.SENSOR_X_M - 0.10 <= p.pos_x_m <= self.SENSOR_X_M + 0.02):
                    piece_in_zone = p
                    break

            if piece_in_zone:
                self.fsm_state = "INSPECCION"
                self.inspect_timer = 0.0
                self.current_piece_under_inspection = piece_in_zone

        elif self.fsm_state == "INSPECCION":
            self.inspect_timer += dt
            if self.inspect_timer >= 1.0:
                target_pkg = self.current_piece_under_inspection
                if target_pkg:
                    target_pkg.is_inspected = True
                    qual = target_pkg.quality
                    if qual == "CONFORME":
                        self.target_servo_angle = 90.0
                        target_pkg.routed_chute = 2
                        self.count_conforme += 1
                    elif qual == "RETRABAJO":
                        self.target_servo_angle = 45.0
                        target_pkg.routed_chute = 1
                        self.count_retrabajo += 1
                    else:
                        self.target_servo_angle = 135.0
                        target_pkg.routed_chute = 3
                        self.count_rechazo += 1
                    
                    self.total_processed += 1
                    target_pkg.is_diverting = True

                self.fsm_state = "DESVIO"
                self.divert_timer = 0.0

        elif self.fsm_state == "DESVIO":
            self.divert_timer += dt
            if self.divert_timer >= 1.6:
                self.target_servo_angle = 90.0
                if self.current_piece_under_inspection:
                    self.current_piece_under_inspection.is_routed = True
                    self.current_piece_under_inspection = None
                
                self.fsm_state = "TRANSPORTE" if any(not p.is_routed for p in self.packages) else "REPOSO"

        # Lazos de Control
        # Lazo 1: Velocidad
        if self.fsm_state == "TRANSPORTE":
            if self.ultrasonic_dist_cm <= self.dist_critica_stop:
                self.motor_pwm = 0
            elif self.ultrasonic_dist_cm >= self.sp_dist_nominal:
                self.motor_pwm = 200
            else:
                factor = (self.ultrasonic_dist_cm - self.dist_critica_stop) / (self.sp_dist_nominal - self.dist_critica_stop)
                self.motor_pwm = int(factor * 200)
                self.motor_pwm = max(0, min(200, self.motor_pwm))
        elif self.fsm_state == "DESVIO":
            self.motor_pwm = 160
        elif self.fsm_state == "INSPECCION":
            self.motor_pwm = 0
        else:
            self.motor_pwm = 0

        # Lazo 2: Servomotor SG90
        servo_speed = 320.0
        angle_diff = self.target_servo_angle - self.servo_angle_deg
        max_step = servo_speed * dt
        if abs(angle_diff) <= max_step:
            self.servo_angle_deg = self.target_servo_angle
        else:
            self.servo_angle_deg += math.copysign(max_step, angle_diff)

        # Lazo 3: Iluminación
        self.error_lux = self.sp_lux_pct - self.total_lux_pct
        if self.fsm_state in ["TRANSPORTE", "INSPECCION"]:
            calc_pwm = self.base_light_pwm + (self.kp_light * self.error_lux * 2.55)
            self.led_pwm = int(max(0, min(255, calc_pwm)))
        else:
            self.led_pwm = 25

        # Dinámica Física Continua
        target_v = (self.motor_pwm / 255.0) * self.MAX_SPEED_MPS
        alpha_m = math.exp(-dt / self.MOTOR_TAU)
        self.belt_speed_mps = alpha_m * self.belt_speed_mps + (1 - alpha_m) * target_v

        nearest_dist_cm = 100.0
        for p in self.packages:
            if not p.is_routed:
                p.pos_x_m += self.belt_speed_mps * dt
                if p.pos_x_m >= 0.96:
                    p.is_routed = True

                dist_pkg_cm = (self.SENSOR_X_M - p.pos_x_m) * 100.0
                if 0.0 < dist_pkg_cm < nearest_dist_cm and not p.is_inspected:
                    nearest_dist_cm = dist_pkg_cm
            else:
                p.chute_anim_progress += dt * 1.6

        if nearest_dist_cm < 95.0:
            self.ultrasonic_dist_cm = max(2.0, nearest_dist_cm + random.gauss(0, 0.2))
        else:
            self.ultrasonic_dist_cm = 100.0 + random.gauss(0, 0.3)

        led_lux_contrib = (self.led_pwm / 255.0) * 60.0
        target_lux = self.ambient_lux_pct + led_lux_contrib + random.gauss(0, 0.15)
        alpha_ldr = math.exp(-dt / self.LDR_TAU)
        self.total_lux_pct = alpha_ldr * self.total_lux_pct + (1 - alpha_ldr) * target_lux
        self.total_lux_pct = max(0.0, min(100.0, self.total_lux_pct))

        self.packages = [p for p in self.packages if p.chute_anim_progress < 1.0]

        now = time.time()
        if self.auto_feed_active and not self.emergency_stop_latched:
            if now - self.last_feed_time >= 3.2:
                self.last_feed_time = now
                self.spawn_package()


# =====================================================================================
# INTERFAZ GRÁFICA MULTI-PESTAÑA SCADA (TEMA CLARO / INGENIERÍA)
# =====================================================================================

class DigitalTwinApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("SCADA INDUSTRIAL - ESTACIÓN LOGÍSTICA DE CLASIFICACIÓN [ANSI / ISA-5.1]")
        self.root.geometry("1380x900")
        self.root.minsize(1240, 800)
        # Fondo Claro de Ingeniería (Gris muy tenue / Blanco técnico)
        self.root.configure(bg="#ECEFF1")

        self.plant = VirtualIndustrialPlant()
        
        self.max_hist = 120
        self.time_hist = []
        self.dist_hist = []
        self.motor_pwm_hist = []
        self.lux_hist = []
        self.ambient_hist = []
        self.led_pwm_hist = []
        self.sim_time = 0.0

        self.roller_angle = 0.0
        self.sonar_wave_phase = 0.0
        self.alarm_blink_state = False
        self.last_blink_time = 0.0

        self.setup_styles()
        self.create_header()
        self.create_tab_system()
        
        self.last_update_time = time.time()
        self.animate()

    def setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        
        style.configure("TFrame", background="#ECEFF1")
        style.configure("Card.TFrame", background="#FFFFFF", relief="solid")
        
        # Pestañas en tema claro
        style.configure("TNotebook", background="#ECEFF1", borderwidth=1)
        style.configure("TNotebook.Tab", background="#CFD8DC", foreground="#37474F", 
                        padding=[16, 6], font=("Segoe UI", 9, "bold"), borderwidth=1)
        style.map("TNotebook.Tab",
                  background=[("selected", "#FFFFFF"), ("active", "#E0E0E0")],
                  foreground=[("selected", "#0D47A1"), ("active", "#111111")])

    def create_header(self):
        header_frame = tk.Frame(self.root, bg="#FFFFFF", height=55, padx=16, pady=8, bd=1, relief="solid")
        header_frame.pack(fill="x", side="top")

        title_box = tk.Frame(header_frame, bg="#FFFFFF")
        title_box.pack(side="left")
        
        lbl_brand = tk.Label(title_box, text="SISTEMA SCADA INDUSTRIAL - ARDUINO UNO R3", 
                             font=("Segoe UI", 12, "bold"), fg="#1A237E", bg="#FFFFFF")
        lbl_brand.pack(anchor="w")
        lbl_sub = tk.Label(title_box, text="Estación Logística de Selección, Clasificación y Enrutamiento | P&ID ANSI/ISA-5.1", 
                           font=("Segoe UI", 8), fg="#546E7A", bg="#FFFFFF")
        lbl_sub.pack(anchor="w")

        kpi_box = tk.Frame(header_frame, bg="#FFFFFF")
        kpi_box.pack(side="right")

        self.kpi_fsm_badge = tk.Label(kpi_box, text="FSM: REPOSO", font=("Segoe UI", 9, "bold"), 
                                      fg="#37474F", bg="#E0E0E0", padx=10, pady=3, bd=1, relief="solid")
        self.kpi_fsm_badge.pack(side="left", padx=4)

        self.kpi_estop_badge = tk.Label(kpi_box, text="E-STOP: OK (INT0)", font=("Segoe UI", 9, "bold"), 
                                        fg="#1B5E20", bg="#C8E6C9", padx=10, pady=3, bd=1, relief="solid")
        self.kpi_estop_badge.pack(side="left", padx=4)

    def create_tab_system(self):
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill="both", expand=True, padx=8, pady=8)

        # Tab 1: SCADA Principal
        self.tab_scada = tk.Frame(self.notebook, bg="#ECEFF1")
        self.notebook.add(self.tab_scada, text="  [1] PLANTA SCADA EN VIVO  ")
        self.build_tab_scada(self.tab_scada)

        # Tab 2: Diagrama P&ID ANSI / ISA-5.1
        self.tab_pid = tk.Frame(self.notebook, bg="#ECEFF1")
        self.notebook.add(self.tab_pid, text="  [2] DIAGRAMA P&ID (NORMA ANSI / ISA-5.1)  ")
        self.build_tab_pid(self.tab_pid)

        # Tab 3: Arquitectura y Componentes Señalizados
        self.tab_hw = tk.Frame(self.notebook, bg="#ECEFF1")
        self.notebook.add(self.tab_hw, text="  [3] COMPONENTES Y ASIGNACIÓN HARDWARE  ")
        self.build_tab_hardware(self.tab_hw)

    # =================================================================================
    # PESTAÑA 1: PLANTA SCADA EN VIVO
    # =================================================================================
    def build_tab_scada(self, parent):
        main_container = tk.Frame(parent, bg="#ECEFF1")
        main_container.pack(fill="both", expand=True, pady=4)

        left_panel = tk.Frame(main_container, bg="#ECEFF1")
        left_panel.pack(side="left", fill="both", expand=True, padx=(0, 6))

        # Canvas SCADA Card (Fondo blanco de ingeniería)
        canvas_card = tk.Frame(left_panel, bg="#FFFFFF", bd=1, relief="solid")
        canvas_card.pack(fill="both", expand=True)
        
        lbl_canvas_title = tk.Label(canvas_card, text="MÍMICO SCADA DE PROCESO (Física Continua 2D)", 
                                    font=("Segoe UI", 9, "bold"), fg="#263238", bg="#FFFFFF", pady=4)
        lbl_canvas_title.pack(anchor="w", padx=8)

        self.canvas = tk.Canvas(canvas_card, bg="#FAFAFA", highlightthickness=1, highlightbackground="#CFD8DC", height=380)
        self.canvas.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        # Panel de Mandos
        control_card = tk.Frame(left_panel, bg="#FFFFFF", bd=1, relief="solid", pady=6, padx=8)
        control_card.pack(fill="x", pady=(6, 0))
        self.create_control_panel(control_card)

        # Lado Derecho
        right_panel = tk.Frame(main_container, bg="#ECEFF1", width=440)
        right_panel.pack(side="right", fill="both", expand=False)
        right_panel.pack_propagate(False)

        self.create_virtual_arduino_card(right_panel)
        self.create_charts_card(right_panel)

    def create_virtual_arduino_card(self, parent):
        card = tk.Frame(parent, bg="#FFFFFF", bd=1, relief="solid", padx=10, pady=8)
        card.pack(fill="x", pady=(0, 6))

        lbl_title = tk.Label(card, text="HMI LOCAL (LCD 16x2 / SEÑALIZACIÓN)", 
                             font=("Segoe UI", 8, "bold"), fg="#263238", bg="#FFFFFF")
        lbl_title.pack(anchor="w", pady=(0, 4))

        # Display LCD 16x2 Clásico Verde/Negro
        lcd_bezel = tk.Frame(card, bg="#78909C", padx=3, pady=3, bd=1, relief="solid")
        lcd_bezel.pack(fill="x")

        lcd_screen = tk.Frame(lcd_bezel, bg="#9CCC65", padx=6, pady=4)
        lcd_screen.pack(fill="x")

        self.lcd_line0 = tk.Label(lcd_screen, text="REPOSO|D: 100cm ", font=("Courier New", 11, "bold"), fg="#1B5E20", bg="#9CCC65", anchor="w")
        self.lcd_line0.pack(fill="x")
        self.lcd_line1 = tk.Label(lcd_screen, text="M:  0 L: 40% S: 90", font=("Courier New", 11, "bold"), fg="#1B5E20", bg="#9CCC65", anchor="w")
        self.lcd_line1.pack(fill="x")

        # Fila de LEDs y Buzzer
        led_row = tk.Frame(card, bg="#FFFFFF", pady=6)
        led_row.pack(fill="x")

        f_g = tk.Frame(led_row, bg="#FFFFFF"); f_g.pack(side="left", expand=True)
        self.led_green_canvas = tk.Canvas(f_g, width=20, height=20, bg="#FFFFFF", highlightthickness=0); self.led_green_canvas.pack()
        tk.Label(f_g, text="RUN (D11)", font=("Segoe UI", 7), fg="#546E7A", bg="#FFFFFF").pack()

        f_y = tk.Frame(led_row, bg="#FFFFFF"); f_y.pack(side="left", expand=True)
        self.led_yellow_canvas = tk.Canvas(f_y, width=20, height=20, bg="#FFFFFF", highlightthickness=0); self.led_yellow_canvas.pack()
        tk.Label(f_y, text="INSP (D12)", font=("Segoe UI", 7), fg="#546E7A", bg="#FFFFFF").pack()

        f_r = tk.Frame(led_row, bg="#FFFFFF"); f_r.pack(side="left", expand=True)
        self.led_red_canvas = tk.Canvas(f_r, width=20, height=20, bg="#FFFFFF", highlightthickness=0); self.led_red_canvas.pack()
        tk.Label(f_r, text="ALARM (D13)", font=("Segoe UI", 7), fg="#546E7A", bg="#FFFFFF").pack()

        f_b = tk.Frame(led_row, bg="#FFFFFF"); f_b.pack(side="left", expand=True)
        self.buzzer_canvas = tk.Canvas(f_b, width=20, height=20, bg="#FFFFFF", highlightthickness=0); self.buzzer_canvas.pack()
        tk.Label(f_b, text="BUZZER (D10)", font=("Segoe UI", 7), fg="#546E7A", bg="#FFFFFF").pack()

    def create_charts_card(self, parent):
        card = tk.Frame(parent, bg="#FFFFFF", bd=1, relief="solid", padx=6, pady=6)
        card.pack(fill="both", expand=True)

        lbl_title = tk.Label(card, text="REGISTRO DE VARIABLES DE CONTROL", 
                             font=("Segoe UI", 8, "bold"), fg="#263238", bg="#FFFFFF")
        lbl_title.pack(anchor="w", pady=(0, 2))

        # Matplotlib en Fondo Blanco Clásico
        self.fig = Figure(figsize=(4.2, 3.3), facecolor="#FFFFFF")
        self.fig.subplots_adjust(left=0.14, right=0.92, top=0.90, bottom=0.15, hspace=0.40)
        
        self.ax1 = self.fig.add_subplot(2, 1, 1)
        self.ax1.set_facecolor("#FAFAFA")
        self.ax1.tick_params(colors="#37474F", labelsize=7)
        self.ax1.grid(True, color="#CFD8DC", linestyle="-", alpha=0.8)
        self.ax1.set_title("Lazo 1: Distancia PV (cm) / PWM", color="#263238", fontsize=8, pad=2)
        self.line_dist, = self.ax1.plot([], [], color="#0D47A1", lw=1.5, label="Dist (cm)")
        self.line_pwm_m, = self.ax1.plot([], [], color="#E65100", lw=1.2, linestyle="--", label="PWM Motor")
        self.ax1.set_ylim(-5, 260)
        self.ax1.legend(loc="upper right", fontsize=6, facecolor="#FFFFFF", edgecolor="#B0BEC5", labelcolor="#263238")

        self.ax2 = self.fig.add_subplot(2, 1, 2)
        self.ax2.set_facecolor("#FAFAFA")
        self.ax2.tick_params(colors="#37474F", labelsize=7)
        self.ax2.grid(True, color="#CFD8DC", linestyle="-", alpha=0.8)
        self.ax2.set_title("Lazo 3: Iluminancia PV (%) / PWM LED", color="#263238", fontsize=8, pad=2)
        self.line_lux, = self.ax2.plot([], [], color="#2E7D32", lw=1.5, label="Lux PV (%)")
        self.line_amb, = self.ax2.plot([], [], color="#78909C", lw=1.0, linestyle=":", label="Ambiente")
        self.line_pwm_led, = self.ax2.plot([], [], color="#BF360C", lw=1.2, linestyle="--", label="PWM LED")
        self.ax2.set_ylim(-5, 260)
        self.ax2.legend(loc="upper right", fontsize=6, facecolor="#FFFFFF", edgecolor="#B0BEC5", labelcolor="#263238")

        self.chart_canvas = FigureCanvasTkAgg(self.fig, master=card)
        self.chart_canvas.get_tk_widget().pack(fill="both", expand=True)

    def create_control_panel(self, parent):
        col_box = tk.Frame(parent, bg="#FFFFFF")
        col_box.pack(fill="x")

        # Marcha
        c1 = tk.LabelFrame(col_box, text=" Mando ", font=("Segoe UI", 7, "bold"), fg="#37474F", bg="#FFFFFF", padx=4, pady=3)
        c1.pack(side="left", fill="both", expand=True, padx=2)
        tk.Button(c1, text="START", bg="#2E7D32", fg="white", font=("Segoe UI", 8, "bold"), bd=1, relief="raised",
                  command=lambda: setattr(self.plant, 'fsm_state', 'TRANSPORTE') if self.plant.fsm_state == 'REPOSO' else None).pack(fill="x", pady=1)
        tk.Button(c1, text="STOP", bg="#546E7A", fg="white", font=("Segoe UI", 8, "bold"), bd=1, relief="raised",
                  command=lambda: setattr(self.plant, 'fsm_state', 'REPOSO')).pack(fill="x", pady=1)
        tk.Button(c1, text="RESET", bg="#0277BD", fg="white", font=("Segoe UI", 7, "bold"), bd=1, relief="raised",
                  command=self.on_reset_clicked).pack(fill="x", pady=1)

        # Alimentador
        c2 = tk.LabelFrame(col_box, text=" Alimentación ", font=("Segoe UI", 7, "bold"), fg="#37474F", bg="#FFFFFF", padx=4, pady=3)
        c2.pack(side="left", fill="both", expand=True, padx=2)
        tk.Button(c2, text="+ Conforme (90°)", bg="#388E3C", fg="white", font=("Segoe UI", 7), bd=1, relief="raised",
                  command=lambda: self.plant.spawn_package("CONFORME")).pack(fill="x", pady=1)
        tk.Button(c2, text="+ Retrabajo (45°)", bg="#F57C00", fg="white", font=("Segoe UI", 7), bd=1, relief="raised",
                  command=lambda: self.plant.spawn_package("RETRABAJO")).pack(fill="x", pady=1)
        tk.Button(c2, text="+ Rechazo (135°)", bg="#D32F2F", fg="white", font=("Segoe UI", 7), bd=1, relief="raised",
                  command=lambda: self.plant.spawn_package("RECHAZO")).pack(fill="x", pady=1)

        self.var_autofeed = tk.BooleanVar(value=True)
        tk.Checkbutton(c2, text="Auto (3.2s)", variable=self.var_autofeed, bg="#FFFFFF", fg="#263238", 
                       selectcolor="#ECEFF1", activebackground="#FFFFFF", font=("Segoe UI", 7),
                       command=lambda: setattr(self.plant, 'auto_feed_active', self.var_autofeed.get())).pack(anchor="w")

        # Perturbación
        c3 = tk.LabelFrame(col_box, text=" Luz Externa ", font=("Segoe UI", 7, "bold"), fg="#37474F", bg="#FFFFFF", padx=4, pady=3)
        c3.pack(side="left", fill="both", expand=True, padx=2)
        self.lbl_lux_val = tk.Label(c3, text="Ambiente: 40%", font=("Segoe UI", 7, "bold"), fg="#E65100", bg="#FFFFFF")
        self.lbl_lux_val.pack(anchor="w")

        self.slider_lux = tk.Scale(c3, from_=0, to=100, orient="horizontal", bg="#FFFFFF", fg="#263238", highlightthickness=0,
                                   troughcolor="#CFD8DC", activebackground="#0277BD", command=self.on_ambient_lux_changed)
        self.slider_lux.set(40)
        self.slider_lux.pack(fill="x", pady=1)

        f_lux_btns = tk.Frame(c3, bg="#FFFFFF"); f_lux_btns.pack(fill="x")
        tk.Button(f_lux_btns, text="Sombra (15%)", bg="#ECEFF1", fg="#263238", font=("Segoe UI", 6), bd=1, relief="raised",
                  command=lambda: self.set_ambient_lux(15)).pack(side="left", fill="x", expand=True, padx=1)
        tk.Button(f_lux_btns, text="Sol (80%)", bg="#ECEFF1", fg="#263238", font=("Segoe UI", 6), bd=1, relief="raised",
                  command=lambda: self.set_ambient_lux(80)).pack(side="left", fill="x", expand=True, padx=1)

        # Emergencia
        c4 = tk.LabelFrame(col_box, text=" Seguridad ", font=("Segoe UI", 7, "bold"), fg="#C62828", bg="#FFFFFF", padx=4, pady=3)
        c4.pack(side="left", fill="both", expand=True, padx=2)
        self.btn_estop = tk.Button(c4, text="PARADA DE\nEMERGENCIA\n(INT0 D2)", bg="#C62828", fg="white", 
                                   font=("Segoe UI", 8, "bold"), bd=2, relief="raised", cursor="hand2",
                                   command=self.on_estop_clicked)
        self.btn_estop.pack(fill="both", expand=True, pady=1)

    def on_ambient_lux_changed(self, val):
        v = float(val)
        self.plant.ambient_lux_pct = v
        self.lbl_lux_val.config(text=f"Ambiente: {int(v)}%")

    def set_ambient_lux(self, val):
        self.slider_lux.set(val)
        self.on_ambient_lux_changed(val)

    def on_estop_clicked(self):
        new_state = not self.plant.emergency_stop_latched
        self.plant.trigger_emergency(new_state)
        if new_state:
            self.btn_estop.config(text="ENCLAVADO\n(Libere y Rearme)", bg="#5C0000")
        else:
            self.btn_estop.config(text="PARADA DE\nEMERGENCIA\n(INT0 D2)", bg="#C62828")

    def on_reset_clicked(self):
        if self.plant.emergency_stop_latched:
            self.plant.emergency_stop_latched = False
            self.btn_estop.config(text="PARADA DE\nEMERGENCIA\n(INT0 D2)", bg="#C62828")
        
        ok = self.plant.reset_emergency()
        if ok:
            self.kpi_estop_badge.config(text="E-STOP: OK (INT0)", bg="#C8E6C9", fg="#1B5E20")

    # =================================================================================
    # PESTAÑA 2: DIAGRAMA P&ID BAJO NORMA ANSI / ISA-5.1 (PLANO BLANCO)
    # =================================================================================
    def build_tab_pid(self, parent):
        scroll_frame = tk.Frame(parent, bg="#ECEFF1")
        scroll_frame.pack(fill="both", expand=True, padx=10, pady=8)

        lbl_h = tk.Label(scroll_frame, text="DIAGRAMA DE TUBERÍAS E INSTRUMENTACIÓN (P&ID) - NORMA ANSI / ISA-5.1",
                         font=("Segoe UI", 11, "bold"), fg="#1A237E", bg="#ECEFF1")
        lbl_h.pack(anchor="w", pady=(0, 6))

        canvas_pid = tk.Canvas(scroll_frame, bg="#FFFFFF", highlightthickness=1, highlightbackground="#B0BEC5", height=660)
        canvas_pid.pack(fill="both", expand=True)

        self.draw_ansi_isa_pid(canvas_pid)

    def draw_ansi_isa_pid(self, c: tk.Canvas):
        c.delete("all")
        
        # Cuadrícula técnica gris muy tenue
        for x in range(0, 1300, 50):
            c.create_line(x, 0, x, 660, fill="#F0F4F8", width=1)
        for y in range(0, 660, 50):
            c.create_line(0, y, 1300, y, fill="#F0F4F8", width=1)

        # Cajetín de Ingeniería ANSI en fondo blanco
        c.create_rectangle(920, 15, 1260, 95, fill="#FFFFFF", outline="#37474F", width=1.5)
        c.create_text(935, 30, text="PROYECTO: ESTACIÓN DE CLASIFICACIÓN", fill="#111111", font=("Segoe UI", 8, "bold"), anchor="w")
        c.create_text(935, 48, text="NORMA: ANSI / ISA-5.1 (P&ID)", fill="#37474F", font=("Segoe UI", 8), anchor="w")
        c.create_text(935, 65, text="CONTROLADOR: ARDUINO UNO R3 (PLC / DCS)", fill="#0D47A1", font=("Segoe UI", 8, "bold"), anchor="w")
        c.create_text(935, 82, text="ESC: S/E | REV: 2.0 | TAGS: ISA-5.1", fill="#546E7A", font=("Segoe UI", 7), anchor="w")

        # Faja Transportadora CV-100 (Línea gruesa negra de proceso)
        c.create_line(60, 320, 1080, 320, fill="#111111", width=4)
        c.create_line(60, 370, 1080, 370, fill="#111111", width=4)
        c.create_text(90, 345, text="FAJA CV-100", fill="#111111", font=("Segoe UI", 8, "bold"))

        c.create_oval(45, 320, 75, 370, fill="#CFD8DC", outline="#111111", width=2)
        c.create_oval(1065, 320, 1095, 370, fill="#CFD8DC", outline="#111111", width=2)

        # 1. HS-100 & I-100 (E-Stop)
        self.draw_isa_bubble(c, 120, 120, "HS", "100", is_shared=False, has_line=True)
        c.create_text(120, 75, text="Pulsador E-Stop\n(Pin 2 / INT0)", fill="#37474F", font=("Segoe UI", 7), justify="center")

        c.create_line(120, 145, 120, 190, fill="#111111", width=1.5, dash=(4, 3))

        c.create_polygon(120, 190, 145, 215, 120, 240, 95, 215, fill="#FFFFFF", outline="#C62828", width=2)
        c.create_text(120, 215, text="I\n100", fill="#C62828", font=("Segoe UI", 7, "bold"), justify="center")

        c.create_line(145, 215, 260, 215, fill="#C62828", width=1.5, dash=(4, 3), arrow="last")
        c.create_text(200, 205, text="Corte Atómico ISR", fill="#C62828", font=("Segoe UI", 7, "bold"))

        # 2. Lazo 1: ZT-101 -> ZIC-101 -> SC-101 -> M-101
        self.draw_isa_bubble(c, 740, 220, "ZT", "101", is_shared=False, has_line=False)
        c.create_text(740, 175, text="Sensor HC-SR04\n(Trig D8 / Echo D9)", fill="#37474F", font=("Segoe UI", 7), justify="center")

        c.create_line(740, 245, 740, 320, fill="#0D47A1", width=2, arrow="last")

        c.create_line(715, 220, 520, 220, fill="#111111", width=1.5, dash=(4, 3), arrow="last")
        c.create_text(620, 210, text="Distancia PV: d(t)", fill="#0D47A1", font=("Segoe UI", 7, "bold"))

        self.draw_isa_bubble(c, 480, 220, "ZIC", "101", is_shared=True, has_line=True)
        c.create_text(480, 165, text="Controlador Lazo 1\n(SP = 25 cm | Kp = 10)", fill="#37474F", font=("Segoe UI", 7), justify="center")

        c.create_line(440, 220, 310, 220, fill="#111111", width=1.5, dash=(4, 3))
        c.create_line(310, 220, 310, 320, fill="#111111", width=1.5, dash=(4, 3), arrow="last")
        c.create_text(360, 210, text="PWM ENA (D3)", fill="#E65100", font=("Segoe UI", 7, "bold"))

        c.create_rectangle(270, 325, 350, 365, fill="#ECEFF1", outline="#111111", width=1.5)
        c.create_text(310, 345, text="SC-101\n(L298N)", fill="#111111", font=("Segoe UI", 7, "bold"), justify="center")

        c.create_oval(180, 325, 230, 365, fill="#ECEFF1", outline="#111111", width=1.5)
        c.create_text(205, 345, text="M\n101", fill="#111111", font=("Segoe UI", 7, "bold"), justify="center")
        c.create_line(230, 345, 270, 345, fill="#111111", width=2)
        c.create_line(180, 345, 75, 345, fill="#111111", width=2)

        # 3. Lazo 3: QE-103 -> QT-103 -> QIC-103 -> QY-103 -> QL-103
        self.draw_isa_bubble(c, 740, 470, "QE", "103", is_shared=False, has_line=False)
        c.create_text(740, 515, text="LDR (Pin A0)\nDivisor 10k", fill="#37474F", font=("Segoe UI", 7), justify="center")

        c.create_line(740, 370, 740, 445, fill="#E65100", width=1.5, arrow="first")

        c.create_line(715, 470, 640, 470, fill="#111111", width=1.5, dash=(4, 3), arrow="last")
        self.draw_isa_bubble(c, 600, 470, "QT", "103", is_shared=False, has_line=False)

        c.create_line(560, 470, 520, 470, fill="#111111", width=1.5, dash=(4, 3), arrow="last")
        self.draw_isa_bubble(c, 480, 470, "QIC", "103", is_shared=True, has_line=True)
        c.create_text(480, 525, text="Controlador Lazo 3\n(SP = 70% | Kp = 2.5)", fill="#37474F", font=("Segoe UI", 7), justify="center")

        c.create_line(480, 430, 480, 390, fill="#111111", width=1.5, dash=(4, 3))
        c.create_line(480, 390, 830, 390, fill="#111111", width=1.5, dash=(4, 3))
        c.create_line(830, 390, 830, 445, fill="#111111", width=1.5, dash=(4, 3), arrow="last")
        c.create_text(650, 400, text="PWM Luz (Pin 6)", fill="#BF360C", font=("Segoe UI", 7, "bold"))

        self.draw_isa_bubble(c, 830, 470, "QY", "103", is_shared=False, has_line=False)
        c.create_text(830, 515, text="Transistor TIP120\n(Pin 6 PWM)", fill="#37474F", font=("Segoe UI", 7), justify="center")

        c.create_line(855, 470, 910, 470, fill="#111111", width=1.5, arrow="last")
        c.create_rectangle(910, 450, 970, 490, fill="#ECEFF1", outline="#111111", width=1.5)
        c.create_text(940, 470, text="QL-103\n(LEDs)", fill="#111111", font=("Segoe UI", 7, "bold"), justify="center")

        c.create_line(940, 450, 940, 370, fill="#E65100", width=2, arrow="last")

        # 4. Lazo 2: ZIC-102 -> ZY-102 -> XV-102
        self.draw_isa_bubble(c, 880, 220, "ZIC", "102", is_shared=True, has_line=True)
        c.create_text(880, 165, text="Controlador Servo (D5)\n(45° / 90° / 135°)", fill="#37474F", font=("Segoe UI", 7), justify="center")

        c.create_line(920, 220, 1000, 220, fill="#111111", width=1.5, dash=(4, 3))
        c.create_line(1000, 220, 1000, 260, fill="#111111", width=1.5, dash=(4, 3), arrow="last")

        self.draw_isa_bubble(c, 1000, 285, "ZY", "102", is_shared=False, has_line=False)
        c.create_text(1060, 285, text="Micro Servo\nSG90 (Pin 5)", fill="#37474F", font=("Segoe UI", 7), anchor="w")

        c.create_line(1000, 310, 1000, 335, fill="#111111", width=2)
        
        c.create_polygon(985, 335, 1015, 335, 1000, 355, fill="#CFD8DC", outline="#111111", width=1.5)
        c.create_polygon(985, 355, 1015, 355, 1000, 335, fill="#CFD8DC", outline="#111111", width=1.5)
        c.create_text(1000, 375, text="XV-102", fill="#111111", font=("Segoe UI", 8, "bold"))

        # 3 Tolvas
        c.create_line(1015, 335, 1140, 270, fill="#111111", width=2.5, arrow="last")
        c.create_text(1185, 270, text="TOLVA RETRABAJO\n(45°)", fill="#E65100", font=("Segoe UI", 7, "bold"))

        c.create_line(1015, 345, 1140, 345, fill="#111111", width=2.5, arrow="last")
        c.create_text(1185, 345, text="TOLVA CONFORME\n(90° DIRECTO)", fill="#2E7D32", font=("Segoe UI", 7, "bold"))

        c.create_line(1015, 355, 1140, 420, fill="#111111", width=2.5, arrow="last")
        c.create_text(1185, 420, text="TOLVA RECHAZO\n(135°)", fill="#C62828", font=("Segoe UI", 7, "bold"))

        # Leyenda ANSI / ISA-5.1
        y_leg = 550
        c.create_rectangle(30, y_leg, 1260, y_leg + 95, fill="#FFFFFF", outline="#37474F", width=1)
        c.create_text(45, y_leg + 14, text="LEYENDA DE INSTRUMENTACIÓN (NORMA ANSI / ISA-5.1)", fill="#111111", font=("Segoe UI", 8, "bold"), anchor="w")

        legend_items = [
            ("◯ (Círculo Simple)", "Instrumento discreto montado en campo (Sensor / Transmisor / Actuador local)."),
            ("🔲 + ◯ (Cuadrado con Círculo)", "Función de visualización / control compartido en Microcontrolador / DCS / PLC."),
            ("Línea Continua Horizontal", "Ubicación principal accesible al operador (HMI / Panel frontal de control)."),
            ("Línea de Trazos (─ ─ ─)", "Señal eléctrica / digital estándar de instrumentación (PWM / ADC / GPIO)."),
            ("Rombo de Seguridad [ I ]", "Enclavamiento lógico / Interrupción externa de seguridad (Safety Shutdown INT0).")
        ]

        for idx, (sym, desc) in enumerate(legend_items):
            col_x = 45 if idx < 3 else 650
            row_y = y_leg + 32 + (idx % 3) * 18
            c.create_text(col_x, row_y, text=f"• {sym}: ", fill="#0D47A1", font=("Segoe UI", 7, "bold"), anchor="w")
            c.create_text(col_x + 190, row_y, text=desc, fill="#37474F", font=("Segoe UI", 7), anchor="w")

    def draw_isa_bubble(self, c: tk.Canvas, cx: int, cy: int, letters: str, loop_num: str, is_shared: bool, has_line: bool):
        r = 24
        if is_shared:
            c.create_rectangle(cx - r - 3, cy - r - 3, cx + r + 3, cy + r + 3, fill="#ECEFF1", outline="#111111", width=1.5)
        
        c.create_oval(cx - r, cy - r, cx + r, cy + r, fill="#FFFFFF", outline="#111111", width=1.5)
        
        if has_line:
            c.create_line(cx - r, cy, cx + r, cy, fill="#111111", width=1.2)

        c.create_text(cx, cy - 8, text=letters, fill="#111111", font=("Segoe UI", 8, "bold"))
        c.create_text(cx, cy + 9, text=loop_num, fill="#111111", font=("Segoe UI", 8))

    # =================================================================================
    # PESTAÑA 3: ARQUITECTURA DE HARDWARE Y ASIGNACIÓN DE PINES
    # =================================================================================
    def build_tab_hardware(self, parent):
        scroll_frame = tk.Frame(parent, bg="#ECEFF1")
        scroll_frame.pack(fill="both", expand=True, padx=10, pady=8)

        lbl_h = tk.Label(scroll_frame, text="ESQUEMA DE COMPONENTES SEÑALIZADOS Y ASIGNACIÓN ESTRICTA DE PINES",
                         font=("Segoe UI", 11, "bold"), fg="#1A237E", bg="#ECEFF1")
        lbl_h.pack(anchor="w", pady=(0, 6))

        canvas_hw = tk.Canvas(scroll_frame, bg="#FFFFFF", highlightthickness=1, highlightbackground="#B0BEC5", height=380)
        canvas_hw.pack(fill="x")

        self.draw_hardware_callouts(canvas_hw)

        table_frame = tk.Frame(scroll_frame, bg="#FFFFFF", bd=1, relief="solid", padx=8, pady=6)
        table_frame.pack(fill="both", expand=True, pady=(6, 0))

        lbl_tbl = tk.Label(table_frame, text="MATRIZ DE ASIGNACIÓN DE PINES Y RECURSOS (ARDUINO UNO R3 - ATmega328P)",
                           font=("Segoe UI", 8, "bold"), fg="#111111", bg="#FFFFFF")
        lbl_tbl.pack(anchor="w", pady=(0, 3))

        headers = ["Pin", "Periférico", "Tipo Señal", "Timer / Recurso", "Función Industrial"]
        cols_w = [10, 24, 16, 18, 48]

        header_row = tk.Frame(table_frame, bg="#CFD8DC")
        header_row.pack(fill="x")
        for h, w in zip(headers, cols_w):
            tk.Label(header_row, text=h, font=("Segoe UI", 7, "bold"), fg="#111111", bg="#CFD8DC", width=w, anchor="w", padx=4, pady=2).pack(side="left")

        pin_data = [
            ("D0 (RX) / D1 (TX)", "Telemetría Serial / BT HC-05", "UART", "USART Nativo", "Transmisión CSV/JSON a 9600 bps y comandos remotos."),
            ("D2 (INT0)", "Pulsador Parada Emergencia", "Digital In (Pull-up)", "Interrupción Externa", "Corte atómico e instantáneo de actuadores en <5 µs."),
            ("D3 (PWM)", "ENA Driver L298N (Motor DC)", "PWM Salida", "Timer 2 (OC2B)", "Lazo 1: Control proporcional de velocidad de la faja."),
            ("D4 / D7", "IN1 / IN2 Driver L298N", "GPIO Salida", "GPIO General", "Sentido de giro y marcha de faja transportadora."),
            ("D5 (PWM)", "Micro Servomotor SG90", "PPM / PWM", "Timer 0 / Servo.h", "Lazo 2: Enrutamiento angular a 3 tolvas (45°, 90°, 135°)."),
            ("D6 (PWM)", "Base Transistor NPN (LEDs)", "PWM Salida", "Timer 0 (OC0A)", "Lazo 3: Estabilización de iluminación de cámara."),
            ("D8 / D9", "Sensor HC-SR04 (Trig/Echo)", "Digital In/Out", "PCINT1", "Medición de tiempo de vuelo no bloqueante."),
            ("D10 / D11..D13", "Buzzer y LEDs Estado", "GPIO Salida", "GPIO General", "Señalización visual y sonora de estados de la FSM."),
            ("A0", "Fotorresistencia LDR", "Analógico In", "ADC Canal 0 (10-bit)", "Lazo 3: Sensor de iluminancia relativa (0-100%)."),
            ("A1", "Potenciómetro de Calibración", "Analógico In", "ADC Canal 1 (10-bit)", "Ajuste manual de Setpoint de iluminación en vivo."),
            ("A4 (SDA) / A5 (SCL)", "Pantalla LCD 16x2 I2C", "Bus I2C (0x27)", "TWI Hardware", "Visualización de HMI local (Estado, Dist, PWM, Lux).")
        ]

        for i, row in enumerate(pin_data):
            bg_col = "#FFFFFF" if i % 2 == 0 else "#F5F5F5"
            r_frame = tk.Frame(table_frame, bg=bg_col)
            r_frame.pack(fill="x")
            for val, w in zip(row, cols_w):
                tk.Label(r_frame, text=val, font=("Segoe UI", 7), fg="#263238", bg=bg_col, width=w, anchor="w", padx=4, pady=1).pack(side="left")

    def draw_hardware_callouts(self, c: tk.Canvas):
        c.delete("all")
        bx0 = 160
        by0 = 150
        bw = 880
        bh = 65
        bx1 = bx0 + bw
        by1 = by0 + bh

        c.create_rectangle(bx0 - 15, by0 - 8, bx1 + 15, by1 + 12, fill="#ECEFF1", outline="#37474F", width=2)
        c.create_rectangle(bx0, by0, bx1, by1, fill="#FAFAFA", outline="#111111", width=1.5)
        
        for i in range(25):
            rx = bx0 + i * 36
            c.create_line(rx, by0 + 2, rx, by1 - 2, fill="#CFD8DC", width=2)

        # Motor DC
        c.create_rectangle(bx0 - 90, by0 + 5, bx0 - 20, by1 - 5, fill="#ECEFF1", outline="#111111", width=1.5)
        c.create_text(bx0 - 55, by0 + 32, text="MOTOR DC\n+ L298N", fill="#111111", font=("Segoe UI", 7, "bold"), justify="center")

        # HC-SR04
        sx = bx0 + int(0.82 * bw)
        sy = by0 - 40
        c.create_rectangle(sx - 22, sy - 10, sx + 22, sy + 10, fill="#ECEFF1", outline="#111111", width=1.2)
        c.create_oval(sx - 16, sy - 7, sx - 4, sy + 7, fill="#B0BEC5")
        c.create_oval(sx + 4, sy - 7, sx + 16, sy + 7, fill="#B0BEC5")

        # Cámara + LDR
        c.create_polygon(sx - 30, sy + 20, sx + 30, sy + 20, sx + 40, by0, sx - 40, by0, fill="#ECEFF1", outline="#111111", width=1.2)
        c.create_oval(sx - 5, sy + 25, sx + 5, sy + 35, fill="#FFB74D")

        # Servo SG90
        gx = bx0 + int(0.90 * bw)
        gy = by0 + bh / 2
        c.create_rectangle(gx - 8, gy - 10, gx + 8, gy + 10, fill="#ECEFF1", outline="#C62828", width=1.2)
        c.create_line(gx, gy, gx + 30, gy - 20, fill="#C62828", width=3, capstyle="round")

        # Tolvas
        cx = bx1 + 15
        c.create_polygon(cx, by0 - 45, cx + 65, by0 - 65, cx + 65, by0 - 20, cx, by0 - 5, fill="#ECEFF1", outline="#111111")
        c.create_text(cx + 32, by0 - 42, text="RETRABAJO\n45°", fill="#E65100", font=("Segoe UI", 6, "bold"), justify="center")

        c.create_polygon(cx, by0 + 10, cx + 65, by0 + 10, cx + 65, by1 - 10, cx, by1 - 10, fill="#ECEFF1", outline="#111111")
        c.create_text(cx + 32, by0 + 28, text="CONFORME\n90°", fill="#2E7D32", font=("Segoe UI", 6, "bold"), justify="center")

        c.create_polygon(cx, by1 + 5, cx + 65, by1 + 20, cx + 65, by1 + 65, cx, by1 + 50, fill="#ECEFF1", outline="#111111")
        c.create_text(cx + 32, by1 + 35, text="RECHAZO\n135°", fill="#C62828", font=("Segoe UI", 6, "bold"), justify="center")

        # Arduino Uno
        ax = bx0 + 200
        ay = by1 + 35
        c.create_rectangle(ax, ay, ax + 200, ay + 95, fill="#E3F2FD", outline="#0D47A1", width=1.5)
        c.create_text(ax + 100, ay + 18, text="ARDUINO UNO R3", fill="#0D47A1", font=("Segoe UI", 8, "bold"))
        c.create_text(ax + 100, ay + 42, text="• FSM Multitarea (millis)\n• 3 Lazos Cerrados\n• ISR INT0 Pin 2", 
                      fill="#37474F", font=("Segoe UI", 7), justify="center")

        # LCD
        c.create_rectangle(ax + 220, ay + 12, ax + 340, ay + 75, fill="#9CCC65", outline="#37474F", width=1.5)
        c.create_text(ax + 280, ay + 43, text="LCD 16x2 I2C\n(A4/A5 - 0x27)", fill="#1B5E20", font=("Segoe UI", 7, "bold"), justify="center")

        # E-Stop
        c.create_oval(ax - 75, ay + 15, ax - 25, ay + 65, fill="#C62828", outline="#111111", width=1.5)
        c.create_text(ax - 50, ay + 40, text="E-STOP\n(Pin 2)", fill="white", font=("Segoe UI", 7, "bold"), justify="center")

        # Callouts
        callouts = [
            (bx0 - 55, by0 + 5, bx0 - 90, 40, "① MOTOR DC + L298N\nPin 3 (ENA) / Pin 4 (IN1) / Pin 7 (IN2)"),
            (bx0 + 200, by0, bx0 + 150, 30, "② ENTRADA DE PIEZAS\nAlimentador Automático / Faja"),
            (sx, sy - 10, sx - 80, 35, "③ SENSOR HC-SR04\nPin 8 (Trig) / Pin 9 (Echo)"),
            (sx + 10, sy + 25, sx + 110, 40, "④ CÁMARA LED + LDR\nPin 6 (PWM LED) / Pin A0 (LDR)"),
            (gx, gy - 10, gx + 70, 95, "⑤ SERVO SG90\nPin 5 (PPM Timer 0)"),
            (cx + 32, by0 - 45, cx + 10, 15, "⑥ TOLVAS 3 VÍAS\n45° Retr / 90° Conf / 135° Rech"),
            (ax - 50, ay + 15, ax - 120, by1 + 5, "⑦ PARADA EMERGENCIA\nPin 2 (INT0 FALLING)"),
            (ax + 280, ay + 12, ax + 360, by1 + 45, "⑧ HMI LCD + POT\nPin A4/A5 (I2C) / Pin A1 (SP)")
        ]

        for orig_x, orig_y, card_x, card_y, txt in callouts:
            c.create_line(orig_x, orig_y, card_x + 40, card_y + 15, fill="#546E7A", width=1, dash=(2, 2))
            c.create_oval(orig_x - 2, orig_y - 2, orig_x + 2, orig_y + 2, fill="#0D47A1")
            
            c.create_rectangle(card_x - 8, card_y - 4, card_x + 165, card_y + 28, fill="#FFFFFF", outline="#78909C", width=1)
            c.create_text(card_x + 78, card_y + 12, text=txt, fill="#263238", font=("Segoe UI", 6, "bold"), justify="center")

    # =================================================================================
    # RENDERIZADO 2D SCADA (PESTAÑA 1)
    # =================================================================================
    def draw_scada_plant(self):
        c = self.canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 100 or h < 100:
            return

        bx0 = 80
        by0 = 130
        bw = w - 240
        bh = 65
        bx1 = bx0 + bw
        by1 = by0 + bh

        # Riel metálico
        c.create_rectangle(bx0 - 10, by0 - 8, bx1 + 10, by1 + 14, fill="#ECEFF1", outline="#B0BEC5", width=2)
        c.create_rectangle(bx0, by0, bx1, by1, fill="#FFFFFF", outline="#37474F", width=1.5)

        # Rodillos
        self.roller_angle += (self.plant.belt_speed_mps / 0.28) * 15.0
        num_rollers = int(bw / 32)
        for i in range(num_rollers + 1):
            rx = bx0 + (i * 32)
            c.create_line(rx, by0 + 2, rx, by1 - 2, fill="#ECEFF1", width=2)
            dot_offset = (self.roller_angle * 1.5) % 32
            dx = bx0 + ((i * 32 + dot_offset) % bw)
            c.create_line(dx, by0 + 8, dx, by1 - 8, fill="#90A4AE", width=1.5)

        c.create_oval(bx0 - 14, by0 + 2, bx0 + 14, by1 - 2, fill="#CFD8DC", outline="#78909C", width=1.5)
        c.create_oval(bx1 - 14, by0 + 2, bx1 + 14, by1 - 2, fill="#CFD8DC", outline="#78909C", width=1.5)

        # Motor DC
        c.create_rectangle(bx0 - 65, by0 + 10, bx0 - 15, by1 - 10, fill="#ECEFF1", outline="#37474F", width=1.5)
        c.create_text(bx0 - 40, by0 + 32, text=f"MOTOR DC\nPWM: {self.plant.motor_pwm}", fill="#111111", font=("Segoe UI", 7, "bold"), justify="center")

        # Sensor HC-SR04
        sensor_px_x = bx0 + int(self.plant.SENSOR_X_M * bw)
        sensor_px_y = by0 - 35
        
        c.create_rectangle(sensor_px_x - 22, sensor_px_y - 12, sensor_px_x + 22, sensor_px_y + 12, fill="#ECEFF1", outline="#37474F", width=1)
        c.create_oval(sensor_px_x - 18, sensor_px_y - 9, sensor_px_x - 4, sensor_px_y + 9, fill="#90A4AE")
        c.create_oval(sensor_px_x + 4, sensor_px_y - 9, sensor_px_x + 18, sensor_px_y + 9, fill="#90A4AE")
        c.create_text(sensor_px_x, sensor_px_y - 20, text=f"HC-SR04: {self.plant.ultrasonic_dist_cm:.1f} cm", fill="#111111", font=("Segoe UI", 7, "bold"))

        # Ondas sonares
        if self.plant.ultrasonic_dist_cm < 80.0 and self.plant.fsm_state != "EMERGENCIA":
            self.sonar_wave_phase = (self.sonar_wave_phase + 0.15) % 1.0
            cone_len = (self.plant.ultrasonic_dist_cm / 100.0) * bw
            wave_x = sensor_px_x - (cone_len * self.sonar_wave_phase)
            if wave_x > bx0:
                c.create_arc(wave_x - 15, by0 - 10, wave_x + 15, by1 + 10, start=120, extent=120, outline="#0277BD", width=1.5, style="arc")

        # Cámara de Inspección y Foco LED
        cam_x = sensor_px_x
        cam_y = by0 - 6
        c.create_polygon(cam_x - 30, cam_y - 20, cam_x + 30, cam_y - 20, cam_x + 45, by0, cam_x - 45, by0, fill="#ECEFF1", outline="#37474F", width=1)
        
        led_brightness = int((self.plant.led_pwm / 255.0) * 100) + 155
        hex_lum = f"#{led_brightness:02x}{led_brightness:02x}B0"
        c.create_polygon(cam_x - 25, by0, cam_x + 25, by0, cam_x + 40, by1, cam_x - 40, by1, fill=hex_lum, outline="", stipple="gray25")

        c.create_oval(cam_x - 6, cam_y - 18, cam_x + 6, cam_y - 6, fill="#FFB74D" if self.plant.led_pwm > 50 else "#B0BEC5")
        c.create_text(cam_x, by0 - 32, text=f"LDR: {self.plant.total_lux_pct:.1f}% (PWM:{self.plant.led_pwm})", fill="#BF360C", font=("Segoe UI", 7, "bold"))

        # Servomotor SG90
        gate_pivot_x = bx0 + int(self.plant.GATE_X_M * bw)
        gate_pivot_y = by0 + (bh / 2)
        gate_len = 48
        
        c.create_rectangle(gate_pivot_x - 10, gate_pivot_y - 12, gate_pivot_x + 10, gate_pivot_y + 12, fill="#ECEFF1", outline="#37474F")
        c.create_oval(gate_pivot_x - 4, gate_pivot_y - 4, gate_pivot_x + 4, gate_pivot_y + 4, fill="#111111")

        rad = math.radians(self.plant.servo_angle_deg - 90)
        gate_end_x = gate_pivot_x + (gate_len * math.cos(rad))
        gate_end_y = gate_pivot_y + (gate_len * math.sin(rad))
        c.create_line(gate_pivot_x, gate_pivot_y, gate_end_x, gate_end_y, fill="#C62828", width=4, capstyle="round")
        c.create_text(gate_pivot_x + 20, gate_pivot_y - 25, text=f"SERVO: {int(self.plant.servo_angle_deg)}°", fill="#C62828", font=("Segoe UI", 7, "bold"))

        # Tolvas
        chute_base_x = bx1 + 15
        c.create_polygon(chute_base_x, by0 - 45, chute_base_x + 85, by0 - 75, chute_base_x + 85, by0 - 25, chute_base_x, by0 + 5, fill="#ECEFF1", outline="#37474F", width=1.5)
        c.create_text(chute_base_x + 45, by0 - 50, text=f"RETRABAJO\n45° ({self.plant.count_retrabajo})", fill="#E65100", font=("Segoe UI", 7, "bold"), justify="center")

        c.create_polygon(chute_base_x, by0 + 15, chute_base_x + 85, by0 + 15, chute_base_x + 85, by1 - 15, chute_base_x, by1 - 15, fill="#ECEFF1", outline="#2E7D32", width=1.5)
        c.create_text(chute_base_x + 45, by0 + 32, text=f"CONFORME\n90° ({self.plant.count_conforme})", fill="#2E7D32", font=("Segoe UI", 7, "bold"), justify="center")

        c.create_polygon(chute_base_x, by1 - 5, chute_base_x + 85, by1 + 25, chute_base_x + 85, by1 + 75, chute_base_x, by1 + 45, fill="#ECEFF1", outline="#C62828", width=1.5)
        c.create_text(chute_base_x + 45, by1 + 35, text=f"RECHAZO\n135° ({self.plant.count_rechazo})", fill="#C62828", font=("Segoe UI", 7, "bold"), justify="center")

        # Paquetes
        for p in self.plant.packages:
            if not p.is_routed:
                px = bx0 + int(p.pos_x_m * bw)
                py = by0 + 14
                pw = int(p.width_m * bw)
                ph = bh - 28
                c.create_rectangle(px, py, px + pw, py + ph, fill=p.color, outline="#111111", width=1)
                c.create_text(px + pw/2, py + ph/2, text=f"#{p.id_num}", fill="#FFFFFF", font=("Segoe UI", 7, "bold"))
            else:
                prog = p.chute_anim_progress
                if p.routed_chute == 1:
                    px = chute_base_x + (prog * 60)
                    py = by0 - 50 + (prog * 10)
                elif p.routed_chute == 3:
                    px = chute_base_x + (prog * 60)
                    py = by1 + 35 + (prog * 10)
                else:
                    px = chute_base_x + (prog * 60)
                    py = by0 + 32
                c.create_rectangle(px - 10, py - 8, px + 10, py + 8, fill=p.color, outline="#111111", width=1)

        # Emergencia Overlay
        if self.plant.emergency_stop_latched:
            c.create_rectangle(bx0, by0, bx1, by1, fill="#FFCDD2", stipple="gray50", outline="#C62828", width=2)
            c.create_text(w/2, by0 + bh/2, text="PARADA DE EMERGENCIA ACTIVADA (PWM=0 / FSM BLOQUEADA)", fill="#B71C1C", font=("Segoe UI", 10, "bold"))

    # =================================================================================
    # ACTUALIZACIÓN DE HMI Y GRÁFICOS
    # =================================================================================
    def update_virtual_hmi(self):
        st_name = self.plant.fsm_state[:6]
        d_val = int(self.plant.ultrasonic_dist_cm)
        self.lcd_line0.config(text=f"{st_name:<6}|D:{d_val:3d}cm ")
        
        m_val = self.plant.motor_pwm
        l_val = int(self.plant.total_lux_pct)
        s_val = int(self.plant.servo_angle_deg)
        self.lcd_line1.config(text=f"M:{m_val:3d} L:{l_val:2d}% S:{s_val:3d}")

        if self.plant.fsm_state == "EMERGENCIA":
            self.kpi_fsm_badge.config(text="FSM: EMERGENCIA", bg="#FFCDD2", fg="#B71C1C")
            self.kpi_estop_badge.config(text="E-STOP: ACTIVO (D2)", bg="#FFCDD2", fg="#B71C1C")
        elif self.plant.fsm_state == "TRANSPORTE":
            self.kpi_fsm_badge.config(text="FSM: TRANSPORTE", bg="#C8E6C9", fg="#1B5E20")
            self.kpi_estop_badge.config(text="E-STOP: OK (INT0)", bg="#C8E6C9", fg="#1B5E20")
        elif self.plant.fsm_state == "INSPECCION":
            self.kpi_fsm_badge.config(text="FSM: INSPECCIÓN", bg="#FFE0B2", fg="#E65100")
        elif self.plant.fsm_state == "DESVIO":
            self.kpi_fsm_badge.config(text="FSM: DESVÍO", bg="#E1BEE7", fg="#4A148C")
        else:
            self.kpi_fsm_badge.config(text="FSM: REPOSO", bg="#ECEFF1", fg="#37474F")

        now = time.time()
        if now - self.last_blink_time >= 0.2:
            self.last_blink_time = now
            self.alarm_blink_state = not self.alarm_blink_state

        green_on = self.plant.fsm_state in ["TRANSPORTE", "DESVIO"]
        self.draw_led(self.led_green_canvas, "#2E7D32" if green_on else "#CFD8DC", green_on)

        yellow_on = self.plant.fsm_state in ["REPOSO", "INSPECCION"]
        self.draw_led(self.led_yellow_canvas, "#F57C00" if yellow_on else "#CFD8DC", yellow_on)

        red_on = (self.plant.fsm_state == "EMERGENCIA") and self.alarm_blink_state
        self.draw_led(self.led_red_canvas, "#C62828" if red_on else "#CFD8DC", red_on)

        buzzer_on = red_on
        self.draw_led(self.buzzer_canvas, "#D32F2F" if buzzer_on else "#CFD8DC", buzzer_on)

    def draw_led(self, canvas: tk.Canvas, color: str, is_lit: bool):
        canvas.delete("all")
        if is_lit:
            canvas.create_oval(2, 2, 18, 18, fill=color, outline="#111111", width=1)
        else:
            canvas.create_oval(3, 3, 17, 17, fill=color, outline="#B0BEC5", width=1)

    def update_charts(self):
        t = self.time_hist
        if len(t) < 2:
            return

        self.line_dist.set_data(t, self.dist_hist)
        self.line_pwm_m.set_data(t, self.motor_pwm_hist)
        self.ax1.set_xlim(t[0], t[-1] + 0.5)

        self.line_lux.set_data(t, self.lux_hist)
        self.line_amb.set_data(t, self.ambient_hist)
        self.line_pwm_led.set_data(t, self.led_pwm_hist)
        self.ax2.set_xlim(t[0], t[-1] + 0.5)

        self.chart_canvas.draw_idle()

    def animate(self):
        now = time.time()
        dt = min(0.08, now - self.last_update_time)
        self.last_update_time = now
        self.sim_time += dt

        self.plant.update_physics_and_firmware(dt)

        self.time_hist.append(self.sim_time)
        self.dist_hist.append(self.plant.ultrasonic_dist_cm)
        self.motor_pwm_hist.append(self.plant.motor_pwm)
        self.lux_hist.append(self.plant.total_lux_pct)
        self.ambient_hist.append(self.plant.ambient_lux_pct)
        self.led_pwm_hist.append(self.plant.led_pwm)

        if len(self.time_hist) > self.max_hist:
            self.time_hist.pop(0)
            self.dist_hist.pop(0)
            self.motor_pwm_hist.pop(0)
            self.lux_hist.pop(0)
            self.ambient_hist.pop(0)
            self.led_pwm_hist.pop(0)

        self.draw_scada_plant()
        self.update_virtual_hmi()

        if int(self.sim_time * 10) % 3 == 0:
            self.update_charts()

        self.root.after(35, self.animate)


# =====================================================================================
# PUNTO DE ENTRADA PRINCIPAL
# =====================================================================================
def main():
    root = tk.Tk()
    app = DigitalTwinApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
