"""
=====================================================================================
SISTEMA DIGITAL TWIN & SUITE SCADA INDUSTRIAL - TEMA CLARO / INGENIERÍA
ESTACIÓN LOGÍSTICA DE SELECCIÓN, CLASIFICACIÓN Y ENRUTAMIENTO
NORMATIVA DE INSTRUMENTACIÓN: ANSI / ISA-5.1 (P&ID Industrial Plano Blanco)
PLATAFORMA: Arduino Uno R3 (ATmega328P) / Entorno Virtual (.venv)
PESTAÑA 4: GEMELO DIGITAL 3D CAD ACELERADO POR GPU OPENGL (GrabCAD / Gabriele Gregorini)
=====================================================================================
"""

import sys
import os
import time
import math
import random
import threading
from dataclasses import dataclass
from typing import List, Optional

import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# Soporte de Aceleración 3D por Hardware (PyOpenGL + pyopengltk)
try:
    from pyopengltk import OpenGLFrame
    from OpenGL.GL import *
    from OpenGL.GLU import *
    HAS_OPENGL_3D = True
except ImportError as e:
    HAS_OPENGL_3D = False
    OPENGL_ERR = str(e)


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


class WokwiHILBridge:
    """
    Cliente de puente Hardware-In-The-Loop (HIL) para Wokwi Simulator en VS Code.
    Se comunica bidireccionalmente vía RFC2217 / TCP Socket en localhost:4000.
    """
    def __init__(self, host="127.0.0.1", port=4000):
        self.host = host
        self.port = port
        self.ser = None
        self.is_connected = False
        self.is_active = False
        self.running = False
        self.lock = threading.Lock()
        self.thread = None
        self.last_send_time = 0.0
        
        # Telemetría recibida desde ATmega328P
        self.tlm_state = {
            "fsm_state": "REPOSO",
            "dist_cm": 50.0,
            "pwm_motor": 0,
            "lux_pct": 70.0,
            "servo_deg": 90.0,
            "emergency": False
        }

    def connect(self):
        try:
            import serial
            import serial.rfc2217
            url = f"rfc2217://{self.host}:{self.port}"
            self.ser = serial.serial_for_url(url, baudrate=9600, timeout=0.1)
            self.is_connected = True
            self.is_active = True
            self.running = True
            self.thread = threading.Thread(target=self._worker_loop, daemon=True)
            self.thread.start()
            return True, f"Conectado a Wokwi ({self.host}:{self.port})"
        except Exception:
            try:
                import serial
                url = f"socket://{self.host}:{self.port}"
                self.ser = serial.serial_for_url(url, baudrate=9600, timeout=0.1)
                self.is_connected = True
                self.is_active = True
                self.running = True
                self.thread = threading.Thread(target=self._worker_loop, daemon=True)
                self.thread.start()
                return True, f"Conectado a Wokwi Socket ({self.host}:{self.port})"
            except Exception as e2:
                self.is_connected = False
                self.is_active = False
                return False, f"Wokwi no responde en {self.host}:{self.port}.\nInicia la simulación en VS Code (F1 -> 'Wokwi: Start Simulator')."

    def disconnect(self):
        self.running = False
        self.is_active = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.is_connected = False

    def send_command(self, cmd_str: str):
        if not self.is_connected or not self.ser:
            return
        with self.lock:
            try:
                line = f"{cmd_str.strip()}\n"
                self.ser.write(line.encode('ascii'))
                self.ser.flush()
            except Exception:
                self.disconnect()

    def sync_sensors(self, dist_cm: float, lux_pct: float):
        if not self.is_connected or not self.ser:
            return
        now = time.time()
        # Espaciado de telemetría a 33 Hz (~30 ms) para transmisión de alta fidelidad sin saturación
        if now - self.last_send_time < 0.030:
            return
        self.last_send_time = now
        with self.lock:
            try:
                # Trama atómica combinada (14 bytes): SIM:<dist>,<lux>
                msg = f"SIM:{dist_cm:.1f},{lux_pct:.1f}\n"
                self.ser.write(msg.encode('ascii'))
                self.ser.flush()
            except Exception:
                self.disconnect()

    def _worker_loop(self):
        buf = ""
        while self.running and self.ser:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    buf += data.decode('ascii', errors='ignore')
                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        line = line.strip()
                        if line.startswith("TLM,"):
                            parts = line.split(',')
                            if len(parts) >= 7:
                                with self.lock:
                                    self.tlm_state["fsm_state"] = parts[1]
                                    self.tlm_state["dist_cm"] = float(parts[2])
                                    self.tlm_state["pwm_motor"] = int(parts[3])
                                    self.tlm_state["lux_pct"] = float(parts[4])
                                    self.tlm_state["servo_deg"] = float(parts[5])
                                    self.tlm_state["emergency"] = (parts[6] == '1')
                        elif "PIEZA_EN_INSPECCION" in line:
                            with self.lock:
                                self.tlm_state["fsm_state"] = "INSPEC"
                                self.tlm_state["pwm_motor"] = 0
                        elif "CLASIFICACION_COMPLETA" in line:
                            with self.lock:
                                self.tlm_state["fsm_state"] = "DESVIO"
                                self.tlm_state["pwm_motor"] = 180
                                if "ANGULO" in line:
                                    try:
                                        import json
                                        d = json.loads(line)
                                        if "ANGULO" in d:
                                            self.tlm_state["servo_deg"] = float(d["ANGULO"])
                                    except Exception:
                                        pass
                        elif "COMPUERTA_RESTABLECIDA_NEUTRO" in line:
                            with self.lock:
                                self.tlm_state["fsm_state"] = "TRANSP"
                                self.tlm_state["servo_deg"] = 90.0
                                self.tlm_state["pwm_motor"] = 200
            except Exception:
                break
        self.is_connected = False
        self.is_active = False


class VirtualIndustrialPlant:
    """
    Simulador de física continua de planta y emulación de firmware Arduino Uno R3.
    """
    def __init__(self):
        self.hil_bridge = WokwiHILBridge()
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
            pos_x_m=0.02,
            quality=quality_type,
            color=col,
            reflectance=ref,
            routed_chute=2 if quality_type == "CONFORME" else (1 if quality_type == "RETRABAJO" else 3)
        )
        self.packages.append(pkg)

        if self.fsm_state == "REPOSO" and not self.emergency_stop_latched:
            self.fsm_state = "TRANSPORTE"

        return pkg

    def trigger_emergency(self, activate: bool):
        self.emergency_stop_latched = activate
        if activate:
            self.fsm_state = "EMERGENCIA"
            self.motor_pwm = 0
            self.led_pwm = 0
            self.belt_speed_mps = 0.0

    def reset_emergency(self):
        if not self.emergency_stop_latched:
            self.fsm_state = "REPOSO"
            self.target_servo_angle = 90.0
            return True
        return False

    def update_physics_and_firmware(self, dt: float):
        # 0. MODO HARDWARE-IN-THE-LOOP (WOKWI ATMEGA328P ACTIVO)
        if self.hil_bridge.is_connected and self.hil_bridge.is_active:
            # Sincronizar lectura física actual hacia Wokwi
            self.hil_bridge.sync_sensors(self.ultrasonic_dist_cm, self.total_lux_pct)

            # Leer actuadores gobernados por el firmware de Wokwi
            with self.hil_bridge.lock:
                tlm = dict(self.hil_bridge.tlm_state)

            self.fsm_state = tlm["fsm_state"]
            self.motor_pwm = tlm["pwm_motor"]
            self.target_servo_angle = tlm["servo_deg"]
            self.led_pwm = int(max(0, min(255, tlm["lux_pct"] * 2.55)))
            
            if tlm["emergency"]:
                self.emergency_stop_latched = True
            elif self.emergency_stop_latched and not tlm["emergency"]:
                self.emergency_stop_latched = False

            # Lazo 2: Cinemática suave del servomotor
            servo_speed = 320.0
            angle_diff = self.target_servo_angle - self.servo_angle_deg
            max_step = servo_speed * dt
            if abs(angle_diff) <= max_step:
                self.servo_angle_deg = self.target_servo_angle
            else:
                self.servo_angle_deg += math.copysign(max_step, angle_diff)

        else:
            # MODO AUTÓNOMO LOCAL (EMULACIÓN PYTHON)
            if self.emergency_stop_latched:
                self.motor_pwm = 0
                self.led_pwm = 0
                self.belt_speed_mps = 0.0
                self.fsm_state = "EMERGENCIA"
                return

            # 1. MÁQUINA DE ESTADOS FINITOS (FSM)
            if self.fsm_state == "REPOSO":
                self.target_servo_angle = 90.0
                if any(not p.is_routed for p in self.packages):
                    self.fsm_state = "TRANSPORTE"

            elif self.fsm_state == "TRANSPORTE":
                self.target_servo_angle = 90.0
                for p in self.packages:
                    if not p.is_routed and not p.is_inspected:
                        if abs(p.pos_x_m - self.SENSOR_X_M) < 0.035:
                            self.fsm_state = "INSPECCION"
                            self.inspect_timer = 0.0
                            self.current_piece_under_inspection = p
                            break

            elif self.fsm_state == "INSPECCION":
                self.inspect_timer += dt
                if self.inspect_timer >= 1.2:
                    target_pkg = self.current_piece_under_inspection
                    if target_pkg:
                        target_pkg.is_inspected = True
                        if target_pkg.quality == "CONFORME":
                            self.target_servo_angle = 90.0
                            self.count_conforme += 1
                            target_pkg.routed_chute = 2
                        elif target_pkg.quality == "RETRABAJO":
                            self.target_servo_angle = 45.0
                            self.count_retrabajo += 1
                            target_pkg.routed_chute = 1
                        else: # RECHAZO
                            self.target_servo_angle = 135.0
                            self.count_rechazo += 1
                            target_pkg.routed_chute = 3
                        
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
# MOTOR DE RENDERIZADO 3D (OPENGL / PYOPENGLTK) - PESTAÑA 4
# =====================================================================================

class ObjLoader:
    """Carga y compila modelos CAD (.obj) directamente en la VRAM de la GPU."""
    def __init__(self, filename):
        self.vertices = []
        self.normals = []
        self.materials = {}
        self.faces = []

        mtl_file = None
        dir_path = os.path.dirname(filename)

        with open(filename, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if not parts:
                    continue
                if parts[0] == 'v':
                    self.vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
                elif parts[0] == 'vn':
                    self.normals.append([float(parts[1]), float(parts[2]), float(parts[3])])
                elif parts[0] == 'mtllib':
                    mtl_file = os.path.join(dir_path, parts[1])
                elif parts[0] == 'usemtl':
                    self.current_mtl = parts[1]
                elif parts[0] == 'f':
                    face_verts = []
                    for p in parts[1:]:
                        vals = p.split('/')
                        v_i = int(vals[0]) - 1
                        vn_i = int(vals[2]) - 1 if len(vals) > 2 and vals[2] else None
                        face_verts.append((v_i, vn_i))
                    self.faces.append((getattr(self, 'current_mtl', 'default'), face_verts))

        if mtl_file and os.path.exists(mtl_file):
            self.load_mtl(mtl_file)

        self.d_list_color = None
        self.d_list_clay = None

    def load_mtl(self, mtl_path):
        cur_mat = None
        with open(mtl_path, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if not parts:
                    continue
                if parts[0] == 'newmtl':
                    cur_mat = parts[1]
                    self.materials[cur_mat] = {
                        'Kd': (0.8, 0.8, 0.8),
                        'Ka': (0.2, 0.2, 0.2),
                        'Ks': (0.5, 0.5, 0.5),
                        'Ns': 32.0
                    }
                elif parts[0] == 'Kd' and cur_mat:
                    self.materials[cur_mat]['Kd'] = (float(parts[1]), float(parts[2]), float(parts[3]))
                elif parts[0] == 'Ka' and cur_mat:
                    self.materials[cur_mat]['Ka'] = (float(parts[1]), float(parts[2]), float(parts[3]))
                elif parts[0] == 'Ks' and cur_mat:
                    self.materials[cur_mat]['Ks'] = (float(parts[1]), float(parts[2]), float(parts[3]))
                elif parts[0] == 'Ns' and cur_mat:
                    self.materials[cur_mat]['Ns'] = float(parts[1])

    def compile_display_lists(self):
        """Compila la geometría en la VRAM para renderizar a 60 FPS con mínimo uso de CPU."""
        # 1. Modo CLAY (CAD Monocromático técnico)
        self.d_list_clay = glGenLists(1)
        glNewList(self.d_list_clay, GL_COMPILE)
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, [0.70, 0.73, 0.78, 1.0])
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, [0.35, 0.36, 0.38, 1.0])
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [0.65, 0.65, 0.70, 1.0])
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0)
        glColor3f(0.70, 0.73, 0.78)

        glBegin(GL_TRIANGLES)
        for _, face in self.faces:
            if len(face) == 3:
                for v_i, vn_i in face:
                    if vn_i is not None and vn_i < len(self.normals):
                        glNormal3fv(self.normals[vn_i])
                    glVertex3fv(self.vertices[v_i])
        glEnd()

        glBegin(GL_QUADS)
        for _, face in self.faces:
            if len(face) == 4:
                for v_i, vn_i in face:
                    if vn_i is not None and vn_i < len(self.normals):
                        glNormal3fv(self.normals[vn_i])
                    glVertex3fv(self.vertices[v_i])
        glEnd()
        glEndList()

        # 2. Modo COLOR (Fotorrealista PBR)
        self.d_list_color = glGenLists(1)
        glNewList(self.d_list_color, GL_COMPILE)

        mat_faces = {}
        for mat_name, face in self.faces:
            if mat_name not in mat_faces:
                mat_faces[mat_name] = []
            mat_faces[mat_name].append(face)

        for mat_name, faces in mat_faces.items():
            mat_props = self.materials.get(mat_name, None)
            if mat_props:
                kd = mat_props['Kd']
                ks = mat_props['Ks']
                ka = mat_props['Ka']
                ns = mat_props['Ns']
                glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, [kd[0], kd[1], kd[2], 1.0])
                glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, [ka[0], ka[1], ka[2], 1.0])
                glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [ks[0], ks[1], ks[2], 1.0])
                glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, ns)
                glColor3f(kd[0], kd[1], kd[2])
            else:
                glColor3f(0.8, 0.8, 0.8)

            triangles = [f for f in faces if len(f) == 3]
            if triangles:
                glBegin(GL_TRIANGLES)
                for f in triangles:
                    for v_i, vn_i in f:
                        if vn_i is not None and vn_i < len(self.normals):
                            glNormal3fv(self.normals[vn_i])
                        glVertex3fv(self.vertices[v_i])
                glEnd()

            quads = [f for f in faces if len(f) == 4]
            if quads:
                glBegin(GL_QUADS)
                for f in quads:
                    for v_i, vn_i in f:
                        if vn_i is not None and vn_i < len(self.normals):
                            glNormal3fv(self.normals[vn_i])
                        glVertex3fv(self.vertices[v_i])
                glEnd()

        glEndList()


if HAS_OPENGL_3D:
    class OpenGL3DConveyorFrame(OpenGLFrame):
        """
        Marco de visualización 3D interactivo acoplado a la planta y controlado por GPU.
        """
        def __init__(self, master, plant_ref, *args, **kw):
            super().__init__(master, *args, **kw)
            self.plant = plant_ref

            # Parámetros de Cámara Orbital
            self.target = [0.0, 10.0, 0.0]
            self.dist = 580.0
            self.yaw = 32.0
            self.pitch = 26.0
            self.render_mode = "COLOR"  # "COLOR" o "CLAY"
            self.wireframe = False

            self.last_x = 0
            self.last_y = 0
            self.is_rotating = False
            self.is_panning = False

            self.belt_offset = 0.0
            self.roller_angle = 0.0
            self.sonar_pulse = 0.0

            self.loader = None
            self.models_loaded = False

            # Eventos de Ratón para Orbit Controls
            self.bind("<ButtonPress-1>", self.on_lmb_down)
            self.bind("<ButtonRelease-1>", self.on_lmb_up)
            self.bind("<B1-Motion>", self.on_lmb_drag)
            self.bind("<ButtonPress-3>", self.on_rmb_down)
            self.bind("<ButtonRelease-3>", self.on_rmb_up)
            self.bind("<B3-Motion>", self.on_rmb_drag)
            self.bind("<MouseWheel>", self.on_mousewheel)

        def on_lmb_down(self, event):
            self.is_rotating = True
            self.last_x = event.x
            self.last_y = event.y

        def on_lmb_up(self, event):
            self.is_rotating = False

        def on_lmb_drag(self, event):
            if self.is_rotating:
                dx = event.x - self.last_x
                dy = event.y - self.last_y
                self.last_x = event.x
                self.last_y = event.y
                self.yaw = (self.yaw + dx * 0.45) % 360.0
                self.pitch = max(5.0, min(85.0, self.pitch - dy * 0.45))
                self._display()

        def on_rmb_down(self, event):
            self.is_panning = True
            self.last_x = event.x
            self.last_y = event.y

        def on_rmb_up(self, event):
            self.is_panning = False

        def on_rmb_drag(self, event):
            if self.is_panning:
                dx = event.x - self.last_x
                dy = event.y - self.last_y
                self.last_x = event.x
                self.last_y = event.y
                rad_yaw = math.radians(self.yaw)
                speed = 0.0018 * self.dist
                rx = -math.sin(rad_yaw) * dx * speed
                rz = math.cos(rad_yaw) * dx * speed
                self.target[0] += rx
                self.target[2] += rz
                self.target[1] += dy * speed
                self._display()

        def on_mousewheel(self, event):
            step = (event.delta / 120.0) * 45.0
            self.dist = max(140.0, min(1800.0, self.dist - step))
            self._display()

        def set_preset(self, preset_name):
            if preset_name == "ISOMETRIC":
                self.target = [0.0, 10.0, 0.0]
                self.dist = 580.0
                self.yaw = 32.0
                self.pitch = 26.0
            elif preset_name == "SENSOR":
                self.target = [-140.0, 15.0, 0.0]
                self.dist = 340.0
                self.yaw = -20.0
                self.pitch = 20.0
            elif preset_name == "CAMERA":
                self.target = [50.0, 20.0, 0.0]
                self.dist = 300.0
                self.yaw = 85.0
                self.pitch = 30.0
            elif preset_name == "SERVO":
                self.target = [135.0, 10.0, 0.0]
                self.dist = 360.0
                self.yaw = 135.0
                self.pitch = 24.0
            elif preset_name == "TOP":
                self.target = [0.0, 10.0, 0.0]
                self.dist = 520.0
                self.yaw = 0.0
                self.pitch = 85.0
            self._display()

        def toggle_color_mode(self):
            self.render_mode = "CLAY" if self.render_mode == "COLOR" else "COLOR"
            self._display()

        def toggle_wireframe(self):
            self.wireframe = not self.wireframe
            self._display()

        def initgl(self):
            glEnable(GL_DEPTH_TEST)
            glDepthFunc(GL_LEQUAL)
            glShadeModel(GL_SMOOTH)
            glEnable(GL_COLOR_MATERIAL)
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
            glEnable(GL_NORMALIZE)

            # Fondo claro estudio técnico
            glClearColor(0.93, 0.94, 0.96, 1.0)

            # Luces de estudio
            glEnable(GL_LIGHTING)
            glEnable(GL_LIGHT0)
            glLightfv(GL_LIGHT0, GL_POSITION, [200.0, 400.0, 300.0, 1.0])
            glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.95, 0.95, 0.98, 1.0])
            glLightfv(GL_LIGHT0, GL_SPECULAR, [0.60, 0.60, 0.65, 1.0])
            glLightfv(GL_LIGHT0, GL_AMBIENT, [0.40, 0.42, 0.45, 1.0])

            glEnable(GL_LIGHT1)
            glLightfv(GL_LIGHT1, GL_POSITION, [-300.0, 200.0, -200.0, 1.0])
            glLightfv(GL_LIGHT1, GL_DIFFUSE, [0.45, 0.48, 0.52, 1.0])
            glLightfv(GL_LIGHT1, GL_AMBIENT, [0.20, 0.22, 0.25, 1.0])

            # Foco dinámico de inspección (GL_LIGHT2)
            glEnable(GL_LIGHT2)
            glLightf(GL_LIGHT2, GL_CONSTANT_ATTENUATION, 1.0)
            glLightf(GL_LIGHT2, GL_LINEAR_ATTENUATION, 0.003)

            # Cargar modelo CAD ensamblado
            obj_path = os.path.join(os.path.dirname(__file__), "modelsd", "faja_transportadora_ensamble.obj")
            if os.path.exists(obj_path) and not self.models_loaded:
                try:
                    self.loader = ObjLoader(obj_path)
                    self.loader.compile_display_lists()
                    self.models_loaded = True
                except Exception as e:
                    print(f"[ERROR OBJ] {e}")

        def update_kinematics(self, dt: float):
            speed_mmps = self.plant.belt_speed_mps * 1000.0
            self.belt_offset = (self.belt_offset + speed_mmps * dt) % 20.0
            self.roller_angle = (self.roller_angle + speed_mmps * dt * 0.15) % 360.0
            self.sonar_pulse = (self.sonar_pulse + dt * 6.0) % (2.0 * math.pi)

        def redraw(self):
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

            w = max(1, self.winfo_width())
            h = max(1, self.winfo_height())
            glViewport(0, 0, w, h)

            glMatrixMode(GL_PROJECTION)
            glLoadIdentity()
            gluPerspective(38.0, w / float(h), 1.0, 3500.0)

            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()

            rad_yaw = math.radians(self.yaw)
            rad_pitch = math.radians(self.pitch)
            eye_x = self.target[0] + self.dist * math.cos(rad_pitch) * math.sin(rad_yaw)
            eye_y = self.target[1] + self.dist * math.sin(rad_pitch)
            eye_z = self.target[2] + self.dist * math.cos(rad_pitch) * math.cos(rad_yaw)

            gluLookAt(
                eye_x, eye_y, eye_z,
                self.target[0], self.target[1], self.target[2],
                0.0, 1.0, 0.0
            )

            # 1. Suelo técnico de estudio
            self.draw_studio_floor()

            # 2. Modelo CAD del ensamble GrabCAD
            if self.models_loaded and self.loader:
                if self.wireframe:
                    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)

                if self.render_mode == "CLAY":
                    glCallList(self.loader.d_list_clay)
                else:
                    glCallList(self.loader.d_list_color)

                if self.wireframe:
                    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

            # 3. Nervaduras móviles de la faja transportadora
            self.draw_moving_belt_ribs()

            # 4. Haz ultrasónico dinámico (HC-SR04 / Lazo 1)
            self.draw_sonar_beam()

            # 5. Gantry de inspección óptica y foco LED dinámico (Lazo 3)
            self.draw_camera_gantry_and_spotlight()

            # 6. Servomotor SG90 y compuerta deflectora móvil (Lazo 2)
            self.draw_diverter_gate()

            # 7. Tolvas de clasificación por gravedad (3 vías)
            self.draw_sorting_chutes()

            # 8. Cajas dinámicas físicas sobre la faja
            self.draw_dynamic_packages()

        def draw_studio_floor(self):
            """Suelo técnico de estudio y cuadrícula métrica de ingeniería."""
            # Base plana
            glBegin(GL_QUADS)
            glNormal3f(0.0, 1.0, 0.0)
            glColor3f(0.90, 0.92, 0.94)
            glVertex3f(-450.0, -45.0, -350.0)
            glVertex3f( 450.0, -45.0, -350.0)
            glVertex3f( 450.0, -45.0,  350.0)
            glVertex3f(-450.0, -45.0,  350.0)
            glEnd()

            # Cuadrícula gris
            glDisable(GL_LIGHTING)
            glColor3f(0.80, 0.82, 0.86)
            glLineWidth(1.0)
            glBegin(GL_LINES)
            for i in range(-15, 16):
                # Líneas paralelas a Z
                glVertex3f(i * 30.0, -44.8, -350.0)
                glVertex3f(i * 30.0, -44.8,  350.0)
                # Líneas paralelas a X
                glVertex3f(-450.0, -44.8, i * 23.0)
                glVertex3f( 450.0, -44.8, i * 23.0)
            glEnd()

            # Perímetro de seguridad amarillo
            glColor3f(0.94, 0.80, 0.15)
            glLineWidth(2.5)
            glBegin(GL_LINE_LOOP)
            glVertex3f(-260.0, -44.6, -160.0)
            glVertex3f( 260.0, -44.6, -160.0)
            glVertex3f( 260.0, -44.6,  160.0)
            glVertex3f(-260.0, -44.6,  160.0)
            glEnd()
            glEnable(GL_LIGHTING)

        def draw_moving_belt_ribs(self):
            """Dibuja nervaduras de tracción sobre la banda que avanzan proporcional a la velocidad."""
            glDisable(GL_LIGHTING)
            glColor3f(0.35, 0.38, 0.42)
            glLineWidth(2.0)
            glBegin(GL_LINES)
            for i in range(18):
                rx = -150.0 + i * 20.0 + self.belt_offset
                if -155.0 <= rx <= 155.0:
                    glVertex3f(rx, 14.8, -30.0)
                    glVertex3f(rx, 14.8,  30.0)
            glEnd()
            glEnable(GL_LIGHTING)

        def draw_sonar_beam(self):
            """Cono sonar translúcido proyectado desde el HC-SR04 hacia las piezas."""
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            
            # Longitud del cono proporcional a la distancia medida (escala en mm)
            d_mm = max(25.0, min(240.0, self.plant.ultrasonic_dist_cm * 3.5))
            pulse = 1.8 * math.sin(self.sonar_pulse)

            glPushMatrix()
            # El sensor en la placa está ubicado aproximadamente en X = -232 mm apuntando hacia +X
            glTranslatef(-210.0, 13.5, 0.0)
            glRotatef(90.0, 0, 0, 1)

            glColor4f(0.15, 0.65, 1.0, 0.22)
            glBegin(GL_TRIANGLE_FAN)
            glVertex3f(0.0, 0.0, 0.0) # Vértice
            for i in range(17):
                ang = 2.0 * math.pi * i / 16.0
                rad = 18.0 + pulse
                glVertex3f(math.cos(ang) * rad, d_mm, math.sin(ang) * rad)
            glEnd()

            glPopMatrix()
            glDisable(GL_BLEND)

        def draw_camera_gantry_and_spotlight(self):
            """Pórtico de inspección óptica cenital y foco dinámico del Lazo 3."""
            gx = 50.0  # Posición longitudinal del pórtico
            
            # Columnas del pórtico (aluminio 2020)
            self._draw_box(gx, 50.0, -55.0, 16.0, 95.0, 16.0, (0.75, 0.77, 0.80))
            self._draw_box(gx, 50.0,  55.0, 16.0, 95.0, 16.0, (0.75, 0.77, 0.80))
            # Travesaño superior
            self._draw_box(gx, 98.0, 0.0, 18.0, 16.0, 126.0, (0.75, 0.77, 0.80))

            # Cuerpo de cámara industrial
            self._draw_box(gx, 80.0, 0.0, 26.0, 32.0, 26.0, (0.18, 0.20, 0.24))
            # Lente apuntando hacia abajo
            self._draw_cylinder(gx, 62.0, 0.0, 8.0, 16.0, (0.08, 0.08, 0.10), axis='y')
            # Anillo de iluminación LED coaxial
            self._draw_cylinder(gx, 58.0, 0.0, 16.0, 6.0, (0.85, 0.85, 0.88), axis='y')

            # Actualizar foco dinámico GL_LIGHT2
            led_intensity = max(0.05, min(1.0, self.plant.led_pwm / 255.0))
            glow = (0.95 * led_intensity, 0.95 * led_intensity, 0.80 * led_intensity)
            glLightfv(GL_LIGHT2, GL_POSITION, [gx, 60.0, 0.0, 1.0])
            glLightfv(GL_LIGHT2, GL_DIFFUSE, [glow[0], glow[1], glow[2], 1.0])
            glLightfv(GL_LIGHT2, GL_SPECULAR, [glow[0], glow[1], glow[2], 1.0])

            # Cono de luz visible proyectado hacia la banda
            glDisable(GL_LIGHTING)
            glDepthMask(GL_FALSE)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE) # Luz aditiva

            glPushMatrix()
            glTranslatef(gx, 58.0, 0.0)
            glBegin(GL_TRIANGLE_FAN)
            glColor4f(1.0, 0.98, 0.85, 0.08 + 0.25 * led_intensity)
            glVertex3f(0.0, 0.0, 0.0)
            glColor4f(1.0, 0.95, 0.80, 0.01 + 0.03 * led_intensity)
            for i in range(17):
                ang = 2.0 * math.pi * i / 16.0
                glVertex3f(math.cos(ang) * 45.0, -44.0, math.sin(ang) * 45.0)
            glEnd()
            glPopMatrix()

            glDisable(GL_BLEND)
            glDepthMask(GL_TRUE)
            glEnable(GL_LIGHTING)

        def draw_diverter_gate(self):
            """Servomotor SG90 y compuerta deflectora rotatoria del Lazo 2."""
            sx = 125.0
            sz = 47.0
            sy = 22.0

            # Soporte y cuerpo del servo
            self._draw_box(sx, sy, sz, 24.0, 22.0, 14.0, (0.15, 0.45, 0.85))
            # Eje de salida
            self._draw_cylinder(sx, sy + 12.0, sz - 5.0, 3.5, 6.0, (0.85, 0.75, 0.25), axis='y')

            # Hoja deflectora giratoria
            glPushMatrix()
            glTranslatef(sx, 16.0, sz - 5.0)
            # 90° es recto; rotamos en Y según el ángulo actual del servo
            gate_rot = self.plant.servo_angle_deg - 90.0
            glRotatef(gate_rot, 0, 1, 0)

            # Hoja de acero inoxidable (longitud 75 mm)
            self._draw_box(-35.0, 0.0, 0.0, 70.0, 20.0, 4.0, (0.88, 0.90, 0.92))
            # Puntera amortiguadora negra
            self._draw_box(-72.0, 0.0, 0.0, 6.0, 20.0, 6.0, (0.15, 0.15, 0.15))
            glPopMatrix()

        def draw_sorting_chutes(self):
            """Tolvas metálicas inclinadas de clasificación hacia gavetas recolectoras."""
            # 1. Tolva Retrabajo (45°) - Lateral Izquierda (Ámbar)
            glPushMatrix()
            glTranslatef(185.0, 6.0, -65.0)
            glRotatef(22.0, 1, 0, 0)
            glRotatef(-15.0, 0, 1, 0)
            self._draw_box(0, 0, 0, 65.0, 5.0, 55.0, (0.75, 0.78, 0.82))
            self._draw_box(0, 6.0, -28.0, 65.0, 12.0, 4.0, (0.90, 0.65, 0.15))
            glPopMatrix()
            # Gaveta Retrabajo
            self._draw_box(210.0, -32.0, -95.0, 65.0, 24.0, 55.0, (0.90, 0.65, 0.15))

            # 2. Tolva Conforme (90°) - Central Frontal (Verde)
            glPushMatrix()
            glTranslatef(210.0, 6.0, 0.0)
            glRotatef(-22.0, 0, 0, 1)
            self._draw_box(0, 0, 0, 75.0, 5.0, 65.0, (0.75, 0.78, 0.82))
            self._draw_box(0, 6.0, -32.0, 75.0, 12.0, 4.0, (0.22, 0.65, 0.28))
            self._draw_box(0, 6.0,  32.0, 75.0, 12.0, 4.0, (0.22, 0.65, 0.28))
            glPopMatrix()
            # Gaveta Conforme
            self._draw_box(270.0, -32.0, 0.0, 70.0, 24.0, 65.0, (0.22, 0.65, 0.28))

            # 3. Tolva Rechazo (135°) - Lateral Derecha (Roja)
            glPushMatrix()
            glTranslatef(185.0, 6.0, 65.0)
            glRotatef(-22.0, 1, 0, 0)
            glRotatef(15.0, 0, 1, 0)
            self._draw_box(0, 0, 0, 65.0, 5.0, 55.0, (0.75, 0.78, 0.82))
            self._draw_box(0, 6.0, 28.0, 65.0, 12.0, 4.0, (0.85, 0.22, 0.20))
            glPopMatrix()
            # Gaveta Rechazo
            self._draw_box(210.0, -32.0, 95.0, 65.0, 24.0, 55.0, (0.85, 0.22, 0.20))

        def draw_dynamic_packages(self):
            """Renderiza las cajas reales generadas por el simulador viajando por la faja."""
            for p in self.plant.packages:
                # Convertir posición de metros (0.0 a 1.0 m) a coordenadas 3D de la faja (-140 a +160 mm)
                box_x = -140.0 + (p.pos_x_m / 1.0) * 300.0
                box_y = 14.5 + 12.0  # Sobre la superficie superior de la banda
                box_z = 0.0
                rot_y = 0.0
                rot_z = 0.0

                # Desvío transversal y deslizamiento en tolvas
                if p.is_routed or p.is_diverting:
                    prog = p.chute_anim_progress
                    if p.routed_chute == 1: # Retrabajo
                        box_x += prog * 45.0
                        box_z -= prog * 75.0
                        box_y -= prog * 40.0
                        rot_y = -35.0
                        rot_z = 20.0
                    elif p.routed_chute == 3: # Rechazo
                        box_x += prog * 45.0
                        box_z += prog * 75.0
                        box_y -= prog * 40.0
                        rot_y = 35.0
                        rot_z = -20.0
                    else: # Conforme
                        box_x += prog * 90.0
                        box_y -= prog * 40.0
                        rot_z = 25.0

                # Color según calidad
                col = (0.22, 0.65, 0.28)
                if p.quality == "RETRABAJO":
                    col = (0.92, 0.65, 0.15)
                elif p.quality == "RECHAZO":
                    col = (0.85, 0.22, 0.20)

                glPushMatrix()
                glTranslatef(box_x, box_y, box_z)
                glRotatef(rot_y, 0, 1, 0)
                glRotatef(rot_z, 0, 0, 1)

                # Cuerpo de la caja (Cartón con tinte técnico)
                self._draw_box(0, 0, 0, 36.0, 24.0, 36.0, col)
                # Etiqueta blanca de código de barras superior
                self._draw_box(0, 12.2, 0, 20.0, 0.5, 20.0, (0.96, 0.96, 0.96))
                # Cinta de embalaje
                self._draw_box(0, 12.3, 0, 34.0, 0.4, 7.0, (0.78, 0.65, 0.45))
                glPopMatrix()

        def _draw_box(self, x, y, z, dx, dy, dz, color):
            glColor3f(color[0], color[1], color[2])
            hx, hy, hz = dx * 0.5, dy * 0.5, dz * 0.5
            glBegin(GL_QUADS)
            # +Z
            glNormal3f(0.0, 0.0, 1.0)
            glVertex3f(x - hx, y - hy, z + hz)
            glVertex3f(x + hx, y - hy, z + hz)
            glVertex3f(x + hx, y + hy, z + hz)
            glVertex3f(x - hx, y + hy, z + hz)
            # -Z
            glNormal3f(0.0, 0.0, -1.0)
            glVertex3f(x + hx, y - hy, z - hz)
            glVertex3f(x - hx, y - hy, z - hz)
            glVertex3f(x - hx, y + hy, z - hz)
            glVertex3f(x + hx, y + hy, z - hz)
            # +Y
            glNormal3f(0.0, 1.0, 0.0)
            glVertex3f(x - hx, y + hy, z - hz)
            glVertex3f(x - hx, y + hy, z + hz)
            glVertex3f(x + hx, y + hy, z + hz)
            glVertex3f(x + hx, y + hy, z - hz)
            # -Y
            glNormal3f(0.0, -1.0, 0.0)
            glVertex3f(x - hx, y - hy, z - hz)
            glVertex3f(x + hx, y - hy, z - hz)
            glVertex3f(x + hx, y - hy, z + hz)
            glVertex3f(x - hx, y - hy, z + hz)
            # +X
            glNormal3f(1.0, 0.0, 0.0)
            glVertex3f(x + hx, y - hy, z - hz)
            glVertex3f(x + hx, y + hy, z - hz)
            glVertex3f(x + hx, y + hy, z + hz)
            glVertex3f(x + hx, y - hy, z + hz)
            # -X
            glNormal3f(-1.0, 0.0, 0.0)
            glVertex3f(x - hx, y - hy, z - hz)
            glVertex3f(x - hx, y - hy, z + hz)
            glVertex3f(x - hx, y + hy, z + hz)
            glVertex3f(x - hx, y + hy, z - hz)
            glEnd()

        def _draw_cylinder(self, x, y, z, radius, height, color, axis='y', segments=16):
            glColor3f(color[0], color[1], color[2])
            glPushMatrix()
            glTranslatef(x, y, z)
            if axis == 'x':
                glRotatef(90, 0, 0, 1)
            elif axis == 'z':
                glRotatef(90, 1, 0, 0)
            
            hh = height * 0.5
            glBegin(GL_QUAD_STRIP)
            for i in range(segments + 1):
                ang = 2.0 * math.pi * i / segments
                nx = math.cos(ang)
                nz = math.sin(ang)
                glNormal3f(nx, 0.0, nz)
                glVertex3f(nx * radius, -hh, nz * radius)
                glVertex3f(nx * radius,  hh, nz * radius)
            glEnd()

            for sign, normal in [(-hh, -1.0), (hh, 1.0)]:
                glBegin(GL_TRIANGLE_FAN)
                glNormal3f(0.0, normal, 0.0)
                glVertex3f(0.0, sign, 0.0)
                for i in range(segments + 1):
                    ang = 2.0 * math.pi * i / segments * (-normal)
                    glVertex3f(math.cos(ang) * radius, sign, math.sin(ang) * radius)
                glEnd()
            glPopMatrix()


# =====================================================================================
# INTERFAZ GRÁFICA MULTI-PESTAÑA SCADA (TEMA CLARO / INGENIERÍA)
# =====================================================================================

class DigitalTwinApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("SCADA INDUSTRIAL - ESTACIÓN LOGÍSTICA DE CLASIFICACIÓN [ANSI / ISA-5.1]")
        self.root.geometry("1400x920")
        self.root.minsize(1260, 800)

        self.plant = VirtualIndustrialPlant()

        # Buffers históricos para gráficos
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
        
        # Pestañas en tema claro de ingeniería
        style.configure("TNotebook", background="#ECEFF1", borderwidth=1)
        style.configure("TNotebook.Tab", background="#CFD8DC", foreground="#37474F", 
                        padding=[14, 6], font=("Segoe UI", 9, "bold"), borderwidth=1)
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

        # Tab 1: SCADA Principal 2D
        self.tab_scada = tk.Frame(self.notebook, bg="#ECEFF1")
        self.notebook.add(self.tab_scada, text="  [1] PLANTA SCADA EN VIVO (2D)  ")
        self.build_tab_scada(self.tab_scada)

        # Tab 2: Diagrama P&ID ANSI / ISA-5.1
        self.tab_pid = tk.Frame(self.notebook, bg="#ECEFF1")
        self.notebook.add(self.tab_pid, text="  [2] DIAGRAMA P&ID (NORMA ANSI / ISA-5.1)  ")
        self.build_tab_pid(self.tab_pid)

        # Tab 3: Arquitectura y Componentes Señalizados
        self.tab_hw = tk.Frame(self.notebook, bg="#ECEFF1")
        self.notebook.add(self.tab_hw, text="  [3] COMPONENTES Y ASIGNACIÓN HARDWARE  ")
        self.build_tab_hardware(self.tab_hw)

        # Tab 4: Gemelo Digital 3D CAD Acelerado por GPU
        self.tab_3d = tk.Frame(self.notebook, bg="#ECEFF1")
        self.notebook.add(self.tab_3d, text="  [4] GEMELO DIGITAL 3D (CAD / GPU)  ")
        self.build_tab_3d(self.tab_3d)

        # Evento de cambio de pestaña para gestionar el renderizado 3D de GPU
        self.notebook.bind("<<NotebookTabChanged>>", self.on_tab_changed)

    def on_tab_changed(self, event):
        try:
            selected_idx = self.notebook.index(self.notebook.select())
            if selected_idx == 3 and hasattr(self, 'gl_frame'):
                self.gl_frame._display()
        except Exception:
            pass

    # =================================================================================
    # PESTAÑA 1: PLANTA SCADA EN VIVO (2D)
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
                  command=self.on_start_clicked).pack(fill="x", pady=1)
        tk.Button(c1, text="STOP", bg="#546E7A", fg="white", font=("Segoe UI", 8, "bold"), bd=1, relief="raised",
                  command=self.on_stop_clicked).pack(fill="x", pady=1)
        tk.Button(c1, text="RESET", bg="#0277BD", fg="white", font=("Segoe UI", 7, "bold"), bd=1, relief="raised",
                  command=self.on_reset_clicked).pack(fill="x", pady=1)

        # Enlace HIL Wokwi
        c_hil = tk.LabelFrame(col_box, text=" Enlace HIL Wokwi ", font=("Segoe UI", 7, "bold"), fg="#01579B", bg="#FFFFFF", padx=4, pady=3)
        c_hil.pack(side="left", fill="both", expand=True, padx=2)
        self.btn_hil = tk.Button(c_hil, text="⚡ Conectar Wokwi\n(Port 4000)", bg="#E1F5FE", fg="#01579B", font=("Segoe UI", 7, "bold"), bd=1, relief="raised",
                                 command=self.toggle_wokwi_hil)
        self.btn_hil.pack(fill="x", pady=1)
        self.lbl_hil_status = tk.Label(c_hil, text="● Modo Local", font=("Segoe UI", 6, "bold"), fg="#546E7A", bg="#FFFFFF")
        self.lbl_hil_status.pack(anchor="center")

        # Alimentador
        c2 = tk.LabelFrame(col_box, text=" Alimentación ", font=("Segoe UI", 7, "bold"), fg="#37474F", bg="#FFFFFF", padx=4, pady=3)
        c2.pack(side="left", fill="both", expand=True, padx=2)
        tk.Button(c2, text="+ Conforme (90°)", bg="#388E3C", fg="white", font=("Segoe UI", 7), bd=1, relief="raised",
                  command=lambda: self.on_spawn_package("CONFORME")).pack(fill="x", pady=1)
        tk.Button(c2, text="+ Retrabajo (45°)", bg="#F57C00", fg="white", font=("Segoe UI", 7), bd=1, relief="raised",
                  command=lambda: self.on_spawn_package("RETRABAJO")).pack(fill="x", pady=1)
        tk.Button(c2, text="+ Rechazo (135°)", bg="#D32F2F", fg="white", font=("Segoe UI", 7), bd=1, relief="raised",
                  command=lambda: self.on_spawn_package("RECHAZO")).pack(fill="x", pady=1)

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
        if hasattr(self, 'lbl3d_lux_val'):
            self.lbl3d_lux_val.config(text=f"Ambiente: {int(v)}%")

    def set_ambient_lux(self, val):
        self.slider_lux.set(val)
        self.on_ambient_lux_changed(val)

    def on_start_clicked(self):
        if self.plant.hil_bridge.is_connected:
            self.plant.hil_bridge.send_command("START")
        if self.plant.fsm_state == "REPOSO" and not self.plant.emergency_stop_latched:
            self.plant.fsm_state = "TRANSPORTE"

    def on_stop_clicked(self):
        if self.plant.hil_bridge.is_connected:
            self.plant.hil_bridge.send_command("STOP")
        self.plant.fsm_state = "REPOSO"

    def on_spawn_package(self, quality_type: Optional[str] = None):
        self.plant.spawn_package(quality_type)
        if self.plant.hil_bridge.is_connected and not self.plant.emergency_stop:
            if self.plant.hil_bridge.tlm_state.get("fsm_state") == "REPOSO":
                self.plant.hil_bridge.send_command("START")

    def toggle_wokwi_hil(self):
        if not self.plant.hil_bridge.is_connected:
            ok, msg = self.plant.hil_bridge.connect()
            if ok:
                self._update_hil_ui_state(True)
                if not self.plant.emergency_stop:
                    time.sleep(0.05)
                    self.plant.hil_bridge.send_command("START")
                messagebox.showinfo(
                    "Enlace HIL Wokwi Exitoso",
                    f"¡Conexión establecida con Wokwi en puerto 4000!\n\n"
                    f"{msg}\n\n"
                    f"• El microcontrolador ATmega328P en Wokwi gobierna ahora la estación.\n"
                    f"• Las cajas 3D transmiten distancias y lux en tiempo real.\n"
                    f"• El motor y la compuerta 3D responden al PWM y servo de Wokwi."
                )
            else:
                self._update_hil_ui_state(False)
                messagebox.showwarning(
                    "Enlace HIL Wokwi No Disponible",
                    f"{msg}\n\n"
                    f"Pasos para activar:\n"
                    f"1. Abre VS Code.\n"
                    f"2. Presiona F1 y escribe 'Wokwi: Start Simulator'.\n"
                    f"3. Vuelve a pulsar este botón para sincronizar."
                )
        else:
            self.plant.hil_bridge.disconnect()
            self._update_hil_ui_state(False)
            messagebox.showinfo("Enlace HIL Desconectado", "Se cerró el enlace con Wokwi.\nLa estación continúa operando con el emulador local en Python.")

    def _update_hil_ui_state(self, connected: bool):
        btn_txt = "✓ Wokwi Online\n(Desconectar)" if connected else "⚡ Conectar Wokwi\n(Port 4000)"
        btn_bg = "#C8E6C9" if connected else "#E1F5FE"
        btn_fg = "#1B5E20" if connected else "#01579B"
        lbl_txt = "● HIL Activo (C++)" if connected else "● Modo Local"
        lbl_fg = "#2E7D32" if connected else "#546E7A"

        if hasattr(self, 'btn_hil'):
            self.btn_hil.config(text=btn_txt, bg=btn_bg, fg=btn_fg)
        if hasattr(self, 'lbl_hil_status'):
            self.lbl_hil_status.config(text=lbl_txt, fg=lbl_fg)
        if hasattr(self, 'btn3d_hil'):
            self.btn3d_hil.config(text=btn_txt, bg=btn_bg, fg=btn_fg)
        if hasattr(self, 'lbl3d_hil_status'):
            self.lbl3d_hil_status.config(text=lbl_txt, fg=lbl_fg)

    def on_estop_clicked(self):
        new_state = not self.plant.emergency_stop_latched
        self.plant.trigger_emergency(new_state)
        if self.plant.hil_bridge.is_connected:
            self.plant.hil_bridge.send_command("STOP" if new_state else "RESET")
        text = "ENCLAVADO\n(Libere y Rearme)" if new_state else "PARADA DE\nEMERGENCIA\n(INT0 D2)"
        bg = "#5C0000" if new_state else "#C62828"
        self.btn_estop.config(text=text, bg=bg)
        if hasattr(self, 'btn3d_estop'):
            self.btn3d_estop.config(text=text, bg=bg)

    def on_reset_clicked(self):
        if self.plant.hil_bridge.is_connected:
            self.plant.hil_bridge.send_command("RESET")
        if self.plant.emergency_stop_latched:
            self.plant.emergency_stop_latched = False
            self.btn_estop.config(text="PARADA DE\nEMERGENCIA\n(INT0 D2)", bg="#C62828")
            if hasattr(self, 'btn3d_estop'):
                self.btn3d_estop.config(text="PARADA DE\nEMERGENCIA\n(INT0 D2)", bg="#C62828")
        
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

        # Cajetín de Ingeniería ANSI
        c.create_rectangle(920, 15, 1260, 95, fill="#FFFFFF", outline="#37474F", width=1.5)
        c.create_text(935, 30, text="PROYECTO: ESTACIÓN DE CLASIFICACIÓN", fill="#111111", font=("Segoe UI", 8, "bold"), anchor="w")
        c.create_text(935, 48, text="NORMA: ANSI / ISA-5.1 (P&ID)", fill="#37474F", font=("Segoe UI", 8), anchor="w")
        c.create_text(935, 65, text="CONTROLADOR: ARDUINO UNO R3 (PLC / DCS)", fill="#0D47A1", font=("Segoe UI", 8, "bold"), anchor="w")
        c.create_text(935, 82, text="ESC: S/E | REV: 2.0 | TAGS: ISA-5.1", fill="#546E7A", font=("Segoe UI", 7), anchor="w")

        # Faja Transportadora CV-100
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
        bx0 = 160; by0 = 150; bw = 880; bh = 65
        bx1 = bx0 + bw; by1 = by0 + bh

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
        ax = bx0 + 200; ay = by1 + 35
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
    # PESTAÑA 4: GEMELO DIGITAL 3D CAD ACELERADO POR GPU OPENGL
    # =================================================================================
    def build_tab_3d(self, parent):
        main_container = tk.Frame(parent, bg="#ECEFF1")
        main_container.pack(fill="both", expand=True, pady=4)

        left_panel = tk.Frame(main_container, bg="#ECEFF1")
        left_panel.pack(side="left", fill="both", expand=True, padx=(0, 6))

        # Card del Visor 3D
        canvas_card = tk.Frame(left_panel, bg="#FFFFFF", bd=1, relief="solid")
        canvas_card.pack(fill="both", expand=True)

        # Barra de Herramientas de Cámara y Modos
        toolbar = tk.Frame(canvas_card, bg="#FAFAFA", pady=4, padx=6)
        toolbar.pack(fill="x", side="top")

        lbl_t = tk.Label(toolbar, text="VISTAS TÉCNICAS:", font=("Segoe UI", 8, "bold"), fg="#263238", bg="#FAFAFA")
        lbl_t.pack(side="left", padx=(0, 4))

        if HAS_OPENGL_3D:
            tk.Button(toolbar, text="🌐 Isométrica", font=("Segoe UI", 7), bg="#FFFFFF", fg="#263238", bd=1, relief="solid",
                      command=lambda: self.gl_frame.set_preset("ISOMETRIC")).pack(side="left", padx=2)
            tk.Button(toolbar, text="🔍 HC-SR04", font=("Segoe UI", 7), bg="#FFFFFF", fg="#263238", bd=1, relief="solid",
                      command=lambda: self.gl_frame.set_preset("SENSOR")).pack(side="left", padx=2)
            tk.Button(toolbar, text="💡 Cámara/Luz", font=("Segoe UI", 7), bg="#FFFFFF", fg="#263238", bd=1, relief="solid",
                      command=lambda: self.gl_frame.set_preset("CAMERA")).pack(side="left", padx=2)
            tk.Button(toolbar, text="🔀 Servomotor", font=("Segoe UI", 7), bg="#FFFFFF", fg="#263238", bd=1, relief="solid",
                      command=lambda: self.gl_frame.set_preset("SERVO")).pack(side="left", padx=2)
            tk.Button(toolbar, text="👁️ Superior", font=("Segoe UI", 7), bg="#FFFFFF", fg="#263238", bd=1, relief="solid",
                      command=lambda: self.gl_frame.set_preset("TOP")).pack(side="left", padx=2)
            tk.Button(toolbar, text="🔄 Reset", font=("Segoe UI", 7), bg="#ECEFF1", fg="#0D47A1", bd=1, relief="solid",
                      command=lambda: self.gl_frame.set_preset("ISOMETRIC")).pack(side="left", padx=4)

            # Modos de Renderizado
            tk.Button(toolbar, text="🎨 Color/Clay", font=("Segoe UI", 7, "bold"), bg="#E0F2F1", fg="#004D40", bd=1, relief="solid",
                      command=lambda: self.gl_frame.toggle_color_mode()).pack(side="right", padx=2)
            tk.Button(toolbar, text="📐 Alámbrico", font=("Segoe UI", 7), bg="#ECEFF1", fg="#37474F", bd=1, relief="solid",
                      command=lambda: self.gl_frame.toggle_wireframe()).pack(side="right", padx=2)

            # Frame OpenGL de Alto Rendimiento
            self.gl_frame = OpenGL3DConveyorFrame(canvas_card, self.plant)
            self.gl_frame.pack(fill="both", expand=True, padx=4, pady=(0, 4))
        else:
            lbl_err = tk.Label(canvas_card, text=f"[AVISO] OpenGL no disponible:\n{OPENGL_ERR}", font=("Segoe UI", 10), fg="#C62828", bg="#FFFFFF")
            lbl_err.pack(fill="both", expand=True)

        # Panel de Mandos Inferior sincronizado
        control_card = tk.Frame(left_panel, bg="#FFFFFF", bd=1, relief="solid", pady=6, padx=8)
        control_card.pack(fill="x", pady=(6, 0))
        self.create_control_panel_3d(control_card)

        # Lado Derecho: Telemetría y Controles en 3D
        right_panel = tk.Frame(main_container, bg="#ECEFF1", width=440)
        right_panel.pack(side="right", fill="both", expand=False)
        right_panel.pack_propagate(False)

        self.create_virtual_arduino_card_3d(right_panel)
        self.create_telemetry_gauges_card_3d(right_panel)

    def create_virtual_arduino_card_3d(self, parent):
        card = tk.Frame(parent, bg="#FFFFFF", bd=1, relief="solid", padx=10, pady=8)
        card.pack(fill="x", pady=(0, 6))

        lbl_title = tk.Label(card, text="HMI LOCAL (LCD 16x2 / SEÑALIZACIÓN EN VIVO)", 
                             font=("Segoe UI", 8, "bold"), fg="#263238", bg="#FFFFFF")
        lbl_title.pack(anchor="w", pady=(0, 4))

        # Display LCD 16x2
        lcd_bezel = tk.Frame(card, bg="#78909C", padx=3, pady=3, bd=1, relief="solid")
        lcd_bezel.pack(fill="x")

        lcd_screen = tk.Frame(lcd_bezel, bg="#9CCC65", padx=6, pady=4)
        lcd_screen.pack(fill="x")

        self.lcd3d_line0 = tk.Label(lcd_screen, text="REPOSO|D: 100cm ", font=("Courier New", 11, "bold"), fg="#1B5E20", bg="#9CCC65", anchor="w")
        self.lcd3d_line0.pack(fill="x")
        self.lcd3d_line1 = tk.Label(lcd_screen, text="M:  0 L: 40% S: 90", font=("Courier New", 11, "bold"), fg="#1B5E20", bg="#9CCC65", anchor="w")
        self.lcd3d_line1.pack(fill="x")

        # Fila de LEDs y Buzzer
        led_row = tk.Frame(card, bg="#FFFFFF", pady=6)
        led_row.pack(fill="x")

        f_g = tk.Frame(led_row, bg="#FFFFFF"); f_g.pack(side="left", expand=True)
        self.led3d_green_canvas = tk.Canvas(f_g, width=20, height=20, bg="#FFFFFF", highlightthickness=0); self.led3d_green_canvas.pack()
        tk.Label(f_g, text="RUN (D11)", font=("Segoe UI", 7), fg="#546E7A", bg="#FFFFFF").pack()

        f_y = tk.Frame(led_row, bg="#FFFFFF"); f_y.pack(side="left", expand=True)
        self.led3d_yellow_canvas = tk.Canvas(f_y, width=20, height=20, bg="#FFFFFF", highlightthickness=0); self.led3d_yellow_canvas.pack()
        tk.Label(f_y, text="INSP (D12)", font=("Segoe UI", 7), fg="#546E7A", bg="#FFFFFF").pack()

        f_r = tk.Frame(led_row, bg="#FFFFFF"); f_r.pack(side="left", expand=True)
        self.led3d_red_canvas = tk.Canvas(f_r, width=20, height=20, bg="#FFFFFF", highlightthickness=0); self.led3d_red_canvas.pack()
        tk.Label(f_r, text="ALARM (D13)", font=("Segoe UI", 7), fg="#546E7A", bg="#FFFFFF").pack()

        f_b = tk.Frame(led_row, bg="#FFFFFF"); f_b.pack(side="left", expand=True)
        self.buzzer3d_canvas = tk.Canvas(f_b, width=20, height=20, bg="#FFFFFF", highlightthickness=0); self.buzzer3d_canvas.pack()
        tk.Label(f_b, text="BUZZER (D10)", font=("Segoe UI", 7), fg="#546E7A", bg="#FFFFFF").pack()

    def create_telemetry_gauges_card_3d(self, parent):
        card = tk.Frame(parent, bg="#FFFFFF", bd=1, relief="solid", padx=10, pady=8)
        card.pack(fill="both", expand=True)

        lbl_title = tk.Label(card, text="MONITOREO DE LAZOS CERRADOS Y PRODUCCIÓN", 
                             font=("Segoe UI", 8, "bold"), fg="#263238", bg="#FFFFFF")
        lbl_title.pack(anchor="w", pady=(0, 6))

        # Lazo 1
        f_l1 = tk.LabelFrame(card, text=" LAZO 1: Tracción y Frenado Proporcional (HC-SR04) ", font=("Segoe UI", 7, "bold"), fg="#0D47A1", bg="#FFFFFF", padx=6, pady=4)
        f_l1.pack(fill="x", pady=3)
        self.lbl3d_lazo1_val = tk.Label(f_l1, text="Distancia: 100 cm | Motor PWM: 0/200 (0.0 cm/s)", font=("Segoe UI", 8), fg="#263238", bg="#FFFFFF")
        self.lbl3d_lazo1_val.pack(anchor="w")
        self.prog3d_dist = ttk.Progressbar(f_l1, orient="horizontal", length=380, mode="determinate")
        self.prog3d_dist.pack(fill="x", pady=2)

        # Lazo 2
        f_l2 = tk.LabelFrame(card, text=" LAZO 2: Servocontrol de Enrutamiento (SG90) ", font=("Segoe UI", 7, "bold"), fg="#E65100", bg="#FFFFFF", padx=6, pady=4)
        f_l2.pack(fill="x", pady=3)
        self.lbl3d_lazo2_val = tk.Label(f_l2, text="Ángulo Servo: 90° | Tolva: TOLVA_CONFORME", font=("Segoe UI", 8), fg="#263238", bg="#FFFFFF")
        self.lbl3d_lazo2_val.pack(anchor="w")

        # Lazo 3
        f_l3 = tk.LabelFrame(card, text=" LAZO 3: Estabilización Fotométrica (LDR + LED) ", font=("Segoe UI", 7, "bold"), fg="#2E7D32", bg="#FFFFFF", padx=6, pady=4)
        f_l3.pack(fill="x", pady=3)
        self.lbl3d_lazo3_val = tk.Label(f_l3, text="Luz en Cámara: 40% (SP: 70%) | Foco LED PWM: 25/255", font=("Segoe UI", 8), fg="#263238", bg="#FFFFFF")
        self.lbl3d_lazo3_val.pack(anchor="w")
        self.prog3d_lux = ttk.Progressbar(f_l3, orient="horizontal", length=380, mode="determinate")
        self.prog3d_lux.pack(fill="x", pady=2)

        # Contadores de Clasificación
        f_cnt = tk.LabelFrame(card, text=" CONTADORES DE CLASIFICACIÓN ", font=("Segoe UI", 7, "bold"), fg="#37474F", bg="#FFFFFF", padx=6, pady=4)
        f_cnt.pack(fill="x", pady=(6, 0))

        c_box = tk.Frame(f_cnt, bg="#FFFFFF")
        c_box.pack(fill="x")
        
        self.card3d_tot = self._create_metric_pill(c_box, "TOTAL", "0", "#37474F")
        self.card3d_conf = self._create_metric_pill(c_box, "CONFORME", "0", "#2E7D32")
        self.card3d_retr = self._create_metric_pill(c_box, "RETRABAJO", "0", "#E65100")
        self.card3d_rech = self._create_metric_pill(c_box, "RECHAZO", "0", "#C62828")

        # Info de Ayuda para Navegación 3D
        f_help = tk.LabelFrame(card, text=" CONTROLES DE RATÓN EN EL VISOR 3D ", font=("Segoe UI", 7, "bold"), fg="#546E7A", bg="#FAFAFA", padx=6, pady=4)
        f_help.pack(fill="x", pady=(10, 0))
        tk.Label(f_help, text="• Clic Izquierdo + Arrastrar: Rotación orbital libre en 360°\n• Clic Derecho + Arrastrar: Traslación y paneo de encuadre\n• Rueda de Ratón: Zoom óptico suave de acercamiento/alejamiento",
                 font=("Segoe UI", 7), fg="#37474F", bg="#FAFAFA", justify="left").pack(anchor="w")

    def _create_metric_pill(self, parent, title, val, color):
        f = tk.Frame(parent, bg="#F5F5F5", bd=1, relief="solid", padx=4, pady=3)
        f.pack(side="left", fill="both", expand=True, padx=2)
        tk.Label(f, text=title, font=("Segoe UI", 6, "bold"), fg="#78909C", bg="#F5F5F5").pack()
        lbl_v = tk.Label(f, text=val, font=("Segoe UI", 10, "bold"), fg=color, bg="#F5F5F5")
        lbl_v.pack()
        return lbl_v

    def create_control_panel_3d(self, parent):
        col_box = tk.Frame(parent, bg="#FFFFFF")
        col_box.pack(fill="x")

        # Mando
        c1 = tk.LabelFrame(col_box, text=" Mando ", font=("Segoe UI", 7, "bold"), fg="#37474F", bg="#FFFFFF", padx=4, pady=3)
        c1.pack(side="left", fill="both", expand=True, padx=2)
        tk.Button(c1, text="START", bg="#2E7D32", fg="white", font=("Segoe UI", 8, "bold"), bd=1, relief="raised",
                  command=self.on_start_clicked).pack(fill="x", pady=1)
        tk.Button(c1, text="STOP", bg="#546E7A", fg="white", font=("Segoe UI", 8, "bold"), bd=1, relief="raised",
                  command=self.on_stop_clicked).pack(fill="x", pady=1)
        tk.Button(c1, text="RESET", bg="#0277BD", fg="white", font=("Segoe UI", 7, "bold"), bd=1, relief="raised",
                  command=self.on_reset_clicked).pack(fill="x", pady=1)

        # Enlace HIL Wokwi
        c_hil = tk.LabelFrame(col_box, text=" Enlace HIL Wokwi ", font=("Segoe UI", 7, "bold"), fg="#01579B", bg="#FFFFFF", padx=4, pady=3)
        c_hil.pack(side="left", fill="both", expand=True, padx=2)
        self.btn3d_hil = tk.Button(c_hil, text="⚡ Conectar Wokwi\n(Port 4000)", bg="#E1F5FE", fg="#01579B", font=("Segoe UI", 7, "bold"), bd=1, relief="raised",
                                   command=self.toggle_wokwi_hil)
        self.btn3d_hil.pack(fill="x", pady=1)
        self.lbl3d_hil_status = tk.Label(c_hil, text="● Modo Local", font=("Segoe UI", 6, "bold"), fg="#546E7A", bg="#FFFFFF")
        self.lbl3d_hil_status.pack(anchor="center")

        # Alimentador
        c2 = tk.LabelFrame(col_box, text=" Alimentación ", font=("Segoe UI", 7, "bold"), fg="#37474F", bg="#FFFFFF", padx=4, pady=3)
        c2.pack(side="left", fill="both", expand=True, padx=2)
        tk.Button(c2, text="+ Conforme (90°)", bg="#388E3C", fg="white", font=("Segoe UI", 7), bd=1, relief="raised",
                  command=lambda: self.on_spawn_package("CONFORME")).pack(fill="x", pady=1)
        tk.Button(c2, text="+ Retrabajo (45°)", bg="#F57C00", fg="white", font=("Segoe UI", 7), bd=1, relief="raised",
                  command=lambda: self.on_spawn_package("RETRABAJO")).pack(fill="x", pady=1)
        tk.Button(c2, text="+ Rechazo (135°)", bg="#D32F2F", fg="white", font=("Segoe UI", 7), bd=1, relief="raised",
                  command=lambda: self.on_spawn_package("RECHAZO")).pack(fill="x", pady=1)

        # Perturbación
        c3 = tk.LabelFrame(col_box, text=" Luz Externa ", font=("Segoe UI", 7, "bold"), fg="#37474F", bg="#FFFFFF", padx=4, pady=3)
        c3.pack(side="left", fill="both", expand=True, padx=2)
        self.lbl3d_lux_val = tk.Label(c3, text="Ambiente: 40%", font=("Segoe UI", 7, "bold"), fg="#E65100", bg="#FFFFFF")
        self.lbl3d_lux_val.pack(anchor="w")

        f_lux_btns = tk.Frame(c3, bg="#FFFFFF"); f_lux_btns.pack(fill="x", pady=2)
        tk.Button(f_lux_btns, text="Sombra (15%)", bg="#ECEFF1", fg="#263238", font=("Segoe UI", 7), bd=1, relief="raised",
                  command=lambda: self.set_ambient_lux(15)).pack(side="left", fill="x", expand=True, padx=1)
        tk.Button(f_lux_btns, text="Sol (80%)", bg="#ECEFF1", fg="#263238", font=("Segoe UI", 7), bd=1, relief="raised",
                  command=lambda: self.set_ambient_lux(80)).pack(side="left", fill="x", expand=True, padx=1)

        # Emergencia
        c4 = tk.LabelFrame(col_box, text=" Seguridad ", font=("Segoe UI", 7, "bold"), fg="#C62828", bg="#FFFFFF", padx=4, pady=3)
        c4.pack(side="left", fill="both", expand=True, padx=2)
        self.btn3d_estop = tk.Button(c4, text="PARADA DE\nEMERGENCIA\n(INT0 D2)", bg="#C62828", fg="white", 
                                     font=("Segoe UI", 8, "bold"), bd=2, relief="raised", cursor="hand2",
                                     command=self.on_estop_clicked)
        self.btn3d_estop.pack(fill="both", expand=True, pady=1)

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

        bx0 = 80; by0 = 130; bw = w - 240; bh = 65
        bx1 = bx0 + bw; by1 = by0 + bh

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
            c.create_oval(dx - 1, by0 + 4, dx + 1, by0 + 6, fill="#B0BEC5", outline="")

        # Banda transportadora superior e inferior
        c.create_line(bx0, by0, bx1, by0, fill="#111111", width=3)
        c.create_line(bx0, by1, bx1, by1, fill="#111111", width=3)

        # Tambores en extremos
        c.create_oval(bx0 - 14, by0, bx0 + 14, by1, fill="#CFD8DC", outline="#111111", width=1.5)
        c.create_oval(bx1 - 14, by0, bx1 + 14, by1, fill="#CFD8DC", outline="#111111", width=1.5)

        # Motor DC acoplado a la derecha
        m_x0 = bx1 + 20; m_y0 = by0 + 5
        m_col = "#0D47A1" if self.plant.motor_pwm > 0 else "#78909C"
        c.create_rectangle(m_x0, m_y0, m_x0 + 45, m_y0 + 55, fill="#ECEFF1", outline="#111111", width=1.5)
        c.create_oval(m_x0 + 8, m_y0 + 10, m_x0 + 37, m_y0 + 45, fill=m_col, outline="#111111")
        c.create_text(m_x0 + 22, m_y0 + 27, text="M", fill="white", font=("Segoe UI", 10, "bold"))
        c.create_line(bx1, by0 + 27, m_x0, m_y0 + 27, fill="#111111", width=4)

        # Sensor Ultrasónico HC-SR04
        sens_canvas_x = bx0 + int(self.plant.SENSOR_X_M * bw)
        sens_y = by0 - 45
        c.create_rectangle(sens_canvas_x - 20, sens_y - 12, sens_canvas_x + 20, sens_y + 12, fill="#0D47A1", outline="#111111", width=1.5)
        c.create_oval(sens_canvas_x - 16, sens_y - 8, sens_canvas_x - 4, sens_y + 8, fill="#CFD8DC", outline="#111111")
        c.create_oval(sens_canvas_x + 4, sens_y - 8, sens_canvas_x + 16, sens_y + 8, fill="#CFD8DC", outline="#111111")

        # Haz Ultrasónico Sonar
        self.sonar_wave_phase = (self.sonar_wave_phase + 0.15) % 1.0
        cone_h = min(by0 - (sens_y + 12), int(self.plant.ultrasonic_dist_cm * 2.2))
        for wave_i in range(3):
            w_prog = (self.sonar_wave_phase + (wave_i * 0.33)) % 1.0
            arc_y = sens_y + 12 + int(w_prog * cone_h)
            arc_w = int(12 + w_prog * 28)
            if arc_y < by0:
                c.create_arc(sens_canvas_x - arc_w, arc_y - 8, sens_canvas_x + arc_w, arc_y + 8, 
                             start=0, extent=180, style="arc", outline="#0288D1", width=1.5)

        # Estación de Cámara e Iluminador LED
        cam_x = sens_canvas_x
        cam_y = by0 - 85
        c.create_rectangle(cam_x - 25, cam_y - 15, cam_x + 25, cam_y + 5, fill="#37474F", outline="#111111", width=1.5)
        c.create_oval(cam_x - 8, cam_y + 2, cam_x + 8, cam_y + 14, fill="#111111", outline="#CFD8DC")
        
        # Cono de luz LED
        led_intensity = self.plant.led_pwm / 255.0
        if led_intensity > 0.05:
            light_w = int(25 + led_intensity * 35)
            c.create_polygon(cam_x, cam_y + 14, cam_x - light_w, by0, cam_x + light_w, by0, 
                             fill="#FFF9C4", outline="", stipple="gray25" if led_intensity < 0.6 else "gray50")

        # Fotorresistencia LDR
        c.create_rectangle(cam_x - 38, by0 - 20, cam_x - 26, by0 - 5, fill="#FFB74D", outline="#111111")
        c.create_text(cam_x - 32, by0 - 12, text="LDR", fill="#111111", font=("Segoe UI", 6, "bold"))

        # Servomotor SG90 y Compuerta
        gate_x = bx0 + int(self.plant.GATE_X_M * bw)
        c.create_rectangle(gate_x - 8, by0 - 25, gate_x + 8, by0 - 5, fill="#C62828", outline="#111111", width=1.5)
        c.create_oval(gate_x - 5, by0 - 8, gate_x + 5, by0 + 2, fill="#B0BEC5")

        rad_angle = math.radians(self.plant.servo_angle_deg)
        arm_len = 45
        arm_dx = -arm_len * math.cos(rad_angle)
        arm_dy = arm_len * math.sin(rad_angle)
        c.create_line(gate_x, by0, gate_x + arm_dx, by0 + arm_dy, fill="#D32F2F", width=4, capstyle="round")

        # 3 Tolvas de Salida
        chute_base_x = bx1 + 15
        c.create_polygon(chute_base_x, by0 - 35, chute_base_x + 75, by0 - 55, chute_base_x + 75, by0 - 15, chute_base_x, by0 + 5, 
                         fill="#ECEFF1", outline="#111111", width=1.5)
        c.create_text(chute_base_x + 38, by0 - 32, text="RETRABAJO\n45°", fill="#E65100", font=("Segoe UI", 6, "bold"), justify="center")

        c.create_polygon(chute_base_x, by0 + 15, chute_base_x + 75, by0 + 15, chute_base_x + 75, by1 - 10, chute_base_x, by1 - 10, 
                         fill="#ECEFF1", outline="#111111", width=1.5)
        c.create_text(chute_base_x + 38, by0 + 32, text="CONFORME\n90°", fill="#2E7D32", font=("Segoe UI", 6, "bold"), justify="center")

        c.create_polygon(chute_base_x, by1 - 5, chute_base_x + 75, by1 + 15, chute_base_x + 75, by1 + 55, chute_base_x, by1 + 35, 
                         fill="#ECEFF1", outline="#111111", width=1.5)
        c.create_text(chute_base_x + 38, by1 + 25, text="RECHAZO\n135°", fill="#C62828", font=("Segoe UI", 6, "bold"), justify="center")

        # Dibujar Paquetes en Tránsito
        for p in self.plant.packages:
            if not p.is_routed and not p.is_diverting:
                px = bx0 + int(p.pos_x_m * bw)
                py = by0 + 32
                c.create_rectangle(px - 14, py - 12, px + 14, py + 12, fill=p.color, outline="#111111", width=1.5)
                c.create_rectangle(px - 8, py - 6, px + 8, py + 6, fill="#FFFFFF", outline="")
            elif p.is_diverting:
                prog = p.chute_anim_progress
                if p.routed_chute == 1:
                    px = chute_base_x + (prog * 60)
                    py = by0 - 25 - (prog * 15)
                elif p.routed_chute == 3:
                    px = chute_base_x + (prog * 60)
                    py = by1 + 15 + (prog * 15)
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
        m_val = self.plant.motor_pwm
        l_val = int(self.plant.total_lux_pct)
        s_val = int(self.plant.servo_angle_deg)

        # LCD Pestaña 1
        self.lcd_line0.config(text=f"{st_name:<6}|D:{d_val:3d}cm ")
        self.lcd_line1.config(text=f"M:{m_val:3d} L:{l_val:2d}% S:{s_val:3d}")

        # LCD Pestaña 4 (3D)
        if hasattr(self, 'lcd3d_line0'):
            self.lcd3d_line0.config(text=f"{st_name:<6}|D:{d_val:3d}cm ")
            self.lcd3d_line1.config(text=f"M:{m_val:3d} L:{l_val:2d}% S:{s_val:3d}")

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
        if hasattr(self, 'led3d_green_canvas'):
            self.draw_led(self.led3d_green_canvas, "#2E7D32" if green_on else "#CFD8DC", green_on)

        yellow_on = self.plant.fsm_state in ["REPOSO", "INSPECCION"]
        self.draw_led(self.led_yellow_canvas, "#F57C00" if yellow_on else "#CFD8DC", yellow_on)
        if hasattr(self, 'led3d_yellow_canvas'):
            self.draw_led(self.led3d_yellow_canvas, "#F57C00" if yellow_on else "#CFD8DC", yellow_on)

        red_on = (self.plant.fsm_state == "EMERGENCIA") and self.alarm_blink_state
        self.draw_led(self.led_red_canvas, "#C62828" if red_on else "#CFD8DC", red_on)
        if hasattr(self, 'led3d_red_canvas'):
            self.draw_led(self.led3d_red_canvas, "#C62828" if red_on else "#CFD8DC", red_on)

        buzzer_on = red_on
        self.draw_led(self.buzzer_canvas, "#D32F2F" if buzzer_on else "#CFD8DC", buzzer_on)
        if hasattr(self, 'buzzer3d_canvas'):
            self.draw_led(self.buzzer3d_canvas, "#D32F2F" if buzzer_on else "#CFD8DC", buzzer_on)

        # Actualizar Gauges de la Pestaña 4
        if hasattr(self, 'lbl3d_lazo1_val'):
            v_cmps = self.plant.belt_speed_mps * 100.0
            self.lbl3d_lazo1_val.config(text=f"Distancia: {d_val} cm | Motor PWM: {m_val}/200 ({v_cmps:4.1f} cm/s)")
            self.prog3d_dist["value"] = max(0, min(100, (50.0 - self.plant.ultrasonic_dist_cm) * 2.0))

            chute_names = {1: "TOLVA_RETRABAJO (45°)", 2: "TOLVA_CONFORME (90°)", 3: "TOLVA_RECHAZO (135°)"}
            dest_chute = chute_names.get(int(1 if s_val < 65 else (3 if s_val > 110 else 2)), "CONFORME")
            self.lbl3d_lazo2_val.config(text=f"Ángulo Servo: {s_val}° | Destino: {dest_chute}")

            self.lbl3d_lazo3_val.config(text=f"Luz en Cámara: {l_val}% (SP: {int(self.plant.sp_lux_pct)}%) | Foco LED PWM: {self.plant.led_pwm}/255")
            self.prog3d_lux["value"] = max(0, min(100, l_val))

            self.card3d_tot.config(text=str(self.plant.total_processed))
            self.card3d_conf.config(text=str(self.plant.count_conforme))
            self.card3d_retr.config(text=str(self.plant.count_retrabajo))
            self.card3d_rech.config(text=str(self.plant.count_rechazo))

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

        # Actualizar SCADA 2D y HMI
        self.draw_scada_plant()
        self.update_virtual_hmi()

        # Si el usuario tiene seleccionada la Pestaña 4 (Gemelo 3D), redibujar con GPU
        try:
            current_tab = self.notebook.index(self.notebook.select())
            if current_tab == 3 and hasattr(self, 'gl_frame'):
                self.gl_frame.update_kinematics(dt)
                self.gl_frame._display()
        except Exception:
            pass

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
