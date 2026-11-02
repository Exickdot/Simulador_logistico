"""
=====================================================================================
GEMELO DIGITAL 3D - ESTACIÓN LOGÍSTICA DE CLASIFICACIÓN Y ENRUTAMIENTO
Motor Gráfico 3D Nativo (OpenGL / Pygame-CE) - Entorno Industrial GrabCAD
=====================================================================================
Este módulo renderiza en una ventana nativa de escritorio un entorno 3D fotorrealista
y técnico de la estación logística controlada por el Arduino Uno R3:

1. Faja transportadora modular CAD (GrabCAD / Gabriele Gregorini) con perfiles de
   aluminio T-Slot 20x20 mm, rodillos con rodamientos 608ZZ, placas laterales y
   banda móvil continua acelerada en GPU por OpenGL Display Lists.
2. Motorreductor DC / NEMA 17 acoplado con cinemática sincronizada.
3. Sensor ultrasónico HC-SR04 (ZT-101) con placa frontal ranurada GrabCAD e incidencia
   perpendicular normal a 90° sobre la cara frontal de las cajas en aproximación.
4. Gantry de inspección óptica con cámara industrial (QE-103) e iluminador LED (QL-103)
   cuyo foco dinámico responde en tiempo real al lazo fotométrico (Lazo 3).
5. Lanzadera / Carro móvil transversal M-102 indexador con 3 contenedores industriales
   segregados (Retrabajo 45°, Conforme 90°, Rechazo 135°).
6. Cajas dinámicas físicas que entran por tolva de gravedad, avanzan, son inspeccionadas,
   sensadas por ultrasonido y caen por la rampa libre hacia el carro móvil.
7. Cámara orbital 3D interactiva (rotación, traslación, zoom y 5 presets técnicos).
8. Consola retro industrial unificada HUD 2D (fósforo verde, matriz LED OEE y mandos).
=====================================================================================
"""

import sys
import os
import math
import time
import random
import queue
import threading
from dataclasses import dataclass
from typing import List, Optional, Tuple

import pygame
from pygame.locals import *

try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
except ImportError:
    print("[ERROR] PyOpenGL no está instalado en el entorno virtual.")
    print("Ejecute: .venv\\Scripts\\pip install PyOpenGL pygame-ce")
    sys.exit(1)

# Importar el emulador de planta física existente
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from emulator.plant_emulator import PlantPhysicsModel, VirtualArduinoFirmware, PlantState


# =====================================================================================
# CLIENTE PUENTE HARDWARE-IN-THE-LOOP (WOKWI SIMULATOR / RFC2217)
# =====================================================================================
class WokwiHILBridge:
    """
    Cliente de puente Hardware-In-The-Loop (HIL) para Wokwi Simulator en VS Code.
    Diseño 100% asíncrono y no bloqueante:
    - Las transmisiones se encolan sin esperar ni bloquear el hilo principal de renderizado.
    - Se elimina la contención de cerraduras (lock contention) y timeouts de socket.
    - Garantiza 60 FPS estables sin micro-parones durante la conexión con Wokwi.
    """
    def __init__(self, host="127.0.0.1", port=4000):
        self.host = host
        self.port = port
        self.ser = None
        self.is_connected = False
        self.is_active = False
        self.running = False
        self.lock = threading.Lock()
        self._tx_queue = queue.Queue(maxsize=8)
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
            self.ser = serial.serial_for_url(url, baudrate=115200, timeout=0.01)
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
                self.ser = serial.serial_for_url(url, baudrate=115200, timeout=0.01)
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
        if not self.is_connected:
            return
        msg = f"{cmd_str.strip()}\n".encode('ascii')
        try:
            self._tx_queue.put_nowait(msg)
        except queue.Full:
            pass

    def sync_sensors(self, dist_cm: float, lux_pct: float):
        if not self.is_connected:
            return
        now = time.time()
        # Espaciado de telemetría a 33 Hz (~30 ms) sin saturar el bus
        if now - self.last_send_time < 0.030:
            return
        self.last_send_time = now
        msg = f"SIM:{dist_cm:.1f},{lux_pct:.1f}\n".encode('ascii')
        try:
            self._tx_queue.put_nowait(msg)
        except queue.Full:
            try:
                self._tx_queue.get_nowait()
                self._tx_queue.put_nowait(msg)
            except Exception:
                pass

    def _worker_loop(self):
        buf = ""
        while self.running and self.ser:
            # 1. Transmisión asíncrona desde la cola sin bloquear al hilo gráfico
            try:
                while not self._tx_queue.empty():
                    item = self._tx_queue.get_nowait()
                    self.ser.write(item)
            except Exception:
                pass

            # 2. Recepción no bloqueante y sincronización de baja latencia
            try:
                waiting = self.ser.in_waiting if hasattr(self.ser, 'in_waiting') else 0
                if waiting > 0:
                    data = self.ser.read(waiting)
                    if data:
                        buf += data.decode('ascii', errors='ignore')
                        # Evitar acumulación de retraso (Buffer Bloat): conservar únicamente las tramas más recientes
                        if len(buf) > 512:
                            last_nl = buf.rfind('\n')
                            if last_nl > 256:
                                buf = buf[last_nl - 256:]
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
                else:
                    time.sleep(0.003)
            except Exception:
                break
        self.is_connected = False
        self.is_active = False


# =====================================================================================
# CLASES DE CÁMARA Y NAVEGACIÓN 3D
# =====================================================================================
class Camera3D:
    """Cámara orbital técnica tipo CAD / Digital Twin."""
    def __init__(self):
        # Coordenadas esféricas
        self.target = [0.08, 0.38, 0.0]  # Punto focal [x, y, z]
        self.distance = 1.65             # Distancia al objetivo (m)
        self.yaw = 42.0                  # Ángulo azimutal (grados)
        self.pitch = 26.0                # Ángulo de elevación (grados)
        
        # Interpolación suave hacia objetivos
        self.target_goal = list(self.target)
        self.distance_goal = self.distance
        self.yaw_goal = self.yaw
        self.pitch_goal = self.pitch

        # Arrastre con ratón
        self.is_rotating = False
        self.is_panning = False
        self.last_mouse = (0, 0)

    def set_preset(self, index: int):
        """Preajustes de vista estándar de ingeniería."""
        if index == 1:   # Isométrica General
            self.set_goals([0.08, 0.38, 0.0], 1.65, 42.0, 26.0)
        elif index == 2: # Sensor Ultrasónico GrabCAD
            self.set_goals([0.42, 0.40, 0.0], 0.70, 160.0, 20.0)
        elif index == 3: # Estación Fotométrica: Banco de 4 LEDs y LDR 5mm (Lazo 3)
            self.set_goals([0.355, 0.450, 0.0], 0.48, 65.0, 24.0)
        elif index == 4: # Servomotor SG90 y Carro Indexador M-102 (Lazo 2)
            self.set_goals([0.64, 0.14, -0.12], 0.76, 142.0, 26.0)
        elif index == 5: # Tolva de Entrada
            self.set_goals([-0.42, 0.48, 0.0], 0.85, 320.0, 30.0)

    def set_goals(self, target, dist, yaw, pitch):
        self.target_goal = list(target)
        self.distance_goal = max(0.4, min(3.5, dist))
        self.yaw_goal = yaw
        self.pitch_goal = max(-85.0, min(85.0, pitch))

    def update(self, dt: float):
        """Interpola suavemente la posición de la cámara."""
        blend = min(1.0, dt * 10.0)
        for i in range(3):
            self.target[i] += (self.target_goal[i] - self.target[i]) * blend
        self.distance += (self.distance_goal - self.distance) * blend
        self.yaw += (self.yaw_goal - self.yaw) * blend
        self.pitch += (self.pitch_goal - self.pitch) * blend

    def handle_event(self, event):
        if event.type == MOUSEBUTTONDOWN:
            if event.button == 1:  # Botón izquierdo: Rotar
                self.is_rotating = True
                self.last_mouse = event.pos
            elif event.button in (2, 3):  # Botón central o derecho: Trasladar
                self.is_panning = True
                self.last_mouse = event.pos
            elif event.button == 4:  # Scroll arriba: Zoom in
                self.distance_goal = max(0.4, self.distance_goal - 0.12)
            elif event.button == 5:  # Scroll abajo: Zoom out
                self.distance_goal = min(3.5, self.distance_goal + 0.12)

        elif event.type == MOUSEBUTTONUP:
            if event.button == 1:
                self.is_rotating = False
            elif event.button in (2, 3):
                self.is_panning = False

        elif event.type == MOUSEMOTION:
            dx = event.pos[0] - self.last_mouse[0]
            dy = event.pos[1] - self.last_mouse[1]
            self.last_mouse = event.pos

            if self.is_rotating:
                self.yaw_goal = (self.yaw_goal + dx * 0.4) % 360.0
                self.pitch_goal = max(5.0, min(85.0, self.pitch_goal - dy * 0.4))
            elif self.is_panning:
                rad_yaw = math.radians(self.yaw)
                speed = 0.0015 * self.distance
                rx = -math.sin(rad_yaw) * dx * speed
                rz = math.cos(rad_yaw) * dx * speed
                self.target_goal[0] += rx
                self.target_goal[2] += rz
                self.target_goal[1] += dy * speed

    def apply(self):
        """Configura la matriz de vista gluLookAt."""
        rad_yaw = math.radians(self.yaw)
        rad_pitch = math.radians(self.pitch)
        
        eye_x = self.target[0] + self.distance * math.cos(rad_pitch) * math.sin(rad_yaw)
        eye_y = self.target[1] + self.distance * math.sin(rad_pitch)
        eye_z = self.target[2] + self.distance * math.cos(rad_pitch) * math.cos(rad_yaw)

        gluLookAt(
            eye_x, eye_y, eye_z,
            self.target[0], self.target[1], self.target[2],
            0.0, 1.0, 0.0
        )


# =====================================================================================
# RENDERIZADOR DE PRIMITIVAS 3D (OPENGL)
# =====================================================================================
class Primitive3D:
    """Funciones de dibujo geométrico optimizado con normales y materiales."""
    
    @staticmethod
    def draw_box(x, y, z, dx, dy, dz, color, alpha=1.0):
        glColor4f(color[0], color[1], color[2], alpha)
        hx, hy, hz = dx * 0.5, dy * 0.5, dz * 0.5
        
        glBegin(GL_QUADS)
        # Frente (+Z)
        glNormal3f(0.0, 0.0, 1.0)
        glVertex3f(x - hx, y - hy, z + hz)
        glVertex3f(x + hx, y - hy, z + hz)
        glVertex3f(x + hx, y + hy, z + hz)
        glVertex3f(x - hx, y + hy, z + hz)
        # Atrás (-Z)
        glNormal3f(0.0, 0.0, -1.0)
        glVertex3f(x + hx, y - hy, z - hz)
        glVertex3f(x - hx, y - hy, z - hz)
        glVertex3f(x - hx, y + hy, z - hz)
        glVertex3f(x + hx, y + hy, z - hz)
        # Superior (+Y)
        glNormal3f(0.0, 1.0, 0.0)
        glVertex3f(x - hx, y + hy, z - hz)
        glVertex3f(x - hx, y + hy, z + hz)
        glVertex3f(x + hx, y + hy, z + hz)
        glVertex3f(x + hx, y + hy, z - hz)
        # Inferior (-Y)
        glNormal3f(0.0, -1.0, 0.0)
        glVertex3f(x - hx, y - hy, z - hz)
        glVertex3f(x + hx, y - hy, z - hz)
        glVertex3f(x + hx, y - hy, z + hz)
        glVertex3f(x - hx, y - hy, z + hz)
        # Derecha (+X)
        glNormal3f(1.0, 0.0, 0.0)
        glVertex3f(x + hx, y - hy, z - hz)
        glVertex3f(x + hx, y + hy, z - hz)
        glVertex3f(x + hx, y + hy, z + hz)
        glVertex3f(x + hx, y - hy, z + hz)
        # Izquierda (-X)
        glNormal3f(-1.0, 0.0, 0.0)
        glVertex3f(x - hx, y - hy, z - hz)
        glVertex3f(x - hx, y - hy, z + hz)
        glVertex3f(x - hx, y + hy, z + hz)
        glVertex3f(x - hx, y + hy, z - hz)
        glEnd()

    @staticmethod
    def draw_cylinder(x, y, z, radius, height, color, axis='y', segments=16, alpha=1.0):
        a = color[3] if len(color) > 3 else alpha
        glColor4f(color[0], color[1], color[2], a)
        glPushMatrix()
        glTranslatef(x, y, z)
        if axis == 'x':
            glRotatef(90, 0, 0, 1)
        elif axis == 'z':
            glRotatef(90, 1, 0, 0)
        
        hh = height * 0.5
        glBegin(GL_QUAD_STRIP)
        for i in range(segments + 1):
            angle = 2.0 * math.pi * i / segments
            nx = math.cos(angle)
            nz = math.sin(angle)
            glNormal3f(nx, 0.0, nz)
            glVertex3f(nx * radius, -hh, nz * radius)
            glVertex3f(nx * radius, hh, nz * radius)
        glEnd()

        for sign, normal in [(-hh, -1.0), (hh, 1.0)]:
            glBegin(GL_TRIANGLE_FAN)
            glNormal3f(0.0, normal, 0.0)
            glVertex3f(0.0, sign, 0.0)
            for i in range(segments + 1):
                angle = 2.0 * math.pi * i / segments * (-normal)
                glVertex3f(math.cos(angle) * radius, sign, math.sin(angle) * radius)
            glEnd()
        glPopMatrix()

    @staticmethod
    def draw_rotating_roller(x, y, z, radius, length, angle_deg, color, num_stripes=8, stripe_color=None, flange_radius=0.0, segments=24):
        """
        Dibuja un rodillo cilíndrico rotativo industrial orientado sobre el eje Z con estrías de tracción,
        coronas de guiado lateral y tapas mecanizadas con patrón de giro visible.
        """
        glPushMatrix()
        glTranslatef(x, y, z)
        glRotatef(angle_deg, 0, 0, 1)

        hh = length * 0.5
        if stripe_color is None:
            stripe_color = (max(0.0, color[0] * 0.42), max(0.0, color[1] * 0.42), max(0.0, color[2] * 0.42))

        # 1. Manto cilíndrico principal con estrías longitudinales de tracción de alto contraste
        stride = max(1, segments // num_stripes) if num_stripes > 0 else 999
        glBegin(GL_QUADS)
        for i in range(segments):
            ang1 = 2.0 * math.pi * i / segments
            ang2 = 2.0 * math.pi * (i + 1) / segments
            x1, y1 = math.cos(ang1) * radius, math.sin(ang1) * radius
            x2, y2 = math.cos(ang2) * radius, math.sin(ang2) * radius
            nx1, ny1 = math.cos(ang1), math.sin(ang1)
            nx2, ny2 = math.cos(ang2), math.sin(ang2)

            is_stripe = (num_stripes > 0) and ((i % stride) == 0)
            c = stripe_color if is_stripe else color
            glColor3f(c[0], c[1], c[2])

            glNormal3f(nx1, ny1, 0.0)
            glVertex3f(x1, y1, -hh)
            glNormal3f(nx1, ny1, 0.0)
            glVertex3f(x1, y1, hh)
            glNormal3f(nx2, ny2, 0.0)
            glVertex3f(x2, y2, hh)
            glNormal3f(nx2, ny2, 0.0)
            glVertex3f(x2, y2, -hh)
        glEnd()

        # 2. Coronas de guiado lateral / bridas (si aplica)
        if flange_radius > radius:
            flange_w = 0.003
            c_flange = (color[0] * 0.88, color[1] * 0.88, color[2] * 0.88)
            for z_f in (-hh, hh - flange_w):
                glColor3f(c_flange[0], c_flange[1], c_flange[2])
                glBegin(GL_QUAD_STRIP)
                for i in range(segments + 1):
                    ang = 2.0 * math.pi * i / segments
                    nx, ny = math.cos(ang), math.sin(ang)
                    glNormal3f(nx, ny, 0.0)
                    glVertex3f(nx * flange_radius, ny * flange_radius, z_f)
                    glVertex3f(nx * flange_radius, ny * flange_radius, z_f + flange_w)
                glEnd()

        # 3. Tapas mecanizadas en los extremos con orificios/rayas radiales de giro
        c_cap = (0.82, 0.84, 0.88)
        for sign, z_pos in [(-1.0, -hh), (1.0, hh)]:
            glBegin(GL_TRIANGLE_FAN)
            glNormal3f(0.0, 0.0, sign)
            glColor3f(c_cap[0], c_cap[1], c_cap[2])
            glVertex3f(0.0, 0.0, z_pos)
            for i in range(segments + 1):
                ang = 2.0 * math.pi * i / segments * sign
                nx, ny = math.cos(ang), math.sin(ang)
                if (i % max(1, (segments // 4))) == 0:
                    glColor3f(0.20, 0.22, 0.25)
                else:
                    glColor3f(c_cap[0], c_cap[1], c_cap[2])
                glVertex3f(nx * radius, ny * radius, z_pos)
            glEnd()

            # Centro del cubo del eje
            glColor3f(0.28, 0.30, 0.33)
            glBegin(GL_TRIANGLE_FAN)
            glNormal3f(0.0, 0.0, sign)
            glVertex3f(0.0, 0.0, z_pos + sign * 0.0005)
            for i in range(13):
                ang = 2.0 * math.pi * i / 12 * sign
                glVertex3f(math.cos(ang) * radius * 0.35, math.sin(ang) * radius * 0.35, z_pos + sign * 0.0005)
            glEnd()

        glPopMatrix()

    @staticmethod
    def draw_cone(x, y, z, r_base, height, color, alpha=0.3, segments=20):
        glDisable(GL_LIGHTING)
        glDepthMask(GL_FALSE)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE)
        
        glPushMatrix()
        glTranslatef(x, y, z)
        
        # Cono volumétrico translúcido apuntando hacia -X (hacia la faja y las cajas entrantes)
        glColor4f(color[0], color[1], color[2], alpha)
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(0.0, 0.0, 0.0)  # Vértice en la cara del sensor HC-SR04
        for i in range(segments + 1):
            angle = 2.0 * math.pi * i / segments
            glVertex3f(-height, math.sin(angle) * r_base, math.cos(angle) * r_base)
        glEnd()

        # Anillo perimetral en la base del haz acústico incidente
        glColor4f(color[0], color[1], color[2], min(1.0, alpha * 2.5))
        glLineWidth(1.8)
        glBegin(GL_LINE_LOOP)
        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            glVertex3f(-height, math.sin(angle) * r_base, math.cos(angle) * r_base)
        glEnd()

        glPopMatrix()
        glDisable(GL_BLEND)
        glDepthMask(GL_TRUE)
        glEnable(GL_LIGHTING)

    @staticmethod
    def draw_rod(p1, p2, radius, color, segments=12):
        """Dibuja una barra cilíndrica rígida orientada entre dos puntos 3D arbitrarios p1 y p2."""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dz = p2[2] - p1[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist < 1e-5:
            return

        glPushMatrix()
        glTranslatef(p1[0], p1[1], p1[2])

        ux, uy, uz = dx / dist, dy / dist, dz / dist
        ax = uz
        ay = 0.0
        az = -ux
        a_len = math.sqrt(ax*ax + az*az)

        dot = max(-1.0, min(1.0, uy))
        angle_deg = math.degrees(math.acos(dot))

        if a_len > 1e-5:
            glRotatef(angle_deg, ax / a_len, 0.0, az / a_len)
        elif uy < 0.0:
            glRotatef(180.0, 1.0, 0.0, 0.0)

        glColor3f(color[0], color[1], color[2])
        glBegin(GL_QUAD_STRIP)
        for i in range(segments + 1):
            ang = 2.0 * math.pi * i / segments
            cx = math.cos(ang) * radius
            cz = math.sin(ang) * radius
            glNormal3f(math.cos(ang), 0.0, math.sin(ang))
            glVertex3f(cx, 0.0, cz)
            glVertex3f(cx, dist, cz)
        glEnd()

        glPopMatrix()


# =====================================================================================
# CARGADOR DE MODELOS OBJ OPTIMIZADO (GPU DISPLAY LISTS)
# =====================================================================================
class CADModelLoader:
    """Carga y compila modelos CAD en VRAM para 120 FPS sin consumo de CPU."""
    def __init__(self, obj_filename):
        self.vertices = []
        self.normals = []
        self.materials = {}
        self.faces = []
        self.d_list = None

        mtl_file = None
        dir_path = os.path.dirname(obj_filename)

        with open(obj_filename, 'r', encoding='utf-8', errors='ignore') as f:
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

    def compile(self, exclude_materials=None):
        """Compila las caras en una Display List de hardware en la GPU, permitiendo excluir piezas dinámicas."""
        if exclude_materials is None:
            exclude_materials = set()
        self.d_list = glGenLists(1)
        glNewList(self.d_list, GL_COMPILE)

        mat_faces = {}
        for mat_name, face in self.faces:
            if mat_name in exclude_materials:
                continue
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

    def draw(self):
        if self.d_list is not None:
            glCallList(self.d_list)


# =====================================================================================
# CLASE CAJA (OBJETO FÍSICO DINÁMICO)
# =====================================================================================
class DynamicBox:
    """Pieza de producto que viaja a lo largo de la estación desde tolva a carro."""
    def __init__(self, quality: str = "CONFORME"):
        self.quality = quality           # "CONFORME", "RETRABAJO", "RECHAZO"
        self.pos_x = -0.42               # Posición bajo la boca de la tolva
        self.pos_y = 0.54                # Inicia en caída libre por gravedad
        self.pos_z = 0.0                 # Posición transversal centrada
        self.size = (0.055, 0.045, 0.055)# Cajas cúbicas normalizadas para la banda útil de 80 mm
        
        # Estados: "CAYENDO_DE_TOLVA", "EN_FAJA", "EN_DESVIO", "FINALIZADA"
        self.state = "CAYENDO_DE_TOLVA"
        self.target_chute = 2            # 1: Retrabajo, 2: Conforme, 3: Rechazo
        self.container_offset_z = 0.0    # Offset local dentro del carro indexador
        self.rotation_y = 0.0
        self.rotation_z = 0.0
        self.settle_timer = 0.0

        if self.quality == "RETRABAJO":
            self.target_chute = 1
            self.container_offset_z = -0.075 # Contenedor Ámbar en Z = -0.075 m
        elif self.quality == "RECHAZO":
            self.target_chute = 3
            self.container_offset_z = 0.075  # Contenedor Rojo en Z = +0.075 m
        else:
            self.target_chute = 2
            self.container_offset_z = 0.0    # Contenedor Verde en el centro Z = 0.0

        self._update_colors()

    def _update_colors(self):
        if self.quality == "CONFORME":
            self.color = (0.28, 0.72, 0.32)   # Verde ingeniería
        elif self.quality == "RETRABAJO":
            self.color = (0.95, 0.70, 0.15)   # Ámbar técnico
        else:
            self.color = (0.85, 0.22, 0.20)   # Rojo descarte

    def update(self, dt: float, belt_speed: float, cart_z: float = 0.0):
        if self.state == "CAYENDO_DE_TOLVA":
            self.pos_y -= 0.65 * dt
            if self.pos_y <= 0.4255:
                self.pos_y = 0.4255
                self.state = "EN_FAJA"

        elif self.state == "EN_FAJA":
            # Avanzar sobre la banda hacia +X con tangencia perfecta a Y = 0.4255 m
            self.pos_x += belt_speed * dt
            self.pos_y = 0.4255
            self.pos_z = 0.0
            self.rotation_y = 0.0
            self.rotation_z = 0.0

            # Al aproximarse a la cresta del rodillo motriz (X >= 0.400 m) inicia transición fluida
            if self.pos_x >= 0.400:
                self.state = "EN_DESVIO"

        elif self.state == "EN_DESVIO":
            # Transición suave sin choques sobre la curvatura del rodillo motriz hacia la rampa (-52°)
            tan52 = 1.27994
            if self.pos_x < 0.435:
                t = (self.pos_x - 0.400) / 0.035
                t = max(0.0, min(1.0, t))
                s = t * t * (3.0 - 2.0 * t)  # Interpolación cúbica suave (smoothstep)

                # Inclinación angular progresiva continua (evita choques angulares con el rodillo)
                self.rotation_z = -52.0 * s
                
                # Velocidad de arrastre acoplada entre banda motriz y deslizamiento por gravedad
                adv_speed = belt_speed * (1.0 - s) + 0.28 * s
                self.pos_x += max(0.12, adv_speed) * dt

                # Altura adaptativa continua que mantiene la base de la caja en contacto suave
                chute_plane_y = 0.404 - (self.pos_x - 0.410) * tan52
                normal_h = 0.0225 / max(0.2, math.cos(math.radians(abs(self.rotation_z))))
                self.pos_y = (1.0 - s) * 0.4255 + s * (chute_plane_y + normal_h)
            else:
                # Descenso continuo regular por la chapa de la rampa (-52°)
                self.rotation_z = -52.0
                self.pos_x += 0.28 * dt
                chute_plane_y = 0.404 - (self.pos_x - 0.410) * tan52
                normal_h = 0.0225 / 0.61566  # cos(52°) = 0.61566
                self.pos_y = chute_plane_y + normal_h

            self.pos_z = 0.0

            if self.pos_y <= 0.18:
                self.pos_y = 0.18
                self.pos_x = 0.58
                self.rotation_z = 0.0
                self.state = "FINALIZADA"

        elif self.state == "FINALIZADA":
            # Reposa dentro del contenedor y se indexa solidariamente con el carro móvil
            self.pos_x = 0.58
            self.pos_y = 0.18
            self.pos_z = cart_z + self.container_offset_z
            self.rotation_y = 0.0
            self.rotation_z = 0.0
            self.settle_timer += dt

    def draw(self):
        glPushMatrix()
        glTranslatef(self.pos_x, self.pos_y, self.pos_z)
        glRotatef(self.rotation_y, 0, 1, 0)
        glRotatef(self.rotation_z, 0, 0, 1)
        
        # Caja de cartón con tinte técnico
        Primitive3D.draw_box(0, 0, 0, self.size[0], self.size[1], self.size[2], self.color)
        
        # Etiqueta de código de barras superior
        Primitive3D.draw_box(0, self.size[1]*0.5 + 0.001, 0, 0.030, 0.001, 0.030, (0.95, 0.95, 0.95))
        # Cinta adhesiva central
        Primitive3D.draw_box(0, self.size[1]*0.5 + 0.0015, 0, self.size[0]*0.9, 0.001, 0.010, (0.75, 0.60, 0.40))
        glPopMatrix()


# =====================================================================================
# MODELO 3D DE LA ESTACIÓN LOGÍSTICA (GRABCAD ASSEMBLY INTEGRATED)
# =====================================================================================
class LogisticsStationModel:
    """Construcción geométrica CAD de la estación logística industrial."""
    def __init__(self):
        # Variables de cinemática continua de la faja y rodillos
        self.belt_distance = 0.0           # Desplazamiento lineal continuo acumulado (m)
        self.drive_roller_angle = 0.0      # Ángulo del rodillo motriz y tensor (grados)
        self.top_roller_angle = 0.0        # Ángulo de los rodillos intermedios superiores (grados)
        self.ret_roller_angle = 0.0        # Ángulo de los rodillos tensores de retorno (grados)
        self.emergency_pressed = False
        
        # Parámetros geométricos exactos de la estación GrabCAD
        self.BELT_L_SPAN = 0.820           # Distancia entre centros de rodillos motriz y tensor (m)
        self.BELT_R_OUTER = 0.023          # Radio exterior de la curva de la faja (m)
        self.BELT_R_INNER = 0.020          # Radio interior de la faja (m)
        self.BELT_WIDTH = 0.076            # Ancho útil de la banda (m)
        self.BELT_X_LEFT = -0.410          # Eje rodillo tensor (m)
        self.BELT_X_RIGHT = 0.410          # Eje rodillo motriz (m)
        self.BELT_Y_CENTER = 0.380         # Altura del eje de rodillos principales (m)
        self.BELT_L_CURVE = math.pi * self.BELT_R_OUTER
        self.BELT_L_TOTAL = 2.0 * self.BELT_L_SPAN + 2.0 * self.BELT_L_CURVE

        # Paleta de colores industriales
        self.C_ALU = (0.78, 0.80, 0.83)        # Perfil de aluminio anodizado
        self.C_MOTOR = (0.12, 0.45, 0.72)      # Azul motor eléctrico
        self.C_STAND = (0.25, 0.28, 0.32)      # Pedestal operador

        # Cargar y compilar el ensamble CAD excluyendo la banda, rodillos estáticos,
        # sensor low-poly, placa flotante de 10 agujeros y motor paso a paso NEMA 17
        # para reemplazarlos por los componentes mecatrónicos reales del proyecto
        modelsd_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "modelsd")
        obj_file = os.path.join(modelsd_dir, "faja_modular_estacion_3d.obj")
        self.cad_loader = None
        if os.path.exists(obj_file):
            try:
                self.cad_loader = CADModelLoader(obj_file)
                self.cad_loader.compile(exclude_materials={
                    'Material_Belt', 'Material_Roller_Red', 'Material_Coupler_Blue',
                    'Material_PCB_Blue', 'Material_Alu_Can', 'Material_Mesh_Dark', 'Material_Crystal',
                    'Material_Sensor_Plate_GrabCAD', 'Material_Motor_Black',
                    'Material_Screw_Inox'
                })
            except Exception as e:
                print(f"[AVISO CAD] No se pudo compilar Display List: {e}")

        # Cargar y compilar el Sensor Ultrasónico HC-SR04 GrabCAD de alta precisión (3,518 caras con PBR)
        sensor_obj = os.path.join(modelsd_dir, "HC-SR04-UltrasonicSensor-meters.obj")
        self.sensor_loader = None
        if os.path.exists(sensor_obj):
            try:
                self.sensor_loader = CADModelLoader(sensor_obj)
                self.sensor_loader.compile()
            except Exception as e:
                print(f"[AVISO CAD SENSOR] No se pudo compilar Sensor HC-SR04: {e}")

    def draw_floor(self):
        """Suelo técnico de taller y cuadrícula métrica ampliada."""
        # Suelo base ampliado que cubre todo el campo visual
        Primitive3D.draw_box(0.0, -0.01, 0.0, 7.0, 0.02, 6.0, (0.90, 0.92, 0.94))
        
        glDisable(GL_LIGHTING)
        glColor3f(0.80, 0.82, 0.85)
        glLineWidth(1.0)
        glBegin(GL_LINES)
        for i in range(-30, 31):
            glVertex3f(i * 0.1, 0.001, -2.5)
            glVertex3f(i * 0.1, 0.001, 2.5)
            glVertex3f(-2.5, 0.001, i * 0.1)
            glVertex3f(2.5, 0.001, i * 0.1)
        glEnd()

        # Perímetro de seguridad amarillo
        glColor3f(0.92, 0.78, 0.15)
        glLineWidth(3.0)
        glBegin(GL_LINE_LOOP)
        glVertex3f(-0.70, 0.002, -0.50)
        glVertex3f(0.80, 0.002, -0.50)
        glVertex3f(0.80, 0.002, 0.50)
        glVertex3f(-0.70, 0.002, 0.50)
        glEnd()
        glEnable(GL_LIGHTING)

    def update_animations(self, belt_speed: float, dt: float):
        """Actualiza la cinemática acoplada continua de la faja y de todos los rodillos."""
        # 1. Avance continuo perimetral en bucle cerrado
        self.belt_distance = (self.belt_distance + belt_speed * dt) % self.BELT_L_TOTAL
        
        # 2. Rodillo motriz y rodillo de retorno (R = 0.0195 m)
        # Giro horario (negativo en OpenGL sobre el eje Z) para avanzar hacia +X en la cara superior
        w_main = (belt_speed / 0.0195) * (180.0 / math.pi)
        self.drive_roller_angle = (self.drive_roller_angle + w_main * dt) % 360.0
        
        # 3. Rodillos intermedios de cama superior (r = 0.010 m)
        w_top = (belt_speed / 0.010) * (180.0 / math.pi)
        self.top_roller_angle = (self.top_roller_angle + w_top * dt) % 360.0
        
        # 4. Rodillos tensores de retorno inferior (r = 0.008 m)
        # La rama inferior avanza hacia -X -> tangencia en la cara superior del rodillo hacia -X
        # por lo que el rodillo gira antihorario (positivo en OpenGL sobre el eje Z)
        w_ret = (belt_speed / 0.008) * (180.0 / math.pi)
        self.ret_roller_angle = (self.ret_roller_angle + w_ret * dt) % 360.0

    def draw_conveyor(self, belt_speed: float, dt: float):
        """Dibuja el ensamble maestro GrabCAD con cinemática sincronizada y faja continua."""
        self.update_animations(belt_speed, dt)

        # 1. Chasis estructural estático compilado en GPU VRAM
        if self.cad_loader:
            self.cad_loader.draw()

        # 2. Sensor ultrasónico HC-SR04 GrabCAD montado en soporte dedicado (sin placa flotante)
        self.draw_ultrasonic_sensor()

        # 3. Todos los rodillos de transporte activos con rotación visible y sin solapamiento
        self.draw_all_transport_rollers()

        # 4. Banda transportadora continua en bucle cerrado con nervaduras y cantos de guiado
        self.draw_endless_belt()

        # 5. Motorreductor DC de tracción (Lazo 1) acoplado al eje del rodillo motriz
        self.draw_dc_gearmotor()

        # 6. Módulo Driver Puente H Dual L298N (modelo GrabCAD exacto con disipador, borneras y jumpers)
        self.draw_l298n_driver()

    def draw_ultrasonic_sensor(self):
        """
        Renderiza el sensor ultrasónico HC-SR04 GrabCAD de alta precisión (3,518 caras PBR)
        montado sobre un soporte tipo puente aéreo ultra-delgado (2 mm) en PLA azul técnico.
        Diseño mecatrónico optimizado:
        - Apoyado lateralmente en los brazos del bastidor a Z = ±0.065 m (completamente fuera de la rampa).
        - Espacio inferior (Y < 0.415 m) 100% despejado para el paso y caída fluida de las cajas sin choques.
        - Placa frontal delgada de 2.0 mm con perforaciones y biseles calibrados para los transductores T/R de 16 mm.
        - Elimina cualquier travesaño o pilar central bajo que obstaculice la trayectoria de desvío.
        """
        sx = 0.484
        sy = 0.4265
        sz = 0.0

        c_bracket_pla = (0.12, 0.42, 0.78)  # Azul técnico PLA
        c_screw = (0.84, 0.86, 0.89)        # Acero inox tornillos
        c_insert = (0.78, 0.68, 0.25)       # Insertos roscados de latón M2/M3

        # 1. MONTANTES LATERALES DELGADOS (Z = -0.065 m y +0.065 m)
        # Anclados al extremo de las placas de extensión del bastidor (X = 0.480 m, Y = 0.380 m)
        # Vano libre central de 126 mm (las cajas miden 55 mm de ancho, dejando 35 mm de margen a cada lado)
        for cz in (-0.065, 0.065):
            # Montante vertical estilizado (X = 0.481 m, Y = 0.375 a 0.438 m, espesor Z = 2.5 mm)
            Primitive3D.draw_box(0.481, 0.4065, cz, 0.008, 0.063, 0.0025, c_bracket_pla)
            # Cartela o nervadura triangular de refuerzo superior
            Primitive3D.draw_box(0.477, 0.432, cz, 0.008, 0.012, 0.0025, c_bracket_pla)
            # Tornillo Allen M3 de fijación al bastidor a Y = 0.380 m
            Primitive3D.draw_cylinder(0.481, 0.380, cz + (0.0018 if cz > 0 else -0.0018), 0.0025, 0.002, c_screw, axis='z', segments=8)
            # Arandela plana de inoxidable
            Primitive3D.draw_cylinder(0.481, 0.380, cz + (0.0014 if cz > 0 else -0.0014), 0.0035, 0.0005, (0.75, 0.77, 0.80), axis='z', segments=8)

        # 2. PLACA FRONTAL PORTA-SENSOR ULTRA-DELGADA (Espesor 2.0 mm, Y = 0.415 a 0.439 m)
        # Puente superior suspendido entre montantes (X = 0.483 m, centro Y = 0.427 m)
        # La cota inferior queda a Y = 0.415 m, dejando más de 34 mm de luz vertical libre sobre las cajas
        plate_w = 0.130  # Ancho total que une ambos montantes en Z = ±0.065
        Primitive3D.draw_box(0.483, sy, 0.0, 0.0020, 0.024, plate_w, c_bracket_pla)

        # Biseles / rebajes estéticos alrededor de las cápsulas ultrasónicas (Ø 16.5 mm)
        for tz in (-0.013, 0.013):
            Primitive3D.draw_cylinder(0.4820, sy, tz, 0.0088, 0.0012, (0.08, 0.32, 0.65), axis='x', segments=16)

        # 4x Mini-tornillos de fijación M2 para la placa PCB del HC-SR04 en esquinas
        for pz in (-0.0215, 0.0215):
            for py_off in (-0.008, 0.008):
                Primitive3D.draw_cylinder(0.4818, sy + py_off, pz, 0.0014, 0.0010, c_screw, axis='x', segments=8)
                Primitive3D.draw_cylinder(0.4842, sy + py_off, pz, 0.0018, 0.0015, c_insert, axis='x', segments=6)

        # 3. MODELO 3D GRABCAD DEL SENSOR HC-SR04 (3,518 caras PBR)
        if self.sensor_loader:
            glPushMatrix()
            # Posicionamiento milimétrico: transductores mirando a -X
            glTranslatef(sx, sy, sz)
            glRotatef(-90, 0, 1, 0)
            self.sensor_loader.draw()
            glPopMatrix()

    def draw_all_transport_rollers(self):
        """Dibuja todos los rodillos de transporte con cinemática sincronizada y sin interferencias físicas."""
        c_red_roller = (0.84, 0.22, 0.18)      # Rojo industrial brillante
        c_red_groove = (0.38, 0.08, 0.06)      # Estría de agarre rojo oscuro
        c_steel_roller = (0.80, 0.82, 0.85)    # Acero inox rectificado
        c_steel_groove = (0.45, 0.47, 0.50)    # Ranura mecanizada acero
        c_shaft = (0.92, 0.94, 0.96)           # Eje brillante pasante
        c_collar = (0.35, 0.38, 0.42)          # Rodamiento 608ZZ
        c_bracket = (0.22, 0.24, 0.28)         # Soporte T-Slot negro satinado

        # A. RODILLO TENSOR DE RETORNO (Cola / Izquierdo en X = -0.410, Y = 0.380)
        # Radio 0.0195 m, pestaña 0.0220 m, ancho 0.076 m
        Primitive3D.draw_rotating_roller(
            self.BELT_X_LEFT, self.BELT_Y_CENTER, 0.0,
            radius=0.0195, length=self.BELT_WIDTH,
            angle_deg=-self.drive_roller_angle,
            color=c_red_roller, num_stripes=8, stripe_color=c_red_groove,
            flange_radius=0.0220, segments=24
        )

        # B. RODILLO MOTRIZ DE ARRASTRE (Cabezal / Derecho en X = +0.410, Y = 0.380)
        Primitive3D.draw_rotating_roller(
            self.BELT_X_RIGHT, self.BELT_Y_CENTER, 0.0,
            radius=0.0195, length=self.BELT_WIDTH,
            angle_deg=-self.drive_roller_angle,
            color=c_red_roller, num_stripes=8, stripe_color=c_red_groove,
            flange_radius=0.0220, segments=24
        )


        # C. CINCO RODILLOS INTERMEDIOS DE APOYO SUPERIOR (Cama de deslizamiento)
        # La cara inferior de la banda útil corre a Y = 0.400. Con radio r = 0.010 m,
        # su centro se ubica a Y = 0.390 m (tangente exacta sin penetración ni holgura).
        top_y = 0.390
        top_x_positions = [-0.27, -0.13, 0.00, 0.13, 0.27]
        for rx in top_x_positions:
            # Eje pasante de acero de 6 mm
            Primitive3D.draw_cylinder(rx, top_y, 0.0, 0.003, 0.114, c_shaft, axis='z', segments=12)
            # Rodillo giratorio de acero inoxidable con estrías de rotación visible
            Primitive3D.draw_rotating_roller(
                rx, top_y, 0.0,
                radius=0.010, length=self.BELT_WIDTH,
                angle_deg=-self.top_roller_angle,
                color=c_steel_roller, num_stripes=6, stripe_color=c_steel_groove,
                segments=20
            )
            # Rodamientos miniatura 608ZZ en los extremos
            Primitive3D.draw_cylinder(rx, top_y, -0.040, 0.006, 0.004, c_collar, axis='z', segments=12)
            Primitive3D.draw_cylinder(rx, top_y,  0.040, 0.006, 0.004, c_collar, axis='z', segments=12)
            # Bridas de anclaje a las ranuras T-Slot del perfil 2020
            Primitive3D.draw_box(rx, top_y - 0.004, -0.048, 0.012, 0.012, 0.008, c_bracket)
            Primitive3D.draw_box(rx, top_y - 0.004,  0.048, 0.012, 0.012, 0.008, c_bracket)

        # D. DOS RODILLOS TENSORES INFERIORES DE RETORNO
        # La cara exterior inferior de la cinta de retorno corre a Y = 0.357.
        # El rodillo de retorno apoya la cinta desde abajo (radio r = 0.008 m),
        # por lo que su centro está en Y = 0.357 - 0.008 = 0.349 m.
        # ¡Tangencia exacta por la cara externa, CERO penetración con la faja!
        ret_y = 0.349
        ret_x_positions = [-0.20, 0.20]
        for rx in ret_x_positions:
            # Eje pasante
            Primitive3D.draw_cylinder(rx, ret_y, 0.0, 0.003, 0.114, c_shaft, axis='z', segments=12)
            # Rodillo giratorio de retorno
            Primitive3D.draw_rotating_roller(
                rx, ret_y, 0.0,
                radius=0.008, length=self.BELT_WIDTH,
                angle_deg=self.ret_roller_angle,
                color=(0.74, 0.76, 0.80), num_stripes=4, stripe_color=(0.42, 0.44, 0.48),
                segments=16
            )
            # Bridas de sujeción al perfil inferior
            Primitive3D.draw_box(rx, ret_y - 0.006, -0.048, 0.010, 0.014, 0.008, c_bracket)
            Primitive3D.draw_box(rx, ret_y - 0.006,  0.048, 0.010, 0.014, 0.008, c_bracket)

    def draw_endless_belt(self):
        """Dibuja la banda transportadora completa en bucle cerrado continuo con nervaduras móviles."""
        # 1. CUERPO BASE DE LA BANDA CONTINUA (Caucho sintético / poliuretano industrial mate)
        c_belt_top = (0.24, 0.28, 0.33)    # Cara superior útil
        c_belt_bot = (0.19, 0.22, 0.26)    # Cara inferior de retorno
        c_belt_side = (0.14, 0.16, 0.19)   # Costados laterales de la banda
        hw = self.BELT_WIDTH * 0.5         # 0.038 m

        # A. Tramo superior útil (X = -0.410 a +0.410, Y = 0.400 a 0.403)
        Primitive3D.draw_box(
            0.0, (0.400 + 0.403) * 0.5, 0.0,
            self.BELT_L_SPAN, 0.003, self.BELT_WIDTH,
            c_belt_top
        )

        # B. Tramo inferior de retorno (X = -0.410 a +0.410, Y = 0.357 a 0.360)
        Primitive3D.draw_box(
            0.0, (0.357 + 0.360) * 0.5, 0.0,
            self.BELT_L_SPAN, 0.003, self.BELT_WIDTH,
            c_belt_bot
        )

        # C. Curvas envolventes en los extremos de rodillos (Cabezal derecho y Cola izquierda)
        # Semicilindro en X = +0.410 (ángulo 0 a pi)
        self._draw_belt_curved_turnaround(self.BELT_X_RIGHT, self.BELT_Y_CENTER, hw, 0.0, math.pi, c_belt_top, c_belt_side)
        # Semicilindro en X = -0.410 (ángulo pi a 2*pi)
        self._draw_belt_curved_turnaround(self.BELT_X_LEFT, self.BELT_Y_CENTER, hw, math.pi, 2.0 * math.pi, c_belt_top, c_belt_side)

        # 2. NERVADURAS DE TRACCIÓN Y MÓDULOS MÓVILES EN BUCLE CERRADO
        # Viajan continuamente sin saltos, envolviendo los rodillos con cinemática exacta
        num_ribs = 48
        spacing = self.BELT_L_TOTAL / num_ribs
        rib_hw = hw - 0.002
        c_rib = (0.13, 0.15, 0.18)
        c_edge = (0.86, 0.72, 0.18)  # Bordes de seguridad amarillos / tracking de alta visibilidad

        glDisable(GL_LIGHTING)
        glLineWidth(2.2)

        # A. Nervaduras transversales 3D elevadas
        glBegin(GL_LINES)
        for i in range(num_ribs):
            s_i = (self.belt_distance + i * spacing) % self.BELT_L_TOTAL
            px, py, nx, ny = self._get_belt_point(s_i)
            # Elevación sutil de 1.2 mm sobre la superficie normal
            rx = px + nx * 0.0012
            ry = py + ny * 0.0012

            glColor3f(c_rib[0], c_rib[1], c_rib[2])
            glVertex3f(rx, ry, -rib_hw)
            glVertex3f(rx, ry,  rib_hw)

            # B. Marcadores de guiado lateral en los bordes
            glColor3f(c_edge[0], c_edge[1], c_edge[2])
            glVertex3f(rx, ry, -rib_hw)
            glVertex3f(rx, ry, -rib_hw + 0.004)
            glVertex3f(rx, ry,  rib_hw - 0.004)
            glVertex3f(rx, ry,  rib_hw)
        glEnd()

        glEnable(GL_LIGHTING)

    def _draw_belt_curved_turnaround(self, cx: float, cy: float, hw: float, start_ang: float, end_ang: float, c_face: Tuple[float, float, float], c_side: Tuple[float, float, float], segments: int = 16):
        """Dibuja la curvatura envolvente semicilíndrica de la banda alrededor de los rodillos principales."""
        r_out = self.BELT_R_OUTER
        r_in = self.BELT_R_INNER
        
        # Superficie cilíndrica exterior
        glColor3f(c_face[0], c_face[1], c_face[2])
        glBegin(GL_QUAD_STRIP)
        for i in range(segments + 1):
            ang = start_ang + (end_ang - start_ang) * (i / segments)
            nx = math.sin(ang)
            ny = math.cos(ang)
            glNormal3f(nx, ny, 0.0)
            glVertex3f(cx + nx * r_out, cy + ny * r_out, -hw)
            glVertex3f(cx + nx * r_out, cy + ny * r_out,  hw)
        glEnd()

        # Tapas de los cantos laterales de la banda en Z = -hw y +hw
        glColor3f(c_side[0], c_side[1], c_side[2])
        for sign, z_pos in [(-1.0, -hw), (1.0, hw)]:
            glBegin(GL_QUAD_STRIP)
            glNormal3f(0.0, 0.0, sign)
            for i in range(segments + 1):
                ang = start_ang + (end_ang - start_ang) * (i / segments)
                nx = math.sin(ang)
                ny = math.cos(ang)
                glVertex3f(cx + nx * r_in,  cy + ny * r_in,  z_pos)
                glVertex3f(cx + nx * r_out, cy + ny * r_out, z_pos)
            glEnd()

    def _get_belt_point(self, s: float) -> Tuple[float, float, float, float]:
        """Calcula posición (x, y) y normal unitaria (nx, ny) a lo largo del bucle cerrado continuo."""
        s = s % self.BELT_L_TOTAL
        
        # Tramo 1: Cama superior (+X)
        if s < self.BELT_L_SPAN:
            x = self.BELT_X_LEFT + s
            y = self.BELT_Y_CENTER + self.BELT_R_OUTER
            return x, y, 0.0, 1.0

        # Tramo 2: Rodillo motriz derecho (curva horaria de arriba a abajo)
        s -= self.BELT_L_SPAN
        if s < self.BELT_L_CURVE:
            ang = s / self.BELT_R_OUTER
            x = self.BELT_X_RIGHT + self.BELT_R_OUTER * math.sin(ang)
            y = self.BELT_Y_CENTER + self.BELT_R_OUTER * math.cos(ang)
            return x, y, math.sin(ang), math.cos(ang)

        # Tramo 3: Cama inferior de retorno (-X)
        s -= self.BELT_L_CURVE
        if s < self.BELT_L_SPAN:
            x = self.BELT_X_RIGHT - s
            y = self.BELT_Y_CENTER - self.BELT_R_OUTER
            return x, y, 0.0, -1.0

        # Tramo 4: Rodillo tensor izquierdo (curva horaria de abajo a arriba)
        s -= self.BELT_L_SPAN
        ang = s / self.BELT_R_OUTER
        x = self.BELT_X_LEFT - self.BELT_R_OUTER * math.sin(ang)
        y = self.BELT_Y_CENTER - self.BELT_R_OUTER * math.cos(ang)
        return x, y, -math.sin(ang), -math.cos(ang)

    def draw_sonar_wave(self, measured_dist_cm: float):
        """Cono volumétrico sonar del sensor HC-SR04 incidiendo a 90° contra el flujo (-X)."""
        # Cápsulas del sensor en X = +0.4718, Y = 0.4265, Z = 0.0, apuntando hacia -X
        cone_len = max(0.04, min(0.38, measured_dist_cm * 0.01))
        pulse = 0.015 * math.sin(time.time() * 8.0)
        Primitive3D.draw_cone(0.4718, 0.4265, 0.0, 0.040 + pulse, cone_len, (0.2, 0.7, 1.0), alpha=0.18)

    def draw_inspection_camera(self, led_pwm: int):
        """
        Estación Fotométrica e Inspección Lumínica (Lazo 3) en X = +0.355 m.
        Basada 100% en el inventario real:
        - Campana / Túnel en arco impreso en 3D (PLA rojo/carbón) sobre perfilería 2020.
        - Banco de 4 LEDs de 5 mm en matriz 2x2 en el techo (QL-103 / Pin D6 vía transistor NPN TIP120).
        - Sensor Fotorresistencia LDR de 5 mm discreto (QE-103 / Pin A0 con divisor 10 kΩ) a 45°
          apuntando a la cara superior de la caja sobre la faja.
        - Cableado jumper (Macho-Hembra) ordenado hacia la protoboard de control.
        """
        gx = 0.355  # Eje central de inspección (11.5 cm antes del sensor ultrasónico)

        # Paleta de materiales de maqueta Maker
        c_pla = (0.80, 0.18, 0.14)          # PLA Rojo técnico de impresión 3D (igual a soportes faja)
        c_pla_dark = (0.24, 0.26, 0.30)     # PLA Negro satinado / insertos
        c_screw = (0.78, 0.80, 0.84)        # Tornillos Allen M4 inox
        c_wire_red = (0.85, 0.15, 0.15)     # Jumper positivo 5V
        c_wire_blk = (0.15, 0.16, 0.18)     # Jumper GND
        c_wire_grn = (0.15, 0.72, 0.25)     # Jumper señal A0 (LDR)

        # -------------------------------------------------------------------------
        # 1. PIES NIVELADORES Y COLUMNAS ESTRUCTURALES AL SUELO (Y = 0.0 m)
        # -------------------------------------------------------------------------
        # Evita que el pórtico flote en el aire: dos columnas T-Slot 2020 completas hasta el suelo
        for cz in (-0.055, 0.055):
            # Base amortiguadora de caucho vulcanizado en el suelo
            Primitive3D.draw_cylinder(gx, 0.003, cz, 0.016, 0.006, (0.16, 0.17, 0.19), axis='y', segments=14)
            # Espárrago roscado M10 y tuerca hexagonal de bloqueo
            Primitive3D.draw_cylinder(gx, 0.013, cz, 0.005, 0.014, (0.82, 0.84, 0.87), axis='y', segments=10)
            Primitive3D.draw_cylinder(gx, 0.016, cz, 0.0085, 0.005, (0.88, 0.90, 0.92), axis='y', segments=6)
            # Placa de asiento de acero al carbono (45 x 45 x 6 mm)
            Primitive3D.draw_box(gx, 0.024, cz, 0.045, 0.006, 0.045, (0.32, 0.35, 0.38))
            # Columna estructural vertical 2020 (desde Y = 0.027 hasta 0.395 m, altura 0.368 m, centro Y = 0.211 m)
            Primitive3D.draw_box(gx, 0.211, cz, 0.020, 0.368, 0.020, self.C_ALU)
            # Escuadra de unión de 90° con el perfil longitudinal de la faja (a Y = 0.380 m)
            Primitive3D.draw_box(gx, 0.380, cz, 0.028, 0.024, 0.024, (0.35, 0.38, 0.42))

        # -------------------------------------------------------------------------
        # 2. CAMPANA / TÚNEL EN ARCO IMPRESO EN 3D (PLA) SOBRE PERFILES T-SLOT 2020
        # -------------------------------------------------------------------------
        # Zapatas de fijación a la ranura superior del perfil 2020 a Z = -0.055 y +0.055 m
        for cz in (-0.055, 0.055):
            # Bloque base con corredera para tuerca T
            Primitive3D.draw_box(gx, 0.405, cz, 0.044, 0.016, 0.022, c_pla)
            # Tornillos Allen M4 de fijación vertical
            Primitive3D.draw_cylinder(gx - 0.012, 0.414, cz, 0.0028, 0.004, c_screw, axis='y', segments=10)
            Primitive3D.draw_cylinder(gx + 0.012, 0.414, cz, 0.0028, 0.004, c_screw, axis='y', segments=10)

        # Columnas / paredes verticales del túnel
        # Altura: de Y = 0.413 a Y = 0.528 m (altura 0.115 m, centro Y = 0.4705 m)
        tunnel_thick = 0.007
        tunnel_len_x = 0.046
        Primitive3D.draw_box(gx, 0.4705, -0.0515, tunnel_len_x, 0.115, tunnel_thick, c_pla)
        Primitive3D.draw_box(gx, 0.4705,  0.0515, tunnel_len_x, 0.115, tunnel_thick, c_pla)

        # Nervios de refuerzo diagonal en los pilares exteriores
        for sign_z in (-1.0, 1.0):
            Primitive3D.draw_box(gx, 0.430, sign_z * 0.058, 0.030, 0.025, 0.006, c_pla)

        # Techo del cabezal porta-LEDs a Y = 0.528 m
        # Ancho 110 mm (Z de -0.055 a +0.055), fondo 46 mm
        Primitive3D.draw_box(gx, 0.528, 0.0, tunnel_len_x, 0.008, 0.110, c_pla)

        # Receso / bisel cónico reflector en el envés del techo en PLA blanco
        Primitive3D.draw_box(gx, 0.5235, 0.0, 0.034, 0.002, 0.056, (0.92, 0.93, 0.96))

        # -------------------------------------------------------------------------
        # 2. BANCO DE 4 LEDs DE 5 mm EN MATRIZ 2x2 (QL-103 / Pin D6 PWM vía TIP120)
        # -------------------------------------------------------------------------
        led_intensity = max(0.04, min(1.0, led_pwm / 255.0))
        # Los LEDs de 5 mm de la lista son amarillos/ámbar cálidos (longitud de onda óptima para LDR CdS)
        glow_r = min(1.0, 0.85 + 0.15 * led_intensity)
        glow_g = min(1.0, 0.65 + 0.32 * led_intensity)
        glow_b = min(1.0, 0.10 + 0.50 * led_intensity)
        c_led_epoxy = (glow_r, glow_g, glow_b, 0.85)

        # Coordenadas de los 4 LEDs de 5 mm en el techo
        led_x_offsets = (-0.0075, 0.0075)
        led_z_offsets = (-0.012, 0.012)
        y_roof_bot = 0.524  # Base de los LEDs en el envés del techo

        for lx_off in led_x_offsets:
            for lz_off in led_z_offsets:
                lx = gx + lx_off
                lz = lz_off

                # A. Reborde / pestaña de montaje (anillo base de 5.8 mm de diámetro)
                Primitive3D.draw_cylinder(lx, y_roof_bot, lz, 0.0029, 0.0012, (0.82, 0.68, 0.20), axis='y', segments=12)

                # B. Cuerpo cilíndrico de 5 mm de la cápsula epoxi
                # Altura = 0.0055 m (desde Y = 0.523 hasta 0.5175 m)
                Primitive3D.draw_cylinder(lx, y_roof_bot - 0.003, lz, 0.0025, 0.0055, c_led_epoxy, axis='y', segments=14)

                # C. Cúpula hemisférica / punta del LED en Y = 0.515 m
                Primitive3D.draw_cylinder(lx, y_roof_bot - 0.006, lz, 0.0019, 0.0018, (1.0, 0.95, 0.70), axis='y', segments=12)
                Primitive3D.draw_cylinder(lx, y_roof_bot - 0.007, lz, 0.0011, 0.0010, (1.0, 1.0, 0.90), axis='y', segments=8)

                # D. Patillas metálicas de ánodo y cátodo saliendo por arriba del techo
                Primitive3D.draw_cylinder(lx - 0.0010, 0.534, lz, 0.00035, 0.008, (0.75, 0.78, 0.82), axis='y', segments=6)
                Primitive3D.draw_cylinder(lx + 0.0010, 0.534, lz, 0.00035, 0.008, (0.75, 0.78, 0.82), axis='y', segments=6)

                # E. Fundas termorretráctiles / cables jumpers saliendo hacia la bancada
                Primitive3D.draw_cylinder(lx - 0.0010, 0.538, lz, 0.00075, 0.005, c_wire_red, axis='y', segments=6)
                Primitive3D.draw_cylinder(lx + 0.0010, 0.538, lz, 0.00075, 0.005, c_wire_blk, axis='y', segments=6)

        # -------------------------------------------------------------------------
        # 3. SENSOR FOTORRESISTOR LDR DE 5 mm DISCRETO (QE-103 / Pin A0)
        # -------------------------------------------------------------------------
        # Ubicado en la pared izquierda interna (Z = -0.044 m, Y = 0.455 m),
        # montado en un resalte impreso en 3D inclinado exactamente a 45°
        ldr_base_x = gx
        ldr_base_y = 0.455
        ldr_base_z = -0.044

        # Soporte / casquillo inclinado impreso a 45°
        glPushMatrix()
        glTranslatef(ldr_base_x, ldr_base_y, ldr_base_z)
        glRotatef(45.0, 1, 0, 0)  # Gira local +Z hacia +Z/-Y (apuntando al centro de la caja)

        # Casquillo soporte de PLA negro
        Primitive3D.draw_cylinder(0.0, 0.0, -0.003, 0.0042, 0.008, c_pla_dark, axis='z', segments=12)

        # Substrato cerámico blanco de alúmina (disco de 5 mm de diámetro, 1.8 mm espesor)
        Primitive3D.draw_cylinder(0.0, 0.0, 0.001, 0.0025, 0.0018, (0.94, 0.92, 0.88), axis='z', segments=16)

        # Pista serpentina fotosensible de Sulfuro de Cadmio (CdS) en naranja/dorado cobrizo
        glDisable(GL_LIGHTING)
        glLineWidth(1.6)
        glColor3f(0.88, 0.46, 0.12)
        glBegin(GL_LINE_STRIP)
        glVertex3f(-0.0018, -0.0014, 0.0020)
        glVertex3f(-0.0018,  0.0014, 0.0020)
        glVertex3f(-0.0009,  0.0014, 0.0020)
        glVertex3f(-0.0009, -0.0014, 0.0020)
        glVertex3f( 0.0,    -0.0014, 0.0020)
        glVertex3f( 0.0,     0.0014, 0.0020)
        glVertex3f( 0.0009,  0.0014, 0.0020)
        glVertex3f( 0.0009, -0.0014, 0.0020)
        glVertex3f( 0.0018, -0.0014, 0.0020)
        glVertex3f( 0.0018,  0.0014, 0.0020)
        glEnd()
        glEnable(GL_LIGHTING)

        # Cúpula de resina epoxi transparente brillante que cubre la cara frontal de la LDR
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        Primitive3D.draw_cylinder(0.0, 0.0, 0.0022, 0.0024, 0.0008, (0.95, 0.97, 1.0, 0.45), axis='z', segments=14)
        glDisable(GL_BLEND)

        # Dos patitas metálicas terminales de la LDR saliendo hacia atrás del soporte
        Primitive3D.draw_cylinder(-0.0013, 0.0, -0.009, 0.00035, 0.010, (0.75, 0.78, 0.82), axis='z', segments=6)
        Primitive3D.draw_cylinder( 0.0013, 0.0, -0.009, 0.00035, 0.010, (0.75, 0.78, 0.82), axis='z', segments=6)

        # Cableado jumper saliendo hacia atrás: Verde (Señal A0) y Rojo (5V)
        Primitive3D.draw_cylinder(-0.0013, 0.0, -0.015, 0.00075, 0.006, c_wire_grn, axis='z', segments=6)
        Primitive3D.draw_cylinder( 0.0013, 0.0, -0.015, 0.00075, 0.006, c_wire_red, axis='z', segments=6)

        # Resistencia de 10 kΩ pull-down del divisor de tensión (cuerpo beige con bandas de color)
        Primitive3D.draw_cylinder(-0.0013, 0.005, -0.013, 0.0011, 0.006, (0.86, 0.80, 0.65), axis='z', segments=8)
        glPopMatrix()

        # Pequeña brida / pasacables en el costado exterior para los jumpers
        Primitive3D.draw_box(gx, 0.440, -0.056, 0.016, 0.010, 0.006, c_pla_dark)
        Primitive3D.draw_cylinder(gx, 0.440, -0.058, 0.0018, 0.024, c_wire_grn, axis='y', segments=6)

        # -------------------------------------------------------------------------
        # 4. ILUMINACIÓN DINÁMICA OPENGL (GL_LIGHT2) Y CONO FOTOMÉTRICO
        # -------------------------------------------------------------------------
        glow_color = (0.95 * led_intensity, 0.88 * led_intensity, 0.45 * led_intensity)
        light_pos = [gx, 0.516, 0.0, 1.0]
        light_diff = [glow_color[0], glow_color[1], glow_color[2], 1.0]
        glLightfv(GL_LIGHT2, GL_POSITION, light_pos)
        glLightfv(GL_LIGHT2, GL_DIFFUSE, light_diff)
        glLightfv(GL_LIGHT2, GL_SPECULAR, light_diff)

        # Cono volumétrico de iluminación cálida proyectada desde los 4 LEDs
        glDisable(GL_LIGHTING)
        glDepthMask(GL_FALSE)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE)

        if led_intensity > 0.01:
            glPushMatrix()
            glTranslatef(gx, 0.516, 0.0)
            glBegin(GL_TRIANGLE_FAN)
            glColor4f(1.0, 0.95, 0.70, 0.06 + 0.22 * led_intensity)
            glVertex3f(0.0, 0.0, 0.0)
            glColor4f(1.0, 0.90, 0.50, 0.01 + 0.03 * led_intensity)
            for i in range(21):
                ang = 2.0 * math.pi * i / 20.0
                glVertex3f(math.cos(ang) * 0.075, -0.112, math.sin(ang) * 0.075)
            glEnd()
            glPopMatrix()

            # Mancha suave sobre la banda con polygon offset para evitar Z-fighting
            glEnable(GL_POLYGON_OFFSET_FILL)
            glPolygonOffset(-2.0, -2.0)
            glPushMatrix()
            glTranslatef(gx, 0.404, 0.0)
            glBegin(GL_TRIANGLE_FAN)
            glColor4f(1.0, 0.96, 0.75, 0.05 + 0.18 * led_intensity)
            glVertex3f(0.0, 0.0, 0.0)
            glColor4f(1.0, 0.90, 0.50, 0.0)
            for i in range(17):
                ang = 2.0 * math.pi * i / 16.0
                glVertex3f(math.cos(ang) * 0.070, 0.0, math.sin(ang) * 0.070)
            glEnd()
            glPopMatrix()
            glDisable(GL_POLYGON_OFFSET_FILL)

        glDisable(GL_BLEND)
        glDepthMask(GL_TRUE)
        glEnable(GL_LIGHTING)

    def draw_discharge_chute(self):
        """Rampa continua de chapa plegada que desciende de la banda hacia el carro móvil indexador."""
        # Inicio en X = 0.41, Y = 0.404 (boca del rodillo motriz)
        # Final en X = 0.58, Y = 0.180 (boca de entrada de las gavetas en el carro)
        # Centro geométrico de la rampa: X = 0.495, Y = 0.292, Z = 0.0
        # Inclinación: -52.0° hacia abajo hacia el eje +X
        glPushMatrix()
        glTranslatef(0.495, 0.292, 0.0)
        glRotatef(-52.0, 0, 0, 1)

        # Chapa principal de deslizamiento (ancho 90 mm, largo 280 mm)
        Primitive3D.draw_box(0.0, 0.0, 0.0, 0.28, 0.005, 0.090, (0.75, 0.78, 0.82))
        # Labios laterales de contención para encauzar las cajas
        Primitive3D.draw_box(0.0, 0.015, -0.046, 0.28, 0.030, 0.004, (0.70, 0.74, 0.78))
        Primitive3D.draw_box(0.0, 0.015,  0.046, 0.28, 0.030, 0.004, (0.70, 0.74, 0.78))
        glPopMatrix()

        # Brazos estructurales de soporte anclados al chasis inferior
        Primitive3D.draw_box(0.50, 0.14, -0.046, 0.012, 0.26, 0.012, (0.35, 0.38, 0.42))
        Primitive3D.draw_box(0.50, 0.14,  0.046, 0.012, 0.26, 0.012, (0.35, 0.38, 0.42))
        # Travesaño de unión
        Primitive3D.draw_box(0.50, 0.16,  0.0,    0.014, 0.012, 0.104, (0.35, 0.38, 0.42))

    def draw_dc_gearmotor(self):
        """
        Motorreductor DC de tracción (Lazo 1 / Pin 3 ENA PWM, Pines 4/7 DIR)
        acoplado rígidamente al eje del rodillo motriz en X = +0.410 m.
        Reemplaza al motor paso a paso NEMA 17 ficticio.
        """
        mx = self.BELT_X_RIGHT  # 0.410 m
        my = self.BELT_Y_CENTER # 0.380 m

        c_steel = (0.78, 0.80, 0.83)
        c_zinc = (0.70, 0.73, 0.77)
        c_black = (0.18, 0.19, 0.22)
        c_screw = (0.85, 0.87, 0.90)

        # 1. ACOPLE RÍGIDO DE DURALUMINIO MECANIZADO (Z = 0.048 a 0.068 m)
        # Gira sincronizado con el rodillo motriz
        Primitive3D.draw_rotating_roller(
            mx, my, 0.058,
            radius=0.008, length=0.020,
            angle_deg=-self.drive_roller_angle,
            color=c_steel, num_stripes=4, stripe_color=(0.45, 0.48, 0.52),
            segments=16
        )

        # 2. ESCUADRA DE MONTAJE DEL MOTOR EN CHAPA DE ACERO PLEGADA DE 3 mm
        # Sujeta rígidamente a la ranura del perfil 2020 longitudinal
        Primitive3D.draw_box(mx, my - 0.025, 0.070, 0.055, 0.030, 0.004, c_black)
        # Placa frontal porta-motor a Z = 0.069 m
        Primitive3D.draw_box(mx, my, 0.069, 0.048, 0.048, 0.003, c_black)
        # Tornillos Allen M4 de fijación de la escuadra al bastidor
        Primitive3D.draw_cylinder(mx - 0.016, my - 0.030, 0.066, 0.0025, 0.006, c_screw, axis='z', segments=8)
        Primitive3D.draw_cylinder(mx + 0.016, my - 0.030, 0.066, 0.0025, 0.006, c_screw, axis='z', segments=8)

        # 3. CAJA REDUCTORA CILÍNDRICA METÁLICA (Z = 0.070 a 0.092 m, diámetro 37 mm)
        Primitive3D.draw_cylinder(mx, my, 0.081, 0.0185, 0.022, c_steel, axis='z', segments=20)
        # Brida frontal con tornillos perimetrales M3
        Primitive3D.draw_cylinder(mx, my, 0.071, 0.0195, 0.002, (0.65, 0.68, 0.72), axis='z', segments=20)

        # 4. CUERPO CILÍNDRICO DEL MOTOR DC (Z = 0.092 a 0.128 m, diámetro 35.5 mm)
        Primitive3D.draw_cylinder(mx, my, 0.110, 0.0177, 0.036, c_zinc, axis='z', segments=20)
        # Ranuras de ventilación longitudinales oscuras
        for ang in (0.0, math.pi * 0.5, math.pi, math.pi * 1.5):
            vx = mx + math.cos(ang) * 0.0178
            vy = my + math.sin(ang) * 0.0178
            Primitive3D.draw_box(vx, vy, 0.110, 0.002, 0.002, 0.020, (0.25, 0.27, 0.30))

        # 5. TAPA TRASERA DE POLÍMERO NEGRO Y TERMINALES DE COBRE (Z = 0.128 a 0.134 m)
        Primitive3D.draw_cylinder(mx, my, 0.131, 0.0170, 0.006, c_black, axis='z', segments=16)
        # Terminal positivo (rojo) y negativo (negro)
        Primitive3D.draw_box(mx, my + 0.008, 0.134, 0.004, 0.002, 0.004, (0.85, 0.65, 0.20))
        Primitive3D.draw_box(mx, my - 0.008, 0.134, 0.004, 0.002, 0.004, (0.85, 0.65, 0.20))

        # Cables de alimentación que descienden desde el motor hacia la bornera OUT1/OUT2 del L298N
        Primitive3D.draw_cylinder(mx, my + 0.008, 0.136, 0.0012, 0.010, (0.85, 0.15, 0.15), axis='z', segments=6)
        Primitive3D.draw_cylinder(mx, my - 0.008, 0.136, 0.0012, 0.010, (0.15, 0.16, 0.18), axis='z', segments=6)

    def draw_l298n_driver(self):
        """
        Módulo Driver Puente H Dual L298N de tracción (Lazo 1 / PWM ENA Pin 3, DIR Pines 4/7).
        Modelado 100% fiel al diseño de GrabCAD:
        - PCB rojo FR4 (43 x 43 mm) con pistas y 4 orificios de montaje en esquinas.
        - Gran disipador de aluminio anodizado negro con 5 aletas verticales y chip L298N Multiwatt-15 fijado con tornillo M3.
        - 2x Capacitores electrolíticos cilíndricos de 220 uF 35V con tapa de aluminio plateada.
        - 3x Borneras de conexión de tornillo azules (Alimentación 3 pines, Motor A 2 pines, Motor B 2 pines).
        - 6x Pines de control con jumpers de selección y cables jumpers Dupont hacia el Arduino.
        - 8x Diodos SMD de protección flyback (1N4007) y regulador de 5V 78M05 con LED testigo de alimentación.
        """
        # Posición central del módulo L298N en el sub-chasis lateral
        lx = 0.380
        ly = 0.252
        lz = 0.108

        c_pcb_red = (0.82, 0.12, 0.12)       # Rojo FR4 característico del módulo
        c_heatsink = (0.12, 0.13, 0.15)      # Aluminio negro anodizado
        c_term_blue = (0.10, 0.35, 0.85)     # Borneras azules normalizadas
        c_cap_can = (0.22, 0.24, 0.26)       # Cuerpo capacitor negro
        c_cap_top = (0.88, 0.90, 0.93)       # Tapa superior aluminio plateado
        c_screw = (0.82, 0.84, 0.88)
        c_gold = (0.88, 0.75, 0.25)
        c_subchassis = (0.24, 0.26, 0.29)    # Bandeja soporte de perfil

        # -------------------------------------------------------------------------
        # 1. SUB-CHASIS / BANDEJA DE MONTAJE MECÁNICO AL BASTIDOR 2020
        # -------------------------------------------------------------------------
        # Brazo de soporte que baja desde el perfil longitudinal (Y = 0.345 m)
        Primitive3D.draw_box(lx, 0.300, 0.075, 0.056, 0.090, 0.008, c_subchassis)
        # Bandeja horizontal de apoyo a Y = 0.246 m
        Primitive3D.draw_box(lx, 0.246, lz, 0.054, 0.006, 0.054, c_subchassis)
        # 4x Separadores hexagonales de nylon/latón (M3 x 6 mm)
        for hx_off in (-0.018, 0.018):
            for hz_off in (-0.018, 0.018):
                Primitive3D.draw_cylinder(lx + hx_off, 0.249, lz + hz_off, 0.0022, 0.006, (0.78, 0.68, 0.25), axis='y', segments=6)

        # -------------------------------------------------------------------------
        # 2. PLACA PCB ROJA FR4 (43 x 43 x 1.6 mm)
        # -------------------------------------------------------------------------
        Primitive3D.draw_box(lx, ly, lz, 0.043, 0.0016, 0.043, c_pcb_red)
        # 4x Orificios metalizados de fijación M3 en las esquinas
        for hx_off in (-0.018, 0.018):
            for hz_off in (-0.018, 0.018):
                Primitive3D.draw_cylinder(lx + hx_off, ly + 0.0009, lz + hz_off, 0.0028, 0.0004, (0.85, 0.88, 0.90), axis='y', segments=10)
                Primitive3D.draw_cylinder(lx + hx_off, ly + 0.0012, lz + hz_off, 0.0016, 0.0010, c_screw, axis='y', segments=8)

        # Serigrafía blanca de pistas y textos
        Primitive3D.draw_box(lx, ly + 0.0009, lz, 0.038, 0.0002, 0.001, (0.95, 0.95, 0.95))

        # -------------------------------------------------------------------------
        # 3. GRAN DISIPADOR DE ALUMINIO NEGRO CON 5 ALETAS Y CHIP L298N MULTIWATT-15
        # -------------------------------------------------------------------------
        hs_x = lx - 0.009
        hs_y_base = ly + 0.001
        hs_w = 0.026  # Ancho 26 mm a lo largo de Z
        hs_h = 0.024  # Alto 24 mm
        hs_d = 0.011  # Fondo 11 mm en X

        # Bloque base del disipador
        Primitive3D.draw_box(hs_x, hs_y_base + hs_h * 0.5, lz, hs_d, hs_h, hs_w, c_heatsink)
        # 5x Aletas verticales de disipación
        fin_spacing = hs_w / 4.0
        for i in range(5):
            fz = lz - hs_w * 0.5 + i * fin_spacing
            Primitive3D.draw_box(hs_x, hs_y_base + hs_h * 0.5 + 0.002, fz, hs_d + 0.002, hs_h, 0.0015, (0.16, 0.17, 0.20))

        # Circuito Integrado L298N (encapsulado Multiwatt-15 adosado a la cara frontal del disipador)
        ic_x = hs_x + hs_d * 0.5 + 0.0018
        ic_y = hs_y_base + 0.012
        # Pestaña metálica de fijación
        Primitive3D.draw_box(ic_x, ic_y + 0.004, lz, 0.0012, 0.010, 0.016, (0.80, 0.82, 0.85))
        # Tornillo M3 con arandela fijando el chip al disipador
        Primitive3D.draw_cylinder(ic_x + 0.0008, ic_y + 0.006, lz, 0.0025, 0.002, c_screw, axis='x', segments=12)
        # Cuerpo epoxi negro del L298N
        Primitive3D.draw_box(ic_x + 0.001, ic_y - 0.003, lz, 0.0025, 0.012, 0.018, (0.10, 0.10, 0.12))
        # Patitas metálicas escalonadas soldadas al PCB
        for p in range(8):
            pz = lz - 0.007 + p * 0.002
            Primitive3D.draw_cylinder(ic_x - 0.001, hs_y_base + 0.002, pz, 0.00035, 0.004, (0.85, 0.88, 0.90), axis='y', segments=6)

        # -------------------------------------------------------------------------
        # 4. DOS CONDENSADORES ELECTROLÍTICOS CILÍNDRICOS (220 uF 35V)
        # -------------------------------------------------------------------------
        cap_x = lx + 0.003
        for cap_z in (lz - 0.007, lz + 0.007):
            # Cuerpo cilíndrico negro del condensador (diámetro 8 mm, altura 11.5 mm)
            Primitive3D.draw_cylinder(cap_x, ly + 0.0065, cap_z, 0.004, 0.0115, c_cap_can, axis='y', segments=16)
            # Tapa superior de aluminio plateada con hendidura de venteo
            Primitive3D.draw_cylinder(cap_x, ly + 0.0124, cap_z, 0.0038, 0.0004, c_cap_top, axis='y', segments=16)
            # Franja lateral blanca de polaridad negativa
            Primitive3D.draw_box(cap_x + 0.0039, ly + 0.0065, cap_z, 0.0003, 0.010, 0.0025, (0.92, 0.92, 0.94))

        # -------------------------------------------------------------------------
        # 5. TRES BORNERAS AZULES DE CONEXIÓN A TORNILLO
        # -------------------------------------------------------------------------
        # A. Bornera frontal de 3 pines (+12V, GND, +5V) en X = lx + 0.015 m
        tb_power_x = lx + 0.015
        tb_power_z = lz
        Primitive3D.draw_box(tb_power_x, ly + 0.0055, tb_power_z, 0.008, 0.010, 0.016, c_term_blue)
        for tz_off in (-0.005, 0.0, 0.005):
            Primitive3D.draw_cylinder(tb_power_x, ly + 0.0108, tb_power_z + tz_off, 0.0016, 0.0012, c_screw, axis='y', segments=10)
            Primitive3D.draw_box(tb_power_x + 0.0041, ly + 0.004, tb_power_z + tz_off, 0.0004, 0.0035, 0.003, (0.15, 0.16, 0.18))

        # B. Bornera lateral izquierda Motor A (OUT1, OUT2) en Z = lz - 0.016 m
        tb_outA_x = lx - 0.003
        tb_outA_z = lz - 0.016
        Primitive3D.draw_box(tb_outA_x, ly + 0.0055, tb_outA_z, 0.012, 0.010, 0.008, c_term_blue)
        for tx_off in (-0.003, 0.003):
            Primitive3D.draw_cylinder(tb_outA_x + tx_off, ly + 0.0108, tb_outA_z, 0.0016, 0.0012, c_screw, axis='y', segments=10)

        # C. Bornera lateral derecha Motor B (OUT3, OUT4) en Z = lz + 0.016 m
        tb_outB_x = lx - 0.003
        tb_outB_z = lz + 0.016
        Primitive3D.draw_box(tb_outB_x, ly + 0.0055, tb_outB_z, 0.012, 0.010, 0.008, c_term_blue)
        for tx_off in (-0.003, 0.003):
            Primitive3D.draw_cylinder(tb_outB_x + tx_off, ly + 0.0108, tb_outB_z, 0.0016, 0.0012, c_screw, axis='y', segments=10)

        # -------------------------------------------------------------------------
        # 6. TIRA DE PINES DE CONTROL (6 PINES: ENA, IN1, IN2, IN3, IN4, ENB)
        # -------------------------------------------------------------------------
        hdr_x = lx + 0.015
        hdr_z = lz + 0.012
        Primitive3D.draw_box(hdr_x, ly + 0.002, hdr_z, 0.004, 0.0025, 0.015, (0.12, 0.13, 0.15))
        for p in range(6):
            pz = hdr_z - 0.006 + p * 0.0024
            Primitive3D.draw_box(hdr_x, ly + 0.0055, pz, 0.0006, 0.007, 0.0006, c_gold)

        # Jumper negro de puente en ENA y ENB
        Primitive3D.draw_box(hdr_x, ly + 0.006, hdr_z - 0.006, 0.0022, 0.006, 0.0022, (0.12, 0.13, 0.15))

        # Cables Jumpers de control saliendo hacia el Arduino:
        # Azul (ENA / D3 PWM), Amarillo (IN1 / D4), Naranja (IN2 / D7)
        Primitive3D.draw_cylinder(hdr_x, ly + 0.010, hdr_z - 0.0036, 0.0008, 0.008, (0.15, 0.45, 0.85), axis='y', segments=6)
        Primitive3D.draw_cylinder(hdr_x, ly + 0.010, hdr_z - 0.0012, 0.0008, 0.008, (0.85, 0.75, 0.15), axis='y', segments=6)
        Primitive3D.draw_cylinder(hdr_x, ly + 0.010, hdr_z + 0.0012, 0.0008, 0.008, (0.88, 0.45, 0.12), axis='y', segments=6)

        # -------------------------------------------------------------------------
        # 7. 8x DIODOS DE PROTECCIÓN FLYBACK (1N4007 SMD) Y REGULADOR 78M05
        # -------------------------------------------------------------------------
        for dz_side in (-0.011, 0.011):
            for di in range(4):
                dx_pos = lx - 0.014 + di * 0.005
                Primitive3D.draw_box(dx_pos, ly + 0.0012, lz + dz_side, 0.0035, 0.0012, 0.0018, (0.15, 0.16, 0.18))
                Primitive3D.draw_box(dx_pos - 0.0012, ly + 0.0013, lz + dz_side, 0.0008, 0.0013, 0.0018, (0.80, 0.82, 0.85))

        # Regulador de voltaje 78M05 (SMD TO-252)
        Primitive3D.draw_box(lx - 0.002, ly + 0.0018, lz + 0.008, 0.006, 0.002, 0.006, (0.15, 0.16, 0.18))
        Primitive3D.draw_box(lx - 0.005, ly + 0.0014, lz + 0.008, 0.002, 0.001, 0.004, (0.75, 0.78, 0.82))

        # Mini LED rojo SMD indicador de alimentación encendido
        Primitive3D.draw_box(lx + 0.008, ly + 0.0014, lz - 0.004, 0.002, 0.0012, 0.0015, (0.95, 0.15, 0.15))

        # -------------------------------------------------------------------------
        # 8. CABLEADO DE POTENCIA ENTRE L298N (OUT1/OUT2) Y MOTOR DC
        # -------------------------------------------------------------------------
        Primitive3D.draw_cylinder(tb_outA_x - 0.003, ly + 0.035, tb_outA_z - 0.002, 0.0012, 0.050, (0.85, 0.15, 0.15), axis='y', segments=6)
        Primitive3D.draw_cylinder(tb_outA_x + 0.003, ly + 0.035, tb_outA_z - 0.002, 0.0012, 0.050, (0.15, 0.16, 0.18), axis='y', segments=6)

    def draw_feed_hopper(self):
        """
        Estación industrial de alimentación por gravedad (Tolva dosificadora en X = -0.42 m).
        Totalmente apoyada sobre el suelo (Y = 0.0) mediante columnas 2020 y pies niveladores,
        unida rígidamente al bastidor de la faja con escuadras estructurales, embudo de chapa
        tronco-piramidal, ventana de inspección, compuerta reguladora y faldones de encauce.
        """
        hx = -0.420
        col_x = -0.450
        col_z_l = -0.075
        col_z_r = 0.075

        # -------------------------------------------------------------------------
        # 1. PIES NIVELADORES Y PLACAS BASE AL SUELO (Y = 0.0 m)
        # -------------------------------------------------------------------------
        for cz in (col_z_l, col_z_r):
            # Base amortiguadora de caucho vulcanizado apoyada en el suelo
            Primitive3D.draw_cylinder(col_x, 0.003, cz, 0.018, 0.006, (0.16, 0.17, 0.19), axis='y', segments=16)
            # Espárrago roscado de regulación M10 en acero inoxidable
            Primitive3D.draw_cylinder(col_x, 0.014, cz, 0.005, 0.016, (0.82, 0.84, 0.87), axis='y', segments=12)
            # Tuerca hexagonal de bloqueo
            Primitive3D.draw_cylinder(col_x, 0.016, cz, 0.009, 0.005, (0.88, 0.90, 0.92), axis='y', segments=6)
            # Placa de asiento inferior de acero al carbono (50 x 50 x 6 mm)
            Primitive3D.draw_box(col_x, 0.024, cz, 0.050, 0.006, 0.050, (0.34, 0.36, 0.40))
            # Tornillos de sujeción al perfil 2020
            for bx in (-0.016, 0.016):
                Primitive3D.draw_cylinder(col_x + bx, 0.028, cz, 0.0025, 0.004, (0.75, 0.78, 0.80), axis='y', segments=8)

        # -------------------------------------------------------------------------
        # 2. COLUMNAS ESTRUCTURALES VERTICALES T-SLOT 2020 (Desde Y = 0.027 hasta 0.760 m)
        # -------------------------------------------------------------------------
        # Altura = 0.733 m, centro Y = 0.3935 m
        Primitive3D.draw_box(col_x, 0.3935, col_z_l, 0.020, 0.733, 0.020, self.C_ALU)
        Primitive3D.draw_box(col_x, 0.3935, col_z_r, 0.020, 0.733, 0.020, self.C_ALU)

        # Travesaño superior de coronación a Y = 0.750 m (ancho 0.019 m para evitar Z-fighting coplanar con columnas)
        Primitive3D.draw_box(col_x, 0.750, 0.0, 0.019, 0.020, 0.149, self.C_ALU)
        # Travesaño intermedio posterior a Y = 0.530 m
        Primitive3D.draw_box(col_x, 0.530, 0.0, 0.019, 0.020, 0.149, self.C_ALU)
        # Travesaño de arriostramiento inferior a Y = 0.180 m
        Primitive3D.draw_box(col_x, 0.180, 0.0, 0.019, 0.020, 0.149, self.C_ALU)

        # -------------------------------------------------------------------------
        # 3. ANCLAJES RÍGIDOS Y ESCUADRAS AL BASTIDOR DE LA FAJA (X = -0.410 m)
        # -------------------------------------------------------------------------
        # Pletinas de enlace horizontal a la altura del chasis de la cinta (Y = 0.380 m)
        for cz in (col_z_l, col_z_r):
            # Barra de acople directo entre columna (-0.450) y perfil de faja (-0.410)
            Primitive3D.draw_box(-0.430, 0.380, cz, 0.039, 0.019, 0.015, (0.40, 0.43, 0.46))
            # Escuadra de refuerzo diagonal inferior
            Primitive3D.draw_box(-0.435, 0.362, cz, 0.025, 0.013, 0.015, (0.32, 0.35, 0.39))

        # Brazos horizontales en voladizo para soportar la tolva y garganta
        for cz in (col_z_l, col_z_r):
            # Brazo superior de soporte a Y = 0.720 m
            Primitive3D.draw_box(-0.420, 0.720, cz, 0.059, 0.015, 0.015, self.C_ALU)
            # Brazo inferior de soporte a Y = 0.570 m
            Primitive3D.draw_box(-0.420, 0.570, cz, 0.059, 0.015, 0.015, self.C_ALU)

        # -------------------------------------------------------------------------
        # 4. EMBUDO TRONCO-PIRAMIDAL DE CHAPA PLEGADA (Azul Industrial RAL 5012)
        # -------------------------------------------------------------------------
        # Coordenadas: Boca superior Y = 0.740, Garganta inferior Y = 0.570
        y_top = 0.740
        y_bot = 0.570
        x_top_min, x_top_max = hx - 0.075, hx + 0.075  # -0.495 a -0.345 (150 mm)
        z_top_min, z_top_max = -0.070, 0.070           # 140 mm
        x_bot_min, x_bot_max = hx - 0.037, hx + 0.037  # -0.457 a -0.383 (74 mm)
        z_bot_min, z_bot_max = -0.037, 0.037           # 74 mm

        # Vértices exteriores de la chapa
        T1 = (x_top_min, y_top, z_top_min)
        T2 = (x_top_max, y_top, z_top_min)
        T3 = (x_top_max, y_top, z_top_max)
        T4 = (x_top_min, y_top, z_top_max)

        B1 = (x_bot_min, y_bot, z_bot_min)
        B2 = (x_bot_max, y_bot, z_bot_min)
        B3 = (x_bot_max, y_bot, z_bot_max)
        B4 = (x_bot_min, y_bot, z_bot_max)

        # Vértices interiores con espesor real de chapa (1.5 mm hacia adentro)
        # Esto elimina 100% el parpadeo / Z-fighting entre cara interior y exterior
        th = 0.0018
        T1_in = (x_top_min + th, y_top, z_top_min + th)
        T2_in = (x_top_max - th, y_top, z_top_min + th)
        T3_in = (x_top_max - th, y_top, z_top_max - th)
        T4_in = (x_top_min + th, y_top, z_top_max - th)

        B1_in = (x_bot_min + th, y_bot, z_bot_min + th)
        B2_in = (x_bot_max - th, y_bot, z_bot_min + th)
        B3_in = (x_bot_max - th, y_bot, z_bot_max - th)
        B4_in = (x_bot_min + th, y_bot, z_bot_max - th)

        # Cara exterior: Azul industrial de alta calidad
        c_ext = (0.18, 0.42, 0.68)
        # Cara interior: Acero pavonado/sombra interior
        c_int = (0.12, 0.30, 0.50)

        # Renderizado de paredes inclinadas de chapa plegada con espesor físico
        glBegin(GL_QUADS)
        # Pared Posterior (-X): Exterior
        glColor3f(c_ext[0], c_ext[1], c_ext[2])
        glNormal3f(-0.976, -0.218, 0.0)
        glVertex3fv(T1); glVertex3fv(T4); glVertex3fv(B4); glVertex3fv(B1)
        # Pared Posterior (-X): Interior
        glColor3f(c_int[0], c_int[1], c_int[2])
        glNormal3f(0.976, 0.218, 0.0)
        glVertex3fv(B1_in); glVertex3fv(B4_in); glVertex3fv(T4_in); glVertex3fv(T1_in)

        # Pared Frontal (+X): Exterior
        glColor3f(c_ext[0], c_ext[1], c_ext[2])
        glNormal3f(0.976, -0.218, 0.0)
        glVertex3fv(T3); glVertex3fv(T2); glVertex3fv(B2); glVertex3fv(B3)
        # Pared Frontal (+X): Interior
        glColor3f(c_int[0], c_int[1], c_int[2])
        glNormal3f(-0.976, 0.218, 0.0)
        glVertex3fv(B3_in); glVertex3fv(B2_in); glVertex3fv(T2_in); glVertex3fv(T3_in)

        # Pared Lateral Izquierda (-Z): Exterior
        glColor3f(c_ext[0] * 0.92, c_ext[1] * 0.92, c_ext[2] * 0.92)
        glNormal3f(0.0, -0.218, -0.976)
        glVertex3fv(T2); glVertex3fv(T1); glVertex3fv(B1); glVertex3fv(B2)
        # Pared Lateral Izquierda (-Z): Interior
        glColor3f(c_int[0], c_int[1], c_int[2])
        glNormal3f(0.0, 0.218, 0.976)
        glVertex3fv(B2_in); glVertex3fv(B1_in); glVertex3fv(T1_in); glVertex3fv(T2_in)

        # Pared Lateral Derecha (+Z): Exterior
        glColor3f(c_ext[0] * 0.92, c_ext[1] * 0.92, c_ext[2] * 0.92)
        glNormal3f(0.0, -0.218, 0.976)
        glVertex3fv(T4); glVertex3fv(T3); glVertex3fv(B3); glVertex3fv(B4)
        # Pared Lateral Derecha (+Z): Interior
        glColor3f(c_int[0], c_int[1], c_int[2])
        glNormal3f(0.0, 0.218, -0.976)
        glVertex3fv(B4_in); glVertex3fv(B3_in); glVertex3fv(T3_in); glVertex3fv(T4_in)
        glEnd()

        # Reborde perimetral de seguridad plegado en acero inoxidable en la boca superior
        c_rim = (0.82, 0.84, 0.88)
        Primitive3D.draw_box(hx, y_top + 0.003, z_top_min, 0.154, 0.008, 0.006, c_rim)
        Primitive3D.draw_box(hx, y_top + 0.003, z_top_max, 0.154, 0.008, 0.006, c_rim)
        Primitive3D.draw_box(x_top_min, y_top + 0.003, 0.0, 0.006, 0.008, 0.144, c_rim)
        Primitive3D.draw_box(x_top_max, y_top + 0.003, 0.0, 0.006, 0.008, 0.144, c_rim)

        # -------------------------------------------------------------------------
        # 5. BRIDA Y GARGANTA VERTICAL DE DOSIFICACIÓN (Y = 0.570 a 0.465 m)
        # -------------------------------------------------------------------------
        # Brida intermedia de acople mecanizada
        Primitive3D.draw_box(hx, y_bot, 0.0, 0.090, 0.008, 0.090, (0.32, 0.35, 0.39))
        for fx in (hx - 0.038, hx + 0.038):
            for fz in (-0.038, 0.038):
                Primitive3D.draw_cylinder(fx, y_bot + 0.004, fz, 0.0025, 0.004, (0.80, 0.82, 0.85), axis='y', segments=8)

        # Paredes de la bajante vertical (altura = 0.095 m, centro Y = 0.5175 m)
        Primitive3D.draw_box(x_bot_min, 0.5175, 0.0, 0.004, 0.095, 0.074, (0.26, 0.28, 0.32))
        Primitive3D.draw_box(hx, 0.5175, z_bot_min, 0.074, 0.095, 0.004, (0.26, 0.28, 0.32))
        Primitive3D.draw_box(hx, 0.5175, z_bot_max, 0.074, 0.095, 0.004, (0.26, 0.28, 0.32))

        # Ventana frontal de inspección en policarbonato translúcido (con offset para no solapar)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glDepthMask(GL_FALSE)
        Primitive3D.draw_box(x_bot_max + 0.0005, 0.525, 0.0, 0.002, 0.075, 0.064, (0.80, 0.90, 0.98), alpha=0.42)
        glDepthMask(GL_TRUE)
        glDisable(GL_BLEND)

        # Guillotina / compuerta corredera reguladora de altura en acero galvanizado
        Primitive3D.draw_box(x_bot_max + 0.0025, 0.490, 0.0, 0.004, 0.040, 0.068, (0.72, 0.75, 0.79))
        # Pomos moleteados de latón para ajuste micrométrico de apertura
        Primitive3D.draw_cylinder(x_bot_max + 0.0055, 0.505, -0.022, 0.004, 0.006, (0.85, 0.70, 0.22), axis='x', segments=12)
        Primitive3D.draw_cylinder(x_bot_max + 0.0055, 0.505,  0.022, 0.004, 0.006, (0.85, 0.70, 0.22), axis='x', segments=12)

        # -------------------------------------------------------------------------
        # 6. SENSOR ÓPTICO DE PRESENCIA / ATASCO EN TOLVA (M12)
        # -------------------------------------------------------------------------
        # Cuerpo cilíndrico roscado M12 en latón niquelado a Z = -0.052 m
        Primitive3D.draw_cylinder(hx, 0.515, -0.046, 0.006, 0.020, (0.76, 0.68, 0.28), axis='z', segments=14)
        # Lente frontal apuntando al interior de la garganta
        Primitive3D.draw_cylinder(hx, 0.515, -0.035, 0.0045, 0.003, (0.15, 0.15, 0.18), axis='z', segments=12)
        # LED indicador de estado en la parte posterior (verde encendido)
        Primitive3D.draw_cylinder(hx, 0.515, -0.057, 0.003, 0.003, (0.18, 0.88, 0.25), axis='z', segments=10)

        # -------------------------------------------------------------------------
        # 7. FALDONES DE ENCAUCE LATERAL SOBRE LA BANDA (X = -0.455 a -0.320 m)
        # -------------------------------------------------------------------------
        # Guían la caja recién caída hacia el centro de la banda transportadora sin salirse
        skirt_len = 0.135
        skirt_x = -0.3875
        skirt_y = 0.422
        skirt_h = 0.034
        c_skirt = (0.86, 0.88, 0.91)   # Polietileno UHMW-PE blanco técnico
        Primitive3D.draw_box(skirt_x, skirt_y, -0.041, skirt_len, skirt_h, 0.004, c_skirt)
        Primitive3D.draw_box(skirt_x, skirt_y,  0.041, skirt_len, skirt_h, 0.004, c_skirt)

        # Soportes en escuadra de los faldones fijados al bastidor
        for sx in (-0.440, -0.340):
            Primitive3D.draw_box(sx, 0.412, -0.050, 0.012, 0.022, 0.015, (0.35, 0.38, 0.42))
            Primitive3D.draw_box(sx, 0.412,  0.050, 0.012, 0.022, 0.015, (0.35, 0.38, 0.42))

        # -------------------------------------------------------------------------
        # 8. CAJAS ESCALONADAS EN ESPERA (STAGING) DENTRO DE LA TOLVA
        # -------------------------------------------------------------------------
        c_box = (0.80, 0.68, 0.46)     # Cartón Kraft industrial
        c_tape = (0.64, 0.52, 0.32)    # Cinta de embalaje

        # Caja en espera dentro del embudo superior (Y = 0.640 m)
        # Se ubica por encima de la garganta para que NUNCA colisione ni parpadee
        # con las cajas dinámicas cayendo en caída libre (que van de Y = 0.54 a 0.4255)
        glPushMatrix()
        glTranslatef(hx, 0.640, 0.0)
        glRotatef(3.5, 0, 0, 1)
        glRotatef(-2.0, 1, 0, 0)
        Primitive3D.draw_box(0.0, 0.0, 0.0, 0.055, 0.045, 0.055, c_box)
        Primitive3D.draw_box(0.0, 0.023, 0.0, 0.056, 0.001, 0.016, c_tape)
        # Etiqueta de código de barras blanca en el lateral
        Primitive3D.draw_box(0.0, 0.0, 0.028, 0.030, 0.018, 0.001, (0.94, 0.95, 0.96))
        glPopMatrix()

    def draw_motorized_container_shuttle(self, cart_z: float, servo_angle_deg: float = 90.0):
        """
        Lanzadera transversal indexadora M-102 en X = +0.58 m con 3 gavetas (Retrabajo, Conforme, Rechazo).
        Accionada cinemáticamente por el Servomotor SG90 Tower Pro 9g mediante mecanismo biela-manivela
        con cinemática rígida exacta y biela de longitud matemáticamente invariante L = 400.00 mm.
        """
        cx = 0.58
        # Bancada y guías en Z
        Primitive3D.draw_box(cx, 0.02, 0.0, 0.10, 0.04, 0.94, self.C_ALU)
        Primitive3D.draw_box(cx, 0.041, 0.0, 0.015, 0.004, 0.92, (0.40, 0.42, 0.45))
        Primitive3D.draw_cylinder(cx - 0.035, 0.055, 0.0, 0.006, 0.92, (0.90, 0.92, 0.94), axis='z')
        Primitive3D.draw_cylinder(cx + 0.035, 0.055, 0.0, 0.006, 0.92, (0.90, 0.92, 0.94), axis='z')

        # Soportes extremos de las guías lineales
        Primitive3D.draw_box(cx, 0.05, -0.46, 0.11, 0.06, 0.03, (0.35, 0.38, 0.42))
        Primitive3D.draw_box(cx, 0.05,  0.46, 0.11, 0.06, 0.03, (0.35, 0.38, 0.42))

        # Topes elásticos amortiguadores de final de carrera en goma negra
        for sz_stop in (-0.445, 0.445):
            Primitive3D.draw_cylinder(cx - 0.035, 0.055, sz_stop, 0.009, 0.015, (0.15, 0.16, 0.18), axis='z')
            Primitive3D.draw_cylinder(cx + 0.035, 0.055, sz_stop, 0.009, 0.015, (0.15, 0.16, 0.18), axis='z')

        # Plataforma móvil portacontenedores
        car_y = 0.08
        Primitive3D.draw_box(cx, car_y, cart_z, 0.16, 0.015, 0.30, self.C_ALU)
        # 4x Bloques de rodamientos lineales sobre las barras guía
        for bz in (-0.11, 0.11):
            Primitive3D.draw_box(cx - 0.035, 0.055, cart_z + bz, 0.025, 0.022, 0.036, (0.40, 0.43, 0.46))
            Primitive3D.draw_box(cx + 0.035, 0.055, cart_z + bz, 0.025, 0.022, 0.036, (0.40, 0.43, 0.46))

        # ---------------------------------------------------------------------
        # 3 GAVETAS INDUSTRIALES DE CLASIFICACIÓN (Proporcionales a la caja de 55 mm)
        # Paso entre centros: dz_spacing = 0.075 m (75 mm)
        # Dimensiones de cada gaveta: X=100 mm, Y=80 mm, Z=70 mm
        # ---------------------------------------------------------------------
        bin_dx, bin_dy, bin_dz = 0.100, 0.080, 0.070
        bin_center_y = car_y + 0.008 + bin_dy * 0.5  # 0.128 m (boca superior a Y = 0.168 m)
        dz_spacing = 0.075

        # 1. RETRABAJO (Ámbar técnico RAL 1028, Z = cart_z - 0.075 m)
        z1 = cart_z - dz_spacing
        Primitive3D.draw_box(cx, bin_center_y, z1, bin_dx, bin_dy, bin_dz, (0.92, 0.70, 0.15))
        Primitive3D.draw_box(cx, bin_center_y + 0.006, z1, bin_dx - 0.010, bin_dy, bin_dz - 0.010, (0.20, 0.15, 0.04))
        Primitive3D.draw_box(cx, bin_center_y + bin_dy * 0.5, z1, bin_dx + 0.004, 0.004, bin_dz + 0.004, (0.98, 0.76, 0.18))
        Primitive3D.draw_box(cx - bin_dx * 0.5 - 0.0006, bin_center_y, z1, 0.001, 0.024, 0.050, (0.15, 0.16, 0.18))
        Primitive3D.draw_box(cx - bin_dx * 0.5 - 0.0008, bin_center_y, z1, 0.001, 0.018, 0.044, (0.96, 0.82, 0.20))

        # 2. CONFORME (Verde ingeniería RAL 6018, Z = cart_z)
        z2 = cart_z
        Primitive3D.draw_box(cx, bin_center_y, z2, bin_dx, bin_dy, bin_dz, (0.22, 0.65, 0.28))
        Primitive3D.draw_box(cx, bin_center_y + 0.006, z2, bin_dx - 0.010, bin_dy, bin_dz - 0.010, (0.05, 0.18, 0.07))
        Primitive3D.draw_box(cx, bin_center_y + bin_dy * 0.5, z2, bin_dx + 0.004, 0.004, bin_dz + 0.004, (0.28, 0.75, 0.34))
        Primitive3D.draw_box(cx - bin_dx * 0.5 - 0.0006, bin_center_y, z2, 0.001, 0.024, 0.050, (0.15, 0.16, 0.18))
        Primitive3D.draw_box(cx - bin_dx * 0.5 - 0.0008, bin_center_y, z2, 0.001, 0.018, 0.044, (0.35, 0.88, 0.42))

        # 3. RECHAZO (Rojo tráfico RAL 3020, Z = cart_z + 0.075 m)
        z3 = cart_z + dz_spacing
        Primitive3D.draw_box(cx, bin_center_y, z3, bin_dx, bin_dy, bin_dz, (0.85, 0.20, 0.18))
        Primitive3D.draw_box(cx, bin_center_y + 0.006, z3, bin_dx - 0.010, bin_dy, bin_dz - 0.010, (0.20, 0.05, 0.05))
        Primitive3D.draw_box(cx, bin_center_y + bin_dy * 0.5, z3, bin_dx + 0.004, 0.004, bin_dz + 0.004, (0.95, 0.26, 0.24))
        Primitive3D.draw_box(cx - bin_dx * 0.5 - 0.0006, bin_center_y, z3, 0.001, 0.024, 0.050, (0.15, 0.16, 0.18))
        Primitive3D.draw_box(cx - bin_dx * 0.5 - 0.0008, bin_center_y, z3, 0.001, 0.018, 0.044, (0.98, 0.30, 0.28))

        # Mecanismo de accionamiento por Servomotor SG90 Tower Pro y Biela-Manivela
        self.draw_sg90_actuator_mechanism(cart_z, servo_angle_deg)

    def draw_sg90_actuator_mechanism(self, cart_z: float, servo_angle_deg: float):
        """
        Renderiza el Micro Servomotor SG90 9g Tower Pro y el mecanismo cinemático de biela-manivela
        rígido que acciona la lanzadera transversal indexadora de 3 gavetas (Lazo 2 / Pin 5 PWM).
        Totalmente fiel a GrabCAD y la imagen del fabricante:
        - Carcasa azul translúcida con escalón de engranajes y torre cilíndrica estriada.
        - Pestañas de montaje con ojales de latón y tornillos de fijación M2.
        - Etiqueta amarilla/dorada serigrafiada 'Tower Pro Micro Servo 9g SG90'.
        - Horn blanco de nylon con orificios de aligeramiento y tornillo central M2.5.
        - Biela de empuje (pushrod) rígida con rótulas esféricas de latón en ambos extremos.
        - Cinemática cerrada con longitud matemáticamente invariante L = 400.00 mm (cero variación de longitud).
        - Cableado plano de 3 hilos (Marrón GND, Rojo 5V, Naranja PWM Pin 5).
        """
        # Parámetros cinemáticos del mecanismo biela-manivela:
        Sx = 0.670
        Sy = 0.082
        Cx = 0.658
        R = 0.105   # Longitud de la manivela / horn (105 mm)
        L = 0.400   # Longitud constante e invariable de la biela (400 mm)
        phi_0 = math.radians(170.0)

        # Posición de referencia en Z para que Cz(90°) = 0.000 m
        hx90 = Sx + R * math.cos(phi_0)
        hz90_rel = R * math.sin(phi_0)
        c90_rel = hz90_rel + math.sqrt(L**2 - (Cx - hx90)**2)
        Sz = -c90_rel   # -0.40765 m

        # Cinemática inversa analítica exacta: resuelve phi garantizando que ||C - H|| == L = 400.00 mm
        Cz_world = cart_z
        U = Cx - Sx
        V = Cz_world - Sz
        K = (U**2 + V**2 + R**2 - L**2) / (2.0 * R)
        mag = math.sqrt(U**2 + V**2)
        cos_val = max(-1.0, min(1.0, K / mag))
        psi = math.atan2(V, U)
        phi = psi + math.acos(cos_val)

        # Punta del horn (Punto H)
        hx = Sx + R * math.cos(phi)
        hy = Sy + 0.0038
        hz = Sz + R * math.sin(phi)

        # Perno de arrastre en el carro (Punto C)
        cx_pin = Cx
        cy_pin = hy
        cz_pin = Cz_world

        c_servo_blue = (0.10, 0.28, 0.80)     # Azul translúcido clásico Tower Pro
        c_horn_white = (0.95, 0.95, 0.97)     # Nylon blanco del horn
        c_gold_label = (0.96, 0.78, 0.12)     # Etiqueta amarilla metálica
        c_dark_label = (0.12, 0.14, 0.16)     # Fondo negro del texto
        c_brass = (0.85, 0.75, 0.32)          # Latón (buje estriado y ojales)
        c_screw = (0.84, 0.86, 0.89)          # Tornillería inoxidable M2/M2.5
        c_cradle = (0.18, 0.19, 0.22)         # Cuna de montaje PLA negro

        # 1. SOPORTE EN VOLADIZO Y CUNA DE MONTAJE DEL SERVO
        # Brazo estructural en perfil de aluminio fijado a la bancada lineal (Y = 0.025 m)
        Primitive3D.draw_box((0.58 + Sx) * 0.5, 0.025, Sz, (Sx - 0.58) + 0.030, 0.020, 0.055, self.C_ALU)
        # Cuna de soporte PLA para el SG90
        Primitive3D.draw_box(Sx + 0.005, 0.052, Sz, 0.040, 0.016, 0.036, c_cradle)
        # Tornillos Allen M3 de fijación de la cuna
        Primitive3D.draw_cylinder(Sx - 0.010, 0.061, Sz + 0.012, 0.0022, 0.004, c_screw, axis='y', segments=8)
        Primitive3D.draw_cylinder(Sx - 0.010, 0.061, Sz - 0.012, 0.0022, 0.004, c_screw, axis='y', segments=8)

        # 2. CUERPO PRINCIPAL DEL MICRO SERVO SG90 (22.8 x 12.2 x 22.5 mm)
        bx = Sx + 0.0054
        by = Sy - 0.0160
        bz = Sz
        Primitive3D.draw_box(bx, by, bz, 0.0228, 0.0225, 0.0122, c_servo_blue)

        # Escalón superior de la caja de engranajes
        Primitive3D.draw_box(Sx, Sy - 0.0035, bz, 0.0120, 0.0050, 0.0120, c_servo_blue)

        # Torre cilíndrica del eje de salida (Ø 11.8 mm, h = 3.5 mm)
        Primitive3D.draw_cylinder(Sx, Sy - 0.0015, bz, 0.0058, 0.0035, c_servo_blue, axis='y', segments=16)

        # Eje estriado de latón
        Primitive3D.draw_cylinder(Sx, Sy + 0.0012, bz, 0.0024, 0.0025, c_brass, axis='y', segments=12)

        # Pestañas de montaje con ojales de latón y tornillos M2
        for x_ear in (bx - 0.0135, bx + 0.0135):
            Primitive3D.draw_box(x_ear, Sy - 0.010, bz, 0.0045, 0.0024, 0.0122, c_servo_blue)
            Primitive3D.draw_cylinder(x_ear, Sy - 0.009, bz, 0.0018, 0.0028, c_brass, axis='y', segments=8)
            Primitive3D.draw_cylinder(x_ear, Sy - 0.006, bz, 0.0011, 0.0040, c_screw, axis='y', segments=8)

        # 3. ETIQUETA SERIGRAFIADA 'TOWER PRO MICRO SERVO 9G SG90'
        label_z = bz + 0.0063
        Primitive3D.draw_box(bx, by + 0.001, label_z, 0.0180, 0.0120, 0.0003, c_gold_label)
        Primitive3D.draw_box(bx, by + 0.001, label_z + 0.0001, 0.0160, 0.0095, 0.0002, c_dark_label)
        # Líneas de serigrafía doradas
        Primitive3D.draw_box(bx, by + 0.0035, label_z + 0.0002, 0.0130, 0.0018, 0.0002, c_gold_label)
        Primitive3D.draw_box(bx, by - 0.0015, label_z + 0.0002, 0.0140, 0.0024, 0.0002, c_gold_label)
        Primitive3D.draw_box(bx, by - 0.0045, label_z + 0.0002, 0.0100, 0.0015, 0.0002, c_gold_label)

        # 4. MANGUERA DE CABLEADO TRIPOLAR DUPONT
        Primitive3D.draw_cylinder(bx + 0.012, by - 0.018, bz - 0.0014, 0.0006, 0.020, (0.38, 0.22, 0.12), axis='y', segments=6) # Marrón (GND)
        Primitive3D.draw_cylinder(bx + 0.012, by - 0.018, bz,          0.0006, 0.020, (0.85, 0.15, 0.15), axis='y', segments=6) # Rojo (+5V)
        Primitive3D.draw_cylinder(bx + 0.012, by - 0.018, bz + 0.0014, 0.0006, 0.020, (0.95, 0.52, 0.10), axis='y', segments=6) # Naranja (PWM Pin 5)

        # 5. HORN BLANCO DE NYLON (MANIVELA R = 105 mm)
        # Cubo central del horn con tornillo fijador M2.5
        Primitive3D.draw_cylinder(Sx, Sy + 0.0025, bz, 0.0042, 0.0032, c_horn_white, axis='y', segments=16)
        Primitive3D.draw_cylinder(Sx, Sy + 0.0042, bz, 0.0016, 0.0006, c_screw, axis='y', segments=10)

        # Brazo cónico del horn extendido desde el eje (Sx, hy, Sz) hacia la punta (hx, hy, hz)
        Primitive3D.draw_rod((Sx, hy, Sz), (hx, hy, hz), radius=0.0024, color=c_horn_white, segments=12)
        # Orificios de aligeramiento/ajuste a lo largo del brazo
        for f in (0.25, 0.45, 0.65, 0.85):
            ox = Sx + (hx - Sx) * f
            oz = Sz + (hz - Sz) * f
            Primitive3D.draw_cylinder(ox, hy + 0.0008, oz, 0.0009, 0.0020, (0.25, 0.26, 0.30), axis='y', segments=8)

        # Rótula esférica de latón y tornillo en la punta del horn (Punto H)
        Primitive3D.draw_cylinder(hx, hy, hz, 0.0030, 0.0045, c_brass, axis='y', segments=10)
        Primitive3D.draw_cylinder(hx, hy + 0.0030, hz, 0.0013, 0.0020, c_screw, axis='y', segments=8)

        # 6. SOPORTE DE CONEXIÓN EN EL CARRO MÓVIL (Punto C en la cara posterior)
        Primitive3D.draw_box(cx_pin - 0.004, cy_pin - 0.004, cz_pin, 0.008, 0.012, 0.012, (0.28, 0.30, 0.34))
        # Rótula esférica de latón y perno articulado en el carro
        Primitive3D.draw_cylinder(cx_pin, cy_pin, cz_pin, 0.0030, 0.0045, c_brass, axis='y', segments=10)
        Primitive3D.draw_cylinder(cx_pin, cy_pin + 0.0030, cz_pin, 0.0013, 0.0020, c_screw, axis='y', segments=8)

        # 7. BIELA DE EMPUJE (PUSHROD) EN ACERO INOXIDABLE PULIDO
        # Conecta rígidamente la punta del horn H con el perno del carro C.
        # Longitud matemáticamente invariante L = 400.00 mm (cero estiramiento ni contracción).
        Primitive3D.draw_rod((hx, hy, hz), (cx_pin, cy_pin, cz_pin), radius=0.0016, color=(0.88, 0.90, 0.93), segments=12)

    def draw_operator_pedestal(self, fsm_state: str, e_stop: bool):
        """
        Consola de mando ergonómica del operador con seta de Parada de Emergencia,
        pulsadores industriales y focos piloto LED de estado integrados en la propia botonera.
        (Se elimina el mástil/semáforo elevado para evitar interferencias visuales).
        """
        px, py, pz = -0.15, 0.25, -0.30

        # 1. Columna del pedestal de acero y base en el suelo
        Primitive3D.draw_cylinder(px, 0.20, pz, 0.022, 0.40, self.C_STAND, axis='y')
        Primitive3D.draw_cylinder(px, 0.01, pz, 0.085, 0.02, (0.12, 0.13, 0.15), axis='y')
        Primitive3D.draw_cylinder(px, 0.02, pz, 0.045, 0.015, (0.35, 0.38, 0.42), axis='y')

        # 2. Caja de la consola de mando inclinada hacia el operador (-20 grados)
        glPushMatrix()
        glTranslatef(px, 0.42, pz)
        glRotatef(-20, 1, 0, 0)

        # Gabinete de chapa plegada industrial gris claro
        Primitive3D.draw_box(0, 0, 0, 0.13, 0.055, 0.15, (0.86, 0.88, 0.90))
        # Carátula frontal oscura / panel de aluminio anodizado
        panel_y = 0.028
        Primitive3D.draw_box(0, panel_y, 0, 0.122, 0.002, 0.142, (0.18, 0.20, 0.24))

        # 3. LÓGICA DE FOCOS PILOTO LED (Integrados directamente en la consola)
        c_red_off = (0.28, 0.05, 0.05)
        c_yel_off = (0.28, 0.22, 0.04)
        c_grn_off = (0.04, 0.25, 0.06)

        c_red_on = (1.00, 0.12, 0.12)
        c_yel_on = (0.98, 0.84, 0.12)
        c_grn_on = (0.12, 0.98, 0.18)

        is_emerg = e_stop or ("EMERG" in fsm_state)
        if is_emerg:
            blink = (int(time.time() * 6.0) % 2) == 0
            cur_red = c_red_on if blink else c_red_off
            cur_yel = c_yel_off
            cur_grn = c_grn_off
        elif fsm_state in ("TRANSPORTE", "TRANSP", "INSPECCION", "INSPEC", "DESVIO"):
            cur_red = c_red_off
            cur_yel = c_yel_off
            cur_grn = c_grn_on
        elif fsm_state in ("REPOSO", "IDLE"):
            cur_red = c_red_off
            cur_yel = c_yel_on
            cur_grn = c_grn_off
        else:
            cur_red = c_red_off
            cur_yel = c_yel_off
            cur_grn = c_grn_off

        # Fila superior de 3 pequeños focos piloto LED industriales con biseles metálicos cromados
        led_z = -0.045
        led_y = panel_y + 0.003
        c_chrome = (0.82, 0.84, 0.88)

        # Foco Piloto ROJO (Falla / Emergencia) en X = -0.036
        Primitive3D.draw_cylinder(-0.036, led_y, led_z, 0.0065, 0.003, c_chrome, axis='y', segments=12)
        Primitive3D.draw_cylinder(-0.036, led_y + 0.002, led_z, 0.0045, 0.004, cur_red, axis='y', segments=12)

        # Foco Piloto ÁMBAR / AMARILLO (Reposo / Standby) en X = 0.0
        Primitive3D.draw_cylinder(0.0, led_y, led_z, 0.0065, 0.003, c_chrome, axis='y', segments=12)
        Primitive3D.draw_cylinder(0.0, led_y + 0.002, led_z, 0.0045, 0.004, cur_yel, axis='y', segments=12)

        # Foco Piloto VERDE (Marcha / Ejecución) en X = +0.036
        Primitive3D.draw_cylinder(0.036, led_y, led_z, 0.0065, 0.003, c_chrome, axis='y', segments=12)
        Primitive3D.draw_cylinder(0.036, led_y + 0.002, led_z, 0.0045, 0.004, cur_grn, axis='y', segments=12)

        # 4. SETA DE PARADA DE EMERGENCIA INDUSTRIAL (Centro del panel, Z = -0.005)
        # Corona/disco amarillo de seguridad normalizado
        Primitive3D.draw_cylinder(0.0, panel_y + 0.002, -0.005, 0.024, 0.002, (0.95, 0.82, 0.10), axis='y', segments=20)
        Primitive3D.draw_cylinder(0.0, panel_y + 0.004, -0.005, 0.019, 0.002, (0.12, 0.12, 0.14), axis='y', segments=16)

        # Botón tipo seta rojo (se hunde al estar activado)
        e_color = (0.92, 0.15, 0.15) if not e_stop else (0.45, 0.08, 0.08)
        e_cap_y = panel_y + (0.016 if not e_stop else 0.009)
        # Cuello metálico
        Primitive3D.draw_cylinder(0.0, panel_y + 0.008, -0.005, 0.008, 0.010, (0.65, 0.68, 0.72), axis='y', segments=14)
        # Cabeza de la seta
        Primitive3D.draw_cylinder(0.0, e_cap_y, -0.005, 0.017, 0.014, e_color, axis='y', segments=20)

        # 5. PULSADORES DE MANDO DEL OPERADOR (Fila inferior, Z = +0.038)
        btn_z = 0.038
        btn_y = panel_y + 0.003

        # Pulsador VERDE (MARCHA / START) en X = -0.032
        Primitive3D.draw_cylinder(-0.032, btn_y, btn_z, 0.010, 0.003, c_chrome, axis='y', segments=14)
        Primitive3D.draw_cylinder(-0.032, btn_y + 0.003, btn_z, 0.0075, 0.005, (0.15, 0.75, 0.25), axis='y', segments=14)

        # Pulsador NEGRO / STOP (PARADA REGULAR) en X = +0.032
        Primitive3D.draw_cylinder(0.032, btn_y, btn_z, 0.010, 0.003, c_chrome, axis='y', segments=14)
        Primitive3D.draw_cylinder(0.032, btn_y + 0.003, btn_z, 0.0075, 0.005, (0.18, 0.19, 0.22), axis='y', segments=14)

        glPopMatrix()


# =====================================================================================
# SISTEMA DE INTERFAZ DE USUARIO 2D (HUD OVERLAY RETRO)
# =====================================================================================
class HUDOverlay:
    """Consola retro industrial unificada en la parte inferior de la pantalla con textura preasignada."""
    def __init__(self, screen_w: int, screen_h: int):
        self.screen_w = screen_w
        self.screen_h = screen_h
        self.console_h = 195
        self.console_y = screen_h - self.console_h
        
        pygame.font.init()
        self.font_title = pygame.font.SysFont("Consolas", 13, bold=True)
        self.font_mono = pygame.font.SysFont("Consolas", 12, bold=True)
        self.font_mono_small = pygame.font.SysFont("Consolas", 11)
        self.font_bold = pygame.font.SysFont("Segoe UI", 12, bold=True)
        self.font_digits = pygame.font.SysFont("Consolas", 15, bold=True)

        self.buttons = []
        self._init_buttons()

        # Superficie de renderizado 2D y Textura OpenGL única pre-asignada (Zero Memory Leak)
        self.hud_surf = pygame.Surface((self.screen_w, self.console_h))
        self.hud_tex = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, self.hud_tex)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.screen_w, self.console_h, 0, GL_RGBA, GL_UNSIGNED_BYTE, None)

    def resize(self, w: int, h: int):
        self.screen_w = w
        self.screen_h = h
        self.console_y = h - self.console_h
        self._init_buttons()
        self.hud_surf = pygame.Surface((self.screen_w, self.console_h))
        glBindTexture(GL_TEXTURE_2D, self.hud_tex)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.screen_w, self.console_h, 0, GL_RGBA, GL_UNSIGNED_BYTE, None)

    def _init_buttons(self):
        console_y = self.screen_h - self.console_h
        margin = 10
        w1 = 430
        w2 = 320
        x3 = margin + w1 + 10 + w2 + 10
        w3 = max(390, self.screen_w - x3 - margin)

        desk_x = x3 + 8
        desk_w = w3 - 16

        btn_y1 = console_y + 36
        btn_h1 = 34
        b1_w = (desk_w - 16) // 3
        b1_w_estop = desk_w - 16 - (b1_w * 2)

        self.buttons = [
            {"id": "START", "label": "INICIAR", "color": (34, 139, 34), "rect": pygame.Rect(desk_x, btn_y1, b1_w, btn_h1), "state": "NORMAL"},
            {"id": "STOP", "label": "DETENER", "color": (160, 82, 45), "rect": pygame.Rect(desk_x + b1_w + 8, btn_y1, b1_w, btn_h1), "state": "NORMAL"},
            {"id": "ESTOP", "label": "! E-STOP !", "color": (178, 34, 34), "rect": pygame.Rect(desk_x + (b1_w + 8) * 2, btn_y1, b1_w_estop, btn_h1), "state": "ESTOP"},
        ]

        btn_y2 = btn_y1 + btn_h1 + 8
        btn_h2 = 28
        b2_w = (desk_w - 24) // 4
        b2_w_last = desk_w - 24 - (b2_w * 3)

        self.buttons.extend([
            {"id": "ADD_BOX", "label": "+ CAJA", "color": (70, 80, 95), "rect": pygame.Rect(desk_x, btn_y2, b2_w, btn_h2), "state": "NORMAL"},
            {"id": "RESET", "label": "RESET HW", "color": (50, 90, 120), "rect": pygame.Rect(desk_x + b2_w + 8, btn_y2, b2_w, btn_h2), "state": "NORMAL"},
            {"id": "LIGHT", "label": "LUZ +/-", "color": (80, 85, 70), "rect": pygame.Rect(desk_x + (b2_w + 8) * 2, btn_y2, b2_w, btn_h2), "state": "NORMAL"},
            {"id": "WOKWI", "label": "WOKWI HIL", "color": (15, 95, 145), "rect": pygame.Rect(desk_x + (b2_w + 8) * 3, btn_y2, b2_w_last, btn_h2), "state": "NORMAL"},
        ])

        v_y = btn_y2 + btn_h2 + 22
        v_h = 24
        v_w = (desk_w - 24) // 5

        v_labels = ["V1:ISO", "V2:SEN", "V3:LUZ", "V4:SRV", "V5:TOL"]
        for i in range(5):
            cur_w = v_w if i < 4 else desk_w - (v_w + 6) * 4
            self.buttons.append({
                "id": f"V{i+1}",
                "label": v_labels[i],
                "color": (55, 65, 78),
                "rect": pygame.Rect(desk_x + i * (v_w + 6), v_y, cur_w, v_h),
                "state": "PRESET"
            })

    def render(self, state: PlantState, fps: float, counts: Optional[Tuple[int, int, int, int]] = None):
        self.console_y = self.screen_h - self.console_h
        margin = 10
        gap = 10

        # Limpiar superficie de la consola y dibujar marco exterior
        self.hud_surf.fill((26, 30, 36))
        pygame.draw.rect(self.hud_surf, (70, 78, 90), (0, 0, self.screen_w, 3))
        pygame.draw.rect(self.hud_surf, (15, 18, 22), (0, 3, self.screen_w, 1))

        self._draw_text(f"FPS: {fps:4.1f}", self.screen_w - 380, self.console_y + 8, (140, 160, 180), self.font_mono)

        x1 = margin
        y_top = self.console_y + 26
        h_mods = self.console_h - 34
        w1 = 430
        self._render_crt_screen(x1, y_top, w1, h_mods, state)

        x2 = x1 + w1 + gap
        w2 = 320
        self._render_stats_screen(x2, y_top, w2, h_mods, state, counts)

        x3 = x2 + w2 + gap
        w3 = self.screen_w - x3 - margin
        self._render_control_desk(x3, y_top, w3, h_mods)

        # Transferencia a GPU mediante glTexSubImage2D en la textura preasignada (CERO asignaciones dinámicas)
        data = pygame.image.tobytes(self.hud_surf, "RGBA", True)
        glBindTexture(GL_TEXTURE_2D, self.hud_tex)
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, self.screen_w, self.console_h, GL_RGBA, GL_UNSIGNED_BYTE, data)

        # Proyección ortogonal 2D en pantalla
        glViewport(0, 0, self.screen_w, self.screen_h)
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, self.screen_w, self.screen_h, 0, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, self.hud_tex)
        glColor4f(1.0, 1.0, 1.0, 1.0)
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 1.0); glVertex2i(0, self.console_y)
        glTexCoord2f(1.0, 1.0); glVertex2i(self.screen_w, self.console_y)
        glTexCoord2f(1.0, 0.0); glVertex2i(self.screen_w, self.screen_h)
        glTexCoord2f(0.0, 0.0); glVertex2i(0, self.screen_h)
        glEnd()
        glDisable(GL_TEXTURE_2D)
        glDisable(GL_BLEND)

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()

    def _render_crt_screen(self, x: int, y: int, w: int, h: int, state: PlantState):
        self._draw_rect(x, y, w, h, (10, 12, 14))
        self._draw_border(x, y, w, h, (40, 48, 56), (5, 6, 8))
        self._draw_rect(x + 4, y + 4, w - 8, h - 8, (8, 24, 16))
        self._draw_rect(x + 4, y + 4, w - 8, 16, (14, 38, 26))
        self._draw_text("TELEMETRÍA", x + 10, y + 6, (80, 220, 140), self.font_mono_small)

        tx = x + 14
        ty = y + 26
        dy = 17
        c_green = (100, 255, 160)
        c_val = (180, 255, 200)

        fsm_str = state.fsm_state[:8]
        self._draw_text("• [FSM]:", tx, ty, c_green, self.font_mono)
        self._draw_text(f"{fsm_str:<8} (D11/D12/D13)", tx + 75, ty, (255, 220, 80) if "EMERG" in fsm_str else c_val, self.font_mono)

        ty += dy
        self._draw_text("• ZT-101 (Dist):", tx, ty, c_green, self.font_mono)
        self._draw_text(f"{state.ultrasonic_dist_cm:5.1f} cm (SP=25.0 cm)", tx + 135, ty, c_val, self.font_mono)

        ty += dy
        self._draw_text("• M-101 (Motor):", tx, ty, c_green, self.font_mono)
        self._draw_text(f"{state.motor_pwm:3d}/200 PWM ({state.belt_speed_mps*100:4.1f} cm/s)", tx + 135, ty, c_val, self.font_mono)

        ty += dy
        self._draw_text("• QL-103 (LED):", tx, ty, c_green, self.font_mono)
        self._draw_text(f"{state.led_pwm:3d}/255 PWM (SP={state.sp_lux_pct:2.0f}%)", tx + 135, ty, c_val, self.font_mono)

        ty += dy
        self._draw_text("• QE-103 (Lux):", tx, ty, c_green, self.font_mono)
        self._draw_text(f"{state.total_lux_pct:5.1f}% [Amb:{state.ambient_lux_pct:4.1f}%]", tx + 135, ty, c_val, self.font_mono)

        ty += dy
        self._draw_text("• Carro M-102:", tx, ty, c_green, self.font_mono)
        chute_tag = "CONFORME" if abs(state.cart_z) < 0.038 else ("RETRABAJO" if state.cart_z > 0 else "RECHAZO")
        self._draw_text(f"Z={state.cart_z:+5.2f}m [{chute_tag}]", tx + 135, ty, c_val, self.font_mono)

        ty += dy
        self._draw_text("• E-STOP (D2):", tx, ty, c_green, self.font_mono)
        es_str = "DISPARADO !!" if state.emergency_stop else "ARMADO / SEGURO"
        self._draw_text(es_str, tx + 135, ty, (255, 80, 80) if state.emergency_stop else (100, 220, 140), self.font_mono)

        ty += dy
        hil_on = getattr(state, 'hil_connected', False)
        hil_str = "WOKWI (TCP 4000)" if hil_on else "LOCAL (PYTHON)"
        hil_col = (100, 255, 140) if hil_on else (255, 190, 70)
        self._draw_text("• MODO HIL:", tx, ty, c_green, self.font_mono)
        self._draw_text(hil_str, tx + 135, ty, hil_col, self.font_mono)

    def _render_stats_screen(self, x: int, y: int, w: int, h: int, state: PlantState, counts: Optional[Tuple[int, int, int, int]]):
        self._draw_rect(x, y, w, h, (10, 12, 14))
        self._draw_border(x, y, w, h, (40, 48, 56), (5, 6, 8))
        self._draw_rect(x + 4, y + 4, w - 8, h - 8, (16, 20, 26))
        self._draw_rect(x + 4, y + 4, w - 8, 16, (26, 32, 42))
        self._draw_text("MATRIZ OEE Y PRODUCCIÓN", x + 10, y + 6, (180, 200, 225), self.font_mono_small)

        tot, conf, retr, rech = counts if counts else (0, 0, 0, 0)
        p_c = (conf / tot * 100.0) if tot > 0 else 0.0
        p_r = (retr / tot * 100.0) if tot > 0 else 0.0
        p_f = (rech / tot * 100.0) if tot > 0 else 0.0

        grid_y = y + 26
        gw = (w - 24) // 2
        gh = 42

        # TOTAL
        self._draw_rect(x + 8, grid_y, gw, gh, (24, 30, 40))
        self._draw_text("TOTAL PIEZAS", x + 14, grid_y + 4, (140, 160, 180), self.font_mono_small)
        self._draw_text(f"{tot:4d} u", x + 14, grid_y + 18, (240, 245, 255), self.font_digits)

        # CONFORME
        self._draw_rect(x + 16 + gw, grid_y, gw, gh, (18, 38, 24))
        self._draw_text("CONFORME (90°)", x + 22 + gw, grid_y + 4, (120, 200, 140), self.font_mono_small)
        self._draw_text(f"{conf:3d} ({p_c:4.1f}%)", x + 22 + gw, grid_y + 18, (100, 255, 140), self.font_digits)

        grid_y += gh + 6
        # RETRABAJO
        self._draw_rect(x + 8, grid_y, gw, gh, (42, 34, 16))
        self._draw_text("RETRABAJO (45°)", x + 14, grid_y + 4, (220, 180, 90), self.font_mono_small)
        self._draw_text(f"{retr:3d} ({p_r:4.1f}%)", x + 14, grid_y + 18, (255, 200, 80), self.font_digits)

        # RECHAZO
        self._draw_rect(x + 16 + gw, grid_y, gw, gh, (42, 20, 20))
        self._draw_text("RECHAZO (135°)", x + 22 + gw, grid_y + 4, (220, 120, 120), self.font_mono_small)
        self._draw_text(f"{rech:3d} ({p_f:4.1f}%)", x + 22 + gw, grid_y + 18, (255, 100, 100), self.font_digits)

        # Barra de Estado Hardware
        stat_y = grid_y + gh + 8
        self._draw_rect(x + 8, stat_y, w - 16, 26, (18, 22, 28))
        led_spacing = (w - 20) // 4

        l1_on = not state.emergency_stop
        l2_on = state.motor_pwm > 0
        l3_on = state.ultrasonic_dist_cm < 45.0
        l4_on = state.led_pwm > 30

        labels = [("SYS OK", l1_on, (40, 220, 80)),
                  ("RUN", l2_on, (40, 200, 80)),
                  ("SONAR", l3_on, (60, 180, 255)),
                  ("LUZ PID", l4_on, (255, 200, 40))]

        for i, (txt, is_on, col) in enumerate(labels):
            lx = x + 12 + i * led_spacing
            ly = stat_y + 7
            c_fill = col if is_on else (40, 48, 56)
            self._draw_rect(lx, ly, 10, 10, c_fill)
            self._draw_border(lx, ly, 10, 10, (180, 190, 200) if is_on else (60, 70, 80), (10, 12, 14))
            self._draw_text(txt, lx + 14, ly - 1, (200, 210, 220) if is_on else (100, 110, 120), self.font_mono_small)

    def _render_control_desk(self, x: int, y: int, w: int, h: int):
        self._draw_rect(x, y, w, h, (32, 36, 44))
        self._draw_border(x, y, w, h, (60, 68, 80), (15, 18, 22))
        self._draw_rect(x + 4, y + 4, w - 8, 16, (44, 50, 62))

        for btn in self.buttons:
            rect = btn["rect"]
            col = btn["color"]
            label = btn["label"]
            st = btn.get("state", "NORMAL")

            if st == "ESTOP":
                for sx in range(rect.x - 4, rect.x + rect.w + 4, 12):
                    self._draw_rect(sx, rect.y - 4, 6, rect.h + 8, (220, 180, 20))
                self._draw_rect(rect.x, rect.y, rect.w, rect.h, col)
                self._draw_border(rect.x, rect.y, rect.w, rect.h, (255, 100, 100), (80, 10, 10))
                self._draw_text_centered(label, rect.centerx, rect.centery, (255, 255, 255), self.font_bold)
            else:
                self._draw_rect(rect.x, rect.y, rect.w, rect.h, col)
                c_light = (min(255, col[0] + 40), min(255, col[1] + 40), min(255, col[2] + 40))
                c_dark = (max(0, col[0] - 30), max(0, col[1] - 30), max(0, col[2] - 30))
                self._draw_border(rect.x, rect.y, rect.w, rect.h, c_light, c_dark)
                self._draw_text_centered(label, rect.centerx, rect.centery, (240, 245, 250), self.font_bold if st != "PRESET" else self.font_mono_small)

    def _draw_rect(self, x: int, y: int, w: int, h: int, color: Tuple[int, int, int]):
        ry = y - self.console_y
        c = tuple(max(0, min(255, int(v))) for v in color)
        pygame.draw.rect(self.hud_surf, c, (x, ry, w, h))

    def _draw_border(self, x: int, y: int, w: int, h: int, light_col: Tuple[int, int, int], dark_col: Tuple[int, int, int]):
        ry = y - self.console_y
        c_light = tuple(max(0, min(255, int(c))) for c in light_col)
        c_dark = tuple(max(0, min(255, int(c))) for c in dark_col)
        pygame.draw.line(self.hud_surf, c_light, (x, ry), (x + w, ry))
        pygame.draw.line(self.hud_surf, c_light, (x, ry), (x, ry + h))
        pygame.draw.line(self.hud_surf, c_dark, (x + w, ry), (x + w, ry + h))
        pygame.draw.line(self.hud_surf, c_dark, (x, ry + h), (x + w, ry + h))

    def _draw_text(self, text: str, x: int, y: int, color: Tuple[int, int, int], font: pygame.font.Font):
        surf = font.render(text, True, color)
        ry = y - self.console_y
        self.hud_surf.blit(surf, (x, ry))

    def _draw_text_centered(self, text: str, cx: int, cy: int, color: Tuple[int, int, int], font: pygame.font.Font, shadow: bool = True):
        surf = font.render(text, True, color)
        w, h = surf.get_size()
        rx = cx - w // 2
        ry = (cy - self.console_y) - h // 2
        if shadow:
            sh = font.render(text, True, (10, 15, 20))
            self.hud_surf.blit(sh, (rx + 1, ry + 1))
        self.hud_surf.blit(surf, (rx, ry))


# =====================================================================================
# BUCLE PRINCIPAL DE LA APLICACIÓN 3D
# =====================================================================================
class DigitalTwin3DApp:
    """Aplicación principal del gemelo digital 3D de la estación logística."""
    def __init__(self, width: int = 1280, height: int = 768):
        self.width = width
        self.height = height
        
        pygame.init()
        pygame.display.set_caption("TIF_G5")
        
        pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLEBUFFERS, 1)
        pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLESAMPLES, 4)
        pygame.display.gl_set_attribute(pygame.GL_DEPTH_SIZE, 24)
        
        self.screen = pygame.display.set_mode(
            (self.width, self.height),
            pygame.DOUBLEBUF | pygame.OPENGL | pygame.RESIZABLE
        )
        self.clock = pygame.time.Clock()

        # Modelos y Subsistemas
        self.camera = Camera3D()
        self.model3d = LogisticsStationModel()
        self.hud = HUDOverlay(self.width, self.height)
        
        # Emulador de física y firmware virtual
        self.physics = PlantPhysicsModel(dt=0.033)
        self.firmware = VirtualArduinoFirmware()
        self.firmware.fsm_state = "REPOSO"

        # Cajas activas en simulación
        self.boxes: List[DynamicBox] = []
        self.box_spawn_timer = 1.0
        
        # Contadores de clasificación
        self.count_total = 0
        self.count_conforme = 0
        self.count_retrabajo = 0
        self.count_rechazo = 0

        # Cliente Hardware-In-The-Loop Wokwi
        self.hil_bridge = WokwiHILBridge()

        self._setup_opengl()

    def toggle_wokwi_hil(self):
        """Conecta o desconecta el puente Hardware-In-The-Loop con Wokwi (Port 4000)."""
        if not self.hil_bridge.is_connected:
            ok, msg = self.hil_bridge.connect()
            if ok:
                print(f"[HIL 3D] {msg}")
                self._update_wokwi_button_state(True)
                if not self.physics.state.emergency_stop:
                    time.sleep(0.05)
                    self.hil_bridge.send_command("START")
            else:
                print(f"[HIL 3D WARN] {msg}")
                self._update_wokwi_button_state(False)
        else:
            self.hil_bridge.disconnect()
            print("[HIL 3D] Desconectado de Wokwi.")
            self._update_wokwi_button_state(False)

    def _update_wokwi_button_state(self, connected: bool):
        for btn in self.hud.buttons:
            if btn["id"] == "WOKWI":
                if connected:
                    btn["label"] = "WOKWI [ON]"
                    btn["color"] = (30, 140, 60)
                else:
                    btn["label"] = "WOKWI HIL"
                    btn["color"] = (15, 95, 145)
                break

    def _setup_opengl(self):
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        # Fondo suave de ingeniería
        glClearColor(0.88, 0.90, 0.93, 1.0)
        
        # Luces de taller
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_POSITION, [1.5, 3.5, 2.0, 0.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.85, 0.85, 0.82, 1.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.45, 0.48, 0.52, 1.0])
        
        glEnable(GL_LIGHT1)
        glLightfv(GL_LIGHT1, GL_POSITION, [-2.0, 2.0, -1.5, 0.0])
        glLightfv(GL_LIGHT1, GL_DIFFUSE, [0.25, 0.28, 0.32, 1.0])

        glEnable(GL_LIGHT2)
        glLightf(GL_LIGHT2, GL_CONSTANT_ATTENUATION, 1.0)
        glLightf(GL_LIGHT2, GL_LINEAR_ATTENUATION, 0.2)

        self._set_projection()

    def _set_projection(self):
        console_h = 195
        view_h = max(100, self.height - console_h)
        glViewport(0, console_h, self.width, view_h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = self.width / float(view_h) if view_h > 0 else 1.0
        gluPerspective(42.0, aspect, 0.05, 50.0)
        glMatrixMode(GL_MODELVIEW)

    def spawn_box(self, quality: Optional[str] = None):
        """Genera una nueva pieza en la tolva de entrada."""
        if quality is None:
            r = random.random()
            if r < 0.65:
                quality = "CONFORME"
            elif r < 0.85:
                quality = "RETRABAJO"
            else:
                quality = "RECHAZO"

        new_box = DynamicBox(quality)
        self.boxes.append(new_box)
        
        self.physics.reset_piece(quality)
        if self.firmware.fsm_state in ("REPOSO", "IDLE"):
            self.firmware.fsm_state = "TRANSP"
        if self.hil_bridge.is_connected and not self.physics.state.emergency_stop:
            self.hil_bridge.send_command("START")

    def run(self):
        running = True
        auto_spawn = True

        while running:
            dt = self.clock.tick(60) / 1000.0
            dt = min(0.05, dt)

            for event in pygame.event.get():
                if event.type == QUIT:
                    running = False
                elif event.type == VIDEORESIZE:
                    self.width, self.height = event.size
                    self._set_projection()
                    self.hud.resize(self.width, self.height)
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        running = False
                    elif event.key == K_SPACE:
                        self.spawn_box()
                    elif event.key == K_w:
                        self.toggle_wokwi_hil()
                    elif event.key == K_1:
                        self.camera.set_preset(1)
                    elif event.key == K_2:
                        self.camera.set_preset(2)
                    elif event.key == K_3:
                        self.camera.set_preset(3)
                    elif event.key == K_4:
                        self.camera.set_preset(4)
                    elif event.key == K_5:
                        self.camera.set_preset(5)
                elif event.type == MOUSEBUTTONDOWN:
                    clicked_hud = False
                    for btn in self.hud.buttons:
                        if btn["rect"].collidepoint(event.pos):
                            clicked_hud = True
                            b_id = btn["id"]
                            if b_id == "START":
                                if self.hil_bridge.is_connected:
                                    self.hil_bridge.send_command("START")
                                self.firmware.fsm_state = "TRANSP"
                                self.physics.state.emergency_stop = False
                            elif b_id == "STOP":
                                if self.hil_bridge.is_connected:
                                    self.hil_bridge.send_command("STOP")
                                self.firmware.fsm_state = "REPOSO"
                            elif b_id == "ADD_BOX":
                                self.spawn_box()
                            elif b_id == "ESTOP":
                                if self.hil_bridge.is_connected:
                                    self.hil_bridge.send_command("STOP")
                                self.physics.trigger_emergency(True)
                                self.firmware.execute_cycle(dt, 50.0, 50.0, True)
                            elif b_id == "RESET":
                                if self.hil_bridge.is_connected:
                                    self.hil_bridge.send_command("RESET")
                                self.physics.trigger_emergency(False)
                                self.firmware.fsm_state = "REPOSO"
                            elif b_id == "LIGHT":
                                if self.physics.state.ambient_lux_pct > 25.0:
                                    self.physics.inject_disturbance(15.0)
                                else:
                                    self.physics.inject_disturbance(65.0)
                            elif b_id == "WOKWI":
                                self.toggle_wokwi_hil()
                            elif b_id in ("V1", "V2", "V3", "V4", "V5"):
                                self.camera.set_preset(int(b_id[1]))
                            break

                    if not clicked_hud:
                        if event.pos[1] < self.height - 195:
                            self.camera.handle_event(event)
                else:
                    self.camera.handle_event(event)

            # Generación automática de cajas (cadencia continua de planta)
            if auto_spawn and self.firmware.fsm_state not in ("EMERG!", "IDLE"):
                self.box_spawn_timer -= dt
                if self.box_spawn_timer <= 0:
                    if not any(b.pos_x < -0.25 for b in self.boxes if b.state in ("EN_FAJA", "CAYENDO_DE_TOLVA")):
                        self.spawn_box()
                    self.box_spawn_timer = random.uniform(3.0, 4.5)

            # Auto-arranque si hay piezas en la faja y el firmware está en reposo
            if any(b.state in ("EN_FAJA", "CAYENDO_DE_TOLVA") for b in self.boxes):
                if self.firmware.fsm_state == "REPOSO":
                    self.firmware.fsm_state = "TRANSP"
                if self.hil_bridge.is_connected and self.hil_bridge.tlm_state.get("fsm_state") == "REPOSO" and not self.physics.state.emergency_stop:
                    self.hil_bridge.send_command("START")

            # Detección sonar: Sensor en X = +0.47 apuntando hacia -X
            sensor_x = 0.47
            closest_dist_cm = 100.0
            approaching_box = None

            for box in self.boxes:
                if box.state == "EN_FAJA":
                    # Borde frontal de la caja mirando hacia el sensor (+X)
                    box_front_x = box.pos_x + 0.0275
                    d_cm = (sensor_x - box_front_x) * 100.0
                    if 0.0 < d_cm < closest_dist_cm:
                        closest_dist_cm = d_cm
                        approaching_box = box

            # 3. Balance fotométrico dinámico (Lazo 3)
            # Solo cuando la caja entra en la zona de aproximación (d <= 14.0 cm), modula la lectura hacia el LDR
            if approaching_box and closest_dist_cm <= 14.0:
                if approaching_box.quality == "CONFORME":
                    box_reflectance = 78.0  # Alta reflectancia
                elif approaching_box.quality == "RETRABAJO":
                    box_reflectance = 52.0  # Media reflectancia
                else:
                    box_reflectance = 28.0  # Baja reflectancia (oscura/defectuosa)
            else:
                box_reflectance = self.physics.state.ambient_lux_pct

            # Ciclo de control: HIL Wokwi vs Emulador Local
            if self.hil_bridge.is_connected and self.hil_bridge.is_active:
                # 1. Transmitir física continua calculada en 3D hacia Wokwi
                self.hil_bridge.sync_sensors(closest_dist_cm, box_reflectance)

                # 2. Leer actuadores gobernados por el firmware de Wokwi
                with self.hil_bridge.lock:
                    tlm = dict(self.hil_bridge.tlm_state)

                fsm_st = tlm["fsm_state"]
                servo_ang = tlm["servo_deg"]
                led_pwm = int(max(0, min(255, tlm["lux_pct"] * 2.55)))

                if tlm["emergency"]:
                    self.physics.state.emergency_stop = True
                elif self.physics.state.emergency_stop and not tlm["emergency"]:
                    self.physics.state.emergency_stop = False

                # Lazo 1 sincronizado: Seguir rigurosamente la ley de control de la simulación Python
                # para asegurar desaceleración suave y parada a exactamente 10.3 cm bajo la cámara
                if self.physics.state.emergency_stop:
                    motor_pwm = 0
                elif fsm_st in ("REPOSO", "IDLE"):
                    motor_pwm = 0
                elif fsm_st == "INSPEC":
                    motor_pwm = 0
                elif fsm_st == "DESVIO":
                    motor_pwm = 180
                elif fsm_st == "TRANSP":
                    if closest_dist_cm <= 11.5:
                        motor_pwm = 0
                    elif closest_dist_cm >= 20.0:
                        motor_pwm = 200
                    else:
                        factor = (closest_dist_cm - 11.5) / 8.5
                        motor_pwm = int(80 + factor * 120)
                        motor_pwm = max(80, min(200, motor_pwm))
                else:
                    motor_pwm = tlm["pwm_motor"]

                self.firmware.fsm_state = fsm_st
                self.firmware.servo_angle = servo_ang
                self.physics.state.hil_connected = True
            else:
                self.physics.state.hil_connected = False
                e_stop_flag = self.physics.state.emergency_stop
                motor_pwm, led_pwm, servo_ang = self.firmware.execute_cycle(
                    dt, closest_dist_cm, box_reflectance, e_stop_flag
                )

            # Sincronizar posición del carro móvil indexador M-102 (tanto en local como en Wokwi)
            # El carro se desplaza hacia la gaveta de clasificación correspondiente a la pieza activa
            active_target_box = None
            for b in self.boxes:
                if b.state in ("EN_FAJA", "EN_DESVIO") and b.pos_x >= 0.28:
                    active_target_box = b
                    break

            if active_target_box:
                if abs(self.firmware.servo_angle - 45.0) < 15.0 or active_target_box.quality == "RETRABAJO":
                    self.firmware.target_cart_z = 0.075   # Desplaza tolva ámbar al centro
                elif abs(self.firmware.servo_angle - 135.0) < 15.0 or active_target_box.quality == "RECHAZO":
                    self.firmware.target_cart_z = -0.075  # Desplaza tolva roja al centro
                else:
                    self.firmware.target_cart_z = 0.0     # Tolva verde en el centro
            elif not any(b.state in ("EN_FAJA", "EN_DESVIO") for b in self.boxes):
                self.firmware.target_cart_z = 0.0

            # Dinámica física continua (integra el movimiento suave del carro hacia target_cart_z)
            self.physics.step(motor_pwm, led_pwm, self.firmware.servo_angle, self.firmware.target_cart_z)
            self.physics.state.fsm_state = self.firmware.fsm_state
            # Sincronizar telemetría de instrumentación 3D en el HUD y CRT
            self.physics.state.ultrasonic_dist_cm = closest_dist_cm
            self.physics.state.total_lux_pct = box_reflectance

            for box in list(self.boxes):
                box.update(dt, self.physics.state.belt_speed_mps, self.physics.state.cart_z)

                if box.state == "FINALIZADA" and box.settle_timer >= 2.5:
                    self.count_total += 1
                    if box.target_chute == 1:
                        self.count_retrabajo += 1
                    elif box.target_chute == 2:
                        self.count_conforme += 1
                    elif box.target_chute == 3:
                        self.count_rechazo += 1
                    self.boxes.remove(box)

            self.camera.update(dt)

            # Renderizado 3D
            self._set_projection()
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glLoadIdentity()
            self.camera.apply()

            glDepthMask(GL_TRUE)
            self.model3d.draw_floor()
            self.model3d.draw_conveyor(self.physics.state.belt_speed_mps, dt)
            self.model3d.draw_discharge_chute()
            self.model3d.draw_feed_hopper()
            self.model3d.draw_motorized_container_shuttle(self.physics.state.cart_z, self.physics.state.servo_angle_deg)
            self.model3d.draw_operator_pedestal(self.physics.state.fsm_state, self.physics.state.emergency_stop)

            # Cajas dinámicas
            for box in self.boxes:
                box.draw()

            # Sensores y conos
            self.model3d.draw_sonar_wave(self.physics.state.ultrasonic_dist_cm)
            self.model3d.draw_inspection_camera(self.physics.state.led_pwm)

            # Renderizado de HUD 2D
            counts_tuple = (self.count_total, self.count_conforme, self.count_retrabajo, self.count_rechazo)
            self.hud.render(self.physics.state, self.clock.get_fps(), counts_tuple)

            pygame.display.flip()

        pygame.quit()


if __name__ == "__main__":
    app = DigitalTwin3DApp()
    app.run()
