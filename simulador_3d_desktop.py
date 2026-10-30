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
        # Espaciado a 10 Hz (100 ms) para mantener la UART 100% holgada y sin desborde
        if now - self.last_send_time < 0.10:
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
        elif index == 3: # Estación de Cámara e Iluminación LED
            self.set_goals([0.05, 0.44, 0.0], 0.65, 75.0, 30.0)
        elif index == 4: # Carro Móvil Indexador M-102 y Contenedores
            self.set_goals([0.55, 0.25, 0.0], 0.90, 130.0, 28.0)
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
    def draw_cylinder(x, y, z, radius, height, color, axis='y', segments=16):
        glColor4f(color[0], color[1], color[2], 1.0)
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

    def compile(self):
        """Compila las caras en una Display List de hardware en la GPU."""
        self.d_list = glGenLists(1)
        glNewList(self.d_list, GL_COMPILE)

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
            self.container_offset_z = -0.16 # Contenedor Ámbar en Z = -0.16
        elif self.quality == "RECHAZO":
            self.target_chute = 3
            self.container_offset_z = 0.16  # Contenedor Rojo en Z = +0.16
        else:
            self.target_chute = 2
            self.container_offset_z = 0.0   # Contenedor Verde en el centro Z = 0.0

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
            if self.pos_y <= 0.425:
                self.pos_y = 0.425
                self.state = "EN_FAJA"

        elif self.state == "EN_FAJA":
            # Avanzar sobre la banda hacia +X
            self.pos_x += belt_speed * dt
            self.pos_y = 0.425
            self.pos_z = 0.0
            self.rotation_y = 0.0
            self.rotation_z = 0.0

            # Al llegar al final de la banda util antes del sensor (X >= 0.41)
            if self.pos_x >= 0.41:
                self.state = "EN_DESVIO"

        elif self.state == "EN_DESVIO":
            # Descenso suave y continuo por la rampa inclinada (-52 grados)
            # Desde X = 0.41, Y = 0.425 hasta X = 0.58, Y = 0.18
            self.pos_x += 0.28 * dt
            self.pos_y -= 0.38 * dt
            self.rotation_z = -52.0 # Inclinacion alineada con la rampa fisica
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
        self.belt_offset_x = 0.0
        self.roller_angle = 0.0
        self.emergency_pressed = False
        
        # Paleta de colores industriales
        self.C_ALU = (0.78, 0.80, 0.83)        # Perfil de aluminio anodizado
        self.C_BELT_RIB = (0.28, 0.30, 0.32)   # Nervaduras de tracción
        self.C_MOTOR = (0.12, 0.45, 0.72)      # Azul motor eléctrico
        self.C_STAND = (0.25, 0.28, 0.32)      # Pedestal operador

        # Cargar y compilar el ensamble CAD de la faja modular GrabCAD
        modelsd_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "modelsd")
        obj_file = os.path.join(modelsd_dir, "faja_modular_estacion_3d.obj")
        self.cad_loader = None
        if os.path.exists(obj_file):
            try:
                self.cad_loader = CADModelLoader(obj_file)
                self.cad_loader.compile()
            except Exception as e:
                print(f"[AVISO CAD] No se pudo compilar Display List: {e}")

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

    def draw_conveyor(self, belt_speed: float, dt: float):
        """Dibuja el ensamble maestro GrabCAD acoplado con nervaduras móviles y rodillos intermedios."""
        # 1. Renderizar geometría CAD compilada en GPU (0% CPU)
        if self.cad_loader:
            self.cad_loader.draw()

        # 2. Rodillos intermedios de soporte superior y tensores de retorno
        self.draw_intermediate_rollers(belt_speed, dt)

        # 3. Nervaduras de la banda móvil en tiempo real
        self.belt_offset_x = (self.belt_offset_x + belt_speed * dt) % 0.035
        glDisable(GL_LIGHTING)
        glColor3f(self.C_BELT_RIB[0], self.C_BELT_RIB[1], self.C_BELT_RIB[2])
        glLineWidth(2.0)
        glBegin(GL_LINES)
        num_ribs = 24
        for i in range(num_ribs):
            rx = -0.40 + i * 0.035 + self.belt_offset_x
            if -0.40 <= rx <= 0.40:
                glVertex3f(rx, 0.404, -0.036)
                glVertex3f(rx, 0.404, 0.036)
        glEnd()
        glEnable(GL_LIGHTING)

    def draw_intermediate_rollers(self, belt_speed: float, dt: float):
        """Dibuja los rodillos intermedios de soporte de cama superior y tensores de retorno."""
        self.roller_angle = (self.roller_angle + (belt_speed / 0.010) * (180.0 / math.pi) * dt) % 360.0

        c_tube = (0.76, 0.78, 0.82)     # Acero inoxidable rectificado
        c_shaft = (0.92, 0.94, 0.96)    # Eje pasante brillante
        c_collar = (0.35, 0.38, 0.42)   # Retenes / rodamientos
        c_bracket = (0.22, 0.24, 0.28)  # Soporte T-Slot negro satinado

        # 1. Cinco rodillos de apoyo superior (cama de deslizamiento de la cinta)
        # La cinta corre a Y = 0.404. El rodillo tiene radio r = 0.010 m (20 mm diámetro)
        # por lo que su centro está en Y = 0.388 m, tocando la cara inferior de la banda.
        top_y = 0.388
        top_x_positions = [-0.27, -0.13, 0.00, 0.13, 0.27]
        for rx in top_x_positions:
            # Eje pasante de acero de 6 mm de diámetro
            Primitive3D.draw_cylinder(rx, top_y, 0.0, 0.003, 0.114, c_shaft, axis='z', segments=12)
            # Tubo cilíndrico rodillo de 20 mm de diámetro (80 mm de ancho de banda)
            Primitive3D.draw_cylinder(rx, top_y, 0.0, 0.010, 0.076, c_tube, axis='z', segments=18)
            # Rodamientos miniatura en extremos del rodillo
            Primitive3D.draw_cylinder(rx, top_y, -0.040, 0.006, 0.004, c_collar, axis='z', segments=12)
            Primitive3D.draw_cylinder(rx, top_y,  0.040, 0.006, 0.004, c_collar, axis='z', segments=12)
            # Soportes de anclaje a las ranuras interiores del perfil T-Slot 2020
            Primitive3D.draw_box(rx, top_y - 0.004, -0.048, 0.012, 0.012, 0.008, c_bracket)
            Primitive3D.draw_box(rx, top_y - 0.004,  0.048, 0.012, 0.012, 0.008, c_bracket)

        # 2. Dos rodillos tensores inferiores para la cinta de retorno
        # Evitan la comba del tramo inferior de la banda a Y = 0.362 m
        ret_y = 0.362
        ret_x_positions = [-0.20, 0.20]
        for rx in ret_x_positions:
            # Eje pasante
            Primitive3D.draw_cylinder(rx, ret_y, 0.0, 0.003, 0.114, c_shaft, axis='z', segments=12)
            # Rodillo de retorno de 16 mm de diámetro
            Primitive3D.draw_cylinder(rx, ret_y, 0.0, 0.008, 0.076, (0.72, 0.74, 0.78), axis='z', segments=16)
            # Bridas de sujeción al perfil
            Primitive3D.draw_box(rx, ret_y + 0.006, -0.048, 0.010, 0.014, 0.008, c_bracket)
            Primitive3D.draw_box(rx, ret_y + 0.006,  0.048, 0.010, 0.014, 0.008, c_bracket)

    def draw_sonar_wave(self, measured_dist_cm: float):
        """Cono volumétrico sonar del sensor HC-SR04 incidiendo a 90° contra el flujo (-X)."""
        # Cápsulas del sensor en X = +0.47, apuntando hacia -X
        cone_len = max(0.04, min(0.38, measured_dist_cm * 0.01))
        pulse = 0.015 * math.sin(time.time() * 8.0)
        Primitive3D.draw_cone(0.47, 0.428, 0.0, 0.040 + pulse, cone_len, (0.2, 0.7, 1.0), alpha=0.18)

    def draw_inspection_camera(self, led_pwm: int):
        """Gantry de inspección óptica (QE-103) y foco LED (QL-103 / Lazo 3) a 10 cm del sensor."""
        gx = 0.370  # Ubicado exactamente a 10 cm del sensor sonar (0.47 - 0.10 = 0.37 m)
        
        # Columnas del pórtico sobre el perfil
        Primitive3D.draw_box(gx, 0.54, -0.085, 0.020, 0.28, 0.020, self.C_ALU)
        Primitive3D.draw_box(gx, 0.54, 0.085, 0.020, 0.28, 0.020, self.C_ALU)
        # Travesaño superior
        Primitive3D.draw_box(gx, 0.67, 0.0, 0.025, 0.025, 0.19, self.C_ALU)

        # Cámara industrial compacta
        Primitive3D.draw_box(gx, 0.61, 0.0, 0.035, 0.055, 0.035, (0.15, 0.16, 0.18))
        Primitive3D.draw_cylinder(gx, 0.56, 0.0, 0.012, 0.025, (0.05, 0.05, 0.05), axis='y')
        Primitive3D.draw_cylinder(gx, 0.55, 0.0, 0.022, 0.008, (0.85, 0.85, 0.88), axis='y')

        # Iluminador LED dinámico
        led_intensity = max(0.05, min(1.0, led_pwm / 255.0))
        glow_color = (0.95 * led_intensity, 0.95 * led_intensity, 0.80 * led_intensity)
        
        light_pos = [gx, 0.55, 0.0, 1.0]
        light_diff = [glow_color[0], glow_color[1], glow_color[2], 1.0]
        glLightfv(GL_LIGHT2, GL_POSITION, light_pos)
        glLightfv(GL_LIGHT2, GL_DIFFUSE, light_diff)
        glLightfv(GL_LIGHT2, GL_SPECULAR, light_diff)

        # Cono de luz pura translúcido proyectado sobre la faja
        glDisable(GL_LIGHTING)
        glDepthMask(GL_FALSE)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE)
        
        if led_intensity > 0.01:
            glPushMatrix()
            glTranslatef(gx, 0.55, 0.0)
            glBegin(GL_TRIANGLE_FAN)
            glColor4f(1.0, 0.98, 0.88, 0.08 + 0.24 * led_intensity)
            glVertex3f(0.0, 0.0, 0.0)
            glColor4f(1.0, 0.95, 0.80, 0.01 + 0.03 * led_intensity)
            for i in range(21):
                ang = 2.0 * math.pi * i / 20.0
                glVertex3f(math.cos(ang) * 0.09, -0.15, math.sin(ang) * 0.09)
            glEnd()
            glPopMatrix()

            # Mancha suave sobre la banda
            glPushMatrix()
            glTranslatef(gx, 0.404, 0.0)
            glBegin(GL_TRIANGLE_FAN)
            glColor4f(1.0, 0.98, 0.85, 0.06 + 0.16 * led_intensity)
            glVertex3f(0.0, 0.0, 0.0)
            glColor4f(1.0, 0.95, 0.80, 0.0)
            for i in range(17):
                ang = 2.0 * math.pi * i / 16.0
                glVertex3f(math.cos(ang) * 0.08, 0.0, math.sin(ang) * 0.08)
            glEnd()
            glPopMatrix()

        glDisable(GL_BLEND)
        glDepthMask(GL_TRUE)
        glEnable(GL_LIGHTING)

        # Sensor LDR lateral
        Primitive3D.draw_box(gx, 0.425, -0.055, 0.012, 0.012, 0.012, (0.4, 0.2, 0.1))

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

    def draw_feed_hopper(self):
        """Tolva de alimentación superior en la entrada (X = -0.42 m)."""
        hx = -0.42
        hy = 0.65

        # Columnas de soporte
        Primitive3D.draw_box(hx, 0.55, -0.075, 0.020, 0.26, 0.020, self.C_ALU)
        Primitive3D.draw_box(hx, 0.55, 0.075, 0.020, 0.26, 0.020, self.C_ALU)
        Primitive3D.draw_box(hx, 0.70, 0.0, 0.025, 0.020, 0.17, self.C_ALU)

        # Embudo de chapa
        Primitive3D.draw_box(hx, hy, -0.065, 0.12, 0.14, 0.006, (0.35, 0.45, 0.58))
        Primitive3D.draw_box(hx, hy, 0.065, 0.12, 0.14, 0.006, (0.35, 0.45, 0.58))
        Primitive3D.draw_box(hx - 0.055, hy, 0.0, 0.006, 0.14, 0.12, (0.30, 0.40, 0.52))
        Primitive3D.draw_box(hx + 0.055, hy, 0.0, 0.006, 0.14, 0.12, (0.30, 0.40, 0.52))

        # Boca de salida
        Primitive3D.draw_box(hx, 0.57, 0.0, 0.09, 0.015, 0.10, (0.85, 0.88, 0.90))

        # Cajas apiladas dentro de la tolva
        Primitive3D.draw_box(hx, 0.60, 0.0, 0.055, 0.045, 0.055, (0.75, 0.65, 0.45))
        Primitive3D.draw_box(hx, 0.65, 0.0, 0.055, 0.045, 0.055, (0.75, 0.65, 0.45))

    def draw_motorized_container_shuttle(self, cart_z: float):
        """Lanzadera transversal indexadora M-102 en X = +0.58 m con 3 gavetas."""
        cx = 0.58
        # Bancada y guías en Z
        Primitive3D.draw_box(cx, 0.02, 0.0, 0.10, 0.04, 0.94, self.C_ALU)
        Primitive3D.draw_box(cx, 0.041, 0.0, 0.015, 0.004, 0.92, (0.40, 0.42, 0.45))
        Primitive3D.draw_cylinder(cx - 0.035, 0.055, 0.0, 0.006, 0.92, (0.90, 0.92, 0.94), axis='z')
        Primitive3D.draw_cylinder(cx + 0.035, 0.055, 0.0, 0.006, 0.92, (0.90, 0.92, 0.94), axis='z')

        # Soportes extremos
        Primitive3D.draw_box(cx, 0.05, -0.46, 0.11, 0.06, 0.03, (0.35, 0.38, 0.42))
        Primitive3D.draw_box(cx, 0.05, 0.46, 0.11, 0.06, 0.03, (0.35, 0.38, 0.42))

        # Motor de indexación
        Primitive3D.draw_box(cx, 0.065, -0.50, 0.050, 0.050, 0.055, (0.22, 0.24, 0.26))
        Primitive3D.draw_box(cx, 0.065, -0.535, 0.040, 0.040, 0.020, self.C_MOTOR)

        # Plataforma móvil portacontenedores
        car_y = 0.08
        Primitive3D.draw_box(cx, car_y, cart_z, 0.16, 0.015, 0.50, self.C_ALU)
        for bz in (-0.18, 0.18):
            Primitive3D.draw_box(cx - 0.035, 0.055, cart_z + bz, 0.025, 0.022, 0.040, (0.40, 0.43, 0.46))
            Primitive3D.draw_box(cx + 0.035, 0.055, cart_z + bz, 0.025, 0.022, 0.040, (0.40, 0.43, 0.46))

        # Contenedores
        bin_dx, bin_dy, bin_dz = 0.14, 0.12, 0.14
        bin_center_y = car_y + 0.008 + bin_dy * 0.5

        # 1. RETRABAJO (Ámbar, Z = cart_z - 0.16)
        z1 = cart_z - 0.16
        Primitive3D.draw_box(cx, bin_center_y, z1, bin_dx, bin_dy, bin_dz, (0.92, 0.70, 0.15))
        Primitive3D.draw_box(cx, bin_center_y + 0.012, z1, bin_dx - 0.02, bin_dy, bin_dz - 0.02, (0.22, 0.16, 0.06))

        # 2. CONFORME (Verde, Z = cart_z)
        z2 = cart_z
        Primitive3D.draw_box(cx, bin_center_y, z2, bin_dx, bin_dy, bin_dz, (0.25, 0.65, 0.30))
        Primitive3D.draw_box(cx, bin_center_y + 0.012, z2, bin_dx - 0.02, bin_dy, bin_dz - 0.02, (0.08, 0.22, 0.10))

        # 3. RECHAZO (Rojo, Z = cart_z + 0.16)
        z3 = cart_z + 0.16
        Primitive3D.draw_box(cx, bin_center_y, z3, bin_dx, bin_dy, bin_dz, (0.85, 0.22, 0.20))
        Primitive3D.draw_box(cx, bin_center_y + 0.012, z3, bin_dx - 0.02, bin_dy, bin_dz - 0.02, (0.25, 0.08, 0.08))

    def draw_operator_pedestal(self, fsm_state: str, e_stop: bool):
        """Pedestal de mando con baliza luminosa y seta de Parada de Emergencia."""
        px, py, pz = -0.15, 0.25, -0.30
        Primitive3D.draw_cylinder(px, 0.20, pz, 0.02, 0.40, self.C_STAND, axis='y')
        Primitive3D.draw_cylinder(px, 0.01, pz, 0.08, 0.02, (0.1, 0.1, 0.1), axis='y')
        
        glPushMatrix()
        glTranslatef(px, 0.42, pz)
        glRotatef(-20, 1, 0, 0)
        Primitive3D.draw_box(0, 0, 0, 0.12, 0.06, 0.14, (0.85, 0.88, 0.90))
        
        e_color = (0.9, 0.1, 0.1) if not e_stop else (0.4, 0.05, 0.05)
        e_y = 0.045 if not e_stop else 0.032
        Primitive3D.draw_cylinder(0.0, e_y, -0.02, 0.018, 0.018, e_color, axis='y')
        Primitive3D.draw_cylinder(0.0, 0.031, -0.02, 0.025, 0.002, (0.95, 0.85, 0.1), axis='y')

        Primitive3D.draw_cylinder(-0.03, 0.033, 0.03, 0.009, 0.008, (0.2, 0.7, 0.2), axis='y')
        Primitive3D.draw_cylinder(0.03, 0.033, 0.03, 0.009, 0.008, (0.2, 0.2, 0.2), axis='y')
        glPopMatrix()

        Primitive3D.draw_cylinder(px, 0.58, pz - 0.05, 0.008, 0.26, (0.7, 0.7, 0.7), axis='y')
        
        c_red = (0.2, 0.0, 0.0)
        c_yel = (0.2, 0.2, 0.0)
        c_grn = (0.0, 0.2, 0.0)

        if e_stop or "EMERG" in fsm_state:
            blink = int(time.time() * 6.0) % 2
            c_red = (1.0, 0.1, 0.1) if blink else (0.3, 0.0, 0.0)
        elif fsm_state in ("TRANSPORTE", "TRANSP", "INSPECCION", "INSPEC", "DESVIO"):
            c_grn = (0.1, 0.95, 0.1)
        elif fsm_state in ("REPOSO", "IDLE"):
            c_yel = (0.95, 0.85, 0.1)

        by = 0.72
        Primitive3D.draw_cylinder(px, by + 0.050, pz - 0.05, 0.018, 0.022, c_red, axis='y')
        Primitive3D.draw_cylinder(px, by + 0.025, pz - 0.05, 0.018, 0.022, c_yel, axis='y')
        Primitive3D.draw_cylinder(px, by + 0.000, pz - 0.05, 0.018, 0.022, c_grn, axis='y')
        Primitive3D.draw_cylinder(px, by + 0.065, pz - 0.05, 0.020, 0.008, (0.1, 0.1, 0.1), axis='y')


# =====================================================================================
# SISTEMA DE INTERFAZ DE USUARIO 2D (HUD OVERLAY RETRO)
# =====================================================================================
class HUDOverlay:
    """Consola retro industrial unificada en la parte inferior de la pantalla."""
    def __init__(self, screen_w: int, screen_h: int):
        self.screen_w = screen_w
        self.screen_h = screen_h
        self.console_h = 195
        
        pygame.font.init()
        self.font_title = pygame.font.SysFont("Consolas", 13, bold=True)
        self.font_mono = pygame.font.SysFont("Consolas", 12, bold=True)
        self.font_mono_small = pygame.font.SysFont("Consolas", 11)
        self.font_bold = pygame.font.SysFont("Segoe UI", 12, bold=True)
        self.font_digits = pygame.font.SysFont("Consolas", 15, bold=True)

        self._text_cache = {}
        self.buttons = []
        self._init_buttons()

    def resize(self, w: int, h: int):
        self.screen_w = w
        self.screen_h = h
        self._init_buttons()

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

        v_labels = ["V1:ISO", "V2:SEN", "V3:CAM", "V4:CAR", "V5:TOL"]
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

        console_y = self.screen_h - self.console_h
        margin = 10
        gap = 10

        self._draw_rect(0, console_y, self.screen_w, self.console_h, (26, 30, 36))
        self._draw_rect(0, console_y, self.screen_w, 3, (70, 78, 90))
        self._draw_rect(0, console_y + 3, self.screen_w, 1, (15, 18, 22))

        self._draw_text("[ CONSOLA DE SUPERVISIÓN INDUSTRIAL // AUTO-TIF v2.5 ]", 14, console_y + 8, (200, 210, 225), self.font_title)
        self._draw_text(f"GrabCAD CAD Engine | FPS: {fps:4.1f} | ATmega328P Cooperativo", self.screen_w - 380, console_y + 8, (140, 160, 180), self.font_mono)

        x1 = margin
        y_top = console_y + 26
        h_mods = self.console_h - 34
        w1 = 430
        self._render_crt_screen(x1, y_top, w1, h_mods, state)

        x2 = x1 + w1 + gap
        w2 = 320
        self._render_stats_screen(x2, y_top, w2, h_mods, state, counts)

        x3 = x2 + w2 + gap
        w3 = self.screen_w - x3 - margin
        self._render_control_desk(x3, y_top, w3, h_mods)

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
        self._draw_text("● TELEMETRÍA FSM & LAZOS (CRT-1)", x + 10, y + 6, (80, 220, 140), self.font_mono_small)

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
        chute_tag = "CONFORME" if abs(state.cart_z) < 0.08 else ("RETRABAJO" if state.cart_z > 0 else "RECHAZO")
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
        self._draw_text("■ MATRIZ OEE Y PRODUCCIÓN (CRT-2)", x + 10, y + 6, (180, 200, 225), self.font_mono_small)

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
        self._draw_text("TABLERO DE ACCIONAMIENTO INDUSTRIAL", x + 10, y + 6, (220, 230, 245), self.font_title)

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
                self._draw_border(rect.x, rect.y, rect.w, rect.h, (col[0]+40, col[1]+40, col[2]+40), (col[0]-30, col[1]-30, col[2]-30))
                self._draw_text_centered(label, rect.centerx, rect.centery, (240, 245, 250), self.font_bold if st != "PRESET" else self.font_mono_small)

    def _draw_rect(self, x: int, y: int, w: int, h: int, color: Tuple[int, int, int]):
        glColor3ub(color[0], color[1], color[2])
        glBegin(GL_QUADS)
        glVertex2i(x, y)
        glVertex2i(x + w, y)
        glVertex2i(x + w, y + h)
        glVertex2i(x, y + h)
        glEnd()

    def _draw_border(self, x: int, y: int, w: int, h: int, light_col: Tuple[int, int, int], dark_col: Tuple[int, int, int]):
        glColor3ub(light_col[0], light_col[1], light_col[2])
        glBegin(GL_LINES)
        glVertex2i(x, y); glVertex2i(x + w, y)
        glVertex2i(x, y); glVertex2i(x, y + h)
        glEnd()
        glColor3ub(dark_col[0], dark_col[1], dark_col[2])
        glBegin(GL_LINES)
        glVertex2i(x + w, y); glVertex2i(x + w, y + h)
        glVertex2i(x, y + h); glVertex2i(x + w, y + h)
        glEnd()

    def _draw_text(self, text: str, x: int, y: int, color: Tuple[int, int, int], font: pygame.font.Font):
        key = (text, color, font)
        cached = self._text_cache.get(key)
        if cached is not None:
            tex, w, h = cached
        else:
            surf = font.render(text, True, color)
            data = pygame.image.tobytes(surf, "RGBA", True)
            w, h = surf.get_size()
            
            tex = glGenTextures(1)
            glBindTexture(GL_TEXTURE_2D, tex)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data)
            
            if len(self._text_cache) > 250:
                old_keys = list(self._text_cache.keys())[:80]
                for k in old_keys:
                    old_tex, _, _ = self._text_cache.pop(k)
                    try:
                        glDeleteTextures([old_tex])
                    except Exception:
                        pass
            self._text_cache[key] = (tex, w, h)

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE)
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, tex)
        glColor4f(1.0, 1.0, 1.0, 1.0)
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 1.0); glVertex2f(x, y)
        glTexCoord2f(1.0, 1.0); glVertex2f(x + w, y)
        glTexCoord2f(1.0, 0.0); glVertex2f(x + w, y + h)
        glTexCoord2f(0.0, 0.0); glVertex2f(x, y + h)
        glEnd()
        glDisable(GL_TEXTURE_2D)

    def _draw_text_centered(self, text: str, cx: int, cy: int, color: Tuple[int, int, int], font: pygame.font.Font, shadow: bool = True):
        surf = font.render(text, True, color)
        w, h = surf.get_size()
        if shadow:
            self._draw_text(text, cx - w//2 + 1, cy - h//2 + 1, (10, 15, 20), font)
        self._draw_text(text, cx - w//2, cy - h//2, color, font)


# =====================================================================================
# BUCLE PRINCIPAL DE LA APLICACIÓN 3D
# =====================================================================================
class DigitalTwin3DApp:
    """Aplicación principal del gemelo digital 3D de la estación logística."""
    def __init__(self, width: int = 1280, height: int = 768):
        self.width = width
        self.height = height
        
        pygame.init()
        pygame.display.set_caption("Estación Logística 3D - Gemelo Digital Industrial (PyOpenGL)")
        
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
            # El lector óptico / LDR está ubicado en la zona de inspección a 10 cm del sensor ultrasónico
            # Solo modula la reflectancia de la pieza cuando esta se ubica dentro de la zona de inspección (d <= 10.0 cm)
            if approaching_box and closest_dist_cm <= 10.0:
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

                motor_pwm = tlm["pwm_motor"]
                led_pwm = int(max(0, min(255, tlm["lux_pct"] * 2.55)))
                servo_ang = tlm["servo_deg"]
                fsm_st = tlm["fsm_state"]

                self.firmware.fsm_state = fsm_st
                self.firmware.servo_angle = servo_ang
                
                # Mover carro móvil indexador M-102 según el ángulo del servo de Wokwi
                if servo_ang <= 60:
                    self.firmware.target_cart_z = 0.16   # Retrabajo (45°) -> Tolva ámbar
                elif servo_ang >= 115:
                    self.firmware.target_cart_z = -0.16  # Rechazo (135°) -> Tolva roja
                else:
                    self.firmware.target_cart_z = 0.0    # Conforme (90°) -> Tolva verde

                if tlm["emergency"]:
                    self.physics.state.emergency_stop = True
                elif self.physics.state.emergency_stop and not tlm["emergency"]:
                    self.physics.state.emergency_stop = False

                self.physics.state.hil_connected = True
            else:
                self.physics.state.hil_connected = False
                e_stop_flag = self.physics.state.emergency_stop
                motor_pwm, led_pwm, servo_ang = self.firmware.execute_cycle(
                    dt, closest_dist_cm, box_reflectance, e_stop_flag
                )

                # Sincronizar posición del carro móvil indexador M-102 en modo local
                active_target_box = None
                for b in self.boxes:
                    if b.state in ("EN_FAJA", "EN_DESVIO") and b.pos_x >= 0.28:
                        active_target_box = b
                        break

                if active_target_box:
                    if active_target_box.quality == "RETRABAJO":
                        self.firmware.servo_angle = 45
                        self.firmware.target_cart_z = 0.16   # Desplaza tolva ámbar al centro
                    elif active_target_box.quality == "RECHAZO":
                        self.firmware.servo_angle = 135
                        self.firmware.target_cart_z = -0.16  # Desplaza tolva roja al centro
                    else:
                        self.firmware.servo_angle = 90
                        self.firmware.target_cart_z = 0.0    # Tolva verde en el centro
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
            self.model3d.draw_motorized_container_shuttle(self.physics.state.cart_z)
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
