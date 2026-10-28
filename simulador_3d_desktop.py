"""
=====================================================================================
GEMELO DIGITAL 3D - ESTACIÓN LOGÍSTICA DE CLASIFICACIÓN Y ENRUTAMIENTO
Motor Gráfico 3D Nativo (OpenGL / Pygame-CE) - Entorno Industrial
=====================================================================================
Este módulo renderiza en una ventana nativa de escritorio un entorno 3D fotorrealista
y técnico de la estación logística controlada por el Arduino Uno R3:

1. Faja transportadora con perfiles de aluminio Bosch Rexroth y banda móvil continua.
2. Motorreductor DC (M-101) con cinemática acoplada a la velocidad de la faja.
3. Sensor ultrasónico HC-SR04 (ZT-101) con cono de detección sonar volumétrico 3D.
4. Gantry de inspección óptica con cámara industrial (QE-103) e iluminador LED (QL-103)
   cuyo foco dinámico responde en tiempo real al lazo fotométrico (Lazo 3).
5. Servocontrol de enrutamiento SG90 (XV-102) con compuerta deflectora móvil (45°, 90°, 135°).
6. Tres tolvas de clasificación por gravedad (Retrabajo, Conforme, Rechazo) con gavetas.
7. Pedestal de mando con baliza luminosa de 3 estados y seta de Parada de Emergencia.
8. Cajas dinámicas físicas que recorren la faja, son sensadas, inspeccionadas y desviadas.
9. Cámara orbital 3D interactiva (rotación, traslación, zoom y 5 vistas preestablecidas).
10. Panel HUD 2D superpuesto con telemetría en tiempo real y botones interactivos.
=====================================================================================
"""

import sys
import os
import math
import time
import random
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
# CLASES DE CÁMARA Y NAVEGACIÓN 3D
# =====================================================================================
class Camera3D:
    """Cámara orbital técnica tipo CAD / Digital Twin."""
    def __init__(self):
        # Coordenadas esféricas
        self.target = [0.05, 0.38, 0.0]  # Punto focal [x, y, z]
        self.distance = 1.65             # Distancia al objetivo (m)
        self.yaw = 45.0                  # Ángulo azimutal (grados)
        self.pitch = 28.0                # Ángulo de elevación (grados)
        
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
            self.set_goals([0.05, 0.38, 0.0], 1.65, 45.0, 28.0)
        elif index == 2: # Entrada y Sensor Ultrasónico
            self.set_goals([0.22, 0.42, 0.0], 0.90, 20.0, 22.0)
        elif index == 3: # Estación de Cámara e Iluminación
            self.set_goals([0.28, 0.46, 0.0], 0.75, 85.0, 32.0)
        elif index == 4: # Carro Móvil Indexador M-102 y Contenedores
            self.set_goals([0.60, 0.25, 0.0], 0.95, 125.0, 28.0)
        elif index == 5: # Tolva de Alimentación
            self.set_goals([-0.45, 0.50, 0.0], 0.85, 315.0, 32.0)

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
                # Trasladar en el plano horizontal respecto al ángulo actual
                rad_yaw = math.radians(self.yaw)
                speed = 0.0015 * self.distance
                # Vector derecho
                rx = -math.sin(rad_yaw) * dx * speed
                rz = math.cos(rad_yaw) * dx * speed
                # Vector adelante/arriba
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
        """Dibuja un paralelepípedo centrado en (x, y, z) con dimensiones (dx, dy, dz)."""
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
        """Dibuja un cilindro centrado con tapas."""
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

        # Tapas
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
    def draw_cone(x, y, z, r_base, height, color, alpha=0.3, segments=16):
        """Dibuja un cono volumétrico translúcido con Z-buffer protegido para no ocultar piezas."""
        glDisable(GL_LIGHTING) # Desactivar sombreado para no oscurecer con tonos grises
        glDepthMask(GL_FALSE)  # No escribir en el Z-buffer para no hacer desaparecer la caja
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE) # Mezcla luminosa
        glColor4f(color[0], color[1], color[2], alpha)
        
        glPushMatrix()
        glTranslatef(x, y, z)
        # Orientar hacia -X
        glRotatef(-90, 0, 0, 1)
        
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(0.0, 0.0, 0.0) # Vértice
        for i in range(segments + 1):
            angle = 2.0 * math.pi * i / segments
            glVertex3f(math.cos(angle) * r_base, height, math.sin(angle) * r_base)
        glEnd()
        glPopMatrix()
        glDisable(GL_BLEND)
        glDepthMask(GL_TRUE)   # Restaurar escritura en Z-buffer
        glEnable(GL_LIGHTING)  # Restaurar iluminación general


# =====================================================================================
# CLASE CAJA (OBJETO FÍSICO DINÁMICO)
# =====================================================================================
class DynamicBox:
    """Representa una pieza de producto que viaja a lo largo de la estación."""
    def __init__(self, quality: str = "CONFORME"):
        self.quality = quality           # "CONFORME", "RETRABAJO", "RECHAZO"
        self.pos_x = -0.46               # Posición longitudinal bajo la tolva de alimentación
        self.pos_y = 0.50                # Inicia en caída por gravedad desde la tolva
        self.pos_z = 0.0                 # Posición transversal
        self.size = (0.07, 0.05, 0.07)   # Dimensiones físicas (L, H, W) en metros
        
        # Estados: "CAYENDO_DE_TOLVA", "EN_FAJA", "EN_DESVIO", "EN_TOLVA", "FINALIZADA"
        self.state = "CAYENDO_DE_TOLVA"
        self.target_chute = 2            # 1: Reproceso, 2: Conforme, 3: Rechazo
        self.container_offset_z = 0.0    # Offset local dentro del carro móvil indexador
        self.rotation_y = 0.0            # Rotación yaw
        self.rotation_z = 0.0            # Inclinación en rampa
        self.settle_timer = 0.0

        if self.quality == "RETRABAJO":
            self.target_chute = 1
            self.container_offset_z = -0.16 # Contenedor Ámbar montado a -0.16m en el carro
        elif self.quality == "RECHAZO":
            self.target_chute = 3
            self.container_offset_z = 0.16  # Contenedor Rojo montado a +0.16m en el carro
        else:
            self.target_chute = 2
            self.container_offset_z = 0.0   # Contenedor Verde en el centro del carro

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
            # Caída libre por gravedad desde la tolva de alimentación hacia la faja
            self.pos_y -= 0.65 * dt
            if self.pos_y <= 0.425:
                self.pos_y = 0.425
                self.state = "EN_FAJA"

        elif self.state == "EN_FAJA":
            # Avanzar sobre la faja hacia adelante
            self.pos_x += belt_speed * dt
            self.pos_y = 0.425
            self.pos_z = 0.0
            self.rotation_y = 0.0
            self.rotation_z = 0.0

            # Al llegar a la cabeza de la faja (X >= 0.49)
            if self.pos_x >= 0.49:
                self.state = "EN_TOLVA"

        elif self.state in ("EN_DESVIO", "EN_TOLVA"):
            # Deslizamiento por el labio de descarga directo al contenedor que está alineado a Z = 0
            self.pos_x += 0.40 * dt
            self.pos_y -= 0.65 * dt
            self.rotation_z = 20.0
            self.pos_z = 0.0 # Cae en el contenedor alineado

            if self.pos_y <= 0.18:
                self.pos_y = 0.18
                self.pos_x = 0.62
                self.rotation_z = 0.0
                self.state = "FINALIZADA"

        elif self.state == "FINALIZADA":
            # Reposa dentro del contenedor y se mueve de forma solidaria con el carro móvil
            self.pos_x = 0.62
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
        
        # Cuerpo principal de la caja (Cartón con tinte de calidad)
        Primitive3D.draw_box(0, 0, 0, self.size[0], self.size[1], self.size[2], self.color)
        
        # Etiqueta blanca de código de barras superior
        Primitive3D.draw_box(0, self.size[1]*0.5 + 0.001, 0, 0.035, 0.001, 0.035, (0.95, 0.95, 0.95))
        # Cinta adhesiva central
        Primitive3D.draw_box(0, self.size[1]*0.5 + 0.0015, 0, self.size[0]*0.9, 0.001, 0.012, (0.75, 0.60, 0.40))
        glPopMatrix()


# =====================================================================================
# MODELO 3D DE LA ESTACIÓN LOGÍSTICA
# =====================================================================================
class LogisticsStationModel:
    """Construcción geométrica CAD de la estación logística industrial."""
    def __init__(self):
        self.belt_offset_x = 0.0
        self.emergency_pressed = False
        
        # Paleta de colores industriales (Clean CAD Engineering)
        self.C_ALU = (0.78, 0.80, 0.83)        # Perfil de aluminio anodizado
        self.C_ALU_DARK = (0.45, 0.48, 0.52)   # Ranuras de perfiles
        self.C_BELT = (0.15, 0.16, 0.18)       # Banda de caucho negro
        self.C_BELT_RIB = (0.28, 0.30, 0.32)   # Nervaduras de tracción
        self.C_MOTOR = (0.12, 0.45, 0.72)      # Azul motor eléctrico
        self.C_SERVO = (0.10, 0.40, 0.85)      # Azul translúcido SG90
        self.C_BLADE = (0.90, 0.90, 0.92)      # Hoja de compuerta de acero
        self.C_CHUTE_CONF = (0.25, 0.65, 0.30) # Tolva Conforme
        self.C_CHUTE_RETR = (0.85, 0.70, 0.15) # Tolva Retrabajo
        self.C_CHUTE_RECH = (0.80, 0.22, 0.20) # Tolva Rechazo
        self.C_SENSOR = (0.18, 0.28, 0.58)     # PCB HC-SR04
        self.C_CAN = (0.85, 0.88, 0.92)        # Cápsulas ultrasónicas
        self.C_STAND = (0.25, 0.28, 0.32)      # Pedestal operador

    def draw_floor(self):
        """Dibuja el suelo técnico y la cuadrícula métrica de referencia."""
        # Suelo base
        Primitive3D.draw_box(0.0, -0.01, 0.0, 3.0, 0.02, 2.5, (0.90, 0.92, 0.94))
        
        # Cuadrícula métrica
        glDisable(GL_LIGHTING)
        glColor3f(0.80, 0.82, 0.85)
        glLineWidth(1.0)
        glBegin(GL_LINES)
        for i in range(-15, 16):
            # Líneas en Z
            glVertex3f(i * 0.1, 0.001, -1.2)
            glVertex3f(i * 0.1, 0.001, 1.2)
            # Líneas en X
            glVertex3f(-1.5, 0.001, i * 0.1)
            glVertex3f(1.5, 0.001, i * 0.1)
        glEnd()

        # Marcas de seguridad amarillas en el suelo alrededor de la estación
        glColor3f(0.92, 0.78, 0.15)
        glLineWidth(3.0)
        glBegin(GL_LINE_LOOP)
        glVertex3f(-0.70, 0.002, -0.45)
        glVertex3f(0.80, 0.002, -0.45)
        glVertex3f(0.80, 0.002, 0.45)
        glVertex3f(-0.70, 0.002, 0.45)
        glEnd()
        glEnable(GL_LIGHTING)

    def draw_conveyor_frame(self, belt_speed: float, dt: float):
        """Dibuja la estructura mecánica de la faja transportadora."""
        # Avance continuo de la textura de la banda
        self.belt_offset_x = (self.belt_offset_x + belt_speed * dt) % 0.04

        # 4 Patas de aluminio estructural 40x40 (X = ±0.45, Z = ±0.09, Y = 0.20)
        for px in (-0.45, 0.45):
            for pz in (-0.09, 0.09):
                # Pata vertical
                Primitive3D.draw_box(px, 0.20, pz, 0.04, 0.40, 0.04, self.C_ALU)
                # Base de nivelación negra en el suelo
                Primitive3D.draw_box(px, 0.01, pz, 0.06, 0.02, 0.06, (0.1, 0.1, 0.1))

        # Travesaños inferiores de refuerzo
        Primitive3D.draw_box(0.0, 0.10, -0.09, 0.90, 0.03, 0.03, self.C_ALU)
        Primitive3D.draw_box(0.0, 0.10, 0.09, 0.90, 0.03, 0.03, self.C_ALU)
        Primitive3D.draw_box(-0.45, 0.10, 0.0, 0.03, 0.03, 0.18, self.C_ALU)
        Primitive3D.draw_box(0.45, 0.10, 0.0, 0.03, 0.03, 0.18, self.C_ALU)

        # Rieles longitudinales superiores portadores de la banda (Y = 0.38)
        Primitive3D.draw_box(0.0, 0.38, -0.095, 1.02, 0.04, 0.03, self.C_ALU)
        Primitive3D.draw_box(0.0, 0.38, 0.095, 1.02, 0.04, 0.03, self.C_ALU)

        # Placa deslizante bajo la faja
        Primitive3D.draw_box(0.0, 0.385, 0.0, 0.98, 0.015, 0.16, (0.35, 0.37, 0.40))

        # Rodillo motor motriz (X = +0.50, Y = 0.385)
        Primitive3D.draw_cylinder(0.50, 0.385, 0.0, 0.025, 0.18, (0.7, 0.72, 0.75), axis='z')
        # Rodillo conducido tensor (X = -0.50, Y = 0.385)
        Primitive3D.draw_cylinder(-0.50, 0.385, 0.0, 0.025, 0.18, (0.7, 0.72, 0.75), axis='z')

        # Superficie de la banda de caucho negra superior (Y = 0.40)
        Primitive3D.draw_box(0.0, 0.40, 0.0, 1.0, 0.005, 0.17, self.C_BELT)

        # Nervaduras/marcas de tracción móviles
        glDisable(GL_LIGHTING)
        glColor3f(self.C_BELT_RIB[0], self.C_BELT_RIB[1], self.C_BELT_RIB[2])
        glLineWidth(2.0)
        glBegin(GL_LINES)
        num_ribs = 25
        for i in range(num_ribs):
            rx = -0.48 + i * 0.04 + self.belt_offset_x
            if -0.49 <= rx <= 0.49:
                glVertex3f(rx, 0.403, -0.075)
                glVertex3f(rx, 0.403, 0.075)
        glEnd()
        glEnable(GL_LIGHTING)

        # Rieles de guía laterales de acero inoxidable (evitan que las cajas caigan de costado)
        Primitive3D.draw_box(-0.05, 0.425, -0.09, 0.90, 0.025, 0.008, (0.85, 0.88, 0.90))
        # Riel derecho con abertura en la zona de desvío
        Primitive3D.draw_box(-0.10, 0.425, 0.09, 0.75, 0.025, 0.008, (0.85, 0.88, 0.90))

        # Motorreductor DC (M-101) acoplado al rodillo delantero
        Primitive3D.draw_cylinder(0.50, 0.385, -0.13, 0.035, 0.08, self.C_MOTOR, axis='z')
        # Caja de engranajes y bornera
        Primitive3D.draw_box(0.50, 0.385, -0.17, 0.05, 0.06, 0.04, (0.2, 0.2, 0.22))
        # Cable flexible de alimentación hacia canaleta
        Primitive3D.draw_cylinder(0.50, 0.30, -0.17, 0.006, 0.15, (0.1, 0.1, 0.1), axis='y')

    def draw_ultrasonic_sensor(self, measured_dist_cm: float):
        """Sensor HC-SR04 (ZT-101) en pórtico mirando contra el flujo de entrada."""
        # Soporte de aluminio en voladizo (X = 0.35, Z = 0.11)
        Primitive3D.draw_box(0.35, 0.46, 0.095, 0.02, 0.12, 0.015, self.C_ALU)
        
        # Placa PCB azul del sensor mirando hacia -X
        Primitive3D.draw_box(0.345, 0.45, 0.0, 0.008, 0.035, 0.08, self.C_SENSOR)
        
        # Dos cápsulas de aluminio (Transmisor TX y Receptor RX) apuntando hacia -X
        Primitive3D.draw_cylinder(0.335, 0.45, -0.022, 0.010, 0.016, self.C_CAN, axis='x')
        Primitive3D.draw_cylinder(0.335, 0.45, 0.022, 0.010, 0.016, self.C_CAN, axis='x')

        # Cono de detección ultrasónica translúcido azul
        # La longitud del cono refleja la distancia medida actual
        cone_len = max(0.05, min(0.40, measured_dist_cm * 0.01))
        # Pulsación armónica sutil
        pulse = 0.02 * math.sin(time.time() * 8.0)
        Primitive3D.draw_cone(0.32, 0.45, 0.0, 0.045 + pulse, cone_len, (0.2, 0.7, 1.0), alpha=0.18)

    def draw_inspection_camera(self, led_pwm: int):
        """Gantry de inspección óptica (QE-103) y foco LED regulado (QL-103 / Lazo 3)."""
        gx = 0.24  # Posición longitudinal del pórtico
        
        # Columnas del pórtico sobre la faja
        Primitive3D.draw_box(gx, 0.54, -0.11, 0.025, 0.28, 0.025, self.C_ALU)
        Primitive3D.draw_box(gx, 0.54, 0.11, 0.025, 0.28, 0.025, self.C_ALU)
        # Travesaño superior
        Primitive3D.draw_box(gx, 0.67, 0.0, 0.03, 0.03, 0.24, self.C_ALU)

        # Cámara industrial compacta (Cuerpo negro)
        Primitive3D.draw_box(gx, 0.61, 0.0, 0.04, 0.06, 0.04, (0.15, 0.16, 0.18))
        # Lente óptico apuntando hacia abajo
        Primitive3D.draw_cylinder(gx, 0.56, 0.0, 0.014, 0.03, (0.05, 0.05, 0.05), axis='y')

        # Anillo de iluminación LED coaxial (QL-103)
        Primitive3D.draw_cylinder(gx, 0.55, 0.0, 0.024, 0.008, (0.85, 0.85, 0.88), axis='y')

        # Brillo dinámico del foco LED proporcional al PWM del Lazo 3 (0..255)
        led_intensity = max(0.05, min(1.0, led_pwm / 255.0))
        glow_color = (0.95 * led_intensity, 0.95 * led_intensity, 0.80 * led_intensity)
        
        # Actualizar luz puntual GL_LIGHT2 en tiempo real
        light_pos = [gx, 0.55, 0.0, 1.0]
        light_diff = [glow_color[0], glow_color[1], glow_color[2], 1.0]
        glLightfv(GL_LIGHT2, GL_POSITION, light_pos)
        glLightfv(GL_LIGHT2, GL_DIFFUSE, light_diff)
        glLightfv(GL_LIGHT2, GL_SPECULAR, light_diff)

        # Cono de luz translúcido visible proyectado hacia la faja (luz pura sin gris)
        glDisable(GL_LIGHTING) # CRUCIAL: Desactivar iluminación general para no teñir el haz de gris
        glDepthMask(GL_FALSE)  # No escribir en Z-buffer para que la caja bajo la cámara no desaparezca
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE) # Luz aditiva pura brillante
        
        if led_intensity > 0.01:
            glPushMatrix()
            glTranslatef(gx, 0.55, 0.0)
            glBegin(GL_TRIANGLE_FAN)
            # Vértice cálido transparente en el LED
            glColor4f(1.0, 0.98, 0.88, 0.08 + 0.24 * led_intensity)
            glVertex3f(0.0, 0.0, 0.0)
            # Base desvanecida a transparente puro (cero bordes grises)
            glColor4f(1.0, 0.95, 0.80, 0.01 + 0.03 * led_intensity)
            for i in range(21):
                ang = 2.0 * math.pi * i / 20.0
                glVertex3f(math.cos(ang) * 0.095, -0.16, math.sin(ang) * 0.095)
            glEnd()
            glPopMatrix()

            # Mancha suave de luz cálida sobre la faja
            glPushMatrix()
            glTranslatef(gx, 0.402, 0.0)
            glBegin(GL_TRIANGLE_FAN)
            glColor4f(1.0, 0.98, 0.85, 0.06 + 0.16 * led_intensity)
            glVertex3f(0.0, 0.0, 0.0)
            glColor4f(1.0, 0.95, 0.80, 0.0)
            for i in range(17):
                ang = 2.0 * math.pi * i / 16.0
                glVertex3f(math.cos(ang) * 0.09, 0.0, math.sin(ang) * 0.09)
            glEnd()
            glPopMatrix()

        glDisable(GL_BLEND)
        glDepthMask(GL_TRUE)   # Restaurar escritura en Z-buffer
        glEnable(GL_LIGHTING)  # Restaurar iluminación general para los objetos opacos

        # Sensor LDR montado lateralmente para medir el balance fotométrico
        Primitive3D.draw_box(gx, 0.43, -0.08, 0.015, 0.015, 0.015, (0.4, 0.2, 0.1))

    # =========================================================================
    # TOLVA / CONTENEDOR DE ALIMENTACIÓN EN LA ENTRADA DE LA FAJA (FEED HOPPER)
    # =========================================================================
    def draw_feed_hopper(self):
        """Contenedor/tolva superior desde donde caen las cajas por gravedad a la faja."""
        hx = -0.46
        hy = 0.62

        # Postes verticales de soporte Bosch Rexroth anclados a la estructura
        Primitive3D.draw_box(hx, 0.54, -0.095, 0.025, 0.28, 0.025, self.C_ALU)
        Primitive3D.draw_box(hx, 0.54, 0.095, 0.025, 0.28, 0.025, self.C_ALU)
        # Travesaño superior
        Primitive3D.draw_box(hx, 0.70, 0.0, 0.03, 0.025, 0.22, self.C_ALU)

        # Embudo / Tolva contenedor de almacenamiento (Acero azulado)
        Primitive3D.draw_box(hx, hy, -0.075, 0.14, 0.16, 0.008, (0.35, 0.45, 0.58))
        Primitive3D.draw_box(hx, hy, 0.075, 0.14, 0.16, 0.008, (0.35, 0.45, 0.58))
        Primitive3D.draw_box(hx - 0.065, hy, 0.0, 0.008, 0.16, 0.14, (0.30, 0.40, 0.52))
        Primitive3D.draw_box(hx + 0.065, hy, 0.0, 0.008, 0.16, 0.14, (0.30, 0.40, 0.52))

        # Boca de descarga inferior acampanada
        Primitive3D.draw_box(hx, 0.52, 0.0, 0.11, 0.02, 0.12, (0.85, 0.88, 0.90))

        # Cajas apiladas de reserva dentro de la tolva (listas para caer)
        Primitive3D.draw_box(hx, 0.57, 0.0, 0.07, 0.05, 0.07, (0.75, 0.65, 0.45))
        Primitive3D.draw_box(hx, 0.63, 0.0, 0.07, 0.05, 0.07, (0.75, 0.65, 0.45))

    def draw_discharge_lip(self):
        """Labio inclinado de acero inoxidable de descarga directa hacia el carro móvil."""
        glPushMatrix()
        glTranslatef(0.53, 0.36, 0.0)
        glRotatef(-28.0, 0, 0, 1)

        # Chapa deslizante
        Primitive3D.draw_box(0.0, 0.0, 0.0, 0.12, 0.006, 0.16, (0.85, 0.88, 0.90))
        # Pestañas laterales de contención
        Primitive3D.draw_box(0.0, 0.02, -0.082, 0.12, 0.035, 0.006, (0.80, 0.82, 0.85))
        Primitive3D.draw_box(0.0, 0.02, 0.082, 0.12, 0.035, 0.006, (0.80, 0.82, 0.85))
        glPopMatrix()

        # Soportes estructurales anclados al chasis frontal
        Primitive3D.draw_box(0.50, 0.30, -0.085, 0.015, 0.14, 0.015, (0.70, 0.72, 0.75))
        Primitive3D.draw_box(0.50, 0.30, 0.085, 0.015, 0.14, 0.015, (0.70, 0.72, 0.75))

    def draw_motorized_container_shuttle(self, cart_z: float):
        """
        Lanzadera / Carro móvil transversal indexador accionado por motor M-102.
        Soporta los 3 contenedores industriales segregados:
        - Ámbar (-0.16 m en carro): Retrabajo
        - Verde ( 0.00 m en carro): Conforme
        - Rojo  (+0.16 m en carro): Rechazo
        """
        # 1. Bancada y Guías Lineales Fijas (Eje Z transversal, X = 0.62)
        # Viga soporte base perfil Rexroth
        Primitive3D.draw_box(0.62, 0.02, 0.0, 0.10, 0.04, 0.94, self.C_ALU)
        # Ranura central
        Primitive3D.draw_box(0.62, 0.041, 0.0, 0.015, 0.004, 0.92, (0.40, 0.42, 0.45))

        # Doble eje guía rectificado
        Primitive3D.draw_cylinder(0.585, 0.055, 0.0, 0.006, 0.92, (0.90, 0.92, 0.94), axis='z')
        Primitive3D.draw_cylinder(0.655, 0.055, 0.0, 0.006, 0.92, (0.90, 0.92, 0.94), axis='z')

        # Soportes de extremos de eje (Pillow blocks)
        Primitive3D.draw_box(0.62, 0.05, -0.46, 0.11, 0.06, 0.03, (0.35, 0.38, 0.42))
        Primitive3D.draw_box(0.62, 0.05, 0.46, 0.11, 0.06, 0.03, (0.35, 0.38, 0.42))

        # Correa dentada síncrona / Husillo de transmisión
        Primitive3D.draw_box(0.62, 0.055, 0.0, 0.012, 0.006, 0.90, (0.15, 0.15, 0.16))

        # 2. Servomotor / Motor Paso a Paso de Indexación M-102 (NEMA 17 / Blue Housing)
        # Cuerpo motor en extremo -Z (-0.50m)
        Primitive3D.draw_box(0.62, 0.065, -0.50, 0.055, 0.055, 0.06, (0.22, 0.24, 0.26))
        # Carcasa azul del encoder / reductor
        Primitive3D.draw_box(0.62, 0.065, -0.54, 0.045, 0.045, 0.025, self.C_MOTOR)
        # Polea de tracción
        Primitive3D.draw_cylinder(0.62, 0.055, -0.465, 0.012, 0.015, (0.85, 0.88, 0.90), axis='y')

        # 3. Carro Móvil Deslizante (Plataforma Portacontenedores a Z = cart_z)
        car_y = 0.08
        # Placa base de aluminio
        Primitive3D.draw_box(0.62, car_y, cart_z, 0.18, 0.016, 0.50, self.C_ALU)

        # 4 Casquillos / Bloques de rodamientos lineales bajo la placa
        for bz in (-0.18, 0.18):
            Primitive3D.draw_box(0.585, 0.055, cart_z + bz, 0.028, 0.024, 0.045, (0.40, 0.43, 0.46))
            Primitive3D.draw_box(0.655, 0.055, cart_z + bz, 0.028, 0.024, 0.045, (0.40, 0.43, 0.46))
        # Tensor central de fijación a correa
        Primitive3D.draw_box(0.62, 0.065, cart_z, 0.035, 0.015, 0.04, (0.85, 0.70, 0.15))

        # 4. Tres Contenedores Industriales Montados sobre el Carro
        bin_dx, bin_dy, bin_dz = 0.15, 0.13, 0.14
        bin_center_y = car_y + 0.008 + bin_dy * 0.5

        # A. CONTENEDOR 1: RETRABAJO (Ámbar, Z = cart_z - 0.16)
        z1 = cart_z - 0.16
        Primitive3D.draw_box(0.62, bin_center_y, z1, bin_dx, bin_dy, bin_dz, (0.92, 0.70, 0.15))
        Primitive3D.draw_box(0.62, bin_center_y + bin_dy*0.5, z1, bin_dx + 0.012, 0.012, bin_dz + 0.012, (0.75, 0.55, 0.10))
        Primitive3D.draw_box(0.62, bin_center_y + 0.015, z1, bin_dx - 0.02, bin_dy, bin_dz - 0.02, (0.22, 0.16, 0.06))
        Primitive3D.draw_box(0.54, bin_center_y, z1, 0.002, 0.04, 0.08, (0.95, 0.95, 0.95))

        # B. CONTENEDOR 2: CONFORME (Verde, Z = cart_z)
        z2 = cart_z
        Primitive3D.draw_box(0.62, bin_center_y, z2, bin_dx, bin_dy, bin_dz, (0.25, 0.65, 0.30))
        Primitive3D.draw_box(0.62, bin_center_y + bin_dy*0.5, z2, bin_dx + 0.012, 0.012, bin_dz + 0.012, (0.18, 0.50, 0.22))
        Primitive3D.draw_box(0.62, bin_center_y + 0.015, z2, bin_dx - 0.02, bin_dy, bin_dz - 0.02, (0.08, 0.22, 0.10))
        Primitive3D.draw_box(0.54, bin_center_y, z2, 0.002, 0.04, 0.08, (0.95, 0.95, 0.95))

        # C. CONTENEDOR 3: RECHAZO (Rojo, Z = cart_z + 0.16)
        z3 = cart_z + 0.16
        Primitive3D.draw_box(0.62, bin_center_y, z3, bin_dx, bin_dy, bin_dz, (0.85, 0.22, 0.20))
        Primitive3D.draw_box(0.62, bin_center_y + bin_dy*0.5, z3, bin_dx + 0.012, 0.012, bin_dz + 0.012, (0.65, 0.16, 0.15))
        Primitive3D.draw_box(0.62, bin_center_y + 0.015, z3, bin_dx - 0.02, bin_dy, bin_dz - 0.02, (0.25, 0.08, 0.08))
        Primitive3D.draw_box(0.54, bin_center_y, z3, 0.002, 0.04, 0.08, (0.95, 0.95, 0.95))

    def draw_diverter_servo(self, servo_angle: float):
        pass # Reemplazado por el carro móvil indexador M-102

    def draw_sorting_chutes(self, counts: Optional[Tuple[int, int, int]] = None):
        pass # Reemplazado por el carro móvil indexador M-102

    def draw_operator_pedestal(self, fsm_state: str, e_stop: bool):
        """Pedestal de mando con baliza luminosa de 3 pisos y pulsador de emergencia."""
        px, py, pz = -0.15, 0.25, -0.30
        
        # Columna del pedestal
        Primitive3D.draw_cylinder(px, 0.20, pz, 0.02, 0.40, self.C_STAND, axis='y')
        # Base de pie
        Primitive3D.draw_cylinder(px, 0.01, pz, 0.08, 0.02, (0.1, 0.1, 0.1), axis='y')
        
        # Caja de mandos inclinada (Y = 0.42)
        glPushMatrix()
        glTranslatef(px, 0.42, pz)
        glRotatef(-20, 1, 0, 0)
        Primitive3D.draw_box(0, 0, 0, 0.12, 0.06, 0.14, (0.85, 0.88, 0.90))
        
        # Seta de Parada de Emergencia (HS-100)
        e_color = (0.9, 0.1, 0.1) if not e_stop else (0.4, 0.05, 0.05)
        e_y = 0.045 if not e_stop else 0.032
        Primitive3D.draw_cylinder(0.0, e_y, -0.02, 0.018, 0.018, e_color, axis='y')
        # Anillo amarillo de advertencia bajo la seta
        Primitive3D.draw_cylinder(0.0, 0.031, -0.02, 0.025, 0.002, (0.95, 0.85, 0.1), axis='y')

        # Pulsador Verde (START) y Pulsador Negro (STOP)
        Primitive3D.draw_cylinder(-0.03, 0.033, 0.03, 0.009, 0.008, (0.2, 0.7, 0.2), axis='y')
        Primitive3D.draw_cylinder(0.03, 0.033, 0.03, 0.009, 0.008, (0.2, 0.2, 0.2), axis='y')
        glPopMatrix()

        # Baliza luminosa industrial de 3 etapas sobre mástil
        # Mástil
        Primitive3D.draw_cylinder(px, 0.58, pz - 0.05, 0.008, 0.26, (0.7, 0.7, 0.7), axis='y')
        
        # Determinar qué luz está activa según el estado del firmware
        c_red = (0.2, 0.0, 0.0)
        c_yel = (0.2, 0.2, 0.0)
        c_grn = (0.0, 0.2, 0.0)

        if e_stop or "EMERG" in fsm_state:
            # Parpadeo rojo rápido
            blink = int(time.time() * 6.0) % 2
            c_red = (1.0, 0.1, 0.1) if blink else (0.3, 0.0, 0.0)
        elif fsm_state in ("TRANSPORTE", "TRANSP", "INSPECCION", "INSPEC", "DESVIO"):
            c_grn = (0.1, 0.95, 0.1) # Verde continuo
        elif fsm_state in ("REPOSO", "IDLE"):
            c_yel = (0.95, 0.85, 0.1) # Ámbar reposo

        # Módulos luminosos cilíndricos
        by = 0.72
        Primitive3D.draw_cylinder(px, by + 0.050, pz - 0.05, 0.018, 0.022, c_red, axis='y')
        Primitive3D.draw_cylinder(px, by + 0.025, pz - 0.05, 0.018, 0.022, c_yel, axis='y')
        Primitive3D.draw_cylinder(px, by + 0.000, pz - 0.05, 0.018, 0.022, c_grn, axis='y')
        # Tapa negra superior
        Primitive3D.draw_cylinder(px, by + 0.065, pz - 0.05, 0.020, 0.008, (0.1, 0.1, 0.1), axis='y')


# =====================================================================================
# SISTEMA DE INTERFAZ DE USUARIO 2D (HUD OVERLAY)
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
        """Define los botones mecánicos del tablero de comando retro."""
        console_y = self.screen_h - self.console_h
        margin = 10
        w1 = 430
        w2 = 320
        x3 = margin + w1 + 10 + w2 + 10
        w3 = max(390, self.screen_w - x3 - margin)

        desk_x = x3 + 8
        desk_w = w3 - 16

        # Fila 1: Botones principales
        btn_y1 = console_y + 36
        btn_h1 = 34
        b1_w = (desk_w - 16) // 3
        b1_w_estop = desk_w - 16 - (b1_w * 2)

        # Fila 2: Botones secundarios
        btn_y2 = console_y + 76
        btn_h2 = 32
        b2_w = (desk_w - 16) // 3
        b2_w_last = desk_w - 16 - (b2_w * 2)

        # Fila 3: Selectores de cámaras
        btn_y3 = console_y + 116
        btn_h3 = 28
        num_cams = 5
        cam_spacing = 6
        cam_w = (desk_w - cam_spacing * (num_cams - 1)) // num_cams

        self.buttons = [
            {"id": "START",   "rect": pygame.Rect(desk_x, btn_y1, b1_w, btn_h1), "label": "INICIAR", "color": (25, 135, 55)},
            {"id": "STOP",    "rect": pygame.Rect(desk_x + b1_w + 8, btn_y1, b1_w, btn_h1), "label": "DETENER", "color": (75, 85, 100)},
            {"id": "ESTOP",   "rect": pygame.Rect(desk_x + (b1_w + 8) * 2, btn_y1, b1_w_estop, btn_h1), "label": "! E-STOP !", "color": (215, 35, 35)},
            {"id": "ADD_BOX", "rect": pygame.Rect(desk_x, btn_y2, b2_w, btn_h2), "label": "+ CAJA", "color": (30, 110, 205)},
            {"id": "RESET",   "rect": pygame.Rect(desk_x + b2_w + 8, btn_y2, b2_w, btn_h2), "label": "RESET ALM", "color": (200, 110, 25)},
            {"id": "LIGHT",   "rect": pygame.Rect(desk_x + (b2_w + 8) * 2, btn_y2, b2_w_last, btn_h2), "label": "LUZ AMB", "color": (180, 145, 30)},
        ]
        cam_labels = ["1:ISO", "2:CAM", "3:LAT", "4:CARRO", "5:TOLVA"]
        for i in range(num_cams):
            cx = desk_x + i * (cam_w + cam_spacing)
            self.buttons.append({
                "id": f"V{i+1}",
                "rect": pygame.Rect(cx, btn_y3, cam_w, btn_h3),
                "label": cam_labels[i],
                "color": (55, 65, 80)
            })

    def render(self, state: PlantState, fps: float, counts: Tuple[int, int, int, int]):
        """Renderizado completo de la consola retro industrial en la parte inferior."""
        glViewport(0, 0, self.screen_w, self.screen_h)

        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        gluOrtho2D(0, self.screen_w, self.screen_h, 0)
        
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        console_y = self.screen_h - self.console_h
        margin = 10
        w1 = 430
        w2 = 320
        x3 = margin + w1 + 10 + w2 + 10
        w3 = max(390, self.screen_w - x3 - margin)
        y_panels = console_y + 28
        h_panels = self.console_h - 36

        # 1. Chasis exterior oscuro de la consola
        self._draw_rect(0, console_y, self.screen_w, self.console_h, (28, 33, 40), 1.0)
        # Embellecedor superior
        self._draw_bevel_box(0, console_y, self.screen_w, 24, (50, 56, 66), False)
        self._draw_text("[ CONSOLA DE SUPERVISION INDUSTRIAL // AUTO-TIF v2.4 ]", 16, console_y + 4, (225, 230, 240), self.font_title)
        self._draw_text(f"ATmega328P • FPS: {fps:4.1f}", self.screen_w - 220, console_y + 4, (150, 190, 220), self.font_mono)

        # 2. MÓDULO A: CRT-1 (Terminal de Fósforo Verde - Lazos & FSM)
        self._draw_bevel_box(margin, y_panels, w1, h_panels, (5, 12, 7), True)
        self._draw_rect_outline(margin, y_panels, w1, h_panels, (38, 115, 55), 1.0)
        self._draw_scanlines(margin, y_panels, w1, h_panels)

        self._draw_text(">> CRT-1: TELEMETRIA FSM & CONTROL INDUSTRIAL <<", margin + 12, y_panels + 6, (80, 255, 140), self.font_mono)

        fsm_col = (70, 255, 130)
        display_state = state.fsm_state
        if state.emergency_stop or "EMERG" in state.fsm_state:
            fsm_col = (255, 60, 60)
            display_state = "EMERGENCIA!"
        elif state.fsm_state in ("TRANSPORTE", "TRANSP"):
            fsm_col = (70, 240, 130)
            display_state = "TRANSPORTE"
        elif state.fsm_state in ("INSPECCION", "INSPEC"):
            fsm_col = (60, 230, 255)
            display_state = "INSPECCION"
        elif state.fsm_state == "DESVIO":
            fsm_col = (255, 220, 50)
            display_state = "DESVIO ACTIVO"

        self._draw_text(f"ESTADO FSM   : [ {display_state:<11} ]", margin + 12, y_panels + 28, fsm_col, self.font_mono)
        self._draw_text(f"SONAR ZT-101: {state.ultrasonic_dist_cm:4.1f} cm", margin + 250, y_panels + 28, (120, 245, 255), self.font_mono)

        # Barra de proximidad analógica
        filled = max(0, min(12, int((state.ultrasonic_dist_cm / 35.0) * 12.0)))
        bar_str = "=" * filled + "." * (12 - filled)
        self._draw_text(f"VELOCIDAD FAJA: {state.belt_speed_mps:4.2f} m/s | PWM {state.motor_pwm:3d} [{bar_str}]", margin + 12, y_panels + 48, (80, 245, 130), self.font_mono)

        # Lazo 2: Carro Móvil
        dest_str = "CENTRO [OK]"
        if getattr(state, 'target_cart_z', 0.0) > 0.05:
            dest_str = "IZQ [RETR]"
        elif getattr(state, 'target_cart_z', 0.0) < -0.05:
            dest_str = "DER [RECH]"
        cart_mm = getattr(state, 'cart_z', 0.0) * 1000.0
        self._draw_text(f"CARRO GAVETAS : {cart_mm:+4.0f} mm  -> DESTINO: {dest_str}", margin + 12, y_panels + 68, (80, 245, 130), self.font_mono)

        # Lazo 3: Luz LDR y LED
        self._draw_text(f"LUZ CAMARA PID: {state.total_lux_pct:4.1f}% (SP: {state.sp_lux_pct:2.0f}%) | LED: {state.led_pwm:3d}/255", margin + 12, y_panels + 88, (80, 245, 130), self.font_mono)

        # Prompt
        self._draw_text(f"ATmega328P::OK [20ms] • LUZ AMB: {state.ambient_lux_pct:2.0f}% > EN LINEA", margin + 12, y_panels + 108, (70, 215, 110), self.font_mono)

        # 3. MÓDULO B: CRT-2 (OEE & Matriz de Producción)
        x2 = margin + w1 + 10
        self._draw_bevel_box(x2, y_panels, w2, h_panels, (8, 12, 18), True)
        self._draw_rect_outline(x2, y_panels, w2, h_panels, (50, 110, 160), 1.0)
        self._draw_scanlines(x2, y_panels, w2, h_panels)

        self._draw_text(">> CRT-2: CLASIFICACION & OEE <<", x2 + 12, y_panels + 6, (120, 225, 255), self.font_mono)

        cell_w = (w2 - 30) // 2
        cell_h = 34
        tot, conf, retr, rech = counts

        # Celda 1: TOTAL
        self._draw_bevel_box(x2 + 10, y_panels + 24, cell_w, cell_h, (25, 32, 45), True)
        self._draw_text("TOTAL", x2 + 16, y_panels + 27, (170, 185, 205), self.font_mono_small)
        self._draw_text(f"{tot:04d}", x2 + 75, y_panels + 31, (240, 248, 255), self.font_digits)

        # Celda 2: CONFORME
        self._draw_bevel_box(x2 + 10 + cell_w + 10, y_panels + 24, cell_w, cell_h, (12, 38, 20), True)
        self._draw_text("CONFORME", x2 + 10 + cell_w + 14, y_panels + 27, (110, 225, 130), self.font_mono_small)
        self._draw_text(f"{conf:04d}", x2 + 10 + cell_w + 75, y_panels + 31, (80, 255, 100), self.font_digits)

        # Celda 3: RETRABAJO
        self._draw_bevel_box(x2 + 10, y_panels + 62, cell_w, cell_h, (40, 32, 10), True)
        self._draw_text("RETRABAJO", x2 + 16, y_panels + 65, (235, 185, 50), self.font_mono_small)
        self._draw_text(f"{retr:04d}", x2 + 75, y_panels + 69, (255, 210, 40), self.font_digits)

        # Celda 4: RECHAZO
        self._draw_bevel_box(x2 + 10 + cell_w + 10, y_panels + 62, cell_w, cell_h, (45, 15, 15), True)
        self._draw_text("RECHAZO", x2 + 10 + cell_w + 14, y_panels + 65, (245, 100, 90), self.font_mono_small)
        self._draw_text(f"{rech:04d}", x2 + 10 + cell_w + 75, y_panels + 69, (255, 70, 70), self.font_digits)

        # Eficiencia OEE
        oee = (conf / tot * 100.0) if tot > 0 else 100.0
        self._draw_text(f"EFICIENCIA OEE: {oee:5.1f}% [CONF/TOTAL]", x2 + 12, y_panels + 102, (225, 238, 250), self.font_mono)

        # Hardware LEDs
        self._draw_led(x2 + 25, y_panels + 124, 4.5, (30, 255, 60), True)
        self._draw_text("POWER", x2 + 34, y_panels + 118, (180, 205, 230), self.font_mono_small)

        self._draw_led(x2 + 95, y_panels + 124, 4.5, (50, 160, 255), state.belt_speed_mps > 0.01)
        self._draw_text("FAJA ON", x2 + 104, y_panels + 118, (180, 205, 230), self.font_mono_small)

        self._draw_led(x2 + 175, y_panels + 124, 4.5, (255, 210, 30), state.fsm_state in ("INSPECCION", "INSPEC"))
        self._draw_text("VISION", x2 + 184, y_panels + 118, (180, 205, 230), self.font_mono_small)

        self._draw_led(x2 + 245, y_panels + 124, 4.5, (255, 50, 50), state.emergency_stop or "EMERG" in state.fsm_state)
        self._draw_text("ALARMA", x2 + 254, y_panels + 118, (180, 205, 230), self.font_mono_small)

        # 4. MÓDULO C: TABLERO DE MANDO ANALÓGICO RETRO
        self._draw_bevel_box(x3, y_panels, w3, h_panels, (36, 42, 52), False)
        self._draw_rect_outline(x3, y_panels, w3, h_panels, (80, 92, 110), 1.0)
        self._draw_text(">> TABLERO DE ACCIONAMIENTO RAPIDO <<", x3 + 12, y_panels + 6, (230, 235, 245), self.font_mono)

        mouse_pos = pygame.mouse.get_pos()
        for btn in self.buttons:
            r = btn["rect"]
            hover = r.collidepoint(mouse_pos)
            col = btn["color"]
            if hover:
                col = (min(255, col[0] + 30), min(255, col[1] + 30), min(255, col[2] + 30))

            if btn["id"] == "ESTOP":
                self._draw_rect_outline(r.x - 2, r.y - 2, r.width + 4, r.height + 4, (242, 200, 38), 2.0)
                self._draw_bevel_box(r.x, r.y, r.width, r.height, col, False)
                self._draw_text_centered(btn["label"], r.centerx, r.centery, (255, 255, 255), self.font_bold)
            else:
                self._draw_bevel_box(r.x, r.y, r.width, r.height, col, False)
                self._draw_text_centered(btn["label"], r.centerx, r.centery, (255, 255, 255), self.font_bold)

        # Restaurar estado 3D
        glDisable(GL_BLEND)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()

    def _draw_rect(self, x, y, w, h, col, alpha=1.0):
        glColor4f(col[0]/255.0, col[1]/255.0, col[2]/255.0, alpha)
        glBegin(GL_QUADS)
        glVertex2f(x, y)
        glVertex2f(x + w, y)
        glVertex2f(x + w, y + h)
        glVertex2f(x, y + h)
        glEnd()

    def _draw_rect_outline(self, x, y, w, h, col, width=1.0):
        glColor3f(col[0]/255.0, col[1]/255.0, col[2]/255.0)
        glLineWidth(width)
        glBegin(GL_LINE_LOOP)
        glVertex2f(x, y)
        glVertex2f(x + w, y)
        glVertex2f(x + w, y + h)
        glVertex2f(x, y + h)
        glEnd()

    def _draw_bevel_box(self, x, y, w, h, col, inset=False):
        self._draw_rect(x, y, w, h, col, 1.0)
        lr = min(255, col[0] + 65)
        lg = min(255, col[1] + 65)
        lb = min(255, col[2] + 65)
        dr = max(0, col[0] - 55)
        dg = max(0, col[1] - 55)
        db = max(0, col[2] - 55)
        top_col = (dr, dg, db) if inset else (lr, lg, lb)
        bot_col = (lr, lg, lb) if inset else (dr, dg, db)

        glDisable(GL_LIGHTING)
        glLineWidth(2.0)
        glBegin(GL_LINES)
        glColor3f(top_col[0]/255.0, top_col[1]/255.0, top_col[2]/255.0)
        glVertex2f(x, y + 1); glVertex2f(x + w, y + 1)
        glVertex2f(x + 1, y); glVertex2f(x + 1, y + h)
        glColor3f(bot_col[0]/255.0, bot_col[1]/255.0, bot_col[2]/255.0)
        glVertex2f(x, y + h - 1); glVertex2f(x + w, y + h - 1)
        glVertex2f(x + w - 1, y); glVertex2f(x + w - 1, y + h)
        glEnd()

    def _draw_scanlines(self, x, y, w, h):
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glColor4f(0.0, 0.0, 0.0, 0.18)
        glLineWidth(1.0)
        glBegin(GL_LINES)
        sy = y + 2.0
        while sy < y + h:
            glVertex2f(x + 1, sy)
            glVertex2f(x + w - 1, sy)
            sy += 3.0
        glEnd()

    def _draw_led(self, cx, cy, radius, col, active=True):
        glDisable(GL_LIGHTING)
        glBegin(GL_TRIANGLE_FAN)
        if active:
            glColor3f(col[0]/255.0, col[1]/255.0, col[2]/255.0)
        else:
            glColor3f(col[0]*0.22/255.0, col[1]*0.22/255.0, col[2]*0.22/255.0)
        glVertex2f(cx, cy)
        for i in range(13):
            a = 2.0 * math.pi * i / 12.0
            glVertex2f(cx + math.cos(a) * radius, cy + math.sin(a) * radius)
        glEnd()

        # Aro metálico
        glColor3f(0.35, 0.40, 0.45)
        glLineWidth(1.0)
        glBegin(GL_LINE_LOOP)
        for i in range(12):
            a = 2.0 * math.pi * i / 12.0
            glVertex2f(cx + math.cos(a) * (radius + 1.0), cy + math.sin(a) * (radius + 1.0))
        glEnd()

    def _draw_text(self, text: str, x: int, y: int, color: Tuple[int, int, int], font: pygame.font.Font):
        key = (text, color, id(font))
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
        
        # Configuración OpenGL y Anti-aliasing
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

        # Iluminación OpenGL
        self._setup_opengl()

    def _setup_opengl(self):
        """Inicializa parámetros de renderizado e iluminación técnica."""
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        # Fondo blanco técnico / gris estudio
        glClearColor(0.93, 0.94, 0.96, 1.0)
        
        # Luz ambiental global y direccional (Sol de taller)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_POSITION, [1.5, 3.5, 2.0, 0.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.85, 0.85, 0.82, 1.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.45, 0.48, 0.52, 1.0])
        
        # Luz secundaria de relleno
        glEnable(GL_LIGHT1)
        glLightfv(GL_LIGHT1, GL_POSITION, [-2.0, 2.0, -1.5, 0.0])
        glLightfv(GL_LIGHT1, GL_DIFFUSE, [0.25, 0.28, 0.32, 1.0])

        # Luz puntual de inspección (GL_LIGHT2) configurada dinámicamente
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
        """Genera una nueva pieza en el alimentador de entrada."""
        if quality is None:
            # Distribución típica de calidad industrial
            r = random.random()
            if r < 0.65:
                quality = "CONFORME"
            elif r < 0.85:
                quality = "RETRABAJO"
            else:
                quality = "RECHAZO"

        new_box = DynamicBox(quality)
        self.boxes.append(new_box)
        
        # Notificar también al emulador interno
        self.physics.reset_piece(quality)
        if self.firmware.fsm_state in ("REPOSO", "IDLE"):
            self.firmware.fsm_state = "TRANSP"

    def run(self):
        """Bucle interactivo principal."""
        running = True
        auto_spawn = True

        while running:
            dt = self.clock.tick(60) / 1000.0  # Límite a 60 FPS
            dt = min(0.05, dt)  # Evitar grandes saltos de integración temporal

            # 1. Procesamiento de Eventos de Usuario
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
                    # Verificar si hizo clic en algún botón del HUD
                    clicked_hud = False
                    for btn in self.hud.buttons:
                        if btn["rect"].collidepoint(event.pos):
                            clicked_hud = True
                            b_id = btn["id"]
                            if b_id == "START":
                                self.firmware.fsm_state = "TRANSP"
                                self.physics.state.emergency_stop = False
                            elif b_id == "STOP":
                                self.firmware.fsm_state = "REPOSO"
                            elif b_id == "ADD_BOX":
                                self.spawn_box()
                            elif b_id == "ESTOP":
                                self.physics.trigger_emergency(True)
                                self.firmware.execute_cycle(dt, 50.0, 50.0, True)
                            elif b_id == "RESET":
                                self.physics.trigger_emergency(False)
                                self.firmware.fsm_state = "REPOSO"
                            elif b_id == "LIGHT":
                                # Alternar perturbación de luz ambiental
                                if self.physics.state.ambient_lux_pct > 25.0:
                                    self.physics.inject_disturbance(15.0)
                                else:
                                    self.physics.inject_disturbance(65.0)
                            elif b_id in ("V1", "V2", "V3", "V4", "V5"):
                                self.camera.set_preset(int(b_id[1]))
                            break

                    if not clicked_hud:
                        if event.pos[1] < self.height - 195:
                            self.camera.handle_event(event)
                else:
                    self.camera.handle_event(event)

            # 2. Lógica y Cinemática de Cajas
            # Generación automática de cajas espaciadas desde la tolva de alimentación
            if auto_spawn and self.firmware.fsm_state not in ("REPOSO", "EMERG!", "IDLE"):
                self.box_spawn_timer -= dt
                if self.box_spawn_timer <= 0:
                    # Solo generar si no hay una caja ocupando la boca de entrada
                    if not any(b.pos_x < -0.30 for b in self.boxes if b.state in ("EN_FAJA", "CAYENDO_DE_TOLVA")):
                        self.spawn_box()
                    self.box_spawn_timer = random.uniform(3.5, 5.0)

            # Determinar distancia al obstáculo más cercano al sensor ultrasónico (X = 0.35)
            sensor_x = 0.35
            closest_dist_cm = 50.0
            active_box_for_cam = None

            for box in self.boxes:
                if box.state == "EN_FAJA":
                    # Distancia desde el sensor hacia la caja
                    d_cm = (sensor_x - box.pos_x) * 100.0
                    if 0.0 < d_cm < closest_dist_cm:
                        closest_dist_cm = d_cm
                    # Detección de caja en la zona de cámara (X entre 0.20 y 0.30)
                    if 0.18 <= box.pos_x <= 0.30:
                        active_box_for_cam = box

            # 3. Ejecución del Ciclo de Control (Firmware Virtual ATmega328P)
            e_stop_flag = self.physics.state.emergency_stop
            motor_pwm, led_pwm, servo_ang = self.firmware.execute_cycle(
                dt, closest_dist_cm, self.physics.state.total_lux_pct, e_stop_flag
            )

            # Si una caja está bajo la cámara y se decide la clasificación
            if active_box_for_cam and self.firmware.fsm_state == "DESVIO":
                # Configurar ángulo del servo y objetivo del carro móvil según calidad de la caja actual
                if active_box_for_cam.quality == "RETRABAJO":
                    self.firmware.servo_angle = 45
                    self.firmware.target_cart_z = 0.16
                elif active_box_for_cam.quality == "RECHAZO":
                    self.firmware.servo_angle = 135
                    self.firmware.target_cart_z = -0.16
                else:
                    self.firmware.servo_angle = 90
                    self.firmware.target_cart_z = 0.0

            # 4. Integración de la Física Continua
            self.physics.step(motor_pwm, led_pwm, self.firmware.servo_angle, self.firmware.target_cart_z)
            self.physics.state.fsm_state = self.firmware.fsm_state

            # Actualizar movimiento 3D de cada caja
            for box in list(self.boxes):
                box.update(dt, self.physics.state.belt_speed_mps, self.physics.state.cart_z)

                # Contabilizar y evacuar tras reposar visiblemente dentro de su gaveta
                if box.state == "FINALIZADA" and box.settle_timer >= 2.5:
                    self.count_total += 1
                    if box.target_chute == 1:
                        self.count_retrabajo += 1
                    elif box.target_chute == 2:
                        self.count_conforme += 1
                    elif box.target_chute == 3:
                        self.count_rechazo += 1
                    self.boxes.remove(box)

            # Actualizar cámara
            self.camera.update(dt)

            # 5. Renderizado 3D
            self._set_projection()
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glLoadIdentity()
            self.camera.apply()

            # Dibujar elementos del modelo opacos
            glDepthMask(GL_TRUE)
            self.model3d.draw_floor()
            self.model3d.draw_conveyor_frame(self.physics.state.belt_speed_mps, dt)
            self.model3d.draw_feed_hopper()
            self.model3d.draw_discharge_lip()
            self.model3d.draw_motorized_container_shuttle(self.physics.state.cart_z)
            self.model3d.draw_operator_pedestal(self.physics.state.fsm_state, self.physics.state.emergency_stop)

            # Dibujar cajas 3D sólidas (siempre visibles)
            for box in self.boxes:
                box.draw()

            # Dibujar sensores y conos volumétricos (con Z-buffer protegido para nunca tapar las cajas)
            self.model3d.draw_ultrasonic_sensor(self.physics.state.ultrasonic_dist_cm)
            self.model3d.draw_inspection_camera(self.physics.state.led_pwm)

            # 6. Renderizado de Interfaz 2D (HUD)
            counts_tuple = (self.count_total, self.count_conforme, self.count_retrabajo, self.count_rechazo)
            self.hud.render(self.physics.state, self.clock.get_fps(), counts_tuple)

            pygame.display.flip()

        pygame.quit()


if __name__ == "__main__":
    app = DigitalTwin3DApp()
    app.run()
