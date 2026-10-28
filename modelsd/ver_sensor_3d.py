"""
=====================================================================================
VISOR 3D INTERACTIVO - SENSOR ULTRASONICO HC-SR04 (CAD VIEWER) - ALTO RENDIMIENTO
=====================================================================================
Renderizado acelerado por GPU (OpenGL Display Lists) a 60+ FPS sin lag.
Controles:
  - Clic Izquierdo + Arrastrar: Rotar camara orbital
  - Clic Derecho + Arrastrar:   Trasladar (pan)
  - Rueda del raton:            Zoom In / Out
  - Tecla 'C':                  Alternar modo Color Realista / Modo CAD Clay (GrabCAD)
  - Tecla 'W':                  Alternar modo solido / modo alambrico (wireframe)
  - Tecla 'R':                  Resetear camara a la vista inicial
  - Tecla '1':                  Vista Frontal
  - Tecla '2':                  Vista Isometria GrabCAD (identica a la referencia)
  - Tecla '3':                  Vista Superior
  - Tecla '4':                  Vista Lateral Derecha
  - Tecla 'S':                  Guardar captura PNG en alta resolucion
=====================================================================================
"""

import sys
import os
import math
import pygame
from pygame.locals import *

try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
except ImportError:
    print("[ERROR] PyOpenGL no esta instalado.")
    sys.exit(1)

class ObjLoader:
    def __init__(self, filename):
        self.vertices = []
        self.normals = []
        self.materials = {}
        self.faces = []

        mtl_file = None
        dir_path = os.path.dirname(filename)

        with open(filename, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
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
        with open(mtl_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
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
        """Compila la geometria directamente en la memoria VRAM de la GPU para 60+ FPS."""
        # 1. Display List: MODO CLAY (Monocromatico CAD GrabCAD)
        self.d_list_clay = glGenLists(1)
        glNewList(self.d_list_clay, GL_COMPILE)
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, [0.65, 0.68, 0.72, 1.0])
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, [0.35, 0.36, 0.38, 1.0])
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [0.75, 0.75, 0.80, 1.0])
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 60.0)
        glColor3f(0.68, 0.70, 0.74)

        glBegin(GL_TRIANGLES)
        for _, face in self.faces:
            if len(face) == 3:
                for v_i, vn_i in face:
                    if vn_i is not None: glNormal3fv(self.normals[vn_i])
                    glVertex3fv(self.vertices[v_i])
        glEnd()

        glBegin(GL_QUADS)
        for _, face in self.faces:
            if len(face) == 4:
                for v_i, vn_i in face:
                    if vn_i is not None: glNormal3fv(self.normals[vn_i])
                    glVertex3fv(self.vertices[v_i])
        glEnd()
        glEndList()

        # 2. Display List: MODO COLOR (Fotorrealista)
        self.d_list_color = glGenLists(1)
        glNewList(self.d_list_color, GL_COMPILE)

        # Agrupar caras por material para minimizar cambios de estado en GPU
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
                        if vn_i is not None: glNormal3fv(self.normals[vn_i])
                        glVertex3fv(self.vertices[v_i])
                glEnd()

            quads = [f for f in faces if len(f) == 4]
            if quads:
                glBegin(GL_QUADS)
                for f in quads:
                    for v_i, vn_i in f:
                        if vn_i is not None: glNormal3fv(self.normals[vn_i])
                        glVertex3fv(self.vertices[v_i])
                glEnd()

        glEndList()

    def render_mesh(self, mode="COLOR"):
        """Llamada ultrarrapida compilada a nivel de hardware."""
        if mode == "CLAY":
            glCallList(self.d_list_clay)
        else:
            glCallList(self.d_list_color)

def main():
    width, height = 1100, 720
    pygame.init()
    screen = pygame.display.set_mode((width, height), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("Visor 3D CAD: Sensor Ultrasonico HC-SR04 [Alto Rendimiento 60+ FPS]")

    # Configuracion de OpenGL
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glShadeModel(GL_SMOOTH)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_LIGHT1)
    glEnable(GL_NORMALIZE)

    glClearColor(0.12, 0.13, 0.15, 1.0) # Fondo oscuro estudio GrabCAD

    # Luz clave superior frontal
    glLightfv(GL_LIGHT0, GL_POSITION, [40.0, 50.0, 60.0, 1.0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.95, 0.95, 0.95, 1.0])
    glLightfv(GL_LIGHT0, GL_SPECULAR, [0.85, 0.85, 0.85, 1.0])
    glLightfv(GL_LIGHT0, GL_AMBIENT, [0.35, 0.35, 0.38, 1.0])

    # Luz de relleno lateral suave
    glLightfv(GL_LIGHT1, GL_POSITION, [-40.0, -30.0, 30.0, 1.0])
    glLightfv(GL_LIGHT1, GL_DIFFUSE, [0.35, 0.38, 0.42, 1.0])
    glLightfv(GL_LIGHT1, GL_SPECULAR, [0.20, 0.20, 0.20, 1.0])
    glLightfv(GL_LIGHT1, GL_AMBIENT, [0.15, 0.15, 0.18, 1.0])

    # Cargar y compilar en GPU
    obj_path = os.path.join(os.path.dirname(__file__), "HC-SR04-UltrasonicSensor.obj")
    print(f"Cargando modelo OBJ: {obj_path}...")
    loader = ObjLoader(obj_path)
    loader.compile_display_lists()
    print("Modelo compilado en GPU VRAM satisfactoriamente.")

    # Parametros de Camara Orbital
    target = [0.0, -1.0, 2.0]
    distance = 70.0
    yaw = 22.0
    pitch = 16.0
    wireframe = False
    render_mode = "CLAY" # Tecla 'C' para alternar a COLOR

    def update_projection():
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(38.0, width / float(height), 1.0, 500.0)
        glMatrixMode(GL_MODELVIEW)

    update_projection()

    clock = pygame.time.Clock()
    is_rotating = False
    is_panning = False
    last_mouse = (0, 0)

    running = True
    while running:
        clock.tick(60)

        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
                elif event.key == K_c:
                    render_mode = "COLOR" if render_mode == "CLAY" else "CLAY"
                    print(f"Modo de renderizado: {render_mode}")
                elif event.key == K_w:
                    wireframe = not wireframe
                elif event.key == K_r: # Reset
                    target = [0.0, -1.0, 2.0]
                    distance = 70.0
                    yaw = 22.0
                    pitch = 16.0
                elif event.key == K_1: # Vista Frontal
                    yaw, pitch = 0.0, 0.0
                elif event.key == K_2: # Vista Isometria GrabCAD
                    yaw, pitch = 22.0, 16.0
                elif event.key == K_3: # Vista Superior
                    yaw, pitch = 0.0, 89.0
                elif event.key == K_4: # Vista Lateral Derecha
                    yaw, pitch = 90.0, 0.0
                elif event.key == K_s: # Screenshot
                    snap_path = os.path.join(os.path.dirname(__file__), f"captura_hcsr04_{render_mode.lower()}.png")
                    glReadBuffer(GL_BACK)
                    data = glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE)
                    surf = pygame.image.frombytes(data, (width, height), "RGB", True)
                    pygame.image.save(surf, snap_path)
                    print(f"Captura guardada en: {snap_path}")

            elif event.type == MOUSEBUTTONDOWN:
                if event.button == 1:
                    is_rotating = True
                    last_mouse = event.pos
                elif event.button in (2, 3):
                    is_panning = True
                    last_mouse = event.pos
                elif event.button == 4: # Scroll Up
                    distance = max(10.0, distance - 4.0)
                elif event.button == 5: # Scroll Down
                    distance = min(200.0, distance + 4.0)

            elif event.type == MOUSEBUTTONUP:
                if event.button == 1:
                    is_rotating = False
                elif event.button in (2, 3):
                    is_panning = False

            elif event.type == MOUSEMOTION:
                dx = event.pos[0] - last_mouse[0]
                dy = event.pos[1] - last_mouse[1]
                last_mouse = event.pos

                if is_rotating:
                    yaw += dx * 0.4
                    pitch = max(-89.0, min(89.0, pitch + dy * 0.4))
                elif is_panning:
                    pan_speed = 0.04 * (distance / 65.0)
                    rad_yaw = math.radians(yaw)
                    target[0] += (-math.sin(rad_yaw) * dx + math.cos(rad_yaw) * dy) * pan_speed
                    target[1] -= dy * pan_speed

        # Renderizado
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        rad_yaw = math.radians(yaw)
        rad_pitch = math.radians(pitch)
        eye_x = target[0] + distance * math.cos(rad_pitch) * math.sin(rad_yaw)
        eye_y = target[1] + distance * math.sin(rad_pitch)
        eye_z = target[2] + distance * math.cos(rad_pitch) * math.cos(rad_yaw)

        gluLookAt(eye_x, eye_y, eye_z, target[0], target[1], target[2], 0, 1, 0)

        if wireframe:
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
            glDisable(GL_LIGHTING)
            glColor3f(0.3, 0.85, 1.0)
        else:
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
            glEnable(GL_LIGHTING)

        loader.render_mesh(mode=render_mode)

        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
