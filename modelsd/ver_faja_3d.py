"""
=====================================================================================
VISOR 3D INTERACTIVO: FAJA TRANSPORTADORA Y CATALOGO DE PIEZAS DESPIEZADAS
Modelo CAD modular basado en GrabCAD (Gabriele Gregorini) con sensor HC-SR04.
Acelerado por Hardware GPU OpenGL mediante Display Lists (60+ FPS).
=====================================================================================
CONTROLES:
  - Clic Izquierdo + Arrastrar:  Rotación Orbital 360 grados
  - Clic Derecho  + Arrastrar:  Desplazamiento / Paneo (Pan)
  - Rueda del Ratón:            Zoom In / Zoom Out
  - Teclas 0 a 9 / TAB:         Selector de Piezas Individuales y Ensamble Completo:
      [0]: Ensamble Completo Acoplado
      [1]: Perfil de Aluminio T-Slot 20x20 mm
      [2]: Rodillo con Eje de Acero y Rodamientos 608ZZ
      [3]: Pata de Soporte Lateral Estándar
      [4]: Pata Lateral con Brazo de Extensión para Sensor
      [5]: Torre Roja Regulable con Corredera
      [6]: Placa de Montaje del Sensor (GrabCAD)
      [7]: Sensor Ultrasónico HC-SR04
      [8]: Banda Transportadora Flexible
      [9]: Motor NEMA 17 con Acople y Escuadra
  - Tecla 'C':                  Alternar Color Realista PBR / Monocromático Clay CAD
  - Tecla 'W':                  Alternar Modo Alámbrico (Wireframe)
  - Tecla 'R':                  Resetear Cámara
  - Tecla 'S':                  Guardar Captura de Pantalla HD en PNG
  - ESC:                        Salir
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
    print("[ERROR] PyOpenGL no esta instalado en este entorno.")
    sys.exit(1)

from ver_sensor_3d import ObjLoader

PARTS_INFO = [
    {"key": K_0, "id": "0", "file": "faja_transportadora_ensamble.obj", "name": "ENSAMBLE COMPLETO ACOPLADO", "dist": 620.0, "target": [0.0, 10.0, 0.0]},
    {"key": K_1, "id": "1", "file": "perfil_aluminio_2020.obj",         "name": "PERFIL ALUMINIO T-SLOT 20x20", "dist": 450.0, "target": [0.0, 0.0, 0.0]},
    {"key": K_2, "id": "2", "file": "rodillo_conveyor.obj",             "name": "RODILLO C/ EJE Y RODAMIENTOS", "dist": 180.0, "target": [0.0, 0.0, 0.0]},
    {"key": K_3, "id": "3", "file": "pata_soporte_lateral.obj",         "name": "PATA SOPORTE ESTANDAR",        "dist": 150.0, "target": [0.0, -15.0, 0.0]},
    {"key": K_4, "id": "4", "file": "pata_soporte_extendida.obj",       "name": "PATA EXTENDIDA P/ SENSOR",     "dist": 180.0, "target": [-30.0, -15.0, 0.0]},
    {"key": K_5, "id": "5", "file": "torre_roja_ajustable.obj",         "name": "TORRE ROJA REGULABLE",         "dist": 160.0, "target": [0.0, 30.0, 0.0]},
    {"key": K_6, "id": "6", "file": "placa_soporte_sensor.obj",         "name": "PLACA MONTAJE SENSOR GRABCAD", "dist": 180.0, "target": [0.0, 0.0, 0.0]},
    {"key": K_7, "id": "7", "file": "sensor_hcsr04_compacto.obj",       "name": "SENSOR ULTRASONICO HC-SR04",   "dist": 90.0,  "target": [0.0, 0.0, 0.0]},
    {"key": K_8, "id": "8", "file": "banda_transportadora.obj",         "name": "BANDA TRANSPORTADORA PLANA",   "dist": 450.0, "target": [0.0, 0.0, 0.0]},
    {"key": K_9, "id": "9", "file": "motor_nema17_drive.obj",           "name": "MOTOR NEMA 17 C/ ACOPLE",      "dist": 150.0, "target": [0.0, -10.0, 0.0]},
]

def run_viewer():
    pygame.init()
    width, height = 1280, 800
    pygame.display.set_caption("Faja Transportadora 3D - Catálogo de Piezas y Ensamble Acoplado (GrabCAD)")

    pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLEBUFFERS, 1)
    pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLESAMPLES, 4)
    pygame.display.gl_set_attribute(pygame.GL_DEPTH_SIZE, 24)

    screen = pygame.display.set_mode(
        (width, height),
        pygame.DOUBLEBUF | pygame.OPENGL | pygame.RESIZABLE
    )

    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_LIGHT1)
    glEnable(GL_NORMALIZE)
    glShadeModel(GL_SMOOTH)
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

    # Iluminación de estudio industrial (3 puntos de luz)
    glLightfv(GL_LIGHT0, GL_POSITION, [200.0, 400.0, 300.0, 1.0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.95, 0.95, 0.98, 1.0])
    glLightfv(GL_LIGHT0, GL_SPECULAR, [0.6, 0.6, 0.6, 1.0])
    glLightfv(GL_LIGHT0, GL_AMBIENT, [0.35, 0.36, 0.40, 1.0])

    glLightfv(GL_LIGHT1, GL_POSITION, [-300.0, 200.0, -200.0, 1.0])
    glLightfv(GL_LIGHT1, GL_DIFFUSE, [0.55, 0.58, 0.65, 1.0])
    glLightfv(GL_LIGHT1, GL_SPECULAR, [0.3, 0.3, 0.3, 1.0])
    glLightfv(GL_LIGHT1, GL_AMBIENT, [0.20, 0.22, 0.25, 1.0])

    glClearColor(0.14, 0.15, 0.17, 1.0) # Fondo oscuro de estudio GrabCAD

    # Pre-cargar y compilar todas las piezas en VRAM
    modelsd_dir = os.path.dirname(os.path.abspath(__file__))
    loaded_parts = []
    print("Compilando catálogo de piezas en GPU VRAM...")

    for p in PARTS_INFO:
        fpath = os.path.join(modelsd_dir, p["file"])
        if os.path.exists(fpath):
            l = ObjLoader(fpath)
            l.compile_display_lists()
            p_data = dict(p)
            p_data["loader"] = l
            loaded_parts.append(p_data)
            print(f"  [OK] {p['name']} ({len(l.vertices)} vértices)")

    current_idx = 0
    cur_part = loaded_parts[current_idx]

    # Parámetros de Cámara Orbital
    target = list(cur_part["target"])
    distance = cur_part["dist"]
    yaw = 28.0
    pitch = 24.0
    wireframe = False
    render_mode = "COLOR"

    def update_projection():
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(38.0, width / float(height), 1.0, 3000.0)
        glMatrixMode(GL_MODELVIEW)

    update_projection()

    clock = pygame.time.Clock()
    is_rotating = False
    is_panning = False
    last_mouse = (0, 0)

    # Fuente para HUD
    font = pygame.font.SysFont("Consolas", 12, bold=True)
    font_title = pygame.font.SysFont("Consolas", 13, bold=True)

    def draw_hud():
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        gluOrtho2D(0, width, height, 0)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()

        glDisable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        # Barra superior semi-transparente
        glColor4f(0.1, 0.12, 0.15, 0.88)
        glBegin(GL_QUADS)
        glVertex2f(10, 10)
        glVertex2f(width - 10, 10)
        glVertex2f(width - 10, 56)
        glVertex2f(10, 56)
        glEnd()

        # Textos informativos
        title_str = f"[{cur_part['id']}] {cur_part['name']} // CATÁLOGO MODULAR 3D GRABCAD"
        title_surf = font_title.render(title_str, True, (240, 245, 255))
        
        info_str = f"Modo: {render_mode} | FPS: {clock.get_fps():4.1f} | Tecla 'C': Color/Clay | 'W': Alámbrico | TAB / 0-9: Cambiar Pieza"
        info_surf = font.render(info_str, True, (160, 210, 255))

        keys_str = "[0] Ensamble | [1] Perfil 2020 | [2] Rodillo | [3] Pata Est | [4] Pata Ext | [5] Torre Roja | [6] Placa | [7] HC-SR04 | [8] Banda | [9] Motor"
        keys_surf = font.render(keys_str, True, (215, 225, 140))

        def draw_surf_at(s, x, y):
            d = pygame.image.tobytes(s, "RGBA", True)
            w, h = s.get_size()
            glEnable(GL_TEXTURE_2D)
            t = glGenTextures(1)
            glBindTexture(GL_TEXTURE_2D, t)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, d)
            glColor4f(1, 1, 1, 1)
            glBegin(GL_QUADS)
            glTexCoord2f(0, 1); glVertex2f(x, y)
            glTexCoord2f(1, 1); glVertex2f(x + w, y)
            glTexCoord2f(1, 0); glVertex2f(x + w, y + h)
            glTexCoord2f(0, 0); glVertex2f(x, y + h)
            glEnd()
            glDeleteTextures([t])
            glDisable(GL_TEXTURE_2D)

        draw_surf_at(title_surf, 20, 14)
        draw_surf_at(info_surf, width - info_surf.get_width() - 20, 14)
        draw_surf_at(keys_surf, 20, 36)

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()

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
                elif event.key == K_w:
                    wireframe = not wireframe
                elif event.key == K_r:
                    target = list(cur_part["target"])
                    distance = cur_part["dist"]
                    yaw, pitch = 28.0, 24.0
                elif event.key == K_TAB:
                    current_idx = (current_idx + 1) % len(loaded_parts)
                    cur_part = loaded_parts[current_idx]
                    target = list(cur_part["target"])
                    distance = cur_part["dist"]
                elif event.key == K_s:
                    snap_name = f"captura_{cur_part['name'].lower().replace(' ', '_')}_{render_mode.lower()}.png"
                    snap_path = os.path.join(modelsd_dir, snap_name)
                    glReadBuffer(GL_BACK)
                    data = glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE)
                    surf = pygame.image.frombytes(data, (width, height), "RGB", True)
                    pygame.image.save(surf, snap_path)
                    print(f"Captura guardada en: {snap_path}")

                else:
                    # Selector por teclas 0..9
                    for i, p in enumerate(loaded_parts):
                        if event.key == p["key"]:
                            current_idx = i
                            cur_part = loaded_parts[current_idx]
                            target = list(cur_part["target"])
                            distance = cur_part["dist"]
                            break

            elif event.type == MOUSEBUTTONDOWN:
                if event.button == 1:
                    is_rotating = True
                    last_mouse = event.pos
                elif event.button in (2, 3):
                    is_panning = True
                    last_mouse = event.pos
                elif event.button == 4:
                    distance = max(20.0, distance * 0.90)
                elif event.button == 5:
                    distance = min(1800.0, distance * 1.10)

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
                    yaw = (yaw + dx * 0.4) % 360.0
                    pitch = max(-89.0, min(89.0, pitch + dy * 0.4))

                elif is_panning:
                    rad_yaw = math.radians(yaw)
                    rx = math.cos(rad_yaw)
                    rz = -math.sin(rad_yaw)
                    pan_speed = distance * 0.0014
                    target[0] -= rx * dx * pan_speed
                    target[2] -= rz * dx * pan_speed
                    target[1] += dy * pan_speed

            elif event.type == VIDEORESIZE:
                width, height = event.w, event.h
                screen = pygame.display.set_mode((width, height), pygame.DOUBLEBUF | pygame.OPENGL | pygame.RESIZABLE)
                update_projection()

        # Render 3D
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        rad_yaw = math.radians(yaw)
        rad_pitch = math.radians(pitch)
        cam_x = target[0] + distance * math.cos(rad_pitch) * math.sin(rad_yaw)
        cam_y = target[1] + distance * math.sin(rad_pitch)
        cam_z = target[2] + distance * math.cos(rad_pitch) * math.cos(rad_yaw)

        gluLookAt(cam_x, cam_y, cam_z, target[0], target[1], target[2], 0.0, 1.0, 0.0)

        # Suelo de referencia cuadriculado
        glDisable(GL_LIGHTING)
        glColor3f(0.20, 0.22, 0.25)
        glLineWidth(1.0)
        glBegin(GL_LINES)
        grid_y = -65.0 if cur_part["id"] == "0" else (target[1] - distance * 0.3)
        grid_extent = 400
        step = 50
        for g in range(-grid_extent, grid_extent + step, step):
            glVertex3f(g, grid_y, -grid_extent)
            glVertex3f(g, grid_y, grid_extent)
            glVertex3f(-grid_extent, grid_y, g)
            glVertex3f(grid_extent, grid_y, g)
        glEnd()
        glEnable(GL_LIGHTING)

        if wireframe:
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        else:
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

        # DIBUJAR PIEZA / ENSAMBLE COMPILADO EN VRAM
        l = cur_part["loader"]
        if render_mode == "CLAY":
            glCallList(l.d_list_clay)
        else:
            glCallList(l.d_list_color)

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

        draw_hud()
        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    run_viewer()
