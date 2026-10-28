import os
import sys
import math
import pygame
from pygame.locals import *

try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
except ImportError:
    print("[ERROR] PyOpenGL no esta instalado.")
    sys.exit(1)

sys.path.append(os.path.dirname(__file__))
from ver_sensor_3d import ObjLoader

def render_part_thumbnail(obj_path, out_path, dist, target, yaw=30.0, pitch=20.0, size=(400, 300)):
    w, h = size
    glViewport(0, 0, w, h)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(38.0, w / float(h), 1.0, 2000.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    rad_yaw = math.radians(yaw)
    rad_pitch = math.radians(pitch)
    cx = target[0] + dist * math.cos(rad_pitch) * math.sin(rad_yaw)
    cy = target[1] + dist * math.sin(rad_pitch)
    cz = target[2] + dist * math.cos(rad_pitch) * math.cos(rad_yaw)
    gluLookAt(cx, cy, cz, target[0], target[1], target[2], 0.0, 1.0, 0.0)

    loader = ObjLoader(obj_path)
    loader.compile_display_lists()

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glCallList(loader.d_list_color)

    glReadBuffer(GL_BACK)
    data = glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE)
    surf = pygame.image.frombytes(data, (w, h), "RGB", True)
    return surf

def create_catalog_poster():
    modelsd_dir = os.path.dirname(os.path.abspath(__file__))
    pygame.init()
    screen = pygame.display.set_mode((1280, 800), pygame.DOUBLEBUF | pygame.OPENGL | pygame.HIDDEN)

    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_LIGHT1)
    glEnable(GL_NORMALIZE)
    glShadeModel(GL_SMOOTH)
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

    glLightfv(GL_LIGHT0, GL_POSITION, [150.0, 300.0, 250.0, 1.0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.95, 0.95, 0.98, 1.0])
    glLightfv(GL_LIGHT0, GL_AMBIENT, [0.35, 0.36, 0.40, 1.0])
    glLightfv(GL_LIGHT1, GL_POSITION, [-250.0, 150.0, -180.0, 1.0])
    glLightfv(GL_LIGHT1, GL_DIFFUSE, [0.55, 0.58, 0.65, 1.0])
    glLightfv(GL_LIGHT1, GL_AMBIENT, [0.20, 0.22, 0.25, 1.0])
    glClearColor(0.12, 0.13, 0.15, 1.0)

    parts_to_render = [
        ("perfil_aluminio_2020.obj",         380.0, [0.0, 0.0, 0.0],  "1. Perfil Aluminio 2020 T-Slot"),
        ("rodillo_conveyor.obj",             160.0, [0.0, 0.0, 0.0],  "2. Rodillo, Eje y Rodamientos 608ZZ"),
        ("pata_soporte_lateral.obj",         130.0, [0.0, -15.0, 0.0],"3. Pata de Soporte Estándar"),
        ("pata_soporte_extendida.obj",       160.0, [-30.0, -15.0, 0.0],"4. Pata con Brazo p/ Sensor"),
        ("torre_roja_ajustable.obj",         140.0, [0.0, 30.0, 0.0], "5. Torre Roja con Corredera"),
        ("placa_soporte_sensor.obj",         160.0, [0.0, 0.0, 0.0],  "6. Placa Montaje GrabCAD"),
        ("sensor_hcsr04_compacto.obj",       80.0,  [0.0, 0.0, 0.0],  "7. Sensor Ultrasónico HC-SR04"),
        ("motor_nema17_drive.obj",           130.0, [0.0, -10.0, 0.0],"8. Motor NEMA 17 con Acople")
    ]

    cell_w, cell_h = 300, 220
    cols, rows = 4, 2
    poster_w, poster_h = cols * cell_w + 40, rows * cell_h + 100

    # Crear superficie 2D en memoria
    poster = pygame.Surface((poster_w, poster_h))
    poster.fill((20, 22, 26))

    font_title = pygame.font.SysFont("Consolas", 15, bold=True)
    font_lbl = pygame.font.SysFont("Consolas", 11, bold=True)

    header = font_title.render("CATÁLOGO DE COMPONENTES 3D DESPIEZADOS // FAJA MODULAR DE LABORATORIO", True, (240, 245, 255))
    poster.blit(header, (25, 18))

    for idx, (fobj, dist, tgt, label) in enumerate(parts_to_render):
        r = idx // cols
        c = idx % cols
        x = 20 + c * cell_w
        y = 55 + r * cell_h

        obj_file = os.path.join(modelsd_dir, fobj)
        thumb = render_part_thumbnail(obj_file, None, dist, tgt, yaw=32.0, pitch=22.0, size=(cell_w - 10, cell_h - 40))
        poster.blit(thumb, (x, y))

        # Marco y etiqueta
        pygame.draw.rect(poster, (45, 50, 60), (x, y, cell_w - 10, cell_h - 10), 1)
        lbl_surf = font_lbl.render(label, True, (170, 215, 255))
        poster.blit(lbl_surf, (x + 8, y + cell_h - 32))

    out_poster = os.path.join(modelsd_dir, "preview_parts_catalog.png")
    pygame.image.save(poster, out_poster)
    print(f"[OK] Poster del catálogo generado: {out_poster}")
    pygame.quit()

if __name__ == "__main__":
    create_catalog_poster()
