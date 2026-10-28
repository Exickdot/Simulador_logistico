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

# Asegurar path para ObjLoader
sys.path.append(os.path.dirname(__file__))
from ver_sensor_3d import ObjLoader

def render_snapshots():
    modelsd_dir = os.path.dirname(os.path.abspath(__file__))
    obj_path = os.path.join(modelsd_dir, "faja_transportadora_ajustable.obj")

    pygame.init()
    width, height = 1280, 800
    pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLEBUFFERS, 1)
    pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLESAMPLES, 4)
    pygame.display.gl_set_attribute(pygame.GL_DEPTH_SIZE, 24)

    screen = pygame.display.set_mode((width, height), pygame.DOUBLEBUF | pygame.OPENGL | pygame.HIDDEN)

    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glEnable(GL_LIGHT1)
    glEnable(GL_NORMALIZE)
    glShadeModel(GL_SMOOTH)
    glEnable(GL_COLOR_MATERIAL)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

    # Iluminación de estudio CAD (3 puntos de luz)
    glLightfv(GL_LIGHT0, GL_POSITION, [150.0, 350.0, 250.0, 1.0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.95, 0.95, 0.98, 1.0])
    glLightfv(GL_LIGHT0, GL_SPECULAR, [0.6, 0.6, 0.6, 1.0])
    glLightfv(GL_LIGHT0, GL_AMBIENT, [0.35, 0.36, 0.40, 1.0])

    glLightfv(GL_LIGHT1, GL_POSITION, [-250.0, 150.0, -180.0, 1.0])
    glLightfv(GL_LIGHT1, GL_DIFFUSE, [0.55, 0.58, 0.65, 1.0])
    glLightfv(GL_LIGHT1, GL_SPECULAR, [0.3, 0.3, 0.3, 1.0])
    glLightfv(GL_LIGHT1, GL_AMBIENT, [0.20, 0.22, 0.25, 1.0])

    glClearColor(0.14, 0.15, 0.17, 1.0) # Fondo oscuro de estudio CAD (GrabCAD Dark)

    print(f"Cargando {obj_path}...")
    loader = ObjLoader(obj_path)
    loader.compile_display_lists()
    print("Modelo compilado en VRAM.")

    def setup_camera(target, distance, yaw_deg, pitch_deg):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(38.0, width / float(height), 1.0, 2000.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        rad_yaw = math.radians(yaw_deg)
        rad_pitch = math.radians(pitch_deg)

        cam_x = target[0] + distance * math.cos(rad_pitch) * math.sin(rad_yaw)
        cam_y = target[1] + distance * math.sin(rad_pitch)
        cam_z = target[2] + distance * math.cos(rad_pitch) * math.cos(rad_yaw)

        gluLookAt(cam_x, cam_y, cam_z, target[0], target[1], target[2], 0.0, 1.0, 0.0)

    snapshots = [
        {
            "name": "preview_conveyor_overview.png",
            "target": [-20.0, 10.0, 0.0],
            "dist": 660.0,
            "yaw": 28.0,
            "pitch": 24.0,
            "list": loader.d_list_color,
            "title": "Vista Isométrica General con Extensión Longitudinal de Sensor"
        },
        {
            "name": "preview_conveyor_discharge_gap.png",
            "target": [-175.0, 18.0, 0.0],
            "dist": 320.0,
            "yaw": 38.0,
            "pitch": 22.0,
            "list": loader.d_list_color,
            "title": "Detalle del Espacio de Descarga y Haz Ultrasónico Frontal"
        },
        {
            "name": "preview_conveyor_sensor_alignment.png",
            "target": [-170.0, 20.0, 0.0],
            "dist": 310.0,
            "yaw": 82.0,
            "pitch": 12.0,
            "list": loader.d_list_color,
            "title": "Alineación Horizontal Perfecta del Sensor y Paso Libre de Cajas"
        },
        {
            "name": "preview_conveyor_clay.png",
            "target": [-20.0, 10.0, 0.0],
            "dist": 660.0,
            "yaw": 28.0,
            "pitch": 24.0,
            "list": loader.d_list_clay,
            "title": "Modo Monocromático CAD (Clay)"
        }
    ]

    for snap in snapshots:
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        setup_camera(snap["target"], snap["dist"], snap["yaw"], snap["pitch"])

        glCallList(snap["list"])

        glReadBuffer(GL_BACK)
        data = glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE)
        surf = pygame.image.frombytes(data, (width, height), "RGB", True)
        out_path = os.path.join(modelsd_dir, snap["name"])
        pygame.image.save(surf, out_path)
        print(f"Captura guardada: {out_path} ({snap['title']})")

    pygame.quit()
    print("Todas las capturas se generaron satisfactoriamente.")

if __name__ == "__main__":
    render_snapshots()
