import math
import os
import struct
from model_builder import ModelBuilder

def build_hcsr04(scale=1.0):
    """
    Construye el modelo 3D detallado del sensor ultrasónico HC-SR04
    según las especificaciones industriales de GrabCAD (Octavi Masià).
    scale=1.0 genera dimensiones en milímetros (CAD estándar).
    scale=0.001 genera dimensiones en metros (para motores gráficos).
    """
    b = ModelBuilder(scale=scale)

    # =========================================================================
    # 1. PLACA DE CIRCUITO IMPRESO (PCB CON 4 ORIFICIOS PASANTES REALES)
    # =========================================================================
    # Dimensiones estándar: 45.0 mm ancho x 20.0 mm alto x 1.6 mm espesor
    pcb_w, pcb_h, pcb_thick = 45.0, 20.0, 1.6
    z_front, z_back = 0.0, -pcb_thick
    z_mid = (z_front + z_back) * 0.5

    # 4 Orificios de montaje en las esquinas (Diámetro 2.0 mm, Radio 1.0 mm)
    hole_r = 1.0
    hole_centers = [
        (-20.5, 8.0),   # Superior Izquierdo
        (20.5, 8.0),    # Superior Derecho
        (-20.5, -8.0),  # Inferior Izquierdo
        (20.5, -8.0)    # Inferior Derecho
    ]

    b.set_material("Material_PCB")

    # Para que los 4 agujeros sean perforaciones pasantes reales (visibles a través):
    # A. Cuerpo central sólido de la PCB (sin agujeros): de X = -18.5 a +18.5 mm (ancho 37 mm)
    b.add_box(0.0, 0.0, z_mid, 37.0, pcb_h, pcb_thick)

    # B. Bloques de los extremos laterales con calado cilíndrico para cada agujero:
    # Para cada lateral (izquierdo X: -22.5 a -18.5, derecho X: +18.5 a +22.5):
    for side in (-1.0, 1.0):
        xc_side = side * 20.5
        # Bloque central vertical entre los dos agujeros (Y: -6.5 a +6.5 mm)
        b.add_box(xc_side, 0.0, z_mid, 4.0, 13.0, pcb_thick)
        # Borde exterior extremo (X: 21.5 a 22.5 o -22.5 a -21.5)
        b.add_box(side * 22.0, 8.0, z_mid, 1.0, 3.0, pcb_thick)
        b.add_box(side * 22.0, -8.0, z_mid, 1.0, 3.0, pcb_thick)
        # Borde interior hacia el centro (X: 18.5 a 19.5 o -19.5 a -18.5)
        b.add_box(side * 19.0, 8.0, z_mid, 1.0, 3.0, pcb_thick)
        b.add_box(side * 19.0, -8.0, z_mid, 1.0, 3.0, pcb_thick)
        # Borde superior e inferior de las esquinas
        b.add_box(xc_side, 9.5, z_mid, 4.0, 1.0, pcb_thick)
        b.add_box(xc_side, -9.5, z_mid, 4.0, 1.0, pcb_thick)

    # C. Ojaletes / Pistas metalizadas cilíndricas en el interior de cada orificio
    b.set_material("Material_Mounting_Holes")
    for (hx_c, hy_c) in hole_centers:
        b.add_hollow_cylinder(hx_c, hy_c, z_back, z_front, hole_r + 0.35, hole_r, segments=20)

    # =========================================================================
    # 2. CÁPSULAS METÁLICAS DE LOS TRANSDUCTORES (T Y R)
    # =========================================================================
    # Dos cilindros de aluminio de 16.0 mm de diámetro exterior
    can_r_outer = 8.0
    can_r_inner = 7.1
    can_centers_x = [-13.0, 13.0] # 26 mm de distancia entre centros
    can_center_y = 1.5

    for can_x in can_centers_x:
        b.set_material("Material_Transducer_Can")

        # A. Cuerpo cilíndrico hueco principal (de Z=0 a Z=11.4 mm)
        b.add_hollow_cylinder(can_x, can_center_y, 0.0, 11.4, can_r_outer, can_r_inner, segments=36)

        # B. Bisel / Chaflán del labio frontal superior (de Z=11.4 a Z=12.2 mm)
        segments = 36
        ring_outer_1 = []
        ring_outer_2 = []
        ring_inner_1 = []
        ring_inner_2 = []

        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            ca, sa = math.cos(angle), math.sin(angle)
            ring_outer_1.append(b.add_vertex(can_x + can_r_outer * ca, can_center_y + can_r_outer * sa, 11.4))
            ring_outer_2.append(b.add_vertex(can_x + (can_r_outer - 0.3) * ca, can_center_y + (can_r_outer - 0.3) * sa, 12.2))
            ring_inner_2.append(b.add_vertex(can_x + (can_r_inner + 0.2) * ca, can_center_y + (can_r_inner + 0.2) * sa, 12.2))
            ring_inner_1.append(b.add_vertex(can_x + can_r_inner * ca, can_center_y + can_r_inner * sa, 11.4))

        n_chamfer = b.add_normal(0, 0, 1)
        for i in range(segments):
            ni = (i + 1) % segments
            # Cara chanfle exterior
            b.add_quad(ring_outer_1[i], ring_outer_1[ni], ring_outer_2[ni], ring_outer_2[i])
            # Cara plana superior de la pestaña
            b.add_quad(ring_outer_2[i], ring_outer_2[ni], ring_inner_2[ni], ring_inner_2[i], n_chamfer, n_chamfer, n_chamfer, n_chamfer)
            # Cara chanfle interior
            b.add_quad(ring_inner_2[i], ring_inner_2[ni], ring_inner_1[ni], ring_inner_1[i])

        # C. Cono piezoeléctrico interior reflectante (visible detrás de la rejilla)
        b.set_material("Material_Transducer_Cone")
        # Cono central que amplifica la onda ultrasónica a 40 kHz
        b.add_cylinder(can_x, can_center_y, 2.0, 7.5, 6.2, 2.8, segments=24, capped=True)
        # Botón central brillante piezoeléctrico
        b.add_cylinder(can_x, can_center_y, 7.5, 8.2, 2.6, 2.0, segments=24, capped=True)

        # D. Rejilla metálica protectora cuadriculada (Wire Mesh Grid)
        # Exactamente como en la imagen de referencia de GrabCAD
        b.set_material("Material_Transducer_Mesh")
        z_mesh = 11.2
        wire_w = 0.26   # Ancho del alambre
        wire_th = 0.22  # Espesor del alambre
        grid_step = 1.05 # Espaciado de cuadrícula (1.05 mm)
        r_grid_limit = can_r_inner - 0.05

        # Barras de alambre horizontales
        y_val = -6.0
        while y_val <= 6.0:
            dy = y_val
            if abs(dy) < r_grid_limit:
                half_span = math.sqrt(r_grid_limit*r_grid_limit - dy*dy)
                b.add_box(can_x, can_center_y + dy, z_mesh, half_span * 2.0, wire_w, wire_th)
            y_val += grid_step

        # Barras de alambre verticales
        x_val = -6.0
        while x_val <= 6.0:
            dx = x_val
            if abs(dx) < r_grid_limit:
                half_span = math.sqrt(r_grid_limit*r_grid_limit - dx*dx)
                b.add_box(can_x + dx, can_center_y, z_mesh + wire_th*0.5, wire_w, half_span * 2.0, wire_th)
            x_val += grid_step

        # Anillo perimetral que aprisiona la malla
        b.add_hollow_cylinder(can_x, can_center_y, z_mesh - 0.2, z_mesh + 0.3, can_r_inner, can_r_inner - 0.45, segments=32)

    # =========================================================================
    # 3. RESONADOR DE CUARZO / CRISTAL OSCILADOR (HC-49 CAN)
    # =========================================================================
    # Ubicado arriba entre los dos transductores: X=0, Y=+7.2 mm
    c_x, c_y = 0.0, 7.2
    c_w, c_h = 10.8, 4.4 # Obround: 6.4 mm largo recto + 2x radio 2.2 mm
    c_rad = c_h * 0.5
    c_straight = (c_w - c_h) * 0.5 # 3.2 mm

    # Junta/gasket aislante negra en la base
    b.set_material("Material_Crystal_Base")
    b.add_box(c_x, c_y, 0.25, c_w + 0.8, c_h + 0.8, 0.5)

    # Cuerpo metálico estañado del cristal
    b.set_material("Material_Crystal_Oscillator")
    z_c_bot, z_c_top = 0.5, 3.8
    osc_segments = 16

    # Construcción de prisma cilíndrico redondeado (Obround)
    # 1. Bloque central rectangular
    b.add_box(c_x, c_y, (z_c_bot + z_c_top)*0.5, c_straight * 2.0, c_h, z_c_top - z_c_bot)

    # 2. Dos semicilindros en los extremos (+X y -X)
    for sign in (-1.0, 1.0):
        xc_end = c_x + sign * c_straight
        for i in range(osc_segments):
            base_angle = -math.pi * 0.5 if sign > 0 else math.pi * 0.5
            a1 = base_angle + (math.pi * i / osc_segments)
            a2 = base_angle + (math.pi * (i + 1) / osc_segments)

            x1_b = xc_end + c_rad * math.cos(a1)
            y1_b = c_y + c_rad * math.sin(a1)
            x2_b = xc_end + c_rad * math.cos(a2)
            y2_b = c_y + c_rad * math.sin(a2)

            v1_b = b.add_vertex(x1_b, y1_b, z_c_bot)
            v2_b = b.add_vertex(x2_b, y2_b, z_c_bot)
            v2_t = b.add_vertex(x2_b, y2_b, z_c_top)
            v1_t = b.add_vertex(x1_b, y1_b, z_c_top)

            n1 = b.add_normal(math.cos(a1), math.sin(a1), 0.0)
            n2 = b.add_normal(math.cos(a2), math.sin(a2), 0.0)
            b.add_quad(v1_b, v2_b, v2_t, v1_t, n1, n2, n2, n1)

            # Tapa superior
            c_top_pt = b.add_vertex(xc_end, c_y, z_c_top)
            n_up = b.add_normal(0, 0, 1)
            b.add_triangle(c_top_pt, v1_t, v2_t, n_up, n_up, n_up)

    # Tapa superior del bloque central
    b.add_quad(
        b.add_vertex(c_x - c_straight, c_y - c_rad, z_c_top),
        b.add_vertex(c_x + c_straight, c_y - c_rad, z_c_top),
        b.add_vertex(c_x + c_straight, c_y + c_rad, z_c_top),
        b.add_vertex(c_x - c_straight, c_y + c_rad, z_c_top),
        b.add_normal(0, 0, 1), b.add_normal(0, 0, 1), b.add_normal(0, 0, 1), b.add_normal(0, 0, 1)
    )

    # Relieve superior ovalado característico de la cápsula HC-49
    b.add_box(c_x, c_y, z_c_top + 0.18, c_w - 0.8, c_h - 0.8, 0.35)

    # =========================================================================
    # 4. CONECTOR HEADER DE 4 PINES (VCC, TRIG, ECHO, GND)
    # =========================================================================
    # Paso estándar: 2.54 mm. Centrado en X=0, borde inferior de la placa
    pin_pitch = 2.54
    pin_xs = [-1.5 * pin_pitch, -0.5 * pin_pitch, 0.5 * pin_pitch, 1.5 * pin_pitch]
    pin_y_base = -8.2
    pin_len = 10.5 # Longitud que sobresale hacia abajo
    pin_thick = 0.64 # Pin cuadrado estándar 0.64 x 0.64 mm

    # Soportes / Pads plásticos de inserción en la PCB (exactos como en la foto)
    b.set_material("Material_Header_Plastic")
    for px in pin_xs:
        b.add_box(px, pin_y_base, 0.45, 1.35, 1.35, 0.9)

    # Pines metálicos estañados / dorados
    b.set_material("Material_Header_Pins")
    for px in pin_xs:
        p_cen_y = pin_y_base - pin_len * 0.5
        b.add_box(px, p_cen_y, 0.45, pin_thick, pin_len, pin_thick)

    # =========================================================================
    # 5. COMPONENTES SMD AUXILIARES (CIRCUITOS INTEGRADOS Y PASIVOS)
    # =========================================================================
    b.set_material("Material_SMD_Chip")
    # Chip receptor amplificador (SOIC-16 / MAX232 o LM324)
    b.add_box(-12.0, -5.0, z_back - 0.7, 8.5, 4.5, 1.4)
    # Chip temporizador / microcontrolador
    b.add_box(12.0, -5.0, z_back - 0.7, 8.5, 4.5, 1.4)
    # Chip comparador central
    b.add_box(0.0, -1.0, z_back - 0.6, 5.0, 4.0, 1.2)

    # Pines SMD de los chips
    b.set_material("Material_Header_Pins")
    for ic_x in (-12.0, 12.0):
        for py_i in range(8):
            py_pos = -6.8 + py_i * 0.52
            b.add_box(ic_x - 4.6, py_pos, z_back - 0.25, 0.9, 0.28, 0.5)
            b.add_box(ic_x + 4.6, py_pos, z_back - 0.25, 0.9, 0.28, 0.5)

    return b

def write_mtl(filename):
    with open(filename, 'w', encoding='utf-8') as f:
        f.write("# Material Library for HC-SR04 Ultrasonic Sensor\n")
        f.write("# PBR / Phong Materials - GrabCAD Realistic Finish\n\n")

        # 1. PCB Blue (Azul FR4 Arduino)
        f.write("newmtl Material_PCB\n")
        f.write("Ka 0.04 0.12 0.25\n")
        f.write("Kd 0.10 0.26 0.52\n")
        f.write("Ks 0.25 0.30 0.35\n")
        f.write("Ns 24.0\n")
        f.write("d 1.0\n\n")

        # 2. Transducer Aluminum Can (Aluminio cepillado)
        f.write("newmtl Material_Transducer_Can\n")
        f.write("Ka 0.25 0.26 0.28\n")
        f.write("Kd 0.78 0.80 0.84\n")
        f.write("Ks 0.85 0.88 0.92\n")
        f.write("Ns 68.0\n")
        f.write("d 1.0\n\n")

        # 3. Transducer Mesh (Malla de alambre acero inoxidable)
        f.write("newmtl Material_Transducer_Mesh\n")
        f.write("Ka 0.18 0.19 0.20\n")
        f.write("Kd 0.40 0.42 0.45\n")
        f.write("Ks 0.65 0.68 0.70\n")
        f.write("Ns 45.0\n")
        f.write("d 1.0\n\n")

        # 4. Transducer Inner Piezo Cone (Cono piezoeléctrico reflectante)
        f.write("newmtl Material_Transducer_Cone\n")
        f.write("Ka 0.30 0.32 0.34\n")
        f.write("Kd 0.88 0.90 0.94\n")
        f.write("Ks 0.90 0.92 0.95\n")
        f.write("Ns 90.0\n")
        f.write("d 1.0\n\n")

        # 5. Crystal Oscillator Can (Cápsula de cuarzo HC-49)
        f.write("newmtl Material_Crystal_Oscillator\n")
        f.write("Ka 0.28 0.29 0.31\n")
        f.write("Kd 0.82 0.84 0.87\n")
        f.write("Ks 0.92 0.94 0.96\n")
        f.write("Ns 95.0\n")
        f.write("d 1.0\n\n")

        # 6. Crystal Base Gasket (Junta aislante)
        f.write("newmtl Material_Crystal_Base\n")
        f.write("Ka 0.08 0.08 0.09\n")
        f.write("Kd 0.15 0.15 0.17\n")
        f.write("Ks 0.10 0.10 0.10\n")
        f.write("Ns 10.0\n")
        f.write("d 1.0\n\n")

        # 7. Header Plastic Bases (Zócalo de pines)
        f.write("newmtl Material_Header_Plastic\n")
        f.write("Ka 0.08 0.08 0.09\n")
        f.write("Kd 0.16 0.16 0.18\n")
        f.write("Ks 0.20 0.20 0.20\n")
        f.write("Ns 18.0\n")
        f.write("d 1.0\n\n")

        # 8. Header Pins (Pines metálicos estañados)
        f.write("newmtl Material_Header_Pins\n")
        f.write("Ka 0.25 0.25 0.28\n")
        f.write("Kd 0.82 0.85 0.89\n")
        f.write("Ks 0.88 0.90 0.94\n")
        f.write("Ns 75.0\n")
        f.write("d 1.0\n\n")

        # 9. Mounting Holes Plating (Ojaletes metalizados)
        f.write("newmtl Material_Mounting_Holes\n")
        f.write("Ka 0.20 0.18 0.10\n")
        f.write("Kd 0.72 0.65 0.35\n")
        f.write("Ks 0.60 0.55 0.30\n")
        f.write("Ns 35.0\n")
        f.write("d 1.0\n\n")

        # 10. SMD Chips (Circuitos integrados SOIC)
        f.write("newmtl Material_SMD_Chip\n")
        f.write("Ka 0.08 0.08 0.08\n")
        f.write("Kd 0.14 0.14 0.15\n")
        f.write("Ks 0.25 0.25 0.25\n")
        f.write("Ns 20.0\n")
        f.write("d 1.0\n\n")

def main():
    out_dir = r"C:\PY\Auto_tif\modelsd"
    os.makedirs(out_dir, exist_ok=True)

    print("======================================================================")
    print(" GENERADOR CAD 3D: SENSOR ULTRASONICO HC-SR04")
    print(" Referencia de fidelidad: GrabCAD / Octavi Masia")
    print("======================================================================")

    # 1. Generar MTL compartido
    mtl_path = os.path.join(out_dir, "HC-SR04-UltrasonicSensor.mtl")
    write_mtl(mtl_path)
    print(f"[1/4] Archivo MTL creado: {mtl_path}")

    # 2. Generar Modelo en Milímetros (CAD estándar 1:1)
    print("Generando geometría de alta resolución en milímetros...")
    builder_mm = build_hcsr04(scale=1.0)
    obj_mm_path = os.path.join(out_dir, "HC-SR04-UltrasonicSensor.obj")
    builder_mm.write_obj(obj_mm_path, mtl_path)
    print(f"[2/4] Archivo OBJ (mm) generado: {obj_mm_path}")
    print(f"      -> Vértices: {len(builder_mm.vertices):,}")
    print(f"      -> Normales: {len(builder_mm.normals):,}")
    print(f"      -> Caras:    {len(builder_mm.faces):,}")

    stl_path = os.path.join(out_dir, "HC-SR04-UltrasonicSensor.stl")
    builder_mm.write_stl(stl_path)
    print(f"[3/4] Archivo STL binario generado: {stl_path} ({os.path.getsize(stl_path):,} bytes)")

    # 3. Generar Modelo en Metros (Escala para motores gráficos 3D: ancho = 0.045m)
    print("Generando geometría a escala métrica (0.001)...")
    builder_m = build_hcsr04(scale=0.001)
    obj_m_path = os.path.join(out_dir, "HC-SR04-UltrasonicSensor-meters.obj")
    mtl_m_path = os.path.join(out_dir, "HC-SR04-UltrasonicSensor-meters.mtl")
    write_mtl(mtl_m_path)
    builder_m.write_obj(obj_m_path, mtl_m_path)
    print(f"[4/4] Archivo OBJ (metros) generado: {obj_m_path}")

    print("======================================================================")
    print(" MODELOS 3D GENERADOS SATISFACTORIAMENTE EN LA CARPETA 'modelsd'")
    print("======================================================================")

if __name__ == "__main__":
    main()
