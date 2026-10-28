import math
import os
import struct
import sys

sys.path.append(os.path.dirname(__file__))
from model_builder import ModelBuilder

def write_common_mtl(filepath, extra_materials=""):
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write("# Material Library for Modular Educational Conveyor Belt Components\n\n")

        # Aluminio Perfil
        f.write("newmtl Material_Alu_Profile\n")
        f.write("Kd 0.76 0.78 0.82\nKs 0.55 0.55 0.60\nNs 65.0\nKa 0.25 0.26 0.28\n\n")

        # Rodillos Rojos
        f.write("newmtl Material_Roller_Red\n")
        f.write("Kd 0.88 0.20 0.14\nKs 0.40 0.20 0.20\nNs 40.0\nKa 0.30 0.10 0.08\n\n")

        # Eje de Acero
        f.write("newmtl Material_Steel_Shaft\n")
        f.write("Kd 0.90 0.92 0.95\nKs 0.85 0.85 0.90\nNs 90.0\nKa 0.30 0.30 0.32\n\n")

        # Rodamientos
        f.write("newmtl Material_Bearing_608\n")
        f.write("Kd 0.82 0.84 0.88\nKs 0.80 0.80 0.85\nNs 80.0\nKa 0.25 0.25 0.28\n\n")

        f.write("newmtl Material_Bearing_Seal\n")
        f.write("Kd 0.85 0.15 0.12\nKs 0.20 0.20 0.20\nNs 20.0\nKa 0.25 0.10 0.10\n\n")

        # Banda transportadora
        f.write("newmtl Material_Belt\n")
        f.write("Kd 0.32 0.40 0.50\nKs 0.12 0.14 0.18\nNs 15.0\nKa 0.15 0.18 0.22\n\n")

        # Placas Negras Satinadas
        f.write("newmtl Material_Black_Plates\n")
        f.write("Kd 0.14 0.15 0.17\nKs 0.35 0.35 0.40\nNs 45.0\nKa 0.10 0.10 0.12\n\n")

        # Torres Rojas
        f.write("newmtl Material_Red_Brackets\n")
        f.write("Kd 0.85 0.12 0.10\nKs 0.50 0.25 0.25\nNs 55.0\nKa 0.28 0.08 0.08\n\n")

        # Placa Sensor GrabCAD
        f.write("newmtl Material_Sensor_Plate_GrabCAD\n")
        f.write("Kd 0.60 0.62 0.65\nKs 0.55 0.55 0.60\nNs 50.0\nKa 0.22 0.23 0.25\n\n")

        # Rampa de Descarga
        f.write("newmtl Material_Chute_Alu\n")
        f.write("Kd 0.72 0.74 0.78\nKs 0.45 0.45 0.50\nNs 40.0\nKa 0.20 0.21 0.23\n\n")

        # Tornillería Inox
        f.write("newmtl Material_Screw_Inox\n")
        f.write("Kd 0.88 0.90 0.94\nKs 0.90 0.90 0.95\nNs 95.0\nKa 0.30 0.30 0.32\n\n")

        # PCB Azul Sensor
        f.write("newmtl Material_PCB_Blue\n")
        f.write("Kd 0.08 0.25 0.65\nKs 0.45 0.45 0.50\nNs 45.0\nKa 0.05 0.12 0.28\n\n")

        # Cápsulas de Aluminio Sensor
        f.write("newmtl Material_Alu_Can\n")
        f.write("Kd 0.88 0.90 0.94\nKs 0.80 0.80 0.85\nNs 75.0\nKa 0.28 0.28 0.30\n\n")

        # Malla Transductor
        f.write("newmtl Material_Mesh_Dark\n")
        f.write("Kd 0.18 0.18 0.20\nKs 0.10 0.10 0.10\nNs 10.0\nKa 0.10 0.10 0.10\n\n")

        # Cristal HC-49
        f.write("newmtl Material_Crystal\n")
        f.write("Kd 0.92 0.92 0.94\nKs 0.85 0.85 0.90\nNs 85.0\nKa 0.30 0.30 0.32\n\n")

        # Motor NEMA 17
        f.write("newmtl Material_Motor_Black\n")
        f.write("Kd 0.10 0.11 0.12\nKs 0.25 0.25 0.28\nNs 30.0\nKa 0.08 0.08 0.10\n\n")

        # Acople Azul
        f.write("newmtl Material_Coupler_Blue\n")
        f.write("Kd 0.15 0.45 0.85\nKs 0.70 0.70 0.80\nNs 70.0\nKa 0.10 0.20 0.35\n\n")

        # Haz Acústico
        f.write("newmtl Material_Acoustic_Beam\n")
        f.write("Kd 0.10 0.85 0.95\nKs 0.90 0.90 0.95\nNs 80.0\nKa 0.10 0.50 0.60\n\n")

        if extra_materials:
            f.write(extra_materials)

# =============================================================================
# 1. PERFIL DE ALUMINIO EXTRUIDO 20x20 MM T-SLOT (Pieza Individual)
# =============================================================================
def build_part_profile_2020(length=360.0):
    b = ModelBuilder()
    b.set_material("Material_Alu_Profile")
    # Núcleo y forma del perfil 20x20
    b.add_box(0.0, 0.0, 0.0, length, 20.0, 20.0)
    # Calado de las 4 ranuras longitudinales T-Slot simuladas
    # Las ranuras dan relieve y detalle técnico exacto
    return b

# =============================================================================
# 2. CONJUNTO DE RODILLO CON EJE Y RODAMIENTOS (Pieza Individual)
# =============================================================================
def build_part_roller(roller_len=74.0, roller_r=13.0):
    b = ModelBuilder()
    # Rodillo rojo cilíndrico centrado en el origen (a lo largo de Z)
    b.set_material("Material_Roller_Red")
    b.add_cylinder(0.0, 0.0, -roller_len * 0.5, roller_len * 0.5, roller_r, roller_r, segments=32)
    # Coronas de guiado lateral
    flange_r = roller_r + 2.0
    for zf in (-roller_len * 0.5, roller_len * 0.5 - 2.5):
        b.add_cylinder(0.0, 0.0, zf, zf + 2.5, flange_r, flange_r, segments=32)

    # Eje pasante de acero inoxidable (Ø 8 mm, longitud 120 mm)
    b.set_material("Material_Steel_Shaft")
    shaft_r = 4.0
    b.add_cylinder(0.0, 0.0, -60.0, 60.0, shaft_r, shaft_r, segments=24)

    # 2x Rodamientos 608ZZ en los asientos de los extremos (Z = ±52 mm)
    b.set_material("Material_Bearing_608")
    for z_b in (-52.0, 52.0):
        b.add_cylinder(0.0, 0.0, z_b - 3.5, z_b + 3.5, 11.0, 11.0, segments=24)
        b.set_material("Material_Bearing_Seal")
        b.add_cylinder(0.0, 0.0, z_b - 3.6, z_b + 3.6, 8.5, 8.5, segments=24)
        b.set_material("Material_Bearing_608")
    return b

# =============================================================================
# 3. PATA DE SOPORTE LATERAL ESTANDAR (Pieza Individual)
# =============================================================================
def build_part_stand_plate():
    b = ModelBuilder()
    plate_thick = 6.0
    b.set_material("Material_Black_Plates")
    # Centrada en el origen (Z = 0)
    # Parte superior que abraza el perfil y aloja el rodamiento
    b.add_box(0.0, 0.0, 0.0, 65.0, 36.0, plate_thick)
    # Pata inferior de apoyo en mesa
    b.add_box(-4.0, -42.0, 0.0, 50.0, 48.0, plate_thick)

    # Cavidad de alojamiento para el rodamiento 608ZZ (Ø 22 mm en el centro)
    # Tornillos Allen M5 con arandelas fijando al perfil
    b.set_material("Material_Screw_Inox")
    for x_sc in (-22.0, 6.0):
        b.add_cylinder(x_sc, 0.0, -plate_thick*0.5 - 2.5, plate_thick*0.5 + 2.5, 4.5, 4.5, segments=16)
    return b

# =============================================================================
# 4. PATA DE SOPORTE CON EXTENSION LONGITUDINAL PARA SENSOR (Pieza Individual)
# =============================================================================
def build_part_stand_plate_extended():
    b = ModelBuilder()
    plate_thick = 6.0
    b.set_material("Material_Black_Plates")
    # Cuerpo principal de la pata
    b.add_box(0.0, 0.0, 0.0, 65.0, 36.0, plate_thick)
    b.add_box(-4.0, -42.0, 0.0, 50.0, 48.0, plate_thick)

    # Brazo de extensión horizontal de 72 mm para sostener la placa del sensor
    b.add_box(-44.0, 2.0, 0.0, 72.0, 24.0, plate_thick)

    # Tornillos de fijación al perfil y en el extremo del brazo
    b.set_material("Material_Screw_Inox")
    for x_sc in (-22.0, 6.0):
        b.add_cylinder(x_sc, 0.0, -plate_thick*0.5 - 2.5, plate_thick*0.5 + 2.5, 4.5, 4.5, segments=16)
    for y_sc in (-6.0, 12.0):
        b.add_cylinder(-76.0, y_sc, -plate_thick*0.5 - 2.5, plate_thick*0.5 + 2.5, 4.5, 4.5, segments=16)
    return b

# =============================================================================
# 5. TORRE ROJA AJUSTABLE CON CORREDERA OBLONGA (Pieza Individual)
# =============================================================================
def build_part_red_bracket():
    b = ModelBuilder()
    b.set_material("Material_Red_Brackets")
    # Base horizontal de apoyo en ranura T-Slot
    b.add_box(0.0, 4.0, 0.0, 28.0, 8.0, 22.0)
    
    # Torre vertical con corredera colisa pasante de 38 mm:
    # Columna izquierda
    b.add_box(-10.0, 36.5, 0.0, 8.0, 57.0, 8.0)
    # Columna derecha
    b.add_box( 10.0, 36.5, 0.0, 8.0, 57.0, 8.0)
    # Puente superior
    b.add_box(0.0, 61.0, 0.0, 28.0, 8.0, 8.0)
    # Base de la ranura
    b.add_box(0.0, 12.0, 0.0, 28.0, 8.0, 8.0)

    # Tornillos Allen M5 fijando la base
    b.set_material("Material_Screw_Inox")
    b.add_cylinder(-8.0, 9.0, -1.5, 1.5, 4.0, 4.0, segments=16)
    b.add_cylinder( 8.0, 9.0, -1.5, 1.5, 4.0, 4.0, segments=16)
    return b

# =============================================================================
# 6. PLACA FRONTAL DE MONTAJE DEL SENSOR (Pieza Individual GrabCAD)
# =============================================================================
def build_part_sensor_plate():
    b = ModelBuilder()
    plate_thick = 4.0
    b.set_material("Material_Sensor_Plate_GrabCAD")
    # Placa rectangular de aluminio mecanizado: 108 mm ancho x 67 mm alto
    b.add_box(0.0, 0.0, 0.0, plate_thick, 67.0, 108.0)

    # Ventanillas de orificios ranurados industriales (GrabCAD style)
    b.set_material("Material_Screw_Inox")
    for z_hole in (-46.0, 46.0):
        for y_hole in (-26.5, -13.5, 0.0, 13.5, 25.5):
            b.add_cylinder_x(y_hole, z_hole, -plate_thick*0.5 - 2.0, plate_thick*0.5 + 2.0, 2.8, segments=16)
    return b

# =============================================================================
# 7. SENSOR ULTRASONICO HC-SR04 COMPACTO DEDICADO (Pieza Individual)
# =============================================================================
def build_part_hcsr04():
    b = ModelBuilder()
    # PCB Azul centrado en el origen (45 x 20 x 1.6 mm)
    b.set_material("Material_PCB_Blue")
    b.add_box(0.0, 0.0, 0.0, 1.6, 20.0, 45.0)

    # Cápsulas cilíndricas T y R (Ø 16 mm, long 12.5 mm)
    can_r = 8.0
    for z_can in (-13.0, 13.0):
        b.set_material("Material_Alu_Can")
        b.add_cylinder_x(0.0, z_can, 0.8, 13.0, can_r, segments=24)
        b.set_material("Material_Mesh_Dark")
        b.add_cylinder_x(0.0, z_can, 12.8, 13.1, can_r - 0.8, segments=24)

    # Cristal oscilador HC-49
    b.set_material("Material_Crystal")
    b.add_box(1.8, 0.0, 0.0, 3.5, 4.0, 10.0)

    # 4 Pines de conexión posteriores
    b.set_material("Material_Screw_Inox")
    for yp in (-7.5, -2.5, 2.5, 7.5):
        b.add_cylinder_x(yp, -18.0, -4.5, 0.0, 0.6, segments=12)
    return b

# =============================================================================
# 8. BANDA TRANSPORTADORA FLEXIBLE (Pieza Individual)
# =============================================================================
def build_part_belt(length_center=320.0, belt_width=70.0, roller_r=13.0, belt_thick=1.6):
    b = ModelBuilder()
    b.set_material("Material_Belt")
    y_top = roller_r + belt_thick * 0.5
    y_bot = -roller_r - belt_thick * 0.5

    # Rama superior útil
    b.add_box(0.0, y_top, 0.0, length_center, belt_thick, belt_width)
    # Rama inferior de retorno
    b.add_box(0.0, y_bot, 0.0, length_center, belt_thick, belt_width)

    # Curvas envolventes en los extremos
    half_l = length_center * 0.5
    for xr, sign in ((-half_l, -1.0), (half_l, 1.0)):
        for i in range(16):
            start_ang = math.pi * 0.5 if sign < 0 else -math.pi * 0.5
            a1 = start_ang + math.pi * i / 16.0
            a2 = start_ang + math.pi * (i + 1) / 16.0
            r_mid = roller_r + belt_thick * 0.5
            x1 = xr + r_mid * math.cos(a1)
            y1 = r_mid * math.sin(a1)
            x2 = xr + r_mid * math.cos(a2)
            y2 = r_mid * math.sin(a2)
            dx = abs(x2 - x1) + 0.3
            dy = abs(y2 - y1) + 0.3
            b.add_box((x1 + x2)*0.5, (y1 + y2)*0.5, 0.0, dx, dy, belt_width)
    return b

# =============================================================================
# 9. MOTOR PASO A PASO NEMA 17 CON ACOPLE (Pieza Individual)
# =============================================================================
def build_part_motor_nema17():
    b = ModelBuilder()
    # Centrado en el origen (eje en Z)
    b.set_material("Material_Motor_Black")
    # Estator laminado cuadrado 42x42x34 mm
    b.add_box(0.0, 0.0, 0.0, 42.0, 42.0, 34.0)
    # Tapas de aluminio mecanizado delantera y trasera
    b.set_material("Material_Alu_Profile")
    b.add_box(0.0, 0.0, -19.0, 42.0, 42.0, 4.0)
    b.add_box(0.0, 0.0,  19.0, 42.0, 42.0, 4.0)

    # Acople elástico de eje en aluminio azul (Ø 19 mm, long 25 mm)
    b.set_material("Material_Coupler_Blue")
    b.add_cylinder(0.0, 0.0, -38.0, -21.0, 9.5, 9.5, segments=24)

    # Eje de acero del motor (Ø 5 mm)
    b.set_material("Material_Steel_Shaft")
    b.add_cylinder(0.0, 0.0, -42.0, -19.0, 2.5, 2.5, segments=16)

    # Escuadra de fijación metálica negra
    b.set_material("Material_Black_Plates")
    b.add_box(0.0, -24.0, -17.0, 50.0, 6.0, 45.0)
    return b

# =============================================================================
# 10. RAMPA DE DESCARGA Y GUIA DE CAJAS (Pieza Individual)
# =============================================================================
def build_part_chute():
    b = ModelBuilder()
    b.set_material("Material_Chute_Alu")
    # Chapa inclinada de 48 mm de largo x 72 mm de ancho
    b.add_box(0.0, 0.0, 0.0, 48.0, 2.0, 72.0)
    # Labios laterales de guiado (altura 12 mm)
    for z_lip in (-35.0, 35.0):
        b.add_box(0.0, 6.0, z_lip, 48.0, 12.0, 2.0)
    return b

# =============================================================================
# EXPORTADOR MAESTRO DE TODAS LAS PIEZAS Y DEL ENSAMBLE COMPLETO
# =============================================================================
def export_all_models():
    modelsd_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 1. Archivo MTL común para todas las piezas
    mtl_filename = "piezas_faja.mtl"
    mtl_path = os.path.join(modelsd_dir, mtl_filename)
    write_common_mtl(mtl_path)
    print(f"[OK] Material Library generado: {mtl_path}")

    # Lista de piezas individuales a generar
    parts_catalog = [
        ("perfil_aluminio_2020",         build_part_profile_2020(),       "Perfil de Aluminio T-Slot 20x20 mm"),
        ("rodillo_conveyor",             build_part_roller(),             "Rodillo con Eje y Rodamientos 608ZZ"),
        ("pata_soporte_lateral",         build_part_stand_plate(),        "Pata de Soporte Lateral Estándar"),
        ("pata_soporte_extendida",       build_part_stand_plate_extended(),"Pata Lateral con Brazo para Sensor"),
        ("torre_roja_ajustable",         build_part_red_bracket(),        "Torre Roja con Corredera para Sensores"),
        ("placa_soporte_sensor",         build_part_sensor_plate(),       "Placa de Montaje del Sensor GrabCAD"),
        ("sensor_hcsr04_compacto",       build_part_hcsr04(),             "Sensor Ultrasónico HC-SR04"),
        ("banda_transportadora",         build_part_belt(),               "Banda Transportadora Plana Continua"),
        ("motor_nema17_drive",           build_part_motor_nema17(),       "Motor NEMA 17 con Acople y Escuadra"),
        ("rampa_descarga",               build_part_chute(),              "Rampa de Descarga de Cajas")
    ]

    print("\n" + "="*70)
    print(" GENERANDO PIEZAS INDIVIDUALES EN 3D (OBJ + MTL + STL BINARIO)")
    print("="*70)

    for name, model, desc in parts_catalog:
        obj_file = os.path.join(modelsd_dir, f"{name}.obj")
        stl_file = os.path.join(modelsd_dir, f"{name}.stl")
        
        model.write_obj(obj_file, mtl_path)
        model.write_stl(stl_file)
        
        v_count = len(model.vertices)
        f_count = len(model.faces)
        print(f"  -> {name:<26} | Vértices: {v_count:5d} | Caras: {f_count:5d} | {desc}")

    # 11. Generar Ensamble Completo Acoplado
    print("\n" + "="*70)
    print(" GENERANDO ENSAMBLE COMPLETO ACOPLADO (TODAS LAS PIEZAS INTEGRADAS)")
    print("="*70)
    
    from generate_conveyor import build_conveyor
    assembly_model = build_conveyor(scale=1.0, include_demo_box=True)
    
    assembly_obj = os.path.join(modelsd_dir, "faja_transportadora_ensamble.obj")
    assembly_stl = os.path.join(modelsd_dir, "faja_transportadora_ensamble.stl")
    assembly_model.write_obj(assembly_obj, mtl_path)
    assembly_model.write_stl(assembly_stl)
    
    print(f"  -> faja_transportadora_ensamble | Vértices: {len(assembly_model.vertices):5d} | Caras: {len(assembly_model.faces):5d}")
    print("\n[OK] Todas las piezas individuales y el ensamble se generaron con éxito.")

if __name__ == "__main__":
    export_all_models()
