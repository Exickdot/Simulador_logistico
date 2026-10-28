import math
import os
import struct
import sys

# Asegurar path para importar ModelBuilder
sys.path.append(os.path.dirname(__file__))
from model_builder import ModelBuilder

def build_conveyor(scale=1.0, include_demo_box=True):
    """
    Genera el modelo 3D detallado de la faja transportadora ajustable de laboratorio
    (Referencia GrabCAD: Gabriele Gregorini) con la modificación correcta:
    - Altura del sensor idéntica al modelo original (apuntando horizontalmente a la cara frontal de las cajas).
    - Longitud extendida: soporte del sensor desplazado longitudinalmente hacia atrás (X = -232 mm),
      creando un espacio libre de descarga/caída entre la faja y la placa para que las cajas pasen sin chocar.
    """
    b = ModelBuilder(scale=scale)

    # =========================================================================
    # PARAMETROS MECANICOS GLOBALES (en milímetros)
    # =========================================================================
    length = 360.0             # Longitud de perfiles 2020 de la faja
    belt_width = 70.0          # Ancho de banda útil
    roller_r = 13.0            # Radio de rodillo (Ø 26 mm)
    roller_len = 74.0          # Ancho rodillo
    roller_y = 0.0             # Altura de los ejes de rodillos
    z_front_prof = -47.0       # Centro del perfil frontal 2020
    z_rear_prof = 47.0         # Centro del perfil trasero 2020
    prof_size = 20.0           # Perfil 20x20
    belt_thick = 1.6           # Espesor de la banda

    # =========================================================================
    # 1. PERFILES DE ALUMINIO EXTRUIDO T-SLOT 20x20 MM (CHASIS)
    # =========================================================================
    b.set_material("Material_Alu_Profile")
    for z_c in (z_front_prof, z_rear_prof):
        # Perfil principal de la faja
        b.add_box(0.0, roller_y, z_c, length, prof_size, prof_size)

    # =========================================================================
    # 2. RODILLOS ROJOS INDUSTRIALES Y EJES DE ACERO
    # =========================================================================
    # Rodillo 1 (Izquierdo / Retorno): X = -160.0
    # Rodillo 2 (Derecho / Motriz - con motor): X = 160.0
    b.set_material("Material_Roller_Red")
    b.add_cylinder(-160.0, roller_y, -roller_len * 0.5, roller_len * 0.5, roller_r, roller_r, segments=32)
    b.add_cylinder( 160.0, roller_y, -roller_len * 0.5, roller_len * 0.5, roller_r, roller_r, segments=32)

    # Coronas de guiado lateral de los rodillos
    flange_r = roller_r + 2.0
    for xr in (-160.0, 160.0):
        for zf in (-roller_len * 0.5, roller_len * 0.5 - 2.5):
            b.add_cylinder(xr, roller_y, zf, zf + 2.5, flange_r, flange_r, segments=32)

    # Ejes pasantes de acero inoxidable (Ø 8 mm)
    b.set_material("Material_Steel_Shaft")
    shaft_r = 4.0
    b.add_cylinder(-160.0, roller_y, -60.0, 60.0, shaft_r, shaft_r, segments=24)
    # Eje derecho extendido hacia el acople del motor (hasta Z = 82 mm)
    b.add_cylinder( 160.0, roller_y, -60.0, 82.0, shaft_r, shaft_r, segments=24)

    # Rodamientos 608ZZ (4 unidades)
    b.set_material("Material_Bearing_608")
    for xr in (-160.0, 160.0):
        for z_b in (-52.0, 52.0):
            b.add_cylinder(xr, roller_y, z_b - 3.5, z_b + 3.5, 11.0, 11.0, segments=24)
            b.set_material("Material_Bearing_Seal")
            b.add_cylinder(xr, roller_y, z_b - 3.6, z_b + 3.6, 8.5, 8.5, segments=24)
            b.set_material("Material_Bearing_608")

    # =========================================================================
    # 3. BANDA TRANSPORTADORA FLEXIBLE (AZUL-GRIS MATE)
    # =========================================================================
    b.set_material("Material_Belt")
    y_belt_top = roller_y + roller_r + belt_thick * 0.5
    y_belt_bot = roller_y - roller_r - belt_thick * 0.5
    b.add_box(0.0, y_belt_top, 0.0, 320.0, belt_thick, belt_width)
    b.add_box(0.0, y_belt_bot, 0.0, 320.0, belt_thick, belt_width)

    # Curvaturas semicilíndricas envolviendo los rodillos
    for i in range(16):
        a1 = math.pi * 0.5 + math.pi * i / 16.0
        a2 = math.pi * 0.5 + math.pi * (i + 1) / 16.0
        r_mid = roller_r + belt_thick * 0.5
        x1 = -160.0 + r_mid * math.cos(a1)
        y1 = roller_y + r_mid * math.sin(a1)
        x2 = -160.0 + r_mid * math.cos(a2)
        y2 = roller_y + r_mid * math.sin(a2)
        dx = abs(x2 - x1) + 0.3
        dy = abs(y2 - y1) + 0.3
        b.add_box((x1 + x2)*0.5, (y1 + y2)*0.5, 0.0, dx, dy, belt_width)

    for i in range(16):
        a1 = -math.pi * 0.5 + math.pi * i / 16.0
        a2 = -math.pi * 0.5 + math.pi * (i + 1) / 16.0
        r_mid = roller_r + belt_thick * 0.5
        x1 = 160.0 + r_mid * math.cos(a1)
        y1 = roller_y + r_mid * math.sin(a1)
        x2 = 160.0 + r_mid * math.cos(a2)
        y2 = roller_y + r_mid * math.sin(a2)
        dx = abs(x2 - x1) + 0.3
        dy = abs(y2 - y1) + 0.3
        b.add_box((x1 + x2)*0.5, (y1 + y2)*0.5, 0.0, dx, dy, belt_width)

    # =========================================================================
    # 4. PATAS DE SOPORTE Y PLACAS LATERALES (NEGRO SATINADO / CORTE LASER)
    # =========================================================================
    b.set_material("Material_Black_Plates")
    plate_thick = 6.0

    # Patas traseras derechas (X = 140)
    for z_p in (-52.0, 52.0):
        b.add_box(152.0, 0.0, z_p, 65.0, 36.0, plate_thick)
        b.add_box(148.0, -42.0, z_p, 50.0, 48.0, plate_thick)
        # Tornillos Allen M5
        b.set_material("Material_Screw_Inox")
        for x_sc in (130.0, 158.0):
            z_head = z_p - 3.5 if z_p < 0 else z_p + 3.5
            b.add_cylinder(x_sc, 0.0, z_head - 2.5, z_head + 2.5, 4.5, 4.5, segments=16)
        b.set_material("Material_Black_Plates")

    # Patas delanteras izquierdas (X = -140) con EXTENSIÓN LONGITUDINAL HACIA ATRÁS:
    # Las placas se extienden longitudinalmente hacia X = -232 mm para sostener la placa del sensor
    # a la altura original, creando el espacio libre de caída entre X = -174 y X = -225!
    for z_p in (-52.0, 52.0):
        # Cuerpo principal de la pata izquierda
        b.add_box(-152.0, 0.0, z_p, 65.0, 36.0, plate_thick)
        b.add_box(-148.0, -42.0, z_p, 50.0, 48.0, plate_thick)
        
        # BRAZO DE EXTENSIÓN LONGITUDINAL (de X = -160 mm hasta X = -232 mm, longitud = 72 mm):
        # Permite alejar la placa del sensor en longitud sin elevar su altura
        b.add_box(-196.0, 2.0, z_p, 72.0, 24.0, plate_thick)

        # Tornillos de sujeción
        b.set_material("Material_Screw_Inox")
        for x_sc in (-130.0, -158.0):
            z_head = z_p - 3.5 if z_p < 0 else z_p + 3.5
            b.add_cylinder(x_sc, 0.0, z_head - 2.5, z_head + 2.5, 4.5, 4.5, segments=16)
        # Tornillos en la unión de la placa del sensor al extremo del brazo (X = -230)
        z_head = z_p - 3.5 if z_p < 0 else z_p + 3.5
        b.add_cylinder(-228.0, 12.0, z_head - 2.5, z_head + 2.5, 4.5, 4.5, segments=16)
        b.add_cylinder(-228.0, -6.0, z_head - 2.5, z_head + 2.5, 4.5, 4.5, segments=16)
        b.set_material("Material_Black_Plates")

    # =========================================================================
    # 5. TORRES ROJAS AJUSTABLES DE SENSORES / GUIAS (MODULARES T-SLOT)
    # =========================================================================
    b.set_material("Material_Red_Brackets")
    for z_br in (z_front_prof, z_rear_prof):
        b.add_box(-60.0, 14.0, z_br, 28.0, 8.0, 22.0)
        b.add_box(-70.0, 46.5, z_br, 8.0, 57.0, 8.0)
        b.add_box(-50.0, 46.5, z_br, 8.0, 57.0, 8.0)
        b.add_box(-60.0, 71.0, z_br, 28.0, 8.0, 8.0)
        b.add_box(-60.0, 22.0, z_br, 28.0, 8.0, 8.0)
        b.set_material("Material_Screw_Inox")
        b.add_cylinder(-68.0, 19.0, z_br - 1.5, z_br + 1.5, 4.0, 4.0, segments=16)
        b.add_cylinder(-52.0, 19.0, z_br - 1.5, z_br + 1.5, 4.0, 4.0, segments=16)
        b.set_material("Material_Red_Brackets")

    # =========================================================================
    # 6. PLACA DEL SENSOR ORIGINAL DESPLAZADA EN LONGITUD (X = -232 MM)
    # =========================================================================
    # Altura del sensor: Y = 26.0 mm (Alineado perfectamente al centro de las cajas en la faja)
    # Posición X: X = -232.0 mm (Separado 58 mm del final de la banda en X = -174.0 mm)
    # Espacio libre de paso/caída de cajas: Ancho 78 mm x Largo 58 mm
    x_sensor_plate = -232.0
    plate_thick_s = 4.0

    b.set_material("Material_Sensor_Plate_GrabCAD")
    # Placa vertical metálica de montaje idéntica a GrabCAD:
    # Ancho total en Z: 108 mm (de Z = -54 a +54 mm)
    # Altura en Y: de Y = -15.0 a +52.0 mm (67 mm alto)
    b.add_box(x_sensor_plate, 18.5, 0.0, plate_thick_s, 67.0, 108.0)

    # Ventanillas laterales con ranuras estéticas/funcionales (fieles a GrabCAD):
    b.set_material("Material_Screw_Inox")
    for z_hole in (-46.0, 46.0):
        for y_hole in (-8.0, 5.0, 18.0, 32.0, 44.0):
            b.add_cylinder_x(y_hole, z_hole, x_sensor_plate - 3.0, x_sensor_plate + 3.0, 2.8, segments=16)

    # =========================================================================
    # 7. SENSOR ULTRASONICO HC-SR04 ALINEADO HORIZONTALMENTE A LAS CAJAS
    # =========================================================================
    # Montado en el centro de la placa a Y = 26.0 mm, Z = 0.0 mm
    # Las cápsulas ultrasónicas (T y R) apuntan horizontalmente hacia +X
    # Inciden a 90° normal en la cara frontal vertical de la caja, garantizando
    # que la onda emitida a 40 kHz rebote directamente a la cápsula receptora R!
    sensor_y = 26.0
    pcb_x = x_sensor_plate + 3.5

    b.set_material("Material_PCB_Blue")
    # PCB Azul 45x20 mm montada en la placa
    b.add_box(pcb_x, sensor_y, 0.0, 1.6, 20.0, 45.0)

    # Transductores ultrasónicos cilíndricos de aluminio (Ø 16 mm, long 12.5 mm)
    # Separación entre ejes: 26 mm (Z = -13.0 y Z = +13.0)
    can_r = 8.0
    for z_can in (-13.0, 13.0):
        b.set_material("Material_Alu_Can")
        b.add_cylinder_x(sensor_y, z_can, pcb_x + 0.8, pcb_x + 13.0, can_r, segments=24)
        # Malla frontal oscura emisora/receptora
        b.set_material("Material_Mesh_Dark")
        b.add_cylinder_x(sensor_y, z_can, pcb_x + 12.8, pcb_x + 13.1, can_r - 0.8, segments=24)

    # Cristal oscilador HC-49 metálico
    b.set_material("Material_Crystal")
    b.add_box(pcb_x + 2.0, sensor_y, 0.0, 3.5, 4.0, 10.0)

    # Pines posteriores de conexión
    b.set_material("Material_Screw_Inox")
    for yp in (sensor_y - 7.0, sensor_y - 2.5, sensor_y + 2.5, sensor_y + 7.0):
        b.add_cylinder_x(yp, -18.0, pcb_x - 4.0, pcb_x, 0.6, segments=12)

    # =========================================================================
    # 8. RAMPA / GUIA INFERIOR DE DESCARGA DE CAJAS EN EL ESPACIO LIBRE
    # =========================================================================
    # En el hueco de 58 mm entre el rodillo (X = -174) y el sensor (X = -232):
    # Rampa de descarga que recibe la caja y la desliza suavemente hacia abajo/afuera
    b.set_material("Material_Chute_Alu")
    ramp_cx = (-174.0 + -224.0) * 0.5
    ramp_cy = (10.0 + -35.0) * 0.5
    b.add_box(ramp_cx, ramp_cy, 0.0, 48.0, 2.0, 72.0)
    # Labios laterales de guiado de la rampa
    for z_lip in (-35.0, 35.0):
        b.add_box(ramp_cx, ramp_cy + 6.0, z_lip, 48.0, 12.0, 2.0)

    # =========================================================================
    # 9. MOTOR DE TRACCION NEMA 17 Y ACOPLE (EXTREMO DERECHO TRASERO)
    # =========================================================================
    b.set_material("Material_Motor_Black")
    b.add_box(160.0, roller_y, 102.0, 42.0, 42.0, 34.0)
    b.set_material("Material_Alu_Profile")
    b.add_box(160.0, roller_y, 83.0, 42.0, 42.0, 4.0)
    b.add_box(160.0, roller_y, 121.0, 42.0, 42.0, 4.0)

    b.set_material("Material_Coupler_Blue")
    b.add_cylinder(160.0, roller_y, 65.0, 81.0, 9.5, 9.5, segments=24)

    b.set_material("Material_Black_Plates")
    b.add_box(160.0, -15.0, 85.0, 50.0, 6.0, 45.0)

    # =========================================================================
    # 10. CAJAS DE DEMOSTRACION (VERIFICACION DE FISICA DEL SENSOR Y PASO)
    # =========================================================================
    if include_demo_box:
        # Caja 1: Sobre la faja, aproximándose al sensor (a X = -105 mm)
        # Su cara frontal vertical está perfectamente perpendicular al haz del sensor!
        b.set_material("Material_Demo_Box")
        box1_y = y_belt_top + 15.0 # Centro en Y = 29.6 mm (frente al sensor en Y = 26.0)
        b.add_box(-105.0, box1_y, 0.0, 32.0, 30.0, 32.0)
        b.set_material("Material_Tape")
        b.add_box(-105.0, box1_y + 15.1, 0.0, 32.2, 0.4, 8.0)

        # Caja 2: Cayendo/saliendo por la rampa en el espacio libre (a X = -196 mm, Y = -10.0 mm)
        # Muestra claramente cómo la caja pasa sin tocar la placa del sensor!
        b.set_material("Material_Demo_Box_Exit")
        b.add_box(-196.0, -10.0, 0.0, 32.0, 30.0, 32.0)

        # Líneas de Haz Ultrasónico representadas sutilmente (demostración física)
        b.set_material("Material_Acoustic_Beam")
        # Cono sonoro entre la caja 1 (X = -121) y el sensor (X = -218)
        b.add_cylinder_x(sensor_y, -13.0, -218.0, -121.0, 0.6, segments=8)
        b.add_cylinder_x(sensor_y,  13.0, -218.0, -121.0, 0.6, segments=8)

    return b

def write_mtl(filename):
    """Escribe los materiales PBR realistas en el archivo MTL."""
    with open(filename, 'w', encoding='utf-8') as f:
        f.write("# Material Library for Modular Educational Conveyor Belt (GrabCAD Extended Length)\n\n")

        # Perfil Aluminio
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

        # Banda transportadora azul-gris mate
        f.write("newmtl Material_Belt\n")
        f.write("Kd 0.32 0.40 0.50\nKs 0.12 0.14 0.18\nNs 15.0\nKa 0.15 0.18 0.22\n\n")

        # Placas Negras Satinadas
        f.write("newmtl Material_Black_Plates\n")
        f.write("Kd 0.14 0.15 0.17\nKs 0.35 0.35 0.40\nNs 45.0\nKa 0.10 0.10 0.12\n\n")

        # Torres Rojas Ajustables
        f.write("newmtl Material_Red_Brackets\n")
        f.write("Kd 0.85 0.12 0.10\nKs 0.50 0.25 0.25\nNs 55.0\nKa 0.28 0.08 0.08\n\n")

        # Placa Original GrabCAD del Sensor
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

        # Caja en Faja
        f.write("newmtl Material_Demo_Box\n")
        f.write("Kd 0.78 0.58 0.38\nKs 0.15 0.12 0.10\nNs 12.0\nKa 0.32 0.24 0.16\n\n")

        # Caja en Salida / Caída
        f.write("newmtl Material_Demo_Box_Exit\n")
        f.write("Kd 0.68 0.50 0.32\nKs 0.12 0.10 0.08\nNs 10.0\nKa 0.28 0.20 0.12\n\n")

        f.write("newmtl Material_Tape\n")
        f.write("Kd 0.88 0.78 0.52\nKs 0.25 0.25 0.20\nNs 20.0\nKa 0.35 0.30 0.22\n\n")

        # Haz Acústico (Cyan brillante)
        f.write("newmtl Material_Acoustic_Beam\n")
        f.write("Kd 0.10 0.85 0.95\nKs 0.90 0.90 0.95\nNs 80.0\nKa 0.10 0.50 0.60\n\n")

def main():
    modelsd_dir = os.path.dirname(os.path.abspath(__file__))
    obj_path = os.path.join(modelsd_dir, "faja_transportadora_ajustable.obj")
    mtl_path = os.path.join(modelsd_dir, "faja_transportadora_ajustable.mtl")
    stl_path = os.path.join(modelsd_dir, "faja_transportadora_ajustable.stl")
    
    print("Construyendo modelo 3D con extensión de longitud y sensor a cota horizontal original...")
    conveyor = build_conveyor(scale=1.0, include_demo_box=True)
    
    print(f"Escribiendo MTL: {mtl_path}...")
    write_mtl(mtl_path)
    
    print(f"Escribiendo OBJ: {obj_path}...")
    conveyor.write_obj(obj_path, mtl_path)
    
    print(f"Escribiendo STL: {stl_path}...")
    conveyor.write_stl(stl_path)
    
    print("Generación completada:")
    print(f"  Vértices: {len(conveyor.vertices)}")
    print(f"  Caras:    {len(conveyor.faces)}")

if __name__ == "__main__":
    main()
