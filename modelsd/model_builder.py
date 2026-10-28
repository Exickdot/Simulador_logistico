import math
import os
import struct

class ModelBuilder:
    def __init__(self, scale=1.0):
        self.scale = scale
        self.vertices = []
        self.normals = []
        self.texcoords = []
        # faces: list of (material, [(v_idx, vt_idx, vn_idx), ...])
        self.faces = []
        self.current_material = "default"

    def set_material(self, mat_name):
        self.current_material = mat_name

    def add_vertex(self, x, y, z):
        self.vertices.append((x * self.scale, y * self.scale, z * self.scale))
        return len(self.vertices)

    def add_normal(self, nx, ny, nz):
        l = math.sqrt(nx*nx + ny*ny + nz*nz)
        if l > 1e-6:
            nx /= l
            ny /= l
            nz /= l
        else:
            nx, ny, nz = 0.0, 0.0, 1.0
        self.normals.append((nx, ny, nz))
        return len(self.normals)

    def add_triangle(self, v1, v2, v3, n1=None, n2=None, n3=None):
        if n1 is None:
            # Calcular normal por producto vectorial
            p1 = self.vertices[v1 - 1]
            p2 = self.vertices[v2 - 1]
            p3 = self.vertices[v3 - 1]
            ux, uy, uz = p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
            vx, vy, vz = p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]
            nx = uy * vz - uz * vy
            ny = uz * vx - ux * vz
            nz = ux * vy - uy * vx
            n_idx = self.add_normal(nx, ny, nz)
            n1 = n2 = n3 = n_idx
        self.faces.append((self.current_material, [(v1, None, n1), (v2, None, n2), (v3, None, n3)]))

    def add_quad(self, v1, v2, v3, v4, n1=None, n2=None, n3=None, n4=None):
        if n1 is None:
            p1 = self.vertices[v1 - 1]
            p2 = self.vertices[v2 - 1]
            p3 = self.vertices[v3 - 1]
            ux, uy, uz = p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
            vx, vy, vz = p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]
            nx = uy * vz - uz * vy
            ny = uz * vx - ux * vz
            nz = ux * vy - uy * vx
            n_idx = self.add_normal(nx, ny, nz)
            n1 = n2 = n3 = n4 = n_idx
        self.faces.append((self.current_material, [(v1, None, n1), (v2, None, n2), (v3, None, n3)]))
        self.faces.append((self.current_material, [(v1, None, n1), (v3, None, n3), (v4, None, n4)]))

    def add_box(self, cx, cy, cz, dx, dy, dz):
        hx, hy, hz = dx * 0.5, dy * 0.5, dz * 0.5
        # 8 vértices
        v1 = self.add_vertex(cx - hx, cy - hy, cz + hz)
        v2 = self.add_vertex(cx + hx, cy - hy, cz + hz)
        v3 = self.add_vertex(cx + hx, cy + hy, cz + hz)
        v4 = self.add_vertex(cx - hx, cy + hy, cz + hz)
        v5 = self.add_vertex(cx - hx, cy - hy, cz - hz)
        v6 = self.add_vertex(cx + hx, cy - hy, cz - hz)
        v7 = self.add_vertex(cx + hx, cy + hy, cz - hz)
        v8 = self.add_vertex(cx - hx, cy + hy, cz - hz)

        # 6 caras
        nz_p = self.add_normal(0, 0, 1)
        self.add_quad(v1, v2, v3, v4, nz_p, nz_p, nz_p, nz_p) # +Z
        nz_m = self.add_normal(0, 0, -1)
        self.add_quad(v6, v5, v8, v7, nz_m, nz_m, nz_m, nz_m) # -Z
        ny_p = self.add_normal(0, 1, 0)
        self.add_quad(v4, v3, v7, v8, ny_p, ny_p, ny_p, ny_p) # +Y
        ny_m = self.add_normal(0, -1, 0)
        self.add_quad(v5, v6, v2, v1, ny_m, ny_m, ny_m, ny_m) # -Y
        nx_p = self.add_normal(1, 0, 0)
        self.add_quad(v2, v6, v7, v3, nx_p, nx_p, nx_p, nx_p) # +X
        nx_m = self.add_normal(-1, 0, 0)
        self.add_quad(v5, v1, v4, v8, nx_m, nx_m, nx_m, nx_m) # -X

    def add_cylinder(self, cx, cy, z_bottom, z_top, r_bottom, r_top, segments=32, capped=True):
        bot_ring = []
        top_ring = []
        normals_ring = []

        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)

            vx_b = cx + r_bottom * cos_a
            vy_b = cy + r_bottom * sin_a
            bot_ring.append(self.add_vertex(vx_b, vy_b, z_bottom))

            vx_t = cx + r_top * cos_a
            vy_t = cy + r_top * sin_a
            top_ring.append(self.add_vertex(vx_t, vy_t, z_top))

            # Normal cónica
            dr = r_bottom - r_top
            dz = z_top - z_bottom
            normals_ring.append(self.add_normal(cos_a * dz, sin_a * dz, dr))

        # Caras laterales
        for i in range(segments):
            next_i = (i + 1) % segments
            b1 = bot_ring[i]
            b2 = bot_ring[next_i]
            t1 = top_ring[i]
            t2 = top_ring[next_i]
            n1 = normals_ring[i]
            n2 = normals_ring[next_i]
            self.add_quad(b1, b2, t2, t1, n1, n2, n2, n1)

        # Tapas si aplica
        if capped:
            # Tapa inferior (-Z)
            c_bot = self.add_vertex(cx, cy, z_bottom)
            n_bot = self.add_normal(0, 0, -1)
            for i in range(segments):
                next_i = (i + 1) % segments
                self.add_triangle(c_bot, bot_ring[next_i], bot_ring[i], n_bot, n_bot, n_bot)

            # Tapa superior (+Z)
            c_top = self.add_vertex(cx, cy, z_top)
            n_top = self.add_normal(0, 0, 1)
            for i in range(segments):
                next_i = (i + 1) % segments
                self.add_triangle(c_top, top_ring[i], top_ring[next_i], n_top, n_top, n_top)

    def add_hollow_cylinder(self, cx, cy, z_bottom, z_top, r_outer, r_inner, segments=32):
        out_b = []
        out_t = []
        in_b = []
        in_t = []
        out_norm = []
        in_norm = []

        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            cos_a = math.cos(angle)
            sin_a = math.sin(angle)

            out_b.append(self.add_vertex(cx + r_outer * cos_a, cy + r_outer * sin_a, z_bottom))
            out_t.append(self.add_vertex(cx + r_outer * cos_a, cy + r_outer * sin_a, z_top))
            in_b.append(self.add_vertex(cx + r_inner * cos_a, cy + r_inner * sin_a, z_bottom))
            in_t.append(self.add_vertex(cx + r_inner * cos_a, cy + r_inner * sin_a, z_top))

            out_norm.append(self.add_normal(cos_a, sin_a, 0.0))
            in_norm.append(self.add_normal(-cos_a, -sin_a, 0.0))

        # Pared exterior
        for i in range(segments):
            next_i = (i + 1) % segments
            self.add_quad(out_b[i], out_b[next_i], out_t[next_i], out_t[i],
                          out_norm[i], out_norm[next_i], out_norm[next_i], out_norm[i])

        # Pared interior
        for i in range(segments):
            next_i = (i + 1) % segments
            self.add_quad(in_b[next_i], in_b[i], in_t[i], in_t[next_i],
                          in_norm[next_i], in_norm[i], in_norm[i], in_norm[next_i])

        # Aro superior
        n_up = self.add_normal(0, 0, 1)
        for i in range(segments):
            next_i = (i + 1) % segments
            self.add_quad(out_t[i], out_t[next_i], in_t[next_i], in_t[i],
                          n_up, n_up, n_up, n_up)

    def add_cylinder_x(self, cy, cz, x_start, x_end, radius, segments=24, capped=True):
        r_start = []
        r_end = []
        normals = []
        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            ca, sa = math.cos(angle), math.sin(angle)
            r_start.append(self.add_vertex(x_start, cy + radius * ca, cz + radius * sa))
            r_end.append(self.add_vertex(x_end, cy + radius * ca, cz + radius * sa))
            normals.append(self.add_normal(0.0, ca, sa))
        for i in range(segments):
            next_i = (i + 1) % segments
            self.add_quad(r_start[i], r_start[next_i], r_end[next_i], r_end[i],
                          normals[i], normals[next_i], normals[next_i], normals[i])
        if capped:
            c1 = self.add_vertex(x_start, cy, cz)
            n1 = self.add_normal(-1.0, 0.0, 0.0)
            c2 = self.add_vertex(x_end, cy, cz)
            n2 = self.add_normal(1.0, 0.0, 0.0)
            for i in range(segments):
                next_i = (i + 1) % segments
                self.add_triangle(c1, r_start[next_i], r_start[i], n1, n1, n1)
                self.add_triangle(c2, r_end[i], r_end[next_i], n2, n2, n2)

    def add_cylinder_y(self, cx, cz, y_start, y_end, radius, segments=24, capped=True):
        r_start = []
        r_end = []
        normals = []
        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            ca, sa = math.cos(angle), math.sin(angle)
            r_start.append(self.add_vertex(cx + radius * ca, y_start, cz + radius * sa))
            r_end.append(self.add_vertex(cx + radius * ca, y_end, cz + radius * sa))
            normals.append(self.add_normal(ca, 0.0, sa))
        for i in range(segments):
            next_i = (i + 1) % segments
            self.add_quad(r_start[i], r_start[next_i], r_end[next_i], r_end[i],
                          normals[i], normals[next_i], normals[next_i], normals[i])
        if capped:
            c1 = self.add_vertex(cx, y_start, cz)
            n1 = self.add_normal(0.0, -1.0, 0.0)
            c2 = self.add_vertex(cx, y_end, cz)
            n2 = self.add_normal(0.0, 1.0, 0.0)
            for i in range(segments):
                next_i = (i + 1) % segments
                self.add_triangle(c1, r_start[next_i], r_start[i], n1, n1, n1)
                self.add_triangle(c2, r_end[i], r_end[next_i], n2, n2, n2)

    def write_obj(self, filename, mtl_filename=None):
        with open(filename, 'w', encoding='utf-8') as f:
            f.write("# 3D Model: HC-SR04 Ultrasonic Sensor\n")
            f.write("# High-Precision CAD Model (Reference: GrabCAD / Octavi Masià)\n")
            if mtl_filename:
                f.write(f"mtllib {os.path.basename(mtl_filename)}\n")
            f.write("\n")

            for v in self.vertices:
                f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
            f.write("\n")

            for vn in self.normals:
                f.write(f"vn {vn[0]:.6f} {vn[1]:.6f} {vn[2]:.6f}\n")
            f.write("\n")

            current_mat = None
            for mat, face in self.faces:
                if mat != current_mat:
                    current_mat = mat
                    f.write(f"usemtl {current_mat}\n")
                if len(face) == 3:
                    f.write(f"f {face[0][0]}//{face[0][2]} {face[1][0]}//{face[1][2]} {face[2][0]}//{face[2][2]}\n")
                elif len(face) == 4:
                    f.write(f"f {face[0][0]}//{face[0][2]} {face[1][0]}//{face[1][2]} {face[2][0]}//{face[2][2]} {face[3][0]}//{face[3][2]}\n")

    def write_stl(self, filename):
        # Binary STL format
        triangles = []
        for mat, face in self.faces:
            if len(face) == 3:
                triangles.append((face[0][0], face[1][0], face[2][0]))
            elif len(face) == 4:
                triangles.append((face[0][0], face[1][0], face[2][0]))
                triangles.append((face[0][0], face[2][0], face[3][0]))

        header = b'HC-SR04 Ultrasonic Sensor 3D CAD Binary STL - GrabCAD Reference'
        header = header.ljust(80, b'\0')

        with open(filename, 'wb') as f:
            f.write(header)
            f.write(struct.pack('<I', len(triangles)))
            for v1_i, v2_i, v3_i in triangles:
                p1 = self.vertices[v1_i - 1]
                p2 = self.vertices[v2_i - 1]
                p3 = self.vertices[v3_i - 1]

                # Normal
                ux, uy, uz = p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]
                vx, vy, vz = p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]
                nx = uy * vz - uz * vy
                ny = uz * vx - ux * vz
                nz = ux * vy - uy * vx
                l = math.sqrt(nx*nx + ny*ny + nz*nz)
                if l > 1e-6:
                    nx /= l
                    ny /= l
                    nz /= l
                else:
                    nx, ny, nz = 0.0, 0.0, 1.0

                f.write(struct.pack('<3f', nx, ny, nz))
                f.write(struct.pack('<3f', p1[0], p1[1], p1[2]))
                f.write(struct.pack('<3f', p2[0], p2[1], p2[2]))
                f.write(struct.pack('<3f', p3[0], p3[1], p3[2]))
                f.write(struct.pack('<H', 0))

print("ModelBuilder class defined successfully.")
