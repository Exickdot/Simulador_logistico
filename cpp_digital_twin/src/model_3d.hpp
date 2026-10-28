// =====================================================================================
// PROYECTO: ESTACIÓN LOGÍSTICA DE SELECCIÓN, CLASIFICACIÓN Y ENRUTAMIENTO INDUSTRIAL
// GEMELO DIGITAL NATIVO C++ (OPENGL / WIN32) - MODELO GEOMÉTRICO 3D PULIDO
// =====================================================================================

#ifndef MODEL_3D_HPP
#define MODEL_3D_HPP

#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <cmath>
#include "plant_simulation.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class Primitives3D {
public:
    static void DrawBox(float x, float y, float z, float dx, float dy, float dz, float r, float g, float b, float a = 1.0f) {
        glColor4f(r, g, b, a);
        float hx = dx * 0.5f, hy = dy * 0.5f, hz = dz * 0.5f;

        glBegin(GL_QUADS);
        // +Z
        glNormal3f(0.0f, 0.0f, 1.0f);
        glVertex3f(x - hx, y - hy, z + hz);
        glVertex3f(x + hx, y - hy, z + hz);
        glVertex3f(x + hx, y + hy, z + hz);
        glVertex3f(x - hx, y + hy, z + hz);
        // -Z
        glNormal3f(0.0f, 0.0f, -1.0f);
        glVertex3f(x + hx, y - hy, z - hz);
        glVertex3f(x - hx, y - hy, z - hz);
        glVertex3f(x - hx, y + hy, z - hz);
        glVertex3f(x + hx, y + hy, z - hz);
        // +Y
        glNormal3f(0.0f, 1.0f, 0.0f);
        glVertex3f(x - hx, y + hy, z - hz);
        glVertex3f(x - hx, y + hy, z + hz);
        glVertex3f(x + hx, y + hy, z + hz);
        glVertex3f(x + hx, y + hy, z - hz);
        // -Y
        glNormal3f(0.0f, -1.0f, 0.0f);
        glVertex3f(x - hx, y - hy, z + hz);
        glVertex3f(x - hx, y - hy, z - hz);
        glVertex3f(x + hx, y - hy, z - hz);
        glVertex3f(x + hx, y - hy, z + hz);
        // +X
        glNormal3f(1.0f, 0.0f, 0.0f);
        glVertex3f(x + hx, y - hy, z - hz);
        glVertex3f(x + hx, y + hy, z - hz);
        glVertex3f(x + hx, y + hy, z + hz);
        glVertex3f(x + hx, y - hy, z + hz);
        // -X
        glNormal3f(-1.0f, 0.0f, 0.0f);
        glVertex3f(x - hx, y - hy, z + hz);
        glVertex3f(x - hx, y + hy, z + hz);
        glVertex3f(x - hx, y + hy, z - hz);
        glVertex3f(x - hx, y - hy, z - hz);
        glEnd();
    }

    static void DrawCylinder(float x, float y, float z, float radius, float height, float r, float g, float b, char axis = 'y', int segs = 16) {
        glColor3f(r, g, b);
        glPushMatrix();
        glTranslatef(x, y, z);
        if (axis == 'x') glRotatef(90.0f, 0, 0, 1);
        else if (axis == 'z') glRotatef(90.0f, 1, 0, 0);

        float hh = height * 0.5f;
        glBegin(GL_QUAD_STRIP);
        for (int i = 0; i <= segs; i++) {
            float angle = 2.0f * (float)M_PI * (float)i / (float)segs;
            float nx = std::cos(angle);
            float nz = std::sin(angle);
            glNormal3f(nx, 0.0f, nz);
            glVertex3f(nx * radius, -hh, nz * radius);
            glVertex3f(nx * radius, hh, nz * radius);
        }
        glEnd();

        // Tapas
        for (float sign : {-hh, hh}) {
            glBegin(GL_TRIANGLE_FAN);
            glNormal3f(0.0f, sign > 0 ? 1.0f : -1.0f, 0.0f);
            glVertex3f(0.0f, sign, 0.0f);
            for (int i = 0; i <= segs; i++) {
                float angle = 2.0f * (float)M_PI * (float)i / (float)segs * (sign > 0 ? 1.0f : -1.0f);
                glVertex3f(std::cos(angle) * radius, sign, std::sin(angle) * radius);
            }
            glEnd();
        }
        glPopMatrix();
    }
};

class Station3DRenderer {
public:
    float BeltOffset;

    Station3DRenderer() : BeltOffset(0.0f) {}

    void Render(PlantSimulation& sim, float dt) {
        BeltOffset = std::fmod(BeltOffset + sim.BeltSpeedMps * dt, 0.04f);

        // =========================================================================
        // 1. FASE OPACA (glDepthMask = GL_TRUE): Toda la geometría física sólida
        // =========================================================================
        glDepthMask(GL_TRUE);
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);

        DrawFloor();
        DrawFrame();
        DrawFeedHopper();            // Tolva de alimentación en la entrada de la faja
        DrawHardwareSensors(sim.LightPwm);
        DrawDischargeLip();          // Labio y rampa inoxidable de descarga directa
        DrawMotorizedContainerShuttle(sim.CartZ); // Carro móvil transversal M-102 con 3 contenedores
        DrawPedestal(sim.State, sim.EmergencyStop);

        // DIBUJAR CAJAS FÍSICAS
        DrawBoxes(sim);

        // =========================================================================
        // 2. FASE TRANSLÚCIDA (glDepthMask = GL_FALSE): Haces de luz y sonar sin gris
        // =========================================================================
        DrawTranslucentEffects(sim);
    }

private:
    void DrawFloor() {
        // Suelo técnico industrial
        Primitives3D::DrawBox(0.0f, -0.01f, 0.0f, 3.4f, 0.02f, 2.6f, 0.90f, 0.92f, 0.94f);

        // Cuadrícula métrica
        glDisable(GL_LIGHTING);
        glColor3f(0.80f, 0.82f, 0.85f);
        glLineWidth(1.0f);
        glBegin(GL_LINES);
        for (int i = -17; i <= 17; i++) {
            glVertex3f(i * 0.1f, 0.001f, -1.3f);
            glVertex3f(i * 0.1f, 0.001f, 1.3f);
            glVertex3f(-1.7f, 0.001f, i * 0.1f);
            glVertex3f(1.7f, 0.001f, i * 0.1f);
        }
        glEnd();

        // Recuadro de seguridad amarillo
        glColor3f(0.92f, 0.78f, 0.15f);
        glLineWidth(3.0f);
        glBegin(GL_LINE_LOOP);
        glVertex3f(-0.75f, 0.002f, -0.58f);
        glVertex3f(0.95f, 0.002f, -0.58f);
        glVertex3f(0.95f, 0.002f, 0.58f);
        glVertex3f(-0.75f, 0.002f, 0.58f);
        glEnd();
        glEnable(GL_LIGHTING);
    }

    void DrawFrame() {
        float aluR = 0.78f, aluG = 0.80f, aluB = 0.83f;

        // 4 Patas estructurales Bosch Rexroth 40x40
        for (float px : {-0.45f, 0.45f}) {
            for (float pz : {-0.09f, 0.09f}) {
                Primitives3D::DrawBox(px, 0.20f, pz, 0.04f, 0.40f, 0.04f, aluR, aluG, aluB);
                Primitives3D::DrawBox(px, 0.01f, pz, 0.06f, 0.02f, 0.06f, 0.12f, 0.12f, 0.12f);
            }
        }

        // Rieles portadores de faja
        Primitives3D::DrawBox(0.0f, 0.38f, -0.095f, 1.02f, 0.04f, 0.03f, aluR, aluG, aluB);
        Primitives3D::DrawBox(0.0f, 0.38f, 0.095f, 1.02f, 0.04f, 0.03f, aluR, aluG, aluB);

        // Rodillos de tracción y retorno
        Primitives3D::DrawCylinder(0.50f, 0.385f, 0.0f, 0.025f, 0.18f, 0.70f, 0.72f, 0.75f, 'z');
        Primitives3D::DrawCylinder(-0.50f, 0.385f, 0.0f, 0.025f, 0.18f, 0.70f, 0.72f, 0.75f, 'z');

        // Banda móvil de caucho negro
        Primitives3D::DrawBox(0.0f, 0.40f, 0.0f, 1.0f, 0.005f, 0.17f, 0.15f, 0.16f, 0.18f);

        // Nervaduras móviles
        glDisable(GL_LIGHTING);
        glColor3f(0.28f, 0.30f, 0.32f);
        glLineWidth(2.0f);
        glBegin(GL_LINES);
        for (int i = 0; i < 25; i++) {
            float rx = -0.48f + i * 0.04f + BeltOffset;
            if (rx >= -0.49f && rx <= 0.49f) {
                glVertex3f(rx, 0.403f, -0.075f);
                glVertex3f(rx, 0.403f, 0.075f);
            }
        }
        glEnd();
        glEnable(GL_LIGHTING);

        // Guías laterales inoxidables
        Primitives3D::DrawBox(-0.05f, 0.425f, -0.09f, 0.75f, 0.025f, 0.008f, 0.85f, 0.88f, 0.90f);
        Primitives3D::DrawBox(-0.05f, 0.425f, 0.09f, 0.75f, 0.025f, 0.008f, 0.85f, 0.88f, 0.90f);

        // Motor DC M-101
        Primitives3D::DrawCylinder(0.50f, 0.385f, -0.13f, 0.035f, 0.08f, 0.12f, 0.45f, 0.72f, 'z');
        Primitives3D::DrawBox(0.50f, 0.385f, -0.17f, 0.05f, 0.06f, 0.04f, 0.20f, 0.20f, 0.22f);
    }

    // =========================================================================
    // TOLVA / CONTENEDOR DE ALIMENTACIÓN EN LA ENTRADA DE LA FAJA (FEED HOPPER)
    // =========================================================================
    void DrawFeedHopper() {
        float hx = -0.46f;
        float hy = 0.62f;
        float aluR = 0.78f, aluG = 0.80f, aluB = 0.83f;

        // Postes verticales de soporte anclados a la estructura
        Primitives3D::DrawBox(hx, 0.54f, -0.095f, 0.025f, 0.28f, 0.025f, aluR, aluG, aluB);
        Primitives3D::DrawBox(hx, 0.54f, 0.095f, 0.025f, 0.28f, 0.025f, aluR, aluG, aluB);
        // Travesaño superior
        Primitives3D::DrawBox(hx, 0.70f, 0.0f, 0.03f, 0.025f, 0.22f, aluR, aluG, aluB);

        // Embudo / Tolva contenedor de almacenamiento (Chapa de acero con bordes)
        // Paredes del contenedor superior
        Primitives3D::DrawBox(hx, hy, -0.075f, 0.14f, 0.16f, 0.008f, 0.35f, 0.45f, 0.58f);
        Primitives3D::DrawBox(hx, hy, 0.075f, 0.14f, 0.16f, 0.008f, 0.35f, 0.45f, 0.58f);
        Primitives3D::DrawBox(hx - 0.065f, hy, 0.0f, 0.008f, 0.16f, 0.14f, 0.30f, 0.40f, 0.52f);
        Primitives3D::DrawBox(hx + 0.065f, hy, 0.0f, 0.008f, 0.16f, 0.14f, 0.30f, 0.40f, 0.52f);

        // Boca de descarga inferior acampanada
        Primitives3D::DrawBox(hx, 0.52f, 0.0f, 0.11f, 0.02f, 0.12f, 0.85f, 0.88f, 0.90f);

        // Cajas apiladas de reserva dentro de la tolva (visibles listas para caer)
        Primitives3D::DrawBox(hx, 0.57f, 0.0f, 0.07f, 0.05f, 0.07f, 0.75f, 0.65f, 0.45f);
        Primitives3D::DrawBox(hx, 0.63f, 0.0f, 0.07f, 0.05f, 0.07f, 0.75f, 0.65f, 0.45f);
    }

    void DrawHardwareSensors(int lightPwm) {
        // --- Sensor Ultrasónico HC-SR04 (ZT-101) ---
        Primitives3D::DrawBox(0.35f, 0.46f, 0.095f, 0.02f, 0.12f, 0.015f, 0.78f, 0.80f, 0.83f);
        Primitives3D::DrawBox(0.345f, 0.45f, 0.0f, 0.008f, 0.035f, 0.08f, 0.18f, 0.28f, 0.58f);
        Primitives3D::DrawCylinder(0.335f, 0.45f, -0.022f, 0.010f, 0.016f, 0.85f, 0.88f, 0.92f, 'x');
        Primitives3D::DrawCylinder(0.335f, 0.45f, 0.022f, 0.010f, 0.016f, 0.85f, 0.88f, 0.92f, 'x');

        // --- Gantry de Inspección y Cámara Óptica (QE-103 / QL-103) ---
        float gx = 0.24f;
        Primitives3D::DrawBox(gx, 0.54f, -0.11f, 0.025f, 0.28f, 0.025f, 0.78f, 0.80f, 0.83f);
        Primitives3D::DrawBox(gx, 0.54f, 0.11f, 0.025f, 0.28f, 0.025f, 0.78f, 0.80f, 0.83f);
        Primitives3D::DrawBox(gx, 0.67f, 0.0f, 0.03f, 0.03f, 0.24f, 0.78f, 0.80f, 0.83f);

        // Cámara industrial oscura
        Primitives3D::DrawBox(gx, 0.61f, 0.0f, 0.04f, 0.06f, 0.04f, 0.15f, 0.16f, 0.18f);

        // Anillo de iluminación LED coaxial (cuerpo físico sólido)
        (void)lightPwm;
        Primitives3D::DrawCylinder(gx, 0.55f, 0.0f, 0.024f, 0.008f, 0.85f, 0.85f, 0.88f, 'y');

        // Sensor LDR lateral
        Primitives3D::DrawBox(gx, 0.43f, -0.08f, 0.015f, 0.015f, 0.015f, 0.4f, 0.2f, 0.1f);
    }

    // =========================================================================
    // LABIO Y RAMPA DE DESCARGA DIRECTA (EXTREMO CABEZA DE FAJA)
    // =========================================================================
    void DrawDischargeLip() {
        // Labio inclinado de acero inoxidable que conduce las cajas al carro móvil
        glPushMatrix();
        glTranslatef(0.53f, 0.36f, 0.0f);
        glRotatef(-28.0f, 0, 0, 1);

        // Chapa deslizante
        Primitives3D::DrawBox(0.0f, 0.0f, 0.0f, 0.12f, 0.006f, 0.16f, 0.85f, 0.88f, 0.90f);
        // Rebordes / pestañas laterales de contención
        Primitives3D::DrawBox(0.0f, 0.02f, -0.082f, 0.12f, 0.035f, 0.006f, 0.80f, 0.82f, 0.85f);
        Primitives3D::DrawBox(0.0f, 0.02f, 0.082f, 0.12f, 0.035f, 0.006f, 0.80f, 0.82f, 0.85f);
        glPopMatrix();

        // Soportes diagonales de acero anclados al chasis frontal
        Primitives3D::DrawBox(0.50f, 0.30f, -0.085f, 0.015f, 0.14f, 0.015f, 0.70f, 0.72f, 0.75f);
        Primitives3D::DrawBox(0.50f, 0.30f, 0.085f, 0.015f, 0.14f, 0.015f, 0.70f, 0.72f, 0.75f);
    }

    // =========================================================================
    // LANZADERA / CARRO MÓVIL INDEXADOR DE CONTENEDORES (M-102)
    // =========================================================================
    void DrawMotorizedContainerShuttle(float cartZ) {
        float aluR = 0.78f, aluG = 0.80f, aluB = 0.83f;
        float darkR = 0.22f, darkG = 0.24f, darkB = 0.26f;

        // --- 1. Bancada y Guías Lineales Fijas (Eje Z transversal, X = 0.62) ---
        // Viga soporte base perfil Rexroth
        Primitives3D::DrawBox(0.62f, 0.02f, 0.0f, 0.10f, 0.04f, 0.94f, aluR, aluG, aluB);
        // Ranura central
        Primitives3D::DrawBox(0.62f, 0.041f, 0.0f, 0.015f, 0.004f, 0.92f, 0.40f, 0.42f, 0.45f);

        // Doble eje guía cilíndrico de acero rectificado
        Primitives3D::DrawCylinder(0.585f, 0.055f, 0.0f, 0.006f, 0.92f, 0.90f, 0.92f, 0.94f, 'z');
        Primitives3D::DrawCylinder(0.655f, 0.055f, 0.0f, 0.006f, 0.92f, 0.90f, 0.92f, 0.94f, 'z');

        // Soportes de extremos de eje (Pillow blocks)
        Primitives3D::DrawBox(0.62f, 0.05f, -0.46f, 0.11f, 0.06f, 0.03f, 0.35f, 0.38f, 0.42f);
        Primitives3D::DrawBox(0.62f, 0.05f, 0.46f, 0.11f, 0.06f, 0.03f, 0.35f, 0.38f, 0.42f);

        // Correa dentada síncrona / Husillo de transmisión
        Primitives3D::DrawBox(0.62f, 0.055f, 0.0f, 0.012f, 0.006f, 0.90f, 0.15f, 0.15f, 0.16f);

        // --- 2. Servomotor / Motor Paso a Paso de Indexación M-102 (NEMA 17 / Blue Housing) ---
        // Cuerpo del motor en extremo -Z (-0.49m)
        Primitives3D::DrawBox(0.62f, 0.065f, -0.50f, 0.055f, 0.055f, 0.06f, darkR, darkG, darkB);
        // Carcasa azul técnica del encoder / reductor
        Primitives3D::DrawBox(0.62f, 0.065f, -0.54f, 0.045f, 0.045f, 0.025f, 0.12f, 0.45f, 0.72f);
        // Polea de tracción de indexación
        Primitives3D::DrawCylinder(0.62f, 0.055f, -0.465f, 0.012f, 0.015f, 0.85f, 0.88f, 0.90f, 'y');

        // --- 3. Carro Móvil Deslizante (Plataforma Portacontenedores a Z = cartZ) ---
        float carY = 0.08f;
        // Placa base de aluminio mecanizado porta-gavetas
        Primitives3D::DrawBox(0.62f, carY, cartZ, 0.18f, 0.016f, 0.50f, aluR, aluG, aluB);

        // 4 Bloques de rodamientos lineales de bolas (Linear Bushings) bajo la placa
        for (float bz : {-0.18f, 0.18f}) {
            Primitives3D::DrawBox(0.585f, 0.055f, cartZ + bz, 0.028f, 0.024f, 0.045f, 0.40f, 0.43f, 0.46f);
            Primitives3D::DrawBox(0.655f, 0.055f, cartZ + bz, 0.028f, 0.024f, 0.045f, 0.40f, 0.43f, 0.46f);
        }
        // Tensor central de sujeción a la correa
        Primitives3D::DrawBox(0.62f, 0.065f, cartZ, 0.035f, 0.015f, 0.04f, 0.85f, 0.70f, 0.15f);

        // --- 4. Tres Contenedores Industriales Segregados (Montados sobre el carro) ---
        // Dimensiones de cada contenedor: 0.15m prof (X), 0.13m alto (Y), 0.14m ancho (Z)
        float binDX = 0.15f, binDY = 0.13f, binDZ = 0.14f;
        float binCenterY = carY + 0.008f + binDY * 0.5f;

        // A. CONTENEDOR 1: RETRABAJO (Ámbar, offset local Z = -0.16m)
        {
            float z1 = cartZ - 0.16f;
            // Carcasa externa ámbar
            Primitives3D::DrawBox(0.62f, binCenterY, z1, binDX, binDY, binDZ, 0.92f, 0.70f, 0.15f);
            // Labio superior reforzado
            Primitives3D::DrawBox(0.62f, binCenterY + binDY * 0.5f, z1, binDX + 0.012f, 0.012f, binDZ + 0.012f, 0.75f, 0.55f, 0.10f);
            // Hueco interior / cavidad oscura
            Primitives3D::DrawBox(0.62f, binCenterY + 0.015f, z1, binDX - 0.02f, binDY, binDZ - 0.02f, 0.22f, 0.16f, 0.06f);
            // Placa frontal de identificación técnica
            Primitives3D::DrawBox(0.54f, binCenterY, z1, 0.002f, 0.04f, 0.08f, 0.95f, 0.95f, 0.95f);
        }

        // B. CONTENEDOR 2: CONFORME (Verde, offset local Z = 0.0m)
        {
            float z2 = cartZ;
            // Carcasa externa verde ingeniería
            Primitives3D::DrawBox(0.62f, binCenterY, z2, binDX, binDY, binDZ, 0.25f, 0.65f, 0.30f);
            // Labio superior reforzado
            Primitives3D::DrawBox(0.62f, binCenterY + binDY * 0.5f, z2, binDX + 0.012f, 0.012f, binDZ + 0.012f, 0.18f, 0.50f, 0.22f);
            // Cavidad interior oscura
            Primitives3D::DrawBox(0.62f, binCenterY + 0.015f, z2, binDX - 0.02f, binDY, binDZ - 0.02f, 0.08f, 0.22f, 0.10f);
            // Placa frontal
            Primitives3D::DrawBox(0.54f, binCenterY, z2, 0.002f, 0.04f, 0.08f, 0.95f, 0.95f, 0.95f);
        }

        // C. CONTENEDOR 3: RECHAZO (Rojo, offset local Z = +0.16m)
        {
            float z3 = cartZ + 0.16f;
            // Carcasa externa roja
            Primitives3D::DrawBox(0.62f, binCenterY, z3, binDX, binDY, binDZ, 0.85f, 0.22f, 0.20f);
            // Labio superior reforzado
            Primitives3D::DrawBox(0.62f, binCenterY + binDY * 0.5f, z3, binDX + 0.012f, 0.012f, binDZ + 0.012f, 0.65f, 0.16f, 0.15f);
            // Cavidad interior oscura
            Primitives3D::DrawBox(0.62f, binCenterY + 0.015f, z3, binDX - 0.02f, binDY, binDZ - 0.02f, 0.25f, 0.08f, 0.08f);
            // Placa frontal
            Primitives3D::DrawBox(0.54f, binCenterY, z3, 0.002f, 0.04f, 0.08f, 0.95f, 0.95f, 0.95f);
        }
    }

    void DrawPedestal(FsmState state, bool eStop) {
        float px = -0.15f, pz = -0.32f;
        Primitives3D::DrawCylinder(px, 0.20f, pz, 0.02f, 0.40f, 0.25f, 0.28f, 0.32f, 'y');
        Primitives3D::DrawBox(px, 0.42f, pz, 0.12f, 0.06f, 0.14f, 0.85f, 0.88f, 0.90f);

        // Seta de Parada de Emergencia
        Primitives3D::DrawCylinder(px, 0.465f, pz - 0.02f, 0.018f, 0.018f, eStop ? 0.4f : 0.9f, 0.1f, 0.1f, 'y');

        // Baliza industrial 3 pisos
        float by = 0.72f;
        bool redOn = eStop || state == FSM_EMERG;
        bool yelOn = state == FSM_REPOSO || state == FSM_INSPEC;
        bool grnOn = state == FSM_TRANSP || state == FSM_DESVIO;

        Primitives3D::DrawCylinder(px, by + 0.050f, pz - 0.05f, 0.018f, 0.022f, redOn ? 1.0f : 0.2f, 0.1f, 0.1f, 'y');
        Primitives3D::DrawCylinder(px, by + 0.025f, pz - 0.05f, 0.018f, 0.022f, yelOn ? 1.0f : 0.2f, yelOn ? 0.85f : 0.2f, 0.1f, 'y');
        Primitives3D::DrawCylinder(px, by + 0.000f, pz - 0.05f, 0.018f, 0.022f, 0.1f, grnOn ? 1.0f : 0.2f, 0.1f, 'y');
    }

    void DrawBoxes(PlantSimulation& sim) {
        for (auto& b : sim.Boxes) {
            glPushMatrix();
            glTranslatef(b.PosX, b.PosY, b.PosZ);
            glRotatef(b.RotY, 0, 1, 0);
            glRotatef(b.RotZ, 0, 0, 1);

            // Cuerpo sólido opaco de la caja
            Primitives3D::DrawBox(0, 0, 0, b.SizeX, b.SizeY, b.SizeZ, b.ColorR, b.ColorG, b.ColorB);

            // Etiqueta superior blanca de código de barras
            Primitives3D::DrawBox(0, b.SizeY * 0.5f + 0.001f, 0, 0.035f, 0.001f, 0.035f, 0.95f, 0.95f, 0.95f);

            // Cinta de sellado
            Primitives3D::DrawBox(0, b.SizeY * 0.5f + 0.0015f, 0, b.SizeX * 0.9f, 0.001f, 0.012f, 0.75f, 0.60f, 0.40f);

            glPopMatrix();
        }
    }

    // =========================================================================
    // EFECTOS VOLUMÉTRICOS TRANSLÚCIDOS (Luz cálida y sonar sin tonos grises)
    // =========================================================================
    void DrawTranslucentEffects(PlantSimulation& sim) {
        glDisable(GL_LIGHTING); // Sin sombras grises
        glDepthMask(GL_FALSE);  // No escribir en Z-buffer para no tapar las cajas
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE); // Mezcla aditiva luminosa limpia

        // 1. Cono sonar ultrasónico (HC-SR04) - Azul cian suave
        float coneLen = std::clamp(sim.UltrasonicDistCm * 0.01f, 0.05f, 0.40f);
        glPushMatrix();
        glTranslatef(0.32f, 0.45f, 0.0f);
        glRotatef(-90.0f, 0, 0, 1);
        glBegin(GL_TRIANGLE_FAN);
        glColor4f(0.15f, 0.65f, 1.0f, 0.22f);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glColor4f(0.15f, 0.65f, 1.0f, 0.02f);
        for (int i = 0; i <= 16; i++) {
            float ang = 2.0f * (float)M_PI * (float)i / 16.0f;
            glVertex3f(std::cos(ang) * 0.045f, coneLen, std::sin(ang) * 0.045f);
        }
        glEnd();
        glPopMatrix();

        // 2. Cono del haz de luz LED de inspección (QL-103) - Cálido ultra-transparente
        float gx = 0.24f;
        float intens = (float)sim.LightPwm / 255.0f;

        if (intens > 0.01f) {
            glPushMatrix();
            glTranslatef(gx, 0.55f, 0.0f);
            glBegin(GL_TRIANGLE_FAN);
            glColor4f(1.0f, 0.98f, 0.88f, 0.08f + 0.24f * intens);
            glVertex3f(0.0f, 0.0f, 0.0f);
            glColor4f(1.0f, 0.95f, 0.80f, 0.01f + 0.03f * intens);
            for (int i = 0; i <= 20; i++) {
                float a = 2.0f * (float)M_PI * (float)i / 20.0f;
                glVertex3f(std::cos(a) * 0.095f, -0.16f, std::sin(a) * 0.095f);
            }
            glEnd();
            glPopMatrix();

            // Mancha suave de luz circular sobre la faja
            glPushMatrix();
            glTranslatef(gx, 0.402f, 0.0f);
            glBegin(GL_TRIANGLE_FAN);
            glColor4f(1.0f, 0.98f, 0.85f, 0.06f + 0.16f * intens);
            glVertex3f(0.0f, 0.0f, 0.0f);
            glColor4f(1.0f, 0.95f, 0.80f, 0.0f);
            for (int i = 0; i <= 16; i++) {
                float a = 2.0f * (float)M_PI * (float)i / 16.0f;
                glVertex3f(std::cos(a) * 0.09f, 0.0f, std::sin(a) * 0.09f);
            }
            glEnd();
            glPopMatrix();
        }

        glDisable(GL_BLEND);
        glDepthMask(GL_TRUE);  // Restaurar Z-buffer
        glEnable(GL_LIGHTING); // Restaurar iluminación general
    }
};

#endif // MODEL_3D_HPP
