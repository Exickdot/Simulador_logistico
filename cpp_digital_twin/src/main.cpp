// =====================================================================================
// PROYECTO: ESTACIÓN LOGÍSTICA DE SELECCIÓN, CLASIFICACIÓN Y ENRUTAMIENTO INDUSTRIAL
// GEMELO DIGITAL NATIVO C++ (OPENGL / WIN32) - CONSOLA RETRO INFERIOR UNIFICADA
// =====================================================================================

#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <vector>
#include <algorithm>
#include "plant_simulation.hpp"
#include "model_3d.hpp"

// Estado global de la aplicación
PlantSimulation g_sim;
Station3DRenderer g_renderer;

// Dimensiones de ventana y consola
int g_windowWidth = 1200;
int g_windowHeight = 720;
const int CONSOLE_H = 195; // Altura de la consola retro inferior

// Cámara orbital CAD
struct CameraCAD {
    float TargetX = 0.05f, TargetY = 0.38f, TargetZ = 0.0f;
    float Distance = 1.65f;
    float Yaw = 45.0f;
    float Pitch = 28.0f;

    bool IsRotating = false;
    bool IsPanning = false;
    POINT LastMouse;

    void SetPreset(int idx) {
        if (idx == 1) { // Vista General Isométrica
            TargetX = 0.05f; TargetY = 0.38f; TargetZ = 0.0f; Distance = 1.65f; Yaw = 45.0f; Pitch = 28.0f;
        } else if (idx == 2) { // Vista Cámara Óptica / Gantry
            TargetX = 0.22f; TargetY = 0.42f; TargetZ = 0.0f; Distance = 0.85f; Yaw = 20.0f; Pitch = 22.0f;
        } else if (idx == 3) { // Vista Lateral de FSM / Sensor
            TargetX = 0.28f; TargetY = 0.46f; TargetZ = 0.0f; Distance = 0.75f; Yaw = 85.0f; Pitch = 30.0f;
        } else if (idx == 4) { // Vista Carro Móvil Indexador M-102 y Contenedores
            TargetX = 0.60f; TargetY = 0.25f; TargetZ = 0.0f; Distance = 0.95f; Yaw = 125.0f; Pitch = 28.0f;
        } else if (idx == 5) { // Vista Tolva de Alimentación
            TargetX = -0.45f; TargetY = 0.50f; TargetZ = 0.0f; Distance = 0.85f; Yaw = 315.0f; Pitch = 32.0f;
        }
    }

    void Apply() {
        float radYaw = Yaw * (float)M_PI / 180.0f;
        float radPitch = Pitch * (float)M_PI / 180.0f;

        float eyeX = TargetX + Distance * std::cos(radPitch) * std::sin(radYaw);
        float eyeY = TargetY + Distance * std::sin(radPitch);
        float eyeZ = TargetZ + Distance * std::cos(radPitch) * std::cos(radYaw);

        gluLookAt(eyeX, eyeY, eyeZ, TargetX, TargetY, TargetZ, 0.0f, 1.0f, 0.0f);
    }
} g_camera;

// Botones de la consola HUD
struct HudButton {
    int id;
    int x, y, w, h;
    const char* label;
    float r, g, b;
};

std::vector<HudButton> g_hudButtons;

void UpdateHudButtons() {
    g_hudButtons.clear();
    int consoleY = g_windowHeight - CONSOLE_H;
    int margin = 10;
    int w1 = 430;
    int w2 = 320;
    int x3 = margin + w1 + 10 + w2 + 10;
    int w3 = g_windowWidth - x3 - margin;
    if (w3 < 390) w3 = 390;

    int deskX = x3 + 8;
    int deskW = w3 - 16;

    // Fila 1: Botones principales de operación
    int btnY1 = consoleY + 36;
    int btnH1 = 34;
    int b1_w = (deskW - 16) / 3;
    int b1_w_estop = deskW - 16 - (b1_w * 2);

    g_hudButtons.push_back({1, deskX, btnY1, b1_w, btnH1, "INICIAR", 0.12f, 0.62f, 0.24f});
    g_hudButtons.push_back({2, deskX + b1_w + 8, btnY1, b1_w, btnH1, "DETENER", 0.36f, 0.40f, 0.46f});
    g_hudButtons.push_back({4, deskX + (b1_w + 8) * 2, btnY1, b1_w_estop, btnH1, "! E-STOP !", 0.85f, 0.14f, 0.14f});

    // Fila 2: Botones secundarios de simulación
    int btnY2 = consoleY + 76;
    int btnH2 = 32;
    int b2_w = (deskW - 16) / 3;
    int b2_w_last = deskW - 16 - (b2_w * 2);

    g_hudButtons.push_back({3, deskX, btnY2, b2_w, btnH2, "+ CAJA", 0.14f, 0.46f, 0.82f});
    g_hudButtons.push_back({5, deskX + b2_w + 8, btnY2, b2_w, btnH2, "RESET", 0.78f, 0.45f, 0.12f});
    g_hudButtons.push_back({6, deskX + (b2_w + 8) * 2, btnY2, b2_w_last, btnH2, "LUZ AMB", 0.72f, 0.58f, 0.15f});

    // Fila 3: Banco selector de cámaras (V1..V5)
    int btnY3 = consoleY + 116;
    int btnH3 = 28;
    int numCams = 5;
    int camSpacing = 6;
    int camW = (deskW - camSpacing * (numCams - 1)) / numCams;

    const char* camLabels[] = {"V1:ISO", "V2:CAM", "V3:LAT", "V4:CARRO", "V5:TOLVA"};
    for (int i = 0; i < numCams; i++) {
        int cx = deskX + i * (camW + camSpacing);
        g_hudButtons.push_back({11 + i, cx, btnY3, camW, btnH3, camLabels[i], 0.22f, 0.26f, 0.32f});
    }
}

// Fuentes tipográficas: UI general y Terminal Monospace Retro
GLuint g_fontBase = 0;
GLuint g_fontMono = 0;

void BuildFonts(HDC hDC) {
    // Fuente para botones y etiquetas
    HFONT hFont = CreateFontA(-13, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
                             ANSI_CHARSET, OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS,
                             ANTIALIASED_QUALITY, FF_DONTCARE | DEFAULT_PITCH, "Segoe UI");
    HFONT hOldFont = (HFONT)SelectObject(hDC, hFont);
    g_fontBase = glGenLists(96);
    wglUseFontBitmaps(hDC, 32, 96, g_fontBase);

    // Fuente Monospace para terminal CRT de fósforo verde
    HFONT hMono = CreateFontA(-13, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE,
                             ANSI_CHARSET, OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS,
                             ANTIALIASED_QUALITY, FIXED_PITCH | FF_MODERN, "Consolas");
    SelectObject(hDC, hMono);
    g_fontMono = glGenLists(96);
    wglUseFontBitmaps(hDC, 32, 96, g_fontMono);

    SelectObject(hDC, hOldFont);
    DeleteObject(hFont);
    DeleteObject(hMono);
}

void DrawText2D(float x, float y, const char* text, float r = 1.0f, float g = 1.0f, float b = 1.0f) {
    glColor3f(r, g, b);
    glRasterPos2f(x, y);
    glPushAttrib(GL_LIST_BIT);
    glListBase(g_fontBase - 32);
    glCallLists((GLsizei)strlen(text), GL_UNSIGNED_BYTE, text);
    glPopAttrib();
}

void DrawTextMono(float x, float y, const char* text, float r = 0.25f, float g = 1.0f, float b = 0.45f) {
    glColor3f(r, g, b);
    glRasterPos2f(x, y);
    glPushAttrib(GL_LIST_BIT);
    glListBase(g_fontMono - 32);
    glCallLists((GLsizei)strlen(text), GL_UNSIGNED_BYTE, text);
    glPopAttrib();
}

void DrawRect2D(float x, float y, float w, float h, float r, float g, float b, float a = 1.0f) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(r, g, b, a);
    glBegin(GL_QUADS);
    glVertex2f(x, y);
    glVertex2f(x + w, y);
    glVertex2f(x + w, y + h);
    glVertex2f(x, y + h);
    glEnd();
    glDisable(GL_BLEND);
}

// Bisel tridimensional para botones y chasis de hardware retro
void DrawBevelBox2D(float x, float y, float w, float h, float r, float g, float b, bool inset = false) {
    DrawRect2D(x, y, w, h, r, g, b, 1.0f);

    float lightR = std::min(1.0f, r + 0.28f);
    float lightG = std::min(1.0f, g + 0.28f);
    float lightB = std::min(1.0f, b + 0.28f);
    float darkR = std::max(0.0f, r - 0.25f);
    float darkG = std::max(0.0f, g - 0.25f);
    float darkB = std::max(0.0f, b - 0.25f);

    float topR = inset ? darkR : lightR;
    float topG = inset ? darkG : lightG;
    float topB = inset ? darkB : lightB;
    float botR = inset ? lightR : darkR;
    float botG = inset ? lightG : darkG;
    float botB = inset ? lightB : darkB;

    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    // Borde superior
    glColor3f(topR, topG, topB);
    glVertex2f(x, y + 1);
    glVertex2f(x + w, y + 1);
    // Borde izquierdo
    glVertex2f(x + 1, y);
    glVertex2f(x + 1, y + h);
    // Borde inferior
    glColor3f(botR, botG, botB);
    glVertex2f(x, y + h - 1);
    glVertex2f(x + w, y + h - 1);
    // Borde derecho
    glVertex2f(x + w - 1, y);
    glVertex2f(x + w - 1, y + h);
    glEnd();
}

// Borde perimetral de un panel
void DrawBorder2D(float x, float y, float w, float h, float r, float g, float b, float width = 1.0f) {
    glDisable(GL_LIGHTING);
    glLineWidth(width);
    glColor3f(r, g, b);
    glBegin(GL_LINE_LOOP);
    glVertex2f(x, y);
    glVertex2f(x + w, y);
    glVertex2f(x + w, y + h);
    glVertex2f(x, y + h);
    glEnd();
}

// Líneas de barrido CRT (Scanlines de pantalla retro)
void DrawScanlines(float x, float y, float w, float h) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.0f, 0.0f, 0.0f, 0.22f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for (float sy = y + 2; sy < y + h; sy += 3.0f) {
        glVertex2f(x + 1, sy);
        glVertex2f(x + w - 1, sy);
    }
    glEnd();
    glDisable(GL_BLEND);
}

// Indicador LED de joya retro encendido/apagado
void DrawLedIndicator(float cx, float cy, float radius, float r, float g, float b, bool active) {
    glDisable(GL_LIGHTING);
    glBegin(GL_TRIANGLE_FAN);
    if (active) {
        glColor3f(r, g, b);
    } else {
        glColor3f(r * 0.22f, g * 0.22f, b * 0.22f);
    }
    glVertex2f(cx, cy);
    for (int i = 0; i <= 12; i++) {
        float a = 2.0f * (float)M_PI * (float)i / 12.0f;
        glVertex2f(cx + std::cos(a) * radius, cy + std::sin(a) * radius);
    }
    glEnd();

    // Aro metálico protector
    glColor3f(0.35f, 0.40f, 0.45f);
    glLineWidth(1.0f);
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 12; i++) {
        float a = 2.0f * (float)M_PI * (float)i / 12.0f;
        glVertex2f(cx + std::cos(a) * (radius + 1.0f), cy + std::sin(a) * (radius + 1.0f));
    }
    glEnd();
}

// Renderizado integral de la Consola Retro Inferior Unificada
void RenderHUD() {
    glViewport(0, 0, g_windowWidth, g_windowHeight);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, g_windowWidth, g_windowHeight, 0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    int consoleY = g_windowHeight - CONSOLE_H;
    int margin = 10;
    int w1 = 430;
    int w2 = 320;
    int x3 = margin + w1 + 10 + w2 + 10;
    int w3 = g_windowWidth - x3 - margin;
    if (w3 < 390) w3 = 390;

    int y_panels = consoleY + 28;
    int h_panels = CONSOLE_H - 36;

    // 1. Chasis exterior de la consola (Acero industrial oscuro)
    DrawRect2D(0, (float)consoleY, (float)g_windowWidth, (float)CONSOLE_H, 0.11f, 0.13f, 0.16f, 1.0f);

    // Embellecedor superior con bisel de aluminio maquinado
    DrawBevelBox2D(0, (float)consoleY, (float)g_windowWidth, 24.0f, 0.20f, 0.22f, 0.26f, false);
    DrawTextMono(16, (float)consoleY + 16, "[ CONSOLA DE SUPERVISION INDUSTRIAL // AUTO-TIF v2.4 ]", 0.88f, 0.90f, 0.94f);
    DrawTextMono((float)g_windowWidth - 370, (float)consoleY + 16, "SISTEMA: ATmega328P • MODO: AUTONOMO", 0.60f, 0.75f, 0.85f);

    // =========================================================================
    // 2. MÓDULO A: CRT-1 (TERMINAL DE FÓSFORO VERDE - TELEMETRÍA Y CONTROL FSM)
    // =========================================================================
    DrawBevelBox2D((float)margin, (float)y_panels, (float)w1, (float)h_panels, 0.02f, 0.05f, 0.03f, true);
    DrawBorder2D((float)margin, (float)y_panels, (float)w1, (float)h_panels, 0.15f, 0.45f, 0.22f, 1.0f);
    DrawScanlines((float)margin, (float)y_panels, (float)w1, (float)h_panels);

    DrawTextMono((float)margin + 12, (float)y_panels + 18, ">> CRT-1: TELEMETRIA FSM & CONTROL <<", 0.25f, 1.0f, 0.50f);

    // FSM y Sonar
    const char* stName = GetFsmStateName(g_sim.State);
    float fsmR = 0.2f, fsmG = 1.0f, fsmB = 0.4f;
    if (g_sim.EmergencyStop || g_sim.State == FSM_EMERG) { fsmR = 1.0f; fsmG = 0.15f; fsmB = 0.15f; }
    else if (g_sim.State == FSM_DESVIO) { fsmR = 1.0f; fsmG = 0.85f; fsmB = 0.2f; }
    else if (g_sim.State == FSM_INSPEC) { fsmR = 0.2f; fsmG = 0.9f; fsmB = 1.0f; }

    char buf[128];
    snprintf(buf, sizeof(buf), "[FSM]: %-7s  | ZT-101 (SONAR): %5.1f cm", stName, g_sim.UltrasonicDistCm);
    DrawTextMono((float)margin + 12, (float)y_panels + 42, buf, fsmR, fsmG, fsmB);

    // Lazo 1: Faja y Velocidad con barra ASCII
    char bar[13];
    int filled = (int)std::clamp((g_sim.UltrasonicDistCm / 35.0f) * 10.0f, 0.0f, 10.0f);
    for (int i = 0; i < 10; i++) bar[i] = (i < filled) ? '=' : '.';
    bar[10] = '\0';
    snprintf(buf, sizeof(buf), "L1 FAJA: %4.2f m/s (PWM %3d) [%s]", g_sim.BeltSpeedMps, g_sim.MotorPwm, bar);
    DrawTextMono((float)margin + 12, (float)y_panels + 64, buf, 0.30f, 0.95f, 0.45f);

    // Lazo 2: Carro Móvil Indexador M-102
    const char* cDest = "CENTRO [CONF]";
    if (g_sim.TargetCartZ > 0.05f) cDest = "IZQ [RETR]";
    else if (g_sim.TargetCartZ < -0.05f) cDest = "DER [RECH]";
    snprintf(buf, sizeof(buf), "L2 CARRO M-102: %+4.0f mm  -> %s", g_sim.CartZ * 1000.0f, cDest);
    DrawTextMono((float)margin + 12, (float)y_panels + 86, buf, 0.30f, 0.95f, 0.45f);

    // Lazo 3: Luz LDR y LED
    snprintf(buf, sizeof(buf), "L3 OPTICA: %5.1f%% LUX (SP:%2.0f%%) | LED: %3d/255", 
             g_sim.InspectionLuxPct, g_sim.TargetLuxPct, g_sim.LightPwm);
    DrawTextMono((float)margin + 12, (float)y_panels + 108, buf, 0.30f, 0.95f, 0.45f);

    // Prompt de estado del microcontrolador
    snprintf(buf, sizeof(buf), "ATmega328P::OK [20ms] • LUZ AMB: %2.0f%% > _", g_sim.AmbientLuxPct);
    DrawTextMono((float)margin + 12, (float)y_panels + 130, buf, 0.22f, 0.80f, 0.38f);

    // =========================================================================
    // 3. MÓDULO B: CRT-2 (MATRIZ DE PRODUCCIÓN OEE & VISUALIZADORES DIGITALES)
    // =========================================================================
    int x2 = margin + w1 + 10;
    DrawBevelBox2D((float)x2, (float)y_panels, (float)w2, (float)h_panels, 0.03f, 0.04f, 0.06f, true);
    DrawBorder2D((float)x2, (float)y_panels, (float)w2, (float)h_panels, 0.18f, 0.35f, 0.50f, 1.0f);
    DrawScanlines((float)x2, (float)y_panels, (float)w2, (float)h_panels);

    DrawTextMono((float)x2 + 12, (float)y_panels + 18, ">> CRT-2: OEE & CLASIFICACION <<", 0.40f, 0.85f, 1.0f);

    // Cuadrícula 2x2 de visualizadores digitales estilo LED/Nixie
    int cellW = (w2 - 30) / 2;
    int cellH = 34;

    // Celda 1: TOTAL PROCESADO
    DrawBevelBox2D((float)x2 + 10, (float)y_panels + 26, (float)cellW, (float)cellH, 0.10f, 0.12f, 0.16f, true);
    DrawTextMono((float)x2 + 16, (float)y_panels + 40, "TOTAL:", 0.65f, 0.70f, 0.78f);
    snprintf(buf, sizeof(buf), "%04d", g_sim.TotalProcessed);
    DrawTextMono((float)x2 + 82, (float)y_panels + 48, buf, 0.90f, 0.95f, 1.0f);

    // Celda 2: CONFORME (Verde)
    DrawBevelBox2D((float)x2 + 10 + cellW + 10, (float)y_panels + 26, (float)cellW, (float)cellH, 0.06f, 0.15f, 0.08f, true);
    DrawTextMono((float)x2 + 10 + cellW + 16, (float)y_panels + 40, "CONF:", 0.40f, 0.85f, 0.45f);
    snprintf(buf, sizeof(buf), "%04d", g_sim.CountConforme);
    DrawTextMono((float)x2 + 10 + cellW + 82, (float)y_panels + 48, buf, 0.25f, 1.0f, 0.35f);

    // Celda 3: RETRABAJO (Ámbar)
    DrawBevelBox2D((float)x2 + 10, (float)y_panels + 66, (float)cellW, (float)cellH, 0.16f, 0.12f, 0.04f, true);
    DrawTextMono((float)x2 + 16, (float)y_panels + 80, "RETR:", 0.85f, 0.70f, 0.20f);
    snprintf(buf, sizeof(buf), "%04d", g_sim.CountRetrabajo);
    DrawTextMono((float)x2 + 82, (float)y_panels + 88, buf, 1.0f, 0.78f, 0.15f);

    // Celda 4: RECHAZO (Rojo)
    DrawBevelBox2D((float)x2 + 10 + cellW + 10, (float)y_panels + 66, (float)cellW, (float)cellH, 0.18f, 0.06f, 0.06f, true);
    DrawTextMono((float)x2 + 10 + cellW + 16, (float)y_panels + 80, "RECH:", 0.90f, 0.40f, 0.35f);
    snprintf(buf, sizeof(buf), "%04d", g_sim.CountRechazo);
    DrawTextMono((float)x2 + 10 + cellW + 82, (float)y_panels + 88, buf, 1.0f, 0.25f, 0.25f);

    // Tasa de Eficiencia OEE
    float oee = (g_sim.TotalProcessed > 0) ? ((float)g_sim.CountConforme / (float)g_sim.TotalProcessed * 100.0f) : 100.0f;
    snprintf(buf, sizeof(buf), "EFICIENCIA OEE: %5.1f%% [CONF/TOTAL]", oee);
    DrawTextMono((float)x2 + 12, (float)y_panels + 116, buf, 0.85f, 0.90f, 0.95f);

    // Fila inferior de LEDs indicadores de hardware
    DrawLedIndicator((float)x2 + 25, (float)y_panels + 134, 4.5f, 0.1f, 1.0f, 0.2f, true); // PWR
    DrawTextMono((float)x2 + 35, (float)y_panels + 138, "PWR", 0.65f, 0.75f, 0.85f);

    DrawLedIndicator((float)x2 + 95, (float)y_panels + 134, 4.5f, 0.2f, 0.6f, 1.0f, g_sim.BeltSpeedMps > 0.01f); // FAJA
    DrawTextMono((float)x2 + 105, (float)y_panels + 138, "FAJA", 0.65f, 0.75f, 0.85f);

    DrawLedIndicator((float)x2 + 175, (float)y_panels + 134, 4.5f, 1.0f, 0.8f, 0.1f, g_sim.State == FSM_INSPEC); // OPT
    DrawTextMono((float)x2 + 185, (float)y_panels + 138, "CAM", 0.65f, 0.75f, 0.85f);

    DrawLedIndicator((float)x2 + 245, (float)y_panels + 134, 4.5f, 1.0f, 0.2f, 0.2f, g_sim.EmergencyStop || g_sim.State == FSM_EMERG); // ALM
    DrawTextMono((float)x2 + 255, (float)y_panels + 138, "ALM", 0.65f, 0.75f, 0.85f);

    // =========================================================================
    // 4. MÓDULO C: TABLERO DE MANDO ANALÓGICO / INDUSTRIAL RETRO
    // =========================================================================
    DrawBevelBox2D((float)x3, (float)y_panels, (float)w3, (float)h_panels, 0.15f, 0.17f, 0.21f, false);
    DrawBorder2D((float)x3, (float)y_panels, (float)w3, (float)h_panels, 0.32f, 0.36f, 0.42f, 1.0f);

    DrawTextMono((float)x3 + 12, (float)y_panels + 18, ">> PANEL DE COMANDO ANALOGICO <<", 0.88f, 0.90f, 0.94f);

    // Botones pulsadores mecánicos con biseles tridimensionales
    for (const auto& btn : g_hudButtons) {
        if (btn.id == 4) { // Botón Seta de Emergencia (E-STOP) con marco de advertencia
            DrawBorder2D((float)btn.x - 2, (float)btn.y - 2, (float)btn.w + 4, (float)btn.h + 4, 0.95f, 0.78f, 0.15f, 2.0f);
            DrawBevelBox2D((float)btn.x, (float)btn.y, (float)btn.w, (float)btn.h, btn.r, btn.g, btn.b, false);
            DrawText2D((float)btn.x + (btn.w - 74) / 2, (float)btn.y + 22, btn.label, 1.0f, 1.0f, 1.0f);
        } else {
            DrawBevelBox2D((float)btn.x, (float)btn.y, (float)btn.w, (float)btn.h, btn.r, btn.g, btn.b, false);
            // Centrado visual de texto
            int txtOffset = (btn.w - (int)strlen(btn.label) * 8) / 2;
            if (txtOffset < 6) txtOffset = 6;
            DrawText2D((float)btn.x + txtOffset, (float)btn.y + (btn.h / 2) + 5, btn.label, 1.0f, 1.0f, 1.0f);
        }
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

// Configuración de proyección 3D ocupando el espacio visual sobre la consola
void Setup3DProjection() {
    int viewH = (g_windowHeight > CONSOLE_H + 50) ? (g_windowHeight - CONSOLE_H) : g_windowHeight;
    glViewport(0, CONSOLE_H, g_windowWidth, viewH);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (double)g_windowWidth / (double)viewH, 0.05, 50.0);
    glMatrixMode(GL_MODELVIEW);
}

// Manejador de eventos de ventana Win32
LRESULT CALLBACK WndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
    switch (uMsg) {
        case WM_SIZE:
            g_windowWidth = LOWORD(lParam);
            g_windowHeight = HIWORD(lParam);
            Setup3DProjection();
            UpdateHudButtons();
            return 0;

        case WM_LBUTTONDOWN: {
            int mx = LOWORD(lParam);
            int my = HIWORD(lParam);

            int consoleY = g_windowHeight - CONSOLE_H;
            if (my >= consoleY) {
                // Verificar clics en botones de la consola retro
                for (const auto& btn : g_hudButtons) {
                    if (mx >= btn.x && mx <= btn.x + btn.w && my >= btn.y && my <= btn.y + btn.h) {
                        if (btn.id == 1) { // INICIAR
                            g_sim.State = FSM_TRANSP;
                            g_sim.EmergencyStop = false;
                        } else if (btn.id == 2) { // DETENER
                            g_sim.State = FSM_REPOSO;
                        } else if (btn.id == 3) { // + CAJA
                            g_sim.SpawnBox();
                        } else if (btn.id == 4) { // PARADA EMERG
                            g_sim.TriggerEmergency(true);
                        } else if (btn.id == 5) { // RESET
                            g_sim.TriggerEmergency(false);
                            g_sim.State = FSM_REPOSO;
                        } else if (btn.id == 6) { // LUZ AMBIENTE
                            g_sim.AmbientLuxPct = (g_sim.AmbientLuxPct > 30.0f) ? 15.0f : 65.0f;
                        } else if (btn.id >= 11 && btn.id <= 15) { // V1..V5
                            g_camera.SetPreset(btn.id - 10);
                        }
                        return 0;
                    }
                }
                return 0; // Clic en el fondo de la consola
            }

            g_camera.IsRotating = true;
            g_camera.LastMouse.x = mx;
            g_camera.LastMouse.y = my;
            SetCapture(hWnd);
            return 0;
        }

        case WM_LBUTTONUP:
            if (g_camera.IsRotating) {
                g_camera.IsRotating = false;
                ReleaseCapture();
            }
            return 0;

        case WM_RBUTTONDOWN:
        case WM_MBUTTONDOWN: {
            int my = HIWORD(lParam);
            if (my >= g_windowHeight - CONSOLE_H) return 0;
            g_camera.IsPanning = true;
            g_camera.LastMouse.x = LOWORD(lParam);
            g_camera.LastMouse.y = my;
            SetCapture(hWnd);
            return 0;
        }

        case WM_RBUTTONUP:
        case WM_MBUTTONUP:
            if (g_camera.IsPanning) {
                g_camera.IsPanning = false;
                ReleaseCapture();
            }
            return 0;

        case WM_MOUSEMOVE: {
            int mx = LOWORD(lParam);
            int my = HIWORD(lParam);
            int dx = mx - g_camera.LastMouse.x;
            int dy = my - g_camera.LastMouse.y;
            g_camera.LastMouse.x = mx;
            g_camera.LastMouse.y = my;

            if (g_camera.IsRotating) {
                g_camera.Yaw += (float)dx * 0.4f;
                g_camera.Pitch = std::clamp(g_camera.Pitch + (float)dy * 0.4f, 5.0f, 85.0f);
            } else if (g_camera.IsPanning) {
                float speed = 0.0015f * g_camera.Distance;
                float radYaw = g_camera.Yaw * (float)M_PI / 180.0f;
                g_camera.TargetX += (-std::sin(radYaw) * (float)dx + std::cos(radYaw) * (float)dy) * speed;
                g_camera.TargetZ += (std::cos(radYaw) * (float)dx + std::sin(radYaw) * (float)dy) * speed;
                g_camera.TargetY += (float)dy * speed;
            }
            return 0;
        }

        case WM_MOUSEWHEEL: {
            short delta = GET_WHEEL_DELTA_WPARAM(wParam);
            g_camera.Distance = std::clamp(g_camera.Distance - (float)delta * 0.001f, 0.4f, 3.5f);
            return 0;
        }

        case WM_KEYDOWN:
            if (wParam == VK_ESCAPE) PostQuitMessage(0);
            else if (wParam == '1') g_camera.SetPreset(1);
            else if (wParam == '2') g_camera.SetPreset(2);
            else if (wParam == '3') g_camera.SetPreset(3);
            else if (wParam == '4') g_camera.SetPreset(4);
            else if (wParam == '5') g_camera.SetPreset(5);
            else if (wParam == VK_SPACE) g_sim.SpawnBox();
            return 0;

        case WM_CLOSE:
            PostQuitMessage(0);
            return 0;
    }
    return DefWindowProc(hWnd, uMsg, wParam, lParam);
}

// Punto de entrada WinMain
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR, int nCmdShow) {
    (void)nCmdShow;
    srand((unsigned int)time(NULL));

    WNDCLASSA wc = {};
    wc.style = CS_OWNDC | CS_HREDRAW | CS_VREDRAW;
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.lpszClassName = "AutoTif_CppWindow";
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    RegisterClassA(&wc);

    HWND hWnd = CreateWindowExA(
        0, "AutoTif_CppWindow",
        "Estacion Logistica Industrial - Gemelo Digital 3D Nativo C++ (OpenGL)",
        WS_OVERLAPPEDWINDOW | WS_VISIBLE,
        CW_USEDEFAULT, CW_USEDEFAULT, g_windowWidth, g_windowHeight,
        NULL, NULL, hInstance, NULL
    );

    HDC hDC = GetDC(hWnd);

    // Configuración del formato de píxeles OpenGL
    PIXELFORMATDESCRIPTOR pfd = {};
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    int pf = ChoosePixelFormat(hDC, &pfd);
    SetPixelFormat(hDC, pf, &pfd);

    HGLRC hRC = wglCreateContext(hDC);
    wglMakeCurrent(hDC, hRC);

    // Inicializar OpenGL
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    float lightPos[] = { 1.0f, 2.0f, 1.5f, 1.0f };
    float lightAmbient[] = { 0.4f, 0.4f, 0.4f, 1.0f };
    float lightDiffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);

    glClearColor(0.92f, 0.94f, 0.96f, 1.0f);

    Setup3DProjection();
    UpdateHudButtons();
    BuildFonts(hDC);

    // Bucle principal de simulación y renderizado a 60 FPS
    LARGE_INTEGER freq, lastTime, currentTime;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&lastTime);

    MSG msg = {};
    bool running = true;

    while (running) {
        while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
            if (msg.message == WM_QUIT) {
                running = false;
                break;
            }
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }

        QueryPerformanceCounter(&currentTime);
        float dt = (float)(currentTime.QuadPart - lastTime.QuadPart) / (float)freq.QuadPart;
        lastTime = currentTime;

        // Actualizar física de planta
        g_sim.Update(dt);

        // Renderizado 3D en área panorámica superior
        Setup3DProjection();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glLoadIdentity();
        g_camera.Apply();

        g_renderer.Render(g_sim, dt);

        // Renderizado de la Consola Retro Unificada Inferior
        RenderHUD();

        SwapBuffers(hDC);
        Sleep(12); // Límite de tasa de refresco a ~60-80 FPS para ahorro de CPU
    }

    if (g_fontBase) glDeleteLists(g_fontBase, 96);
    if (g_fontMono) glDeleteLists(g_fontMono, 96);
    wglMakeCurrent(NULL, NULL);
    wglDeleteContext(hRC);
    ReleaseDC(hWnd, hDC);
    DestroyWindow(hWnd);

    return 0;
}
