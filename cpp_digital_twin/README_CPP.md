# Gemelo Digital 3D Nativo en C++ (OpenGL / Win32)

Este módulo contiene el **Motor Gráfico 3D y Gemelo Digital en C++ Nativo** para la *Estación Logística de Selección, Clasificación y Enrutamiento* (ANSI / ISA-5.1).

---

## ⚡ Características Principales

* **Ejecutable Standalone:** Compilado como un único archivo `.exe` de **485 KB** (`simulador_3d_cpp.exe`).
* **Cero dependencias pesadas:** No requiere Unity Hub, ni .NET pesado, ni librerías externas DLL. Utiliza las librerías nativas de Windows (`opengl32.dll`, `glu32.dll`, `gdi32.dll`, `user32.dll`).
* **Rendimiento extremo:** Corre a más de 60-120 FPS consumiendo apenas **~25 MB de memoria RAM** (frente a los 2-3 GB de Unity).
* **Física y Control de Lazo Cerrado:**
  * **Lazo 1:** Rampa de desaceleración y parada anticolisión ultrasónica.
  * **Lazo 2:** Servocontrol cinemático de compuerta deflectora en 45°, 90° y 135°.
  * **Lazo 3:** Regulador PI discreto con Anti-Windup para iluminación fotométrica de inspección.
  * **Tracking Multiobjeto:** Cola continua de desplazamiento de cajas sobre la faja.
* **Cámara Orbital CAD 3D:**
  * **Arrastre con Clic Izquierdo:** Rotación esférica continua (Yaw / Pitch).
  * **Arrastre con Clic Derecho / Central:** Desplazamiento horizontal (Pan).
  * **Rueda del Ratón:** Zoom suave in / out.
  * **Teclas 1 a 5:** Vistas técnicas de ingeniería (Isométrica, Ultrasonido, Cámara/Luz, Servo/Tolvas, Pedestal).
* **SCADA HUD Integrado:**
  * Telemetría de proceso completa (Distancia, PWM motor, Lux %, PWM luz, Ángulo de servo).
  * Contadores de producción en tiempo real (Conforme, Retrabajo, Rechazo, Total).
  * Botones táctiles/clicables: `INICIAR`, `DETENER`, `+ CAJA`, `PARADA EMERG.`, `RESET`, `LUZ AMBIENTE`, `V1..V5`.

---

## 🚀 Cómo Ejecutar

Basta con hacer **doble clic** en cualquiera de estos dos archivos:

1. Desde la raíz del proyecto:
   ```cmd
   iniciar_simulador_cpp.bat
   ```
2. O directamente ejecutando:
   ```cmd
   cpp_digital_twin\simulador_3d_cpp.exe
   ```

---

## 🛠️ Cómo Recompilar el Código Fuente C++

El compilador portátil MinGW GCC 16.2.0 está incluido dentro de `cpp_digital_twin\w64devkit`, por lo que **no necesitas instalar nada en Windows**:

Simplemente ejecuta:
```cmd
cpp_digital_twin\compilar.bat
```
Generará inmediatamente el ejecutable optimizado (`-O2 -mwindows`).
