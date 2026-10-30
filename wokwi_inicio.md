# ESTAS SON LAS CONVERSACIONES QUE SE HIZO CON GEMINI Y EL DESARROLLO QUE SE TIENE HASTA EL MOMENTO EN WOKWI

El archivo diagram.json está configurado con el mapeo exacto de pines de tu proyecto (EstacionLogistica.ino) para que el circuito se monte automáticamente en Wokwi sin necesidad de cablear manualmente.   1. Código para diagram.jsonCopia y pega este bloque en la pestaña diagram.json de tu proyecto en Wokwi:

{
  "version": 1,
  "author": "Estacion Logistica Auto-TIF UNSA",
  "editor": "wokwi",
  "parts": [
    {
      "type": "wokwi-arduino-uno",
      "id": "uno",
      "top": 120,
      "left": 100,
      "attrs": {}
    },
    {
      "type": "wokwi-lcd1602",
      "id": "lcd1",
      "top": -40,
      "left": 450,
      "attrs": { "pins": "i2c" }
    },
    {
      "type": "wokwi-hc-sr04",
      "id": "ultrasonic1",
      "top": -60,
      "left": 80,
      "attrs": { "distance": "30" }
    },
    {
      "type": "wokwi-servo",
      "id": "servo1",
      "top": 110,
      "left": 480,
      "attrs": {}
    },
    {
      "type": "wokwi-photoresistor-sensor",
      "id": "ldr1",
      "top": 270,
      "left": -160,
      "attrs": {}
    },
    {
      "type": "wokwi-potentiometer",
      "id": "pot1",
      "top": 150,
      "left": -140,
      "attrs": { "value": "700" }
    },
    {
      "type": "wokwi-pushbutton",
      "id": "btn_estop",
      "top": 40,
      "left": -120,
      "attrs": { "color": "red", "label": "E-STOP (D2)" }
    },
    {
      "type": "wokwi-buzzer",
      "id": "bz1",
      "top": -30,
      "left": 330,
      "attrs": { "volume": "0.1" }
    },
    {
      "type": "wokwi-led",
      "id": "led_run",
      "top": 240,
      "left": 450,
      "attrs": { "color": "green", "label": "RUN (D11)" }
    },
    {
      "type": "wokwi-led",
      "id": "led_insp",
      "top": 240,
      "left": 510,
      "attrs": { "color": "yellow", "label": "INSP (D12)" }
    },
    {
      "type": "wokwi-led",
      "id": "led_alarm",
      "top": 240,
      "left": 570,
      "attrs": { "color": "red", "label": "ALARM (D13)" }
    },
    {
      "type": "wokwi-led",
      "id": "led_motor_pwm",
      "top": 340,
      "left": 450,
      "attrs": { "color": "cyan", "label": "Lazo1 ENA (D3)" }
    },
    {
      "type": "wokwi-led",
      "id": "led_motor_in1",
      "top": 340,
      "left": 510,
      "attrs": { "color": "blue", "label": "IN1 (D4)" }
    },
    {
      "type": "wokwi-led",
      "id": "led_light_pwm",
      "top": 340,
      "left": 570,
      "attrs": { "color": "white", "label": "Lazo3 Luz (D6)" }
    }
  ],
  "connections": [
    [ "uno:GND.1", "lcd1:GND", "black", [ "v-20", "h260" ] ],
    [ "uno:5V", "lcd1:VCC", "red", [ "v20", "h280", "v-280" ] ],
    [ "uno:A4", "lcd1:SDA", "green", [ "v30", "h240", "v-310" ] ],
    [ "uno:A5", "lcd1:SCL", "yellow", [ "v40", "h250", "v-310" ] ],

    [ "ultrasonic1:VCC", "uno:5V", "red", [ "v30", "h-60", "v220" ] ],
    [ "ultrasonic1:GND", "uno:GND.1", "black", [ "v20", "h50" ] ],
    [ "uno:8", "ultrasonic1:TRIG", "orange", [ "v-40", "h-120" ] ],
    [ "uno:9", "ultrasonic1:ECHO", "purple", [ "v-50", "h-100" ] ],

    [ "servo1:V+", "uno:5V", "red", [ "h-40", "v150" ] ],
    [ "servo1:GND", "uno:GND.2", "black", [ "h-50", "v160" ] ],
    [ "uno:5", "servo1:PWM", "orange", [ "v-20", "h180" ] ],

    [ "btn_estop:1.l", "uno:2", "red", [ "h0" ] ],
    [ "btn_estop:2.l", "uno:GND.1", "black", [ "h-20", "v60" ] ],

    [ "ldr1:VCC", "uno:5V", "red", [ "h40", "v40" ] ],
    [ "ldr1:GND", "uno:GND.2", "black", [ "h50", "v30" ] ],
    [ "ldr1:AO", "uno:A0", "green", [ "h60", "v10" ] ],

    [ "pot1:VCC", "uno:5V", "red", [ "v40", "h120" ] ],
    [ "pot1:GND", "uno:GND.2", "black", [ "v30", "h130" ] ],
    [ "pot1:SIG", "uno:A1", "blue", [ "v20", "h150" ] ],

    [ "uno:10", "bz1:2", "purple", [ "v-60", "h120" ] ],
    [ "uno:GND.1", "bz1:1", "black", [ "v-20", "h140" ] ],

    [ "uno:11", "led_run:A", "green", [ "v-10", "h220", "v150" ] ],
    [ "led_run:C", "uno:GND.2", "black", [ "v20", "h-220" ] ],

    [ "uno:12", "led_insp:A", "yellow", [ "v-15", "h280", "v155" ] ],
    [ "led_insp:C", "uno:GND.2", "black", [ "v20", "h-280" ] ],

    [ "uno:13", "led_alarm:A", "red", [ "v-20", "h340", "v160" ] ],
    [ "led_alarm:C", "uno:GND.2", "black", [ "v20", "h-340" ] ],

    [ "uno:3", "led_motor_pwm:A", "cyan", [ "v-5", "h190", "v240" ] ],
    [ "led_motor_pwm:C", "uno:GND.2", "black", [ "v20", "h-220" ] ],

    [ "uno:4", "led_motor_in1:A", "blue", [ "v-10", "h250", "v245" ] ],
    [ "led_motor_in1:C", "uno:GND.2", "black", [ "v20", "h-280" ] ],

    [ "uno:6", "led_light_pwm:A", "white", [ "v-15", "h310", "v250" ] ],
    [ "led_light_pwm:C", "uno:GND.2", "black", [ "v20", "h-340" ] ]
  ],
  "dependencies": {}
}


2. Archivos de Configuración en Wokwi (libraries.txt)Para que Wokwi compile sin errores las librerías de tu código (LiquidCrystal_I2C.h y Servo.h), ve a la pestaña Library Manager (o crea el archivo libraries.txt) y asegúrate de incluir exactamente estas dos líneas:   

LiquidCrystal I2C
Servo

(Nota: En el módulo sensor LDR de Wokwi, a mayor lux la tensión analógica baja por su configuración interna pull-up; al mover el deslizador de lux podrás variar la lectura en A0 de 0% a 100% sin inconvenientes).

3. Secuencia para Capturar las 4 Pantallas del Capítulo 4.1
Una vez pegado tu código EstacionLogistica.ino y el diagram.json, presiona el botón verde Start the Simulation y toma estas cuatro capturas para replicar la estructura del informe base:   
Desconocido
Captura 1 — Estado de Reposo (REPOSO) e Inicio:

Al arrancar, verás la pantalla LCD mostrando REPOSO | D: 30cm y el LED Amarillo (D12) encendido.   
MD

En el Monitor Serial inferior de Wokwi, escribe START y presiona Enter para iniciar la faja.   
MD
Captura 2 — Lazo 1 (Control de Velocidad y Desaceleración):

Con el sistema en TRANSP (LED Verde D11 activo), haz clic sobre el sensor HC-SR04 y mueve la barra de distancia de 30 cm hacia 12 cm.   
MD

Verás en el LCD cómo el valor de M: (PWM del motor en D3) baja proporcionalmente de 200 a 70 y el LED cian (Lazo1 ENA) reduce su brillo.   
MD

Captura 3 — Lazo 2 y Lazo 3 (Inspección Óptica y Desvío con Servomotor):

Baja la distancia del HC-SR04 a 8 cm (entre 5 cm y 10 cm). El sistema pasará automáticamente a INSPEC durante 1.5 segundos y luego a DESVIO.   
MD

Haz clic en el sensor LDR para cambiar la iluminación y verás cómo el brazo del Servomotor SG90 gira a 45°, 90° o 135° según la clasificación y cómo varía el PWM de iluminación en D6.   
MD

Captura 4 — Interrupción de Parada de Emergencia (EMERG!):

Presiona el botón rojo E-STOP (D2).   
MD

De inmediato el motor cae a M: 0, el LCD muestra EMERG!, suena el Buzzer (D10) y parpadea el LED Rojo (D13). Luego escribe RESET en el monitor serial para mostrar el rearme seguro.   
MD


Aquí tienes el código completo de sketch.ino con todas las impresiones seriales corregidas para que compile directamente en Wokwi sin errores:

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// =====================================================================================
// DEFINICIÓN DE PINES (HARDWARE MAP)
// =====================================================================================
#define PIN_EMERGENCY_STOP  2   // INT0
#define PIN_L298N_ENA       3   // PWM Lazo 1
#define PIN_L298N_IN1       4   // Control dirección DC
#define PIN_SERVO_GATE      5   // PWM Lazo 2 (Servo SG90)
#define PIN_LED_LIGHT_PWM   6   // PWM Lazo 3 (Luz de inspección)
#define PIN_L298N_IN2       7   // Control dirección DC
#define PIN_HCSR04_TRIG     8   // Trigger Ultrasonido
#define PIN_HCSR04_ECHO     9   // Echo Ultrasonido
#define PIN_BUZZER          10  // Alarma acústica
#define PIN_LED_RUN         11  // Indicador Verde
#define PIN_LED_INSPECT     12  // Indicador Amarillo
#define PIN_LED_ALARM       13  // Indicador Rojo
#define PIN_LDR_SENSOR      A0  // Analógico LDR
#define PIN_POT_CALIB       A1  // Analógico Potenciómetro

// =====================================================================================
// CONSTANTES DE CONTROL Y CONFIGURACIÓN
// =====================================================================================
#define SERIAL_BAUD_RATE    9600
#define LCD_I2C_ADDR        0x27
#define LCD_COLS            16
#define LCD_ROWS            2

// Lazo 1: Transporte y Anticolisión
#define DIST_MIN_STOP_CM    5.0f   // Distancia crítica de parada total (cm)
#define DIST_MAX_NOMINAL_CM 25.0f  // Distancia segura nominal (cm)
#define PWM_MOTOR_NOMINAL   200    // Velocidad nominal de transporte
#define PWM_MOTOR_MIN       0      // Parada completa

// Lazo 2: Servomotor de Enrutamiento
#define SERVO_ANG_RETRABAJO 45     // Tolva de Retrabajo
#define SERVO_ANG_CONFORME  90     // Tolva Conforme / Paso Directo
#define SERVO_ANG_RECHAZO   135    // Tolva de Rechazo
#define TIEMPO_DESVIO_MS    2000   // Tiempo antes de retornar compuerta a neutro

// Lazo 3: Iluminación de Inspección
#define SP_LUX_DEFAULT      70.0f  // Setpoint nominal de iluminancia (0 - 100%)
#define PWM_LIGHT_BASE      128    // Salida base a 50%
#define KP_LIGHT            2.5f   // Ganancia Proporcional de Iluminación

// Intervalos de Tareas No Bloqueantes (Task Scheduler basado en millis())
#define PERIOD_ULTRASONIC_MS 60    // Muestreo HC-SR04 (no bloqueante)
#define PERIOD_CONTROL_MS    50    // Lazos de control 1 y 3
#define PERIOD_TELEMETRY_MS  200   // Envío de tramas por Serial/Bluetooth
#define PERIOD_LCD_MS        250   // Refresco de pantalla LCD
#define PERIOD_FSM_MS        50    // Evaluación periódica de la FSM
#define PERIOD_ALARM_BLINK_MS 200  // Parpadeo visual/acústico en Emergencia

// =====================================================================================
// ENUMERACIONES Y ESTRUCTURAS
// =====================================================================================
enum EstadoSistema {
    ESTADO_INIT,
    ESTADO_REPOSO,
    ESTADO_TRANSPORTE,
    ESTADO_INSPECCION,
    ESTADO_DESVIO,
    ESTADO_EMERGENCIA
};

enum ClasificacionPieza {
    PIEZA_NINGUNA,
    PIEZA_CONFORME,
    PIEZA_RECHAZO,
    PIEZA_RETRABAJO
};

// =====================================================================================
// VARIABLES GLOBALES Y DE ESTADO
// =====================================================================================
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
Servo servoCompuerta;

EstadoSistema estadoActual = ESTADO_INIT;
ClasificacionPieza piezaClasificada = PIEZA_NINGUNA;

volatile bool paradaEmergenciaActivada = false;
volatile bool flancoEmergenciaDetectado = false;

float pvDistanciaCm = 50.0f;
int   cvPwmMotor = 0;

float spIluminacionPct = SP_LUX_DEFAULT;
float pvIluminacionPct = 0.0f;
int   cvPwmIluminacion = 0;

int   cvAnguloServo = SERVO_ANG_CONFORME;

unsigned long tLastUltrasonic = 0;
unsigned long tLastControl    = 0;
unsigned long tLastTelemetry  = 0;
unsigned long tLastLCD        = 0;
unsigned long tLastFSM        = 0;
unsigned long tLastBlinkAlarm = 0;
unsigned long tInicioDesvio   = 0;
unsigned long tInicioInspeccion = 0;

bool estadoBlinkAlarma = false;
bool simulacionActiva = false;

// =====================================================================================
// DECLARACIÓN DE FUNCIONES / PROTOTIPOS
// =====================================================================================
void isrParadaEmergencia();
void tareaMedicionUltrasonido();
void tareaLazosControl();
void tareaMaquinaEstados();
void tareaActualizarHMI();
void tareaTelemetriaSerial();
void procesarComandosSeriales();
void actuarMotores(int pwmVelocidad, bool marchaAdelante);
void actuarIluminacion(int pwmBrillo);
void actuarServo(int angulo);
void setSenalizadores(bool verde, bool amarillo, bool rojo, bool buzzer);
float leerLDRPorcentaje();
const char* obtenerNombreEstado(EstadoSistema st);

// =====================================================================================
// SETUP: INICIALIZACIÓN DEL SISTEMA
// =====================================================================================
void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial && millis() < 1000);

    pinMode(PIN_EMERGENCY_STOP, INPUT_PULLUP);
    pinMode(PIN_L298N_ENA, OUTPUT);
    pinMode(PIN_L298N_IN1, OUTPUT);
    pinMode(PIN_L298N_IN2, OUTPUT);
    pinMode(PIN_LED_LIGHT_PWM, OUTPUT);
    pinMode(PIN_HCSR04_TRIG, OUTPUT);
    pinMode(PIN_HCSR04_ECHO, INPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED_RUN, OUTPUT);
    pinMode(PIN_LED_INSPECT, OUTPUT);
    pinMode(PIN_LED_ALARM, OUTPUT);

    actuarMotores(0, true);
    actuarIluminacion(0);
    setSenalizadores(false, false, false, false);

    servoCompuerta.attach(PIN_SERVO_GATE);
    actuarServo(SERVO_ANG_CONFORME);

    Wire.begin();
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("ESTACION LOGIST."));
    lcd.setCursor(0, 1);
    lcd.print(F("INIT SISTEMA..."));

    attachInterrupt(digitalPinToInterrupt(PIN_EMERGENCY_STOP), isrParadaEmergencia, FALLING);

    delay(500);
    estadoActual = ESTADO_REPOSO;
    lcd.clear();
    
    Serial.println(F("{\"MSG\":\"SISTEMA_INICIADO\",\"STATUS\":\"OK\",\"BAUD\":9600}"));
}

// =====================================================================================
// LOOP PRINCIPAL: GESTOR DE TAREAS COOPERATIVO (NON-BLOCKING SCHEDULER)
// =====================================================================================
void loop() {
    unsigned long now = millis();

    // 1. Verificación prioritaria de señal de Emergencia proveniente de ISR
    if (paradaEmergenciaActivada && estadoActual != ESTADO_EMERGENCIA) {
        estadoActual = ESTADO_EMERGENCIA;
        actuarMotores(0, false);
        actuarIluminacion(0);
        Serial.print(F("{\"ALERTA\":\"PARADA_EMERGENCIA_ACTIVADA\",\"TIMESTAMP_MS\":"));
        Serial.print(now);
        Serial.println(F("}"));
    }

    // 2. Procesamiento de Comandos Seriales / Bluetooth entrantes
    procesarComandosSeriales();

    // 3. Tarea: Muestreo No Bloqueante del Sensor Ultrasónico (60 ms)
    if (now - tLastUltrasonic >= PERIOD_ULTRASONIC_MS) {
        tLastUltrasonic = now;
        tareaMedicionUltrasonido();
    }

    // 4. Tarea: Ejecución de Lazos de Control Cerrados (50 ms)
    if (now - tLastControl >= PERIOD_CONTROL_MS) {
        tLastControl = now;
        tareaLazosControl();
    }

    // 5. Tarea: Máquina de Estados Finitos (50 ms)
    if (now - tLastFSM >= PERIOD_FSM_MS) {
        tLastFSM = now;
        tareaMaquinaEstados();
    }

    // 6. Tarea: Actualización de Pantalla LCD (250 ms)
    if (now - tLastLCD >= PERIOD_LCD_MS) {
        tLastLCD = now;
        tareaActualizarHMI();
    }

    // 7. Tarea: Telemetría Serial/Bluetooth (200 ms)
    if (now - tLastTelemetry >= PERIOD_TELEMETRY_MS) {
        tLastTelemetry = now;
        tareaTelemetriaSerial();
    }

    // 8. Tarea: Gestión de Destellos de Alarma en Estado de Emergencia
    if (estadoActual == ESTADO_EMERGENCIA) {
        if (now - tLastBlinkAlarm >= PERIOD_ALARM_BLINK_MS) {
            tLastBlinkAlarm = now;
            estadoBlinkAlarma = !estadoBlinkAlarma;
            digitalWrite(PIN_LED_ALARM, estadoBlinkAlarma ? HIGH : LOW);
            digitalWrite(PIN_BUZZER, estadoBlinkAlarma ? HIGH : LOW);
        }
    }
}

// =====================================================================================
// RUTINA DE INTERRUPCIÓN (ISR) - PARADA DE EMERGENCIA
// =====================================================================================
void isrParadaEmergencia() {
    paradaEmergenciaActivada = true;
    flancoEmergenciaDetectado = true;
    
    analogWrite(PIN_L298N_ENA, 0);
    digitalWrite(PIN_L298N_IN1, LOW);
    digitalWrite(PIN_L298N_IN2, LOW);
    analogWrite(PIN_LED_LIGHT_PWM, 0);
}

// =====================================================================================
// MEDICIÓN DE DISTANCIA NO BLOQUEANTE (HC-SR04)
// =====================================================================================
void tareaMedicionUltrasonido() {
    if (simulacionActiva) {
        return;
    }

    digitalWrite(PIN_HCSR04_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_HCSR04_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_HCSR04_TRIG, LOW);

    unsigned long duracion = pulseIn(PIN_HCSR04_ECHO, HIGH, 18000);

    if (duracion == 0) {
        pvDistanciaCm = 100.0f;
    } else {
        float distanciaCalculada = (duracion * 0.0343f) / 2.0f;
        pvDistanciaCm = (0.7f * distanciaCalculada) + (0.3f * pvDistanciaCm);
    }
}

// =====================================================================================
// LAZOS DE CONTROL CERRADOS (1, 2 Y 3)
// =====================================================================================
void tareaLazosControl() {
    if (estadoActual == ESTADO_EMERGENCIA) {
        cvPwmMotor = 0;
        cvPwmIluminacion = 0;
        actuarMotores(0, false);
        actuarIluminacion(0);
        return;
    }

    // LAZO 1: Regulación Proporcional de Velocidad y Frenado Anticolisión
    if (estadoActual == ESTADO_TRANSPORTE) {
        if (pvDistanciaCm <= DIST_MIN_STOP_CM) {
            cvPwmMotor = PWM_MOTOR_MIN;
        } else if (pvDistanciaCm >= DIST_MAX_NOMINAL_CM) {
            cvPwmMotor = PWM_MOTOR_NOMINAL;
        } else {
            float factorProporcional = (pvDistanciaCm - DIST_MIN_STOP_CM) / (DIST_MAX_NOMINAL_CM - DIST_MIN_STOP_CM);
            cvPwmMotor = (int)(factorProporcional * PWM_MOTOR_NOMINAL);
            cvPwmMotor = constrain(cvPwmMotor, PWM_MOTOR_MIN, PWM_MOTOR_NOMINAL);
        }
        actuarMotores(cvPwmMotor, true);
    } else {
        cvPwmMotor = 0;
        actuarMotores(0, true);
    }

    // LAZO 3: Estabilización Proporcional de Iluminación de Inspección
    if (!simulacionActiva) {
        pvIluminacionPct = leerLDRPorcentaje();
    }

    int lecturaPot = analogRead(PIN_POT_CALIB);
    spIluminacionPct = map(lecturaPot, 0, 1023, 20, 95);

    if (estadoActual == ESTADO_INSPECCION || estadoActual == ESTADO_TRANSPORTE) {
        float errorLuz = spIluminacionPct - pvIluminacionPct;
        float calculoPwm = PWM_LIGHT_BASE + (KP_LIGHT * errorLuz * 2.55f);
        cvPwmIluminacion = constrain((int)calculoPwm, 0, 255);
        actuarIluminacion(cvPwmIluminacion);
    } else {
        cvPwmIluminacion = 25;
        actuarIluminacion(cvPwmIluminacion);
    }
}

// =====================================================================================
// MÁQUINA DE ESTADOS FINITOS (FSM)
// =====================================================================================
void tareaMaquinaEstados() {
    unsigned long now = millis();

    switch (estadoActual) {
        case ESTADO_INIT:
            setSenalizadores(false, false, false, false);
            estadoActual = ESTADO_REPOSO;
            break;

        case ESTADO_REPOSO:
            setSenalizadores(false, true, false, false);
            actuarServo(SERVO_ANG_CONFORME);
            break;

        case ESTADO_TRANSPORTE:
            setSenalizadores(true, false, false, false);
            if (pvDistanciaCm > 5.0f && pvDistanciaCm <= 10.0f) {
                estadoActual = ESTADO_INSPECCION;
                tInicioInspeccion = now;
                Serial.println(F("{\"EVENTO\":\"PIEZA_EN_INSPECCION\"}"));
            }
            break;

        case ESTADO_INSPECCION:
            setSenalizadores(false, true, false, false);
            if (now - tInicioInspeccion >= 1500) {
                if (piezaClasificada == PIEZA_NINGUNA) {
                    if (pvIluminacionPct >= 65.0f) {
                        piezaClasificada = PIEZA_CONFORME;
                    } else if (pvIluminacionPct >= 40.0f) {
                        piezaClasificada = PIEZA_RETRABAJO;
                    } else {
                        piezaClasificada = PIEZA_RECHAZO;
                    }
                }
                
                estadoActual = ESTADO_DESVIO;
                tInicioDesvio = now;

                if (piezaClasificada == PIEZA_RETRABAJO) {
                    cvAnguloServo = SERVO_ANG_RETRABAJO;
                } else if (piezaClasificada == PIEZA_RECHAZO) {
                    cvAnguloServo = SERVO_ANG_RECHAZO;
                } else {
                    cvAnguloServo = SERVO_ANG_CONFORME;
                }
                actuarServo(cvAnguloServo);

                Serial.print(F("{\"EVENTO\":\"CLASIFICACION_COMPLETA\",\"RESULTADO\":\""));
                Serial.print((piezaClasificada == PIEZA_CONFORME) ? F("CONFORME") : 
                             (piezaClasificada == PIEZA_RETRABAJO) ? F("RETRABAJO") : F("RECHAZO"));
                Serial.print(F("\",\"ANGULO\":"));
                Serial.print(cvAnguloServo);
                Serial.println(F("}"));
            }
            break;

        case ESTADO_DESVIO:
            setSenalizadores(true, true, false, false);
            if (now - tInicioDesvio >= TIEMPO_DESVIO_MS) {
                actuarServo(SERVO_ANG_CONFORME);
                piezaClasificada = PIEZA_NINGUNA;
                estadoActual = ESTADO_TRANSPORTE;
                Serial.println(F("{\"EVENTO\":\"COMPUERTA_RESTABLECIDA_NEUTRO\"}"));
            }
            break;

        case ESTADO_EMERGENCIA:
            break;
    }
}

// =====================================================================================
// INTERFAZ DE USUARIO HMI (PANTALLA LCD 16x2 I2C)
// =====================================================================================
void tareaActualizarHMI() {
    lcd.setCursor(0, 0);
    char linea0[17];
    const char* nombreEst = obtenerNombreEstado(estadoActual);
    int distInt = (int)pvDistanciaCm;
    snprintf(linea0, sizeof(linea0), "%-6s|D:%3dcm ", nombreEst, distInt);
    lcd.print(linea0);

    lcd.setCursor(0, 1);
    char linea1[17];
    int luxInt = (int)pvIluminacionPct;
    snprintf(linea1, sizeof(linea1), "M:%3d L:%2d%% S:%3d", cvPwmMotor, luxInt, cvAnguloServo);
    lcd.print(linea1);
}

// =====================================================================================
// TELEMETRÍA SERIAL / BLUETOOTH HC-05
// =====================================================================================
void tareaTelemetriaSerial() {
    Serial.print(F("TLM,"));
    Serial.print(obtenerNombreEstado(estadoActual));
    Serial.print(F(","));
    Serial.print(pvDistanciaCm, 1);
    Serial.print(F(","));
    Serial.print(cvPwmMotor);
    Serial.print(F(","));
    Serial.print(pvIluminacionPct, 1);
    Serial.print(F(","));
    Serial.print(cvAnguloServo);
    Serial.print(F(","));
    Serial.println(paradaEmergenciaActivada ? 1 : 0);
}

// =====================================================================================
// PARSER DE COMANDOS SERIALES / BLUETOOTH
// =====================================================================================
void procesarComandosSeriales() {
    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd.length() == 0) return;

    if (cmd == F("START")) {
        if (estadoActual == ESTADO_REPOSO) {
            estadoActual = ESTADO_TRANSPORTE;
            Serial.println(F("{\"RESP\":\"CMD_OK\",\"ACCION\":\"INICIAR_TRANSPORTE\"}"));
        } else {
            Serial.println(F("{\"RESP\":\"CMD_ERROR\",\"MOTIVO\":\"ESTADO_NO_VALIDO\"}"));
        }
    } else if (cmd == F("STOP")) {
        if (estadoActual != ESTADO_EMERGENCIA) {
            estadoActual = ESTADO_REPOSO;
            actuarMotores(0, false);
            Serial.println(F("{\"RESP\":\"CMD_OK\",\"ACCION\":\"DETENER_EN_REPOSO\"}"));
        }
    } else if (cmd == F("RESET")) {
        if (digitalRead(PIN_EMERGENCY_STOP) == HIGH) {
            paradaEmergenciaActivada = false;
            flancoEmergenciaDetectado = false;
            estadoActual = ESTADO_REPOSO;
            setSenalizadores(false, true, false, false);
            actuarServo(SERVO_ANG_CONFORME);
            Serial.println(F("{\"RESP\":\"REARME_EXITOSO\",\"NUEVO_ESTADO\":\"REPOSO\"}"));
        } else {
            Serial.println(F("{\"RESP\":\"REARME_FALLIDO\",\"MOTIVO\":\"PULSADOR_AUN_PRESIONADO\"}"));
        }
    } else if (cmd.startsWith(F("CLASSIFY:"))) {
        String tipo = cmd.substring(9);
        if (tipo == F("CONFORME")) {
            piezaClasificada = PIEZA_CONFORME;
        } else if (tipo == F("RECHAZO")) {
            piezaClasificada = PIEZA_RECHAZO;
        } else if (tipo == F("RETRABAJO")) {
            piezaClasificada = PIEZA_RETRABAJO;
        }
        Serial.print(F("{\"RESP\":\"CMD_OK\",\"CLASIFICACION_ASIGNADA\":\""));
        Serial.print(tipo);
        Serial.println(F("\"}"));
    } else if (cmd.startsWith(F("SIM_DIST:"))) {
        simulacionActiva = true;
        pvDistanciaCm = cmd.substring(9).toFloat();
        Serial.print(F("{\"RESP\":\"SIM_OK\",\"DIST_FORZADA\":"));
        Serial.print(pvDistanciaCm);
        Serial.println(F("}"));
    } else if (cmd.startsWith(F("SIM_LDR:"))) {
        simulacionActiva = true;
        pvIluminacionPct = cmd.substring(8).toFloat();
        Serial.print(F("{\"RESP\":\"SIM_OK\",\"LUX_FORZADO\":"));
        Serial.print(pvIluminacionPct);
        Serial.println(F("}"));
    } else if (cmd == F("SIM_OFF")) {
        simulacionActiva = false;
        Serial.println(F("{\"RESP\":\"SIM_OFF\",\"MODO\":\"HARDWARE_REAL\"}"));
    } else {
        Serial.print(F("{\"RESP\":\"COMANDO_DESCONOCIDO\",\"CMD\":\""));
        Serial.print(cmd);
        Serial.println(F("\"}"));
    }
}

// =====================================================================================
// CONTROLADORES DE ACTUADORES FÍSICOS
// =====================================================================================
void actuarMotores(int pwmVelocidad, bool marchaAdelante) {
    if (pwmVelocidad <= 0) {
        analogWrite(PIN_L298N_ENA, 0);
        digitalWrite(PIN_L298N_IN1, LOW);
        digitalWrite(PIN_L298N_IN2, LOW);
    } else {
        if (marchaAdelante) {
            digitalWrite(PIN_L298N_IN1, HIGH);
            digitalWrite(PIN_L298N_IN2, LOW);
        } else {
            digitalWrite(PIN_L298N_IN1, LOW);
            digitalWrite(PIN_L298N_IN2, HIGH);
        }
        analogWrite(PIN_L298N_ENA, constrain(pwmVelocidad, 0, 255));
    }
}

void actuarIluminacion(int pwmBrillo) {
    analogWrite(PIN_LED_LIGHT_PWM, constrain(pwmBrillo, 0, 255));
}

void actuarServo(int angulo) {
    cvAnguloServo = constrain(angulo, 0, 180);
    servoCompuerta.write(cvAnguloServo);
}

void setSenalizadores(bool verde, bool amarillo, bool rojo, bool buzzer) {
    digitalWrite(PIN_LED_RUN, verde ? HIGH : LOW);
    digitalWrite(PIN_LED_INSPECT, amarillo ? HIGH : LOW);
    digitalWrite(PIN_LED_ALARM, rojo ? HIGH : LOW);
    digitalWrite(PIN_BUZZER, buzzer ? HIGH : LOW);
}

float leerLDRPorcentaje() {
    int adcVal = analogRead(PIN_LDR_SENSOR);
    float pct = (adcVal / 1023.0f) * 100.0f;
    return constrain(pct, 0.0f, 100.0f);
}

const char* obtenerNombreEstado(EstadoSistema st) {
    switch (st) {
        case ESTADO_INIT:        return "INIT";
        case ESTADO_REPOSO:      return "REPOSO";
        case ESTADO_TRANSPORTE:  return "TRANSP";
        case ESTADO_INSPECCION:  return "INSPEC";
        case ESTADO_DESVIO:      return "DESVIO";
        case ESTADO_EMERGENCIA:  return "EMERG!";
        default:                 return "DESCON";
    }
}

Reemplaza todo el contenido de sketch.ino con este bloque y presiona Start Simulation; compilará e iniciará el circuito de inmediato.

La simulación en Wokwi replica exactamente el comportamiento de la estación logística mediante sensores interactivos, indicadores luminosos y el Monitor Serial. Sigue este flujo operativo para probar cada subsistema y ver cómo responde el circuito:

---

### 1. Cómo equivalen los LEDs de Wokwi a tus actuadores reales

Dado que Wokwi no incluye un motor DC con puente H L298N ni una cabina de iluminación real, el esquema utiliza **3 LEDs inferiores** para representar visualmente las señales de potencia que salen del Arduino:

* **LED Cian (`Lazo1 ENA - Pin D3`):** Representa la **velocidad del motor DC de la faja** (`M:` en el LCD). A mayor PWM (hasta 200), brilla más; cuando desacelera, su brillo baja proporcionalmente; cuando el motor frena (`M: 0`), se apaga por completo.


* **LED Azul (`IN1 - Pin D4`):** Indica que el puente H tiene habilitado el **giro hacia adelante** de la faja. Solo se enciende cuando el motor está en movimiento.


* **LED Blanco (`Lazo3 Luz - Pin D6`):** Representa la **potencia de la lámpara LED de la cabina óptica** controlada por el transistor TIP122. Su brillo sube o baja automáticamente para mantener estable la iluminación.



Los **3 LEDs superiores** son las balizas de estado de la planta:

* **Verde (`RUN - Pin D11`):** Faja transportadora en movimiento (`TRANSP`).


* **Amarillo (`INSP - Pin D12`):** Sistema en espera (`REPOSO`) o evaluando una caja (`INSPEC`).


* **Rojo (`ALARM - Pin D13`) + Buzzer (`Pin D10`):** Parpadeo y alarma sonora en parada de emergencia (`EMERG!`).



---

### 2. Guía paso a paso para operar la simulación

#### Paso 1: Arrancar la faja (Salir de `REPOSO`)

1. Al darle al botón verde **Start Simulation**, el sistema inicia por seguridad en modo **`REPOSO`** (verás `REPOSO | D: 30cm` en la pantalla LCD y el LED amarillo encendido). En este estado el motor está apagado (`M: 0`).


2. Haz clic en la caja de texto del **Monitor Serial** (abajo a la derecha en Wokwi), escribe **`START`** y presiona **Enter**.


3. El estado cambiará a **`TRANSP`**, se encenderá el **LED Verde (`RUN`)**, y como la distancia inicial es de 30 cm (mayor a 25 cm), el motor arrancará a su velocidad nominal máxima (`M: 200` en el LCD, con los LEDs Cian y Azul encendidos).



#### Paso 2: Probar el Lazo 1 (Desaceleración por distancia con el HC-SR04)

1. Mientras está en `TRANSP`, haz clic sobre el **sensor ultrasónico HC-SR04** (aparecerá una barra deslizante de distancia en cm).


2. Desliza la distancia lentamente desde **30 cm hacia 12 cm**:
* Verás en la segunda fila del LCD cómo el valor de **`M:`** baja en tiempo real (por ejemplo, a 15 cm marca `M:100`, a 12 cm marca `M: 70`) siguiendo la rampa proporcional $PWM = 10 \times (d - 5)$.





#### Paso 3: Probar el Lazo 2 (Inspección y desvío de cajas con el Servomotor)

1. Mueve el deslizador del **HC-SR04** a cualquier valor **entre 6 cm y 10 cm** (por ejemplo, **8 cm**). Esto simula que la caja llegó exactamente a la cabina de inspección.


2. Observarás la secuencia automática de la Máquina de Estados (FSM):


* El sistema pasa a **`INSPEC`**, detiene la faja (`M: 0`) y espera **1.5 segundos** midiendo la luz del sensor LDR.


* Cumplidos los 1.5 segundos, pasa al estado **`DESVIO`** y mueve el brazo del **Servomotor SG90** según el porcentaje de luz (`L:`) que tenga el LDR en ese instante:


* Si `L:` es **65% o más** $\rightarrow$ Pieza **CONFORME** (Servo en **90°**).


* Si `L:` está **entre 40% y 64%** $\rightarrow$ Pieza a **RETRABAJO** (Servo gira a **45°**).


* Si `L:` es **menor a 40%** $\rightarrow$ Pieza a **RECHAZO** (Servo gira a **135°**).




* Tras **2 segundos** de expulsión, el servo regresa solo a **90°** y el sistema vuelve a `TRANSP`. *(Sube la distancia del HC-SR04 a más de 15 cm para simular que la caja ya salió y que la faja vuelva a correr).*





#### Paso 4: Probar el Lazo 3 (Compensación de iluminación con LDR y Potenciómetro)

1. Haz clic sobre el módulo **LDR (fotorresistencia)** y mueve su deslizador de luz (*Lux*):
* Nota: En el módulo estándar de Wokwi, al subir el deslizador de Lux baja el voltaje analógico en `A0` (y al bajar los Lux sube el porcentaje `L:` en el LCD).


2. Al variar el deslizador del LDR o girar la perilla del **Potenciómetro (`A1`)** (que ajusta el Setpoint deseado entre 20% y 95%), el controlador calcula la diferencia y ajusta inmediatamente el brillo del **LED Blanco (`D6`)** para compensar la falta o exceso de luz.



#### Paso 5: Probar la Parada de Emergencia por Interrupción (`E-STOP`)

1. En cualquier momento de la simulación, haz clic sobre el **botón rojo `E-STOP (D2)**`.


2. Sin importar en qué tarea esté el programa, la interrupción por hardware (`INT0`) corta el motor (`M: 0`) y la luz al instante, el LCD muestra **`EMERG!`**, parpadea el **LED Rojo (`ALARM`)** y suena el **Buzzer**.


3. Para rearmar la estación, escribe **`RESET`** en el Monitor Serial y presiona Enter: la alarma se apaga y el sistema regresa de forma segura a **`REPOSO`**.





