/* =====================================================================================
 * PROYECTO: ESTACIÓN LOGÍSTICA DE SELECCIÓN, CLASIFICACIÓN Y ENRUTAMIENTO INDUSTRIAL
 * PLATAFORMA: ARDUINO UNO R3 (ATmega328P)
 * AUTOR: Senior Industrial Automation & Embedded Systems Engineer
 * ARQUITECTURA: Multitarea Cooperativa No Bloqueante (FSM + 3 Lazos de Control Cerrados)
 * =====================================================================================
 * 
 * ASIGNACIÓN ESTRICTA DE PINES:
 *   - Pin 0 (RX), Pin 1 (TX) : Telemetría Serial / Bluetooth HC-05 (9600 baudios)
 *   - Pin 2 (INT0)          : Pulsador Parada de Emergencia (FALLING, INPUT_PULLUP)
 *   - Pin 3 (PWM / OC2B)    : ENA Driver L298N (Lazo 1: Control Velocidad Faja DC)
 *   - Pin 4                 : IN1 Driver L298N (Dirección Motor DC)
 *   - Pin 5 (PWM / OC0B)    : Servomotor SG90 (Lazo 2: Enrutamiento Angular)
 *   - Pin 6 (PWM / OC0A)    : Base Transistor NPN (Lazo 3: Iluminación LED Inspección)
 *   - Pin 7                 : IN2 Driver L298N (Dirección Motor DC)
 *   - Pin 8                 : Trigger Sensor Ultrasónico HC-SR04
 *   - Pin 9 (PCINT1)        : Echo Sensor Ultrasónico HC-SR04 (Medición No Bloqueante)
 *   - Pin 10                : Buzzer Activo de Alarma Sonora
 *   - Pin 11                : LED Verde (Marcha / En Proceso)
 *   - Pin 12                : LED Amarillo (Inspección / Espera)
 *   - Pin 13                : LED Rojo (Alarma / Emergencia)
 *   - Pin A0                : Fotorresistencia LDR (Divisor 10k a GND - Lazo 3)
 *   - Pin A1                : Potenciómetro de Calibración / Setpoint
 *   - Pin A4 (SDA), A5 (SCL): Bus I2C Pantalla LCD 16x2 (Dirección 0x27)
 * ===================================================================================== */

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

// Lazo 1: Transporte y Anticolisión (Calibrado 1:1 con simulación Python)
#define DIST_MIN_STOP_CM    11.5f  // Distancia de parada para inspección (cm)
#define DIST_MAX_NOMINAL_CM 20.0f  // Distancia inicio de desaceleración nominal (cm)
#define PWM_MOTOR_NOMINAL   200    // Velocidad nominal de transporte
#define PWM_MOTOR_MIN_RAMP  80     // Velocidad mínima en rampa proporcional
#define PWM_MOTOR_EXPULSION 180    // Velocidad de expulsión en desvío
#define PWM_MOTOR_MIN       0      // Parada completa

// Lazo 2: Servomotor de Enrutamiento
#define SERVO_ANG_RETRABAJO 45     // Tolva de Retrabajo
#define SERVO_ANG_CONFORME  90     // Tolva Conforme / Paso Directo
#define SERVO_ANG_RECHAZO   135    // Tolva de Rechazo
#define TIEMPO_INSPECCION_MS 800   // Duración de inspección (0.8s) idéntica a Python
#define TIEMPO_DESVIO_MS    1000   // Tiempo de expulsión (1.0s) idéntico a Python

// Lazo 3: Iluminación de Inspección
#define SP_LUX_DEFAULT      70.0f  // Setpoint nominal de iluminancia (0 - 100%)
#define PWM_LIGHT_BASE      128    // Salida base a 50%
#define KP_LIGHT            2.5f   // Ganancia Proporcional de Iluminación

// Intervalos de Tareas No Bloqueantes (Task Scheduler basado en millis())
#define PERIOD_ULTRASONIC_MS 60    // Muestreo HC-SR04 (no bloqueante)
#define PERIOD_CONTROL_MS    50    // Lazos de control 1 y 3
#define PERIOD_TELEMETRY_MS  30    // Envío de tramas periódicas por Serial/Bluetooth (33 Hz)
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
// Instancias de periféricos
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);
Servo servoCompuerta;

// Estado de la Máquina de Estados Finitos
EstadoSistema estadoActual = ESTADO_INIT;
ClasificacionPieza piezaClasificada = PIEZA_NINGUNA;

// Variable de Interrupción Atómica
volatile bool paradaEmergenciaActivada = false;
volatile bool flancoEmergenciaDetectado = false;

// Variables de Proceso (PV, SP, CV)
float pvDistanciaCm = 50.0f;
int   cvPwmMotor = 0;

float spIluminacionPct = SP_LUX_DEFAULT;
float pvIluminacionPct = 0.0f;
int   cvPwmIluminacion = 0;

int   cvAnguloServo = SERVO_ANG_CONFORME;

// Temporizadores de Tareas
unsigned long tLastUltrasonic = 0;
unsigned long tLastControl    = 0;
unsigned long tLastTelemetry  = 0;
unsigned long tLastLCD        = 0;
unsigned long tLastFSM        = 0;
unsigned long tLastBlinkAlarm = 0;
unsigned long tInicioDesvio   = 0;
unsigned long tInicioInspeccion = 0;

// Estado de señalizadores
bool estadoBlinkAlarma = false;
bool modoInspeccionManual = false;
bool simulacionActiva = false; // Permite sobreescribir entradas por comando Serial
bool zonaInspeccionArmada = true; // Previene re-inspección cíclica de la misma pieza

// Variables auxiliares para HC-SR04 No Bloqueante
volatile unsigned long echoStartTime = 0;
volatile unsigned long echoEndTime = 0;
volatile bool echoRecibido = false;
volatile bool esperandoEcho = false;

// =====================================================================================
// DECLARACIÓN DE FUNCIONES / PROTOTIPOS
// =====================================================================================
void isrParadaEmergencia();
void inicializarHardware();
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
    // 1. Inicialización de Comunicación Serial (Telemetría / Bluetooth HC-05)
    Serial.begin(SERIAL_BAUD_RATE);
    while (!Serial && millis() < 1000); // Espera breve no crítica para USB nativo

    // 2. Configuración de Entradas y Salidas Digitales
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

    // Estado seguro inmediato de actuadores
    actuarMotores(0, true);
    actuarIluminacion(0);
    setSenalizadores(false, false, false, false);

    // 3. Inicialización del Servomotor SG90
    servoCompuerta.attach(PIN_SERVO_GATE);
    actuarServo(SERVO_ANG_CONFORME);

    // 4. Inicialización de Pantalla LCD I2C
    Wire.begin();
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("ESTACION LOGIST."));
    lcd.setCursor(0, 1);
    lcd.print(F("INIT SISTEMA..."));

    // 5. Configuración de Interrupción Externa INT0 (Pin 2)
    // Se activa por flanco de bajada (FALLING) con resistencia PULL-UP interna
    attachInterrupt(digitalPinToInterrupt(PIN_EMERGENCY_STOP), isrParadaEmergencia, FALLING);

    // 6. Transición a Estado Inicial
    delay(500); // Retardo controlado solo en arranque
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
        // Corte instantáneo de actuadores
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
    // Corte de hardware instantáneo mediante registros directos o variables atómicas
    paradaEmergenciaActivada = true;
    flancoEmergenciaDetectado = true;
    
    // Forzado a 0 de las salidas PWM críticas de manera directa
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
        // En modo simulación, el valor de pvDistanciaCm es provisto por el emulador
        return;
    }

    // Disparo de pulso Trigger de 10 microsegundos
    digitalWrite(PIN_HCSR04_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_HCSR04_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_HCSR04_TRIG, LOW);

    // Medición con timeout de 18ms (equivale a ~3 metros) para evitar bloqueo de ciclo
    unsigned long duracion = pulseIn(PIN_HCSR04_ECHO, HIGH, 18000);

    if (duracion == 0) {
        // Si no hubo eco en el rango, se asume distancia libre máxima (> 100 cm)
        pvDistanciaCm = 100.0f;
    } else {
        // Distancia = (Tiempo * VelocidadSonido 0.0343 cm/us) / 2
        float distanciaCalculada = (duracion * 0.0343f) / 2.0f;
        // Filtro pasa-bajas exponencial suave (alfa = 0.7) para rechazar ruido acústico
        pvDistanciaCm = (0.7f * distanciaCalculada) + (0.3f * pvDistanciaCm);
    }
}

// =====================================================================================
// LAZOS DE CONTROL CERRADOS (1, 2 Y 3)
// =====================================================================================
void tareaLazosControl() {
    // Si estamos en emergencia, los lazos de control quedan inhibidos
    if (estadoActual == ESTADO_EMERGENCIA) {
        cvPwmMotor = 0;
        cvPwmIluminacion = 0;
        actuarMotores(0, false);
        actuarIluminacion(0);
        return;
    }

    // ---------------------------------------------------------------------------------
    // LAZO 1: Regulación Proporcional de Velocidad y Frenado Anticolisión
    // ---------------------------------------------------------------------------------
    if (estadoActual == ESTADO_TRANSPORTE) {
        if (pvDistanciaCm <= DIST_MIN_STOP_CM) {
            // Parada total en zona de inspección
            cvPwmMotor = PWM_MOTOR_MIN;
        } else if (pvDistanciaCm >= DIST_MAX_NOMINAL_CM) {
            // Régimen de velocidad nominal
            cvPwmMotor = PWM_MOTOR_NOMINAL;
        } else {
            // Rampa Proporcional continua idéntica a Python: 80 a 200 PWM entre 11.5 cm y 20.0 cm
            float factor = (pvDistanciaCm - DIST_MIN_STOP_CM) / (DIST_MAX_NOMINAL_CM - DIST_MIN_STOP_CM);
            cvPwmMotor = PWM_MOTOR_MIN_RAMP + (int)(factor * (PWM_MOTOR_NOMINAL - PWM_MOTOR_MIN_RAMP));
            cvPwmMotor = constrain(cvPwmMotor, PWM_MOTOR_MIN_RAMP, PWM_MOTOR_NOMINAL);
        }
        actuarMotores(cvPwmMotor, true);
    } else if (estadoActual == ESTADO_DESVIO) {
        // En desvío/expulsión: pulso controlado a 180 PWM idéntico a Python
        cvPwmMotor = PWM_MOTOR_EXPULSION;
        actuarMotores(cvPwmMotor, true);
    } else {
        // En reposo o inspección estática, la faja permanece detenida
        cvPwmMotor = 0;
        actuarMotores(0, true);
    }

    // ---------------------------------------------------------------------------------
    // LAZO 3: Estabilización Proporcional de Iluminación de Inspección
    // ---------------------------------------------------------------------------------
    if (!simulacionActiva) {
        pvIluminacionPct = leerLDRPorcentaje();
    }

    // Lectura del Setpoint desde A1 si no ha sido fijado por comando
    int lecturaPot = analogRead(PIN_POT_CALIB);
    spIluminacionPct = map(lecturaPot, 0, 1023, 20, 95); // SP ajustable entre 20% y 95%

    if (estadoActual == ESTADO_INSPECCION || estadoActual == ESTADO_TRANSPORTE) {
        // Algoritmo Proporcional: Error = SP - PV
        float errorLuz = spIluminacionPct - pvIluminacionPct;
        
        // Salida = SalidaBase + Kp * Error
        float calculoPwm = PWM_LIGHT_BASE + (KP_LIGHT * errorLuz * 2.55f);
        cvPwmIluminacion = constrain((int)calculoPwm, 0, 255);
        actuarIluminacion(cvPwmIluminacion);
    } else {
        // En reposo o desvío, luz tenue de ahorro energético (PWM = 25)
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
            setSenalizadores(false, true, false, false); // Amarillo fijo = Espera
            actuarServo(SERVO_ANG_CONFORME);
            // Si el operario o sistema envía comando de inicio o se detecta pieza en cola (>30cm)
            break;

        case ESTADO_TRANSPORTE:
            setSenalizadores(true, false, false, false); // Verde fijo = Marcha
            // Re-armar la zona de inspección cuando la faja se despeje (> 15 cm)
            if (pvDistanciaCm > 15.0f) {
                zonaInspeccionArmada = true;
            }
            // Si una nueva pieza entra a la zona de inspección (distancia <= 11.5 cm)
            if (zonaInspeccionArmada && pvDistanciaCm <= DIST_MIN_STOP_CM) {
                estadoActual = ESTADO_INSPECCION;
                cvPwmMotor = PWM_MOTOR_MIN;
                actuarMotores(0, true);
                tInicioInspeccion = now;
                zonaInspeccionArmada = false;
                tareaTelemetriaSerial(); // Notificación inmediata de parada con latencia cero
                Serial.println(F("{\"EVENTO\":\"PIEZA_EN_INSPECCION\"}"));
            }
            break;

        case ESTADO_INSPECCION:
            setSenalizadores(false, true, false, false); // Amarillo parpadeante/fijo en inspección
            // Simulación o evaluación de inspección durante 0.8 segundos (idéntico a Python)
            if (now - tInicioInspeccion >= TIEMPO_INSPECCION_MS) {
                // Si no se recibió clasificación por serial, se asigna lógica basada en lux y distancia
                if (piezaClasificada == PIEZA_NINGUNA) {
                    if (pvIluminacionPct >= 65.0f) {
                        piezaClasificada = PIEZA_CONFORME;
                    } else if (pvIluminacionPct >= 40.0f) {
                        piezaClasificada = PIEZA_RETRABAJO;
                    } else {
                        piezaClasificada = PIEZA_RECHAZO;
                    }
                }
                
                // Transición a enrutamiento
                estadoActual = ESTADO_DESVIO;
                tInicioDesvio = now;
                cvPwmMotor = PWM_MOTOR_EXPULSION;
                actuarMotores(cvPwmMotor, true);

                // Aplicar Lazo 2 (Posicionamiento de Servomotor)
                if (piezaClasificada == PIEZA_RETRABAJO) {
                    cvAnguloServo = SERVO_ANG_RETRABAJO;
                } else if (piezaClasificada == PIEZA_RECHAZO) {
                    cvAnguloServo = SERVO_ANG_RECHAZO;
                } else {
                    cvAnguloServo = SERVO_ANG_CONFORME;
                }
                actuarServo(cvAnguloServo);

                tareaTelemetriaSerial(); // Notificación inmediata de desvío y nuevo ángulo

                Serial.print(F("{\"EVENTO\":\"CLASIFICACION_COMPLETA\",\"RESULTADO\":\""));
                Serial.print((piezaClasificada == PIEZA_CONFORME) ? F("CONFORME") : 
                             (piezaClasificada == PIEZA_RETRABAJO) ? F("RETRABAJO") : F("RECHAZO"));
                Serial.print(F("\",\"ANGULO\":"));
                Serial.print(cvAnguloServo);
                Serial.println(F("}"));
            }
            break;

        case ESTADO_DESVIO:
            setSenalizadores(true, true, false, false); // Verde + Amarillo durante expulsión
            // Retorno temporizado de la compuerta a neutro (90°) tras el paso (1.0s)
            if (now - tInicioDesvio >= TIEMPO_DESVIO_MS) {
                actuarServo(SERVO_ANG_CONFORME);
                piezaClasificada = PIEZA_NINGUNA;
                zonaInspeccionArmada = true; // Rearme garantizado para la siguiente pieza
                estadoActual = ESTADO_TRANSPORTE; // Vuelve a transporte
                cvPwmMotor = PWM_MOTOR_NOMINAL;
                actuarMotores(cvPwmMotor, true);
                tareaTelemetriaSerial(); // Notificación inmediata de retorno a marcha
                Serial.println(F("{\"EVENTO\":\"COMPUERTA_RESTABLECIDA_NEUTRO\"}"));
            }
            break;

        case ESTADO_EMERGENCIA:
            // Todo el control dinámico se ejecuta en la ISR y en tareaLazosControl
            // El rearme se procesa exclusivamente vía comando "RESET" tras desbloqueo físico
            break;
    }
}

// =====================================================================================
// INTERFAZ DE USUARIO HMI (PANTALLA LCD 16x2 I2C)
// =====================================================================================
void tareaActualizarHMI() {
    // Fila 0: [ESTADO] y Distancia
    // Ej: "TRANSP | D: 18cm"
    lcd.setCursor(0, 0);
    char linea0[17];
    const char* nombreEst = obtenerNombreEstado(estadoActual);
    int distInt = (int)pvDistanciaCm;
    snprintf(linea0, sizeof(linea0), "%-6s|D:%3dcm ", nombreEst, distInt);
    lcd.print(linea0);

    // Fila 1: PWM Motor, Lux %, Ángulo Servo
    // Ej: "M:140 L:70% S:90"
    lcd.setCursor(0, 1);
    char linea1[17];
    int luxInt = (int)pvIluminacionPct;
    snprintf(linea1, sizeof(linea1), "M:%3d L:%2d%% S:%3d", cvPwmMotor, luxInt, cvAnguloServo);
    lcd.print(linea1);
}

// =====================================================================================
// TELEMETRÍA SERIAL / BLUETOOTH HC-05
// Formato CSV estándar y JSON para monitoreo industrial
// =====================================================================================
void tareaTelemetriaSerial() {
    // Trama CSV compacta: STATUS,DIST_CM,PWM_MOTOR,LUX_PCT,SERVO_DEG,EMERGENCY
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
// PARSER DE COMANDOS SERIALES / BLUETOOTH (NO BLOQUEANTE)
// =====================================================================================
void ejecutarComando(char* cmd) {
    if (cmd[0] == '\0') return;

    if (strncmp(cmd, "SIM:", 4) == 0) {
        simulacionActiva = true;
        char* comma = strchr(cmd + 4, ',');
        if (comma) {
            *comma = '\0';
            pvDistanciaCm = atof(cmd + 4);
            pvIluminacionPct = atof(comma + 1);
        }
        return; // Silencio absoluto para mantener el canal UART 100% fluido
    }
    if (strncmp(cmd, "SIM_DIST:", 9) == 0) {
        simulacionActiva = true;
        pvDistanciaCm = atof(cmd + 9);
        return;
    }
    if (strncmp(cmd, "SIM_LDR:", 8) == 0) {
        simulacionActiva = true;
        pvIluminacionPct = atof(cmd + 8);
        return;
    }
    if (strcmp(cmd, "START") == 0) {
        if (estadoActual == ESTADO_REPOSO || (estadoActual != ESTADO_EMERGENCIA && estadoActual != ESTADO_TRANSPORTE)) {
            estadoActual = ESTADO_TRANSPORTE;
            zonaInspeccionArmada = true;
            actuarServo(SERVO_ANG_CONFORME);
            cvPwmMotor = PWM_MOTOR_NOMINAL;
            actuarMotores(cvPwmMotor, true);
            tareaTelemetriaSerial();
            Serial.println(F("{\"RESP\":\"CMD_OK\",\"ACCION\":\"INICIAR_TRANSPORTE\"}"));
        } else if (estadoActual == ESTADO_TRANSPORTE) {
            Serial.println(F("{\"RESP\":\"CMD_OK\",\"INFO\":\"YA_EN_TRANSPORTE\"}"));
        } else {
            Serial.println(F("{\"RESP\":\"CMD_ERROR\",\"MOTIVO\":\"ESTADO_EMERGENCIA_ACTIVO\"}"));
        }
        return;
    }
    if (strcmp(cmd, "STOP") == 0) {
        if (estadoActual != ESTADO_EMERGENCIA) {
            estadoActual = ESTADO_REPOSO;
            cvPwmMotor = 0;
            actuarMotores(0, false);
            tareaTelemetriaSerial();
            Serial.println(F("{\"RESP\":\"CMD_OK\",\"ACCION\":\"DETENER_EN_REPOSO\"}"));
        }
        return;
    }
    if (strcmp(cmd, "RESET") == 0) {
        if (digitalRead(PIN_EMERGENCY_STOP) == HIGH) {
            paradaEmergenciaActivada = false;
            flancoEmergenciaDetectado = false;
            estadoActual = ESTADO_REPOSO;
            zonaInspeccionArmada = true;
            cvPwmMotor = 0;
            setSenalizadores(false, true, false, false);
            actuarServo(SERVO_ANG_CONFORME);
            tareaTelemetriaSerial();
            Serial.println(F("{\"RESP\":\"REARME_EXITOSO\",\"NUEVO_ESTADO\":\"REPOSO\"}"));
        } else {
            Serial.println(F("{\"RESP\":\"REARME_FALLIDO\",\"MOTIVO\":\"PULSADOR_AUN_PRESIONADO\"}"));
        }
        return;
    }
    if (strncmp(cmd, "CLASSIFY:", 9) == 0) {
        char* tipo = cmd + 9;
        if (strcmp(tipo, "CONFORME") == 0) piezaClasificada = PIEZA_CONFORME;
        else if (strcmp(tipo, "RECHAZO") == 0) piezaClasificada = PIEZA_RECHAZO;
        else if (strcmp(tipo, "RETRABAJO") == 0) piezaClasificada = PIEZA_RETRABAJO;
        Serial.print(F("{\"RESP\":\"CMD_OK\",\"CLASIFICACION\":\""));
        Serial.print(tipo);
        Serial.println(F("\"}"));
        return;
    }
    if (strcmp(cmd, "SIM_OFF") == 0) {
        simulacionActiva = false;
        Serial.println(F("{\"RESP\":\"SIM_OFF\",\"MODO\":\"HARDWARE_REAL\"}"));
        return;
    }
}

void procesarComandosSeriales() {
    static char rxBuf[64];
    static uint8_t rxIdx = 0;

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            rxBuf[rxIdx] = '\0';
            if (rxIdx > 0) {
                ejecutarComando(rxBuf);
            }
            rxIdx = 0;
        } else {
            if (rxIdx < sizeof(rxBuf) - 1) {
                rxBuf[rxIdx++] = c;
            } else {
                rxIdx = 0; // Descarte de desborde
            }
        }
    }
}

// =====================================================================================
// CONTROLADORES DE ACTUADORES FÍSICOS (DRIVER L298N, PWM LUZ, SERVO, LEDS)
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
    // Divisor de tensión: 5V -> LDR -> Pin A0 -> R_10k -> GND
    // Mayor luz = Menor resistencia LDR = Mayor voltaje en A0 (lectura ADC alta)
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
