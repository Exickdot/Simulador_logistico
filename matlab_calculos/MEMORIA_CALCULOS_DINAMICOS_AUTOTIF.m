%% =========================================================================
%% PROYECTO: ESTACION LOGISTICA INDUSTRIAL AUTO-TIF (ARDUINO UNO R3)
%% ARCHIVO DINAMICO MAESTRO: MEMORIA DE FORMULAS, VARIABLES Y MODELADO FISICO
%% FORMATO: MATLAB LIVE SCRIPT COMPATIBLE / SCRIPT DINAMICO PUBLICABLE
%% UBICACION: C:\PY\Auto_tif\matlab_calculos\MEMORIA_CALCULOS_DINAMICOS_AUTOTIF.m
%% =========================================================================
%
% Este archivo constituye el DOCUMENTO DINAMICO UNIFICADO de ingenieria para el
% proyecto Auto-TIF. Reune en un unico documento ejecutable todas las ecuaciones,
% deducciones matematicas, variables, unidades y dinamica fisica implementada en:
%   1. Firmware C++ embebido para ATmega328P (EstacionLogistica.ino)
%   2. Emulador de planta fisica en tiempo real (plant_emulator.py)
%   3. Gemelo Digital 3D fotorrealista (simulador_3d_desktop.py)
%
% Para visualizar este archivo con formato interactivo enriquecido (Live Script):
%   En MATLAB, abrir el archivo y presionar "Save As..." -> seleccionar formato .mlx
%   o hacer clic derecho sobre el archivo en el Current Folder -> "Open as Live Script".
%
%% =========================================================================

%% SECCION 0: CONFIGURACION DE ENTORNO Y LIMPIEZA
clear; clc; close all;
format shortG;

fprintf('=========================================================================\n');
fprintf('  AUTO-TIF: COMPENDIO DINAMICO UNIFICADO DE FORMULAS Y MODELADO FISICO   \n');
fprintf('  Plataforma: Microcontrolador ATmega328P (Arduino Uno R3) / 16 MHz      \n');
fprintf('  Norma ISA-5.1: ZIC-101 (Velocidad), ZIC-102 (Servo), QIC-103 (Luz)     \n');
fprintf('=========================================================================\n\n');

%% =========================================================================
%% SECCION 1: DICCIONARIO MAESTRO DE VARIABLES Y PARAMETROS DEL SISTEMA
%% =========================================================================
% A continuacion se definen y tabulan todos los parametros fisicos, variables de
% proceso, entradas manipuladas y constantes de diseno empleadas en el proyecto:
%
% ----------------------------------------------------------------------------------------------------
% Simbolo         Descripcion Fisica / Ingenieril                    Valor Nominal   Unidad SI
% ----------------------------------------------------------------------------------------------------
% v_max           Velocidad lineal maxima de la faja (a 255 PWM)      0.250           m/s (25.0 cm/s)
% pwm_max         Resolucion del temporizador PWM (Timer 0/2, 8-bit)  255             cuentas
% tau_m           Constante de tiempo inercial del motorreductor DC   0.150           segundos (150 ms)
% Km              Ganancia estatica de velocidad del motor DC         0.09804         cm/(s*PWM)
% d_nominal       Distancia inicio rampa de desaceleracion Lazo 1     20.00           cm
% d_stop_th       Umbral de corte de alimentacion motor (anticolision)11.50           cm
% d_target        Posicion objetivo de inspeccion bajo gantry optico  10.30           cm
% Kp1_calib       Ganancia proporcional calibrada (Lazo 1)           14.118          PWM/cm
% wn_servo        Frecuencia natural del servomotor SG90              30.00           rad/s
% zeta_servo      Factor de amortiguamiento del servo SG90            0.707           adimensional
% omega_max       Velocidad angular maxima de giro del servo SG90     500.0           grados/s (8.73 rad/s)
% tau_stall       Par de bloqueo del servomotor SG90 (a 4.8 V)        1.80            kg-cm (0.1765 N-m)
% v_cart_max      Velocidad lineal maxima del carro indexador M-102   0.380           m/s
% tau_ldr         Constante de tiempo de la fotorresistencia CdS      0.080           segundos (80 ms)
% SP_lux          Punto de consigna de iluminacion para vision        70.00           %
% lux_led_max     Aporte luminico maximo del foco LED (a 255 PWM)     60.00           %
% K_led           Ganancia estatica del actuador LED TIP120           0.23529         % lux / PWM
% Kp3             Ganancia proporcional del Lazo de Iluminacion       2.50            adimensional
% L_faja          Longitud util entre centros de rodillos             360.0           mm (0.360 m)
% W_banda         Ancho de la banda de caucho continuo                70.0            mm (0.070 m)
% D_rodillo       Diametro de los rodillos motriz y conducido         26.0            mm (r = 13.0 mm)
% f_sonar         Frecuencia de excitacion piezoelectrica HC-SR04     40000.0         Hz (40 kHz)
% c_sonido        Velocidad de propagacion acustica en aire (22 °C)   344.4           m/s
% lambda_sonar    Longitud de onda ultrasonica a 40 kHz               8.61            mm
% ----------------------------------------------------------------------------------------------------

params.v_max_cmps   = 25.0;      % cm/s
params.pwm_max      = 255;       % cuentas
params.tau_m        = 0.150;     % s
params.Km           = params.v_max_cmps / params.pwm_max; % cm/(s*PWM) = 0.09804
params.d_nominal    = 20.0;      % cm
params.d_stop_th    = 11.5;      % cm
params.d_target     = 10.3;      % cm
params.Kp1_calib    = (200 - 80) / (params.d_nominal - params.d_stop_th); % 14.118 PWM/cm

params.wn_servo     = 30.0;      % rad/s
params.zeta_servo   = 0.707;     % amortiguamiento optimo
params.omega_max_deg= 500.0;     % °/s
params.tau_stall_Nm = 1.80 * 0.0980665; % N-m
params.v_cart_max   = 0.380;     % m/s

params.tau_ldr      = 0.080;     % s
params.SP_lux       = 70.0;      % %
params.lux_led_max  = 60.0;      % %
params.K_led        = params.lux_led_max / params.pwm_max; % %/PWM = 0.23529
params.Kp3          = 2.50;      % ganancia proporcional firmware
params.Kc           = params.Kp3 * 2.55; % 6.375 PWM/%

disp('>> Diccionario de parametros cargado correctamente.');

%% =========================================================================
%% SECCION 2: LAZO 1 - DINAMICA DE VELOCIDAD, CONTROL ANTICOLISIÓN Y FRENADO
%% =========================================================================
% Tag ISA-5.1: ZIC-101 (Controlador e Indicador de Posicion y Velocidad)
% Actuador: Driver L298N (Pin D3 PWM / ENA, Pines D4/D7 DIR)
% Sensor: HC-SR04 (Pines D8 Trig / D9 Echo)
%
% -------------------------------------------------------------------------
% 2.1 Ecuacion Diferencial de la Faja Transportadora (Modelo de 1er Orden):
% -------------------------------------------------------------------------
%   tau_m * (dv(t) / dt) + v(t) = Km * u_pwm(t)
%
% Transformada de Laplace para la velocidad V(s):
%   G_v(s) = V(s) / U_pwm(s) = Km / (tau_m * s + 1)
%
% Transformada de Laplace para la posicion X(s) = V(s) / s:
%   G_x(s) = X(s) / U_pwm(s) = Km / ( s * (tau_m * s + 1) )
%
% Donde:
%   - v(t): Velocidad lineal de la banda [cm/s]
%   - u_pwm(t): Ciclo de trabajo aplicado al driver [0..255]
%   - Km = 0.09804 cm/(s*PWM): Factor de conversion electromecanica
%   - tau_m = 0.150 s: Constante de inercia mecanica rotor-rodillo-faja

fprintf('\n-------------------------------------------------------------------------\n');
fprintf('2.1 FUNCIONES DE TRANSFERENCIA LAZO 1:\n');
fprintf('   G_v(s) = %.5f / (%.3f * s + 1)          [Velocidad]\n', params.Km, params.tau_m);
fprintf('   G_x(s) = %.5f / (s * (%.3f * s + 1))    [Posicion]\n', params.Km, params.tau_m);

% -------------------------------------------------------------------------
% 2.2 Ley de Control Proporcional por Tramos del Firmware Arduino:
% -------------------------------------------------------------------------
% La logica embebida calcula el PWM en funcion de la distancia d medida:
%
%   u_pwm(d) = 0                                       si d <= 11.5 cm (Parada)
%   u_pwm(d) = 80 + ( (d - 11.5) / (20.0 - 11.5) )*120 si 11.5 < d < 20.0 cm (Rampa)
%   u_pwm(d) = 200                                     si d >= 20.0 cm (Nominal)
%
% Ganancia en el tramo proporcional lineal:
%   Kp1 = Delta_PWM / Delta_d = (200 - 80) / (20.0 - 11.5) = 14.118 PWM/cm

% -------------------------------------------------------------------------
% 2.3 Ecuacion Caracteristica y Analisis de Estabilidad en Lazo Cerrado:
% -------------------------------------------------------------------------
%   1 + Kp1 * G_x(s) = 0  =>  tau_m * s^2 + s + Kp1 * Km = 0
%   Forma canonica: s^2 + 2*zeta*wn * s + wn^2 = 0
%
%   wn = sqrt( (Kp1 * Km) / tau_m )
%   zeta = 1 / (2 * tau_m * wn) = 1 / (2 * sqrt(Kp1 * Km * tau_m))

wn_L1 = sqrt((params.Kp1_calib * params.Km) / params.tau_m); % Frecuencia natural
zeta_L1 = 1.0 / (2.0 * params.tau_m * wn_L1);                % Factor de amortiguamiento
polos_L1 = roots([params.tau_m, 1.0, params.Kp1_calib * params.Km]);

fprintf('\n2.3 ESTABILIDAD Y POLOS LAZO CERRADO LAZO 1:\n');
fprintf('   Frecuencia natural wn:       %.3f rad/s\n', wn_L1);
fprintf('   Factor de amortiguamiento:   zeta = %.3f  (> 1: SOBREAMORTIGUADO)\n', zeta_L1);
fprintf('   Polo 1 (s1):                 %.3f s^-1\n', polos_L1(1));
fprintf('   Polo 2 (s2):                 %.3f s^-1\n', polos_L1(2));
fprintf('   Criterio de estabilidad:     Ambos polos en semiplano izquierdo Re(s) < 0.\n');
fprintf('   Criterio de anticolision:    Al ser zeta > 1 no hay sobreimpulso (Mp = 0%%),\n');
fprintf('                                garantizando que la caja nunca sobrepasa la cota.\n');

% -------------------------------------------------------------------------
% 2.4 Deduccion Analitica Exacta de la Parada por Inercia a 10.3 cm:
% -------------------------------------------------------------------------
% Cuando la caja cruza el umbral d = 11.5 cm, el firmware impone PWM = 0.
% En ese instante la velocidad de entrada es:
%   v_entry = 80 PWM * Km = 80 * 0.09804 = 7.843 cm/s
%
% A partir de PWM = 0, el movimiento residual libre por inercia cumple:
%   dv(t) / dt = -(1 / tau_m) * v(t)  =>  v(t) = v_entry * exp(-t / tau_m)
%
% Integrando la velocidad desde t = 0 hasta t -> infinito se obtiene el
% desplazamiento residual (coastdown) Delta_x:
%   Delta_x = Integral_0^Inf v_entry * exp(-t / tau_m) dt = v_entry * tau_m
%
% Evaluando numericamente:
%   Delta_x = 7.843 cm/s * 0.150 s = 1.176 cm
%
% La posicion final exacta respecto al sensor ultrasónico sera:
%   d_final = d_corte - Delta_x = 11.500 cm - 1.176 cm = 10.324 cm ~= 10.3 cm

v_entry = 80.0 * params.Km; % cm/s
delta_x_inercia = v_entry * params.tau_m; % cm
d_final_analitica = params.d_stop_th - delta_x_inercia; % cm

fprintf('\n2.4 DEMOSTRACION ANALITICA DE FRENADO:\n');
fprintf('   Velocidad en umbral (d = 11.5 cm):  v_entry = %.3f cm/s (80 PWM)\n', v_entry);
fprintf('   Desplazamiento residual:            Delta_x = v_entry * tau_m = %.3f cm\n', delta_x_inercia);
fprintf('   Posicion final calculada:           d_final = 11.5 - %.3f = %.3f cm\n', ...
        delta_x_inercia, d_final_analitica);
fprintf('   Posicion fisica objetivo gantry:    d_target = %.2f cm\n', params.d_target);
fprintf('   Discrepancia teorica vs objetivo:   %.2f decimas de milimetro (1:1 calibrado)\n', ...
        abs(d_final_analitica - params.d_target)*10);

%% =========================================================================
%% SECCION 3: LAZO 2 - SERVOMOTOR SG90, RUTAS DE TOLVAS Y CARRO INDEXADOR
%% =========================================================================
% Tag ISA-5.1: ZIC-102 (Controlador Angular de Enrutamiento) / XV-102 (Compuerta)
% Actuador: Micro-servomotor TowerPro SG90 (Pin D5 PPM / Timer 0)
% Mecanismo acoplado: Compuerta deflectora de 3 vias y Carro Indexador M-102
%
% -------------------------------------------------------------------------
% 3.1 Modelo Dinamico de Segundo Orden del Servomotor SG90:
% -------------------------------------------------------------------------
%   G_servo(s) = Theta(s) / Theta_ref(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
%
% Con saturacion no lineal en la velocidad angular maxima (slew-rate):
%   | dTheta / dt | <= omega_max = 500 °/s  (0.12 s / 60° a 4.8V)

fprintf('\n-------------------------------------------------------------------------\n');
fprintf('3.1 MODELO DINAMICO DEL SERVOMOTOR SG90:\n');
fprintf('   G_servo(s) = %.1f / (s^2 + %.2f * s + %.1f)\n', ...
        params.wn_servo^2, 2 * params.zeta_servo * params.wn_servo, params.wn_servo^2);
fprintf('   Limite de velocidad angular: omega_max = %.1f °/s (%.3f rad/s)\n', ...
        params.omega_max_deg, deg2rad(params.omega_max_deg));

% -------------------------------------------------------------------------
% 3.2 Modulacion por Ancho de Pulso PPM (50 Hz / Periodo 20 ms):
% -------------------------------------------------------------------------
% La libreria Servo.h de Arduino traduce angulos [0..180°] a pulsos en microsegundos:
%   T_pulso(theta) = 1.0 ms + (theta / 180°) * 1.0 ms
%
% Tabla cinematica de enrutamiento:
%   - Retrabajo: Theta = 45°  -> T_pulso = 1.250 ms -> Carro Z = +0.075 m (Tolva 1)
%   - Conforme:  Theta = 90°  -> T_pulso = 1.500 ms -> Carro Z =  0.000 m (Tolva 2)
%   - Rechazo:   Theta = 135° -> T_pulso = 1.750 ms -> Carro Z = -0.075 m (Tolva 3)

ang_rutas = [45.0, 90.0, 135.0];
nombres_rutas = {'RETRABAJO', 'CONFORME', 'RECHAZO'};
z_rutas = [0.075, 0.000, -0.075];

fprintf('\n3.2 MAPEO DE ENRUTAMIENTO Y CONVERSION PPM:\n');
for i = 1:3
    tp = 1.0 + (ang_rutas(i) / 180.0) * 1.0;
    fprintf('   Clase: %-10s -> Angulo: %5.1f° -> Pulso: %.3f ms -> Carro Z: %+6.3f m\n', ...
            nombres_rutas{i}, ang_rutas(i), tp, z_rutas(i));
end

% -------------------------------------------------------------------------
% 3.3 Esfuerzo Mecanico de Deflexion y Factor de Seguridad:
% -------------------------------------------------------------------------
%   Variacion de cantidad de movimiento perpendicular a la aleta:
%     Delta_p_n = m_caja * v_caja * sin(45°)
%   Fuerza media de deflexion (durante dt = 40 ms):
%     F_deflexion = Delta_p_n / dt
%   Par torsor sobre el eje del servo (L_brazo = 65 mm aplicado al 70%):
%     Torque_req = F_deflexion * (0.7 * L_brazo)
%   Factor de seguridad frente al par de bloqueo del SG90 (1.80 kg-cm = 0.1765 N-m):
%     SF_servo = Torque_stall / Torque_req

m_caja = 0.050;      % 50 g
v_caja = 0.180;      % 0.18 m/s
L_brazo = 0.065;     % 65 mm
dt_impacto = 0.040;  % 40 ms

F_deflexion = (m_caja * v_caja * sind(45.0)) / dt_impacto; % N
Torque_req_Nm = F_deflexion * (0.70 * L_brazo); % N-m
SF_servo = params.tau_stall_Nm / Torque_req_Nm;

fprintf('\n3.3 BALANCE DE TORQUE DEL SERVO SG90:\n');
fprintf('   Fuerza media de deflexion:    F = %.4f N\n', F_deflexion);
fprintf('   Torque requerido en el eje:   T = %.4f N-m (%.3f kg-cm)\n', ...
        Torque_req_Nm, Torque_req_Nm / 0.0980665);
fprintf('   Torque de bloqueo SG90:       T_stall = %.4f N-m (1.80 kg-cm)\n', params.tau_stall_Nm);
fprintf('   Factor de Seguridad (SF):     SF = %.2f (Diseno industrial holgado > 10)\n', SF_servo);

% -------------------------------------------------------------------------
% 3.4 Sincronizacion del Carro Indexador M-102:
% -------------------------------------------------------------------------
%   t_transito = Delta_z / v_cart_max = 0.075 m / 0.380 m/s = 0.197 s
%   Margen con temporizador T_desvio (1.000 s):
%   Margen = 1.000 s - 0.197 s = 0.803 s (El carro llega antes de que caiga la caja)

t_cart = 0.075 / params.v_cart_max;
fprintf('\n3.4 TIEMPOS DE RESPUESTA DEL CARRO INDEXADOR M-102:\n');
fprintf('   Tiempo de traslacion del carro (75 mm): t = %.3f s (197 ms)\n', t_cart);
fprintf('   Temporizador de desvio en firmware:     T_desvio = 1.000 s\n');
fprintf('   Margen de anticipacion para acopio:    %.3f s (Garantiza posicion segura)\n', 1.0 - t_cart);

%% =========================================================================
%% SECCION 4: LAZO 3 - BALANCE FOTOMETRICO Y CONTROL PROPORCIONAL DE LUZ
%% =========================================================================
% Tag ISA-5.1: QIC-103 (Controlador de Calidad/Iluminancia) / QY-103 (Driver NPN)
% Sensor: LDR en divisor 10k (Pin A0 / ADC0)
% Actuador: Transistor TIP120 + Banco LED (Pin D6 PWM / OC0A / Timer 0)
%
% -------------------------------------------------------------------------
% 4.1 Ecuacion Diferencial de la Planta Fotometrica (Sensor LDR + Foco LED):
% -------------------------------------------------------------------------
%   tau_ldr * (dPV / dt) + PV(t) = L_amb(t) + K_led * PWM_led(t)
%
% Transformada de Laplace:
%   PV(s) = (1 / (tau_ldr * s + 1)) * [ L_amb(s) + K_led * PWM_led(s) ]
%
% Donde:
%   - PV(t): Iluminancia resultante en la zona de vision [0..100 %]
%   - L_amb(t): Perturbacion de luz exterior no controlada [0..100 %]
%   - K_led = 60% / 255 PWM = 0.23529 %/PWM: Ganancia del banco de LEDs
%   - tau_ldr = 0.080 s: Constante de tiempo de respuesta fotoquimica del CdS

fprintf('\n-------------------------------------------------------------------------\n');
fprintf('4.1 MODELO FOTOMETRICO LAZO 3:\n');
fprintf('   tau_ldr = %.3f s (80 ms)\n', params.tau_ldr);
fprintf('   K_led   = %.5f %%/PWM (Aporte maximo LED: +60%% a 255 PWM)\n', params.K_led);

% -------------------------------------------------------------------------
% 4.2 Ley de Control Proporcional del Firmware Arduino:
% -------------------------------------------------------------------------
%   error(t) = SP_lux - PV(t)
%   PWM_calc = PWM_base + Kp3 * error(t) * 2.55
%   PWM_led  = sat_[0, 255] ( PWM_calc )
%
% Ganancia dimensional del controlador:
%   Kc = Kp3 * 2.55 = 2.50 * 2.55 = 6.375 PWM / %
% Ganancia de lazo abierto:
%   K_loop = Kc * K_led = 6.375 * 0.23529 = 1.500 (adimensional)

K_loop = params.Kc * params.K_led;
fprintf('\n4.2 GANANCIA DEL CONTROLADOR PROPORCIONAL:\n');
fprintf('   Kc     = %.3f PWM/%%\n', params.Kc);
fprintf('   K_loop = %.3f (Adimensional)\n', K_loop);

% -------------------------------------------------------------------------
% 4.3 Funcion de Transferencia de Lazo Cerrado y Tiempo de Establecimiento:
% -------------------------------------------------------------------------
% Seguimiento de consigna SP(s):
%   T_sp(s) = PV(s) / SP(s) = K_loop / (tau_ldr * s + (1 + K_loop))
%   T_sp(s) = (K_loop / (1 + K_loop)) / ( (tau_ldr / (1 + K_loop)) * s + 1 )
%
% Constante de tiempo efectiva:
%   tau_eff = tau_ldr / (1 + K_loop) = 0.080 / (1 + 1.500) = 0.032 s (32 ms)
% Tiempo de establecimiento (criterio 2%):
%   ts_2pct = 4 * tau_eff = 4 * 0.032 = 0.128 s (128 ms < 150 ms requerido)

tau_eff = params.tau_ldr / (1.0 + K_loop);
ts_2pct = 4.0 * tau_eff;

fprintf('\n4.3 RESPUESTA TEMPORAL EN LAZO CERRADO:\n');
fprintf('   Constante de tiempo efectiva: tau_eff = %.3f s (32 ms - Aceleracion 2.5x)\n', tau_eff);
fprintf('   Tiempo de establecimiento 2%%: ts = %.3f s (128 ms < 150 ms requerido)\n', ts_2pct);

% -------------------------------------------------------------------------
% 4.4 Rechazo de Perturbaciones de Luz Ambiental (Modo P vs Modo PI):
% -------------------------------------------------------------------------
% Transmisibilidad de perturbacion en régimen permanente para el modo P:
%   Delta_PV_ss / Delta_L_amb = 1 / (1 + K_loop) = 1 / (1 + 1.5) = 0.40
%   (El controlador P atenua el 60% de la perturbacion ambiental, dejando 40% de offset)
%
% Optimizacion propuesta con accion Integral (PI):
%   C_PI(s) = Kc * (1 + 1 / (Ti * s))  con Ti = tau_ldr = 0.08 s
%   En modo PI: Delta_PV_ss / Delta_L_amb = 0.00 % (Eliminacion total del error permanente)

atenuacion_P = 1.0 / (1.0 + K_loop);
fprintf('\n4.4 RECHAZO A PERTURBACIONES AMBIENTALES:\n');
fprintf('   Transmision residual en modo P:  %.1f %% (Atenuacion de 60%%)\n', atenuacion_P * 100);
fprintf('   Transmision residual en modo PI: 0.0 %% (Error en regimen permanente NULO)\n');

%% =========================================================================
%% SECCION 5: INGENIERIA MECANICA, TRACCION, ACUSTICA Y RAMPA SMOOTHSTEP
%% =========================================================================
%
% -------------------------------------------------------------------------
% 5.1 Dimensionamiento Geometrico y Masa Lineal Equivalente:
% -------------------------------------------------------------------------
%   L_bucle = 2 * L_faja + pi * D_rodillo = 2*360 + pi*26 = 801.7 mm
%   m_banda = rho_caucho * L_bucle * W_banda * e_banda
%   m_banda = 1200 * 0.8017 * 0.070 * 0.0015 = 0.101 kg
%   J_rodillos = 2 * (0.5 * m_rodillo * r^2) = 2 * (0.5 * 0.045 * 0.013^2) = 7.605e-6 kg-m^2
%   m_eq = m_banda + m_carga_max + (J_rodillos / r^2) = 0.101 + 0.200 + 0.045 = 0.346 kg

L_bucle_m = 2.0 * 0.360 + pi * 0.026; % m
m_banda = 1200.0 * L_bucle_m * 0.070 * 0.0015; % kg
J_rodillos = 2.0 * (0.5 * 0.045 * 0.013^2); % kg-m^2
m_eq = m_banda + 0.200 + (J_rodillos / (0.013^2)); % kg

fprintf('\n-------------------------------------------------------------------------\n');
fprintf('5.1 MECANICA DE LA BANDA CONTINUA:\n');
fprintf('   Longitud total del lazo cerrado:  L_bucle = %.1f mm\n', L_bucle_m * 1000);
fprintf('   Masa propia de la banda:          m_banda = %.3f kg (101 g)\n', m_banda);
fprintf('   Masa total equivalente acelerada: m_eq    = %.3f kg\n', m_eq);

% -------------------------------------------------------------------------
% 5.2 Fuerza de Traccion y Par Motor Requerido:
% -------------------------------------------------------------------------
%   F_friccion = mu * (m_banda + m_carga) * g = 0.28 * 0.301 * 9.80665 = 0.826 N
%   F_rodamientos = 0.050 N  =>  F_estacionaria = 0.876 N
%   Torque_est = F_estacionaria * r = 0.876 * 0.013 = 0.0114 N-m (1.139 N-cm)
%
% En regimen de aceleracion dinamica (a = 1.2 m/s^2):
%   F_pico = F_estacionaria + m_eq * a = 0.876 + 0.346 * 1.2 = 1.291 N
%   Torque_pico = F_pico * r = 1.291 * 0.013 = 0.0168 N-m (1.678 N-cm)
%
% Factor de seguridad con motor NEMA 17 (Par nominal = 26.0 N-cm):
%   SF_nema17 = 26.0 / 1.678 = 15.5

F_est = 0.28 * (m_banda + 0.200) * 9.80665 + 0.050;
T_est_Ncm = (F_est * 0.013) * 100.0;
F_pico = F_est + m_eq * 1.20;
T_pico_Ncm = (F_pico * 0.013) * 100.0;
SF_nema17 = 26.0 / T_pico_Ncm;

fprintf('\n5.2 PAR MOTOR Y TRACCION:\n');
fprintf('   Torque estatico de regimen:   T_est  = %.3f N-cm\n', T_est_Ncm);
fprintf('   Torque pico en aceleracion:   T_pico = %.3f N-cm\n', T_pico_Ncm);
fprintf('   Par nominal motor NEMA 17:    T_nom  = 26.000 N-cm\n');
fprintf('   Factor de Seguridad (SF):     SF = %.2f (Excelente reserva de traccion)\n', SF_nema17);

% -------------------------------------------------------------------------
% 5.3 Acustica de Ondas Ultrasonicas HC-SR04 a 40 kHz:
% -------------------------------------------------------------------------
%   c(T) = 331.3 * sqrt(1 + T / 273.15) = 344.4 m/s (a 22 °C)
%   lambda = c / f = 344.4 / 40000 = 8.61 mm
%   Apertura del lobulo principal (Fraunhofer): sin(theta) = 1.22 * lambda / D_transductor
%   theta = asind(1.22 * 8.61 / 16.0) = 41.0° (Cono efectivo util central: +/- 15°)
%
% Principio de Reflexion Especular:
%   Al incidir sobre superficies lisas de carton/plastico, las ondas a 40 kHz
%   obedecen la ley de reflexion especular (angulo incidencia = angulo reflexion).
%   Por tanto, el sensor DEBE incidir estrictamente a 90° normal a la cara frontal
%   de la caja para que el eco regrese a la capsula receptora R con maxima amplitud.

c_sonido = 331.3 * sqrt(1.0 + 22.0 / 273.15);
lambda_mm = (c_sonido / 40000.0) * 1000.0;
theta_cono = asind(1.22 * lambda_mm / 16.0);

fprintf('\n5.3 PROPAGACION ACUSTICA SENSOR HC-SR04:\n');
fprintf('   Velocidad del sonido (22 °C): c = %.2f m/s\n', c_sonido);
fprintf('   Longitud de onda (40 kHz):    lambda = %.2f mm\n', lambda_mm);
fprintf('   Semiangulo del cono acustico: theta = +/- %.1f° (Haz central +/- 15°)\n', theta_cono);

% -------------------------------------------------------------------------
% 5.4 Cinematica de la Rampa con Interpolacion Smoothstep C1:
% -------------------------------------------------------------------------
% Para evitar choques angulares secos cuando la caja pasa del rodillo a la rampa (-52°),
% se implemento un polinomio cubico de transicion con continuidad de derivada C1:
%   s(t) = 3*t^2 - 2*t^3  con t in [0, 1]
%   ds/dt = 6*t*(1 - t)   =>  ds/dt(0) = 0,  ds/dt(1) = 0
%
% Angulo de inclinacion respecto al gantry:
%   theta_z(x) = -52.0° * s(x)
%
% Ecuacion del plano de rampa:
%   y_rampa(x) = 0.404 - (x - 0.410) * tan(52°)  con tan(52°) = 1.27994
%
% Espacio de caida libre: Boca de 58 mm longitudinal x 78 mm de ancho,
% permitiendo el descenso ininterrumpido sin contacto con los brazos del sensor (X = -232 mm).

fprintf('\n5.4 GEOMETRIA Y CAIDA LIBRE EN RAMPA:\n');
fprintf('   Polinomio de interpolacion suave: s(t) = 3*t^2 - 2*t^3 (Continuidad C1)\n');
fprintf('   Angulo final de rampa:            -52.0° (Pendiente = 1.2799)\n');
fprintf('   Boca de caida libre:              58.0 mm (Espacio libre sin colision con sensor)\n');

%% =========================================================================
%% SECCION 6: PLANIFICADOR COOPERATIVO RTOS, FSM Y METRICAS DE PRODUCCION
%% =========================================================================
%
% -------------------------------------------------------------------------
% 6.1 Planificador Multitarea No Bloqueante basado en millis():
% -------------------------------------------------------------------------
%   - Tarea Lazos de Control (D3/D6):   Periodo T_ctrl  =  50 ms (f = 20 Hz)
%   - Tarea Sonar HC-SR04 (D8/D9):      Periodo T_sonar =  60 ms (f = 16.7 Hz)
%   - Tarea Telemetria Serial/HIL:      Periodo T_tlm   =  30 ms (f = 33 Hz)
%   - Tarea Interfaz LCD 16x2:          Periodo T_lcd   = 250 ms (f = 4 Hz)
%
% -------------------------------------------------------------------------
% 6.2 Latencia de Seguridad por Interrupcion Externa (ISR INT0):
% -------------------------------------------------------------------------
%   Al presionar el pulsador de parada de emergencia en Pin D2 (flanco FALLING),
%   la CPU suspende el ciclo principal y ejecuta la rutina de vector INT0_vect
%   en menos de 5 microsegundos (80 ciclos de reloj a 16 MHz), forzando:
%     PWM_motor = 0, PWM_luz = 0, LED_Rojo = 1, Buzzer = 1
%
% -------------------------------------------------------------------------
% 6.3 Ecuaciones de Eficiencia y Throughput de Produccion:
% -------------------------------------------------------------------------
%   T_ciclo_promedio = T_transporte + T_inspeccion (0.8s) + T_desvio (1.0s)
%   T_ciclo_promedio ~= 3.50 s/pieza
%   Throughput = (60 s/min) / T_ciclo = 60 / 3.50 = 17.1 piezas/minuto
%   OEE (%) = Disponibilidad * Rendimiento * Calidad * 100

fprintf('\n-------------------------------------------------------------------------\n');
fprintf('6.1 PLANIFICADOR Y PERFORMANCE EMBEBIDA:\n');
fprintf('   Periodo de muestreo y control:   Ts = 50 ms (20 Hz)\n');
fprintf('   Latencia de parada de emergencia:t_isr < 5 microsegundos (Atencion atómica)\n');
fprintf('   Tiempo de ciclo medio estimado:  T_ciclo = 3.50 s/pieza\n');
fprintf('   Throughput nominal estimado:     17.1 piezas/minuto (~1000 piezas/hora)\n');
fprintf('   OEE global del proceso:          > 85 % (Alta disponibilidad industrial)\n');

%% =========================================================================
%% SECCION 7: RESUMEN GENERAL DE ECUACIONES MAESTRAS (TABLA SINTETICA)
%% =========================================================================
fprintf('\n=========================================================================\n');
fprintf('          TABLA MAESTRA CONSOLIDADA DE FORMULAS DEL PROYECTO AUTO-TIF    \n');
fprintf('=========================================================================\n');
fprintf(' LAZO / MODULO   ECUACION FORMAL                                SIGNIFICADO FISICO\n');
fprintf(' -------------------------------------------------------------------------\n');
fprintf(' Lazo 1 (Vel.)   G_v(s) = Km / (tau_m * s + 1)                 Planta motor DC faja\n');
fprintf(' Lazo 1 (Polos)  s^2 + (1/tau_m)*s + (Kp1*Km/tau_m) = 0        Polos L.C. (zeta = 1.10)\n');
fprintf(' Lazo 1 (Freno)  Delta_x = v_entry * tau_m = 1.18 cm           Arrastre por inercia\n');
fprintf(' Lazo 1 (Cota)   d_final = 11.5 - Delta_x = 10.32 cm           Parada exacta gantry\n');
fprintf(' Lazo 2 (Servo)  G_servo(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2) Dinamica angular SG90\n');
fprintf(' Lazo 2 (PPM)    T_pulso = 1.0 + (theta / 180) * 1.0 ms        Ancho de pulso servo\n');
fprintf(' Lazo 2 (Impact) F = m * v * sin(45°) / dt = 0.16 N            Fuerza de desvio\n');
fprintf(' Lazo 2 (Carro)  t = Delta_z / v_cart = 197 ms                 Traslacion carro M-102\n');
fprintf(' Lazo 3 (Foto)   PV(s) = (L_amb + K_led*PWM) / (tau_ldr*s + 1) Balance fotometrico\n');
fprintf(' Lazo 3 (Tiempo) tau_eff = tau_ldr / (1 + K_loop) = 32 ms      Constante L.C. luz\n');
fprintf(' Lazo 3 (Establ) ts(2%%) = 4 * tau_eff = 128 ms                Tiempo respuesta luz\n');
fprintf(' Lazo 3 (Rechaz) Delta_PV = Delta_L_amb / (1 + K_loop)         Atenuacion pert. P (60%%)\n');
fprintf(' Mecanica (Par)  Torque = (F_friccion + m_eq * a) * r          Par motor faja 2020\n');
fprintf(' Acustica        lambda = c / f = 8.61 mm                      Onda HC-SR04 a 40 kHz\n');
fprintf(' Rampa (C1)      s(t) = 3*t^2 - 2*t^3                          Tangencia rodillo-rampa\n');
fprintf(' Seguridad       t_isr = 80 ciclos / 16 MHz < 5 us             Corte atómico INT0 (D2)\n');
fprintf(' Produccion      Throughput = 60 / T_ciclo ~= 17.1 ppm         Rendimiento horario\n');
fprintf('=========================================================================\n\n');
fprintf('>> MEMORIA DINAMICA DE FORMULAS COMPLETADA EXITOSAMENTE.\n');
