%% MODULO: LAZO 1 - CONTROL PROPORCIONAL DE VELOCIDAD Y FRENADO ANTICOLISIÓN

% Este script calcula y valida matemáticamente la dinámica de:
% 1. Motor DC + Driver L298N + Faja transportadora.
% 2. Sensor ultrasónico HC-SR04 y lazo de realimentación de distancia.
% 3. Funciones de transferencia en lazo abierto y lazo cerrado.
% 4. Respuesta temporal al escalón y análisis de estabilidad (Polos, Margen de Fase).
% 5. Dinámica de frenado por inercia: demostración analítica del punto de parada
%    en exactamente d = 10.3 cm bajo la cámara de inspección.
% =========================================================================

clear; clc; close all;

fprintf('=========================================================================\n');
fprintf('  AUTO-TIF: ANALISIS MATEMATICO DEL LAZO 1 (VELOCIDAD Y ANTICOLISIÓN)   \n');
fprintf('=========================================================================\n\n');

%% 1. PARAMETROS FISICOS DEL SISTEMA
v_max_mps   = 0.25;        % Velocidad máxima a 255 PWM (m/s)
v_max_cmps  = v_max_mps * 100.0; % Velocidad máxima (cm/s) = 25.0 cm/s
pwm_max     = 255;         % Rango máximo PWM (8 bits)
tau_m       = 0.150;       % Constante de tiempo mecánica del motor DC (s) [150 ms]
d_nominal   = 20.0;        % Distancia de inicio de desaceleración (cm)
d_stop_th   = 11.5;        % Umbral de corte de motor (cm) en firmware calibrado
d_target    = 10.3;        % Cota de parada objetivo bajo el gantry de inspección (cm)

% Ganancia estática del motor (Planta de velocidad)
% Km = Delta_v / Delta_PWM
Km_cmps_pwm = v_max_cmps / pwm_max;    % [cm / (s * PWM)] = 0.09804
Km_mps_pwm  = v_max_mps / pwm_max;     % [m / (s * PWM)]   = 9.8039e-4

fprintf('1. PARAMETROS DE LA PLANTA:\n');
fprintf('   - Velocidad maxima: %.2f m/s (%.2f cm/s)\n', v_max_mps, v_max_cmps);
fprintf('   - Constante de tiempo del motor (tau_m): %.3f s\n', tau_m);
fprintf('   - Ganancia estatica Km: %.5f cm/(s*PWM)\n', Km_cmps_pwm);
fprintf('   - Umbral de corte PWM (firmware): %.2f cm\n', d_stop_th);
fprintf('   - Posicion objetivo de inspeccion: %.2f cm\n\n', d_target);

%% 2. MODELADO DE LA FUNCION DE TRANSFERENCIA
% Dinámica del motor DC (Velocidad):
% Gv(s) = V(s) / PWM(s) = Km / (tau_m * s + 1)
% Dinámica de posición:
% Gx(s) = X(s) / PWM(s) = Gv(s) / s = Km / (s * (tau_m * s + 1))

num_v = Km_cmps_pwm;
den_v = [tau_m, 1];

num_x = Km_cmps_pwm;
den_x = [tau_m, 1, 0];

% Controlador Proporcional Calibrado en Firmware:
% Entre 20.0 cm y 11.5 cm el PWM pasa de 200 a 80:
% Kp1_calib = (PWM_nom - PWM_min_ramp) / (d_nominal - d_stop_th)
Kp1_calib = (200 - 80) / (d_nominal - d_stop_th); % 120 / 8.5 = 14.118 PWM/cm
Kp1_nominal = (200 - 0) / (25.0 - 5.0);            % 10.0 PWM/cm (Modelo nominal doc)

fprintf('2. GANANCIA PROPORCIONAL DEL CONTROLADOR:\n');
fprintf('   - Kp (Nominal ISA doc):    %.2f PWM/cm\n', Kp1_nominal);
fprintf('   - Kp (Calibrado Firmware): %.3f PWM/cm (Rampa 200 -> 80 PWM)\n\n', Kp1_calib);

%% 3. ANALISIS DE POLOS Y ESTABILIDAD EN LAZO CERRADO
% Polinomio característico de lazo cerrado:
% 1 + Kp * Gx(s) = 0  =>  tau_m * s^2 + s + Kp * Km = 0
% s^2 + (1 / tau_m)*s + (Kp * Km / tau_m) = 0
% Forma estándar: s^2 + 2*zeta*wn*s + wn^2 = 0

wn_nom = sqrt(Kp1_nominal * Km_cmps_pwm / tau_m);
zeta_nom = 1 / (2 * tau_m * wn_nom);

wn_cal = sqrt(Kp1_calib * Km_cmps_pwm / tau_m);
zeta_cal = 1 / (2 * tau_m * wn_cal);

polos_nom = roots([tau_m, 1, Kp1_nominal * Km_cmps_pwm]);
polos_cal = roots([tau_m, 1, Kp1_calib * Km_cmps_pwm]);

fprintf('3. DINAMICA EN LAZO CERRADO:\n');
fprintf('   - Modo Nominal:   wn = %.3f rad/s, zeta = %.3f (Sobreamortiguado)\n', wn_nom, zeta_nom);
fprintf('     Polos: s1 = %.3f, s2 = %.3f (Estabilidad Asintotica Estricta)\n', polos_nom(1), polos_nom(2));
fprintf('   - Modo Calibrado: wn = %.3f rad/s, zeta = %.3f (Amortiguamiento optimo)\n', wn_cal, zeta_cal);
fprintf('     Polos: s1 = %.3f, s2 = %.3f\n\n', polos_cal(1), polos_cal(2));

%% 4. DEMOSTRACION ANALITICA DE FRENADO Y PARADA EXACTA A 10.3 CM
% Al llegar la caja a d = 11.5 cm, el microcontrolador corta la alimentación (PWM = 0).
% A partir de ese instante (t = 0), la ecuación diferencial de deslizamiento libre por inercia es:
% m * dv/dt = -b * v  =>  dv/dt = - (1 / tau_m) * v
% v(t) = v_entry * exp(-t / tau_m)
% La distancia de desplazamiento residual (coastdown) es:
% Delta_x = Integral de 0 a Inf de v(t) dt = v_entry * tau_m

v_entry_pwm = 80; % PWM de entrada al umbral de corte
v_entry_cmps = v_entry_pwm * Km_cmps_pwm; % = 80 * 0.09804 = 7.843 cm/s
delta_x_frenado = v_entry_cmps * tau_m;   % = 7.843 * 0.150 = 1.176 cm
d_final_teorica = d_stop_th - delta_x_frenado; % 11.5 - 1.176 = 10.324 cm

fprintf('4. DEMOSTRACION ANALITICA DEL PUNTO DE PARADA:\n');
fprintf('   - Velocidad al alcanzar d = %.2f cm: %.3f cm/s (%.1f PWM)\n', d_stop_th, v_entry_cmps, v_entry_pwm);
fprintf('   - Distancia de arrastre por inercia: Delta_x = v_entry * tau_m\n');
fprintf('     Delta_x = %.3f cm/s * %.3f s = %.3f cm\n', v_entry_cmps, tau_m, delta_x_frenado);
fprintf('   - Posicion final resultante: d_final = %.2f cm - %.3f cm = %.3f cm\n', ...
        d_stop_th, delta_x_frenado, d_final_teorica);
fprintf('   - Error respecto al objetivo gantry (10.30 cm): %.3f mm (Exactitud milimetrica 1:1)\n\n', ...
        abs(d_final_teorica - d_target)*10);

%% 5. SIMULACION NUMERICA CONTINUA DE APROXIMACION Y FRENADO (ODE)
% Simulación temporal de una caja aproximándose desde d = 50 cm
dt = 0.001;               % Paso de integración temporal (1 ms)
t_total = 3.5;            % Tiempo total (s)
t_vec = 0:dt:t_total;
N = length(t_vec);

d_sim = zeros(1, N);
v_sim = zeros(1, N);
pwm_sim = zeros(1, N);

% Condiciones iniciales
d_sim(1) = 45.0; % Caja ingresa a 45 cm del sensor
v_sim(1) = 0.0;
pwm_sim(1) = 0;

for k = 1:(N - 1)
    curr_d = d_sim(k);
    curr_v = v_sim(k);
    
    % Ley de control del microcontrolador ATmega328P
    if curr_d <= d_stop_th
        u_pwm = 0; % Corte anticolisión inmediato
    elseif curr_d >= d_nominal
        u_pwm = 200; % Velocidad de transporte nominal
    else
        % Rampa proporcional continua
        factor = (curr_d - d_stop_th) / (d_nominal - d_stop_th);
        u_pwm = 80 + factor * (200 - 80);
    end
    pwm_sim(k) = u_pwm;
    
    % Dinámica de 1er orden del motor: dv/dt = (Km*PWM - v) / tau_m
    v_target_k = u_pwm * Km_cmps_pwm;
    dv_dt = (v_target_k - curr_v) / tau_m;
    
    v_sim(k+1) = curr_v + dv_dt * dt;
    if v_sim(k+1) < 0
        v_sim(k+1) = 0; % Freno unidireccional / no retrocede
    end
    
    % Dinámica de distancia: la caja se aproxima al sensor (dd/dt = -v)
    d_sim(k+1) = curr_d - v_sim(k) * dt;
end
pwm_sim(N) = pwm_sim(N-1);

%% 6. GRAFICACION TECNICA DE RESULTADOS
figure('Name', 'Lazo 1 - Dinamica de Velocidad y Frenado Anticolision', ...
       'Color', [0.96 0.96 0.96], 'Position', [100, 100, 1050, 750]);

% Subplot 1: Trayectoria de Distancia
subplot(3, 1, 1);
plot(t_vec, d_sim, 'LineWidth', 2.0, 'Color', [0.10, 0.40, 0.75]); hold on;
yline(d_nominal, '--', 'Inicio Rampa (20.0 cm)', 'Color', [0.85, 0.50, 0.10], 'LineWidth', 1.3);
yline(d_stop_th, '--', 'Corte PWM (11.5 cm)', 'Color', [0.75, 0.20, 0.15], 'LineWidth', 1.3);
yline(d_target, '-.', sprintf('Parada Final (%.2f cm)', d_final_teorica), 'Color', [0.15, 0.65, 0.25], 'LineWidth', 1.5);
grid on; set(gca, 'Color', [1 1 1], 'GridColor', [0.85 0.85 0.85]);
ylabel('Distancia d(t) [cm]', 'FontWeight', 'bold');
title('Lazo 1: Trayectoria Temporal de Aproximacion y Parada Anticolision', 'FontSize', 11, 'FontWeight', 'bold');
legend('Distancia medida', 'Location', 'northeast');
xlim([0, t_total]);

% Subplot 2: Velocidad de la Faja
subplot(3, 1, 2);
plot(t_vec, v_sim, 'LineWidth', 2.0, 'Color', [0.20, 0.65, 0.35]); hold on;
yline(200 * Km_cmps_pwm, ':', 'Vel. Nominal (19.6 cm/s)', 'Color', [0.4 0.4 0.4]);
grid on; set(gca, 'Color', [1 1 1], 'GridColor', [0.85 0.85 0.85]);
ylabel('Velocidad v(t) [cm/s]', 'FontWeight', 'bold');
title('Perfil de Velocidad de la Faja Transportadora', 'FontSize', 11, 'FontWeight', 'bold');
xlim([0, t_total]);

% Subplot 3: Señal de Control PWM
subplot(3, 1, 3);
plot(t_vec, pwm_sim, 'LineWidth', 1.8, 'Color', [0.85, 0.25, 0.20]); hold on;
grid on; set(gca, 'Color', [1 1 1], 'GridColor', [0.85 0.85 0.85]);
xlabel('Tiempo t [s]', 'FontWeight', 'bold');
ylabel('Control ENA PWM [0-255]', 'FontWeight', 'bold');
title('Salida Manipulada PWM del Microcontrolador (Pin D3 / ENA)', 'FontSize', 11, 'FontWeight', 'bold');
ylim([-10, 260]);
xlim([0, t_total]);

fprintf('Simulacion completada exitosPamente. Grafico generado.\n');





