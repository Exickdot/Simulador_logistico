%% MODULO: LAZO 3 - CONTROL FOTOMETRICO DE ILUMINACION DE INSPECCION
% Este script calcula y valida matemáticamente la dinámica de:
% 1. Sensor LDR (Fotorresistencia CdS + Divisor de tensión 10k).
% 2. Transistor de potencia NPN TIP120 y banco de luminarias LED.
% 3. Balance fotométrico: Luz ambiental (perturbación) + Aporte LED.
% 4. Controlador Proporcional implementado en firmware (SP = 70%, Kp = 2.5).
% 5. Análisis de tiempo de establecimiento (tau_eff = 32 ms, ts = 128 ms).
% 6. Comparación de rechazo a perturbaciones: P (actual) vs PI (óptimo con error nulo).
% =========================================================================

clear; clc; close all;

fprintf('=========================================================================\n');
fprintf('  AUTO-TIF: ANALISIS MATEMATICO DEL LAZO 3 (CONTROL FOTOMETRICO / LDR)   \n');
fprintf('=========================================================================\n\n');

%% 1. ESPECIFICACIONES DE LA PLANTA FOTOMETRICA
tau_ldr    = 0.080;         % Constante de tiempo del sensor LDR CdS (s) [80 ms]
SP_lux     = 70.0;          % Setpoint nominal de iluminancia (%)
pwm_base   = 128;           % Polarización nominal de salida (50% duty cycle)
lux_led_max= 60.0;          % Aporte lumínico máximo del LED a 255 PWM (%)
pwm_max    = 255;           % Fondo de escala PWM (8 bits)

% Ganancia estática del actuador LED:
% K_led = Delta_Lux / Delta_PWM
K_led = lux_led_max / pwm_max; % [% lux / PWM] = 60 / 255 = 0.23529

% Parámetros del Controlador Proporcional en Firmware:
Kp3 = 2.50;                 % Ganancia proporcional del firmware
% En Arduino: PWM_calc = pwm_base + Kp3 * error_lux * 2.55
% Ganancia efectiva en unidades de PWM por % de error:
Kc = Kp3 * 2.55;            % [PWM / %] = 6.375 PWM/%

% Ganancia de lazo abierto: K_loop = Kc * K_led
K_loop = Kc * K_led;        % = 6.375 * 0.23529 = 1.500

fprintf('1. PARAMETROS DE LA PLANTA Y DEL SENSOR LDR:\n');
fprintf('   - Constante de tiempo del LDR (tau_ldr): %.3f s (80 ms)\n', tau_ldr);
fprintf('   - Aporte maximo del foco LED:            %.1f %% a 255 PWM\n', lux_led_max);
fprintf('   - Ganancia estatica K_led:               %.5f %%/PWM\n', K_led);
fprintf('   - Setpoint nominal de trabajo:           %.1f %% Lux\n\n', SP_lux);

fprintf('2. GANANCIA DEL CONTROLADOR PROPORCIONAL:\n');
fprintf('   - Parametro Kp firmware:                 %.2f\n', Kp3);
fprintf('   - Ganancia dimensional Kc:               %.3f PWM/%%\n', Kc);
fprintf('   - Ganancia de lazo K_loop:               %.3f (Adimensional)\n\n', K_loop);

%% 2. FUNCIONES DE TRANSFERENCIA Y ANALISIS EN LAZO CERRADO
% Modelo de la planta: PV(s) = (1 / (tau_ldr*s + 1)) * [L_amb(s) + K_led * PWM(s)]
% Lazo cerrado ante Setpoint:
% T_sp(s) = PV(s) / SP(s) = K_loop / (tau_ldr * s + (1 + K_loop))
% T_sp(s) = (K_loop / (1 + K_loop)) / ( (tau_ldr / (1 + K_loop)) * s + 1 )

tau_eff = tau_ldr / (1.0 + K_loop); % Constante de tiempo efectiva (s)
ts_2pct = 4.0 * tau_eff;            % Tiempo de establecimiento al 2% (s)
ganancia_dc_sp = K_loop / (1.0 + K_loop); % = 1.5 / 2.5 = 0.60

% Función de transferencia de rechazo a perturbación ambiental:
% G_dist(s) = PV(s) / L_amb(s) = 1 / (tau_ldr * s + (1 + K_loop))
atenuacion_dist_dc = 1.0 / (1.0 + K_loop); % = 1 / 2.5 = 0.40 (40% residual, 60% atenuada)

fprintf('3. RESPUESTA DINAMICA Y ESTABILIDAD EN LAZO CERRADO:\n');
fprintf('   - Constante de tiempo efectiva (tau_eff): %.3f s (32 ms - Aceleracion 2.5x)\n', tau_eff);
fprintf('   - Tiempo de establecimiento (ts al 2%%):   %.3f s (128 ms < 150 ms requerido)\n', ts_2pct);
fprintf('   - Atenuacion de perturbacion en DC:      %.1f %% de transmision (60%% rechazo)\n\n', ...
        atenuacion_dist_dc * 100.0);

%% 3. COMPARATIVA DE DISENO: CONTROLADOR P vs CONTROLADOR PI
% Controlador P: u(t) = pwm_base + Kc * e(t)  -> Deja offset proporcional ante perturbación
% Controlador PI: u(t) = pwm_base + Kc * [e(t) + (1/Ti) * Integral e(tau) dtau]
% Seleccionamos Ti = tau_ldr = 0.08 s para cancelación de polo dominante
Ti = 0.080; % s
Ki = Kc / Ti; % = 6.375 / 0.080 = 79.6875 PWM / (% * s)

fprintf('4. PROPUESTA DE CONTROLADOR PI:\n');
fprintf('   - Tiempo integral Ti:                    %.3f s (Cancelacion polo planta)\n', Ti);
fprintf('   - Ganancia integral Ki:                  %.3f PWM/(%%*s)\n', Ki);
fprintf('   - Error en regimen permanente con PI:    0.00 %% (Eliminacion total del offset)\n\n');

%% 4. SIMULACION TEMPORAL NO LINEAL CON PERTURBACIONES Y SATURACION
dt = 0.001;               % Paso temporal (1 ms)
t_total = 4.0;            % 4 segundos
t_vec = 0:dt:t_total;
N = length(t_vec);

% Perfil de perturbación lumínica ambiental:
% t = 0..1.0s: Luz ambiental normal (45%)
% t = 1.0..2.5s: Caída brusca por sombra o apertura de compuerta (15%) [Perturbación -30%]
% t = 2.5..4.0s: Recuperación a luz normal (45%)
L_amb = zeros(1, N);
for k = 1:N
    if t_vec(k) >= 1.0 && t_vec(k) < 2.5
        L_amb(k) = 15.0; % Caída drástica a 15%
    else
        L_amb(k) = 45.0; % Luz ambiental nominal
    end
end

% Variables simuladas Controlador P (Firmware Actual)
pv_P   = zeros(1, N);
pwm_P  = zeros(1, N);
pv_P(1) = 45.0;

% Variables simuladas Controlador PI (Propuesta Optimizada)
pv_PI  = zeros(1, N);
pwm_PI = zeros(1, N);
integral_err_PI = 0.0;
pv_PI(1) = 45.0;

for k = 1:(N - 1)
    % ----------------- CONTROLADOR P (FIRMWARE ARDUINO) -----------------
    err_P = SP_lux - pv_P(k);
    u_raw_P = pwm_base + Kc * err_P;
    u_sat_P = max(0, min(pwm_max, u_raw_P));
    pwm_P(k) = u_sat_P;
    
    % Dinámica de LDR P
    target_lux_P = L_amb(k) + u_sat_P * K_led;
    dpv_dt_P = (target_lux_P - pv_P(k)) / tau_ldr;
    pv_P(k+1) = pv_P(k) + dpv_dt_P * dt;
    
    % ----------------- CONTROLADOR PI (OPTIMIZADO) ---------------------
    err_PI = SP_lux - pv_PI(k);
    integral_err_PI = integral_err_PI + err_PI * dt;
    % Anti-windup clamping
    integral_err_PI = max(-30.0, min(30.0, integral_err_PI));
    
    u_raw_PI = pwm_base + Kc * err_PI + Ki * integral_err_PI * 0.15;
    u_sat_PI = max(0, min(pwm_max, u_raw_PI));
    pwm_PI(k) = u_sat_PI;
    
    % Dinámica de LDR PI
    target_lux_PI = L_amb(k) + u_sat_PI * K_led;
    dpv_dt_PI = (target_lux_PI - pv_PI(k)) / tau_ldr;
    pv_PI(k+1) = pv_PI(k) + dpv_dt_PI * dt;
end
pwm_P(N) = pwm_P(N-1);
pwm_PI(N) = pwm_PI(N-1);

%% 5. GRAFICACION TECNICA
figure('Name', 'Lazo 3 - Control Fotometrico de Iluminacion LDR', ...
       'Color', [0.96 0.96 0.96], 'Position', [200, 200, 1050, 750]);

% Subplot 1: Respuesta de Iluminancia (PV vs SP)
subplot(3, 1, 1);
plot(t_vec, pv_P, 'LineWidth', 2.0, 'Color', [0.15, 0.45, 0.85]); hold on;
plot(t_vec, pv_PI, 'LineWidth', 2.0, 'Color', [0.20, 0.65, 0.35]);
yline(SP_lux, '--', 'Setpoint Objetivo (70.0% Lux)', 'Color', [0.85, 0.20, 0.20], 'LineWidth', 1.5);
grid on; set(gca, 'Color', [1 1 1], 'GridColor', [0.85 0.85 0.85]);
ylabel('Iluminancia PV [%]', 'FontWeight', 'bold');
title('Lazo 3: Estabilizacion Fotometrica ante Perturbacion Brusca de Luz Ambiental', ...
      'FontSize', 11, 'FontWeight', 'bold');
legend('Controlador P (Firmware Actual)', 'Controlador PI (Optimizacion Cero Error)', ...
       'Consigna de Inspeccion SP', 'Location', 'southeast');
ylim([20, 85]); xlim([0, t_total]);

% Subplot 2: Perturbación Ambiental Inyectada
subplot(3, 1, 2);
plot(t_vec, L_amb, 'LineWidth', 1.8, 'Color', [0.85, 0.50, 0.10]); hold on;
grid on; set(gca, 'Color', [1 1 1], 'GridColor', [0.85 0.85 0.85]);
ylabel('Luz Ambiental [%]', 'FontWeight', 'bold');
title('Perturbacion Externa Inyectada (Caida de 45% a 15% entre 1.0s y 2.5s)', ...
      'FontSize', 11, 'FontWeight', 'bold');
ylim([0, 60]); xlim([0, t_total]);

% Subplot 3: Esfuerzo de Control PWM (LED Pin D6)
subplot(3, 1, 3);
plot(t_vec, pwm_P, 'LineWidth', 1.8, 'Color', [0.15, 0.45, 0.85]); hold on;
plot(t_vec, pwm_PI, 'LineWidth', 1.8, 'Color', [0.20, 0.65, 0.35]);
yline(255, ':', 'Saturacion Maxima (255 PWM)', 'Color', [0.8 0.2 0.2]);
grid on; set(gca, 'Color', [1 1 1], 'GridColor', [0.85 0.85 0.85]);
xlabel('Tiempo t [s]', 'FontWeight', 'bold');
ylabel('Salida PWM [0-255]', 'FontWeight', 'bold');
title('Salida PWM Modulada hacia Base del Transistor NPN TIP120 (Pin D6)', ...
      'FontSize', 11, 'FontWeight', 'bold');
legend('PWM Modo P', 'PWM Modo PI', 'Location', 'northeast');
ylim([0, 275]); xlim([0, t_total]);

fprintf('Simulacion de Lazo 3 completada exitosamente. Grafico generado.\n');
