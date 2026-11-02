%% MODULO: LAZO 2 - CINEMATICA Y DINAMICA DEL SERVOMOTOR SG90 Y ENRUTAMIENTO
% Este script calcula y valida matemáticamente la dinámica de:
% 1. Servomotor SG90 (Modelo dinámico de 2do orden + Saturación de velocidad angular).
% 2. Mapeo cinemático de enrutamiento a tolvas:
%    - Retrabajo: 45° (Pulso 1.25 ms) -> Chute Izquierdo (Z = +0.075 m)
%    - Conforme:  90° (Pulso 1.50 ms) -> Chute Central   (Z =  0.000 m)
%    - Rechazo:  135° (Pulso 1.75 ms) -> Chute Derecho   (Z = -0.075 m)
% 3. Esfuerzo de deflexión mecánica y par torsor requerido vs Par SG90 (1.8 kg-cm).
% 4. Cinemática del Carro Indexador Móvil M-102 (v_max = 0.38 m/s).
% 5. Sincronización de temporizadores: Inspección (0.8s) y Desvío (1.0s).
% =========================================================================

clear; clc; close all;

fprintf('=========================================================================\n');
fprintf('  AUTO-TIF: ANALISIS MATEMATICO DEL LAZO 2 (SERVOMOTOR Y ENRUTAMIENTO)   \n');
fprintf('=========================================================================\n\n');

%% 1. ESPECIFICACIONES TECNICAS DEL SERVOMOTOR TOWERPRO SG90
V_operacion      = 4.8;     % Tensión de alimentación (V)
torque_stall_kgcm= 1.80;    % Par de bloqueo a 4.8V (kg-cm)
torque_stall_Nm  = torque_stall_kgcm * 0.0980665; % [N-m] = 0.1765 N-m
t_60deg          = 0.120;   % Tiempo para girar 60 grados a 4.8V (s)
omega_max_deg_s  = 60.0 / t_60deg; % Velocidad angular máxima = 500 °/s
omega_max_rad_s  = deg2rad(omega_max_deg_s); % = 8.7266 rad/s

% Parámetros del modelo dinámico de 2do orden en lazo cerrado:
% G_servo(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
wn_servo   = 30.0;          % Frecuencia natural no amortiguada (rad/s)
zeta_servo = 0.707;         % Factor de amortiguamiento óptimo Butterworth

% Temporizadores del sistema embebido (firmware Arduino)
T_inspeccion_s = 0.80;      % Tiempo en estado ESTADO_INSPECCION (s)
T_desvio_s     = 1.00;      % Tiempo en estado ESTADO_DESVIO (s)
ang_neutral    = 90.0;      % Posición de reposo/conforme (°)

fprintf('1. PARAMETROS DEL ACTUADOR SG90:\n');
fprintf('   - Par de bloqueo nominal:    %.2f kg-cm (%.4f N-m)\n', torque_stall_kgcm, torque_stall_Nm);
fprintf('   - Velocidad angular maxima:  %.1f °/s (%.3f rad/s)\n', omega_max_deg_s, omega_max_rad_s);
fprintf('   - Modelo de segundo orden:   wn = %.1f rad/s, zeta = %.3f\n', wn_servo, zeta_servo);
fprintf('   - Temporizador de Inspeccion:%.2f s\n', T_inspeccion_s);
fprintf('   - Temporizador de Desvio:    %.2f s (Retorno automatico a 90°)\n\n', T_desvio_s);

%% 2. MAPEO CINEMATICO Y MODULACION POR ANCHO DE PULSO (PPM/PWM)
% El microcontrolador genera pulsos entre 1.0 ms y 2.0 ms en periodo de 20 ms (50 Hz):
% T_pulse(theta) = 1.0 ms + (theta / 180°) * 1.0 ms
rutas = {'RETRABAJO', 'CONFORME', 'RECHAZO'};
angulos = [45.0, 90.0, 135.0];
cart_z_targets = [0.075, 0.000, -0.075]; % Posición del carro móvil (m)

fprintf('2. TABLA DE ENRUTAMIENTO Y CONVERSION PPM:\n');
fprintf('   --------------------------------------------------------------------\n');
fprintf('   Clasificacion   Angulo (°)   Ancho Pulso (ms)   Destino Chute   Carro Z (m)\n');
fprintf('   --------------------------------------------------------------------\n');
for i = 1:length(rutas)
    ang = angulos(i);
    t_pulse = 1.0 + (ang / 180.0) * 1.0;
    fprintf('   %-13s   %7.1f°   %12.3f ms   Tolva %d         %+7.3f m\n', ...
            rutas{i}, ang, t_pulse, i, cart_z_targets(i));
end
fprintf('   --------------------------------------------------------------------\n\n');

%% 3. CALCULO DE ESFUERZOS MECANICOS Y PAR REQUERIDO
% Masa de la caja = 0.050 kg (50 g)
% Velocidad lineal de impacto en desvío = 0.18 m/s
% Longitud de la aleta deflectora L_paleta = 0.065 m (65 mm)
% Ángulo de deflexión respecto a trayectoria recta: Delta_theta = 45°
m_caja      = 0.050; % kg
v_caja      = 0.180; % m/s
L_paleta    = 0.065; % m
Delta_theta = deg2rad(45.0);
dt_impacto  = 0.040; % Tiempo estimado de deflexión progresiva (40 ms)

% Variación de cantidad de movimiento perpendicular a la aleta:
% Delta_p_normal = m_caja * v_caja * sin(Delta_theta)
Delta_p = m_caja * v_caja * sin(Delta_theta);
F_deflexion = Delta_p / dt_impacto; % Fuerza media normal de desvío (N)
Torque_deflexion_Nm = F_deflexion * (L_paleta * 0.70); % Aplicada al 70% del brazo
Torque_deflexion_kgcm = Torque_deflexion_Nm / 0.0980665;

% Coeficiente de seguridad del actuador SG90:
SF_servo = torque_stall_Nm / Torque_deflexion_Nm;

fprintf('3. BALANCE DE ESFUERZOS MECANICOS DE DEFLEXION:\n');
fprintf('   - Fuerza media de deflexion (F): %.4f N\n', F_deflexion);
fprintf('   - Par torsor requerido en el eje: %.4f N-m (%.3f kg-cm)\n', ...
        Torque_deflexion_Nm, Torque_deflexion_kgcm);
fprintf('   - Par disponible del servo SG90:  %.4f N-m (%.2f kg-cm)\n', ...
        torque_stall_Nm, torque_stall_kgcm);
fprintf('   - Factor de Seguridad (SF):       %.2f (Diseno holgado y robusto > 5)\n\n', SF_servo);

%% 4. CINEMATICA DEL CARRO INDEXADOR M-102
% Velocidad transversal del carro: v_cart = 0.38 m/s
% Desplazamiento máximo: Delta_z = 0.075 m
v_cart_max = 0.380; % m/s
delta_z_max = 0.075; % m
t_transito_cart = delta_z_max / v_cart_max; % = 0.197 s
margen_tiempo = T_desvio_s - t_transito_cart;

fprintf('4. SINCRONIZACION DEL CARRO INDEXADOR M-102:\n');
fprintf('   - Velocidad lineal del carro:    %.3f m/s\n', v_cart_max);
fprintf('   - Desplazamiento transversal:    %.3f m (75 mm)\n', delta_z_max);
fprintf('   - Tiempo de posicionamiento:     %.3f s (197 ms)\n', t_transito_cart);
fprintf('   - Margen dentro del pulso desvio:%.3f s (El carro llega antes de que caiga la caja)\n\n', margen_tiempo);

%% 5. SIMULACION TEMPORAL CONTINUA DE UNA SECUENCIA DE CLASIFICACION
% Simulamos el paso secuencial de 3 piezas:
% 1) Pieza 1 Conforme (90°)
% 2) Pieza 2 Retrabajo (45°)
% 3) Pieza 3 Rechazo   (135°)
dt = 0.002; % 2 ms
t_total = 7.0; % 7 segundos
t_vec = 0:dt:t_total;
N = length(t_vec);

theta_cmd = zeros(1, N);
theta_real = zeros(1, N);
theta_dot = zeros(1, N);
cart_z = zeros(1, N);
cart_z_cmd = zeros(1, N);

% Inicializar en 90° (neutro)
theta_real(1) = 90.0;
theta_cmd(:) = 90.0;
cart_z(1) = 0.0;

% Cronograma de eventos (Inyecciones de piezas)
% Pieza 1: Conforme en t = 0.5s -> Inspira en 0.5..1.3s -> Desvío 90° (1.3..2.3s)
% Pieza 2: Retrabajo en t = 2.5s -> Inspira en 2.5..3.3s -> Desvío 45° (3.3..4.3s)
% Pieza 3: Rechazo en t = 4.8s -> Inspira en 4.8..5.6s -> Desvío 135° (5.6..6.6s)

for k = 1:N
    t_curr = t_vec(k);
    if t_curr >= 3.3 && t_curr < 4.3
        theta_cmd(k) = 45.0;     % Retrabajo
        cart_z_cmd(k) = 0.075;
    elseif t_curr >= 5.6 && t_curr < 6.6
        theta_cmd(k) = 135.0;    % Rechazo
        cart_z_cmd(k) = -0.075;
    else
        theta_cmd(k) = 90.0;     % Conforme / Neutro
        cart_z_cmd(k) = 0.000;
    end
end

% Integración numérica del modelo de 2do orden con saturación de slew-rate
for k = 1:(N - 1)
    err_ang = theta_cmd(k) - theta_real(k);
    
    % Aceleración angular: d2_theta = wn^2 * err - 2*zeta*wn * d_theta
    theta_ddot = wn_servo^2 * err_ang - 2.0 * zeta_servo * wn_servo * theta_dot(k);
    
    % Integrar velocidad angular
    theta_dot_next = theta_dot(k) + theta_ddot * dt;
    % Saturación de velocidad angular por física del reductor
    theta_dot_next = max(-omega_max_deg_s, min(omega_max_deg_s, theta_dot_next));
    theta_dot(k+1) = theta_dot_next;
    
    % Integrar posición angular
    theta_real(k+1) = theta_real(k) + theta_dot(k) * dt;
    
    % Dinámica de traslación del carro M-102
    dz = cart_z_cmd(k) - cart_z(k);
    step_cart = v_cart_max * dt;
    if abs(dz) <= step_cart
        cart_z(k+1) = cart_z_cmd(k);
    else
        cart_z(k+1) = cart_z(k) + sign(dz) * step_cart;
    end
end

%% 6. GRAFICACION TECNICA
figure('Name', 'Lazo 2 - Dinamica del Servomotor y Enrutamiento', ...
       'Color', [0.96 0.96 0.96], 'Position', [150, 150, 1050, 750]);

% Subplot 1: Ángulo del Servomotor
subplot(3, 1, 1);
plot(t_vec, theta_cmd, '--', 'LineWidth', 1.5, 'Color', [0.85, 0.40, 0.10]); hold on;
plot(t_vec, theta_real, 'LineWidth', 2.0, 'Color', [0.10, 0.45, 0.85]);
yline(90, ':', 'Conforme / Neutro (90°)', 'Color', [0.3 0.3 0.3]);
yline(45, ':', 'Retrabajo (45°)', 'Color', [0.8 0.6 0.1]);
yline(135, ':', 'Rechazo (135°)', 'Color', [0.8 0.2 0.2]);
grid on; set(gca, 'Color', [1 1 1], 'GridColor', [0.85 0.85 0.85]);
ylabel('Angulo \theta [°]', 'FontWeight', 'bold');
title('Lazo 2: Respuesta Temporal Angular del Servomotor SG90', 'FontSize', 11, 'FontWeight', 'bold');
legend('Consigna de Enrutamiento', 'Angulo Real SG90', 'Location', 'northeast');
ylim([30, 150]); xlim([0, t_total]);

% Subplot 2: Velocidad Angular del Servo
subplot(3, 1, 2);
plot(t_vec, theta_dot, 'LineWidth', 1.8, 'Color', [0.55, 0.20, 0.70]); hold on;
yline(omega_max_deg_s, '--', 'Limite +500 °/s', 'Color', [0.8 0.2 0.2]);
yline(-omega_max_deg_s, '--', 'Limite -500 °/s', 'Color', [0.8 0.2 0.2]);
grid on; set(gca, 'Color', [1 1 1], 'GridColor', [0.85 0.85 0.85]);
ylabel('Vel. Angular d\theta/dt [°/s]', 'FontWeight', 'bold');
title('Velocidad Angular y Saturacion por Slew-Rate Mecanico', 'FontSize', 11, 'FontWeight', 'bold');
ylim([-600, 600]); xlim([0, t_total]);

% Subplot 3: Posición Transversal del Carro Móvil M-102
subplot(3, 1, 3);
plot(t_vec, cart_z_cmd * 1000, '--', 'LineWidth', 1.5, 'Color', [0.85, 0.40, 0.10]); hold on;
plot(t_vec, cart_z * 1000, 'LineWidth', 2.0, 'Color', [0.20, 0.65, 0.35]);
grid on; set(gca, 'Color', [1 1 1], 'GridColor', [0.85 0.85 0.85]);
xlabel('Tiempo t [s]', 'FontWeight', 'bold');
ylabel('Posicion Carro Z [mm]', 'FontWeight', 'bold');
title('Cinematica Sincronizada del Carro Indexador M-102', 'FontSize', 11, 'FontWeight', 'bold');
legend('Posicion Objetivo Z', 'Posicion Real Carro', 'Location', 'northeast');
ylim([-100, 100]); xlim([0, t_total]);

fprintf('Simulacion de Lazo 2 completada exitosamente. Grafico generado.\n');
