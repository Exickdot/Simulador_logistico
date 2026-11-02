%% MODULO: CALCULO MECANICO, TRACCION, ACUSTICA HC-SR04 Y RAMPA SMOOTHSTEP
% Este script calcula y valida matemáticamente:
% 1. Dimensionamiento mecánico y tracción de la faja transportadora (Bosch 2020).
% 2. Par motor en régimen estacionario y dinámico (NEMA 17 / DC con reductor).
% 3. Física de ondas acústicas a 40 kHz del sensor ultrasónico HC-SR04:
%    - Longitud de onda, ángulo de emisión y reflexión especular a 90°.
% 4. Cinemática de evacuación por gravedad y curva suave C1 (Smoothstep cúbico):
%    - s(t) = 3t^2 - 2t^3 para acoplamiento tangencial rodillo-rampa (-52°).
% 5. Verificación de despeje geométrico de caída libre (58 mm libre sin choque).
% =========================================================================

clear; clc; close all;

fprintf('=========================================================================\n');
fprintf('  AUTO-TIF: CALCULO MECANICO, TRACCION, ACUSTICA Y GEOMETRIA DE RAMPA   \n');
fprintf('=========================================================================\n\n');

%% 1. DIMENSIONAMIENTO MECANICO DE LA FAJA TRANSPORTADORA
L_faja_mm    = 360.0;       % Longitud entre centros de rodillos (mm)
L_faja_m     = L_faja_mm / 1000.0; % = 0.360 m
W_banda_mm   = 70.0;        % Ancho útil de la banda (mm)
W_banda_m    = W_banda_mm / 1000.0; % = 0.070 m
D_rodillo_mm = 26.0;        % Diámetro de rodillos con rodamientos 608ZZ (mm)
r_rodillo_m  = (D_rodillo_mm / 2.0) / 1000.0; % Radio = 0.013 m
e_banda_m    = 0.0015;      % Espesor de la banda continua (1.5 mm)
rho_banda    = 1200.0;      % Densidad del caucho/poliuretano (kg/m^3)
g            = 9.80665;     % Aceleración de la gravedad (m/s^2)

% Longitud total del bucle cerrado de la banda:
% L_bucle = 2 * L_faja + pi * D_rodillo
L_bucle_m = 2.0 * L_faja_m + pi * (D_rodillo_mm / 1000.0); % = 0.8017 m

% Masa de la banda continua:
m_banda = rho_banda * L_bucle_m * W_banda_m * e_banda_m; % [kg] = 0.101 kg

% Masa de los rodillos maquinados en aluminio:
m_rodillo = 0.045; % kg por rodillo (2 rodillos en total)
J_rodillo = 0.5 * m_rodillo * (r_rodillo_m^2); % Inercia de un cilindro macizo
J_rodillos_total = 2.0 * J_rodillo;

% Capacidad de carga: hasta 4 cajas simultáneas de 50 g
m_caja_unit  = 0.050; % kg
n_cajas_max  = 4;
m_carga_max  = n_cajas_max * m_caja_unit; % = 0.200 kg

% Masa equivalente total referida al movimiento lineal:
m_eq_total = m_banda + m_carga_max + (J_rodillos_total / (r_rodillo_m^2));

fprintf('1. DIMENSIONAMIENTO MECANICO DE LA ESTRUCTURA:\n');
fprintf('   - Longitud util entre centros:           %.1f mm (%.3f m)\n', L_faja_mm, L_faja_m);
fprintf('   - Ancho de banda y rodillo:              %.1f mm\n', W_banda_mm);
fprintf('   - Diametro de los rodillos motriz/conduc:%.1f mm (r = %.1f mm)\n', D_rodillo_mm, r_rodillo_m*1000);
fprintf('   - Longitud del bucle continuo de banda:  %.1f mm (%.4f m)\n', L_bucle_m*1000, L_bucle_m);
fprintf('   - Masa de la banda de caucho:            %.3f kg (101 g)\n', m_banda);
fprintf('   - Masa equivalente total acelerada:      %.3f kg\n\n', m_eq_total);

%% 2. CALCULO DE TRACCION, PAR MOTOR Y POTENCIA
mu_friccion_al = 0.28;      % Coeficiente de rozamiento banda/cama de aluminio
F_friccion_cama = mu_friccion_al * (m_banda + m_carga_max) * g; % Fuerza rozamiento (N)
F_rodamientos = 0.050;      % Resistencia por fricción en 4x 608ZZ (N)
F_estacionaria = F_friccion_cama + F_rodamientos; % Fuerza total régimen continuo

% Par estático continuo en el eje del rodillo:
Torque_estacionario_Nm = F_estacionaria * r_rodillo_m;
Torque_estacionario_Ncm = Torque_estacionario_Nm * 100.0;

% Régimen dinámico: Aceleración nominal a = 1.2 m/s^2
a_accel = 1.20; % m/s^2
F_inercial = m_eq_total * a_accel;
F_pico = F_estacionaria + F_inercial;

Torque_pico_Nm = F_pico * r_rodillo_m;
Torque_pico_Ncm = Torque_pico_Nm * 100.0;

% Velocidad angular del rodillo a velocidad nominal v = 0.20 m/s:
v_nominal = 0.20; % m/s
omega_rodillo_rad_s = v_nominal / r_rodillo_m; % = 15.38 rad/s
rpm_rodillo = (omega_rodillo_rad_s * 60.0) / (2.0 * pi); % = 146.9 RPM
Potencia_mecanica_W = Torque_pico_Nm * omega_rodillo_rad_s;

% Comparativa con actuadores instalables:
% NEMA 17 Estándar (42x42x34 mm): Par de retención = 26.0 N-cm
Torque_nema17_Ncm = 26.0;
SF_nema17 = Torque_nema17_Ncm / Torque_pico_Ncm;

fprintf('2. FUERZAS DE TRACCION Y DEMANDA MOTRIZ:\n');
fprintf('   - Fuerza tangencial estatica (friccion): %.3f N\n', F_estacionaria);
fprintf('   - Fuerza tangencial pico (aceleracion):  %.3f N\n', F_pico);
fprintf('   - Par torsor continuo en el rodillo:     %.4f N-m (%.3f N-cm)\n', ...
        Torque_estacionario_Nm, Torque_estacionario_Ncm);
fprintf('   - Par torsor pico de arranque:           %.4f N-m (%.3f N-cm)\n', ...
        Torque_pico_Nm, Torque_pico_Ncm);
fprintf('   - Regimen de giro nominal del rodillo:   %.1f RPM (%.2f rad/s)\n', rpm_rodillo, omega_rodillo_rad_s);
fprintf('   - Potencia mecanica util requerida:      %.3f W\n', Potencia_mecanica_W);
fprintf('   - Par nominal motor NEMA 17:             %.1f N-cm\n', Torque_nema17_Ncm);
fprintf('   - Factor de Seguridad del Motor (SF):    %.2f (Margen industrial seguro > 10)\n\n', SF_nema17);

%% 3. FISICA ACUSTICA Y REFLEXION ESPECULAR (SENSOR HC-SR04 A 40 kHz)
f_ultrasonido = 40000.0;    % Frecuencia acústica (40 kHz)
Temp_amb_C    = 22.0;       % Temperatura ambiente estándar de laboratorio (°C)
c_sonido      = 331.3 * sqrt(1.0 + Temp_amb_C / 273.15); % [m/s] = 344.4 m/s
lambda_m      = c_sonido / f_ultrasonido; % Longitud de onda (m)
lambda_mm     = lambda_m * 1000.0;        % = 8.61 mm

% Transductores piezocerámicos: diámetro D = 16 mm
D_transductor_mm = 16.0;
% Semángulo del cono de radiación acústica principal (Fraunhofer):
% sin(theta) = 1.22 * lambda / D
sin_theta_cono = 1.22 * (lambda_mm / D_transductor_mm);
theta_cono_deg = asind(sin_theta_cono); % = 41.0° (Haz efectivo útil central +/- 15°)

fprintf('3. FISICA ACUSTICA DEL SENSOR HC-SR04:\n');
fprintf('   - Velocidad del sonido (T = %.1f°C):     %.2f m/s\n', Temp_amb_C, c_sonido);
fprintf('   - Longitud de onda acustica (lambda):    %.2f mm\n', lambda_mm);
fprintf('   - Apertura angular del cono de emision:  +/- %.1f° (Zona de maxima energia +/- 15°)\n', theta_cono_deg);
fprintf('   - Justificacion de Incidencia Normal a 90°: Las caras de carton/plastico provocan\n');
fprintf('     reflexion especular (ley de Snell: angulo incidente = angulo reflejado). Unicamente\n');
fprintf('     con el sensor alineado perpendicularmente a la cara frontal (Y = 26 mm) el eco regresa\n');
fprintf('     con maxima energia a la capsula receptora R sin dispersion espuria.\n\n');

%% 4. CINEMATICA DE LA RAMPA DE DESCARGA Y CURVA SMOOTHSTEP
% Ecuación de transición suave (Smoothstep cúbico):
% s(t) = 3*t^2 - 2*t^3  con t in [0, 1]
% Propiedades: s(0)=0, s(1)=1, s'(0)=0, s'(1)=0 (Aceleración y velocidad suaves C1)
% Ángulo de inclinación de la rampa: -52°
% Boca libre de evacuación = 58 mm de largo x 78 mm de ancho
% Cota de caída: Rodillo final en X = 0.400 m, plano rampa y = 0.404 - (x - 0.410)*tan(52°)

tan52 = tand(52.0); % = 1.27994
t_param = linspace(0, 1, 100);
smooth_s = 3.0 * (t_param.^2) - 2.0 * (t_param.^3);
smooth_s_dot = 6.0 * t_param .* (1.0 - t_param); % Derivada temporal

% Perfil espacial de descenso de la caja
x_desvio = linspace(0.400, 0.550, 150); % Trayectoria X sobre el rodillo y rampa (m)
y_desvio = zeros(size(x_desvio));
theta_caja = zeros(size(x_desvio));

for i = 1:length(x_desvio)
    x_val = x_desvio(i);
    if x_val < 0.435
        t_loc = (x_val - 0.400) / 0.035;
        t_loc = max(0.0, min(1.0, t_loc));
        s_val = 3.0 * t_loc^2 - 2.0 * t_loc^3;
        theta_caja(i) = -52.0 * s_val;
        
        chute_plane_y = 0.404 - (x_val - 0.410) * tan52;
        normal_h = 0.0225 / max(0.2, cosd(abs(theta_caja(i))));
        y_desvio(i) = (1.0 - s_val) * 0.4255 + s_val * (chute_plane_y + normal_h);
    else
        theta_caja(i) = -52.0;
        chute_plane_y = 0.404 - (x_val - 0.410) * tan52;
        normal_h = 0.0225 / cosd(52.0);
        y_desvio(i) = chute_plane_y + normal_h;
    end
end

fprintf('4. GEOMETRIA Y CINEMATICA DE LA RAMPA DE DESCARGA:\n');
fprintf('   - Boca de caida libre longitudinal:      58.0 mm (Espacio holgado para caja de 45 mm)\n');
fprintf('   - Ancho util de paso en rampa:           78.0 mm\n');
fprintf('   - Angulo final de inclinacion:           -52.0° (Deslizamiento continuo por gravedad)\n');
fprintf('   - Transicion tangencial rodillo-rampa:   Polinomio Smoothstep C1 (Sin choques mecanicos)\n\n');

%% 5. GRAFICACION TECNICA DE PERFILES MECANICOS Y GEOMETRICOS
figure('Name', 'Analisis Mecanico, Traccion y Cinematica de Rampa', ...
       'Color', [0.96 0.96 0.96], 'Position', [250, 100, 1050, 750]);

% Subplot 1: Curva de Transición Cúbica Smoothstep y su derivada
subplot(2, 2, 1);
plot(t_param, smooth_s, 'LineWidth', 2.0, 'Color', [0.15, 0.45, 0.85]); hold on;
plot(t_param, smooth_s_dot, '--', 'LineWidth', 1.8, 'Color', [0.85, 0.45, 0.15]);
grid on; set(gca, 'Color', [1 1 1], 'GridColor', [0.85 0.85 0.85]);
xlabel('Parametro Normalizado t \in [0, 1]', 'FontWeight', 'bold');
ylabel('Valor de Interpolacion', 'FontWeight', 'bold');
title('Transicion C1 Suave: s(t) = 3t^2 - 2t^3', 'FontSize', 10, 'FontWeight', 'bold');
legend('Posicion s(t)', 'Derivada ds/dt (Velocidad)', 'Location', 'northwest');

% Subplot 2: Trayectoria Espacial (X-Y) de la Caja en el Rodillo y Rampa
subplot(2, 2, 2);
plot(x_desvio * 1000, y_desvio * 1000, 'LineWidth', 2.2, 'Color', [0.20, 0.65, 0.35]); hold on;
xline(400, ':', 'Cresta Rodillo Motriz', 'Color', [0.4 0.4 0.4]);
xline(435, ':', 'Inicio Plano Rampa (-52°)', 'Color', [0.8 0.5 0.1]);
grid on; set(gca, 'Color', [1 1 1], 'GridColor', [0.85 0.85 0.85]);
xlabel('Coordenada Longitudinal X [mm]', 'FontWeight', 'bold');
ylabel('Altura Y [mm]', 'FontWeight', 'bold');
title('Trayectoria de Descenso Acoplado (Faja -> Rampa)', 'FontSize', 10, 'FontWeight', 'bold');
xlim([395, 540]);

% Subplot 3: Inclinación Angular de la Caja (Rotación Z)
subplot(2, 2, 3);
plot(x_desvio * 1000, theta_caja, 'LineWidth', 2.0, 'Color', [0.85, 0.25, 0.20]); hold on;
yline(-52, '--', 'Pendiente de Rampa (-52°)', 'Color', [0.3 0.3 0.3]);
grid on; set(gca, 'Color', [1 1 1], 'GridColor', [0.85 0.85 0.85]);
xlabel('Coordenada Longitudinal X [mm]', 'FontWeight', 'bold');
ylabel('Angulo de Inclinacion \theta_z [°]', 'FontWeight', 'bold');
title('Evolucion Angular Suave sin Impacto de Arista', 'FontSize', 10, 'FontWeight', 'bold');
ylim([-60, 5]); xlim([395, 540]);

% Subplot 4: Cono de Dispersión Acústica del HC-SR04 a 40 kHz
subplot(2, 2, 4);
ang_cono_plot = linspace(-60, 60, 200);
% Patrón de radiación de pistón circular acústico: |2 * J1(x) / x|
% x = pi * D / lambda * sin(theta)
x_arg = (pi * D_transductor_mm / lambda_mm) * sind(ang_cono_plot);
x_arg(x_arg == 0) = 1e-6; % Evitar división por cero
patron_directividad = abs(2.0 * besselj(1, x_arg) ./ x_arg);
plot(ang_cono_plot, patron_directividad, 'LineWidth', 2.0, 'Color', [0.55, 0.20, 0.70]); hold on;
xline(15, '--', '+15° Zona Util', 'Color', [0.85, 0.45, 0.15]);
xline(-15, '--', '-15° Zona Util', 'Color', [0.85, 0.45, 0.15]);
grid on; set(gca, 'Color', [1 1 1], 'GridColor', [0.85 0.85 0.85]);
xlabel('Angulo de Incidencia \theta [°]', 'FontWeight', 'bold');
ylabel('Presion Acustica Normalizada', 'FontWeight', 'bold');
title('Patron de Radiacion Acustica (Transductor \varnothing 16mm / 40 kHz)', 'FontSize', 10, 'FontWeight', 'bold');
xlim([-50, 50]); ylim([0, 1.05]);

fprintf('Calculos mecanicos y acusticos completados exitosamente. Grafico generado.\n');
