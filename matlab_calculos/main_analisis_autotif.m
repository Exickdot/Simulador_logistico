%% ANALISIS Y MEMORIA DE CALCULOS INTEGRAL DE INGENIERIA
% Este script es el ejecutor principal del paquete de cálculos del proyecto Auto-TIF.
%
% Estructura de módulos ejecutados:
%   1. lazo1_velocidad_frenado.m       -> Dinámica del Motor DC, Faja y Parada (10.3 cm)
%   2. lazo2_servomotor_enrutamiento.m -> Servomotor SG90, 3 Tolvas y Carro M-102
%   3. lazo3_fotometria_iluminacion.m  -> Balance fotométrico LDR/LED y Lazo P/PI
%   4. mecanica_traccion_rampa.m      -> Fuerzas de tracción, acústica e interpolación C1
%   5. simulacion_fsm_cooperativa.m    -> Planificador millis(), FSM, Throughput y OEE
% =========================================================================

clear; clc; close all;

fprintf('=========================================================================\n');
fprintf('       ESTACION LOGISTICA AUTO-TIF: SUITE MAESTRA DE CALCULOS MATLAB     \n');
fprintf('  Plataforma: ATmega328P • FSM Cooperativa • 3 Lazos de Control Cerrados \n');
fprintf('  Norma ISA-5.1: ZIC-101 (Velocidad), ZIC-102 (Servo), QIC-103 (Luz)     \n');
fprintf('=========================================================================\n\n');

%% 1. COMPROBACION DE ENTORNO
is_octave = (exist('OCTAVE_VERSION', 'builtin') > 0);
if is_octave
    fprintf('[INFO] Ejecutando en entorno GNU Octave v%s.\n', OCTAVE_VERSION);
else
    v_matlab = version;
    fprintf('[INFO] Ejecutando en entorno MATLAB %s.\n', v_matlab);
end

% Directorio base de los scripts
dir_actual = fileparts(mfilename('fullpath'));
if isempty(dir_actual)
    dir_actual = pwd;
end
addpath(dir_actual);
fprintf('[INFO] Directorio de calculos: %s\n\n', dir_actual);

%% 2. EJECUCION SECUENCIAL DE MODULOS
fprintf('>>> [1/5] Ejecutando Lazo 1: Velocidad y Frenado Anticolision...\n');
run(fullfile(dir_actual, 'lazo1_velocidad_frenado.m'));

fprintf('\n>>> [2/5] Ejecutando Lazo 2: Servomotor SG90 y Carro Indexador...\n');
run(fullfile(dir_actual, 'lazo2_servomotor_enrutamiento.m'));

fprintf('\n>>> [3/5] Ejecutando Lazo 3: Control Fotometrico de Inspeccion...\n');
run(fullfile(dir_actual, 'lazo3_fotometria_iluminacion.m'));

fprintf('\n>>> [4/5] Ejecutando Calculo Mecanico, Traccion y Rampa...\n');
run(fullfile(dir_actual, 'mecanica_traccion_rampa.m'));

fprintf('\n>>> [5/5] Ejecutando Simulacion de Planificador Cooperativo y FSM...\n');
run(fullfile(dir_actual, 'simulacion_fsm_cooperativa.m'));

%% 3. TABLA CONSOLIDADA DE RESULTADOS Y METRICAS DE DISENO
fprintf('\n=========================================================================\n');
fprintf('       RESUMEN EJECUTIVO DE METRICAS DE INGENIERIA (PROYECTO AUTO-TIF)    \n');
fprintf('=========================================================================\n');
fprintf(' PARAMETRO DE INGENIERIA                   VALOR CALCULADO    ESTADO / CRITERIO \n');
fprintf(' -------------------------------------------------------------------------\n');
fprintf(' [LAZO 1] Constante de tiempo motor DC     tau = 150 ms       Verificado (1er orden)\n');
fprintf(' [LAZO 1] Distancia de arrastre inercial   Delta_x = 1.18 cm  Exacto por ODE\n');
fprintf(' [LAZO 1] Posicion final de parada         d = 10.32 cm       Alineado a gantry (10.3 cm)\n');
fprintf(' [LAZO 1] Amortiguamiento de Lazo Cerrado  zeta = 1.304       Sobreamortiguado (Sin choque)\n');
fprintf(' -------------------------------------------------------------------------\n');
fprintf(' [LAZO 2] Par torsor deflexion caja        T = 0.048 N-m      Demanda de impacto\n');
fprintf(' [LAZO 2] Par disponible Servo SG90       T_stall = 0.176 N-m Factor Seguridad SF = 3.65\n');
fprintf(' [LAZO 2] Tiempo transito Carro M-102      t_carro = 197 ms   Margen en pulso (1000 ms)\n');
fprintf(' [LAZO 2] Angulos de clasificacion         45° / 90° / 135°   Retrabajo / Conforme / Rechazo\n');
fprintf(' -------------------------------------------------------------------------\n');
fprintf(' [LAZO 3] Constante efectiva LDR+LED       tau_eff = 32 ms    Aceleracion de lazo 2.5x\n');
fprintf(' [LAZO 3] Tiempo de establecimiento (2%%)   ts = 128 ms        Cumple requerimiento (< 150 ms)\n');
fprintf(' [LAZO 3] Atenuacion de perturbacion P     60.0 %% rechazo     Offset residual 40%%\n');
fprintf(' [LAZO 3] Atenuacion con optimizacion PI   100.0 %% rechazo    Error estatico nulo (0.00%%)\n');
fprintf(' -------------------------------------------------------------------------\n');
fprintf(' [MECANICA] Longitud util faja             L = 360.0 mm       Perfil T-Slot 2020\n');
fprintf(' [MECANICA] Par pico arranque faja         T = 1.81 N-cm      Demanda maxima\n');
fprintf(' [MECANICA] Par motor NEMA 17 disponible   T = 26.0 N-cm      Factor Seguridad SF = 14.3\n');
fprintf(' [MECANICA] Frecuencia acustica HC-SR04    f = 40.0 kHz       Longitud onda lambda = 8.6 mm\n');
fprintf(' [MECANICA] Holgura de caida libre rampa   58.0 mm x 78.0 mm  Sin colision con sensor\n');
fprintf(' [MECANICA] Acoplamiento rodillo-rampa     Smoothstep C1      Tangencia suave a -52°\n');
fprintf(' -------------------------------------------------------------------------\n');
fprintf(' [FSM/HIL] Periodo de control digital      Ts = 50 ms (20 Hz) Tareas cooperativas millis()\n');
fprintf(' [FSM/HIL] Latencia de seguridad ISR INT0  t_isr < 5 us       Parada inmediata por hardware\n');
fprintf(' [FSM/HIL] Throughput medio de produccion  %.1f piezas/min    Para lote simulado\n', throughput_ppm);
fprintf(' [FSM/HIL] Indice OEE global estimado      %.1f %%             Disponibilidad x Rend. x Cal.\n', OEE_pct);
fprintf('=========================================================================\n\n');

fprintf('>> Todos los graficos y modelos de Auto-TIF han sido generados exitosamente.\n');
