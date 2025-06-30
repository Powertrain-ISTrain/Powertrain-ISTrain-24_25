clc;
clear;

%% ================================================================
% ECO-WORTHY LiFePO4 Battery Technical Specifications and Features
% ================================================================
%
% Características principais:
% - Ideal para sistemas solares 24V off-grid e instalações em interiores ou exteriores
% - Células LiFePO4 de grau A com mais de 4000 ciclos de vida útil
% - Mantém até 80% da capacidade após 4000 ciclos profundos
% - BMS integrado com proteção contra sobrecarga, descarga, curto-circuito, sobrecorrente e alta temperatura
% - Suporta ligação em paralelo para aumento de capacidade
% - Recarga rápida: até 80% em 3 a 4 horas
% - Peso reduzido (1/3 do peso de baterias de chumbo-ácido) para facilitar transporte e instalação
% - Compatível com autocaravanas, embarcações, carrinhos de golfe, sistemas solares e alimentação de emergência
%
% Especificações técnicas:
% - Capacidade da bateria:          100 Ah
% - Energia total:                  2560 Wh
% - Tensão nominal:                 25.6 V
% - Tensão máxima de carga:         29.2 V
% - Corrente máx. de carga:         100 A ±5 A
% - Corrente máx. de descarga:      100 A ±5 A
% - Temperatura de funcionamento:   -20 a 55 °C
% - Ciclos de carga:                >4000 ciclos (a 80% capacidade residual)
% - Sistema de gestão:              BMS integrado com autoequilíbrio de células
% - Tipo de rosca dos terminais:    M8
% - Dimensões:                      36.5 × 18.8 × 27.2 cm
% - Peso:                           20.3 kg
%
% ================================================================

%% Time Parameters
dt = 1;                        % Time step [s]
t_end = 3 * 3600;              % Total simulation time: 3 hours [s]
t = 0:dt:t_end;

%% Power Profile (Constant)
base_power = 540.7 + 0.1 * 540.7;   % Base power in Watts
power_profile = base_power * ones(size(t));

%% Battery Pack Configuration (ECO-Worthy LiFePO4)
cell_voltage = 25.6;               % Nominal voltage per battery pack [V]
cell_capacity = 100;               % Ah
usable_capacity_Ah = cell_capacity * 0.8;  % 80% DoD

num_series = 2;                    % 2 in series
num_parallel = 2;                  % 2 in parallel

pack_voltage_nom = cell_voltage * num_series;          
pack_capacity_Ah = usable_capacity_Ah * num_parallel;
Q_total = pack_capacity_Ah * 3600;                      % [Coulombs]

% --- Print Battery Pack Specifications ---
fprintf('\n==== BATTERY PACK SPECIFICATIONS ====\n');
fprintf('Battery chemistry     : LiFePO₄\n');
fprintf('Battery model         : ECO-WORTHY 25.6V 100Ah\n');
fprintf('Nominal voltage       : %.1f V\n', pack_voltage_nom);
fprintf('Nominal capacity      : %.0f Ah\n', cell_capacity);
fprintf('Usable capacity       : %.0f Ah (80%% DoD)\n', pack_capacity_Ah);
fprintf('Pack energy (nominal) : %.0f Wh\n', pack_capacity_Ah * pack_voltage_nom);
fprintf('Total charge capacity : %.0f Coulombs\n', Q_total);

%% Battery Model Parameters
R0 = 0.015;                   % Internal resistance [Ohm]
R1 = 0.004;                   % RC branch resistance [Ohm]
C1 = 1500;                    % Capacitance [F]
voc_min = 22 * num_series;    % Cutoff voltage [V]

% --- Print Electrical Model Parameters ---
fprintf('\n---- ELECTRICAL MODEL PARAMETERS ----\n');
fprintf('Internal resistance R0: %.4f Ohm\n', R0);
fprintf('RC resistance R1      : %.4f Ohm\n', R1);
fprintf('RC capacitance C1     : %.0f F\n', C1);
fprintf('Voltage cutoff (min)  : %.1f V\n', voc_min);
fprintf('=====================================\n\n');

%% Energy Density and Mass Estimation
energy_density_Wh_per_kg = 126;  % [Wh/kg] from datasheet
pack_energy_Wh = 2560;           % Per module
battery_mass_kg = pack_energy_Wh / energy_density_Wh_per_kg;

fprintf('\n==== MASS ESTIMATION ====\n');
fprintf('Total energy stored per module   : %.1f Wh\n', pack_energy_Wh);
fprintf('Energy density                   : %.1f Wh/kg\n', energy_density_Wh_per_kg);
fprintf('Estimated battery mass (1 module): %.1f kg\n', battery_mass_kg);
fprintf('Estimated total battery mass     : %.1f kg\n', battery_mass_kg * num_series * num_parallel);
fprintf('=====================================\n\n');

%% Preallocate
soc = ones(size(t));
current = zeros(size(t));
voltage = zeros(size(t));
V_RC = zeros(size(t));
V_oc = zeros(size(t));
c_rate = zeros(size(t));
power_applied = zeros(size(t));

%% V_oc vs SoC Model (LiFePO4 curve)
soc_points = linspace(0, 1, 11);
voc_points = [22.0 24.0 25.0 25.5 25.9 26.2 26.5 26.9 27.5 28.3 29.2] * num_series; % [V]
V_ocv_lookup = @(soc_val) interp1(soc_points, voc_points, soc_val, 'linear', 'extrap');

%% Simulation Loop
for k = 2:length(t)
    V_oc(k) = V_ocv_lookup(soc(k-1));

    % Check SoC and voltage limits
    if soc(k-1) <= 0.1 || voltage(k-1) <= voc_min
        power = 0;
    else
        power = power_profile(k);
    end

    power_applied(k) = power;

    % Calculate current
    I = power / max(V_oc(k), 1e-3);
    % Enforce maximum discharge current
    if I / num_parallel > 100
        I = 100 * num_parallel;
        power = I * V_oc(k);
    end
    dV_RC = (-V_RC(k-1) + I * R1) / (R1 * C1) * dt;
    V_RC(k) = V_RC(k-1) + dV_RC;

    V_terminal = V_oc(k) - I * R0 - V_RC(k);

    if V_terminal < voc_min
        V_terminal = voc_min;
        I = 0;
        power_applied(k) = 0;
    end

    % Update SoC
    dQ = I * dt;
    Q_used = (1 - soc(k-1)) * Q_total + dQ;
    soc(k) = max(0, 1 - Q_used / Q_total);

    % Save outputs
    current(k) = I;
    voltage(k) = V_terminal;
    c_rate(k) = current(k) / pack_capacity_Ah;
end

%% Plot Results
figure;

subplot(5,1,1);
plot(t/60, power_applied, 'k', 'LineWidth', 1.5);
ylabel('Power (W)');
title('Power Demand Profile'); grid on;

subplot(5,1,2);
plot(t/60, soc*100, 'b', 'LineWidth', 1.5);
ylabel('SoC (%)');
title('State of Charge'); grid on;

subplot(5,1,3);
plot(t/60, voltage, 'r', 'LineWidth', 1.5);
ylabel('Voltage (V)');
title('Terminal Voltage'); grid on;

subplot(5,1,4);
plot(t/60, current, 'm', 'LineWidth', 1.5);
ylabel('Current (A)');
title('Battery Current'); grid on;

subplot(5,1,5);
plot(t/60, c_rate, 'c', 'LineWidth', 1.5);
xlabel('Time (min)');
ylabel('C-rate (1/h)');
title('C-rate Over Time'); grid on;

%% Save Power Profile
power_signal.time = t';
power_signal.signals.values = power_profile';
power_signal.signals.dimensions = 1;
save('power_profile.mat', 'power_signal');

