clc;
clear;

%% Time Parameters
dt = 1;                        % Time step [s]
t_end = 70*60;                % Total simulation time [s] (60 min)
t = 0:dt:t_end;

%% Power Profile
base_power = 540.7;     % Base power in Watts

% Define peak times (in seconds) and corresponding peak values (in Watts)
t_peaks = [100, 250, 400, 550, 700, 850, 1000, 1150, 1300, ...
           1450, 1600, 1750, 1900, 2050, 2200, 2350, 2500, ...
           2650, 2800, 2950, 3100, 3250, 3400, 3550, 3700, 3850, 4000, 4150];

peak_values = [2000, 3000, 5000, 3500, 6900, 2500, 4500, 3000, 5500, ...
               2000, 4000, 3000, 6500, 2500, 5000, 6000, 3500, ...
               2500, 5800, 4200, 3900, 6100, 3000, 4300, 4700, 3500, 4000, 6900];

sigma = 50;  % Width of each peak in seconds

% Start with base power profile
power_profile = base_power * ones(size(t));

% Superimpose each Gaussian peak
for i = 1:length(t_peaks)
    power_profile = power_profile + ...
        (peak_values(i) - base_power) * exp(-0.5 * ((t - t_peaks(i))/sigma).^2);
end


%% Battery Pack Configuration (LiFePO4)
cell_voltage = 24;             % Each battery
cell_capacity = 100;           % Ah
num_series = 2;                % 2 in series => 48 V
num_parallel = 2;              % 2 branches in parallel => 200 Ah

pack_voltage_nom = cell_voltage * num_series;           % 48 V
pack_capacity_Ah = cell_capacity * num_parallel;        % 200 Ah
Q_total = pack_capacity_Ah * 3600;                      % [Coulombs]

% --- Print Battery Pack Specifications ---
fprintf('\n==== BATTERY PACK SPECIFICATIONS ====\n');
fprintf('Battery chemistry     : LiFePO₄\n');
fprintf('Cell voltage          : %.1f V\n', cell_voltage);
fprintf('Cell capacity         : %.0f Ah\n', cell_capacity);
fprintf('Cells in series       : %d\n', num_series);
fprintf('Cells in parallel     : %d\n', num_parallel);
fprintf('Pack nominal voltage  : %.1f V\n', pack_voltage_nom);
fprintf('Pack total capacity   : %.0f Ah\n', pack_capacity_Ah);
fprintf('Pack energy (nominal) : %.0f Wh\n', pack_capacity_Ah * pack_voltage_nom);
fprintf('Total charge capacity : %.0f Coulombs\n', Q_total);

%% Battery Model Parameters
R0 = 0.02;                   % Internal resistance [Ohm]
R1 = 0.005;                  
C1 = 1000;                   % Assumed large for LiFePO₄
voc_min = 42;                % Cutoff voltage

% --- Print Electrical Model Parameters ---
fprintf('\n---- ELECTRICAL MODEL PARAMETERS ----\n');
fprintf('Internal resistance R0: %.4f Ohm\n', R0);
fprintf('RC resistance R1      : %.4f Ohm\n', R1);
fprintf('RC capacitance C1     : %.0f F\n', C1);
fprintf('Voltage cutoff (min)  : %.1f V\n', voc_min);
fprintf('=====================================\n\n');

%% Preallocate
soc = ones(size(t));
current = zeros(size(t));
voltage = zeros(size(t));
V_RC = zeros(size(t));
V_oc = zeros(size(t));
c_rate = zeros(size(t));
power_applied = zeros(size(t));

%% V_oc vs SoC Model (from LiFePO4 0.1C curve)
soc_points = linspace(0, 1, 11);  % SoC from 0 to 100%
voc_points = [24.0 25.2 25.4 25.6 25.7 25.8 25.9 26.0 26.2 26.4 26.8] * num_series;  % [V]
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
    c_rate(k) = current(k) / pack_capacity_Ah;  % [1/h]
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

power_signal.time = t';
power_signal.signals.values = power_profile';
power_signal.signals.dimensions = 1;
save('power_profile.mat', 'power_signal');