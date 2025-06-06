clc;
clear;

%% Simulation Parameters
dt = 1;                        % Time step [s]
t_end = 100*60;                % Total simulation time [s] (180 min)
t = 0:dt:t_end;

%% Power Profile (includes a peak at t = 600 s)
base_power = 540.7;            % Base power in Watts
peak_power = 1000000;%3689;             % Peak power in Watts
power_profile = base_power + (peak_power - base_power) * exp(-0.5 * ((t - 600)/30).^2);

%% Cell Parameters (21700 Li-ion)
cell_voltage_nom = 3.6;        % Nominal voltage [V]
cell_capacity_Ah = 4.0;        % Capacity [Ah]
cell_mass_g = 70;              % Mass per cell [g]
R0 = 0.01;                     % Internal resistance [Ohm]
R1 = 0.005;                    % RC resistance [Ohm]
C1 = 500;                      % RC capacitance [F]

%% Pack Configuration
num_series = 20;               % Number of cells in series
num_parallel = 6;              % Number of cells in parallel
pack_voltage_nom = cell_voltage_nom * num_series;
pack_capacity_Ah = cell_capacity_Ah * num_parallel;
Q_total = pack_capacity_Ah * 3600;  % Total charge [Coulombs]

%% Safety Limits
soc_min = 0.10;                % Minimum SoC threshold (10%)
voc_min = 50;                    % Minimum terminal voltage [V]

%% Preallocate Variables
soc = ones(size(t));           % State of Charge (initial = 100%)
voltage = zeros(size(t));
current = zeros(size(t));
V_oc = zeros(size(t));
V_RC = zeros(size(t));
power_applied = zeros(size(t)); % Power actually drawn from the pack

%% Open-Circuit Voltage vs SoC Function (piecewise linear)
V_ocv_lookup = @(soc_val) interp1([0 0.5 1.0], [2.5 3.6 4.2]*num_series, soc_val, 'linear', 'extrap');

%% Simulation Loop
for k = 2:length(t)
    V_oc(k) = V_ocv_lookup(soc(k-1));

    % Limit discharge if below SoC or voltage threshold
    if soc(k-1) <= soc_min || voltage(k-1) <= voc_min
        power = 0;
    else
        power = power_profile(k);
    end

    power_applied(k) = power;

    % Compute current
    I = power / max(V_oc(k), 1e-3);   % Avoid divide by zero
    dV_RC = (-V_RC(k-1) + I * R1) / (R1 * C1) * dt;
    V_RC(k) = V_RC(k-1) + dV_RC;
    V_terminal = V_oc(k) - I * R0 - V_RC(k);

    % Enforce voltage cutoff
    if V_terminal < voc_min
        V_terminal = voc_min;
        I = 0;
        power_applied(k) = 0;
    end

    % Update SoC
    dQ = I * dt;  % Coulombs
    Q_used = (1 - soc(k-1)) * Q_total + dQ;
    soc(k) = max(0, 1 - Q_used / Q_total);

    % Store current and voltage
    current(k) = I;
    voltage(k) = V_terminal;
end

%% Plot Results
figure;

subplot(4,1,1);
plot(t/60, power_applied, 'k', 'LineWidth', 1.5);
ylabel('Power (W)');
title('Power Demand Profile'); grid on;

subplot(4,1,2);
plot(t/60, soc*100, 'b', 'LineWidth', 1.5);
ylabel('SoC (%)');
title('State of Charge'); grid on;

subplot(4,1,3);
plot(t/60, voltage, 'r', 'LineWidth', 1.5);
ylabel('Voltage (V)');
title('Terminal Voltage'); grid on;

subplot(4,1,4);
plot(t/60, current, 'm', 'LineWidth', 1.5);
xlabel('Time (min)');
ylabel('Current (A)');
title('Battery Current'); grid on;