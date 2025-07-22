%% run_ISTrain2025.m
% This script configures and runs the ISTrain2025 and  it plots the drive cycle,
% efficiency, torque‐speed map, and battery signals.

clear; close all; clc;
modelName = 'ISTrain2025';   

%% Define fixed vehicle parameters
m_loco    = 600;      % loco mass [kg]
Crr       = 0.004;    % rolling resistance coeff
g         = 9.81;     % gravity [m/s^2]
rw        = 0.100;    % wheel radius [m]
mech_eff  = 0.85;     % drivetrain mechanical efficiency
G         = 8;       % gearbox ratio
mu=0.35; % Friction coefficient (for adhesion purposes)
n_m=2; % Number of motors
SF=15; % Safety margin on torque (in percentage)

%% Solver fixed‑step size (for model config)
StepSize = 0.001;               % [s]
assignin('base','StepSize',StepSize);

%% 0) User choices
mt = 1;    % Motor choice
bt=1;      % Battery choice
dc = 4;    % Drive‑cycle choice


%% 1) Motor parameters as Simulink.Parameter objects
% BLDC HMP‑3000 example values

switch mt

    case 1

    %Description: 
    %BLDC Motor HMP-3000 /   Nominal Power : 3000 W / Nominal Voltage: 48V / Nominal Current: 80 / Nominal Torque: 9.4 N.m / Nominal
    %speed 4000 rpm

    P_nom   = 3000;        % Potência nominal [W]
    V_nom   = 48;          % Tensão nominal [V]
    I_nom   = 80;          % Corrente nominal [A]
    T_nom   = 9.4;         % Binário nominal [Nm]
    rpm_nom = 4000;        % Velocidade nominal [rpm]
    eta_nom=0.9;
    %Motor Parameters
    [Kt, Ke, R, kc, kf, invEff] = estimate_motor_constants(P_nom, V_nom, I_nom, T_nom, rpm_nom, eta_nom);
    % Display results
    
    Kt = Simulink.Parameter(Kt);   Kt.CoderInfo.StorageClass = 'SimulinkGlobal';
    Ke = Simulink.Parameter(Ke);   Ke.CoderInfo.StorageClass = 'SimulinkGlobal';
    R  = Simulink.Parameter(R);    R.CoderInfo.StorageClass  = 'SimulinkGlobal';
    kc = Simulink.Parameter(kc);    kc.CoderInfo.StorageClass = 'SimulinkGlobal';
    kf = Simulink.Parameter(kf);    kf.CoderInfo.StorageClass = 'SimulinkGlobal';
    invEff = Simulink.Parameter(invEff); invEff.CoderInfo.StorageClass = 'SimulinkGlobal';

    % Torque Speed Map
    mapData = readmatrix('HMP-3000.xlsx');
    speed_map_rpm = mapData(:,1);
    torque_map_Nm = mapData(:,2);

    case 2

    %Description: 
    %BLDC_108 /   Nominal Power : 1.5 W / Nominal Voltage: 48V / Nominal Current: 20 / Nominal Torque: 2.5 N.m / Nominal
    %speed 3000 rpm


     %Motor Parameters
    Kt = Simulink.Parameter(0.138);   Kt.CoderInfo.StorageClass = 'SimulinkGlobal';
    Ke = Simulink.Parameter(0.138);   Ke.CoderInfo.StorageClass = 'SimulinkGlobal';
    R  = Simulink.Parameter(0.14);    R.CoderInfo.StorageClass  = 'SimulinkGlobal';
    kc = Simulink.Parameter(0.193);    kc.CoderInfo.StorageClass = 'SimulinkGlobal';
    kf = Simulink.Parameter(2.6e-4);    kf.CoderInfo.StorageClass = 'SimulinkGlobal';
    invEff = Simulink.Parameter(1.11); invEff.CoderInfo.StorageClass = 'SimulinkGlobal';

    % Torque Speed Map
    mapData = readmatrix('BLDC_108.xlsx');
    speed_map_rpm = mapData(:,1);
    torque_map_Nm = mapData(:,2);


    case 3

     %Motor option 3
end

% Push to base workspace
assignin('base','Kt',           Kt);
assignin('base','Ke',           Ke);
assignin('base','R',            R);
assignin('base','kc',           kc);
assignin('base','kf',           kf);
assignin('base','invEff',       invEff);
assignin('base','speed_map_rpm',speed_map_rpm);
assignin('base','torque_map_Nm',torque_map_Nm);

%% 2) Battery Definition

switch bt
    case 1
        % LiFePO4 100Ah x2 in series (2 batteries total)
        parameters = struct;
        parameters.usable_capacity_Ah = 100*0.9 ;        % 80 Ah usable (series = same Ah)
        parameters.pack_voltage_nom   = 2 * 25.6;         % 51.2 V nominal
        parameters.R0 = 0.015 * 2;                        % Series: R adds up
        parameters.R1 = 0.004 * 2;
        parameters.C1 = 1500 / 2;                         % Series: C1 halves
        parameters.voc_min = 2 * 22;                      % 44 V minimum
        parameters.I_max = 120;                           % Same as single battery

        % VOC vs SOC based on LiFePO₄ cell characteristics
        voc_cell = [2.5 3.0 3.15 3.25 3.3 3.35 3.6];       % single cell
        soc_cell = [0.0 0.1 0.2 0.5 0.8 0.9 1.0];
        voc_pack = 16 * voc_cell;                         % 2S: 8×2 = 16 cells
        parameters.soc_points = linspace(0, 1, 21);       % high-resolution curve
        parameters.voc_points = interp1(soc_cell, voc_pack, parameters.soc_points, 'pchip');  % smooth

    case 2
        % LiFePO4 100Ah x2 in parallel AND x2 in series (4 batteries total)
        parameters = struct;
        parameters.usable_capacity_Ah = 2 * 100 * 0.8;   % 160 Ah usable (parallel defines capacity)
        parameters.pack_voltage_nom   = 2 * 25.6;        % 51.2 V (series doubles voltage)
        parameters.R0 = 0.015 * 2;                       % Series: R adds up → double R0
        parameters.R1 = 0.004 * 2;                       % Series: R adds up → double R1
        parameters.C1 = 1500 / 2;                        % Series halves C1
        parameters.voc_min = 2 * 22;                     % 44 V cutoff for 2 cells in series
        parameters.I_max = 200;                          % Same current as 2P config
        voc_cell = [2.5 3.0 3.15 3.25 3.3 3.35 3.6];
        soc_cell = [0.0 0.1 0.2 0.5 0.8 0.9 1.0];
        voc_pack = 16 * voc_cell;
        parameters.soc_points = linspace(0, 1, 21);
        parameters.voc_points = interp1(soc_cell, voc_pack, parameters.soc_points, 'pchip');


    case 3
        % Example: LTO 40Ah x3 in parallel
        parameters = struct;
        parameters.usable_capacity_Ah = 3 * 40 * 0.95;   % 114 Ah usable
        parameters.pack_voltage_nom   = 23.1;
        parameters.R0 = 0.008;
        parameters.R1 = 0.003;
        parameters.C1 = 1800;
        parameters.voc_min = 20.5;
        parameters.I_max = 240;
        parameters.soc_points = linspace(0, 1, 11);
        parameters.voc_points = [20.5 21 21.5 22 22.5 23 23.5 24 24.5 25 25.5];

    otherwise
        error('Battery type %d not implemented.', bt);
end

% Define Bus Elements using property-value syntax
elems(1) = Simulink.BusElement;
elems(1).Name = 'usable_capacity_Ah';

elems(2) = Simulink.BusElement;
elems(2).Name = 'pack_voltage_nom';

elems(3) = Simulink.BusElement;
elems(3).Name = 'R0';

elems(4) = Simulink.BusElement;
elems(4).Name = 'R1';

elems(5) = Simulink.BusElement;
elems(5).Name = 'C1';

elems(6) = Simulink.BusElement;
elems(6).Name = 'voc_min';

elems(7) = Simulink.BusElement;
elems(7).Name = 'I_max';

elems(8) = Simulink.BusElement;
elems(8).Name = 'soc_points';
elems(8).Dimensions = 21;  % or however many points you use

elems(9) = Simulink.BusElement;
elems(9).Name = 'voc_points';
elems(9).Dimensions = 21;

% Create and assign the bus object
parametersBus = Simulink.Bus;
parametersBus.Elements = elems;

% Push to base workspace
assignin('base','parametersBus',parametersBus);



%% 3) Drive‑Cycle Definition



    switch dc
        case 1  % Three hours constant 5 km/h, 2% gradient
            % Parameters
            a = 0.3;                     % [m/s^2] accel (unused for constant speed)
            v_const = 5 * 1000/3600;     % [m/s]
            duration_h = 3;              % [h]
            duration_s = duration_h * 3600;  % [s]

            % Time vector: sampled at 10 Hz
            numPts = duration_s * 10 + 1;
            time = linspace(0, duration_s, numPts)';  % [s]

            % Speed vector: constant speed, converted to km/h
            speed = repmat(v_const * 3.6, numPts, 1);

            % Create timeseries
            ts_speed = timeseries(speed, time);
            assignin('base', 'ts_speed', ts_speed);

            % Assign parameters
            assignin('base', 'grade', 0.02);          % [%]
            assignin('base', 'friction_coeff', 0.004);
            assignin('base', 'm_trail', 400);

            stopTime = time(end);

      case 2  % 0–15 km/h piecewise acceleration
    % Breakpoints (km/h → m/s)
    v_kmh = [0, 5, 10, 15];
    v_ms  = v_kmh * 1000/3600;

    % Corresponding accel rates (m/s^2)
    a_ms2 = [0.40, 0.25, 0.15];  

    % Preallocate time & speed arrays
    time = [];
    speed = [];

    % Loop over each segment
    t_cum = 0;
    for seg = 1:length(a_ms2)
        v_start = v_ms(seg);
        v_end   = v_ms(seg+1);
        a       = a_ms2(seg);

        % Duration of this segment
        t_seg = (v_end - v_start) / a;  % seconds

        % Build time vector at 10 Hz sampling (0.1 s steps)
        nPts = ceil(t_seg*10) + 1;
        t_local = linspace(0, t_seg, nPts)';

        % Build speed vector (m/s) then convert to km/h
        v_local = (v_start + a * t_local) * 3.6;  % km/h

        % Offset local time by cumulative time and append
        time  = [time;  t_cum + t_local];
        speed = [speed; v_local];

        % Update cumulative time
        t_cum = t_cum + t_seg;
    end

    % Report total accel time
    fprintf('Time to accelerate 0→15 km/h in piecewise ramps: %.2f s\n', time(end));

    % Create timeseries and assign to base
    ts_speed           = timeseries(speed, time);
    assignin('base', 'ts_speed', ts_speed);
    assignin('base', 'grade', 0.02);
    assignin('base', 'friction_coeff', 0.004);
    assignin('base', 'm_trail', 1800);

    stopTime = time(end);


        case 3  % 0-11 km/h acceleration
            a      = 0.3;                           % [m/s^2]
            v_end  = 11 * 1000/3600;               % [m/s]
            t_acc  = v_end / a;                    % [s]

            % Time and speed vectors
            numPts = ceil(t_acc * 10) + 1;
            time   = linspace(0, t_acc, numPts)';   % [s]
            speed  = (a * time) * 3.6;             % [km/h]

            % Report acceleration time
            fprintf('Time to accelerate 0 to 11 km/h: %.2f seconds\n', t_acc);

            ts_speed = timeseries(speed, time);
            assignin('base', 'ts_speed', ts_speed);
            assignin('base', 'grade', 0.02);
            assignin('base', 'friction_coeff', 0.004);
            assignin('base', 'm_trail', 1800);

            stopTime = time(end);

        case 4  % Three hours at peaks each hour
            a      = [0.40,0.25,0.1];                     % [m/s^2] accel
            v1     = 5 * 1000/3600;           % [m/s]
            v2     = 10 * 1000/3600;          % [m/s]
            v3     = 15 * 1000/3600;          % [m/s]
            hour_s = 3600;                    % seconds in an hour

            % First hour: accel 0->5 at start, then hold
            t_acc1 = v1 / a(1);
            t1 = linspace(0, t_acc1, ceil(t_acc1*10))';
            t1_hold = (t1(end)+0.1 : 1/10 : hour_s)';
            v1_vec = [ (a(1) * t1)*3.6; repmat(v1*3.6, numel(t1_hold),1)];
            time1  = [t1; t1_hold];

            % Second hour: hold at 5, accel in middle to 10
            t2_start = hour_s;
            t2_mid   = t2_start + hour_s/2;
            t2_acc_start = t2_mid;
            t_acc2 = (v2 - v1) / a(2);
            t2_acc = linspace(t2_acc_start, t2_acc_start + t_acc2, ceil(t_acc2*10))';
            t2_pre = linspace(t2_start+0.1, t2_acc_start, ceil((t2_acc_start - t2_start)*10))';
            t2_post = linspace(t2_acc(end), hour_s*2, ceil((hour_s*2 - t2_acc(end))*10))';
            v2_vec = [ repmat(v1*3.6, numel(t2_pre),1);
                       ((v1 + a(2)*(t2_acc - t2_acc_start))*3.6);
                       repmat(v2*3.6, numel(t2_post),1)];
            time2 = [t2_pre; t2_acc; t2_post];

            % Third hour: hold at 10, accel in middle to 15
            t3_start = hour_s*2;
            t3_mid   = t3_start + hour_s/2;
            t3_acc_start = t3_mid;
            t_acc3 = (v3 - v2) / a(3);
            t3_acc = linspace(t3_acc_start, t3_acc_start + t_acc3, ceil(t_acc3*10))';
            t3_pre = linspace(t3_start+0.1, t3_acc_start, ceil((t3_acc_start - t3_start)*10))';
            t3_post = linspace(t3_acc(end), hour_s*3, ceil((hour_s*3 - t3_acc(end))*10))';
            v3_vec = [ repmat(v2*3.6, numel(t3_pre),1);
                       ((v2 + a(3)*(t3_acc - t3_acc_start))*3.6);
                       repmat(v3*3.6, numel(t3_post),1)];
            time3 = [t3_pre; t3_acc; t3_post];

            % Concatenate
            time = [time1; time2; time3];
            speed = [v1_vec; v2_vec; v3_vec];

            ts_speed = timeseries(speed, time);
            assignin('base', 'ts_speed', ts_speed);
            assignin('base', 'grade', 0.02);
            assignin('base', 'friction_coeff', 0.004);
            assignin('base', 'm_trail', 400);

            stopTime = time(end);

        otherwise
            error('Drive-cycle %d not defined.', dc);
    end



%% 4) Plot drive cycle
figure;
plot(time, speed, 'LineWidth',1.5);
xlabel('Time [s]'); ylabel('Speed [km/h]');
title('Drive Cycle'); grid on;

%% 5) Run the Simulink model
simOut = sim(modelName, ...
    'StopTime',   num2str(stopTime), ...
    'FixedStep',  num2str(StepSize), ...
    'SaveOutput', 'on', ...
    'SaveFormat', 'Dataset');

%% 6) Extract logged signals directly from simOut

T_struct     = simOut.T_mech;    
omega_struct = simOut.omega_el;
P_struct     = simOut.P_elec;

volTS        = simOut.voltage;
curTS        = simOut.current;
socTS        = simOut.soc;
crateTS      = simOut.c_rate;
PdrawnTS     = simOut.P_drawn_out;

time_sim  = T_struct.time;                    % [s]
T_sim     = T_struct.signals.values;          % [Nm]
omega_sim = omega_struct.signals.values;      % [rad/s]
P_elec    = P_struct.signals.values;          % [W]

tV   = volTS.time;    yV   = volTS.signals.values;
tI   = curTS.time;    yI   = curTS.signals.values;
tSOC = socTS.time;    ySOC = socTS.signals.values;
tC   = crateTS.time;  yC   = crateTS.signals.values;

tPdrawn = PdrawnTS.time;    yPdrawn = PdrawnTS.signals.values;

%% 7) Compute & plot efficiency
P_mech = T_sim .* omega_sim;
eff    = P_mech ./ P_elec; eff(P_elec<1e-6) = 0;
%%

figure;
plot(time_sim, eff, 'LineWidth',1.5);
xlabel('Time [s]'); ylabel('Efficiency \eta');
title('Motor Efficiency'); grid on; ylim([0 1]);

%% 8) Plot torque‑speed map vs. trajectory

%Calculating adhesion quantities

F_adh    = mu * m_loco * g*cos(grade);                  % [N]
T_adh    = F_adh * rw / (G * mech_eff); 
T_adh    = T_adh/2; % number of motors

%Plotting
rpm_axis = linspace(min(speed_map_rpm), max(speed_map_rpm), 200);


figure; hold on; grid on;
plot(speed_map_rpm, torque_map_Nm, 'k--o','LineWidth',1.5);
plot(omega_sim*60/(2*pi), T_sim, '.','MarkerSize',8);
plot(rpm_axis, T_adh * ones(size(rpm_axis)), 'r-', 'LineWidth',2);
xlabel('Speed [RPM]'); ylabel('Torque [Nm]');
title('Torque‑Speed Map'); 
legend('Static','Trajectory','Adhesion Limit','Location','best');


%% 9) Plot battery signals
figure;
subplot(5,1,1);
plot(tV,  yV,  'LineWidth',1.2);
ylabel('Voltage [V]'); title('Battery Voltage'); grid on;

subplot(5,1,2);
plot(tI,  yI,  'LineWidth',1.2);
ylabel('Current [A]'); title('Battery Current'); grid on;

subplot(5,1,3);
plot(tSOC,ySOC,'LineWidth',1.2);
ylabel('SOC [%]'); title('State of Charge'); grid on;

subplot(5,1,4);
plot(tC,  yC,  'LineWidth',1.2);
xlabel('Time [s]'); ylabel('C‑rate'); title('Charge/Discharge C‑rate'); grid on;

subplot(5,1,5);
plot(tC,  yC,  'LineWidth',1.2);
xlabel('Time [s]'); ylabel('C‑rate'); title('Charge/Discharge C‑rate'); grid on;

%% 10) Plot all power signals for the two motors
figure;
plot(time_sim, 2*P_elec/0.95,      'b-',  'LineWidth',1.5); hold on;
plot(tPdrawn,  yPdrawn,     'r--', 'LineWidth',1.5);
plot(time_sim, 2*T_sim .* omega_sim, 'g-.', 'LineWidth',1.5);  % P_mech

xlabel('Time [s]');
ylabel('Power [W]');
title('Power Signals Over Time for the two motors');
legend('P_{elec} (Input to Battery)', ...
       'P_{drawn} (Output of Battery)', ...
       'P_{mech} (Motor Output)', ...
       'Location','best');
grid on;