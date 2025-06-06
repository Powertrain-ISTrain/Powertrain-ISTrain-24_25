% === Train Parameters ===
m_loco   = 700;      % kg
m_trail  = 1800;     % kg
r_w      = 0.100;    % m
Crr      = 0.004;    
grade    = 0.02;
g        = 9.81;     % m/s^2

% === Drivetrain Parameters ===
G        = 5;
n_mot    = 2;
eta_mech = 0.90;
eta_elec = 0.85;

% === Velocity Targets ===
v_max     = 4.17;        % m/s
v_cruise  = 5 / 3.6;     % m/s
rpm_motor = 1996;        % rpm at v_max (rounded)

% === Battery / Energy Sizing ===
t_run   = 3;             % hours
V_pack  = 72;            % V
E_kWh   = 0.64;          % kWh
I_cont  = 124.7;         % A
Ah_req  = 26.2;          % Ah
C_rate  = 4.76;

% === Derived Mass ===
m_total = m_loco + m_trail;

StepSize = 0.001;
StopTime = 400;
