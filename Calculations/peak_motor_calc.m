%% A. Peak Tractive‑Effort & Motor Torque Sizing (no aero drag, split efficiencies)
% Given parameters
v_max    = 4.17;        % m/s
r_w      = 0.100;       % m
m_loco   = 450;         % kg
m_trail  = 1800;        % kg
Crr      = 0.004;       % rolling resistance
g        = 9.81;        % m/s^2
grade    = 0.02;        % 2% incline
G        = 5;           % gearbox ratio
eta_mech = 0.90;        % mech efficiency
eta_elec = 0.85;        % elec efficiency
n_mot    = 2;           % number of motors
a=0.5;

%% 1) Total mass
m_total = m_loco + m_trail;

%% 2) Resistance forces at stand‑start (no aero) and at v_max
F_grade = m_total * g * grade;  % grade
F_rr    = m_total * g * Crr;% rolling
F_a=m_total*a; % Force due to a imposed acceleration
F_req   = F_grade + F_rr+F_a;       % total traction force

%% 3) Wheel torque required at start
T_wheel = F_req * r_w;        

%% 4) Motor torque (per motor) for start
T_motor = (T_wheel / n_mot) / (G * eta_mech);
T_motor = 1.15 * T_motor;       % +15% margin

%% 5) Motor speed at v_max
omega_wheel = v_max / r_w;           
omega_motor = G * omega_wheel;       
rpm_motor   = omega_motor * 60/(2*pi);

%% 6) Mechanical power at wheels & apply safety margin
P_mech_wheel = F_req * v_max;        % P = F * v
P_mech_wheel = 1.15 * P_mech_wheel;  % +15% margin

%% 7) Mechanical power at motor shaft (total & per‑motor)
P_mech_motor_total = P_mech_wheel / eta_mech;
P_mech_motor_per   = P_mech_motor_total / n_mot;

%% 8) Electrical power draw (total & per‑motor)
P_elec_total = P_mech_motor_total / eta_elec;
P_elec_per   = P_elec_total / n_mot;

%% 9) Display results
fprintf('--- Peak Sizing ---\n');
fprintf('Required force (no aero): %.1f N\n', F_req);
fprintf('Wheel torque @ start: %.2f Nm\n', T_wheel);
fprintf('Motor torque (per motor): %.2f Nm\n', T_motor);
fprintf('Motor speed @ v_max: %.0f rpm\n', rpm_motor);
fprintf('Mechanical power @ wheels (with margin): %.1f W\n', P_mech_wheel);
fprintf('Mechanical power @ motor shaft: %.1f W total, %.1f W/motor\n', ...
        P_mech_motor_total, P_mech_motor_per);
fprintf('Electrical power draw: %.1f W total, %.1f W/motor\n\n', ...
        P_elec_total, P_elec_per);