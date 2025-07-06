%% MOTOR PARAMETERS

%Description: This script contains the motor parameters and  torque-speed
%curves. This scipt is supposed to be ran alongside the "ISTrain2025.slx"
%file

%The user must run the section of the motor he wants to simulate

% Motor electrical/mechanical constants
R_par           = Simulink.Parameter(0.5);        % Ω
Kt_par          = Simulink.Parameter(0.1);        % N·m/A
Ke_par          = Simulink.Parameter(0.1);        % V/(rad/s)
B_par           = Simulink.Parameter(1e-3);       % N·m/(rad/s)

% Measured torque–speed envelope
speedCurve_par  = Simulink.Parameter([0 1000 2000 3000]); 
torqueCurve_par = Simulink.Parameter([4   3    2    0.5]); 



