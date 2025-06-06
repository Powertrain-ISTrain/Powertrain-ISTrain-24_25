%Main script for the performance calculations

%{
This script has as inputs some fixed parameters (as well as some variable
parameters) and as output the Torque(Max), rpm's(Max) and Power(Max) of
the motor. These are the values for the most 
critical scenario (maximum velocity at maximum gradient)
%}

%Note: Fixed parameters are based on the Technical Specifications 2025 by
%IMech

%% Parameters

% Fixed Parameters

max_velocity=4.17; %m/s
wheel_radius=0.100; %m - Minimum wheel diameter is 200 mm
adhesion_coeff= 0.15:0.05:0.6; % Lower and Upper bounds of the friction coefficient betwenn the wheels and rails
loco_mass=2000 ; % kg , estimate
max_trailing_mass=1800; %kg, specified in technical specifications
driven_wheels=8;
nr_motors=2;% one motor per bogie
Rolling_coeff=0.004;%Rolling Ressístance coeff
g=9.81; %m/s^2
transmission_eff=0.85;%Average value
Aerodynamic_drag=5.1; % N
max_gradient=2; % 

%Variable Parameters

G=5; %Gear Ratio between the axle and the motor  
gear_ratio=1/5; % gear ratio (w_motor/w_axle)
%%
max_velocity=4.17; %m/s
wheel_radius=0.125; %m - Minimum wheel diameter is 200 mm
adhesion_coeff= 0.15:0.05:0.6; % Lower and Upper bounds of the friction coefficient betwenn the wheels and rails
loco_mass=450 ; % kg , estimate
max_trailing_mass=600; %kg, specified in technical specifications
driven_wheels=8;
nr_motors=2;% one motor per bogie
Rolling_coeff=0.004;%Rolling Ressístance coeff
g=9.81; %m/s^2
transmission_eff=0.85;%Average value
Aerodynamic_drag=5.1; % N
max_gradient=2; % 

%Variable Parameters

G=5; %Gear Ratio between the axle and the motor  
gear_ratio=1/5; % gear ratio (w_motor/w_axle)




%% calculating the forces
[f_motive, f_wheel] = forces(max_gradient, loco_mass,max_trailing_mass, g, driven_wheels, ...
    adhesion_coeff, Rolling_coeff, Aerodynamic_drag);
%% calculating the accelerations
[max_accel_loaded,max_accel_unloaded,time_15kmh] = accelerations(f_motive,adhesion_coeff,loco_mass,max_trailing_mass, max_velocity);
%% calculating the Maximum torque , speed and Power

[max_motor_torque,max_motor_speed,max_motor_power] = MotorParameters(f_motive,wheel_radius,transmission_eff,G,adhesion_coeff,max_velocity);