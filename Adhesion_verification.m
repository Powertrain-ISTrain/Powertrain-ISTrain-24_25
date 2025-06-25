%% Parameters

% Fixed Parameters

max_velocity=4.17; %m/s
adhesion_coeff= 0.15:0.05:0.6; % Lower and Upper bounds of the friction coefficient betwenn the wheels and rails
max_trailing_mass=1800; %kg, specified in technical specifications
Rolling_coeff=0.004;%Rolling Ressístance coeff
g=9.81; %m/s^2
Aerodynamic_drag=5.1; % N
max_gradient=2; % 

%Variable Parameters

G=2;
loco_mass=400 ; %KG
driven_wheels=8; % 
nr_motors=2;% one motor per bogie
transmission_eff=0.85;%Average value
wheel_radius=0.100; %m - Minimum wheel diameter is 200 mm

%% force calculation

alpha = atan(max_gradient / 100); % gradient angle in radians

% Calculating the maximum values
M_wheel = (loco_mass * g)/driven_wheels;
f_wheel = adhesion_coeff .* M_wheel * cos(alpha);
f_traction = driven_wheels .* f_wheel;

f_drag = Rolling_coeff * (loco_mass) * g * cos(alpha) + Aerodynamic_drag; % Total resistive force
f_motive = f_traction - f_drag;

%% acceleration calculation
max_accel_loaded=f_motive/(loco_mass+max_trailing_mass);
max_accel_unloaded=f_motive/(loco_mass);
time_15kmh=max_velocity./max_accel_loaded;



%% Torque and Power Calculation

motor_torque=2*(f_wheel.*wheel_radius)./0.5./(transmission_eff.*G);


max_motor_speed=max_velocity.*60.*G./(pi*wheel_radius*2); %in rpm's

max_motor_power=(max_motor_speed).*motor_torque*((2*pi)/60);



%% Plots

%forces

plot(adhesion_coeff, f_wheel, 'r-', 'LineWidth', 2)
hold on;
plot(adhesion_coeff, f_traction, 'b--', 'LineWidth', 2)
plot(adhesion_coeff, f_motive, 'g-.', 'LineWidth', 2)
hold off;

legend('Wheel force', 'Traction force', 'Motive force')
xlabel('Adhesion coefficient')
ylabel('Force (N)')
title('Forces vs Adhesion')
grid on;


% accelerations

figure;

subplot(2,2,1);
plot(adhesion_coeff, max_accel_loaded);
title('max acceleration loaded (m/s^2)');

subplot(2,2,2);
plot(adhesion_coeff,max_accel_unloaded);
title('max  acceleration unloaded (m/s^2)');

subplot(2,2,3);
plot(adhesion_coeff, time_15kmh);
title('time to get to 15km/h (s)');

grid on;


% Motor

figure;

subplot(2,2,1);
plot(adhesion_coeff, motor_torque);
title('Motor Torque vs adhesion coefficient');

subplot(2,2,2);
plot(adhesion_coeff,max_motor_power);
title('Motor Power vs adhesion coefficient');

grid on;





