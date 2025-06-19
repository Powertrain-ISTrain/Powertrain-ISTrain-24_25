% defineDriveCycles.m
% Returns a struct array `driveCycles` with fields name, time, speed.

function driveCycles = DriveCycles()
  % Scenario 1: Constant Cruise
  t1 = 0:1:60;
  v1 = 15 * ones(size(t1));
  
  % Scenario 2: Ramp–Cruise–Decel
  t2 = [0:1:10, 11:1:50, 51:1:60]';
  v2 = [linspace(0,20,11), 20*ones(1,40), linspace(20,0,10)]';
  
  % Scenario 3: Urban Stop‑Start
  t3 = 0:1:120;  v3 = zeros(size(t3));
  for i=0:2
    idx = t3 >= i*40 & t3 < i*40+10;
    v3(idx) = linspace(0,10,sum(idx));
    idx = t3 >= i*40+10 & t3 < i*40+30;
    v3(idx) = 10;
  end
  
  % Scenario 4: Sinusoidal ("Hilly") Profile
  t4 = 0:0.5:120;  
  v4 = 12 + 6*sin(2*pi*t4/60);
  
  % Pack into struct
  driveCycles = struct( ...
    'name',  {'Constant','RampCruise','Urban','Sinusoidal'}, ...
    'time',  {t1, t2, t3, t4}, ...
    'speed', {v1, v2, v3, v4} ...
  );
end
