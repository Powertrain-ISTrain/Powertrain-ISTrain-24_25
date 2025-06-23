


driveCycles = DriveCycles();
for k = 1:numel(driveCycles)
  currentCycle = timeseries(driveCycles(k).speed, driveCycles(k).time);
  % …set up simIn, run sim, extract metrics…
end


%% Batch‑Simulation Script for Drive‑Cycle Scenarios

% 0) Set up
modelName = 'yourModelName';    % <-- change to your Simulink model
useParallel = true;             % set false if you don't have Parallel Toolbox

%% 1) Define your scenarios
% Example: two scenarios using CSV + MAT files
scenarios = struct( ...
  'name',           {'Urban',         'Highway'}, ...
  'trackFile',      {'trackUrban.txt','trackHighway.txt'}, ...
  'speedMatFile',   {'speedUrban.mat','speedHighway.mat'}, ...
  'extraWeight',    {0,               100}, ...      % kg
  'roadAngleDeg',   {0,               3} ...        % degrees
);

%% 2) Load or generate your drive‑cycle data
% (This block runs once, before the sim loop)
for k = 1:numel(scenarios)
  sc = scenarios(k);
  
  % — If you have time/speed in CSV:
  % raw = readmatrix(sc.trackFile);    % [time, speed] columns
  % scenarios(k).time  = raw(:,1);
  % scenarios(k).speed = raw(:,2);
  
  % — Or if you saved a .mat with variables t and v:
  tmp = load(sc.speedMatFile);        % assumes 't' and 'v' inside
  scenarios(k).time  = tmp.t(:);
  scenarios(k).speed = tmp.v(:);
end

%% 3) Build SimulationInput array
nScen = numel(scenarios);
simIn(nScen) = Simulink.SimulationInput(modelName);  % pre-allocate

for k = 1:nScen
  sc = scenarios(k);
  
  % Create a timeseries for this cycle
  currentCycle = timeseries(sc.speed, sc.time);
  
  % Assign variables into model workspace
  simIn(k) = simIn(k)                   ...
    .setVariable('currentCycle',currentCycle)               ...
    .setVariable('extraWeight',sc.extraWeight)               ...
    .setVariable('roadAngleRad',sc.roadAngleDeg*pi/180)      ...
    .setModelParameter('StopTime',sprintf('%g', max(sc.time)));
end

%% 4) Run all simulations
if useParallel
  simOut = simBatch(simIn,'UseParallel','on');
else
  simOut = simBatch(simIn,'UseParallel','off');
end

%% 5) Extract metrics
results(nScen) = struct();  % pre-allocate
for k = 1:nScen
  out = simOut(k);
  logs = out.logsout;
  
  % Example: peak motor torque
  Tm = logs.get('MotorTorque').Values.Data;  
  results(k).peakTorque = max(abs(Tm));
  
  % Example: total battery energy in (Wh)
  Pbat = logs.get('BatteryPower').Values.Data;
  t    = logs.get('BatteryPower').Values.Time;
  results(k).energyIn_Wh = trapz(t, Pbat)/3600;
  
  % Example: SOC swing
  soc = logs.get('SoC').Values.Data;
  results(k).socSwing = soc(end) - soc(1);
  
  % Add scenario name
  results(k).name = scenarios(k).name;
end

%% 6) Display summary
T = struct2table(results);
disp(T);
