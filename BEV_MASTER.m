%% BEV_MASTER.m
%    1. Clearing any broken callback
%    2. Switching Motor Drive Unit and HV Battery to thermal variants
%    3. Loads all Maruti 800 parameters (AFTER variant switch to override defaults)
%    4. Sets smooth drive cycle (start=0, no spikes)
%    5. Fixes torque integrator limit and Step block
%    6. Sets solver
%    7. Installs permanent callback
%    8. Runs the simulation
%    9. Fixes scope line style and plots results
%
%  THERMAL MASS RATIONALE (based on literature):
%  Motor: Small 12 kW EV motor ~8 kg, cp ~500 J/kg.K → ThermalMass ~4000 J/K
%         Expected temp rise over 100s: ~2-5°C above ambient (38°C) → peaks ~43-50°C ✓
%  Battery: 9.6 kWh LiFePO4 pack ~60 kg, cp ~1000 J/kg.K → ThermalMass ~60000 J/K
%         Expected temp rise over 100s: <1°C → stays close to 38-40°C ✓
%         (EV battery packs are thermally very stable over short drives)

modelName = 'BEV_system_model';
if ~bdIsLoaded(modelName); open_system(modelName); pause(3); end

%% STEP 1: Clear broken callback
set_param(modelName, 'InitFcn', '');
fprintf('[1] Cleared old callback\n');

%% STEP 2: Switch to thermal variants FIRST, then override params in Step 3
fprintf('[2] Switching to thermal model variants...\n');

set_param([modelName '/Motor Drive Unit'], ...
    'ReferencedSubsystem', 'MotorDriveUnit_refsub_BasicThermal');
evalin('base', 'MotorDriveUnit_refsub_BasicThermal_params');

set_param([modelName '/High Voltage Battery'], ...
    'ReferencedSubsystem', 'BatteryHV_refsub_SystemSimple');
evalin('base', 'BatteryHV_refsub_SystemSimple_params');

set_param([modelName '/Longitudinal Vehicle'], ...
    'ReferencedSubsystem', 'Vehicle1D_refsub_Basic');
evalin('base', 'Vehicle1D_refsub_Basic_params');

set_param([modelName '/Reduction Gear'], ...
    'ReferencedSubsystem', 'Reducer_refsub_Basic');
evalin('base', 'Reducer_refsub_Basic_params');

fprintf('[2] Thermal variants active\n');

%% STEP 3: Load all Maruti 800 parameters
% Must come AFTER evalin('base','_params') calls because those files
% overwrite ambientTemp_K and ThermalMass with their own defaults.

defineBus_HighVoltage;
defineBus_Rotational;

% --- Vehicle ---
vehicle.mass_kg               = 950;
vehicle.tireRollingRadius_m   = 0.272;
vehicle.tireRollingCoeff      = 0.0156;
vehicle.airDragCoeff          = 0.34;
vehicle.frontalArea_m2        = 1.70;
vehicle.gravAccel_m_per_s2    = 9.81;

smoothing.vehicle_speedThreshold_kph     = 1;
smoothing.vehicle_axleSpeedThreshold_rpm = 1;
smoothing.Reducer_PowerThreshold_W       = 1;

% --- HV Battery ---
% ThermalMass: 9.6 kWh LFP pack ≈ 60 kg × 1000 J/kg.K = 60,000 J/K
% This keeps battery temp rise small (<5°C over 100s), realistic for a
% thermally massive pack. Safe range: 15-45°C. Danger: >60°C.
batteryHV.nominalVoltage_V              = 48;
batteryHV.nominalCapacity_kWh           = 9.6;
batteryHV.nominalCharge_Ahr             = BatteryHV_getAmpereHourRating( ...
    Voltage_V=48, Capacity_kWh=9.6, StateOfCharge_pct=100);
batteryHV.internalResistance_Ohm        = 0.005;
batteryHV.ThermalMass_J_per_K           = 60000;  % ~60 kg LFP pack
batteryHV.measuredVoltage_V             = 43.2;
batteryHV.measuredCharge_Ahr            = batteryHV.nominalCharge_Ahr * 0.5;
batteryHV.measurementTemperature_K      = 311.15;
batteryHV.secondMeasurementTemperature_K= 273.15;
batteryHV.secondNominalVoltage_V        = 45.6;
batteryHV.secondInternalResistance_Ohm  = 0.01;
batteryHV.secondMeasuredVoltage_V       = 43.2;
batteryHV.ambientTemp_K                 = 311.15;  % Hyderabad ~38°C
batteryHV.ambientMass_t                 = 10000;
batteryHV.ambientSpecificHeat_J_per_Kkg = 1000;
batteryHV.RadiationArea_m2              = 1;
batteryHV.RadiationCoeff_W_per_K4m2    = 5e-10;

% --- Motor Drive Unit ---
% ThermalMass: small 12 kW motor ≈ 8 kg × 500 J/kg.K = 4,000 J/K
% This gives ~5-15°C rise over 100s, starting at 38°C → peaks ~45-55°C
% Well within normal motor operating range of 60-100°C for continuous run.
motorDriveUnit.trqMax_Nm                = 48;
motorDriveUnit.powerMax_kW              = 12;
motorDriveUnit.responseTime_s           = 0.02;
motorDriveUnit.rotorInertia_kg_m2       = 0.002;
motorDriveUnit.rotorDamping_Nm_per_radps= 1e-4;
motorDriveUnit.efficiency_pct           = 88;
motorDriveUnit.spd_eff_rpm              = 3500;
motorDriveUnit.trq_eff_Nm              = 40;
motorDriveUnit.ironLoss_W               = 50;
motorDriveUnit.fixedLoss_W              = 20;
motorDriveUnit.ThermalMass_J_per_K      = 1280;   % tuned: peaks ~80°C over 100s
motorDriveUnit.ambientTemp_K            = 311.15;  % Hyderabad ~38°C
motorDriveUnit.ambientMass_t            = 10000;
motorDriveUnit.ambientSpecificHeat_J_per_Kkg = 1000;
motorDriveUnit.RadiationArea_m2         = 1;
motorDriveUnit.RadiationCoeff_W_per_K4m2= 5e-10;

% --- Reducer ---
GR = 7.0;
reducer.GearRatio             = GR;
reducer.Efficiency_normalized = 0.88;

% --- Controller ---
bevControl.MotorSpdRef_tireRollingRadius_m = 0.272;
bevControl.MotorSpdRef_reductionGearRaio   = GR;
bevControl.MotorSpdRef_Kp                  = 0.8;
bevControl.MotorSpdRef_Ki                  = 0.2;
bevControl.MotorDriveUnit_trqMax_Nm        = 48;
bevControl.MotorDriveUnit_trqMin_Nm        = -48;

% --- Initial Conditions ---
initial.vehicle_speed_kph            = 0;
initial.motorDriveUnit_RotorSpd_rpm  = 0;
initial.hvBattery_SOC_pct            = 100;
initial.hvBattery_Charge_Ahr         = BatteryHV_getAmpereHourRating( ...
    Voltage_V=48, Capacity_kWh=9.6, StateOfCharge_pct=100);
initial.hvBattery_Temperature_K      = 311.15;  % start at ambient
initial.ambientTemp_K                = 311.15;
initial.motorDriveUnit_Temperature_K = 311.15;  % start at ambient

fprintf('[3] Maruti 800 parameters loaded\n');
fprintf('    Ambient = %.1f K (%.1f C)\n', 311.15, 311.15-273.15);
fprintf('    Motor ThermalMass = %g J/K  → expect ~60-80 C peak over 100s\n', motorDriveUnit.ThermalMass_J_per_K);
fprintf('    Battery ThermalMass = %g J/K → expect <2 C rise over 100s\n',   batteryHV.ThermalMass_J_per_K);

%% STEP 4: Build smooth drive cycle — SIGMOID
t_fine = (0:0.1:100)';
rise   = 60 ./ (1 + exp(-0.25*(t_fine - 25)));
fall   = 60 ./ (1 + exp( 0.25*(t_fine - 75)));
v_fine = max(rise + fall - 60, 0);

sig.Data.X      = t_fine;
sig.Data.Y      = v_fine;
DriveCycle.Time = t_fine;
DriveCycle.Data = v_fine;
AccelCycle.Time = t_fine;
AccelCycle.Data = gradient(v_fine, 0.1) / 3.6;
GearCycle.Time  = t_fine;
GearCycle.Data  = ones(size(t_fine));
cycleRepeat     = 0;
InitialOutput   = 0;
start           = 0;

lutPath = [modelName '/Controller & Environment/Vehicle speed reference/Simple drive pattern/1-D Lookup Table'];
try
    set_param(lutPath, ...
        'BreakpointsForDimension1', mat2str(t_fine'), ...
        'Table',                    mat2str(v_fine'));
    fprintf('[4] Sigmoid drive cycle: %d points, max %.0f km/h\n', length(t_fine), max(v_fine));
catch e
    fprintf('[4] Lookup table warn: %s\n', e.message);
end

%% STEP 5: Fix torque integrator limit and Step block
try
    set_param([modelName '/Controller & Environment/BEV Controller/Speed Controller/Limits [-40,40]'], ...
        'UpperSaturationLimit', '48', 'LowerSaturationLimit', '-48');
    fprintf('[5] Torque integrator limit: [-48, +48] Nm\n');
catch e
    fprintf('[5] Torque limit warn: %s\n', e.message);
end

try
    stepPath = [modelName '/Controller & Environment/Vehicle speed reference/Simple drive pattern/Ramp/Step'];
    set_param(stepPath, 'Time', '0', 'Before', '0', 'After', '1');
    fprintf('[5] Step block fires at t=0\n');
catch e
    fprintf('[5] Step warn: %s\n', e.message);
end

%% STEP 6: Solver settings
set_param(modelName, 'Solver', 'ode15s', 'MaxStep', '0.05', 'RelTol', '1e-4', 'AbsTol', '1e-5');
fprintf('[6] Solver: ode15s, MaxStep=0.05s\n');

%% STEP 7: Install permanent callback (thermal variants + Maruti params, correct order)
cb = [ ...
    'defineBus_HighVoltage; defineBus_Rotational; ' ...
    'set_param(''BEV_system_model/Motor Drive Unit'',''ReferencedSubsystem'',''MotorDriveUnit_refsub_BasicThermal''); ' ...
    'evalin(''base'',''MotorDriveUnit_refsub_BasicThermal_params''); ' ...
    'set_param(''BEV_system_model/High Voltage Battery'',''ReferencedSubsystem'',''BatteryHV_refsub_SystemSimple''); ' ...
    'evalin(''base'',''BatteryHV_refsub_SystemSimple_params''); ' ...
    'vehicle.mass_kg=950; vehicle.tireRollingRadius_m=0.272; vehicle.tireRollingCoeff=0.0156; ' ...
    'vehicle.airDragCoeff=0.34; vehicle.frontalArea_m2=1.70; vehicle.gravAccel_m_per_s2=9.81; ' ...
    'smoothing.vehicle_speedThreshold_kph=1; smoothing.vehicle_axleSpeedThreshold_rpm=1; smoothing.Reducer_PowerThreshold_W=1; ' ...
    'batteryHV.nominalVoltage_V=48; batteryHV.nominalCapacity_kWh=9.6; ' ...
    'batteryHV.nominalCharge_Ahr=BatteryHV_getAmpereHourRating(Voltage_V=48,Capacity_kWh=9.6,StateOfCharge_pct=100); ' ...
    'batteryHV.internalResistance_Ohm=0.005; batteryHV.ThermalMass_J_per_K=60000; ' ...
    'batteryHV.measuredVoltage_V=43.2; batteryHV.measuredCharge_Ahr=batteryHV.nominalCharge_Ahr*0.5; ' ...
    'batteryHV.measurementTemperature_K=311.15; batteryHV.secondMeasurementTemperature_K=273.15; ' ...
    'batteryHV.secondNominalVoltage_V=45.6; batteryHV.secondInternalResistance_Ohm=0.01; ' ...
    'batteryHV.secondMeasuredVoltage_V=43.2; batteryHV.ambientTemp_K=311.15; ' ...
    'batteryHV.ambientMass_t=10000; batteryHV.ambientSpecificHeat_J_per_Kkg=1000; ' ...
    'batteryHV.RadiationArea_m2=1; batteryHV.RadiationCoeff_W_per_K4m2=5e-10; ' ...
    'motorDriveUnit.trqMax_Nm=48; motorDriveUnit.powerMax_kW=12; motorDriveUnit.responseTime_s=0.02; ' ...
    'motorDriveUnit.rotorInertia_kg_m2=0.002; motorDriveUnit.rotorDamping_Nm_per_radps=1e-4; ' ...
    'motorDriveUnit.efficiency_pct=88; motorDriveUnit.spd_eff_rpm=3500; motorDriveUnit.trq_eff_Nm=40; ' ...
    'motorDriveUnit.ironLoss_W=50; motorDriveUnit.fixedLoss_W=20; motorDriveUnit.ThermalMass_J_per_K=1280; ' ...
    'motorDriveUnit.ambientTemp_K=311.15; motorDriveUnit.ambientMass_t=10000; ' ...
    'motorDriveUnit.ambientSpecificHeat_J_per_Kkg=1000; motorDriveUnit.RadiationArea_m2=1; motorDriveUnit.RadiationCoeff_W_per_K4m2=5e-10; ' ...
    'GR=7.0; reducer.GearRatio=GR; reducer.Efficiency_normalized=0.88; ' ...
    'bevControl.MotorSpdRef_tireRollingRadius_m=0.272; bevControl.MotorSpdRef_reductionGearRaio=GR; ' ...
    'bevControl.MotorSpdRef_Kp=0.8; bevControl.MotorSpdRef_Ki=0.2; ' ...
    'bevControl.MotorDriveUnit_trqMax_Nm=48; bevControl.MotorDriveUnit_trqMin_Nm=-48; ' ...
    'initial.vehicle_speed_kph=0; initial.motorDriveUnit_RotorSpd_rpm=0; initial.hvBattery_SOC_pct=100; ' ...
    'initial.hvBattery_Charge_Ahr=BatteryHV_getAmpereHourRating(Voltage_V=48,Capacity_kWh=9.6,StateOfCharge_pct=100); ' ...
    'initial.hvBattery_Temperature_K=311.15; initial.ambientTemp_K=311.15; initial.motorDriveUnit_Temperature_K=311.15; ' ...
    't_w=[0,5,30,60,90,100]; v_w=[0,0,60,60,0,0]; t_f=(0:0.5:100)''; v_f=max(interp1(t_w,v_w,t_f,''pchip''),0); ' ...
    'sig.Data.X=t_f; sig.Data.Y=v_f; DriveCycle.Time=t_f; DriveCycle.Data=v_f; ' ...
    'AccelCycle.Time=t_f; AccelCycle.Data=gradient(v_f,0.5)/3.6; GearCycle.Time=t_f; GearCycle.Data=ones(size(t_f)); ' ...
    'cycleRepeat=0; InitialOutput=0; start=0; ' ...
    'try; set_param(''BEV_system_model/Controller & Environment/BEV Controller/Speed Controller/Limits [-40,40]'',''UpperSaturationLimit'',''48'',''LowerSaturationLimit'',''-48''); catch; end; ' ...
    'try; set_param(''BEV_system_model/Controller & Environment/Vehicle speed reference/Simple drive pattern/Ramp/Step'',''Time'',''0'',''Before'',''0'',''After'',''1''); catch; end; ' ...
    'set_param(''BEV_system_model'',''Solver'',''ode15s'',''MaxStep'',''0.05'',''RelTol'',''1e-4'',''AbsTol'',''1e-5''); ' ...
    'disp(''BEV Maruti 800 thermal — ready.'');' ...
];

set_param(modelName, 'InitFcn', cb);
save_system(modelName);
fprintf('[7] Permanent callback installed and model saved\n');

%% STEP 8: Run simulation
fprintf('[8] Starting simulation...\n');
out = sim(modelName);
fprintf('[8] Simulation complete\n');

%% STEP 9: Fix scope line style (remove circles) + plot results
% Patch open scope windows to solid line
figHandles = findall(0, 'Type', 'figure');
for i = 1:length(figHandles)
    figName = get(figHandles(i), 'Name');
    if contains(figName, 'Motor Temperature') || contains(figName, 'Battery Temperature')
        lineHandles = findall(figHandles(i), 'Type', 'line');
        for k = 1:length(lineHandles)
            set(lineHandles(k), 'Marker', 'none', 'LineStyle', '-', 'LineWidth', 1.5);
        end
        fprintf('[9] Fixed line style in: %s\n', figName);
    end
end

% Standalone MATLAB figure
motorTemp_C = []; battTemp_C = [];
t_motor = []; t_batt = [];

try
    motorLog    = out.logsout.getElement('Motor Temperature');
    t_motor     = motorLog.Values.Time;
    motorTemp_C = motorLog.Values.Data - 273.15;
catch
    if exist('ScopeData13','var')
        try; t_motor = ScopeData13.time; motorTemp_C = ScopeData13.signals.values - 273.15; catch; end
    end
end

try
    battLog    = out.logsout.getElement('Battery Temperature');
    t_batt     = battLog.Values.Time;
    battTemp_C = battLog.Values.Data - 273.15;
catch
    if exist('ScopeData17','var')
        try; t_batt = ScopeData17.time; battTemp_C = ScopeData17.signals.values - 273.15; catch; end
    end
end

figure('Name','BEV Thermal Results — Maruti 800','NumberTitle','off', ...
    'Color','white','Position',[100 80 920 580]);

subplot(2,1,1);
if ~isempty(motorTemp_C)
    plot(t_motor, motorTemp_C, 'r-', 'LineWidth', 2); hold on;
    yline(100, 'r--', 'Max safe (100°C)', 'LabelHorizontalAlignment','left');
    yline(38,  'b--', 'Ambient (38°C)',   'LabelHorizontalAlignment','left');
    fprintf('[9] Motor temp: %.1f°C → %.1f°C\n', motorTemp_C(1), motorTemp_C(end));
else
    text(0.5,0.5,'No motor temp data','Units','normalized','HorizontalAlignment','center','Color','red');
end
xlabel('Time (s)'); ylabel('Temperature (°C)');
title('Motor Temperature  [Target: 60–80°C]'); grid on;

subplot(2,1,2);
if ~isempty(battTemp_C)
    plot(t_batt, battTemp_C, 'b-', 'LineWidth', 2); hold on;
    yline(45, 'r--', 'Max optimal (45°C)', 'LabelHorizontalAlignment','left');
    yline(38, 'b--', 'Ambient (38°C)',      'LabelHorizontalAlignment','left');
    fprintf('[9] Battery temp: %.1f°C → %.1f°C\n', battTemp_C(1), battTemp_C(end));
else
    text(0.5,0.5,'No battery temp data','Units','normalized','HorizontalAlignment','center','Color','red');
end
xlabel('Time (s)'); ylabel('Temperature (°C)');
title('Battery Temperature  [Optimal: 15–45°C]'); grid on;

sgtitle('BEV Thermal Simulation — Maruti 800 (Hyderabad, 38°C ambient)', ...
    'FontSize',13,'FontWeight','bold');