%% CLEAR ALL AND CLOSE ALL
close all
clear all
clc

%% NOMINAL PARAMETERS
% VEHICLE DYNAMICS PARAMETERS
m = 1430;                       % Vehicle mass [kg]
Af = 2.46;                      % Frontal area [m^2]
Cd = 0.29;                      % Air-drag coefficient [-]
Cr = 1.75;                      % Rolling resistance (tyre type) [-]
c1 = 0.0328;                    % Rolling resistance (surface type) [-]
c2 = 4.575;                     % Rolling resistance (surface condition) [-]
g = 9.81;                       % Gravity [m/s^2]
slope = 0;                      % Road slope (rad)  (MAKE SURE your Simulink model uses radians)
rho_air = 1.2256;               % Air density [kg/m^3]

% (ACC) CONTROLLER PARAMETERS
time_gap            = 1.5;      % ACC time gap [s]
standstill_distance = 1.5;      % ACC default spacing [m]
verr_gain           = 0.5;      % ACC velocity error gain [-]
xerr_gain           = 0.2;      % ACC spacing error gain [-]
vx_gain             = 0.6;      % ACC relative velocity gain [-]
max_ac              = 2;        % Max acceleration [m/s^2]
min_ac              = -3;       % Min acceleration [m/s^2]
v_des = 25;

% INITIAL CONDITIONS
x_ego = 0;                      % Ego initial position [m]
v_ego = 1;                      % Ego initial speed [m/s]
x_leader = 30;                  % Leader initial position [m]
v_leader = 1;                   % Leader initial speed [m/s]
ego_length = 4;                 % Ego length [m]


%% CREATE A MATRIX OF THE INPUT FACTORS (NOW 6 VARIABLES)
m_bound        = [1300 1600];
Cd_bound       = [0.25 0.35];
time_gap_bound = [1.0  2.0];

% NEW BOUNDS
Cr_bound       = [1.2  2.2];       % Rolling resistance factor [-]
slope_bound    = [-0.07 0.07];     % Road slope [rad]  (~ +/-4 deg)
v_des_bound    = [20   30];        % Desired speed [m/s] (72–108 km/h)

n = 200;                            % number of Monte Carlo runs
LHS = lhsdesign(n,6);

m_vector        = m_bound(1)        + (m_bound(2)        - m_bound(1))        .* LHS(:,1);
Cd_vector       = Cd_bound(1)       + (Cd_bound(2)       - Cd_bound(1))       .* LHS(:,2);
time_gap_vector = time_gap_bound(1) + (time_gap_bound(2) - time_gap_bound(1)) .* LHS(:,3);
Cr_vector       = Cr_bound(1)       + (Cr_bound(2)       - Cr_bound(1))       .* LHS(:,4);
slope_vector    = slope_bound(1)    + (slope_bound(2)    - slope_bound(1))    .* LHS(:,5);
v_des_vector    = v_des_bound(1)    + (v_des_bound(2)    - v_des_bound(1))    .* LHS(:,6);

Input_Parameters = [m_vector, Cd_vector, time_gap_vector, Cr_vector, slope_vector, v_des_vector];

%% BASIC STATS (THEORETICAL FOR UNIFORM + ESTIMATED FROM SAMPLES)
Stat = zeros(6,2);
[Stat(1,1), Stat(1,2)] = unifstat(m_bound(1),        m_bound(2));
[Stat(2,1), Stat(2,2)] = unifstat(Cd_bound(1),       Cd_bound(2));
[Stat(3,1), Stat(3,2)] = unifstat(time_gap_bound(1), time_gap_bound(2));
[Stat(4,1), Stat(4,2)] = unifstat(Cr_bound(1),       Cr_bound(2));
[Stat(5,1), Stat(5,2)] = unifstat(slope_bound(1),    slope_bound(2));
[Stat(6,1), Stat(6,2)] = unifstat(v_des_bound(1),    v_des_bound(2));

Stat_estimate          = zeros(6,6);
Stat_estimate(:,1)     = mean(Input_Parameters).';
Stat_estimate(:,2)     = var(Input_Parameters).';
Stat_estimate(:,3)     = median(Input_Parameters).';
Stat_estimate(:,4)     = mode(Input_Parameters).';
Stat_estimate(:,5)     = std(Input_Parameters).';
Stat_estimate(:,6)     = skewness(Input_Parameters).';

%% QUICK VISUAL CHECKS
figure()
plotmatrix(Input_Parameters)
title('Input Parameters (LHS Samples)')

figure()
tiledlayout(3,2)

nexttile
histogram(Input_Parameters(:,1),'Normalization','probability'); xlabel("Mass [kg]"); ylabel("Rel. Probability")

nexttile
histogram(Input_Parameters(:,2),'Normalization','probability'); xlabel("Cd [-]"); ylabel("Rel. Probability")

nexttile
histogram(Input_Parameters(:,3),'Normalization','probability'); xlabel("Time-Gap [s]"); ylabel("Rel. Probability")

nexttile
histogram(Input_Parameters(:,4),'Normalization','probability'); xlabel("Cr [-]"); ylabel("Rel. Probability")

nexttile
histogram(Input_Parameters(:,5),'Normalization','probability'); xlabel("Slope [rad]"); ylabel("Rel. Probability")

nexttile
histogram(Input_Parameters(:,6),'Normalization','probability'); xlabel("Desired Speed v\_des [m/s]"); ylabel("Rel. Probability")

%% CREATE OUTPUT FOLDER
cartella = strcat('Simulation_Output_', string(datetime('now','Format','yyyy-MM-dd''_''HH-mm')));
mkdir(cartella)

PathName = 'G:\Third Semester\TVARV\Project_Final';
addpath(PathName)

%% SIMULATION LOOP
N = size(Input_Parameters,1);
digitsN = floor(log10(N))+1;

for i = 1:N

    % Progress Counter
    fprintf('  Simulation %d / %d\n', i, N);

    % Assign sampled parameters (workspace variables used by Simulink)
    m        = Input_Parameters(i,1);
    Cd       = Input_Parameters(i,2);
    time_gap = Input_Parameters(i,3);

    Cr       = Input_Parameters(i,4);
    slope    = Input_Parameters(i,5);
    v_des    = Input_Parameters(i,6);

    fprintf(['Run %3d/%3d | ', ...
         'v_des=%6.2f m/s | ', ...
         'time_gap=%4.2f s | ', ...
         'slope=%6.3f rad | ', ...
         'm=%6.0f kg | ', ...
         'Cd=%5.3f | ', ...
         'Cr=%5.2f\n'], ...
         i, N, v_des, time_gap, slope, m, Cd, Cr);

    % Run simulation AND CAPTURE OUTPUT
    simOut = sim('ModelF.slx');   % <-- THIS is the fix

    % Save input vector for traceability
    Input = [m, Cd, time_gap, Cr, slope, v_des];

    % Save outputs
    name = sprintf('Simulation_%0*d.mat', digitsN, i);

    % Save ONLY what we need 
    save(name, 'Input', 'simOut');

    % Move file into output folder
    movefile(name, cartella);

end

