clear all
close all
clc

dt = 0.1;   % <-- Simulink sample time [s]


%% TAKE DATA FROM FOLDER (newest Simulation_Output_*)
Folders = dir(pwd);
dirFlags = [Folders.isdir];
subFolders = Folders(dirFlags);
subFolderNames = {subFolders(3:end).name};     % skip . and ..
subFolderNames = subFolderNames(contains(subFolderNames,'Simulation_Output_'));
% consider the newest folder
PathName = append(pwd,'\',subFolderNames(end));
PathName = PathName{1};

%% Add all matrixes
addpath(PathName)                   % Add the Output folder to Math Path
file_all = dir(fullfile(PathName,'*.mat'));
matfile = file_all([file_all.isdir] == 0); 
clear file_all
sim_ciclo = 1;  

%% MATRIX CREATION
% KPIs:
% 1) min Relative Distance
% 2) collision flag (0/1)
% 3) min TTC
% 4) max abs jerk
T = 4;                                      % Number of Outputs
Output_Matrix = zeros(length(matfile), T);  % Pre-allocate

%% LOOP (extract inputs + compute KPIs)


for i = 1:length(matfile)

    S = load(matfile(i).name);

    if i == 1
        Input_Matrix = zeros(length(matfile), length(S.Input));
    end
    Input_Matrix(i,:) = S.Input;

    % ---- Extract signals ----
    d_rel = S.simOut.Relative_Distance(:);
    v_rel = S.simOut.Relative_Velocity(:);
    acc   = S.simOut.Acceleration(:);


    % ---- KPI 1: min relative distance ----
    Output_Matrix(i,1) = min(d_rel);

    % ---- KPI 2: collision flag ----
    Output_Matrix(i,2) = any(d_rel <= 0);

    % ---- KPI 3: min TTC (closing only) ----
    % TTC = d_rel / (-v_rel) when v_rel < 0 (closing)
    idx_closing = (v_rel < 0) & (d_rel > 0);
    TTC = d_rel(idx_closing) ./ (-v_rel(idx_closing));
    TTC = TTC(TTC > 0);   % remove invalid / negative
    if isempty(TTC)
        Output_Matrix(i,3) = NaN;
    else
        Output_Matrix(i,3) = min(TTC);
    end

    % ---- KPI 4: max absolute jerk ----
    jerk = diff(acc) / dt;
    Output_Matrix(i,4) = max(abs(jerk));

end


clearvars -except Input_Matrix Output_Matrix PathName T dt


%% label inputs (helps interpret plotmatrix / correlations)
% Input order from our simulation code:
% 1=m, 2=Cd, 3=time_gap, 4=Cr, 5=slope, 6=v_des
Input_Names = ["m","Cd","time_gap","Cr","slope","v_des"];

KPI_Names = ["Min Relative Distance [m]", ...
             "Collision (0/1)", ...
             "Min TTC [s]", ...
             "Max |Jerk| [m/s^3]"];
%% OUTPUT ANALYSIS (stats)
Stat_Out = zeros(T,6);
Stat_Out(:,1) = mean(Output_Matrix);
Stat_Out(:,2) = var(Output_Matrix);
Stat_Out(:,3) = median(Output_Matrix);
Stat_Out(:,4) = mode(Output_Matrix);
Stat_Out(:,5) = std(Output_Matrix);
Stat_Out(:,6) = skewness(Output_Matrix);

%% COLLISION RATE (meaningful KPI summary)
collision_rate = mean(Output_Matrix(:,2));
fprintf('Collision rate: %.2f %%\n', 100*collision_rate);

%% PLOTS: PDF + CDF (only meaningful for continuous KPIs)
continuous_idx = [1 3 4];   % skip collision


for k = continuous_idx
    figure()
    subplot(1,2,1)
    histogram(Output_Matrix(:,k),'Normalization','probability')
    xlabel(KPI_Names(k))
    ylabel('PDF')

    subplot(1,2,2)
    cdfplot(Output_Matrix(:,k))
    xlabel(KPI_Names(k))
    ylabel('CDF')
end

%% NORMALITY TESTS (only meaningful for continuous KPIs)
norm_test = NaN(numel(continuous_idx), 2, 2); % (kpi_index, test#(1=lillie,2=ks), [h p])

for ii = 1:numel(continuous_idx)
    k = continuous_idx(ii);
    x = Output_Matrix(:,k);
    x = x(isfinite(x));
    if numel(x) >= 5
        [h1,p1] = lillietest(x,'Alpha',0.05);
        [h2,p2] = kstest((x - mean(x)) ./ std(x),'Alpha',0.05); % standardize data before kstest
        norm_test(ii,1,:) = [h1 p1];
        norm_test(ii,2,:) = [h2 p2];
    end
end

%% BOXPLOTS (only meaningful for continuous KPIs)
for k = continuous_idx
    figure()
    boxplot(Output_Matrix(:,k))
    ylabel(KPI_Names(k))
    title(['Boxplot - ' char(KPI_Names(k))])
end

%% PLOTMATRIX (inputs vs each continuous KPI)
for k = continuous_idx
    figure()
    plotmatrix([Input_Matrix, Output_Matrix(:,k)])
    sgtitle(['Inputs vs ' char(KPI_Names(k))])
end

% -5- Quantitative  analysis of the input-output relationship:

%% CORRELATION: Quantitative analysis of the input-output relationship(inputs vs each KPI separately)
% Correlation Coefficients
R_threshold = 0.4;
P_threshold = 0.05;

for k = 1:T
    y = Output_Matrix(:,k);

    % For TTC and jerk, remove NaNs
    valid = isfinite(y);
    X = Input_Matrix(valid,:);
    y = y(valid);

    [R,P] = corrcoef([X, y],'Alpha',0.05);

% We need to obtain the index of the input variables in M2 that satisfy 
% two conditions: 1) R > R_threshold; 2) P < P_threshold. We need to look
% at the last row (first three columns) of R and P matrices.

    Index_Sign = [];    % create an enmpty vector
    for j = 1:size(X,2)
        if R(end,j) >= R_threshold && P(end,j) <= P_threshold
            Index_Sign = [Index_Sign; j];
        end
    end

    fprintf('\nKPI: %s\n', char(KPI_Names(k)));
    if isempty(Index_Sign)
        fprintf('  No significant inputs found (R>=%.2f and P<=%.2f).\n', R_threshold, P_threshold);
        continue
    else
        fprintf('  Significant inputs: ');
        fprintf('%s ', Input_Names(Index_Sign));
        fprintf('\n');
    end

% Multiple linear regression with Significant variables 
    
    if k == 2
        fprintf('  Regression skipped for Collision (binary KPI).\n');
        continue
    end

    x_dummy = [ones(size(X,1),1), X(:,Index_Sign)];    %regression matrix
    [beta,bint,r] = regress(y, x_dummy);               %build regression model
    y_hat = x_dummy * beta;


 % ---- Print regression equation ----


    fprintf('\nRegression equation for KPI: %s\n', char(KPI_Names(k)));
    fprintf('y = %.4g', beta(1));   % intercept

    for j = 1:length(Index_Sign)
         coeff = beta(j+1);
         varname = Input_Names(Index_Sign(j));

         if coeff >= 0
              fprintf(' + %.4g·%s', coeff, varname);
          else
              fprintf(' - %.4g·%s', abs(coeff), varname);
         end
    end

    fprintf('\n');


    %parity plot
    figure()
    plot(y, y_hat, 'o')
    grid on
    xlabel(['True - ' char(KPI_Names(k))])
    ylabel(['Predicted - ' char(KPI_Names(k))])
    title(['Regression: True vs Predicted (' char(KPI_Names(k)) ')'])

    hold on
    mn = min([y; y_hat]);
    mx = max([y; y_hat]);
    plot([mn mx], [mn mx], 'LineWidth', 1.5)
    axis equal
    xlim([mn mx]); ylim([mn mx]);
    hold off

end

%% REMOVE PATH
rmpath(PathName)
