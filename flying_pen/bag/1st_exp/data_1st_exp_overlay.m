%% CSV overlay normal/force analysis (1009 first experiment)
% Put this script beside the cropped CSV files and run it. Every CSV in the
% script directory is loaded; each recording keeps its own time origin.
clear; close all; clc;

%% Configuration and CSV loading
% Values copied from su_params.yaml on 2026-10-10. This script deliberately
% does not read the YAML file at run time.
% Prefer the file currently open/running in MATLAB Editor. Fall back cleanly
% for command-line execution and copied folders.
data_dir = activeScriptDirectory();
fprintf('CSV selection folder: %s\n',data_dir);
global_xlim = [0 62];                  % [s], e.g. [10 60]; [] uses full time range
cfg.beta = 3.0;                    % su_position.normBeta [1/s]
cfg.gamma = 2.5;                   % su_position.normGamma [1/s]
cfg.eps_v = 0.0009;                % su_position.normEpsV [m^2/s^2]
cfg.eps_f = 0.005;                 % su_position.normEpsF [N]
% Five-section surface fitted to wind0_*_trimmed.csv (0-60 s).
% Robust spatial-bin fit RMS: 0.99 mm in the X-Z plane. The two arc angles
% are independent; a common radius was better conditioned than two radii.
cfg.origin = [1.376816 0.020818 0.840479]; % fitted [x,y,z] origin [m]
cfg.initial_tangent_deg = 112.1065; % fitted lower-plane tangent [deg]
cfg.plane_lengths = [0.098803 0.294582 0.105241]; % lower/middle/upper [m]
cfg.curve_radius = 0.711997;        % fitted common arc radius [m]
cfg.turn_angles_deg = [21.8979 29.4695]; % lower/upper arc angles [deg]
cfg.true_normal_hold_time = 54.0;   % [s], hold endpoint normal after this time

% Select one CSV for a single plot, or Ctrl/Shift-select multiple CSV files
% to overlay them on the same axes.
[selected_files,selected_path] = uigetfile( ...
    {'*.csv','Normal-estimator CSV files (*.csv)'}, ...
    'Select one or more CSV files',data_dir,'MultiSelect','on');
if isequal(selected_files,0), return; end
if ischar(selected_files) || isstring(selected_files)
    selected_files = cellstr(selected_files);
end
selected_files = sort(selected_files);
records = struct([]);
for file_index = 1:numel(selected_files)
    csv_path = fullfile(selected_path,selected_files{file_index});
    fprintf('Loading %s\n',csv_path);
    table_data = readtable(csv_path,detectImportOptions(csv_path,'Delimiter',','));
    record = prepareRecord(table_data,selected_files{file_index},cfg);
    if file_index == 1, records = record; else, records(file_index) = record; end
end
% Display one common surface at the mean fitted pose; true normals remain
% based on each recording's own fitted pose.
fittedOrigins=vertcat(records.geometry_cfg);
fittedOrigins=vertcat(fittedOrigins.origin);
cfg.origin=mean(fittedOrigins,1);
fprintf(['Loaded %d CSV files | beta=%.4g, gamma=%.4g, ' ...
    'eps_v=%.4g, eps_f=%.4g\n'],numel(records),cfg.beta,cfg.gamma,cfg.eps_v,cfg.eps_f);

dataset_labels = repmat({''},numel(records),1);
if numel(records) > 1
    for file_index = 1:numel(records)
        dataset_labels{file_index} = extractWindLabel(records(file_index).label);
    end
end

axis_names = {'x','y','z'};
record_colors = lines(numel(records));
method_names = {'$m_1$: $\hat{f}_{\ell}$ + velocity correction', ...
    '$m_2$: $\bar{f}_{\ell}$ + velocity correction', ...
    '$m_3$: $\hat{f}_{c}$, no velocity correction', ...
    '$m_4$: $\hat{f}_{c}$ + velocity correction'};
method_styles = {':','--','-.','-'};
method_widths = [1.8 1.8 2.0 2.4];

printOfflineMetricTable(records,global_xlim);

%% Key displays: traversal summary, geometry view, and plane-2 comparison
plotFirstExperimentKeyDisplays(records,dataset_labels,record_colors, ...
    global_xlim,cfg,[40 60],680,115);

%% 0. Overview panel
plotOverviewPanel(records,dataset_labels,record_colors,method_names, ...
    method_styles,method_widths,axis_names,global_xlim,cfg, ...
    [80 60 1350 800]); % figure [left bottom width height] [px]

%% 0-1. Repeated-experiment normal-error statistics
ensemble_error_xlim = [0 36]; % [s]; use [] for the full available time range
comparison_method_indices = [3 4 2 1];
all_comparison_method_labels = {'$\hat{f}_{\ell}$','$\bar{f}_{\ell}$', ...
    '$\hat{f}_c$','$\hat{f}_c$ without $\epsilon_v$'};
comparison_method_labels = all_comparison_method_labels(comparison_method_indices);
plotEnsembleErrorStatistics(records,method_names,ensemble_error_xlim, ...
    comparison_method_indices,comparison_method_labels);

%% 1. Tangential velocity t1/t2 and normal force Fn
tracking_ylim = {[-0.2 0.2],[-0.2 0.2],[0 0.04]}; % {t1,t2,Fn}; use [] for auto
figure('Name','Tangential velocity and normal force','Color','w','Position',[80 90 1350 850]);
tracking_axes = gobjects(3,1);
tracking_names = {'t1 velocity','t2 velocity','normal force Fn'};
tracking_units = {'v_{t1} [m/s]','v_{t2} [m/s]','F_n [N]'};
for component = 1:3
    tracking_axes(component) = subplot(3,1,component); hold on;
    for file_index = 1:numel(records)
        if component == 1
            desired = records(file_index).u1_des;
            measured = records(file_index).v_t1;
        elseif component == 2
            desired = records(file_index).u2_des;
            measured = records(file_index).v_t2;
        else
            desired = records(file_index).force_desired;
            measured = records(file_index).fn_hat;
        end
        plot(records(file_index).t,desired,'--','Color', ...
            curveColor(record_colors(file_index,:),1,numel(records)), ...
            'LineWidth',1.6,'DisplayName',legendLabel(dataset_labels{file_index},'desired'));
        plot(records(file_index).t,measured,'-','Color', ...
            curveColor(record_colors(file_index,:),2,numel(records)), ...
            'LineWidth',2.0,'DisplayName',legendLabel(dataset_labels{file_index},'measured'));
    end
    grid on; box on; ylabel(tracking_units{component});
    applyConfiguredYLim(tracking_axes(component),tracking_ylim{component});
    title(tracking_names{component});
    if component == 1, legend('Location','eastoutside','Interpreter','none'); end
end
xlabel(tracking_axes(end),'Time [s]');
set(tracking_axes(1:end-1),'XTickLabel',[]);
linkaxes(tracking_axes,'x');
applyGlobalXLim(tracking_axes,global_xlim);

%% 2. Normal estimation x/y/z: firmware and offline validation variants
normal_ylim = {[-1.1 1.1],[-1.1 1.1],[-1.1 1.1],[0 90]}; % {x,y,z,error angle deg}; [] for auto
figure('Name','Normal estimation overlay','Color','w','Position',[100 80 1350 850]);
normal_axes = gobjects(4,1);
for component = 1:3
    normal_axes(component) = subplot(4,1,component); hold on;
    % Draw every true-normal curve before any estimate so all stay behind.
    for file_index = 1:numel(records)
        plot(records(file_index).t,records(file_index).true_normal(:,component), ...
            'Color',[0.76 0.76 0.76],'LineStyle','-','LineWidth',3.0, ...
            'DisplayName',legendLabel(dataset_labels{file_index},'true normal'));
    end
    for file_index = 1:numel(records)
        normal_set = cat(3,records(file_index).n_velocity_on_fraw, ...
            records(file_index).n_velocity_on_fbar, ...
            records(file_index).n_velocity_off_fc,records(file_index).n_velocity_on_fc);
        for method = 1:numel(method_names)
            label = legendLabel(dataset_labels{file_index},method_names{method});
            plot(records(file_index).t,normal_set(:,component,method), ...
                'Color',curveColor(record_colors(file_index,:),method,numel(records)), ...
                'LineStyle',method_styles{method},'LineWidth',method_widths(method), ...
                'DisplayName',label);
        end
    end
    grid on; box on;
    applyConfiguredYLim(normal_axes(component),normal_ylim{component});
    ylabel(sprintf('n_%s [-]',axis_names{component}));
    if component == 1
        title('Normal Estimation');
        legend('Location','eastoutside','Interpreter','latex');
    end
end
normal_axes(4) = subplot(4,1,4); hold on;
for file_index = 1:numel(records)
    normal_set = cat(3,records(file_index).n_velocity_on_fraw, ...
        records(file_index).n_velocity_on_fbar, ...
        records(file_index).n_velocity_off_fc,records(file_index).n_velocity_on_fc);
    for method = 1:numel(method_names)
        angle_error = angleBetweenRowsDeg(normal_set(:,:,method), ...
            records(file_index).true_normal);
        plot(records(file_index).t,angle_error, ...
            'Color',curveColor(record_colors(file_index,:),method,numel(records)), ...
            'LineStyle',method_styles{method},'LineWidth',method_widths(method), ...
            'DisplayName',legendLabel(dataset_labels{file_index},method_names{method}));
    end
end
grid on; box on; ylabel('angle [deg]'); title('Angle between estimated and true normal');
applyConfiguredYLim(normal_axes(4),normal_ylim{4});
legend('Location','eastoutside','Interpreter','latex');
xlabel(normal_axes(end),'Time [s]');
set(normal_axes(1:end-1),'XTickLabel',[]);
linkaxes(normal_axes,'x');
applyGlobalXLim(normal_axes,global_xlim);

%% 3. Force estimation x/y/z: raw, bar, and contact-consistent force
force_ylim = {[-60 60],[-60 60],[-60 60]}; % [mN], {x,y,z}; use [] for auto
figure('Name','Force estimation overlay','Color','w','Position',[120 70 1350 850]);
force_axes = gobjects(3,1);
force_names = {'$\hat{f}_{\ell}$','$\bar{f}_{\ell}$','$\hat{f}_{c}$'};
force_styles = {':','--','-'};
for component = 1:3
    force_axes(component) = subplot(3,1,component); hold on;
    for file_index = 1:numel(records)
        force_set = cat(3,records(file_index).f_raw,records(file_index).f_bar,records(file_index).f_c);
        for method = 1:3
            plot(records(file_index).t,1e3*force_set(:,component,method), ...
                'Color',curveColor(record_colors(file_index,:),method,numel(records)), ...
                'LineStyle',force_styles{method},'LineWidth',1.7, ...
                'DisplayName',legendLabel(dataset_labels{file_index},force_names{method}));
        end
    end
    grid on; box on; ylabel(sprintf('f_%s [mN]',axis_names{component}));
    applyConfiguredYLim(force_axes(component),force_ylim{component});
    if component == 1
        title('Force Estimation');
        legend('Location','eastoutside','Interpreter','latex');
    end
end
xlabel(force_axes(end),'Time [s]');
set(force_axes(1:end-1),'XTickLabel',[]);
linkaxes(force_axes,'x');
applyGlobalXLim(force_axes,global_xlim);

%% 4. Normalized force estimation x/y/z versus true surface normal
unit_force_ylim = {[-1.1 1.1],[-1.1 1.1],[-1.1 1.1],[0 90]}; % {x,y,z,error angle deg}; [] for auto
figure('Name','Normalized force vs true normal','Color','w','Position',[140 60 1350 850]);
unit_axes = gobjects(4,1);
for component = 1:3
    unit_axes(component) = subplot(4,1,component); hold on;
    % Draw every true-normal curve before any force direction.
    for file_index = 1:numel(records)
        plot(records(file_index).t,records(file_index).true_normal(:,component), ...
            'Color',[0.76 0.76 0.76],'LineStyle','-','LineWidth',3.0, ...
            'DisplayName',legendLabel(dataset_labels{file_index},'true normal'));
    end
    for file_index = 1:numel(records)
        force_set = cat(3,records(file_index).f_raw_unit, ...
            records(file_index).f_bar_unit,records(file_index).f_c_unit);
        for method = 1:3
            plot(records(file_index).t,force_set(:,component,method), ...
                'Color',curveColor(record_colors(file_index,:),method,numel(records)), ...
                'LineStyle',force_styles{method},'LineWidth',1.6, ...
                'DisplayName',legendLabel(dataset_labels{file_index},force_names{method}));
        end
    end
    grid on; box on;
    applyConfiguredYLim(unit_axes(component),unit_force_ylim{component});
    ylabel(sprintf('%s component [-]',axis_names{component}));
    if component == 1
        title('Normalized force directions');
        legend('Location','eastoutside','Interpreter','latex');
    end
end
unit_axes(4) = subplot(4,1,4); hold on;
for file_index = 1:numel(records)
    unit_set = cat(3,records(file_index).f_raw_unit, ...
        records(file_index).f_bar_unit,records(file_index).f_c_unit);
    for method = 1:3
        direction_error = angleBetweenRowsDeg(unit_set(:,:,method), ...
            records(file_index).true_normal);
        plot(records(file_index).t,direction_error, ...
            'Color',curveColor(record_colors(file_index,:),method,numel(records)), ...
            'LineStyle',force_styles{method},'LineWidth',1.7, ...
            'DisplayName',legendLabel(dataset_labels{file_index},force_names{method}));
    end
end
grid on; box on; ylabel('angle [deg]');
title('Angle between normalized force direction and true normal');
applyConfiguredYLim(unit_axes(4),unit_force_ylim{4});
legend('Location','eastoutside','Interpreter','latex');
xlabel(unit_axes(end),'Time [s]');
set(unit_axes(1:end-1),'XTickLabel',[]);
linkaxes(unit_axes,'x');
applyGlobalXLim(unit_axes,global_xlim);

%% 5. End-effector trajectory and estimated-normal arrows
trajectory_iso_time = [0 60];       % [s], left: isometric view
trajectory_xz_time_1 = [0 30];      % [s], middle: X-Z view
trajectory_xz_time_2 = [30 60];     % [s], right: X-Z view
trajectory_iso_arrow_period = 6.0;  % [s], isometric normal-arrow interval
trajectory_xz_arrow_period = 3.0;   % [s], X-Z normal-arrow interval
trajectory_arrow_length = 0.08;     % [m], displayed arrow length
surface_y_half_width = 0.20;        % [m], surface width = origin_y +/- 20 cm

figure('Name','End-effector trajectory and normal arrows','Color','w', ...
    'Position',[60 180 1700 560]);
trajectory_axes = gobjects(3,1);

% Left: 3-D isometric view and known surface.
trajectory_axes(1) = subplot(1,3,1); hold on;
surface_u = linspace(0,1,501).';
[surface_xz,~] = fiveSectionGeometry(surface_u,cfg);
surface_y = cfg.origin(2)+linspace(-surface_y_half_width,surface_y_half_width,31);
surface_x = repmat(surface_xz(:,1),1,numel(surface_y));
surface_z = repmat(surface_xz(:,2),1,numel(surface_y));
surface_y_mesh = repmat(surface_y,size(surface_xz,1),1);
surf(trajectory_axes(1),surface_x,surface_y_mesh,surface_z, ...
    'FaceColor',[0.45 0.86 0.82],'FaceAlpha',0.34, ...
    'EdgeColor',[0.60 0.82 0.86],'EdgeAlpha',0.10, ...
    'FaceLighting','gouraud','AmbientStrength',0.55, ...
    'DiffuseStrength',0.75,'SpecularStrength',0.22,'SpecularExponent',18, ...
    'DisplayName','reference surface');
for file_index = 1:numel(records)
    selected = timeMask(records(file_index).t,trajectory_iso_time);
    plot3(records(file_index).ee_pos(selected,1),records(file_index).ee_pos(selected,2), ...
        records(file_index).ee_pos(selected,3),'LineWidth',3.2, ...
        'Color',[0.42 0.42 0.42],'DisplayName', ...
        legendLabel(dataset_labels{file_index},'end-effector trajectory'));
    trajectory_normals = cat(3,records(file_index).n_velocity_on_fraw, ...
        records(file_index).n_velocity_on_fbar,records(file_index).n_velocity_off_fc, ...
        records(file_index).n_velocity_on_fc);
    for method = 1:numel(method_names)
        normal_data = trajectory_normals(:,:,method);
        arrows = timeSpacedIndices(records(file_index).t,selected& ...
            all(isfinite(records(file_index).ee_pos),2)& ...
            all(isfinite(normal_data),2),trajectory_iso_arrow_period);
        quiver3(records(file_index).ee_pos(arrows,1),records(file_index).ee_pos(arrows,2), ...
            records(file_index).ee_pos(arrows,3), ...
            trajectory_arrow_length*normal_data(arrows,1), ...
            trajectory_arrow_length*normal_data(arrows,2), ...
            trajectory_arrow_length*normal_data(arrows,3),0, ...
            'Color',curveColor(record_colors(file_index,:),method,numel(records)), ...
            'LineStyle',method_styles{method},'LineWidth',2.2,'MaxHeadSize',0.9, ...
            'DisplayName',legendLabel(dataset_labels{file_index},method_names{method}));
    end
end
grid on; box on; axis equal; view(-42,24);
camlight(trajectory_axes(1),'headlight'); material(trajectory_axes(1),'dull');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Isometric view: 0-60 s'); legend('Location','best','Interpreter','latex');

% Middle/right: X-Z projections for the requested time intervals.
xz_windows = {trajectory_xz_time_1,trajectory_xz_time_2};
for panel = 1:2
    trajectory_axes(panel+1) = subplot(1,3,panel+1); hold on;
    plot(surface_xz(:,1),surface_xz(:,2),'Color',[0.45 0.82 0.84], ...
        'LineWidth',6.0,'DisplayName','reference surface');
    for file_index = 1:numel(records)
        selected = timeMask(records(file_index).t,xz_windows{panel});
        plot(records(file_index).ee_pos(selected,1),records(file_index).ee_pos(selected,3), ...
            'LineWidth',3.2,'Color',[0.58 0.58 0.58], ...
            'DisplayName',legendLabel(dataset_labels{file_index},'end-effector trajectory'));
        trajectory_normals = cat(3,records(file_index).n_velocity_on_fraw, ...
            records(file_index).n_velocity_on_fbar,records(file_index).n_velocity_off_fc, ...
            records(file_index).n_velocity_on_fc);
        for method = 1:numel(method_names)
            normal_data = trajectory_normals(:,:,method);
            arrows = timeSpacedIndices(records(file_index).t,selected& ...
                all(isfinite(records(file_index).ee_pos),2)& ...
                all(isfinite(normal_data),2),trajectory_xz_arrow_period);
            quiver(records(file_index).ee_pos(arrows,1),records(file_index).ee_pos(arrows,3), ...
                trajectory_arrow_length*normal_data(arrows,1), ...
                trajectory_arrow_length*normal_data(arrows,3),0, ...
                'Color',curveColor(record_colors(file_index,:),method,numel(records)), ...
                'LineStyle',method_styles{method},'LineWidth',2.2,'MaxHeadSize',0.9, ...
                'DisplayName',legendLabel(dataset_labels{file_index},method_names{method}));
        end
    end
    grid on; box on; axis equal;
    xlabel('x [m]'); ylabel('z [m]');
    title(sprintf('X-Z view: %.0f-%.0f s',xz_windows{panel}));
    legend('Location','best','Interpreter','latex');
end

%% Local functions
function record = prepareRecord(T,file_name,cfg)
t = safeColumn(T,'t_sec');
if all(~isfinite(t)), t = (0:height(T)-1).'; end
first_finite = find(isfinite(t),1);
if ~isempty(first_finite), t = t-t(first_finite); end
vc = columnsXYZ(T,'fp_normal_vc');
t1 = columnsXYZ(T,'fp_normal_t1');
t2 = columnsXYZ(T,'fp_normal_t2');
f_c = columnsXYZ(T,'fp_normal_fcHat');
f_bar = columnsXYZ(T,'fp_normal_fBar');
f_raw = columnsXYZ(T,'fp_normal_fRaw');
ee_pos = columnsXYZ(T,'fp_normal_eePos');

record.label = erase(file_name,'.csv');
record.t = t;
record.f_c = f_c;
record.f_bar = f_bar;
record.f_raw = f_raw;
record.ee_pos = ee_pos;
record.u1_des = safeColumn(T,'u1_des');
record.u2_des = safeColumn(T,'u2_des');
record.v_t1 = sum(vc.*t1,2);
record.v_t2 = sum(vc.*t2,2);
record.velocity = vc;
record.speed_filtered = zeroPhaseLowPass(vecnorm(vc,2,2),t,1.0);
record.force_desired = safeColumn(T,'forceDesired');
record.fn_hat = safeColumn(T,'fp_normal_fnHat');
record.n_firmware = columnsXYZ(T,'fp_normal_nHat');
record.n_velocity_on_fbar = offlineNormalEstimator(t,vc,f_bar,cfg,true);
record.n_velocity_on_fraw = offlineNormalEstimator(t,vc,f_raw,cfg,true);
record.n_velocity_on_fc = offlineNormalEstimator(t,vc,f_c,cfg,true);
record.n_velocity_off_fc = offlineNormalEstimator(t,vc,f_c,cfg,false);
record.f_c_unit = normalizeRows(f_c,1e-12);
record.f_bar_unit = normalizeRows(f_bar,1e-12);
record.f_raw_unit = normalizeRows(f_raw,1e-12);
[record.geometry_cfg,record.geometry_rms,record.surface_loop_time] = ...
    fitFiveSectionPose(t,ee_pos,cfg);
[record.true_normal,record.surface_section,record.surface_arc,record.surface_distance, ...
    record.surface_bounds] = trueNormalFromKnownGeometry(ee_pos,record.geometry_cfg,0.04);
middlePlaneMid=0.5*(record.surface_bounds(3)+record.surface_bounds(4));
upperCurveMid=0.5*(record.surface_bounds(4)+record.surface_bounds(5));
record.friction_plane_mask=record.surface_distance<=0.04 & ...
    record.surface_arc>=middlePlaneMid-0.10&record.surface_arc<=middlePlaneMid;
record.friction_curve_mask=record.surface_distance<=0.04 & ...
    record.surface_arc>=upperCurveMid&record.surface_arc<=record.surface_bounds(5);
record.friction_mask=record.friction_plane_mask|record.friction_curve_mask;
fprintf('  %s | surface RMS=%.2f mm, closed sweep=%.2f-%.2f s\n', ...
    record.label,1e3*record.geometry_rms,record.surface_loop_time);
end

function n_history = offlineNormalEstimator(t,velocity,force,cfg,use_qg)
% Offline directional-memory normal estimator. The normal-rate update is
% intentionally not velocity-gated; velocity is used only for optional q_g.
n_samples = numel(t); n_history = nan(n_samples,3);
L = zeros(3); n = nan(3,1); initialized = false;
for sample = 1:n_samples
    if ~all(isfinite([velocity(sample,:) force(sample,:)])), continue; end
    force_norm = norm(force(sample,:));
    if force_norm <= cfg.eps_f, continue; end
    qf = force(sample,:).'/force_norm;
    v = velocity(sample,:).'; v2 = dot(v,v);
    if use_qg && cfg.eps_v > 0
        q = qf-v*(dot(v,qf)/(v2+cfg.eps_v));
    else
        q = qf;
    end
    q_norm = norm(q);
    if ~isfinite(q_norm) || q_norm <= 1e-6, continue; end
    q = q/q_norm;
    if sample == 1, dt = 0.01; else, dt = t(sample)-t(sample-1); end
    if ~isfinite(dt) || dt <= 0, dt = 0.01; end
    % Firmware task runs at 100 Hz; split larger logger intervals accordingly.
    steps = max(1,ceil(dt/0.01)); sub_dt = dt/steps;
    for substep = 1:steps
        L = L+sub_dt*(-cfg.beta*L+cfg.beta*(q*q.'));
        if ~initialized
            n = q; initialized = true;
        else
            Ln = L*n;
            n_dot = cfg.gamma*(Ln-n*dot(n,Ln));
            n_next = n+sub_dt*n_dot;
            if norm(n_next) > 1e-6, n = n_next/norm(n_next); else, n = q; end
        end
    end
    if dot(n,force(sample,:).') < 0, n = -n; end
    n_history(sample,:) = n.';
end
end

function filtered=zeroPhaseLowPass(data,t,cutoffHz)
% Forward/backward first-order RC low-pass for display without phase delay.
filtered=nan(size(data)); rc=1/(2*pi*cutoffHz);
for column=1:size(data,2)
    valid=isfinite(t)&isfinite(data(:,column));
    if sum(valid)<2, continue; end
    x=interp1(t(valid),data(valid,column),t,'linear','extrap');
    forward=x;
    for sample=2:numel(t)
        dt=t(sample)-t(sample-1);
        if ~isfinite(dt)||dt<=0, dt=0.01; end
        alpha=dt/(rc+dt);
        forward(sample)=forward(sample-1)+alpha*(x(sample)-forward(sample-1));
    end
    backward=forward;
    for sample=numel(t)-1:-1:1
        dt=t(sample+1)-t(sample);
        if ~isfinite(dt)||dt<=0, dt=0.01; end
        alpha=dt/(rc+dt);
        backward(sample)=backward(sample+1)+alpha*(forward(sample)-backward(sample+1));
    end
    backward(~valid)=nan; filtered(:,column)=backward;
end
end

function [fittedCfg,rmsError,loopTime] = fitFiveSectionPose(t,eePos,cfg)
% Fit only the X-Z translation of the already calibrated five-section shape.
% The longest closed out-and-back surface sweep is detected geometrically,
% so approach/departure tails cannot pull the fit away from the surface.
valid=isfinite(t)&all(isfinite(eePos(:,[1 3])),2);
validIdx=find(valid); pointsAll=eePos(validIdx,[1 3]);
[loopStart,loopEnd]=longestClosedLoop(pointsAll,0.035,1.0);
loopIdx=validIdx(loopStart:loopEnd); loopPoints=eePos(loopIdx,[1 3]);

% Spatial binning prevents slow sections and dwell time from dominating.
zEdges=linspace(min(loopPoints(:,2)),max(loopPoints(:,2)),101);
fitPoints=zeros(0,2);
for bin=1:numel(zEdges)-1
    selected=loopPoints(:,2)>=zEdges(bin)&loopPoints(:,2)<zEdges(bin+1);
    if bin==numel(zEdges)-1, selected=selected|loopPoints(:,2)==zEdges(end); end
    if any(selected), fitPoints(end+1,:)=median(loopPoints(selected,:),1); end %#ok<AGROW>
end
localCfg=cfg; localCfg.origin([1 3])=[0 0];
[localCenterline,~]=fiveSectionGeometry(linspace(0,1,2001).',localCfg);
originGuess=median(fitPoints-localCenterline( ...
    round(linspace(1,size(localCenterline,1),size(fitPoints,1))),:),1);
cost=@(origin)mean(min(pairwiseDistance2D(fitPoints,localCenterline+origin),[],2).^2);
originXZ=fminsearch(cost,originGuess,optimset('Display','off','TolX',1e-10,'TolFun',1e-12));
fittedCfg=cfg; fittedCfg.origin=[originXZ(1),median(eePos(loopIdx,2)),originXZ(2)];
distances=min(pairwiseDistance2D(fitPoints,localCenterline+originXZ),[],2);
rmsError=sqrt(mean(distances.^2)); loopTime=[t(loopIdx(1)) t(loopIdx(end))];
end

function [bestStart,bestEnd] = longestClosedLoop(points,closureTolerance,minLoopLength)
s=[0;cumsum(vecnorm(diff(points,1,1),2,2))];
bestStart=1; bestEnd=size(points,1); bestLength=-inf;
stride=max(1,floor(size(points,1)/1200));
for startIndex=1:stride:size(points,1)
    eligible=find(s-s(startIndex)>=minLoopLength & ...
        vecnorm(points-points(startIndex,:),2,2)<=closureTolerance);
    if isempty(eligible), continue; end
    [candidateLength,which]=max(s(eligible)-s(startIndex));
    if candidateLength>bestLength
        bestLength=candidateLength; bestStart=startIndex; bestEnd=eligible(which);
    end
end
if bestLength<minLoopLength
    warning('No closed surface sweep detected; using all finite EE samples.');
    bestStart=1; bestEnd=size(points,1);
end
end

function distance=pairwiseDistance2D(a,b)
delta=reshape(a,[],1,2)-reshape(b,1,[],2);
distance=sqrt(sum(delta.^2,3));
end

function [true_normal,section_id,arc_position,surface_distance,bounds] = ...
    trueNormalFromKnownGeometry(ee_pos,cfg,max_distance)
u = linspace(0,1,2001).';
[centerline,theta] = fiveSectionGeometry(u,cfg);
normal_dense = [-sin(theta),zeros(size(theta)),cos(theta)];
true_normal = nan(size(ee_pos));
section_id=nan(size(ee_pos,1),1); arc_position=nan(size(ee_pos,1),1);
surface_distance=nan(size(ee_pos,1),1);
turns=abs(deg2rad(cfg.turn_angles_deg));
lengths=[cfg.plane_lengths(1),cfg.curve_radius*turns(1),cfg.plane_lengths(2), ...
    cfg.curve_radius*turns(2),cfg.plane_lengths(3)];
bounds=[0 cumsum(lengths)]; denseArc=u*bounds(end);
for sample = 1:size(ee_pos,1)
    if ~all(isfinite(ee_pos(sample,[1 3]))), continue; end
    [distance_squared,nearest] = min(sum((centerline-ee_pos(sample,[1 3])).^2,2));
    surface_distance(sample)=sqrt(distance_squared); arc_position(sample)=denseArc(nearest);
    section_id(sample)=find(arc_position(sample)<=bounds(2:end)+eps,1,'first');
    if sqrt(distance_squared)<=max_distance
        true_normal(sample,:) = normal_dense(nearest,:);
    end
end
end

function [points,theta_out] = fiveSectionGeometry(u,cfg)
theta = deg2rad(cfg.initial_tangent_deg);
turns = -abs(deg2rad(cfg.turn_angles_deg)); radius = cfg.curve_radius;
plane_lengths = cfg.plane_lengths;
lengths = [plane_lengths(1) radius*abs(turns(1)) plane_lengths(2) ...
    radius*abs(turns(2)) plane_lengths(3)];
bounds = [0 cumsum(lengths)]; s = u*bounds(end);
points = nan(numel(u),2); theta_out = nan(numel(u),1);
position = cfg.origin([1 3]);
for segment = 1:5
    if segment < 5, selected = s>=bounds(segment)&s<bounds(segment+1); ...
    else, selected = s>=bounds(segment)&s<=bounds(segment+1)+eps; end
    local_s = s(selected)-bounds(segment);
    if mod(segment,2)==1
        points(selected,:) = position+local_s.*[cos(theta) sin(theta)];
        theta_out(selected) = theta;
        position = position+plane_lengths((segment+1)/2)*[cos(theta) sin(theta)];
    else
        turn = turns(segment/2);
        curvature = sign(turn)/radius; local_turn = curvature*local_s;
        points(selected,1) = position(1)+(sin(theta+local_turn)-sin(theta))/curvature;
        points(selected,2) = position(2)+(-cos(theta+local_turn)+cos(theta))/curvature;
        theta_out(selected) = theta+local_turn;
        position = position+[(sin(theta+turn)-sin(theta))/curvature, ...
            (-cos(theta+turn)+cos(theta))/curvature];
        theta = theta+turn;
    end
end
end

function x = safeColumn(T,name)
variables = string(T.Properties.VariableNames);
index = find(strcmpi(variables,string(name)),1);
if isempty(index), x = nan(height(T),1); else, x = double(T{:,index}); end
x = x(:);
end

function xyz = columnsXYZ(T,prefix)
xyz = [safeColumn(T,string(prefix)+'X'),safeColumn(T,string(prefix)+'Y'), ...
    safeColumn(T,string(prefix)+'Z')];
end

function unit = normalizeRows(x,threshold)
magnitude = vecnorm(x,2,2); unit = nan(size(x));
valid = all(isfinite(x),2)&isfinite(magnitude)&magnitude>threshold;
unit(valid,:) = x(valid,:)./magnitude(valid);
end

function color = curveColor(base,method,record_count)
if method == 4
    % Velocity correction OFF + f_c: fixed light purple for easy comparison.
    color = [0.72 0.52 0.82];
    return;
end
if record_count == 1
    % With one CSV, distinguish curve meanings using clearly different hues.
    palette = lines(7);
    color = palette(method,:);
else
    % With overlays, preserve each CSV's identity and vary its shade by method.
    mix = [0.00 0.18 0.35 0.52 0.68];
    color = (1-mix(method))*base+mix(method)*[0.15 0.15 0.15];
end
end

function printOfflineMetricTable(records,metricXLim)
methodLabels={ ...
    'M1: Raw Force + Velocity Projection', ...
    'M2: Thrust-Corrected Force + Velocity Projection', ...
    'M3: Reconstructed Force', ...
    'M4: Proposed (Full Method)'};
forceEvidence={'Raw force f_l_hat','Thrust-corrected force f_l_bar', ...
    'Reconstructed force f_c_hat','Reconstructed force f_c_hat'};
velocityCorrection={'ON','ON','OFF','ON'};
overallSquared=zeros(1,4); overallCount=zeros(1,4);
windSquared=zeros(1,4); windCount=zeros(1,4);
staticSquared=zeros(1,4); staticCount=zeros(1,4);
for fileIndex=1:numel(records)
    r=records(fileIndex);
    normalSet=cat(3,r.n_velocity_on_fraw,r.n_velocity_on_fbar, ...
        r.n_velocity_off_fc,r.n_velocity_on_fc);
    overallMask=isfinite(r.t);
    if ~isempty(metricXLim)
        overallMask=overallMask&r.t>=metricXLim(1)&r.t<=metricXLim(2);
    end
    intervals=planeSequenceIntervals(r);
    planeTwo=find([intervals.plane]==2);
    windMask=false(size(r.t));
    if numel(planeTwo)>=2
        windInterval=intervals(planeTwo(2));
        windMask=r.t>=windInterval.start&r.t<=windInterval.stop;
    end
    for method=1:4
        errorDeg=angleBetweenRowsDeg(normalSet(:,:,method),r.true_normal);
        overallValid=overallMask&isfinite(errorDeg);
        windValid=overallMask&windMask&isfinite(errorDeg);
        staticValid=overallMask&~windMask&isfinite(errorDeg);
        overallSquared(method)=overallSquared(method)+sum(errorDeg(overallValid).^2);
        overallCount(method)=overallCount(method)+sum(overallValid);
        windSquared(method)=windSquared(method)+sum(errorDeg(windValid).^2);
        windCount(method)=windCount(method)+sum(windValid);
        staticSquared(method)=staticSquared(method)+sum(errorDeg(staticValid).^2);
        staticCount(method)=staticCount(method)+sum(staticValid);
    end
end
overallRmse=sqrt(overallSquared./max(overallCount,1));
windRmse=sqrt(windSquared./max(windCount,1));
staticRmse=sqrt(staticSquared./max(staticCount,1));
fprintf('\nOffline normal-estimator definitions\n');
fprintf('%-6s %-34s %-20s\n','Mode','Force Evidence','Velocity Correction');
fprintf('%s\n',repmat('-',1,66));
for method=1:4
    fprintf('%-6s %-34s %-20s\n',sprintf('M%d',method), ...
        forceEvidence{method},velocityCorrection{method});
end
fprintf('\n%-52s %18s %18s %20s\n','Method','Overall RMSE [deg]', ...
    'Wind RMSE [deg]','Static RMSE [deg]');
fprintf('%s\n',repmat('-',1,112));
for method=1:4
    fprintf('%-52s %18.3f %18.3f %20.3f\n',methodLabels{method}, ...
        overallRmse(method),windRmse(method),staticRmse(method));
end
fprintf('\n');
end

function plotFirstExperimentKeyDisplays(records,datasetLabels,recordColors,globalXLim,cfg, ...
    windowOrigin,windowWidth,subplotRowHeight)
windowGap=[20 45];
summaryHeight=5*subplotRowHeight;
visualHeight=3*subplotRowHeight;
x=windowOrigin(1); y=windowOrigin(2);
summaryFig=figure('Name','First experiment: traversal and normal estimation', ...
    'Color','w','Position',[x y windowWidth summaryHeight]);
ascendingFig=figure('Name','First experiment: ascending traversal', ...
    'Color','w','Position',[x+windowWidth+windowGap(1), ...
    y+visualHeight+windowGap(2),windowWidth,visualHeight]);
descendingFig=figure('Name','First experiment: descending traversal', ...
    'Color','w','Position',[x+2*(windowWidth+windowGap(1)), ...
    y+visualHeight+windowGap(2),windowWidth,visualHeight]);
barFig=figure('Name','First experiment: plane-2 comparison', ...
    'Color','w','Position',[x+windowWidth+windowGap(1),y,windowWidth,visualHeight]);
leftPanel=uipanel(summaryFig,'Units','normalized','Position',[0 0 1 1], ...
    'BackgroundColor','w','BorderType','none');
trajPanels=gobjects(2,1);
trajPanels(1)=uipanel(ascendingFig,'Units','normalized','Position',[0 0 1 1], ...
    'BackgroundColor','w','BorderType','none');
trajPanels(2)=uipanel(descendingFig,'Units','normalized','Position',[0 0 1 1], ...
    'BackgroundColor','w','BorderType','none');
barPanel=uipanel(barFig,'Units','normalized','Position',[0 0 1 1], ...
    'BackgroundColor','w','BorderType','none');

% Left: speed and the four offline normal-estimator metrics.
layout=tiledlayout(leftPanel,5,1,'TileSpacing','compact','Padding','compact');
ax=gobjects(5,1);
for row=1:5, ax(row)=nexttile(layout); hold(ax(row),'on'); end
metricLabels={'m1','m2','m3','m4'};
metricStyles={':','--','-','-.'};
for fileIndex=1:numel(records)
    r=records(fileIndex); c=recordColors(fileIndex,:);
    plot(ax(1),r.t,r.speed_filtered,'Color',c,'LineWidth',1.7, ...
        'DisplayName',legendLabel(datasetLabels{fileIndex},'measured'));
    normalSet=cat(3,r.n_velocity_on_fraw,r.n_velocity_on_fbar, ...
        r.n_velocity_off_fc,r.n_velocity_on_fc);
    for component=1:3
        plot(ax(component+1),r.t,r.true_normal(:,component),'--', ...
            'Color',[0.25 0.25 0.25],'LineWidth',1.6,'HandleVisibility','off');
        for method=1:4
            plot(ax(component+1),r.t,normalSet(:,component,method), ...
                'Color',curveColor(c,method,numel(records)), ...
                'LineStyle',metricStyles{method},'LineWidth',1.7, ...
                'DisplayName',legendLabel(datasetLabels{fileIndex},metricLabels{method}));
        end
    end
    for method=1:4
        errorDeg=angleBetweenRowsDeg(normalSet(:,:,method),r.true_normal);
        plot(ax(5),r.t,errorDeg,'Color',curveColor(c,method,numel(records)), ...
            'LineStyle',metricStyles{method},'LineWidth',1.7, ...
            'DisplayName',legendLabel(datasetLabels{fileIndex},metricLabels{method}));
    end
end
ylabel(ax(1),'|v| [m/s]'); title(ax(1),'Traversal Velocity Norm');
ylabel(ax(2),'n_x [-]'); ylabel(ax(3),'n_y [-]'); ylabel(ax(4),'n_z [-]');
title(ax(2),'Normal Vector Estimation');
ylabel(ax(5),'e_n [deg]'); title(ax(5),'Normal Estimation Error Angle');
ylim(ax(1),[0 0.15]);
ylim(ax(2:4),[-1.1 1.1]); ylim(ax(5),[0 60]);
for row=1:5, grid(ax(row),'on'); box(ax(row),'on'); end
legend(ax(1),'Location','best','Interpreter','none');
legend(ax(2),'Location','best','Interpreter','none');
addPlaneSequenceShading(ax,records(1));
finishOverviewAxes(ax,globalXLim);

% Right: the known surface, EE trajectory, and only f-bar/f-hat normals.
trajAx=gobjects(2,1);
surfaceU=linspace(0,1,501).';
[surfaceXZ,~]=fiveSectionGeometry(surfaceU,cfg);
surfaceY=cfg.origin(2)+linspace(-0.20,0.20,31);
surfaceX=repmat(surfaceXZ(:,1),1,numel(surfaceY));
surfaceZ=repmat(surfaceXZ(:,2),1,numel(surfaceY));
surfaceYMesh=repmat(surfaceY,size(surfaceXZ,1),1);
arrowPathPeriod=0.08; arrowLength=0.08; % [m] path spacing and arrow length
normalColors=[0.18 0.62 0.32; 0.12 0.38 0.82];
normalLabels={'$m_2$: $\bar{f}_{\ell}$ + correction', ...
    '$m_3$: $\hat{f}_{c}$, no correction'};
for direction=1:2
    trajAx(direction)=axes(trajPanels(direction)); hold(trajAx(direction),'on');
    surf(trajAx(direction),surfaceX,surfaceYMesh,surfaceZ, ...
        'FaceColor',[0.45 0.86 0.82],'FaceAlpha',0.34, ...
        'EdgeColor',[0.60 0.82 0.86],'EdgeAlpha',0.10, ...
        'DisplayName','reference surface');
    for fileIndex=1:numel(records)
        r=records(fileIndex);
        loopMask=isfinite(r.t)&r.t>=r.surface_loop_time(1)& ...
            r.t<=r.surface_loop_time(2)&all(isfinite(r.ee_pos),2)& ...
            isfinite(r.surface_arc);
        loopIndices=find(loopMask);
        if isempty(loopIndices), continue; end
        [~,turnLocal]=max(r.surface_arc(loopIndices));
        turnIndex=loopIndices(turnLocal);
        if direction==1
            selected=loopMask&(1:numel(r.t)).'<=turnIndex;
        else
            selected=loopMask&(1:numel(r.t)).'>=turnIndex;
        end
        if ~isempty(globalXLim), selected=selected&timeMask(r.t,globalXLim); end
        plot3(trajAx(direction),r.ee_pos(selected,1),r.ee_pos(selected,2), ...
            r.ee_pos(selected,3),'Color',[0.42 0.42 0.42],'LineWidth',2.5, ...
            'DisplayName',legendLabel(datasetLabels{fileIndex},'EE trajectory'));
        normalData={r.n_velocity_on_fbar,r.n_velocity_off_fc}; % m2 and m3 only
        for method=1:2
            normalMethod=normalData{method};
            valid=selected&all(isfinite(normalMethod),2);
            arrows=distanceSpacedIndices(r.ee_pos,valid,arrowPathPeriod, ...
                (method-1)*0.5*arrowPathPeriod);
            quiver3(trajAx(direction),r.ee_pos(arrows,1),r.ee_pos(arrows,2), ...
                r.ee_pos(arrows,3),arrowLength*normalMethod(arrows,1), ...
                arrowLength*normalMethod(arrows,2),arrowLength*normalMethod(arrows,3),0, ...
                'Color',normalColors(method,:),'LineWidth',1.7,'MaxHeadSize',0.8, ...
                'DisplayName',legendLabel(datasetLabels{fileIndex},normalLabels{method}));
        end
    end
    grid(trajAx(direction),'on'); box(trajAx(direction),'on');
    axis(trajAx(direction),'equal'); axis(trajAx(direction),'vis3d');
    view(trajAx(direction),[1 -0.5 -0.5]);
    xlim(trajAx(direction),[min(surfaceXZ(:,1))-0.12 max(surfaceXZ(:,1))+0.12]);
    ylim(trajAx(direction),[min(surfaceY)-0.08 max(surfaceY)+0.08]);
    zlim(trajAx(direction),[min(surfaceXZ(:,2))-0.12 max(surfaceXZ(:,2))+0.12]);
    xlabel(trajAx(direction),'x [m]'); ylabel(trajAx(direction),'y [m]');
    zlabel(trajAx(direction),'z [m]');
end
title(trajAx(1),'Ascending Traversal'); title(trajAx(2),'Descending Traversal');
legend(trajAx(1),'Location','best','Interpreter','latex');
plotPlaneTwoErrorBars(records,barPanel);
end

function plotPlaneTwoErrorBars(records,parentPanel)
% Compare direction-error RMS on the first (no-wind) and second (wind)
% visits to plane 2.
values=nan(numel(records),2,3);
for fileIndex=1:numel(records)
    r=records(fileIndex); intervals=planeSequenceIntervals(r);
    planeTwo=find([intervals.plane]==2);
    if numel(planeTwo)<2, continue; end
    unitSet=cat(3,r.f_raw_unit,r.f_bar_unit,r.f_c_unit);
    for condition=1:2
        interval=intervals(planeTwo(condition));
        selected=r.t>=interval.start&r.t<=interval.stop;
        for method=1:3
            errorDeg=angleBetweenRowsDeg(unitSet(:,:,method),r.true_normal);
            values(fileIndex,condition,method)=sqrt(mean(errorDeg(selected).^2,'omitnan'));
        end
    end
end
summary=squeeze(mean(values,1,'omitnan'));
ax=axes(parentPanel); hold(ax,'on');
b=bar(ax,summary,'grouped');
barColors=[0.25 0.56 0.92; 0.34 0.72 0.45; 0.94 0.45 0.24];
for method=1:3, b(method).FaceColor=barColors(method,:); end
grid(ax,'on'); box(ax,'on');
xticks(ax,1:2); xticklabels(ax,{'No Wind','Wind'}); xtickangle(ax,0);
xlim(ax,[0.5 2.5]);
ylabel(ax,'Angular-error RMS [deg]'); title(ax,'Plane 2: Force-Direction Error');
legend(ax,{'Raw MOB','Thrust Corrected','Force Reconstructed'}, ...
    'Location','southoutside','Orientation','horizontal');
end

function addPlaneSequenceShading(ax,record)
intervals=planeSequenceIntervals(record);
if isempty(intervals), return; end
windPlaneTwoSeen=0;
for intervalIndex=1:numel(intervals)
    interval=intervals(intervalIndex);
    if interval.plane==2, windPlaneTwoSeen=windPlaneTwoSeen+1; end
    isWind=interval.plane==2&&windPlaneTwoSeen==2;
    if isWind, color=[0.96 0.68 0.48]; alpha=0.22;
    else, color=[0.68 0.76 0.88]; alpha=0.14; end
    for axisIndex=1:numel(ax)
        yLimits=ylim(ax(axisIndex)); holdWasOn=ishold(ax(axisIndex)); hold(ax(axisIndex),'on');
        p=patch(ax(axisIndex),[interval.start interval.stop interval.stop interval.start], ...
            [yLimits(1) yLimits(1) yLimits(2) yLimits(2)],color, ...
            'FaceAlpha',alpha,'EdgeColor','none','HandleVisibility','off');
        uistack(p,'bottom');
        if ~holdWasOn, hold(ax(axisIndex),'off'); end
    end
end
end

function intervals=planeSequenceIntervals(record)
% Extract the five planar visits 1-2-3-2-1 from the fitted closed sweep.
intervals=struct('plane',{},'start',{},'stop',{});
section=record.surface_section;
plane=nan(size(section));
plane(section==1)=1; plane(section==3)=2; plane(section==5)=3;
inside=isfinite(record.t)&record.t>=record.surface_loop_time(1)& ...
    record.t<=record.surface_loop_time(2)&isfinite(plane);
starts=find(inside&[true;~inside(1:end-1)|plane(1:end-1)~=plane(2:end)]);
stops=find(inside&[~inside(2:end)|plane(1:end-1)~=plane(2:end);true]);
for run=1:min(numel(starts),numel(stops))
    intervals(end+1)=struct('plane',plane(starts(run)), ... %#ok<AGROW>
        'start',record.t(starts(run)),'stop',record.t(stops(run)));
end
expected=[1 2 3 2 1]; selected=zeros(1,0); cursor=1;
for k=1:numel(intervals)
    if cursor<=numel(expected)&&intervals(k).plane==expected(cursor)
        selected(end+1)=k; cursor=cursor+1; %#ok<AGROW>
    end
end
if numel(selected)==numel(expected)
    intervals=intervals(selected);
    % Include the wind onset and decay around the return visit to plane 2.
    windPlaneTwo=find([intervals.plane]==2,1,'last');
    intervals(windPlaneTwo).start=max(record.surface_loop_time(1), ...
        intervals(windPlaneTwo).start-2.0);
    intervals(windPlaneTwo).stop=min(record.surface_loop_time(2), ...
        intervals(windPlaneTwo).stop+1.0);
end
end

function plotOverviewPanel(records,datasetLabels,recordColors,methodNames, ...
    methodStyles,methodWidths,axisNames,globalXLim,cfg,figurePosition)
overviewFigure=figure('Name','First experiment overview','Color','w', ...
    'Position',figurePosition);
% Allocate vertical space in a 3:4 ratio so individual axes in the upper
% three-row and lower four-row panels have approximately equal heights.
panelPositions=[0.015 0.585 0.477 0.405; ... % top left
                0.015 0.015 0.477 0.545; ... % bottom left
                0.508 0.585 0.477 0.405; ... % top right
                0.508 0.015 0.477 0.545];    % bottom right
overviewPanels=gobjects(4,1);
for panelIndex=1:4
    overviewPanels(panelIndex)=uipanel(overviewFigure,'Units','normalized', ...
        'Position',panelPositions(panelIndex,:),'BackgroundColor','w', ...
        'BorderType','line','BorderWidth',1,'HighlightColor',[0.55 0.55 0.55]);
end

% Top left (quadrant II): X-Z surface/trajectory views split at 30 s.
layout=tiledlayout(overviewPanels(1),1,2,'TileSpacing','compact','Padding','compact');
if isempty(globalXLim)
    finiteTimes=[];
    for fileIndex=1:numel(records)
        finiteTimes=[finiteTimes; records(fileIndex).t(isfinite(records(fileIndex).t))]; %#ok<AGROW>
    end
    overviewEnd=max(finiteTimes);
else
    overviewEnd=globalXLim(2);
end
overviewWindows={[0,min(30,overviewEnd)],[30,overviewEnd]};
surfaceU=linspace(0,1,501).';
[surfaceXZ,~]=fiveSectionGeometry(surfaceU,cfg);
trajectoryMap=turbo(256);
overviewArrowPeriod=1.5; % [s]
overviewArrowLength=0.08; % [m]
for panelIndex=1:2
    ax=nexttile(layout); hold(ax,'on');
    plot(ax,surfaceXZ(:,1),surfaceXZ(:,2),'Color',[0.66 0.80 0.81], ...
        'LineWidth',7.0,'DisplayName','reference surface');
    timeWindow=overviewWindows{panelIndex};
    for fileIndex=1:numel(records)
        selected=timeMask(records(fileIndex).t,timeWindow) & ...
            all(isfinite(records(fileIndex).ee_pos),2);
        plotTimeColoredXZ(ax,records(fileIndex).ee_pos(selected,1), ...
            records(fileIndex).ee_pos(selected,3),records(fileIndex).t(selected), ...
            timeWindow,trajectoryMap);
        normalData=records(fileIndex).n_velocity_on_fc;
        arrows=timeSpacedIndices(records(fileIndex).t,selected & ...
            all(isfinite(normalData),2),overviewArrowPeriod);
        quiver(ax,records(fileIndex).ee_pos(arrows,1),records(fileIndex).ee_pos(arrows,3), ...
            overviewArrowLength*normalData(arrows,1), ...
            overviewArrowLength*normalData(arrows,3),0, ...
            'Color',[0.12 0.12 0.12],'LineWidth',1.4,'MaxHeadSize',0.8, ...
            'DisplayName','estimated normal');
    end
    colormap(ax,trajectoryMap); caxis(ax,timeWindow);
    cb=colorbar(ax); cb.Label.String='Time [s]';
    grid(ax,'on'); box(ax,'on'); axis(ax,'equal');
    view(ax,2);
    xlabel(ax,'x [m]'); ylabel(ax,'z [m]');
    title(ax,sprintf('X-Z surface and EE: %.0f-%.0f s',timeWindow));
end

% Bottom left: estimated normal and angular error.
layout=tiledlayout(overviewPanels(2),4,1,'TileSpacing','compact','Padding','compact');
ax=gobjects(4,1);
normalYLim={[-1.1 1.1],[-1.1 1.1],[-1.1 1.1],[0 90]};
for component=1:3
    ax(component)=nexttile(layout); hold(ax(component),'on');
    for fileIndex=1:numel(records)
        plot(ax(component),records(fileIndex).t,records(fileIndex).true_normal(:,component), ...
            'Color',[0.76 0.76 0.76],'LineWidth',3.0,'DisplayName', ...
            legendLabel(datasetLabels{fileIndex},'true normal'));
        normalSet=cat(3,records(fileIndex).n_velocity_on_fraw, ...
            records(fileIndex).n_velocity_on_fbar,records(fileIndex).n_velocity_off_fc, ...
            records(fileIndex).n_velocity_on_fc);
        for method=1:numel(methodNames)
            plot(ax(component),records(fileIndex).t,normalSet(:,component,method), ...
                'Color',curveColor(recordColors(fileIndex,:),method,numel(records)), ...
                'LineStyle',methodStyles{method},'LineWidth',methodWidths(method), ...
                'DisplayName',legendLabel(datasetLabels{fileIndex},methodNames{method}));
        end
    end
    grid(ax(component),'on'); box(ax(component),'on');
    ylabel(ax(component),sprintf('n_%s [-]',axisNames{component}));
    applyConfiguredYLim(ax(component),normalYLim{component});
end
title(ax(1),'Normal estimation'); legend(ax(1),'Location','best','Interpreter','latex');
ax(4)=nexttile(layout); hold(ax(4),'on');
for fileIndex=1:numel(records)
    normalSet=cat(3,records(fileIndex).n_velocity_on_fraw, ...
        records(fileIndex).n_velocity_on_fbar,records(fileIndex).n_velocity_off_fc, ...
        records(fileIndex).n_velocity_on_fc);
    for method=1:numel(methodNames)
        errorDeg=angleBetweenRowsDeg(normalSet(:,:,method),records(fileIndex).true_normal);
        plot(ax(4),records(fileIndex).t,errorDeg,'Color', ...
            curveColor(recordColors(fileIndex,:),method,numel(records)), ...
            'LineStyle',methodStyles{method},'LineWidth',methodWidths(method));
    end
end
grid(ax(4),'on'); box(ax(4),'on'); ylabel(ax(4),'angle [deg]');
title(ax(4),'Error angle');
applyConfiguredYLim(ax(4),normalYLim{4});
addPlaneSequenceShading(ax,records(1)); finishOverviewAxes(ax,globalXLim);

% Top right: force estimation.
layout=tiledlayout(overviewPanels(3),3,1,'TileSpacing','compact','Padding','compact');
ax=gobjects(3,1);
forceNames={'$\hat{f}_{\ell}$','$\bar{f}_{\ell}$','$\hat{f}_{c}$'};
forceStyles={':','--','-'};
for component=1:3
    ax(component)=nexttile(layout); hold(ax(component),'on');
    for fileIndex=1:numel(records)
        forceSet=cat(3,records(fileIndex).f_raw,records(fileIndex).f_bar,records(fileIndex).f_c);
        for method=1:3
            plot(ax(component),records(fileIndex).t,1e3*forceSet(:,component,method), ...
                'Color',curveColor(recordColors(fileIndex,:),method,numel(records)), ...
                'LineStyle',forceStyles{method},'LineWidth',1.5,'DisplayName', ...
                legendLabel(datasetLabels{fileIndex},forceNames{method}));
        end
    end
    grid(ax(component),'on'); box(ax(component),'on');
    ylabel(ax(component),sprintf('f_%s [mN]',axisNames{component})); ylim(ax(component),[-60 60]);
end
title(ax(1),'Force estimation'); legend(ax(1),'Location','best','Interpreter','latex');
addPlaneSequenceShading(ax,records(1));
finishOverviewAxes(ax,globalXLim);

% Bottom right: normalized force direction and angular error.
layout=tiledlayout(overviewPanels(4),4,1,'TileSpacing','compact','Padding','compact');
ax=gobjects(4,1);
for component=1:3
    ax(component)=nexttile(layout); hold(ax(component),'on');
    for fileIndex=1:numel(records)
        plot(ax(component),records(fileIndex).t,records(fileIndex).true_normal(:,component), ...
            'Color',[0.76 0.76 0.76],'LineWidth',3.0,'DisplayName', ...
            legendLabel(datasetLabels{fileIndex},'true normal'));
        forceSet=cat(3,records(fileIndex).f_raw_unit,records(fileIndex).f_bar_unit, ...
            records(fileIndex).f_c_unit);
        for method=1:3
            plot(ax(component),records(fileIndex).t,forceSet(:,component,method), ...
                'Color',curveColor(recordColors(fileIndex,:),method,numel(records)), ...
                'LineStyle',forceStyles{method},'LineWidth',1.5,'DisplayName', ...
                legendLabel(datasetLabels{fileIndex},forceNames{method}));
        end
    end
    grid(ax(component),'on'); box(ax(component),'on');
    ylabel(ax(component),sprintf('%s [-]',axisNames{component})); ylim(ax(component),[-1.1 1.1]);
end
title(ax(1),'Force direction vs. true normal');
legend(ax(1),'Location','best','Interpreter','latex');
ax(4)=nexttile(layout); hold(ax(4),'on');
for fileIndex=1:numel(records)
    forceSet=cat(3,records(fileIndex).f_raw_unit,records(fileIndex).f_bar_unit, ...
        records(fileIndex).f_c_unit);
    for method=1:3
        errorDeg=angleBetweenRowsDeg(forceSet(:,:,method),records(fileIndex).true_normal);
        plot(ax(4),records(fileIndex).t,errorDeg,'Color', ...
            curveColor(recordColors(fileIndex,:),method,numel(records)), ...
            'LineStyle',forceStyles{method},'LineWidth',1.5);
    end
end
grid(ax(4),'on'); box(ax(4),'on'); ylabel(ax(4),'angle [deg]');
title(ax(4),'Error angle'); ylim(ax(4),[0 90]);
addPlaneSequenceShading(ax,records(1));
finishOverviewAxes(ax,globalXLim);
end

function plotTimeColoredXZ(ax,x,z,t,timeWindow,colorMap)
% Draw a continuous trajectory whose edge color is interpolated from time.
if numel(t)<2, return; end
surface(ax,[x(:) x(:)],[z(:) z(:)],zeros(numel(t),2),[t(:) t(:)], ...
    'FaceColor','none','EdgeColor','interp','LineWidth',3.6, ...
    'HandleVisibility','off');
colormap(ax,colorMap); caxis(ax,timeWindow);
end

function plotEnsembleErrorStatistics(records,normalMethodNames,timeLimits, ...
    methodIndices,comparisonLabels)
% Time-resolved mean +/- SD and per-run RMSE comparison.
if isempty(timeLimits)
    allTimes=vertcat(records.t); allTimes=allTimes(isfinite(allTimes));
    timeLimits=[min(allTimes) max(allTimes)];
end
validateattributes(timeLimits,{'numeric'},{'vector','numel',2,'finite','increasing'}, ...
    mfilename,'ensemble_error_xlim');

samplePeriods=[];
for fileIndex=1:numel(records)
    dt=diff(records(fileIndex).t);
    samplePeriods=[samplePeriods; dt(isfinite(dt)&dt>0)]; %#ok<AGROW>
end
gridStep=median(samplePeriods);
if isempty(gridStep)||~isfinite(gridStep), gridStep=0.01; end
gridStep=max(gridStep,0.01);
timeGrid=(timeLimits(1):gridStep:timeLimits(2)).';
if timeGrid(end)<timeLimits(2), timeGrid(end+1,1)=timeLimits(2); end

normalErrors=nan(numel(timeGrid),numel(records),numel(normalMethodNames));
forceErrors=nan(numel(timeGrid),numel(records),4);
for fileIndex=1:numel(records)
    normalSet=cat(3,records(fileIndex).n_velocity_on_fraw, ...
        records(fileIndex).n_velocity_on_fbar,records(fileIndex).n_velocity_off_fc, ...
        records(fileIndex).n_velocity_on_fc);
    for method=1:size(normalSet,3)
        errorDeg=angleBetweenRowsDeg(normalSet(:,:,method),records(fileIndex).true_normal);
        normalErrors(:,fileIndex,method)=interpolateFiniteSeries( ...
            records(fileIndex).t,errorDeg,timeGrid);
    end
    % Velocity correction changes the normal estimator, not the force vector;
    % therefore the first two force-direction series intentionally coincide.
    forceSet=cat(3,records(fileIndex).f_raw_unit,records(fileIndex).f_bar_unit, ...
        records(fileIndex).f_c_unit,records(fileIndex).f_c_unit);
    for method=1:size(forceSet,3)
        errorDeg=angleBetweenRowsDeg(forceSet(:,:,method),records(fileIndex).true_normal);
        forceErrors(:,fileIndex,method)=interpolateFiniteSeries( ...
            records(fileIndex).t,errorDeg,timeGrid);
    end
end

validateattributes(methodIndices,{'numeric'}, ...
    {'vector','integer','positive','<=',size(normalErrors,3)});
if numel(methodIndices)~=numel(comparisonLabels)
    error('comparison_method_indices and comparison_method_labels must have equal lengths.');
end
normalErrors=normalErrors(:,:,methodIndices);
forceErrors=forceErrors(:,:,methodIndices);
allMethodColors=[0.47 0.67 0.19; 0.85 0.33 0.10; ...
    0.00 0.45 0.74; 0.72 0.52 0.82];
methodColors=allMethodColors(methodIndices,:);

normalRmseByRun=nan(numel(records),numel(methodIndices));
forceRmseByRun=nan(numel(records),numel(methodIndices));
for method=1:numel(methodIndices)
    for fileIndex=1:numel(records)
        values=normalErrors(:,fileIndex,method);
        normalRmseByRun(fileIndex,method)=sqrt(mean(values.^2,'omitnan'));
        values=forceErrors(:,fileIndex,method);
        forceRmseByRun(fileIndex,method)=sqrt(mean(values.^2,'omitnan'));
    end
end

figure('Name','Repeated-experiment estimation performance','Color','w', ...
    'Position',[110 80 1480 900]);
layout=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
ax=nexttile(layout,1);
forceDisplayMask=methodIndices~=4; % omit duplicate f_c without epsilon_v
plotErrorMeanAndSpread(ax,timeGrid,forceErrors(:,:,forceDisplayMask), ...
    comparisonLabels(forceDisplayMask),methodColors(forceDisplayMask,:));
title(ax,sprintf('Force-direction Error Time Series (n = %d)',numel(records)));
ylabel(ax,'Force-direction Error [deg]'); ylim(ax,[0 90]);
ax=nexttile(layout,2);
plotErrorMeanAndSpread(ax,timeGrid,normalErrors,comparisonLabels,methodColors);
title(ax,sprintf('Normal Estimation Error Time Series (n = %d)',numel(records)));
ylabel(ax,'Normal Estimation Error [deg]'); ylim(ax,[0 90]);
ax=nexttile(layout,3);
plotRmseComparison(ax,forceRmseByRun(:,forceDisplayMask), ...
    comparisonLabels(forceDisplayMask));
title(ax,'Quantitative Force-direction Performance'); ylim(ax,[0 55]);
ax=nexttile(layout,4);
plotRmseComparison(ax,normalRmseByRun,comparisonLabels);
title(ax,'Quantitative Normal-estimation Performance'); ylim(ax,[0 55]);
end

function valuesOnGrid=interpolateFiniteSeries(t,values,timeGrid)
valid=isfinite(t)&isfinite(values);
tValid=t(valid); valuesValid=values(valid);
[tValid,uniqueIndex]=unique(tValid,'stable'); valuesValid=valuesValid(uniqueIndex);
if numel(tValid)<2
    valuesOnGrid=nan(size(timeGrid));
else
    valuesOnGrid=interp1(tValid,valuesValid,timeGrid,'linear',nan);
end
end

function plotErrorMeanAndSpread(ax,timeGrid,errorCube,methodNames,methodColors)
hold(ax,'on');
for method=1:size(errorCube,3)
    acrossLogs=errorCube(:,:,method);
    meanError=mean(acrossLogs,2,'omitnan');
    varianceError=var(acrossLogs,0,2,'omitnan');
    sigma=sqrt(varianceError);
    valid=isfinite(meanError)&isfinite(sigma);
    lightColor=0.72*[1 1 1]+0.28*methodColors(method,:);
    fill(ax,[timeGrid(valid);flipud(timeGrid(valid))], ...
        [meanError(valid)-sigma(valid);flipud(meanError(valid)+sigma(valid))], ...
        lightColor,'EdgeColor','none','FaceAlpha',0.42,'HandleVisibility','off');
    plot(ax,timeGrid,meanError,'Color',methodColors(method,:), ...
        'LineWidth',2.2,'DisplayName',methodNames{method});
end
grid(ax,'on'); box(ax,'on'); xlim(ax,[timeGrid(1) timeGrid(end)]);
xlabel(ax,'Time [s]'); legend(ax,'Location','best','Interpreter','latex');
end

function plotRmseComparison(ax,rmseByRun,methodLabels)
methodCount=size(rmseByRun,2);
meanRmse=mean(rmseByRun,1,'omitnan');
sdRmse=std(rmseByRun,0,1,'omitnan');
hold(ax,'on');
errorbar(ax,1:methodCount,meanRmse,sdRmse,'ko','LineStyle','none', ...
    'MarkerFaceColor','k','MarkerSize',7,'LineWidth',1.8,'CapSize',14);
xticks(ax,1:methodCount); xticklabels(ax,methodLabels);
set(ax,'TickLabelInterpreter','latex');
xlabel(ax,'Method'); ylabel(ax,'RMSE [deg]');
grid(ax,'on'); box(ax,'on'); xlim(ax,[0.5 methodCount+0.5]);
end

function finishOverviewAxes(ax,globalXLim)
xlabel(ax(end),'Time [s]');
set(ax(1:end-1),'XTickLabel',[]);
linkaxes(ax,'x');
applyGlobalXLim(ax,globalXLim);
end

function applyGlobalXLim(axes_handles,limits)
if isempty(limits), return; end
validateattributes(limits,{'numeric'},{'vector','numel',2,'finite','increasing'}, ...
    mfilename,'global_xlim');
set(axes_handles,'XLim',limits);
end

function applyConfiguredYLim(axis_handle,limits)
if isempty(limits), return; end
validateattributes(limits,{'numeric'},{'vector','numel',2,'finite','increasing'}, ...
    mfilename,'section_ylim');
ylim(axis_handle,limits);
end

function angle_deg = angleBetweenRowsDeg(a,b)
% Included angle acos((a dot b)/(|a||b|)), reported in degrees.
a_norm = vecnorm(a,2,2);
b_norm = vecnorm(b,2,2);
cosine = sum(a.*b,2)./(a_norm.*b_norm);
valid = all(isfinite(a),2)&all(isfinite(b),2)&a_norm>1e-12&b_norm>1e-12;
cosine = min(1,max(-1,cosine));
angle_deg = nan(size(cosine));
angle_deg(valid) = acosd(cosine(valid));
end

function label = extractWindLabel(file_label)
token = regexp(file_label,'(?i)wind(?:X|90|45|0)','match','once');
if isempty(token)
    label = file_label;
else
    suffix = extractAfter(string(token),4);
    label = char("wind"+suffix);
end
end

function label = legendLabel(dataset_label,curve_label)
if isempty(dataset_label)
    label = curve_label;
else
    label = sprintf('%s | %s',dataset_label,curve_label);
end
end

function selected = timeMask(t,time_window)
selected = isfinite(t)&t>=time_window(1)&t<=time_window(2);
end

function indices=distanceSpacedIndices(position,valid,period,offset)
% Select samples on an absolute, evenly spaced cumulative-path grid.
candidates=find(valid(:)&all(isfinite(position),2));
indices=zeros(0,1);
if numel(candidates)<2||~isfinite(period)||period<=0, return; end
path=[0;cumsum(vecnorm(diff(position(candidates,:),1,1),2,2))];
firstTarget=ceil((path(1)-offset)/period)*period+offset;
lastTarget=floor((path(end)-offset)/period)*period+offset;
targets=(firstTarget:period:lastTarget).';
for targetIndex=1:numel(targets)
    [~,nearest]=min(abs(path-targets(targetIndex)));
    indices(end+1,1)=candidates(nearest); %#ok<AGROW>
end
indices=unique(indices,'stable');
end

function indices = timeSpacedIndices(t,valid,period)
candidates = find(valid(:)&isfinite(t(:)));
indices = zeros(0,1);
next_time = -inf;
for candidate_number = 1:numel(candidates)
    candidate = candidates(candidate_number);
    if t(candidate) >= next_time
        indices(end+1,1) = candidate; %#ok<AGROW>
        next_time = t(candidate)+period;
    end
end
end

function folder=activeScriptDirectory()
folder='';
stack=dbstack('-completenames');
for k=1:numel(stack)
    if endsWith(string(stack(k).file),'data_1st_exp_overlay.m')
        folder=fileparts(stack(k).file); break;
    end
end
try
    activeFile=matlab.desktop.editor.getActiveFilename;
    if isempty(folder)&&~isempty(activeFile)&&endsWith(string(activeFile),'.m','IgnoreCase',true)
        folder=fileparts(activeFile);
    end
catch
end
if isempty(folder), folder=fileparts(mfilename('fullpath')); end
if isempty(folder), folder=pwd; end
end
