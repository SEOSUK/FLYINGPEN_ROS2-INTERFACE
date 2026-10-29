%% CSV overlay normal/force analysis: per-trial crop AFTER full-history estimation
% Use ORIGINAL, UNCROPPED CSVs. No source CSV is modified or overwritten.
% Every offline estimator and geometry fit runs on the complete log first.
% records_full retains those results; records contains the analysis windows.
clear; close all; clc;

%% USER SETTINGS: CSV folder
% Leave empty to look beside the saved script, then in MATLAB Current Folder.
% If neither contains CSVs, a folder chooser opens instead of using Editor_*.
% Set an existing absolute path here to override automatic folder selection.
% Windows example: csv_data_dir = 'D:\FlyingPen\experiment1';
% Linux example:   csv_data_dir = '/home/your_name/FlyingPen/experiment1';
csv_data_dir = '/home/seosuk/hitl_ws/src/flying_pen/bag/1023_1st_exp/selected_and_crop';

%% USER SETTINGS: timing for 1st, 2nd, ..., 11th
% Times below use t_elapsed = CSV.t_sec - first finite CSV.t_sec [s].
% They are NOT wall-clock times, row numbers, or times after a previous crop.
% Each default is first positive u2_des onset minus 5 seconds.
% The trial number comes from the FILENAME PREFIX, not selection order.
thrust_data = "comp"; % "raw": 0.920*PWM, "comp": CF21BL voltage/thrust model

time_crop_start = [ ...
    27.080328, ... %  1st: 20261023_063311
    25.340201, ... %  2nd: 20261023_063806
    28.419923, ... %  3rd: 20261023_064353
    38.940222, ... %  4th: 20261023_064733
    30.120147, ... %  5th: 20261023_065500
    30.140129, ... %  6th: 20261023_070229
    31.559994, ... %  7th: 20261023_070627
    30.400179, ... %  8th: 20261023_071016
    26.520010, ... %  9th: 20261023_071349
    25.820070, ... % 10th: 20261023_071738
    33.260162  ... % 11th: 20261023_072735 (replacement, not 072035)
];

% Optional manual endpoints, in the SAME original elapsed-time coordinates.
% NaN = first zero u2_des sample after the final motion + 10 seconds.
% A number overrides that trial's automatic endpoint only.
time_crop_end = nan(1,11);

% 'crop'    : crop start = t=0; first motion is approximately t=5 s.
% 'command' : first positive u2_des = t=0; retained baseline is [-5,0) s.
% This changes labels only, NOT the selected samples or estimator results.
time_zero_mode = 'crop';

% Explicit mapping also supports the original filenames without prefixes.
% If your prefix numbering differs, update BOTH this map and the timing list.
recording_ids = { ...
    '20261023_063311','20261023_063806','20261023_064353', ...
    '20261023_064733','20261023_065500','20261023_070229', ...
    '20261023_070627','20261023_071016','20261023_071349', ...
    '20261023_071738','20261023_072735'};
excluded_recording_ids = {'20261023_072035'};

% Empty limits use the retained data, instead of the old [0,36] s window.
% All optional display limits below refer to the NEW plotted time axis.
ensemble_error_xlim = [];
trajectory_iso_time = [];    % [] = full retained interval
trajectory_xz_time_1 = [];   % [] = before mean upper-turn time
trajectory_xz_time_2 = [];   % [] = after mean upper-turn time

%% USER SETTINGS: paper Fig. 7 / Fig. 8 / Table I
% New figures are generated BEFORE the original Overview panel.
% The original estimators, geometry fit, crop, and alignment are unchanged.
% IEEE two-column / single-column figure proportions. Pixel positions control
% the on-screen preview; export sizes below control the manuscript artwork.
paper.fig7_position = [40 35 740 980];  % initial screen location, pixels
paper.fig8_position = [790 150 520 610];
paper.fig7_size_in = [7.20 9.60];       % comfortable tall on-screen preview
paper.fig8_size_in = [5.10 5.65];       % screenshot-scale on-screen preview
paper.fig7_row_weights = [0.80 1.00 1.00 1.10 1.45]; % speed, nx, ny, nz, error
paper.font_name = 'Times New Roman';
paper.font_size = 9;                 % remains legible at final IEEE size
paper.fig7_header_font_size = 7;
paper.fig8_legend_font_size = 10;
paper.band_alpha = 0.12;
% Colorblind-safe, print-friendly palette. M4 uses purple so that it remains
% visually separate from the dashed grey reference in dense overlays.
paper.method_colors = [0.000 0.447 0.698; 0.835 0.369 0.000; ...
                       0.000 0.620 0.451; 0.494 0.184 0.556]; % M1--M4
paper.method_styles = {':','--','-.','-'};
paper.method_widths = [1.35 1.35 1.45 1.85];
paper.fig7_line_scale = 1.20;        % slightly bolder Fig. 7 lines only
paper.component_bands = [true true true]; % SD bands on nx, ny, nz
paper.error_band = true;
paper.clip_error_band_at_zero = true; % display only; stored SD is unchanged
paper.grid_step_s = [];              % [] = median CSV sample period (>=0.01 s)
paper.require_all_trials = true;      % do not silently average fewer trials
paper.normal_ylim = [];              % [] = common XYZ limits containing all bands
paper.error_ylim = [];               % [] = limits containing all displayed bands
paper.show_condition_strips = true;
paper.fig7_xtick_s = 5.0;            % Fig. 7 major ticks/grid every 5 s

% Wind RMSE includes moving + commanded stationary portions of this window.
% Default: COMPLETE descending plane-2 visit, expanded by 2 s before / 1 s after.
% This is the ORIGINAL geometry-based proxy, NOT measured airflow on/off.
% Override using PLOTTED times: [start stop] for every run, or rows
% [trial_id start stop] for individual runs; [] keeps the geometric proxy.
paper.wind_padding_s = [2.0 1.0];
paper.wind_windows = [];

% Fig. 8: the two commanded stationary holds on plane 2, not the whole visits.
% Fixed 2 s settling exclusion after EACH stop command is a configurable
% analysis choice, not a detected wind-settling time. No samples are selected
% by estimator error, force magnitude, or speed after the window is chosen.
% Measured speed is audited (not used to cherry-pick quiet samples).
paper.stationary_settle_s = 2.0;
paper.stationary_end_trim_s = 0.0;
paper.stationary_speed_check_mps = 0.03;
paper.show_trial_dots = false;
paper.print_window_audit = true;

% New metrics use the same valid timestamps for all methods within each run.
% They are MEAN +/- SAMPLE SD of per-trial RMSEs, never pooled RMSE.
paper.only_paper_figures = false;     % true: skip the old diagnostic figures
paper.export_files = false;          % optional .fig/.png and metric .csv files
paper.export_dir = '';               % []/'' = <selected CSV folder>/paper_figures

%% Configuration and CSV loading
% Original estimator gains and calibrated surface geometry are retained.
% Capture the executing script path HERE, not from a local-function stack.
% Editor execution can still supply a temporary path; the resolver rejects it.
data_dir = resolveCsvDataDirectory(csv_data_dir,mfilename('fullpath'));
if isempty(data_dir), return; end % user cancelled folder selection
fprintf('CSV selection folder: %s\n',data_dir);
cfg.beta = 3.5;
cfg.gamma = 3.0;
cfg.eps_v = 0.0004;
cfg.eps_f = 0.005;
cfg.final_contact_hold_sec = 10.0; % command-zero event + 10 s, INCLUDING settling
cfg.fit_post_motion_sec = 10.0;
cfg.true_normal_max_distance = 0.05;
cfg.origin = [1.313408 -0.000529 0.869405];
cfg.plane_inclinations_deg = [-23 0 21];
cfg.initial_tangent_deg = 90-cfg.plane_inclinations_deg(1);
cfg.plane_lengths = [0.052901 0.232250 0.195486];
cfg.curve_radius = 0.810816;
cfg.turn_angles_deg = diff(cfg.plane_inclinations_deg);
cfg.true_normal_hold_time = 54.0; % legacy setting; not used as a crop endpoint
% CF21BL voltage-aware MOB input model. These fixed constants match the
% firmware build used for this experiment (battery compensation enabled,
% maximum thrust 200 mN per motor). The original raw MOB is retained.
cfg.mob_lpf_hz = 1.0;
cfg.motor_thrust_scale = 0.920;
cfg.motor_thrust_max = 0.200;
cfg.arm_length = 0.050;
cfg.thrust_to_torque = 0.00569278844371417;
cfg.com_offset_body = [-0.000440 -0.000234 0.0];
cfg.contact_offset_body = [0.11 0.0 0.017];
cfg.motor_voltage_to_thrust = [-0.014058926705279723, ...
    0.04265273261724981,0.0018327760144017432,0.0020576974784587178];
cfg.thrust_effectiveness_gamma = 2.0;
cfg.thrust_effectiveness_rho = 0.002;
cfg.thrust_data = validatestring(lower(char(thrust_data)),{'raw','comp'}, ...
    mfilename,'thrust_data');
fprintf('MOB motor-thrust source: %s\n',cfg.thrust_data);

validateattributes(time_crop_start,{'numeric'}, ...
    {'vector','numel',numel(recording_ids),'finite','nonnegative'}, ...
    mfilename,'time_crop_start');
validateattributes(time_crop_end,{'numeric'}, ...
    {'vector','numel',numel(recording_ids)},mfilename,'time_crop_end');
if any(~isnan(time_crop_end)&(~isfinite(time_crop_end)|time_crop_end<0))
    error('time_crop_end entries must be NaN or finite nonnegative times.');
end
if ~any(strcmpi(time_zero_mode,{'crop','command'}))
    error('time_zero_mode must be ''crop'' or ''command''.');
end

[selected_files,selected_path] = uigetfile( ...
    {'*.csv','Normal-estimator CSV files (*.csv)'}, ...
    'Select original CSVs (1st ... 11th)', ...
    [data_dir filesep],'MultiSelect','on'); % explicit folder path, not filename
if isequal(selected_files,0), return; end
if ischar(selected_files) || isstring(selected_files)
    selected_files = cellstr(selected_files);
end
selected_files = selected_files(:);
trial_ids = nan(numel(selected_files),1);
keep_files = true(numel(selected_files),1);
for file_index = 1:numel(selected_files)
    name = selected_files{file_index};
    is_excluded = any(cellfun(@(id) contains(name,id,'IgnoreCase',true), ...
        excluded_recording_ids));
    if is_excluded
        warning('FlyingPen:ExcludedRecording', ...
            'Excluded incomplete recording: %s',name);
        keep_files(file_index) = false;
        continue;
    end
    trial_ids(file_index) = resolveTrialNumber(name,recording_ids);
end
selected_files = selected_files(keep_files);
trial_ids = trial_ids(keep_files);
if isempty(selected_files)
    warning('No eligible recordings were selected.');
    return;
end
if numel(unique(trial_ids))~=numel(trial_ids)
    error('The same trial number was selected more than once. Check filenames.');
end
% Natural numeric ordering: 1st,2nd,...,10th,11th, never lexicographic ordering.
[trial_ids,sort_index] = sort(trial_ids);
selected_files = selected_files(sort_index);

records_full = struct([]);
records = struct([]);
for file_index = 1:numel(selected_files)
    trial_id = trial_ids(file_index);
    csv_path = fullfile(selected_path,selected_files{file_index});
    fprintf('\nLoading %s (trial %d)\n',selected_files{file_index},trial_id);
    table_data = readtable(csv_path,detectImportOptions(csv_path,'Delimiter',','));

    % IMPORTANT: estimator memory receives the ENTIRE original log.
    full_record = prepareRecord(table_data,selected_files{file_index},cfg);
    record = cropRecordForAnalysis(full_record,trial_id, ...
        time_crop_start(trial_id),time_crop_end(trial_id),time_zero_mode,cfg);
    if file_index == 1
        records_full = full_record;
        records = record;
    else
        records_full(file_index) = full_record;
        records(file_index) = record;
    end
end

% Every plot and metric below receives the SAME post-estimation analysis copy.
% Endpoints remain per-trial. The axis spans their union; data are not padded.
all_plot_times = vertcat(records.t);
global_xlim = [min(all_plot_times) max(all_plot_times)];
timing_summary = buildTimingSummary(records);
fprintf('\nCrop timing audit (all *_elapsed_s columns use the original time base)\n');
disp(timing_summary);
fprintf('Time origin mode: %s | plot range: %.6f to %.6f s\n', ...
    time_zero_mode,global_xlim);
fprintf(['Original CSVs are unchanged. Estimator histories are retained in ' ...
    'records_full.\n']);
% Optional export, uncomment if needed:
% writetable(timing_summary,fullfile(selected_path,'crop_timing_summary.csv'));

% Display one common surface at the mean fitted pose; true normals remain
% based on each recording's original, independently fitted pose.
fittedOrigins = vertcat(records.geometry_cfg);
fittedOrigins = vertcat(fittedOrigins.origin);
cfg.origin = mean(fittedOrigins,1);
fprintf(['Loaded %d CSV files | beta=%.4g, gamma=%.4g, ' ...
    'eps_v=%.4g, eps_f=%.4g\n'],numel(records), ...
    cfg.beta,cfg.gamma,cfg.eps_v,cfg.eps_f);

dataset_labels = repmat({''},numel(records),1);
if numel(records)>1
    for file_index=1:numel(records)
        dataset_labels{file_index}=ordinalLabel(records(file_index).trial_id);
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

%% Paper results: Fig. 7, Fig. 8 and Table I (before the Overview panel)
% Replaces the old pooled-RMSE printout. Source CSVs and records are untouched.
[paper_stats,fig7,fig8] = makeFirstExperimentPaperResults( ...
    records,global_xlim,paper);
table_I = paper_stats.table_I;
table_I_numeric = paper_stats.table_I_numeric;
fig8_numeric = paper_stats.fig8_numeric;
paper_window_audit = paper_stats.window_audit;

if paper.export_files
    output_dir = paper.export_dir;
    if isempty(output_dir), output_dir = fullfile(selected_path,'paper_figures'); end
    exportFirstExperimentPaperResults(fig7,fig8,paper_stats,output_dir);
end
drawnow;
if paper.only_paper_figures, return; end

%% 0. Overview panel
plotOverviewPanel(records,dataset_labels,record_colors,method_names, ...
    method_styles,method_widths,axis_names,global_xlim, ...
    [80 60 1350 800]); % figure [left bottom width height] [px]

%% Key displays: traversal summary, geometry view, and plane-2 comparison
plotFirstExperimentKeyDisplays(records,dataset_labels,record_colors, ...
    global_xlim,cfg,[40 60],680,115);
%% 0-1. Repeated-experiment normal-error statistics
comparison_method_indices = [3 4 2 1];
all_comparison_method_labels = method_names; % M3=OFF, M4=ON (no label swap)
comparison_method_labels = all_comparison_method_labels(comparison_method_indices);
[normal_rmse_by_run,force_rmse_by_run] = ...
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
        normal_set = normalMethodSet(records(file_index));
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
    normal_set = normalMethodSet(records(file_index));
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
        force_set = cat(3,records(file_index).f_raw,records(file_index).f_bar, ...
            records(file_index).f_c);
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
        force_set = forceCueSet(records(file_index));
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
    unit_set = forceCueSet(records(file_index));
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
% Resolve optional display windows only after the new time axis is available.
turn_times = nan(numel(records),1);
for file_index=1:numel(records)
    r=records(file_index);
    valid=isfinite(r.surface_arc)&isfinite(r.t)& ...
        r.t>=r.surface_loop_time(1)&r.t<=r.surface_loop_time(2);
    ii=find(valid);
    if ~isempty(ii)
        [~,kk]=max(r.surface_arc(ii));
        turn_times(file_index)=r.t(ii(kk));
    end
end
turn_time=mean(turn_times,'omitnan');
if ~isfinite(turn_time)||turn_time<=global_xlim(1)||turn_time>=global_xlim(2)
    turn_time=mean(global_xlim);
end
if isempty(trajectory_iso_time), trajectory_iso_time=global_xlim; end
if isempty(trajectory_xz_time_1), trajectory_xz_time_1=[global_xlim(1) turn_time]; end
if isempty(trajectory_xz_time_2), trajectory_xz_time_2=[turn_time global_xlim(2)]; end
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
    trajectory_normals = normalMethodSet(records(file_index));
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
title(sprintf('Isometric view: %.1f-%.1f s',trajectory_iso_time)); legend('Location','best','Interpreter','latex');

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
        trajectory_normals = normalMethodSet(records(file_index));
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

function trialId=resolveTrialNumber(fileName,recordingIds)
% Prefix takes precedence, but known timestamps must agree with its mapping.
token=regexp(fileName,'(?i)^\s*(\d+)\s*(?:st|nd|rd|th)','tokens','once');
knownId=regexp(fileName,'\d{8}_\d{6}','match','once');
knownIndex=find(strcmp(recordingIds,knownId));
if ~isempty(token)
    trialId=str2double(token{1});
    if ~isfinite(trialId)||trialId<1||trialId>numel(recordingIds)||fix(trialId)~=trialId
        error('FlyingPen:TrialNumber','Invalid trial prefix in %s.',fileName);
    end
    if ~isempty(knownId)&&~strcmp(recordingIds{trialId},knownId)
        error('FlyingPen:TrialMapping', ...
            ['%s: prefix %d disagrees with recording_ids{%d}=%s. ' ...
             'Update recording_ids AND time_crop_start to match your numbering.'], ...
            fileName,trialId,trialId,recordingIds{trialId});
    end
elseif numel(knownIndex)==1
    trialId=knownIndex;
else
    error('FlyingPen:UnknownRecording', ...
        ['Cannot map %s to a trial. Use a 1st...11th filename prefix, ' ...
         'or add its timestamp to recording_ids.'],fileName);
end
end

function label=ordinalLabel(value)
if any(mod(value,100)==[11 12 13])
    suffix='th';
else
    switch mod(value,10)
        case 1, suffix='st';
        case 2, suffix='nd';
        case 3, suffix='rd';
        otherwise, suffix='th';
    end
end
label=sprintf('%d%s',value,suffix);
end

function [commandStart,commandStop,lastActiveIndex]=commandMotionTimes(t,u2)
% First upward command and FIRST zero-command sample after the final pass.
commandStart=nan; commandStop=nan; lastActiveIndex=[];
finiteCommand=abs(u2(isfinite(u2)));
if isempty(finiteCommand), return; end
threshold=max(1e-6,0.1*max(finiteCommand));
firstPositive=find(isfinite(t)&isfinite(u2)&u2>threshold,1,'first');
lastActiveIndex=find(isfinite(t)&isfinite(u2)&abs(u2)>threshold,1,'last');
if ~isempty(firstPositive), commandStart=t(firstPositive); end
if isempty(lastActiveIndex), return; end
firstZero=find((1:numel(t)).'>lastActiveIndex & ...
    isfinite(t)&isfinite(u2)&abs(u2)<=threshold,1,'first');
if ~isempty(firstZero), commandStop=t(firstZero); end
end

function record=cropRecordForAnalysis(fullRecord,trialId,startTime,endTime,zeroMode,cfg)
% Full-history estimation is already finished when this function is called.
% Only this analysis copy is sliced. No estimator is restarted here.
t=fullRecord.t;
[cmdStart,cmdStop]=commandMotionTimes(t,fullRecord.u2_des);
if ~isfinite(cmdStart)||~isfinite(cmdStop)
    error('FlyingPen:IncompleteSequence', ...
        '%s: a complete first-motion / final-zero command pair was not found.', ...
        fullRecord.label);
end
u=fullRecord.u2_des;
threshold=max(1e-6,0.1*max(abs(u(isfinite(u)))));
if ~any(u< -threshold)
    error('FlyingPen:IncompleteSweep', ...
        '%s: no downward pass is present; not a complete first experiment.',fullRecord.label);
end
if isnan(endTime), endTime=cmdStop+cfg.final_contact_hold_sec; end
contactTime=contactStartTime(fullRecord);
if startTime<t(1)||startTime>=t(end)||endTime<=startTime
    error('FlyingPen:CropWindow', ...
        '%s: invalid requested crop [%.6f, %.6f] s.',fullRecord.label,startTime,endTime);
end
if startTime>=cmdStart
    warning('FlyingPen:NoInitialDwell', ...
        '%s: crop starts at or after first motion; there is no initial 5 s dwell.', ...
        fullRecord.label);
end
if isfinite(contactTime)&&startTime<contactTime
    warning('FlyingPen:PreContactCrop', ...
        '%s: crop includes samples before the force command onset.',fullRecord.label);
end
if abs((cmdStart-startTime)-5.0)>0.05
    warning('FlyingPen:BaselineLength', ...
        '%s: selected initial dwell is %.3f s, not 5 s.', ...
        fullRecord.label,cmdStart-startTime);
end
requestedEnd=endTime;
if endTime>t(end)
    warning('FlyingPen:ShortFinalHold', ...
        ['%s: requested endpoint %.6f s exceeds the log end %.6f s. ' ...
         'The hold is truncated; see timing_summary.'], ...
        fullRecord.label,endTime,t(end));
    endTime=t(end);
end
keep=isfinite(t)&t>=startTime&t<=endTime;
if nnz(keep)<2, error('Crop window contains fewer than two samples.'); end
if strcmpi(zeroMode,'command'), offset=cmdStart; else, offset=startTime; end

% Slice all per-sample arrays, including masks, firmware signals, and GT.
% Small metadata arrays (e.g. surface_bounds) are deliberately not sliced.
record=fullRecord;
nSamples=numel(t);
names=fieldnames(fullRecord);
for fieldIndex=1:numel(names)
    name=names{fieldIndex};
    value=fullRecord.(name);
    if (isnumeric(value)||islogical(value))&&size(value,1)==nSamples
        subs=repmat({':'},1,ndims(value));
        subs{1}=keep;
        record.(name)=value(subs{:});
    end
end
record.t_original_elapsed=t(keep);
record.t=t(keep)-offset;
% Every interval must undergo exactly the same shift as signal timestamps.
record.surface_loop_time=[max(fullRecord.surface_loop_time(1),startTime), ...
    min(fullRecord.surface_loop_time(2),endTime)]-offset;
record.trial_id=trialId;
record.crop_start_elapsed_s=startTime;
record.crop_end_elapsed_s=endTime;
record.requested_end_elapsed_s=requestedEnd;
record.contact_on_elapsed_s=contactTime;
record.command_start_elapsed_s=cmdStart;
record.command_stop_elapsed_s=cmdStop;
record.time_offset_s=offset;
record.command_start_plot_s=cmdStart-offset;
record.command_stop_plot_s=cmdStop-offset;
record.time_zero_mode=zeroMode;
record.initial_dwell_s=cmdStart-startTime;
record.final_command_hold_s=endTime-cmdStop;
record.final_hold_complete=(t(end)>=requestedEnd);
record.full_indices=find(keep);
% Used by the optional boom-offset analysis to preserve its warmup too.
record.full_history=fullRecord;

baseline=t>=startTime&t<cmdStart&isfinite(fullRecord.fn_hat);
record.baseline_force_mean_mN=1e3*mean(fullRecord.fn_hat(baseline),'omitnan');
record.baseline_force_sd_mN=1e3*std(fullRecord.fn_hat(baseline),0,'omitnan');
fprintf(['  %s | crop %.6f -> %.6f s | u2 onset %.6f s | ' ...
    'plotted u2 onset %.6f s | pre-motion %.3f s | final-command hold %.3f s\n'], ...
    ordinalLabel(trialId),startTime,endTime,cmdStart, ...
    record.command_start_plot_s,record.initial_dwell_s,record.final_command_hold_s);
end

function summary=buildTimingSummary(records)
trial=[records.trial_id].';
file=string({records.label}).';
contact_on_elapsed_s=[records.contact_on_elapsed_s].';
u2_on_elapsed_s=[records.command_start_elapsed_s].';
crop_start_elapsed_s=[records.crop_start_elapsed_s].';
u2_stop_elapsed_s=[records.command_stop_elapsed_s].';
crop_end_elapsed_s=[records.crop_end_elapsed_s].';
initial_dwell_s=[records.initial_dwell_s].';
final_command_hold_s=[records.final_command_hold_s].';
final_hold_complete=[records.final_hold_complete].';
u2_on_plot_s=[records.command_start_plot_s].';
baseline_force_mean_mN=[records.baseline_force_mean_mN].';
baseline_force_sd_mN=[records.baseline_force_sd_mN].';
summary=table(trial,file,contact_on_elapsed_s,u2_on_elapsed_s, ...
    crop_start_elapsed_s,u2_stop_elapsed_s,crop_end_elapsed_s, ...
    initial_dwell_s,final_command_hold_s,final_hold_complete,u2_on_plot_s, ...
    baseline_force_mean_mN,baseline_force_sd_mN);
end

function record = prepareRecord(T,file_name,cfg)
t = safeColumn(T,'t_sec');
if isempty(t)||any(~isfinite(t))
    error('FlyingPen:InvalidTime','%s: t_sec must be finite for every row.',file_name);
end
record.t_csv = t; % retain exact original CSV timestamps
record.first_csv_time_sec = t(1);
t = t-t(1);
if any(diff(t)<=0)
    error('FlyingPen:InvalidTime','%s: t_sec must be strictly increasing.',file_name);
end
vc = columnsXYZ(T,'fp_normal_vc');
t1 = columnsXYZ(T,'fp_normal_t1');
t2 = columnsXYZ(T,'fp_normal_t2');
f_c_firmware = columnsXYZ(T,'fp_normal_fcHat');
f_bar_firmware = columnsXYZ(T,'fp_normal_fBar');
f_raw_firmware = columnsXYZ(T,'fp_normal_fRaw');
ee_pos = columnsXYZ(T,'fp_normal_eePos');

record.label = erase(file_name,'.csv');
record.thrust_data = cfg.thrust_data;
record.t = t;
record.f_c_firmware = f_c_firmware;
record.f_bar_firmware = f_bar_firmware;
record.f_raw_firmware = f_raw_firmware;
record.ee_pos = ee_pos;
record.u1_des = safeColumn(T,'u1_des');
record.u2_des = safeColumn(T,'u2_des');
record.v_t1 = sum(vc.*t1,2);
record.v_t2 = sum(vc.*t2,2);
record.velocity = vc;
record.speed_filtered = zeroPhaseLowPass(vecnorm(vc,2,2),t,1.0);
record.battery_voltage = safeColumn(T,'fp_normal_batteryVoltage');
record.thrust_effectiveness_firmware = safeColumn(T,'fp_normal_etaHat');
record.thrust_effectiveness = record.thrust_effectiveness_firmware;
record.attitude_rpy_deg = [safeColumn(T,'attitudeRoll'), ...
    safeColumn(T,'attitudePitch'),safeColumn(T,'attitudeYaw')];
record.mob_torque_world_firmware = columnsXYZ(T,'fp_normal_mobTorque');
record.mob_input_force_world = columnsXYZ(T,'fp_normal_mobInputForce');
record.mob_input_torque_body = columnsXYZ(T,'fp_normal_mobInputTorqueBody');
voltageModel = voltageModelMobCorrection(t,record.mob_input_force_world, ...
    record.mob_input_torque_body,record.attitude_rpy_deg, ...
    record.battery_voltage,cfg);
record.motor_force_linear = voltageModel.motor_force_linear;
record.motor_force_voltage_model = voltageModel.motor_force_model;
record.voltage_model_eta = voltageModel.eta;
record.mob_input_force_voltage_model = voltageModel.force_input_world;
record.mob_input_torque_voltage_model = voltageModel.torque_input_world;
record.f_voltage = f_raw_firmware+voltageModel.force_correction_world;
record.torque_voltage_world = record.mob_torque_world_firmware+voltageModel.torque_correction_world;
if strcmp(cfg.thrust_data,'comp')
    compPipeline=offlineVoltageModelPipeline(t,record.f_voltage, ...
        record.torque_voltage_world,voltageModel.force_input_world, ...
        voltageModel.torque_input_world,record.attitude_rpy_deg,cfg);
    record.f_raw=record.f_voltage;
    record.mob_torque_world=record.torque_voltage_world;
    record.f_bar=compPipeline.f_bar;
    record.f_c=compPipeline.f_c;
    record.thrust_effectiveness=compPipeline.eta;
    record.torque_bar_world=compPipeline.torque_bar;
else
    record.f_raw=f_raw_firmware;
    record.mob_torque_world=record.mob_torque_world_firmware;
    record.f_bar=f_bar_firmware;
    record.f_c=f_c_firmware;
    record.torque_bar_world=nan(size(record.mob_torque_world));
end
record.force_desired = safeColumn(T,'forceDesired');
record.fn_hat = safeColumn(T,'fp_normal_fnHat');
record.n_firmware = columnsXYZ(T,'fp_normal_nHat');
record.n_velocity_on_fbar = offlineNormalEstimator(t,vc,record.f_bar,cfg,true);
record.n_velocity_on_fraw = offlineNormalEstimator(t,vc,record.f_raw,cfg,true);
record.n_velocity_on_fc = offlineNormalEstimator(t,vc,record.f_c,cfg,true);
record.n_velocity_off_fc = offlineNormalEstimator(t,vc,record.f_c,cfg,false);
record.f_c_unit = normalizeRows(record.f_c,1e-12);
record.f_bar_unit = normalizeRows(record.f_bar,1e-12);
record.f_raw_unit = normalizeRows(record.f_raw,1e-12);
[record.geometry_cfg,record.geometry_rms,record.surface_loop_time] = ...
    fitFiveSectionPose(t,ee_pos,record.force_desired,record.u2_des, ...
        cfg.fit_post_motion_sec,cfg);
[record.true_normal,record.surface_section,record.surface_arc,record.surface_distance, ...
    record.surface_bounds] = trueNormalFromKnownGeometry( ...
    ee_pos,record.geometry_cfg,cfg.true_normal_max_distance);
record.true_normal=holdEndpointNormalThroughFinalDwell( ...
    record.true_normal,t,record.u2_des,cfg.final_contact_hold_sec);
middlePlaneMid=0.5*(record.surface_bounds(3)+record.surface_bounds(4));
upperCurveMid=0.5*(record.surface_bounds(4)+record.surface_bounds(5));
record.friction_plane_mask=record.surface_distance<=cfg.true_normal_max_distance & ...
    record.surface_arc>=middlePlaneMid-0.10&record.surface_arc<=middlePlaneMid;
record.friction_curve_mask=record.surface_distance<=cfg.true_normal_max_distance & ...
    record.surface_arc>=upperCurveMid&record.surface_arc<=record.surface_bounds(5);
record.friction_mask=record.friction_plane_mask|record.friction_curve_mask;
fprintf('  %s | surface RMS=%.2f mm, closed sweep=%.2f-%.2f s\n', ...
    record.label,1e3*record.geometry_rms,record.surface_loop_time);
end

function out=voltageModelMobCorrection(t,forceInputWorld,torqueInputBody,rpyDeg,batteryVoltage,cfg)
% Recover the four post-battery-compensation motor commands from the logged
% MOB input wrench, evaluate the fixed CF21BL voltage-to-thrust polynomial,
% and filter the input-model difference with the same critically damped MOB
% bandwidth. This creates an additional comparison; it never overwrites raw.
n=numel(t);
forceInputBody=rotateWorldRows(forceInputWorld,rpyDeg);
fz=forceInputBody(:,3);
tauCom=[cfg.com_offset_body(2).*fz, ...
       -cfg.com_offset_body(1).*fz,zeros(n,1)];
tauMotor=torqueInputBody-tauCom;
lever=cfg.arm_length/sqrt(2);
kq=cfg.thrust_to_torque;
mixer=[1 1 1 1; -lever -lever lever lever; ...
       -lever lever lever -lever; -kq kq -kq kq];
wrench=[fz tauMotor].';
motorForce=(mixer\wrench).';

ratio=motorForce/(cfg.motor_thrust_scale*cfg.motor_thrust_max);
ratio=max(0,min(1,ratio));
motorVoltage=ratio.*batteryVoltage;
a=cfg.motor_voltage_to_thrust;
motorForceModel=a(1)+a(2).*motorVoltage+a(3).*motorVoltage.^2+ ...
    a(4).*motorVoltage.^3;
motorForceModel=max(0,motorForceModel);
invalid=~all(isfinite([forceInputWorld torqueInputBody rpyDeg]),2)| ...
    ~isfinite(batteryVoltage);
motorForce(invalid,:)=nan;
motorForceModel(invalid,:)=nan;

forceModelBody=[zeros(n,2) sum(motorForceModel,2)];
forceModelWorld=rotateBodyRows(forceModelBody,rpyDeg);
tauMotorModel=(mixer(2:4,:)*motorForceModel.').';
tauComModel=cross(repmat(cfg.com_offset_body,n,1),forceModelBody,2);
tauModelBody=tauMotorModel+tauComModel;
tauInputWorld=rotateBodyRows(torqueInputBody,rpyDeg);
tauModelWorld=rotateBodyRows(tauModelBody,rpyDeg);

% The external-wrench estimate changes by nominal_linear-nominal_model.
out.force_correction_world=causalMatchedFilter(t, ...
    forceInputWorld-forceModelWorld,cfg.mob_lpf_hz);
out.torque_correction_world=causalMatchedFilter(t, ...
    tauInputWorld-tauModelWorld,cfg.mob_lpf_hz);
out.motor_force_linear=motorForce;
out.motor_force_model=motorForceModel;
out.force_input_world=forceModelWorld;
out.torque_input_world=tauModelWorld;
out.eta=sum(motorForceModel,2)./sum(motorForce,2);
out.eta(~isfinite(out.eta))=nan;
end

function output=causalMatchedFilter(t,input,cutoffHz)
% Continuous-time critically damped second-order filter integrated with
% short Euler substeps. Full-history execution preserves estimator warmup.
output=nan(size(input)); signal=zeros(1,size(input,2)); signalDot=signal;
wn=2*pi*cutoffHz;
for k=1:numel(t)
    if ~all(isfinite(input(k,:))), continue; end
    if k==1, dt=0; else, dt=t(k)-t(k-1); end
    if ~isfinite(dt)||dt<0, continue; end
    steps=max(1,ceil(dt/0.005)); h=dt/steps;
    for j=1:steps
        signalDdot=wn^2*(input(k,:)-signal)-2*wn*signalDot;
        signalDot=signalDot+h*signalDdot;
        signal=signal+h*signalDot;
    end
    output(k,:)=signal;
end
end

function out=offlineVoltageModelPipeline(t,forceRaw,torqueRaw,forceInput,torqueInput,rpyDeg,cfg)
% Re-run the thrust-effectiveness and point-contact stages downstream of a
% voltage-model-based MOB input. No logged adaptive eta is reused.
matchedForce=causalMatchedFilter(t,forceInput,cfg.mob_lpf_hz);
matchedTorque=causalMatchedFilter(t,torqueInput,cfg.mob_lpf_hz);
offsetWorld=rotateBodyRows(repmat(cfg.contact_offset_body,numel(t),1),rpyDeg);
eta=nan(numel(t),1); value=1.0;
for k=1:numel(t)
    valid=all(isfinite([offsetWorld(k,:) matchedForce(k,:) matchedTorque(k,:) ...
        forceRaw(k,:) torqueRaw(k,:)]));
    if ~valid, continue; end
    y=cross(offsetWorld(k,:),matchedForce(k,:))-matchedTorque(k,:);
    residual=cross(offsetWorld(k,:),forceRaw(k,:))-torqueRaw(k,:);
    epsilon=residual-(value-1.0).*y;
    if k==1, dt=0; else, dt=t(k)-t(k-1); end
    if isfinite(dt)&&dt>0
        value=value+dt*max(0,cfg.thrust_effectiveness_gamma)* ...
            dot(y,epsilon)/(cfg.thrust_effectiveness_rho+dot(y,y));
        value=max(value,1e-6);
    end
    eta(k)=value;
end
etaOffset=eta-1.0;
out.f_bar=forceRaw-etaOffset.*matchedForce;
out.torque_bar=torqueRaw-etaOffset.*matchedTorque;
out.f_c=reconstructContactForceRows(out.f_bar,out.torque_bar,offsetWorld);
out.eta=eta;
end

function force=reconstructContactForceRows(forceBar,torqueBar,offsetWorld)
r2=sum(offsetWorld.^2,2);
parallel=offsetWorld.*(sum(offsetWorld.*forceBar,2)./r2);
transverse=-cross(offsetWorld,torqueBar,2)./r2;
force=parallel+transverse;
invalid=~all(isfinite([offsetWorld forceBar torqueBar]),2)|r2<=1e-12;
force(invalid,:)=nan;
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

function [fittedCfg,rmsError,loopTime] = ...
    fitFiveSectionPose(t,eePos,forceDesired,u2Desired,postMotionSec,cfg)
% Fit only the X-Z translation of the calibrated five-section shape. Use
% the commanded contact/motion interval so approach and return tails cannot
% pull the fitted surface away from the actual traversal.
valid=isfinite(t)&all(isfinite(eePos(:,[1 3])),2);
forceFinite=forceDesired(isfinite(forceDesired));
u2Finite=abs(u2Desired(isfinite(u2Desired)));
if ~isempty(forceFinite)&&~isempty(u2Finite)
    forceThreshold=max(1e-6,0.1*max(forceFinite));
    motionThreshold=max(1e-6,0.1*max(u2Finite));
    contactStart=find(isfinite(forceDesired)&forceDesired>forceThreshold,1,'first');
    motionEnd=find(isfinite(u2Desired)&abs(u2Desired)>motionThreshold,1,'last');
else
    contactStart=[]; motionEnd=[];
end
if ~isempty(contactStart)&&~isempty(motionEnd)&&motionEnd>contactStart
    fitEndTime=t(motionEnd)+postMotionSec;
    loopIdx=find(valid&(1:numel(t)).'>=contactStart&t<=fitEndTime);
else
    warning('Contact/motion commands unavailable; falling back to closed-loop detection.');
    validIdx=find(valid); pointsAll=eePos(validIdx,[1 3]);
    [loopStart,loopEnd]=longestClosedLoop(pointsAll,0.035,1.0);
    loopIdx=validIdx(loopStart:loopEnd);
end
loopPoints=eePos(loopIdx,[1 3]);

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

function contactTime=contactStartTime(record)
% The automatic sequence raises forceDesired when surface contact begins.
% Fall back to a sustained measured-normal-force threshold for older CSVs.
contactTime=nan;
desired=record.force_desired;
finiteDesired=desired(isfinite(desired));
if ~isempty(finiteDesired)
    threshold=max(1e-6,0.1*max(finiteDesired));
    first=find(isfinite(desired)&desired>threshold,1,'first');
    if ~isempty(first), contactTime=record.t(first); return; end
end
measured=record.fn_hat;
finiteTime=record.t(isfinite(record.t));
if numel(finiteTime)<2, return; end
sampleDt=median(diff(finiteTime));
if ~isfinite(sampleDt)||sampleDt<=0, return; end
candidate=isfinite(measured)&measured>0.005;
runLength=max(1,round(0.2/sampleDt));
sustained=movsum(candidate,[0 runLength-1])>=runLength;
first=find(sustained,1,'first');
if ~isempty(first), contactTime=record.t(first); end
end

function trueNormal=holdEndpointNormalThroughFinalDwell(trueNormal,t,u2Desired,holdSec)
% An extended final descent can pass beyond the fitted surface endpoint.
% Preserve its last valid normal through the descent tail and final dwell.
finiteCommand=abs(u2Desired(isfinite(u2Desired)));
if isempty(finiteCommand), return; end
threshold=max(1e-6,0.1*max(finiteCommand));
motionEnd=find(isfinite(u2Desired)&abs(u2Desired)>threshold,1,'last');
if isempty(motionEnd), return; end
lastValid=find((1:numel(t)).'<=motionEnd&all(isfinite(trueNormal),2),1,'last');
if isempty(lastValid), return; end
[~,commandStop]=commandMotionTimes(t,u2Desired);
if ~isfinite(commandStop), commandStop=t(motionEnd); end
holdEnd=find(isfinite(t)&t<=commandStop+holdSec,1,'last');
if isempty(holdEnd), holdEnd=motionEnd; end
fillRange=lastValid:holdEnd;
trueNormal(fillRange,:)=repmat(trueNormal(lastValid,:),numel(fillRange),1);
end


function endTime=motionEndTime(record,finalContactHoldSec)
% Zero-command event is a reproducible stopping marker, not measured rest.
[~,stopTime]=commandMotionTimes(record.t,record.u2_des);
endTime=nan;
if isfinite(stopTime)
    endTime=min(stopTime+finalContactHoldSec,max(record.t));
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

function x=normalMethodSet(r)
x=cat(3,r.n_velocity_on_fraw,r.n_velocity_on_fbar, ...
    r.n_velocity_off_fc,r.n_velocity_on_fc);
end

function x=forceCueSet(r)
x=cat(3,r.f_raw_unit,r.f_bar_unit,r.f_c_unit);
end

function color = curveColor(base,method,record_count)
if method == 4
    % M4: velocity correction ON + f_c, fixed light purple.
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
    normalSet=normalMethodSet(r);
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
    normalSet=normalMethodSet(r);
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
        % MATLAB does not allow uistack to reorder children on yyaxis axes.
        % The translucent patch is still valid there; only reorder ordinary axes.
        if numel(ax(axisIndex).YAxis)==1
            uistack(p,'bottom');
        end
        if ~holdWasOn, hold(ax(axisIndex),'off'); end
    end
end
end

function intervals=planeSequenceIntervals(record)
% Legacy diagnostics keep their original -2/+1 s wind proxy, but use the
% COMPLETE planar visits. Brief plane/curve classification gaps no longer
% truncate the descending plane-2 visit (observed in trials 5 and 9).
intervals=paperPlanarVisits(record);
planeTwo=find([intervals.plane]==2);
if numel(planeTwo)>=2
    k=planeTwo(2);
    intervals(k).start=max(record.surface_loop_time(1),intervals(k).start-2.0);
    intervals(k).stop=min(record.surface_loop_time(2),intervals(k).stop+1.0);
end
end

function plotOfflineBoomOverview(records,datasetLabels,boomX,boomZ,globalXLim,cfg,figurePosition)
boomX=boomX(:); boomZ=boomZ(:);
if isempty(boomX)||isempty(boomZ)
    error('boom_offset_x and boom_offset_z must both be nonempty.');
end
% Cartesian product: evaluate every X candidate with every Z candidate.
[boomXGrid,boomZGrid]=ndgrid(boomX,boomZ);
boomX=boomXGrid(:); boomZ=boomZGrid(:);
pairCount=numel(boomX); pairColors=lines(pairCount);
axisLetters='xyz';
recordStyles={'-','--',':','-.'};
offline=cell(numel(records),pairCount);
for fileIndex=1:numel(records)
    for pairIndex=1:pairCount
        r=records(fileIndex);
        if isfield(r,'full_history')
            source=r.full_history;
            retained=r.full_indices;
        else
            source=r;
            retained=(1:numel(r.t)).';
        end
        forceFull=reconstructContactForce(source,boomX(pairIndex),boomZ(pairIndex));
        normalFull=offlineNormalEstimator(source.t,source.velocity,forceFull,cfg,true);
        force=forceFull(retained,:);
        offline{fileIndex,pairIndex}.force=force;
        offline{fileIndex,pairIndex}.force_unit=normalizeRows(force,1e-12);
        offline{fileIndex,pairIndex}.normal=normalFull(retained,:);
        offline{fileIndex,pairIndex}.normal_rmse=angularRmse( ...
            offline{fileIndex,pairIndex}.normal,records(fileIndex).true_normal, ...
            records(fileIndex).t,globalXLim);
        offline{fileIndex,pairIndex}.force_rmse=angularRmse( ...
            offline{fileIndex,pairIndex}.force_unit,records(fileIndex).true_normal, ...
            records(fileIndex).t,globalXLim);
    end
end
fprintf('\nOffline boom-offset RMSE metrics\n');
fprintf('%-20s %10s %10s %18s %18s\n','Dataset','boom X','boom Z', ...
    'Normal RMSE [deg]','Force RMSE [deg]');
for fileIndex=1:numel(records)
    for pairIndex=1:pairCount
        fprintf('%-20s %10.4f %10.4f %18.3f %18.3f\n',records(fileIndex).label, ...
            boomX(pairIndex),boomZ(pairIndex),offline{fileIndex,pairIndex}.normal_rmse, ...
            offline{fileIndex,pairIndex}.force_rmse);
    end
end
fig=figure('Name','Offline boom-offset overview','Color','w','Position',figurePosition);
panels(1)=uipanel(fig,'Units','normalized','Position',[0.015 0.015 0.477 0.970], ...
    'BackgroundColor','w','BorderType','line');
panels(2)=uipanel(fig,'Units','normalized','Position',[0.508 0.015 0.477 0.970], ...
    'BackgroundColor','w','BorderType','line');

% Left: velocity-corrected normal reconstructed for every boom pair.
layout=tiledlayout(panels(1),6,1,'TileSpacing','compact','Padding','compact');
ax=gobjects(6,1); ax(1)=nexttile(layout); hold(ax(1),'on');
for fileIndex=1:numel(records)
    plot(ax(1),records(fileIndex).t,pitchFromNormal(records(fileIndex).true_normal), ...
        'Color',[0.25 0.25 0.25],'LineWidth',3,'DisplayName', ...
        legendLabel(datasetLabels{fileIndex},'true pitch'));
    for pairIndex=1:pairCount
        plot(ax(1),records(fileIndex).t,pitchFromNormal(offline{fileIndex,pairIndex}.normal), ...
            'Color',pairColors(pairIndex,:),'LineStyle',recordStyles{1+mod(fileIndex-1,4)}, ...
            'LineWidth',1.8,'DisplayName',boomPairLabel(datasetLabels{fileIndex}, ...
            boomX(pairIndex),boomZ(pairIndex),offline{fileIndex,pairIndex}.normal_rmse));
    end
end
grid(ax(1),'on'); box(ax(1),'on'); ylabel(ax(1),'pitch [deg]'); ylim(ax(1),[-90 90]);
title(ax(1),'True pitch vs. offline estimated pitch');
legend(ax(1),'Location','eastoutside','Interpreter','none');
for component=1:3
    axisIndex=component+1; ax(axisIndex)=nexttile(layout); hold(ax(axisIndex),'on');
    for fileIndex=1:numel(records)
        plot(ax(axisIndex),records(fileIndex).t,records(fileIndex).true_normal(:,component), ...
            'Color',[0.76 0.76 0.76],'LineWidth',3,'HandleVisibility','off');
        for pairIndex=1:pairCount
            plot(ax(axisIndex),records(fileIndex).t,offline{fileIndex,pairIndex}.normal(:,component), ...
                'Color',pairColors(pairIndex,:),'LineStyle',recordStyles{1+mod(fileIndex-1,4)},'LineWidth',1.7);
        end
    end
    grid(ax(axisIndex),'on'); box(ax(axisIndex),'on'); ylim(ax(axisIndex),[-1.1 1.1]);
    ylabel(ax(axisIndex),sprintf('n_%s [-]',axisLetters(component)));
end
title(ax(2),'Offline normal (velocity correction ON)');
ax(5)=nexttile(layout); hold(ax(5),'on');
for fileIndex=1:numel(records)
    for pairIndex=1:pairCount
        errorDeg=angleBetweenRowsDeg(offline{fileIndex,pairIndex}.normal,records(fileIndex).true_normal);
        plot(ax(5),records(fileIndex).t,errorDeg,'Color',pairColors(pairIndex,:), ...
            'LineStyle',recordStyles{1+mod(fileIndex-1,4)},'LineWidth',1.7);
    end
end
grid(ax(5),'on'); box(ax(5),'on'); ylabel(ax(5),'angle [deg]'); ylim(ax(5),[0 90]); title(ax(5),'Error angle');
ax(6)=nexttile(layout); hold(ax(6),'on');
for fileIndex=1:numel(records)
    plot(ax(6),records(fileIndex).t,records(fileIndex).speed_filtered,'LineWidth',1.5);
end
grid(ax(6),'on'); box(ax(6),'on'); ylabel(ax(6),'||v_{EE}|| [m/s]'); ylim(ax(6),[-0.12 0.12]);
title(ax(6),'End-effector velocity norm'); addPlaneSequenceShading(ax,records(1));
finishOverviewAxes(ax,globalXLim);

% Right: reconstructed contact-force direction for every boom pair.
layout=tiledlayout(panels(2),6,1,'TileSpacing','compact','Padding','compact');
ax=gobjects(6,1); ax(1)=nexttile(layout); hold(ax(1),'on');
for fileIndex=1:numel(records)
    plot(ax(1),records(fileIndex).t,pitchFromNormal(records(fileIndex).true_normal), ...
        'Color',[0.25 0.25 0.25],'LineWidth',3,'DisplayName', ...
        legendLabel(datasetLabels{fileIndex},'true pitch'));
    for pairIndex=1:pairCount
        plot(ax(1),records(fileIndex).t,pitchFromNormal(offline{fileIndex,pairIndex}.force_unit), ...
            'Color',pairColors(pairIndex,:),'LineStyle',recordStyles{1+mod(fileIndex-1,4)}, ...
            'LineWidth',1.8,'DisplayName',boomPairLabel(datasetLabels{fileIndex}, ...
            boomX(pairIndex),boomZ(pairIndex),offline{fileIndex,pairIndex}.force_rmse));
    end
end
grid(ax(1),'on'); box(ax(1),'on'); ylabel(ax(1),'pitch [deg]'); ylim(ax(1),[-90 90]);
title(ax(1),'True pitch vs. reconstructed-force pitch');
legend(ax(1),'Location','eastoutside','Interpreter','none');
for component=1:3
    axisIndex=component+1; ax(axisIndex)=nexttile(layout); hold(ax(axisIndex),'on');
    for fileIndex=1:numel(records)
        plot(ax(axisIndex),records(fileIndex).t,records(fileIndex).true_normal(:,component), ...
            'Color',[0.76 0.76 0.76],'LineWidth',3,'HandleVisibility','off');
        for pairIndex=1:pairCount
            plot(ax(axisIndex),records(fileIndex).t,offline{fileIndex,pairIndex}.force_unit(:,component), ...
                'Color',pairColors(pairIndex,:),'LineStyle',recordStyles{1+mod(fileIndex-1,4)},'LineWidth',1.7);
        end
    end
    grid(ax(axisIndex),'on'); box(ax(axisIndex),'on'); ylim(ax(axisIndex),[-1.1 1.1]);
    ylabel(ax(axisIndex),sprintf('f_%s/||f||',axisLetters(component)));
end
title(ax(2),'Offline reconstructed force direction');
ax(5)=nexttile(layout); hold(ax(5),'on');
for fileIndex=1:numel(records)
    for pairIndex=1:pairCount
        errorDeg=angleBetweenRowsDeg(offline{fileIndex,pairIndex}.force_unit,records(fileIndex).true_normal);
        plot(ax(5),records(fileIndex).t,errorDeg,'Color',pairColors(pairIndex,:), ...
            'LineStyle',recordStyles{1+mod(fileIndex-1,4)},'LineWidth',1.7);
    end
end
grid(ax(5),'on'); box(ax(5),'on'); ylabel(ax(5),'angle [deg]'); ylim(ax(5),[0 90]); title(ax(5),'Error angle');
ax(6)=nexttile(layout); hold(ax(6),'on');
for fileIndex=1:numel(records)
    for pairIndex=1:pairCount
        plot(ax(6),records(fileIndex).t,vecnorm(offline{fileIndex,pairIndex}.force,2,2), ...
            'Color',pairColors(pairIndex,:),'LineStyle',recordStyles{1+mod(fileIndex-1,4)}, ...
            'LineWidth',1.5);
    end
end
grid(ax(6),'on'); box(ax(6),'on'); ylabel(ax(6),'||f_c|| [N]'); title(ax(6),'Reconstructed contact-force norm');
addPlaneSequenceShading(ax,records(1)); finishOverviewAxes(ax,globalXLim);
end

function force=reconstructContactForce(record,boomX,boomZ)
offsetBody=repmat([boomX 0 boomZ],numel(record.t),1);
offsetWorld=rotateBodyRows(offsetBody,record.attitude_rpy_deg);
inputTorqueWorld=rotateBodyRows(record.mob_input_torque_body,record.attitude_rpy_deg);
if strcmp(record.thrust_data,'comp')
    torqueBar=record.torque_bar_world;
else
    etaScale=record.thrust_effectiveness-1.0;
    torqueBar=record.mob_torque_world-etaScale.*inputTorqueWorld;
end
r2=sum(offsetWorld.^2,2);
parallel=offsetWorld.*(sum(offsetWorld.*record.f_bar,2)./r2);
transverse=-cross(offsetWorld,torqueBar,2)./r2;
force=parallel+transverse;
invalid=~all(isfinite([offsetWorld record.f_bar torqueBar]),2)|r2<=1e-12;
force(invalid,:)=nan;
end

function world=rotateBodyRows(body,rpyDeg)
world=nan(size(body));
for sample=1:size(body,1)
    if ~all(isfinite([body(sample,:) rpyDeg(sample,:)])), continue; end
    roll=deg2rad(rpyDeg(sample,1)); pitch=deg2rad(rpyDeg(sample,2)); yaw=deg2rad(rpyDeg(sample,3));
    cr=cos(roll); sr=sin(roll); cp=cos(pitch); sp=sin(pitch); cy=cos(yaw); sy=sin(yaw);
    rotation=[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr; ...
              sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr; ...
              -sp,cp*sr,cp*cr];
    world(sample,:)=(rotation*body(sample,:).').';
end
end

function body=rotateWorldRows(world,rpyDeg)
body=nan(size(world));
for sample=1:size(world,1)
    if ~all(isfinite([world(sample,:) rpyDeg(sample,:)])), continue; end
    roll=deg2rad(rpyDeg(sample,1)); pitch=deg2rad(rpyDeg(sample,2)); yaw=deg2rad(rpyDeg(sample,3));
    cr=cos(roll); sr=sin(roll); cp=cos(pitch); sp=sin(pitch); cy=cos(yaw); sy=sin(yaw);
    rotation=[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr; ...
              sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr; ...
              -sp,cp*sr,cp*cr];
    body(sample,:)=(rotation.'*world(sample,:).').';
end
end

function rmseDeg=angularRmse(estimate,trueNormal,t,timeLimits)
errorDeg=angleBetweenRowsDeg(estimate,trueNormal);
selected=isfinite(errorDeg)&isfinite(t);
if ~isempty(timeLimits), selected=selected&t>=timeLimits(1)&t<=timeLimits(2); end
rmseDeg=sqrt(mean(errorDeg(selected).^2,'omitnan'));
end

function label=boomPairLabel(datasetLabel,boomX,boomZ,rmseDeg)
label=sprintf('(x,z)=(%.4g, %.4g) m | RMSE %.2f deg',boomX,boomZ,rmseDeg);
if ~isempty(datasetLabel), label=sprintf('%s | %s',datasetLabel,label); end
end

function plotOverviewPanel(records,datasetLabels,recordColors,methodNames, ...
    methodStyles,methodWidths,axisNames,globalXLim,figurePosition)
overviewFigure=figure('Name','First experiment overview','Color','w', ...
    'Position',figurePosition);
% Keep only the two analysis panels, side by side at full height.
panelPositions=[0.015 0.015 0.477 0.970; ... % normal estimation
                0.508 0.015 0.477 0.970];    % force direction
overviewPanels=gobjects(2,1);
for panelIndex=1:2
    overviewPanels(panelIndex)=uipanel(overviewFigure,'Units','normalized', ...
        'Position',panelPositions(panelIndex,:),'BackgroundColor','w', ...
        'BorderType','line','BorderWidth',1,'HighlightColor',[0.55 0.55 0.55]);
end

% Left: pitch, estimated normal, angular error, and EE velocity norm.
layout=tiledlayout(overviewPanels(1),6,1,'TileSpacing','compact','Padding','compact');
ax=gobjects(6,1);
normalYLim={[-1.1 1.1],[-1.1 1.1],[-1.1 1.1],[0 90]};
ax(1)=nexttile(layout); hold(ax(1),'on');
for fileIndex=1:numel(records)
    plot(ax(1),records(fileIndex).t,pitchFromNormal(records(fileIndex).true_normal), ...
        'Color',[0.25 0.25 0.25],'LineWidth',3.0,'DisplayName', ...
        legendLabel(datasetLabels{fileIndex},'true pitch'));
    normalSet=normalMethodSet(records(fileIndex));
    for method=1:numel(methodNames)
        plot(ax(1),records(fileIndex).t,pitchFromNormal(normalSet(:,:,method)), ...
            'Color',curveColor(recordColors(fileIndex,:),method,numel(records)), ...
            'LineStyle',methodStyles{method},'LineWidth',methodWidths(method), ...
            'DisplayName',legendLabel(datasetLabels{fileIndex},methodNames{method}));
    end
end
grid(ax(1),'on'); box(ax(1),'on'); ylabel(ax(1),'pitch [deg]');
title(ax(1),'True pitch vs. estimated pitch');
ylim(ax(1),[-90 90]);
legend(ax(1),'Location','best','Interpreter','latex');
for component=1:3
    axisIndex=component+1;
    ax(axisIndex)=nexttile(layout); hold(ax(axisIndex),'on');
    for fileIndex=1:numel(records)
        plot(ax(axisIndex),records(fileIndex).t,records(fileIndex).true_normal(:,component), ...
            'Color',[0.76 0.76 0.76],'LineWidth',3.0,'DisplayName', ...
            legendLabel(datasetLabels{fileIndex},'true normal'));
        normalSet=normalMethodSet(records(fileIndex));
        for method=1:numel(methodNames)
            plot(ax(axisIndex),records(fileIndex).t,normalSet(:,component,method), ...
                'Color',curveColor(recordColors(fileIndex,:),method,numel(records)), ...
                'LineStyle',methodStyles{method},'LineWidth',methodWidths(method), ...
                'DisplayName',legendLabel(datasetLabels{fileIndex},methodNames{method}));
        end
    end
    grid(ax(axisIndex),'on'); box(ax(axisIndex),'on');
    ylabel(ax(axisIndex),sprintf('n_%s [-]',axisNames{component}));
    applyConfiguredYLim(ax(axisIndex),normalYLim{component});
end
title(ax(2),'Normal estimation');
ax(5)=nexttile(layout); hold(ax(5),'on');
for fileIndex=1:numel(records)
    normalSet=normalMethodSet(records(fileIndex));
    for method=1:numel(methodNames)
        errorDeg=angleBetweenRowsDeg(normalSet(:,:,method),records(fileIndex).true_normal);
        plot(ax(5),records(fileIndex).t,errorDeg,'Color', ...
            curveColor(recordColors(fileIndex,:),method,numel(records)), ...
            'LineStyle',methodStyles{method},'LineWidth',methodWidths(method));
    end
end
grid(ax(5),'on'); box(ax(5),'on'); ylabel(ax(5),'angle [deg]');
title(ax(5),'Error angle');
applyConfiguredYLim(ax(5),normalYLim{4});
ax(6)=nexttile(layout); hold(ax(6),'on');
for fileIndex=1:numel(records)
    plot(ax(6),records(fileIndex).t,records(fileIndex).speed_filtered, ...
        'Color',recordColors(fileIndex,:),'LineWidth',1.7,'DisplayName', ...
        legendLabel(datasetLabels{fileIndex},'EE velocity norm'));
end
grid(ax(6),'on'); box(ax(6),'on'); ylabel(ax(6),'||v_{EE}|| [m/s]');
title(ax(6),'End-effector velocity norm');
ylim(ax(6),[-0.12 0.12]);
addPlaneSequenceShading(ax,records(1)); finishOverviewAxes(ax,globalXLim);

forceNames={'$\hat{f}_{\ell}$','$\bar{f}_{\ell}$','$\hat{f}_{c}$'};
forceStyles={':','--','-'};
% Right: force-direction pitch, components, angular error, and battery voltage.
layout=tiledlayout(overviewPanels(2),6,1,'TileSpacing','compact','Padding','compact');
ax=gobjects(6,1);
ax(1)=nexttile(layout); hold(ax(1),'on');
for fileIndex=1:numel(records)
    plot(ax(1),records(fileIndex).t,pitchFromNormal(records(fileIndex).true_normal), ...
        'Color',[0.25 0.25 0.25],'LineWidth',3.0,'DisplayName', ...
        legendLabel(datasetLabels{fileIndex},'true pitch'));
    forceSet=forceCueSet(records(fileIndex));
    for method=1:3
        plot(ax(1),records(fileIndex).t,pitchFromNormal(forceSet(:,:,method)), ...
            'Color',curveColor(recordColors(fileIndex,:),method,numel(records)), ...
            'LineStyle',forceStyles{method},'LineWidth',1.5,'DisplayName', ...
            legendLabel(datasetLabels{fileIndex},forceNames{method}));
    end
end
grid(ax(1),'on'); box(ax(1),'on'); ylabel(ax(1),'pitch [deg]');
title(ax(1),'True pitch vs. force-direction pitch');
ylim(ax(1),[-90 90]);
legend(ax(1),'Location','best','Interpreter','latex');
for component=1:3
    axisIndex=component+1;
    ax(axisIndex)=nexttile(layout); hold(ax(axisIndex),'on');
    for fileIndex=1:numel(records)
        plot(ax(axisIndex),records(fileIndex).t,records(fileIndex).true_normal(:,component), ...
            'Color',[0.76 0.76 0.76],'LineWidth',3.0,'DisplayName', ...
            legendLabel(datasetLabels{fileIndex},'true normal'));
        forceSet=forceCueSet(records(fileIndex));
        for method=1:3
            plot(ax(axisIndex),records(fileIndex).t,forceSet(:,component,method), ...
                'Color',curveColor(recordColors(fileIndex,:),method,numel(records)), ...
                'LineStyle',forceStyles{method},'LineWidth',1.5,'DisplayName', ...
                legendLabel(datasetLabels{fileIndex},forceNames{method}));
        end
    end
    grid(ax(axisIndex),'on'); box(ax(axisIndex),'on');
    ylabel(ax(axisIndex),sprintf('%s [-]',axisNames{component})); ylim(ax(axisIndex),[-1.1 1.1]);
end
title(ax(2),'Force direction vs. true normal');
ax(5)=nexttile(layout); hold(ax(5),'on');
for fileIndex=1:numel(records)
    forceSet=forceCueSet(records(fileIndex));
    for method=1:3
        errorDeg=angleBetweenRowsDeg(forceSet(:,:,method),records(fileIndex).true_normal);
        plot(ax(5),records(fileIndex).t,errorDeg,'Color', ...
            curveColor(recordColors(fileIndex,:),method,numel(records)), ...
            'LineStyle',forceStyles{method},'LineWidth',1.5);
    end
end
grid(ax(5),'on'); box(ax(5),'on'); ylabel(ax(5),'angle [deg]');
title(ax(5),'Error angle'); ylim(ax(5),[0 90]);
ax(6)=nexttile(layout); hold(ax(6),'on');
yyaxis(ax(6),'left');
for fileIndex=1:numel(records)
    plot(ax(6),records(fileIndex).t,records(fileIndex).battery_voltage, ...
        'Color',recordColors(fileIndex,:),'LineWidth',1.7,'DisplayName', ...
        legendLabel(datasetLabels{fileIndex},'battery voltage'));
end
ylabel(ax(6),'V_{bat} [V]');
yyaxis(ax(6),'right'); hold(ax(6),'on');
for fileIndex=1:numel(records)
    plot(ax(6),records(fileIndex).t,records(fileIndex).thrust_effectiveness, ...
        '--','Color',recordColors(fileIndex,:),'LineWidth',1.7,'DisplayName', ...
        legendLabel(datasetLabels{fileIndex},'thrust effectiveness'));
end
ylabel(ax(6),'$\hat{\eta}_T$ [-]','Interpreter','latex');
ylim(ax(6),[0.7 1.1]);
grid(ax(6),'on'); box(ax(6),'on');
title(ax(6),'Battery voltage and thrust effectiveness estimation');
legend(ax(6),'Location','best','Interpreter','latex');
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

function [normalRmseByRun,forceRmseByRun]=plotEnsembleErrorStatistics(records,normalMethodNames,timeLimits, ...
    methodIndices,comparisonLabels)
% Time-resolved mean +/- SD and per-run RMSE comparison.
metricLimits=timeLimits;
commonStart=max(arrayfun(@(r) min(r.t),records));
commonStop=min(arrayfun(@(r) max(r.t),records));
if isempty(timeLimits)
    timeLimits=[commonStart commonStop];
else
    timeLimits=[max(timeLimits(1),commonStart) min(timeLimits(2),commonStop)];
end
if timeLimits(2)<=timeLimits(1)
    error('FlyingPen:NoCommonTime','Selected records have no overlapping analysis interval.');
end
fprintf(['Ensemble time series: common observed support %.6f to %.6f s. ' ...
    'RMSE uses each retained run, without padding or extrapolation.\n'],timeLimits);
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
    normalSet=normalMethodSet(records(fileIndex));
    for method=1:size(normalSet,3)
        errorDeg=angleBetweenRowsDeg(normalSet(:,:,method),records(fileIndex).true_normal);
        normalErrors(:,fileIndex,method)=interpolateFiniteSeries( ...
            records(fileIndex).t,errorDeg,timeGrid);
    end
    % Velocity correction changes the normal estimator, not the force vector;
    % therefore M3 and M4 have the same force-direction series.
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
for fileIndex=1:numel(records)
    r=records(fileIndex);
    normalSet=normalMethodSet(r);
    forceSet=cat(3,r.f_raw_unit,r.f_bar_unit,r.f_c_unit,r.f_c_unit);
    for method=1:numel(methodIndices)
        index=methodIndices(method);
        normalRmseByRun(fileIndex,method)=angularRmse( ...
            normalSet(:,:,index),r.true_normal,r.t,metricLimits);
        forceRmseByRun(fileIndex,method)=angularRmse( ...
            forceSet(:,:,index),r.true_normal,r.t,metricLimits);
    end
end

figure('Name','Repeated-experiment estimation performance','Color','w', ...
    'Position',[110 80 1480 900]);
layout=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
ax=nexttile(layout,1);
forceDisplayMask=methodIndices~=3; % omit duplicate M3; retain full-method M4
allForceLabels={'$\hat{f}_{\ell}$','$\bar{f}_{\ell}$','$\hat{f}_c$','$\hat{f}_c$'};
forceLabels=allForceLabels(methodIndices);
plotErrorMeanAndSpread(ax,timeGrid,forceErrors(:,:,forceDisplayMask), ...
    forceLabels(forceDisplayMask),methodColors(forceDisplayMask,:));
title(ax,sprintf('Force-direction Error Time Series (n = %d)',numel(records)));
ylabel(ax,'Force-direction Error [deg]'); ylim(ax,[0 90]);
ax=nexttile(layout,2);
plotErrorMeanAndSpread(ax,timeGrid,normalErrors,comparisonLabels,methodColors);
title(ax,sprintf('Normal Estimation Error Time Series (n = %d)',numel(records)));
ylabel(ax,'Normal Estimation Error [deg]'); ylim(ax,[0 90]);
ax=nexttile(layout,3);
plotRmseComparison(ax,forceRmseByRun(:,forceDisplayMask), ...
    forceLabels(forceDisplayMask));
title(ax,'Quantitative Force-direction Performance'); ylim(ax,[0 55]);
ax=nexttile(layout,4);
plotRmseComparison(ax,normalRmseByRun,comparisonLabels);
title(ax,'Quantitative Normal-estimation Performance'); ylim(ax,[0 55]);
end


function valuesOnGrid=interpolateFiniteSeries(t,values,timeGrid)
% Interpolate only inside contiguous valid spans, never through missing data.
t=t(:); values=values(:); valuesOnGrid=nan(size(timeGrid));
dt=diff(t); dt=dt(isfinite(dt)&dt>0);
if isempty(dt), return; end
maxGap=3*median(dt);
valid=isfinite(t)&isfinite(values);
indices=find(valid);
if isempty(indices), return; end
breaks=[true;diff(indices)>1|diff(t(indices))>maxGap];
starts=find(breaks);
stops=[starts(2:end)-1;numel(indices)];
for run=1:numel(starts)
    ii=indices(starts(run):stops(run));
    if numel(ii)<2, continue; end
    onGrid=timeGrid>=t(ii(1))&timeGrid<=t(ii(end));
    valuesOnGrid(onGrid)=interp1(t(ii),values(ii),timeGrid(onGrid),'linear');
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

function pitchDeg=pitchFromNormal(normal)
% Ignore world yaw and recover the wall inclination from its X-Z normal.
% With this script's inward normal convention, [-cos(pitch),0,sin(pitch)],
% atan2(n_z,-n_x) returns the configured -23/0/21 deg plane pitch.
pitchDeg=atan2d(normal(:,3),-normal(:,1));
pitchDeg(~all(isfinite(normal(:,[1 3])),2))=nan;
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


function folder=resolveCsvDataDirectory(explicitFolder,scriptFullPath)
% Resolve only the input directory; do not cd(), alter CSVs, or reset estimates.
% Priority: explicit path > executing saved script > matching active editor
% file > Current Folder. Automatic choices must contain CSVs and must not be
% MATLAB Editor scratch directories. Otherwise ask for the data folder.
folder='';
if isstring(explicitFolder)
    if ~isscalar(explicitFolder)
        error('FlyingPen:CsvFolderType','csv_data_dir must be a text scalar.');
    end
    explicitFolder=char(explicitFolder);
end
if ~ischar(explicitFolder)||(~isempty(explicitFolder)&&~isrow(explicitFolder))
    error('FlyingPen:CsvFolderType','csv_data_dir must be a character vector or string scalar.');
end
explicitFolder=strtrim(explicitFolder);
if ~isempty(explicitFolder)
    if ~isfolder(explicitFolder)
        error('FlyingPen:CsvFolderMissing', ...
            'csv_data_dir does not exist: %s',explicitFolder);
    end
    if ~folderContainsCsv(explicitFolder)
        error('FlyingPen:NoCsvInFolder', ...
            'No .csv files in csv_data_dir: %s. Choose the folder containing the original CSVs.', ...
            explicitFolder);
    end
    folder=explicitFolder;
    return;
end

candidates={};
scriptFullPath=char(scriptFullPath);
[scriptFolder,scriptName]=fileparts(scriptFullPath);
if ~isempty(scriptFolder)
    candidates{end+1}=scriptFolder;
end
% Editor information is only a fallback, and only for this script's filename.
% Do not mistake an unrelated active editor tab for the executing script.
try
    activeFile=char(matlab.desktop.editor.getActiveFilename);
    [activeFolder,activeName,activeExtension]=fileparts(activeFile);
    if ~isempty(activeFile)&&isfile(activeFile)&&strcmpi(activeExtension,'.m')&& ...
            (isempty(scriptName)||strcmpi(activeName,scriptName))
        candidates{end+1}=activeFolder;
    end
catch
    % Desktop editor is unavailable in some execution modes; use other paths.
end
candidates{end+1}=pwd;
candidates=unique(candidates,'stable');

% Pick the first real folder WITH CSVs; never accept a path just because it exists.
browseStart='';
for k=1:numel(candidates)
    candidate=char(candidates{k});
    if isempty(candidate)||~isfolder(candidate)||isEditorScratchFolder(candidate)
        continue;
    end
    if isempty(browseStart), browseStart=candidate; end
    if folderContainsCsv(candidate)
        folder=candidate;
        return;
    end
end

% Neither the saved script folder nor Current Folder has the data.
% Fall back to a normal user directory, not the local function's temp folder.
if isempty(browseStart)
    homeFolder=getenv('HOME');
    if isempty(homeFolder), homeFolder=getenv('USERPROFILE'); end
    if ~isempty(homeFolder)&&isfolder(homeFolder)&&~isEditorScratchFolder(homeFolder)
        browseStart=homeFolder;
    else
        browseStart=filesep;
    end
end
fprintf(['No CSVs were found beside the saved script or in Current Folder.\n' ...
    'Select the folder containing the ORIGINAL CSV recordings.\n']);
while true
    chosen=uigetdir(browseStart,'Select the folder containing the original CSVs');
    if isequal(chosen,0), return; end
    if folderContainsCsv(chosen)
        folder=char(chosen);
        return;
    end
    warning('FlyingPen:NoCsvInFolder', ...
        'No .csv files in %s. Choose the data folder, or cancel.',chosen);
    browseStart=char(chosen);
end
end

function tf=folderContainsCsv(folder)
entries=dir(fullfile(folder,'*.csv'));
tf=~isempty(entries)&&any(~[entries.isdir]);
end

function tf=isEditorScratchFolder(folder)
% The screenshot showed Editor_cjwx. Match a whole path component, not a
% substring such as a legitimate project named MyEditor_Project.
normalized=strrep(char(folder),'\','/');
tf=~isempty(regexp(normalized,'(^|/)Editor_[^/]*(/|$)','once'));
end

%% Paper-figure helpers: added without modifying the estimator pipeline

function [stats,fig7,fig8]=makeFirstExperimentPaperResults(records,metricLimits,opt)
if isempty(records), error('FlyingPen:NoRecords','No records to summarize.'); end
validateattributes(opt.stationary_settle_s,{'numeric'},{'scalar','finite','nonnegative'});
validateattributes(opt.stationary_end_trim_s,{'numeric'},{'scalar','finite','nonnegative'});
validateattributes(opt.wind_padding_s,{'numeric'},{'vector','numel',2,'finite','nonnegative'});
validateattributes(opt.band_alpha,{'numeric'},{'scalar','finite','>=',0,'<=',1});
validateattributes(opt.stationary_speed_check_mps,{'numeric'},{'scalar','finite','positive'});
validateattributes(opt.method_colors,{'numeric'},{'size',[4 3],'finite','>=',0,'<=',1});
validateattributes(opt.fig7_row_weights,{'numeric'},{'vector','numel',5,'finite','positive'});
validateattributes(opt.fig7_position,{'numeric'},{'vector','numel',4,'finite'});
validateattributes(opt.fig8_position,{'numeric'},{'vector','numel',4,'finite'});
validateattributes(opt.fig7_size_in,{'numeric'},{'vector','numel',2,'finite','positive'});
validateattributes(opt.fig8_size_in,{'numeric'},{'vector','numel',2,'finite','positive'});
validateattributes(opt.fig7_header_font_size,{'numeric'},{'scalar','finite','positive'});
validateattributes(opt.fig8_legend_font_size,{'numeric'},{'scalar','finite','positive'});
if numel(opt.component_bands)~=3
    error('FlyingPen:BandConfig','paper.component_bands must have three entries.');
end
n=numel(records);
stats.options=opt;
stats.thrust_data=records(1).thrust_data;
stats.n_trials=n;
stats.trial_ids=[records.trial_id].';
stats.normal_rmse_by_trial=nan(n,4,2); % trial x method x {overall,wind}
stats.force_rmse_by_trial=nan(n,3,2);  % trial x cue x {no-wind hold,wind hold}
stats.normal_samples_by_trial=zeros(n,2);
stats.force_samples_by_trial=zeros(n,2);
stats.normal_available_samples_by_trial=zeros(n,4,2);
stats.force_available_samples_by_trial=zeros(n,3,2);
windowData=nan(n,13);
voltageAudit=nan(n,5);
for k=1:n
    r=records(k);
    visits=paperPlanarVisits(r);
    wind=paperWindWindow(r,visits,opt);
    holds=paperStationaryPlaneTwoWindows(r,visits,wind,opt);
    w.visits=visits;
    w.wind=wind;
    w.no_wind_hold=holds(1,:);
    w.wind_hold=holds(2,:);
    w.wind_is_proxy=isempty(paperWindOverride(r,opt));
    if k==1, stats.windows=w; else, stats.windows(k)=w; end

    normalSet=normalMethodSet(r);
    forceSet=forceCueSet(r);
    normalError=nan(numel(r.t),4);
    forceError=nan(numel(r.t),3);
    for m=1:4
        normalError(:,m)=angleBetweenRowsDeg(normalSet(:,:,m),r.true_normal);
    end
    for m=1:3
        forceError(:,m)=angleBetweenRowsDeg(forceSet(:,:,m),r.true_normal);
    end
    overall=isfinite(r.t);
    if ~isempty(metricLimits), overall=overall&timeMask(r.t,metricLimits); end
    auditMask=overall&isfinite(r.battery_voltage)&isfinite(r.voltage_model_eta)& ...
        isfinite(r.thrust_effectiveness_firmware);
    voltageAudit(k,:)=[r.trial_id,mean(r.battery_voltage(auditMask),'omitnan'), ...
        mean(r.voltage_model_eta(auditMask),'omitnan'), ...
        mean(r.thrust_effectiveness_firmware(auditMask),'omitnan'), ...
        mean(abs(r.voltage_model_eta(auditMask)-r.thrust_effectiveness_firmware(auditMask)),'omitnan')];
    windMask=overall&timeMask(r.t,wind);
    regionMasks=[overall windMask];
    for region=1:2
        % Common-valid sample set: apples-to-apples comparison of M1--M4.
        valid=regionMasks(:,region)&all(isfinite(normalError),2);
        stats.normal_samples_by_trial(k,region)=nnz(valid);
        for m=1:4
            stats.normal_available_samples_by_trial(k,m,region)= ...
                nnz(regionMasks(:,region)&isfinite(normalError(:,m)));
            stats.normal_rmse_by_trial(k,m,region)=paperRmse(normalError(:,m),valid);
        end
        if nnz(valid)<2
            warning('FlyingPen:PaperNormalMetric','Trial %d: insufficient common valid samples in region %d.',r.trial_id,region);
        end
    end
    speedMean=nan(1,2); speedMax=nan(1,2); speedAbove=nan(1,2);
    for condition=1:2
        % Half-open hold: exclude the first sample of the next motion command.
        regionMask=overall&r.t>=holds(condition,1)&r.t<holds(condition,2)& ...
            r.surface_section==3;
        valid=regionMask&all(isfinite(forceError),2);
        stats.force_samples_by_trial(k,condition)=nnz(valid);
        for m=1:3
            stats.force_available_samples_by_trial(k,m,condition)= ...
                nnz(regionMask&isfinite(forceError(:,m)));
            stats.force_rmse_by_trial(k,m,condition)=paperRmse(forceError(:,m),valid);
        end
        s=r.speed_filtered(regionMask&isfinite(r.speed_filtered));
        if ~isempty(s)
            speedMean(condition)=mean(s); speedMax(condition)=max(s);
            speedAbove(condition)=100*mean(s>opt.stationary_speed_check_mps);
        end
        if nnz(valid)<2
            warning('FlyingPen:PaperForceMetric','Trial %d: no valid plane-2 stationary comparison in condition %d.',r.trial_id,condition);
        end
    end
    windowData(k,:)=[r.trial_id wind holds(1,:) holds(2,:) ...
        speedMean speedMax speedAbove];
end
stats.window_audit=array2table(windowData,'VariableNames', ...
    {'Trial','WindStart_s','WindStop_s','NoWindHoldStart_s','NoWindHoldStop_s', ...
    'WindHoldStart_s','WindHoldStop_s','NoWindSpeedMean_mps','WindSpeedMean_mps', ...
    'NoWindSpeedMax_mps','WindSpeedMax_mps','NoWindAboveCheck_pct','WindAboveCheck_pct'});
stats.voltage_model_audit=array2table(voltageAudit,'VariableNames', ...
    {'Trial','BatteryMean_V','VoltageModelEtaMean','AdaptiveEtaMean','EtaAbsDifferenceMean'});
[mu,sd,count]=paperAcrossTrials(stats.normal_rmse_by_trial,1,1);
stats.normal_mean=reshape(mu,4,2);
stats.normal_sd=reshape(sd,4,2);
stats.normal_n=reshape(count,4,2);
[mu,sd,count]=paperAcrossTrials(stats.force_rmse_by_trial,1,1);
stats.force_mean=reshape(mu,3,2);
stats.force_sd=reshape(sd,3,2);
stats.force_n=reshape(count,3,2);
stats=paperMakeMetricTables(stats);
stats.ensemble=paperBuildEnsemble(records,stats.windows,metricLimits,opt);
printPaperExperimentTables(stats,opt);
fig7=plotPaperFigure7(stats,opt);
fig8=plotPaperFigure8(stats,opt);
end

function visits=paperPlanarVisits(r)
% Complete planar-visit ENVELOPES for this experiment's prescribed up/down
% sweep. Splitting at the first descent COMMAND prevents brief plane/curve
% classification jitter from truncating the second plane-2 visit (trials 5/9).
% This repairs window metadata only: no signal is interpolated or relabeled.
visits=struct('plane',{},'start',{},'stop',{});
u=r.u2_des(:); t=r.t(:);
uFinite=abs(u(isfinite(u)));
if isempty(uFinite), return; end
threshold=max(1e-6,0.1*max(uFinite));
firstDown=find(isfinite(t)&isfinite(u)&u< -threshold,1,'first');
if isempty(firstDown)
    warning('FlyingPen:PaperSequence','%s: descending command not found; planar visits are undefined.',r.label);
    return;
end
turn=t(firstDown);
inside=isfinite(t)&isfinite(r.surface_section)& ...
    t>=r.surface_loop_time(1)&t<=r.surface_loop_time(2);
sections=[1 3 5 3 1]; planeIds=[1 2 3 2 1];
for j=1:5
    mask=inside&r.surface_section==sections(j);
    if j<=2, mask=mask&t<turn; elseif j>=4, mask=mask&t>=turn; end
    ii=find(mask);
    if isempty(ii), continue; end
    visits(end+1)=struct('plane',planeIds(j),'start',t(ii(1)), ...
        'stop',t(ii(end))); %#ok<AGROW>
end
if numel(visits)~=5||~isequal([visits.plane],[1 2 3 2 1])
    warning('FlyingPen:PaperSequence','Trial %d: expected planar sequence 1-2-3-2-1 is incomplete.',r.trial_id);
end
end

function override=paperWindOverride(r,opt)
override=[];
v=opt.wind_windows;
if isempty(v), return; end
if ~isnumeric(v)||any(~isfinite(v(:)))
    error('FlyingPen:WindOverride','paper.wind_windows must be a finite numeric array.');
end
if isequal(size(v),[1 2])
    override=v;
elseif size(v,2)==3
    if any(v(:,1)~=fix(v(:,1)))||any(v(:,1)<1)||numel(unique(v(:,1)))~=size(v,1)
        error('FlyingPen:WindOverride','Wind override trial IDs must be unique positive integers.');
    end
    if any(v(:,3)<=v(:,2))
        error('FlyingPen:WindOverride','Every wind override stop must exceed its start.');
    end
    row=find(v(:,1)==r.trial_id);
    if ~isempty(row), override=v(row,2:3); end
else
    error('FlyingPen:WindOverride','Use [start stop] or rows [trial_id start stop].');
end
if ~isempty(override)&&override(2)<=override(1)
    error('FlyingPen:WindOverride','Wind stop time must exceed start time.');
end
end

function window=paperWindWindow(r,visits,opt)
window=paperWindOverride(r,opt);
if isempty(window)
    p2=find([visits.plane]==2);
    if numel(p2)<2, window=[nan nan]; return; end
    v=visits(p2(2));
    window=[v.start-opt.wind_padding_s(1),v.stop+opt.wind_padding_s(2)];
end
window=[max(window(1),min(r.t)),min(window(2),max(r.t))];
if window(2)<=window(1), window=[nan nan]; end
end

function windows=paperStationaryPlaneTwoWindows(r,visits,wind,opt)
% Longest commanded-zero hold overlapping each plane-2 visit.
% Apply the SAME fixed settling/end exclusions to both conditions. Realized
% speed is reported separately, never used to discard bad force/error samples.
windows=nan(2,2);
p2=find([visits.plane]==2);
if numel(p2)<2, return; end
u=r.u2_des(:); t=r.t(:);
uFinite=abs(u(isfinite(u)));
threshold=max(1e-6,0.1*max(uFinite));
zero=isfinite(u)&abs(u)<=threshold;
if isfield(r,'u1_des')&&any(isfinite(r.u1_des))
    u1=r.u1_des(:);
    threshold1=max(1e-6,0.1*max(abs(u1(isfinite(u1)))));
    zero=zero&isfinite(u1)&abs(u1)<=threshold1;
end
[startIdx,stopIdx]=paperRuns(zero);
for condition=1:2
    v=visits(p2(condition));
    candidates=zeros(0,3);
    for j=1:numel(startIdx)
        a=startIdx(j); b=stopIdx(j);
        if t(b)<v.start||t(a)>v.stop, continue; end
        % The end is the first following command sample, not the last zero.
        if b<numel(t), stop=t(b+1); else, stop=t(b); end
        start=max(t(a)+opt.stationary_settle_s,v.start);
        stop=min(stop-opt.stationary_end_trim_s,v.stop);
        if stop>start, candidates(end+1,:)=[start stop stop-start]; end %#ok<AGROW>
    end
    if isempty(candidates), continue; end
    [~,best]=max(candidates(:,3));
    windows(condition,:)=candidates(best,1:2);
end
% Do not silently alter a hold if a user-defined wind window conflicts with it.
if all(isfinite(wind))&&all(isfinite(windows(1,:)))&& ...
        windows(1,1)<wind(2)&&windows(1,2)>wind(1)
    error('FlyingPen:NoWindHoldOverlap', ...
        'Trial %d: the no-wind hold overlaps paper.wind_windows; review the window definition.',r.trial_id);
end
if all(isfinite(wind))&&all(isfinite(windows(2,:)))&& ...
        (windows(2,1)<wind(1)||windows(2,2)>wind(2))
    warning('FlyingPen:WindHoldCoverage', ...
        'Trial %d: its stationary wind hold is not wholly inside the wind evaluation window.',r.trial_id);
end
end

function value=paperRmse(errorDeg,mask)
x=errorDeg(mask&isfinite(errorDeg));
if numel(x)<2, value=nan; else, value=sqrt(mean(x.^2)); end
end

function [mu,sd,n]=paperAcrossTrials(x,dimension,minCount)
% Explicit sample SD (N-1). One trial has an observable mean but undefined SD.
n=sum(isfinite(x),dimension);
y=x; y(~isfinite(y))=0;
mu=sum(y,dimension)./max(n,1);
delta=bsxfun(@minus,x,mu); delta(~isfinite(delta))=0;
sd=sqrt(sum(delta.^2,dimension)./max(n-1,1));
mu(n<minCount|n==0)=nan;
sd(n<max(2,minCount))=nan;
end

function stats=paperMakeMetricTables(stats)
method={'M1';'M2';'M3';'M4 (Proposed)'};
thrust=[false;true;true;true]; recon=[false;false;true;true];
projection=[true;true;false;true];
mu=stats.normal_mean; sd=stats.normal_sd; n=stats.normal_n;
overall=cell(4,1); wind=cell(4,1);
for j=1:4
    overall{j}=sprintf('%.3f +/- %.3f',mu(j,1),sd(j,1));
    wind{j}=sprintf('%.3f +/- %.3f',mu(j,2),sd(j,2));
end
stats.table_I=table(method,thrust,recon,projection,overall,wind,n(:,1),n(:,2), ...
    'VariableNames',{'Method','ThrustComp','ForceRecon','VelProj', ...
    'OverallRMSE_deg','WindRMSE_deg','nOverall','nWind'});
stats.table_I_numeric=table(method,thrust,recon,projection,mu(:,1),sd(:,1), ...
    mu(:,2),sd(:,2),n(:,1),n(:,2),'VariableNames', ...
    {'Method','ThrustComp','ForceRecon','VelProj','OverallMean_deg','OverallSD_deg', ...
    'WindMean_deg','WindSD_deg','nOverall','nWind'});
condition=repelem({'No wind';'Wind'},3,1);
cue=repmat({'Raw MOB';'Thrust-corrected';'Reconstructed'},2,1);
stats.fig8_numeric=table(condition,cue,stats.force_mean(:),stats.force_sd(:), ...
    stats.force_n(:),'VariableNames',{'Condition','ForceCue','MeanRMSE_deg','SDRMSE_deg','nTrials'});
trial=repmat(stats.trial_ids,4,1);
methodLong=repelem(method,stats.n_trials,1);
x=stats.normal_rmse_by_trial;
stats.normal_per_trial=table(trial,methodLong,reshape(x(:,:,1),[],1), ...
    reshape(x(:,:,2),[],1),'VariableNames',{'Trial','Method','OverallRMSE_deg','WindRMSE_deg'});
x=stats.force_rmse_by_trial;
trial=repmat(stats.trial_ids,6,1);
conditionLong=repelem(condition,stats.n_trials,1);
cueLong=repelem(cue,stats.n_trials,1);
stats.force_per_trial=table(trial,conditionLong,cueLong,x(:), ...
    'VariableNames',{'Trial','Condition','ForceCue','AngularRMSE_deg'});
end

function e=paperBuildEnsemble(records,windows,limits,opt)
% Only common observed TIME support. No time warping, output-dependent shifts,
% extrapolation, vector-mean renormalization, or interpolation through gaps.
a=max(arrayfun(@(r) min(r.t),records));
b=min(arrayfun(@(r) max(r.t),records));
if ~isempty(limits), a=max(a,limits(1)); b=min(b,limits(2)); end
if b<=a, error('FlyingPen:PaperSupport','No common observed time support.'); end
samplePeriods=[];
for k=1:numel(records)
    dt=diff(records(k).t); samplePeriods=[samplePeriods;dt(isfinite(dt)&dt>0)]; %#ok<AGROW>
end
step=opt.grid_step_s;
if isempty(step), step=max(0.01,median(samplePeriods)); end
validateattributes(step,{'numeric'},{'scalar','finite','positive'});
t=(a:step:b).';
if numel(t)<2, error('FlyingPen:PaperGrid','The selected time grid has fewer than two points.'); end
nr=numel(records); nt=numel(t);
speed=nan(nt,nr); reference=nan(nt,nr,3);
normal=nan(nt,nr,3,4); errors=nan(nt,nr,4);
curve=nan(nt,nr); friction=nan(nt,nr); wind=nan(nt,nr);
for k=1:nr
    r=records(k);
    speed(:,k)=interpolateFiniteSeries(r.t,r.speed_filtered,t);
    normalSet=normalMethodSet(r);
    allValid=all(isfinite(r.true_normal),2);
    for m=1:4, allValid=allValid&all(isfinite(normalSet(:,:,m)),2); end
    for c=1:3
        q=r.true_normal(:,c); q(~allValid)=nan;
        reference(:,k,c)=interpolateFiniteSeries(r.t,q,t);
        for m=1:4
            q=normalSet(:,c,m); q(~allValid)=nan;
            normal(:,k,c,m)=interpolateFiniteSeries(r.t,q,t);
        end
    end
    for m=1:4
        q=angleBetweenRowsDeg(normalSet(:,:,m),r.true_normal);
        q(~allValid)=nan;
        errors(:,k,m)=interpolateFiniteSeries(r.t,q,t);
    end
    q=double(ismember(r.surface_section,[2 4]));
    q(~isfinite(r.surface_section))=nan;
    curve(:,k)=interp1(r.t,q,t,'nearest',nan);
    friction(:,k)=interp1(r.t,double(r.friction_mask),t,'nearest',nan);
    if all(isfinite(windows(k).wind))
        wind(:,k)=double(timeMask(t,windows(k).wind));
    end
end
minimum=1; if opt.require_all_trials, minimum=nr; end
[e.speed_mean,e.speed_sd,e.speed_n]=paperAcrossTrials(speed,2,minimum);
[mu,sd,n]=paperAcrossTrials(reference,2,minimum);
e.reference_mean=reshape(mu,nt,3); e.reference_sd=reshape(sd,nt,3);
e.reference_n=reshape(n,nt,3);
[mu,sd,n]=paperAcrossTrials(normal,2,minimum);
e.normal_mean=reshape(mu,nt,3,4); e.normal_sd=reshape(sd,nt,3,4);
e.normal_n=reshape(n,nt,3,4);
[mu,sd,n]=paperAcrossTrials(errors,2,minimum);
e.error_mean=reshape(mu,nt,4); e.error_sd=reshape(sd,nt,4);
e.error_n=reshape(n,nt,4);
e.curve_fraction=mean(curve,2,'omitnan');
e.friction_fraction=mean(friction,2,'omitnan');
e.wind_fraction=mean(wind,2,'omitnan');
e.t=t; e.grid_step_s=step; e.time_support=[a b];
e.minimum_trials_for_display=minimum;
% Retained for auditing / changing the display without rerunning estimators.
e.speed_by_trial=speed; e.reference_by_trial=reference;
e.normal_by_trial=normal; e.error_by_trial=errors;
end

function printPaperExperimentTables(stats,opt)
fprintf('\n%s\n',repmat('=',1,108));
fprintf('TABLE I | Surface-normal estimation | n = %d selected trials\n',stats.n_trials);
fprintf('Motor-thrust source for M1--M4: %s\n',stats.thrust_data);
fprintf('Trial IDs:'); fprintf(' %d',stats.trial_ids); fprintf('\n');
fprintf('Entries = mean +/- SAMPLE SD of per-trial angular RMSEs [deg], not pooled RMSE.\n');
fprintf('Overall = each cropped run. Wind = moving + stationary within the printed wind window.\n');
fprintf('Identical valid timestamps are used for M1--M4 within each trial and region.\n\n');
fprintf('%-16s %11s %11s %9s %24s %24s\n','Method','Thrust comp','Force recon','Vel proj','Overall RMSE [deg]','Wind RMSE [deg]');
fprintf('%s\n',repmat('-',1,108));
for m=1:4
    row=stats.table_I(m,:);
    fprintf('%-16s %11d %11d %9d %24s %24s\n',row.Method{1},row.ThrustComp, ...
        row.ForceRecon,row.VelProj,row.OverallRMSE_deg{1},row.WindRMSE_deg{1});
end
fprintf('0 = disabled; 1 = enabled. Valid trial counts: overall [');
fprintf(' %d',stats.normal_n(:,1)); fprintf(' ], wind [');
fprintf(' %d',stats.normal_n(:,2)); fprintf(' ].\n');
if stats.n_trials<2
    fprintf('NOTE: n=1 has no between-trial sample SD; SD is NaN and no SD band is drawn.\n');
end
if any([stats.windows.wind_is_proxy])
    fprintf('NOTE: Wind windows marked as proxy use geometry, not a recorded fan/airflow flag.\n');
end
fprintf('\nFIG. 8 | Force-direction angular RMSE | plane-2 stationary holds\n');
fprintf('Fixed exclusions after stop / before restart: %.2f / %.2f s.\n', ...
    opt.stationary_settle_s,opt.stationary_end_trim_s);
fprintf('No force/error/speed-based sample rejection is applied to these holds.\n');
disp(stats.fig8_numeric);
fprintf('Voltage-model audit (fixed CF21BL polynomial; no fitted trial parameters):\n');
disp(stats.voltage_model_audit);
if opt.print_window_audit
    fprintf('Window audit [plotted time, s]:\n');
    fprintf('%-6s %-22s %-22s %-22s %-8s\n','Trial','Wind metric','Fig8 no-wind hold','Fig8 wind hold','Proxy');
    for k=1:stats.n_trials
        w=stats.windows(k);
        fprintf('%-6d %9.3f -- %9.3f %9.3f -- %9.3f %9.3f -- %9.3f %8d\n', ...
            stats.trial_ids(k),w.wind,w.no_wind_hold,w.wind_hold,w.wind_is_proxy);
    end
    fprintf('Realized speed audit (not a selection rule), threshold %.3f m/s:\n',opt.stationary_speed_check_mps);
    disp(stats.window_audit(:,{'Trial','NoWindSpeedMean_mps','WindSpeedMean_mps', ...
        'NoWindSpeedMax_mps','WindSpeedMax_mps','NoWindAboveCheck_pct','WindAboveCheck_pct'}));
end
e=stats.ensemble;
fprintf('Fig. 7: common observed time %.3f--%.3f s, grid %.5f s; min contributors = %d/%d.\n', ...
    e.t(1),e.t(end),e.grid_step_s,e.minimum_trials_for_display,stats.n_trials);
fprintf('Pointwise SD is across trials at a fixed aligned time; no extra temporal smoothing is applied.\n');
if opt.clip_error_band_at_zero
    fprintf('Angular-error band lower edge is clipped at 0 FOR DISPLAY ONLY; numerical SD is unchanged.\n');
end
fprintf('Common valid metric samples [overall,wind] per trial:\n');
disp(array2table([stats.trial_ids stats.normal_samples_by_trial], ...
    'VariableNames',{'Trial','OverallSamples','WindSamples'}));
fprintf('%s\n\n',repmat('=',1,108));
end

function fig=plotPaperFigure7(stats,opt)
e=stats.ensemble; t=e.t;
lineScale=opt.fig7_line_scale;
fig=figure('Name','Fig. 7 - Repeated surface-normal estimation', ...
    'NumberTitle','off','Color','w','Position',opt.fig7_position,'Renderer','painters');
setPaperFigureSize(fig,opt.fig7_size_in);
% Explicit layout makes the last panel taller while keeping all x axes aligned.
left=0.105; right=0.975; bottom=0.065; top=0.850; gap=0.017;
if ~opt.show_condition_strips, top=0.915; end
weights=opt.fig7_row_weights(:)/sum(opt.fig7_row_weights);
heights=(top-bottom-4*gap)*weights;
ax=gobjects(5,1); upper=top;
for j=1:5
    ax(j)=axes('Parent',fig,'Units','normalized', ...
        'Position',[left upper-heights(j) right-left heights(j)]);
    hold(ax(j),'on'); upper=upper-heights(j)-gap;
    set(ax(j),'FontName',opt.font_name,'FontSize',opt.font_size, ...
        'LineWidth',0.7*lineScale,'Box','on','TickDir','out','TickLength',[0.004 0.004], ...
        'XGrid','on','YGrid','on','GridAlpha',0.10,'GridLineStyle',':','Layer','top');
    xlim(ax(j),[t(1) t(end)]);
    if isfield(opt,'fig7_xtick_s') && isfinite(opt.fig7_xtick_s) && opt.fig7_xtick_s>0
        firstTick=ceil(t(1)/opt.fig7_xtick_s)*opt.fig7_xtick_s;
        lastTick=floor(t(end)/opt.fig7_xtick_s)*opt.fig7_xtick_s;
        if lastTick>=firstTick
            set(ax(j),'XTick',firstTick:opt.fig7_xtick_s:lastTick);
        end
    end
end
normalExtent=[e.normal_mean(:)-e.normal_sd(:);e.normal_mean(:)+e.normal_sd(:); ...
    e.normal_mean(:);e.reference_mean(:)];
finiteExtent=normalExtent(isfinite(normalExtent));
if isempty(opt.normal_ylim)
    if isempty(finiteExtent), amplitude=1.1; else, amplitude=max(1.1,ceil(10*max(abs(finiteExtent)))/10); end
    normalLimits=[-amplitude amplitude];
else
    normalLimits=opt.normal_ylim;
end
upperError=e.error_mean+e.error_sd;
lowerError=e.error_mean-e.error_sd;
if opt.clip_error_band_at_zero, lowerError=max(0,lowerError); end
if isempty(opt.error_ylim)
    x=[upperError(:);e.error_mean(:)]; x=x(isfinite(x));
    if isempty(x), errorTop=90; else, errorTop=max(10,10*ceil(1.03*max(x)/10)); end
    x=lowerError(isfinite(lowerError));
    errorBottom=0; if ~isempty(x), errorBottom=min(0,10*floor(min(x)/10)); end
    errorLimits=[errorBottom errorTop];
else
    errorLimits=opt.error_ylim;
end
s=e.speed_mean(isfinite(e.speed_mean));
if isempty(s), speedTop=0.15; else, speedTop=max(0.10,0.05*ceil(1.10*max(s)/0.05)); end
ylim(ax(1),[0 speedTop]);
for j=2:4, ylim(ax(j),normalLimits); end
ylim(ax(5),errorLimits);
% Consensus surface shading is drawn BEFORE data/bands; exact trial masks stay
% in the numeric metrics. Grey regions mean >=50% of trials are on a curve.
curveMask=e.curve_fraction>=0.5&isfinite(e.curve_fraction);
for j=1:5, paperShadeIntervals(ax(j),t,curveMask,[0.7 0.7 0.7],0.13); end
plot(ax(1),t,e.speed_mean,'Color',[0.20 0.32 0.42],'LineWidth',1.25*lineScale);
ylabel(ax(1),'$\|\mathbf{v}_c\|$ (m/s)','Interpreter','latex');
letters='xyz'; handles=gobjects(1,5);
for c=1:3
    a=ax(c+1);
    if opt.component_bands(c)
        for m=1:4
            paperDrawBand(a,t,e.normal_mean(:,c,m)-e.normal_sd(:,c,m), ...
                e.normal_mean(:,c,m)+e.normal_sd(:,c,m),opt.method_colors(m,:),opt.band_alpha);
        end
    end
    % Plot every line after every band so no band's fill covers another line.
    referenceLine=plot(a,t,e.reference_mean(:,c),'--', ...
        'Color',[0.38 0.38 0.38],'LineWidth',1.35*lineScale, ...
        'DisplayName','Reference');
    for m=1:4
        h=plot(a,t,e.normal_mean(:,c,m),'Color',opt.method_colors(m,:), ...
            'LineStyle',opt.method_styles{m},'LineWidth',opt.method_widths(m)*lineScale, ...
            'DisplayName',sprintf('M%d',m));
        if c==1, handles(m+1)=h; end
    end
    if c==1, handles(1)=referenceLine; end
    ylabel(a,sprintf('$n_%c$',letters(c)),'Interpreter','latex');
end
if opt.error_band
    for m=1:4
        paperDrawBand(ax(5),t,lowerError(:,m),upperError(:,m), ...
            opt.method_colors(m,:),opt.band_alpha);
    end
end
for m=1:4
    plot(ax(5),t,e.error_mean(:,m),'Color',opt.method_colors(m,:), ...
        'LineStyle',opt.method_styles{m},'LineWidth',opt.method_widths(m)*lineScale);
end
ylabel(ax(5),'$e_n$ (deg)','Interpreter','latex');
xlabel(ax(5),'Time (s)','Interpreter','latex');
set(ax(1:4),'XTickLabel',[]); linkaxes(ax,'x');
labels={'Reference','M1','M2','M3','M4 (Proposed)'};
lg=legend(ax(2),handles,labels,'Orientation','horizontal','NumColumns',5, ...
    'Interpreter','none','Box','off','FontName',opt.font_name, ...
    'FontSize',opt.fig7_header_font_size);
set(lg,'Units','normalized','Position',[left 0.958 right-left 0.020]);
if opt.show_condition_strips
    strip=axes('Parent',fig,'Units','normalized','Position',[left 0.875 right-left 0.045]);
    hold(strip,'on'); xlim(strip,[t(1) t(end)]); ylim(strip,[0.5 3.5]);
    set(strip,'YTick',[1 2 3],'YTickLabel',{'Surface','High friction','Wind window*'}, ...
        'XTick',[],'Box','off','TickLength',[0 0],'FontName',opt.font_name, ...
        'FontSize',opt.fig7_header_font_size);
    paperDrawStrip(strip,t,~curveMask&isfinite(e.curve_fraction),1,[0.95 0.95 0.95],'Plane');
    paperDrawStrip(strip,t,curveMask,1,[0.78 0.78 0.78],'Curve');
    paperDrawStrip(strip,t,e.friction_fraction>=0.5,2,[0.81 0.87 0.91],'');
    paperDrawStrip(strip,t,e.wind_fraction>=0.5,3,[0.96 0.77 0.66],'Wind');
    linkaxes([ax;strip],'x');
    annotation(fig,'textbox',[left 0.925 right-left 0.013], ...
        'String','Grey: curved surface. Strips: majority across trials. *Geometry proxy unless overridden.', ...
        'EdgeColor','none','HorizontalAlignment','right','FontName',opt.font_name, ...
        'FontSize',opt.fig7_header_font_size, ...
        'Interpreter','none','FitBoxToText','off');
end
if opt.clip_error_band_at_zero
    annotation(fig,'textbox',[left 0.005 right-left 0.018], ...
        'String','Angular-error band clipped at 0 for display only. Component means are not renormalized.', ...
        'EdgeColor','none','FontName',opt.font_name, ...
        'FontSize',max(7,opt.font_size-1),'Interpreter','none');
end
setappdata(fig,'PaperStatistics',stats);
end

function fig=plotPaperFigure8(stats,opt)
fig=figure('Name','Fig. 8 - Force-direction angular RMSE', ...
    'NumberTitle','off','Color','w','Position',opt.fig8_position,'Renderer','painters');
setPaperFigureSize(fig,opt.fig8_size_in);
ax=axes('Parent',fig,'Position',[0.13 0.20 0.84 0.75]); hold(ax,'on');
mu=stats.force_mean.'; sd=stats.force_sd.'; % condition x cue
colors=opt.method_colors([1 2 4],:);
labels={'$\hat{f}_l$','$\bar{f}_l$','$\hat{f}_c$'};
offsets=[-0.24 0 0.24]; b=gobjects(1,3);
for m=1:3
    centers=(1:2)+offsets(m);
    % Explicit group positions keep bar centers/error bars/dots identical.
    b(m)=bar(ax,centers,mu(:,m),0.20,'FaceColor',colors(m,:), ...
        'FaceAlpha',0.82,'EdgeColor',[0.20 0.20 0.20],'LineWidth',0.55);
    for condition=1:2
        values=stats.force_rmse_by_trial(:,m,condition);
        if opt.show_trial_dots
            if stats.n_trials==1, jitter=0; else, jitter=linspace(-0.065,0.065,stats.n_trials).'; end
            plot(ax,centers(condition)+jitter,values,'o','LineStyle','none', ...
                'Color',[0.18 0.18 0.18],'MarkerFaceColor','w','MarkerSize',4.5, ...
                'LineWidth',0.8,'HandleVisibility','off');
        end
        if isfinite(mu(condition,m))&&isfinite(sd(condition,m))
            errorbar(ax,centers(condition),mu(condition,m),sd(condition,m),'k', ...
                'LineStyle','none','LineWidth',1.0,'CapSize',6,'HandleVisibility','off');
        end
    end
end
set(ax,'XTick',[1 2],'XTickLabel',{'Without wind','Wind'},'FontName',opt.font_name, ...
    'FontSize',opt.font_size,'LineWidth',0.7,'Box','on','TickDir','out', ...
    'TickLength',[0.008 0.008],'XGrid','off','YGrid','on', ...
    'GridAlpha',0.12,'GridLineStyle',':','Layer','top');
xlim(ax,[0.5 2.5]);
lo=mu-sd; hi=mu+sd;
x=[hi(:);stats.force_rmse_by_trial(:)]; x=x(isfinite(x));
if isempty(x), top=90; else, top=max(10,10*ceil(1.12*max(x)/10)); end
x=lo(isfinite(lo)); bottom=0; if ~isempty(x), bottom=min(0,10*floor(min(x)/10)); end
ylim(ax,[bottom top]);
ylabel(ax,'Angular RMSE (deg)','FontName',opt.font_name);
legend(ax,b,labels,'Location','northwest','Orientation','vertical', ...
    'Box','off','FontName',opt.font_name,'FontSize',opt.fig8_legend_font_size, ...
    'Interpreter','latex','NumColumns',1);
if opt.show_trial_dots
    annotation(fig,'textbox',[0.105 0.015 0.86 0.035], ...
        'String','Each dot is one trial RMSE. Error bars show between-trial sample SD.', ...
        'HorizontalAlignment','center','EdgeColor','none','FontName',opt.font_name, ...
        'FontSize',max(7,opt.font_size-1), ...
        'Interpreter','none');
end
setappdata(fig,'TrialRMSE',stats.force_rmse_by_trial);
setappdata(fig,'WindowAudit',stats.window_audit);
end

function paperDrawBand(ax,t,lower,upper,color,alpha)
% Fill each contiguous valid span separately: never bridge missing intervals.
valid=isfinite(t)&isfinite(lower)&isfinite(upper);
[a,b]=paperRuns(valid);
for j=1:numel(a)
    ii=a(j):b(j); if numel(ii)<2, continue; end
    fill(ax,[t(ii);flipud(t(ii))],[lower(ii);flipud(upper(ii))],color, ...
        'FaceAlpha',alpha,'EdgeColor','none','HandleVisibility','off');
end
end

function paperShadeIntervals(ax,t,mask,color,alpha)
[a,b]=paperRuns(mask); y=ylim(ax);
for j=1:numel(a)
    [x0,x1]=paperIntervalEdges(t,a(j),b(j));
    patch(ax,[x0 x1 x1 x0],[y(1) y(1) y(2) y(2)],color, ...
        'FaceAlpha',alpha,'EdgeColor','none','HandleVisibility','off');
end
end

function paperDrawStrip(ax,t,mask,lane,color,label)
[a,b]=paperRuns(mask&isfinite(t));
for j=1:numel(a)
    [x0,x1]=paperIntervalEdges(t,a(j),b(j));
    patch(ax,[x0 x1 x1 x0],lane+[-0.32 -0.32 0.32 0.32],color, ...
        'EdgeColor','none','HandleVisibility','off');
    if ~isempty(label)&&x1-x0>0.025*(t(end)-t(1))
        text(ax,(x0+x1)/2,lane,label,'HorizontalAlignment','center', ...
            'VerticalAlignment','middle','FontSize',7,'Interpreter','none');
    end
end
end

function [x0,x1]=paperIntervalEdges(t,a,b)
x0=t(a); x1=t(b);
if a>1, x0=0.5*(t(a-1)+t(a)); end
if b<numel(t), x1=0.5*(t(b)+t(b+1)); end
end

function [a,b]=paperRuns(mask)
mask=logical(mask(:));
a=find(mask&[true;~mask(1:end-1)]);
b=find(mask&[~mask(2:end);true]);
end

function setPaperFigureSize(fig,sizeIn)
% Fix the physical canvas so exported artwork drops into an IEEE column at
% its intended size without subsequent scaling (and font-size surprises).
set(fig,'Units','inches');
position=get(fig,'Position');
position(3:4)=sizeIn;
set(fig,'Position',position,'PaperUnits','inches', ...
    'PaperPosition',[0 0 sizeIn],'PaperSize',sizeIn);
end

function exportFirstExperimentPaperResults(fig7,fig8,stats,folder)
if ~isfolder(folder), mkdir(folder); end
savefig(fig7,fullfile(folder,'Fig7_normal_estimation.fig'));
savefig(fig8,fullfile(folder,'Fig8_force_direction.fig'));
% Export is opt-in. Only derived outputs are written; input CSVs are untouched.
if exist('exportgraphics','file')==2
    % Vector PDF is the preferred manuscript asset; 600-dpi PNG is retained
    % for quick review and submission systems that require raster artwork.
    exportgraphics(fig7,fullfile(folder,'Fig7_normal_estimation.pdf'), ...
        'ContentType','vector','BackgroundColor','white');
    exportgraphics(fig8,fullfile(folder,'Fig8_force_direction.pdf'), ...
        'ContentType','vector','BackgroundColor','white');
    exportgraphics(fig7,fullfile(folder,'Fig7_normal_estimation.png'),'Resolution',600);
    exportgraphics(fig8,fullfile(folder,'Fig8_force_direction.png'),'Resolution',600);
else
    print(fig7,fullfile(folder,'Fig7_normal_estimation.pdf'),'-dpdf','-painters');
    print(fig8,fullfile(folder,'Fig8_force_direction.pdf'),'-dpdf','-painters');
    print(fig7,fullfile(folder,'Fig7_normal_estimation.png'),'-dpng','-r600');
    print(fig8,fullfile(folder,'Fig8_force_direction.png'),'-dpng','-r600');
end
writetable(stats.table_I_numeric,fullfile(folder,'TableI_mean_sd.csv'));
writetable(stats.fig8_numeric,fullfile(folder,'Fig8_mean_sd.csv'));
writetable(stats.normal_per_trial,fullfile(folder,'normal_rmse_per_trial.csv'));
writetable(stats.force_per_trial,fullfile(folder,'force_rmse_per_trial.csv'));
writetable(stats.window_audit,fullfile(folder,'paper_window_audit.csv'));
writetable(stats.voltage_model_audit,fullfile(folder,'voltage_model_audit.csv'));
fprintf('Paper figures and numeric tables exported to: %s\n',folder);
end
