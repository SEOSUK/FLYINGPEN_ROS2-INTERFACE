%% Load velocity debug data
clear; close all; clc;

defaultDir = fullfile(getenv("HOME"), "hitl_ws", "src", ...
    "flying_pen", "bag", "logging");
axisNames = ["x", "y", "z"];
lineWidth = 2;

[files, folder] = uigetfile(fullfile(defaultDir, "*_velocity*.csv"), ...
    "Select one or more velocity debug CSVs", "MultiSelect", "on");
if isequal(files, 0), return; end
if ischar(files) || isstring(files), files = cellstr(files); end

runs = struct([]);
for runIdx = 1:numel(files)
    filePath = fullfile(folder, files{runIdx});
    T = readtable(filePath, detectImportOptions(filePath, 'Delimiter', ','));

    if ismember("t_sec", string(T.Properties.VariableNames))
        t = col(T, "t_sec");
        firstValid = find(isfinite(t), 1, 'first');
        if ~isempty(firstValid), t = t - t(firstValid); end
    else
        fallbackSampleHz = 50;
        warning('%s has no t_sec; using %.3g Hz.', files{runIdx}, fallbackSampleHz);
        t = (0:height(T)-1)' / fallbackSampleHz;
    end

    runs(runIdx).name = string(files{runIdx});
    runs(runIdx).t = t;
    runs(runIdx).vcRaw = cols(T, ["vcRawX", "vcRawY", "vcRawZ"]);
    runs(runIdx).vc = cols(T, ["vcX", "vcY", "vcZ"]);
    runs(runIdx).vRaw = cols(T, ["posRawVx", "posRawVy", "posRawVz"]);
    runs(runIdx).v = cols(T, ["posVx", "posVy", "posVz"]);
    runs(runIdx).rotOffsetVel = cols(T, ...
        ["rotOffsetVelX", "rotOffsetVelY", "rotOffsetVelZ"]);
end

%% vc (raw / filtered) - x, y, z
for runIdx = 1:numel(runs)
    figure('Name', runs(runIdx).name + " - vc (raw / filtered)");
    ax = gobjects(3, 1);

    for k = 1:3
        ax(k) = subplot(3, 1, k);
        plot(runs(runIdx).t, ...
            [runs(runIdx).vcRaw(:, k), runs(runIdx).vc(:, k)], ...
            'LineWidth', lineWidth);
        ylabel("vc_" + axisNames(k) + " [m/s]");
        legend('raw', 'filtered');
        grid on;
    end

    xlabel('t [s]');
    linkaxes(ax, 'x');
end

%% v (raw / filtered) - x, y, z
for runIdx = 1:numel(runs)
    figure('Name', runs(runIdx).name + " - v (raw / filtered)");
    ax = gobjects(3, 1);

    for k = 1:3
        ax(k) = subplot(3, 1, k);
        plot(runs(runIdx).t, ...
            [runs(runIdx).vRaw(:, k), runs(runIdx).v(:, k)], ...
            'LineWidth', lineWidth);
        ylabel("v_" + axisNames(k) + " [m/s]");
        legend('raw', 'filtered');
        grid on;
    end

    xlabel('t [s]');
    linkaxes(ax, 'x');
end

%% v_lpf and R*(omega x r) - 3-by-2 component panel
for runIdx = 1:numel(runs)
    figure('Name', runs(runIdx).name + " - v_lpf / R(omega x r)");
    ax = gobjects(3, 2);

    for k = 1:3
        ax(k, 1) = subplot(3, 2, 2*k - 1);
        plot(runs(runIdx).t, runs(runIdx).v(:, k), ...
            'LineWidth', lineWidth);
        ylabel("v_{lpf," + axisNames(k) + "} [m/s]");
        if k == 1, title('v_{lpf}'); end
        grid on;

        ax(k, 2) = subplot(3, 2, 2*k);
        plot(runs(runIdx).t, runs(runIdx).rotOffsetVel(:, k), ...
            'LineWidth', lineWidth);
        ylabel("R(omega x r)_{" + axisNames(k) + "} [m/s]");
        if k == 1, title('R(omega x r)'); end
        grid on;
    end

    xlabel(ax(3, 1), 't [s]');
    xlabel(ax(3, 2), 't [s]');
    set(ax(:), 'YLim', [-0.2, 0.2]);
    linkaxes(ax(:), 'x');
end

function x = col(T, name)
if ismember(name, string(T.Properties.VariableNames))
    x = T.(name);
else
    warning('Column %s is missing.', name);
    x = nan(height(T), 1);
end
end

function x = cols(T, names)
x = nan(height(T), numel(names));
for i = 1:numel(names)
    x(:, i) = col(T, names(i));
end
end
