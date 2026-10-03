%% File loading
clear; close all; clc;
defaultDir = fullfile(getenv("HOME"), "hitl_ws", "src", "flying_pen", "bag", "logging");
[file, path] = uigetfile({"*_force_control*.csv", "Force-control CSV"; "*.csv", "All CSV"}, ...
    "Select force-control logging CSV", defaultDir);
if isequal(file, 0), return; end
T = readtable(fullfile(path, file), detectImportOptions(fullfile(path, file), 'Delimiter', ','));
t = safeTime(T);

%% Position
plotXYZ(t, T, ["mocapRawX","mocapRawY","mocapRawZ"], 'Mocap position', 'm');

%% Tilted-wall normal
plotXYZ(t, T, ["wallNormalX","wallNormalY","wallNormalZ"], 'Tilted-wall normal (world)', '-');

%% End-effector position
plotXYZ(t, T, ["fwEePosX","fwEePosY","fwEePosZ"], 'Firmware EE measured position', 'm');

%% Firmware EE command
plotXYZ(t, T, ["fwCmdX","fwCmdY","fwCmdZ"], 'Firmware final EE command', 'm');

%% EE command versus measured
figure('Name','EE position tracking');
names = ["X","Y","Z"];
for k=1:3
    subplot(3,1,k); plot(t, safeColumn(T,"fwCmd"+names(k)), 'LineWidth',1.5); hold on;
    plot(t, safeColumn(T,"fwEePos"+names(k)), 'LineWidth',1.5); grid on;
    ylabel(names(k)+' [m]'); legend('command','measured');
end
xlabel('t [s]'); sgtitle('Firmware EE position tracking');

%% Force command
plotScalar(t, safeColumn(T,"forceCmd"), 'Final force command', 'N');

%% Momentum Observer force
plotXYZ(t, T, ["mobForceX","mobForceY","mobForceZ"], 'Raw MOB force (world)', 'N');

%% Momentum Observer torque
plotXYZ(t, T, ["mobTorqueX","mobTorqueY","mobTorqueZ"], 'Raw MOB torque (world)', 'N m');

%% Momentum Observer input force
plotXYZ(t, T, ["mobInputForceX","mobInputForceY","mobInputForceZ"], 'MOB input force (world)', 'N');

%% Momentum Observer input torque
plotXYZ(t, T, ["mobInputTorqueX","mobInputTorqueY","mobInputTorqueZ"], 'MOB input torque (body)', 'N m');

%% Motor thrust used by Momentum Observer
figure('Name','Motor thrust used by MOB');
plot(t,[safeColumn(T,"motorThrust1"),safeColumn(T,"motorThrust2"), ...
    safeColumn(T,"motorThrust3"),safeColumn(T,"motorThrust4")],'LineWidth',1.5);
grid on; xlabel('t [s]'); ylabel('thrust [N]'); legend('motor 1','motor 2','motor 3','motor 4');
title('Per-motor thrust used by Momentum Observer');

%% Momentum Observer force bar
plotXYZ(t, T, ["mobForceBarX","mobForceBarY","mobForceBarZ"], 'Corrected force bar (world)', 'N');

%% Contact-consistent force hat C
plotXYZ(t, T, ["mobForceHatCX","mobForceHatCY","mobForceHatCZ"], 'Contact-consistent force hat C (world)', 'N');

%% Firmware force-normal estimate
plotXYZ(t, T, ["forceNormalEstX","forceNormalEstY","forceNormalEstZ"], 'Firmware force-normal estimate (world)', '-');

%% Firmware end-effector velocity
plotXYZ(t, T, ["fwEeVelX","fwEeVelY","fwEeVelZ"], 'Firmware EE velocity (world)', 'm/s');

%% Thrust effectiveness
plotScalar(t, safeColumn(T,"etaT"), 'Thrust effectiveness eta_T', '-');

%% Battery voltage
plotScalar(t, safeColumn(T,"batteryVoltage"), 'Battery voltage', 'V');

%% Momentum Observer force processing
figure('Name','MOB force processing');
for k=1:3
    subplot(3,1,k); plot(t,safeColumn(T,"mobForce"+names(k)),'LineWidth',1.5); hold on;
    plot(t,safeColumn(T,"mobForceBar"+names(k)),'LineWidth',1.5);
    plot(t,safeColumn(T,"mobForceHatC"+names(k)),'LineWidth',1.5); grid on;
    ylabel(names(k)+' [N]'); legend('raw','bar','hat C');
end
xlabel('t [s]'); sgtitle('MOB force processing');

%% Logger timing
plotScalar(t, safeColumn(T,"loggerDtMs"), 'Logger period', 'ms');

function x = safeColumn(T, name)
if ismember(name, string(T.Properties.VariableNames))
    x = double(T.(name));
else
    x = zeros(height(T),1);
end
x(~isfinite(x)) = 0;
end
function t = safeTime(T)
t = safeColumn(T,"t_sec");
if ~ismember("t_sec",string(T.Properties.VariableNames)) || numel(t)~=height(T) || any(diff(t)<0) || all(t==0)
    t=(0:height(T)-1)';
end
end
function plotXYZ(t,T,cols,ttl,unit)
figure('Name',ttl); labels=["X","Y","Z"];
for k=1:3, subplot(3,1,k); plot(t,safeColumn(T,cols(k)),'LineWidth',1.5); grid on; ylabel(labels(k)+' ['+unit+']'); end
xlabel('t [s]'); sgtitle(ttl);
end
function plotScalar(t,x,ttl,unit)
figure('Name',ttl); plot(t,x,'LineWidth',1.5); grid on; xlabel('t [s]'); ylabel(unit); title(ttl);
end
