%% File loading
clear; close all; clc;
defaultDir = fullfile(getenv("HOME"), "hitl_ws", "src", "flying_pen", "bag", "logging");
[file,path] = uigetfile({"*_force_control*.csv","Force-control CSV";"*.csv","All CSV"}, ...
    "Select force-control logging CSV", defaultDir);
if isequal(file,0), return; end
T=readtable(fullfile(path,file),detectImportOptions(fullfile(path,file),'Delimiter',','));
t=safeTime(T); axesName=["X","Y","Z"];

%% Figure 0 - IMU acceleration and onboard attitude estimate
imuMovingAverageCutoffHz = 0.1;  % [Hz], <= 0 disables the MATLAB moving-average filter
imuAccRaw = columnsXYZ(T,"imuAccRaw");
imuAccTrimmed = columnsXYZ(T,"imuAccTrimmed");
imuAccRawFiltered = movingAverageCutoff(imuAccRaw,t,imuMovingAverageCutoffHz);
imuAccTrimmedFiltered = movingAverageCutoff(imuAccTrimmed,t,imuMovingAverageCutoffHz);
% attitudeRoll/Pitch/Yaw are converted from the existing /cf2/pose quaternion by data_logging.
attitudeEstimate = [safeColumn(T,"attitudeRoll"),safeColumn(T,"attitudePitch"),safeColumn(T,"attitudeYaw")];
figure('Name','IMU acceleration and attitude estimate');
for k=1:3
    subplot(3,2,2*k-1);
    plot(t,imuAccRaw(:,k),'Color',[0.60 0.78 0.92],'LineWidth',2); hold on;
    plot(t,imuAccTrimmed(:,k),'Color',[0.95 0.70 0.65],'LineWidth',2);
    plot(t,imuAccRawFiltered(:,k),'Color',[0.00 0.35 0.70],'LineWidth',2);
    plot(t,imuAccTrimmedFiltered(:,k),'Color',[0.80 0.15 0.10],'LineWidth',2); grid on;
    ylabel(axesName(k)+' [g]');
    if k==1
        title('Acceleration before / after accTrim');
        if imuMovingAverageCutoffHz>0
            legend('before raw','after raw', ...
                sprintf('before MA %.2f Hz',imuMovingAverageCutoffHz), ...
                sprintf('after MA %.2f Hz',imuMovingAverageCutoffHz));
        else
            legend('before','after','before (filter off)','after (filter off)');
        end
    end
    subplot(3,2,2*k);
    plot(t,attitudeEstimate(:,k),'LineWidth',2); grid on;
    ylabel(axesName(k)+' [deg]');
    if k==1, title('Onboard attitude estimate: roll / pitch / yaw'); end
end
xlabel(subplot(3,2,5),'t [s]'); xlabel(subplot(3,2,6),'t [s]');
sgtitle('IMU trim diagnostics');

%% Figure 1 - Firmware EE position tracking
figure('Name','Firmware EE position tracking');
for k=1:3
    subplot(3,1,k); plot(t,safeColumn(T,"fwCmd"+axesName(k)),'LineWidth',2); hold on;
    plot(t,safeColumn(T,"fwEePos"+axesName(k)),'LineWidth',2); grid on;
    ylabel(axesName(k)+' [m]'); legend('final firmware command','firmware measured');
end
xlabel('t [s]'); sgtitle('Firmware EE position tracking');

%% Figure 2 - Force command and reconstructed contact force X (hat C)
figure('Name','Force command and reconstructed contact force X (hat C)');
subplot(2,1,1);
plot(t,safeColumn(T,"forceCmd"),'LineWidth',2); hold on;
plot(t,-safeColumn(T,"mobForceHatCX"),'LineWidth',2); grid on;
ylabel('force [N]');
legend('force command','-hat C F_x');
title('Force command vs. sign-inverted reconstructed contact force X');
subplot(2,1,2);
plot(t,safeColumn(T,"fwCmdX"),'LineWidth',2); hold on;
plot(t,safeColumn(T,"fwEePosX"),'LineWidth',2); grid on;
ylabel('EE x [m]'); xlabel('t [s]');
legend('command','measured');
title('End-effector X position command vs. measurement');

%% Figure 3 - Momentum Observer force and torque
figure('Name','Momentum Observer force and torque');
for k=1:3
    subplot(3,2,2*k-1);
    plot(t,safeColumn(T,"mobForce"+axesName(k)),'LineWidth',2); hold on;
    plot(t,safeColumn(T,"mobForceBar"+axesName(k)),'LineWidth',2);
    plot(t,safeColumn(T,"mobForceHatC"+axesName(k)),'LineWidth',2); grid on;
    ylabel(axesName(k)+' [N]'); legend('raw','bar','hat C');
    subplot(3,2,2*k);
    plot(t,safeColumn(T,"mobTorque"+axesName(k)),'LineWidth',2); grid on;
    ylabel(axesName(k)+' [N m]');
end
xlabel(subplot(3,2,5),'t [s]'); xlabel(subplot(3,2,6),'t [s]');
sgtitle('Momentum Observer force processing and raw torque');

%% Figure 4 - Force vectors, normalized force, and tilted-wall normal
forceRaw = columnsXYZ(T,"mobForce");
forceBar = columnsXYZ(T,"mobForceBar");
forceHat = columnsXYZ(T,"mobForceHatC");
forceRawUnit = normalizeRows(forceRaw);
forceBarUnit = normalizeRows(forceBar);
forceHatUnit = normalizeRows(forceHat);
wallNormalUnit = normalizeRows(columnsXYZ(T,"wallNormal"));
figure('Name','Force direction diagnostics');
for k=1:3
    subplot(3,3,3*k-2);
    plot(t,forceRaw(:,k),'LineWidth',2); hold on; plot(t,forceBar(:,k),'LineWidth',2);
    plot(t,forceHat(:,k),'LineWidth',2); grid on; ylabel(axesName(k)+' [N]');
    if k==1, title('Force: raw / bar / hat C'); legend('raw','bar','hat C'); end
    subplot(3,3,3*k-1);
    plot(t,forceRawUnit(:,k),'LineWidth',2); hold on; plot(t,forceBarUnit(:,k),'LineWidth',2);
    plot(t,forceHatUnit(:,k),'LineWidth',2); grid on; ylabel(axesName(k)); ylim([-1.05 1.05]);
    if k==1, title('Normalized force'); legend('raw','bar','hat C'); end
    subplot(3,3,3*k);
    plot(t,wallNormalUnit(:,k),'k','LineWidth',2); hold on;
    plot(t,forceRawUnit(:,k),'LineWidth',1.5); plot(t,forceBarUnit(:,k),'LineWidth',1.5);
    plot(t,forceHatUnit(:,k),'LineWidth',1.5); grid on; ylabel(axesName(k)); ylim([-1.05 1.05]);
    if k==1, title('Wall normal vs normalized force'); legend('wall','raw','bar','hat C'); end
end
for k=7:9, xlabel(subplot(3,3,k),'t [s]'); end
sgtitle('MOB force-vector processing in world frame');

%% Figure 5 - Wall-normal angle, eta_T, and voltage
angleRaw = includedAngleDeg(wallNormalUnit,forceRawUnit);
angleBar = includedAngleDeg(wallNormalUnit,forceBarUnit);
angleHat = includedAngleDeg(wallNormalUnit,forceHatUnit);
angleRawAvg = movingAverage5s(angleRaw,t);
angleBarAvg = movingAverage5s(angleBar,t);
angleHatAvg = movingAverage5s(angleHat,t);
dark = lines(3); light = 0.68 + 0.32*dark;
figure('Name','Wall-normal force angle and effectiveness');
layout=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
ax=nexttile([2 1]);
plot(ax,t,angleRaw,'Color',light(1,:),'LineWidth',3); hold(ax,'on');
plot(ax,t,angleBar,'Color',light(2,:),'LineWidth',3);
plot(ax,t,angleHat,'Color',light(3,:),'LineWidth',3);
plot(ax,t,angleRawAvg,'Color',dark(1,:),'LineWidth',1.5);
plot(ax,t,angleBarAvg,'Color',dark(2,:),'LineWidth',1.5);
plot(ax,t,angleHatAvg,'Color',dark(3,:),'LineWidth',1.5); grid(ax,'on');
ylabel(ax,'included angle [deg]'); xlabel(ax,'t [s]'); ylim(ax,[0 180]);
title(ax,'Wall normal vs normalized force');
legend(ax,{'raw','bar','hat C','raw 5 s avg','bar 5 s avg','hat C 5 s avg'},'Location','best');
axEta=nexttile; plot(axEta,t,safeColumn(T,"etaT"),'LineWidth',1.5); grid(axEta,'on');
ylabel(axEta,'\eta_T'); title(axEta,'Thrust effectiveness');
axVoltage=nexttile; plot(axVoltage,t,safeColumn(T,"batteryVoltage"),'LineWidth',1.5); grid(axVoltage,'on');
ylabel(axVoltage,'voltage [V]'); xlabel(axVoltage,'t [s]'); title(axVoltage,'Battery voltage');
title(layout,'Wall-normal alignment, \eta_T, and voltage');

function x=safeColumn(T,name)
if ismember(name,string(T.Properties.VariableNames)), x=double(T.(name)); else, x=zeros(height(T),1); end
x(~isfinite(x))=0;
end
function t=safeTime(T)
t=safeColumn(T,"t_sec");
if ~ismember("t_sec",string(T.Properties.VariableNames)) || any(diff(t)<0) || all(t==0), t=(0:height(T)-1)'; end
end
function xyz=columnsXYZ(T,prefix)
xyz=[safeColumn(T,prefix+"X"),safeColumn(T,prefix+"Y"),safeColumn(T,prefix+"Z")];
end
function unit=normalizeRows(xyz)
norms=sqrt(sum(xyz.^2,2)); unit=zeros(size(xyz)); valid=norms>1e-12;
unit(valid,:)=xyz(valid,:)./norms(valid);
end
function angle=includedAngleDeg(a,b)
valid=sqrt(sum(a.^2,2))>0 & sqrt(sum(b.^2,2))>0; angle=zeros(size(a,1),1);
d=sum(a(valid,:).*b(valid,:),2); angle(valid)=acosd(max(-1,min(1,d)));
end
function y=movingAverage5s(x,t)
if numel(t)>1 && all(isfinite(t)) && all(diff(t)>0)
    y=movmean(x,[2.5 2.5],'SamplePoints',t);
else
    y=x;
end
end
function y=movingAverageCutoff(x,t,cutoffHz)
if isempty(cutoffHz) || ~isfinite(cutoffHz) || cutoffHz<=0 || numel(t)<2 || any(~isfinite(t)) || any(diff(t)<=0)
    y=x; return;
end
% A rectangular moving average has f_(-3 dB) approximately 0.443/windowSec.
windowSec=0.443/cutoffHz;
y=movmean(x,[windowSec/2 windowSec/2],'SamplePoints',t);
end
