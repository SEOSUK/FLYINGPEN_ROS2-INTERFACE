%% Flying Pen normal / curvature estimator offline analysis
% Analysis only: this script never publishes or modifies a flight command.
clear; close all; clc;

%% Global x-axis range and analysis parameters
global_xlim = [];                    % [s], e.g. [10 30]; [] uses the full recording
firmware_epsilon_v = 0.0004;       % [(m/s)^2], current su_position.normEpsV
epsilon_v_candidates = firmware_epsilon_v .* [0.5 1.0 2.0];
firmware_a_bar_n = 0.013;           % [m/s^2], current su_position.velModABar
a_bar_n_candidates = firmware_a_bar_n .* [0.5 1.0 2.0];
firmware_v_min = 0.001;             % [m/s], current su_position.velModVMin
firmware_kappa_lpf_hz = 0.0;        % [Hz], current su_position.velModKLPF; <=0 bypasses
selected_epsilon_index = find(abs(epsilon_v_candidates-firmware_epsilon_v) < eps,1);
if isempty(selected_epsilon_index), selected_epsilon_index = 1; end

normal_quiver_stride = 10;
normal_quiver_scale = 0.25;
show_contact_frame_quivers = false;
low_force_threshold = 5.0e-4;       % [N], offline safe-normalization mask only
small_vector_threshold = 1.0e-8;

%% Select normal-estimator CSV
defaultDir = fullfile(getenv("HOME"),"hitl_ws","src","flying_pen","bag","logging");
[file,path] = uigetfile({"flyingpen_normal_*.csv","Normal-estimator CSV";"*.csv","All CSV"}, ...
    "Select normal-estimator logging CSV",defaultDir);
if isequal(file,0), return; end
csvFile = fullfile(path,file);
fprintf('Loading %s\n',csvFile);
T = readtable(csvFile,detectImportOptions(csvFile,'Delimiter',','));
t = safeColumn(T,"t_sec");
if all(~isfinite(t)), t=(0:height(T)-1).'; end

%% Firmware/ROS raw signals
vehiclePos = columnsXYZ(T,"vehiclePos");
attitude = [safeColumn(T,"attitudeRoll"),safeColumn(T,"attitudePitch"),safeColumn(T,"attitudeYaw")];
eePos = columnsXYZ(T,"fp_normal_eePos");
eeCmd = columnsXYZ(T,"fp_normal_eeCmd");
vc = columnsXYZ(T,"fp_normal_vc");          % exact filtered vc used by firmware estimator
fcHat = columnsXYZ(T,"fp_normal_fcHat");    % reconstructed contact force, hat_c
fnHat = safeColumn(T,"fp_normal_fnHat");
nHat = columnsXYZ(T,"fp_normal_nHat");
nDot = columnsXYZ(T,"fp_normal_nDot");      % signal actually used by firmware kappa estimator
kappaHat = safeColumn(T,"fp_normal_kappaHat");
t1 = columnsXYZ(T,"fp_normal_t1");
t2 = columnsXYZ(T,"fp_normal_t2");
u1Des = safeColumn(T,"u1_des");
u2Des = safeColumn(T,"u2_des");
forceDesired = safeColumn(T,"forceDesired");

%% Offline-derived projection diagnostics
vc2 = sum(vc.^2,2);
vcNorm = sqrt(vc2);
[fcBar,fcNorm] = safeNormalizeRows(fcHat,low_force_threshold);
nEps = numel(epsilon_v_candidates);
gV = nan(height(T),nEps);
qG = nan(height(T),3,nEps);
thetaProjDeg = nan(height(T),nEps);
for j=1:nEps
    epsilonV = epsilon_v_candidates(j);
    denom = vc2 + epsilonV;
    gV(:,j) = vc2 ./ denom;
    projectionScale = sum(vc.*fcBar,2) ./ denom;
    q = fcBar - vc.*projectionScale;
    q(~all(isfinite(fcBar),2),:) = NaN;
    qG(:,:,j) = q;
    qNorm = sqrt(sum(q.^2,2));
    cosine = sum(fcBar.*q,2) ./ qNorm;
    valid = isfinite(cosine) & qNorm > small_vector_threshold;
    cosine = min(1,max(-1,cosine));
    thetaProjDeg(valid,j) = acosd(cosine(valid));
end
qGSelected = qG(:,:,selected_epsilon_index);
[qGSelectedUnit,~] = safeNormalizeRows(qGSelected,small_vector_threshold);

%% Offline curvature verification
nDotNorm = sqrt(sum(nDot.^2,2));
kappaOfflineRaw = abs(sum(nDot.*vc,2)) ./ vc2;
kappaOfflineRaw(~isfinite(kappaOfflineRaw) | vc2 <= small_vector_threshold) = NaN;

% Firmware-equivalent reconstruction: speed gate, last-valid hold, and optional
% first-order kappa LPF. The unlogged normalEstimateValid flag cannot be recovered
% exactly; finite logged nDot/vc samples are treated as valid estimator samples.
kappaOfflineFirmwareEquivalent = nan(height(T),1);
kappaState = NaN;
for k=1:height(T)
    validInput = all(isfinite(nDot(k,:))) && all(isfinite(vc(k,:))) && ...
        isfinite(vcNorm(k)) && vcNorm(k) > firmware_v_min && vc2(k) > 1.0e-12;
    if validInput
        raw = abs(dot(nDot(k,:),vc(k,:))) / vc2(k);
        if isfinite(raw)
            if ~isfinite(kappaState) || firmware_kappa_lpf_hz <= 0
                kappaState = raw;
            elseif k>1
                dt = t(k)-t(k-1);
                if isfinite(dt) && dt>0
                    tau = 1/(2*pi*firmware_kappa_lpf_hz);
                    alpha = dt/(tau+dt);
                    kappaState = kappaState + alpha*(raw-kappaState);
                end
            end
        end
    end
    kappaOfflineFirmwareEquivalent(k) = kappaState;
end

%% Contact-frame and shadow velocity modulation
vtDesWorld = u1Des.*t1 + u2Des.*t2;
vtDesSpeed = sqrt(sum(vtDesWorld.^2,2));
vtDesSpeedOrthonormal = sqrt(u1Des.^2+u2Des.^2);
vT1Actual = sum(vc.*t1,2);
vT2Actual = sum(vc.*t2,2);
nA = numel(a_bar_n_candidates);
alphaStarShadow = ones(height(T),nA);
vtModShadowSpeed = nan(height(T),nA);
for j=1:nA
    denominator = kappaHat .* vtDesSpeed.^2;
    valid = isfinite(denominator) & denominator>0 & ...
        isfinite(a_bar_n_candidates(j)) & a_bar_n_candidates(j)>0;
    alphaStarShadow(valid,j) = min(1,sqrt(a_bar_n_candidates(j)./denominator(valid)));
    alphaStarShadow(~isfinite(alphaStarShadow(:,j)),j) = 1;
    vtModShadowSpeed(:,j) = alphaStarShadow(:,j).*vtDesSpeed;
end

%% Figure 0 - 3x2 overview
overview_xlim = [122 170] ;          % [s], e.g. [80 140]; [] uses the full recording
overview_epsilon_v = 0.0004;         % [(m/s)^2], epsilon_v used for overview q_g
overview_highlight_time_1 = [94 96];  % [s], pale orange; [] disables interval 1
overview_highlight_time_2 = [81 90];       % [s], pale blue; [] disables interval 2
overview_normal_arrow_scale = 0.125;  % [m], XY normal-arrow length
overview_normal_arrow_width = 2.4;    % XY normal-arrow line width
overview_normal_arrow_period = 2.0;   % [s], draw one XY normal arrow per interval
validateattributes(overview_normal_arrow_period,{'numeric'}, ...
    {'scalar','finite','positive'},mfilename,'overview_normal_arrow_period');
overviewHighlightTimes={overview_highlight_time_1,overview_highlight_time_2};
for highlightNumber=1:2
    if ~isempty(overviewHighlightTimes{highlightNumber})
        validateattributes(overviewHighlightTimes{highlightNumber},{'numeric'}, ...
            {'vector','numel',2,'finite','increasing'},mfilename, ...
            sprintf('overview_highlight_time_%d',highlightNumber));
    end
end

if overview_epsilon_v <= 0
    qGOverview=fcBar;
else
    overviewProjectionScale=sum(vc.*fcBar,2)./(vc2+overview_epsilon_v);
    qGOverview=fcBar-vc.*overviewProjectionScale;
end
[qGOverview,~]=safeNormalizeRows(qGOverview,small_vector_threshold);

figure('Name','Normal tuning: overview','Position',[80 40 1550 920]);
tl=tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
axOverviewTime=gobjects(18,1);

leftTop=tiledlayout(tl,3,1,'TileSpacing','compact','Padding','compact');
leftTop.Layout.Tile=1;
axOverviewTime(1)=nexttile(leftTop); plot(t,u1Des,'--','LineWidth',1.8); hold on; plot(t,vT1Actual,'LineWidth',2.1); grid on;
ylabel('v_{t1}'); title('t_1 velocity'); legend('desired','actual','Location','best');
axOverviewTime(2)=nexttile(leftTop); plot(t,u2Des,'--','LineWidth',1.8); hold on; plot(t,vT2Actual,'LineWidth',2.1); grid on;
ylabel('v_{t2}'); title('t_2 velocity'); legend('desired','actual','Location','best');
axOverviewTime(3)=nexttile(leftTop); plot(t,forceDesired,'--','LineWidth',1.8); hold on; plot(t,fnHat,'LineWidth',2.4); grid on;
ylabel('f_n [N]'); title('Normal force'); legend('desired','f hat_c dot n','Location','best');

leftBottom=tiledlayout(tl,3,1,'TileSpacing','compact','Padding','compact');
leftBottom.Layout.Tile=6;
axisNames={'x','y','z'};
for component=1:3
    axOverviewTime(3+component)=nexttile(leftBottom);
    plot(t,fcBar(:,component),'--','LineWidth',1.8); hold on;
    plot(t,qGOverview(:,component),'LineWidth',1.95); grid on;
    ylabel(axisNames{component});
    if component==1, legend('q_f','q_g','Location','best'); end
end
title(leftBottom,sprintf('q_f vs. q_g (epsilon_v = %.4g)',overview_epsilon_v));

middleTop=tiledlayout(tl,3,1,'TileSpacing','compact','Padding','compact');
middleTop.Layout.Tile=2;
for component=1:3
    axOverviewTime(6+component)=nexttile(middleTop);
    plot(t,nHat(:,component),'LineWidth',1.95); grid on;
    ylabel(['n_' axisNames{component}]);
end
title(middleTop,'Normal estimate');

middleBottom=tiledlayout(tl,3,1,'TileSpacing','compact','Padding','compact');
middleBottom.Layout.Tile=5;
for component=1:3
    axOverviewTime(9+component)=nexttile(middleBottom);
    plot(t,nDot(:,component),'LineWidth',1.95); grid on;
    ylabel(['dn_' axisNames{component} ' [1/s]']);
end
title(middleBottom,'Normal-dot estimate');

rightTop=tiledlayout(tl,3,1,'TileSpacing','compact','Padding','compact');
rightTop.Layout.Tile=3;
axOverviewTime(13)=nexttile(rightTop);
plot(t,vcNorm,'LineWidth',2.1); grid on; ylabel('|v_c|');
axOverviewTime(14)=nexttile(rightTop);
plot(t,nDotNorm,'LineWidth',2.1); grid on; ylabel('|n hat dot|');
axOverviewTime(15)=nexttile(rightTop);
plot(t,abs(kappaHat),'LineWidth',2.4); grid on; ylabel('|kappa hat|');
title(rightTop,'Estimator magnitudes');

rightBottom=tiledlayout(tl,3,1,'TileSpacing','compact','Padding','compact');
rightBottom.Layout.Tile=4;
attitudeNames={'roll','pitch','yaw'};
for component=1:3
    axOverviewTime(15+component)=nexttile(rightBottom);
    plot(t,attitude(:,component),'LineWidth',1.95); grid on;
    ylabel([attitudeNames{component} ' [deg]']);
end
title(rightBottom,'Attitude');

if isempty(overview_highlight_time_1) && isempty(overview_highlight_time_2)
    overviewTitle='Normal Estimation Overview | highlight: off';
else
    intervalLabels=cell(1,2);
    for highlightNumber=1:2
        interval=overviewHighlightTimes{highlightNumber};
        if isempty(interval)
            intervalLabels{highlightNumber}=sprintf('%d: off',highlightNumber);
        else
            intervalLabels{highlightNumber}=sprintf('%d: %.2f-%.2f s', ...
                highlightNumber,interval(1),interval(2));
        end
    end
    overviewTitle=sprintf('Normal Estimation Overview | %s | %s', ...
        intervalLabels{1},intervalLabels{2});
end
title(tl,overviewTitle); xlabel(tl,'t [s]');
linkaxes(axOverviewTime,'x');
applyTimeXLim(axOverviewTime,overview_xlim);
addTimeHighlight(axOverviewTime,overview_highlight_time_1,[1.00 0.75 0.30]);
addTimeHighlight(axOverviewTime,overview_highlight_time_2,[0.35 0.65 1.00]);

% Separate window in the same Figure 0 section
figure('Name','Normal tuning: EE XY trajectory and estimated normal');
axOverviewXY=axes;
xyWindowMask=isfinite(t);
if ~isempty(overview_xlim)
    xyWindowMask=xyWindowMask&t>=overview_xlim(1)&t<=overview_xlim(2);
end
plot(eePos(xyWindowMask,1),eePos(xyWindowMask,2), ...
    'Color',[0.25 0.25 0.25],'LineWidth',2.1); hold on;
validXYMask=xyWindowMask&all(isfinite(eePos(:,1:2)),2)&all(isfinite(nHat(:,1:2)),2);
idxXY=timeSpacedIndices(t,validXYMask,overview_normal_arrow_period);
quiver(eePos(idxXY,1),eePos(idxXY,2), ...
    overview_normal_arrow_scale*nHat(idxXY,1),overview_normal_arrow_scale*nHat(idxXY,2), ...
    0,'Color',[0.35 0.35 0.35],'LineWidth',overview_normal_arrow_width);
highlightColors={[0.95 0.45 0.10],[0.10 0.45 0.95]};
for highlightNumber=1:2
    interval=overviewHighlightTimes{highlightNumber};
    if isempty(interval), continue; end
    highlightMask=xyWindowMask&t>=interval(1)&t<=interval(2)& ...
        all(isfinite(eePos(:,1:2)),2)&all(isfinite(nHat(:,1:2)),2);
    highlightIdx=find(highlightMask);
    plot(eePos(highlightIdx,1),eePos(highlightIdx,2), ...
        'Color',highlightColors{highlightNumber},'LineWidth',3.0);
    highlightArrowIdx=timeSpacedIndices(t,highlightMask,overview_normal_arrow_period);
    quiver(eePos(highlightArrowIdx,1),eePos(highlightArrowIdx,2), ...
        overview_normal_arrow_scale*nHat(highlightArrowIdx,1), ...
        overview_normal_arrow_scale*nHat(highlightArrowIdx,2), ...
        0,'Color',highlightColors{highlightNumber}, ...
        'LineWidth',overview_normal_arrow_width+0.5);
end
grid on; xlabel('EE x [m]'); ylabel('EE y [m]');
title('EE XY trajectory and estimated normal');

%% Figure 1 - Vehicle / end-effector motion
figure('Name','Normal tuning: vehicle and end-effector motion');
tl=tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
ax(1)=nexttile; plot(t,vehiclePos,'LineWidth',2.1); grid on; ylabel('vehicle [m]'); legend('x','y','z');
ax(2)=nexttile; plot(t,attitude,'LineWidth',2.1); grid on; ylabel('attitude [deg]'); legend('roll','pitch','yaw');
ax(3)=nexttile; plot(t,eeCmd,'--','LineWidth',1.8); hold on; plot(t,eePos,'LineWidth',2.1); grid on; ylabel('EE [m]'); legend('cmd x','cmd y','cmd z','meas x','meas y','meas z');
ax(4)=nexttile; plot(t,vc,'LineWidth',1.8); hold on; plot(t,vcNorm,'k','LineWidth',2.7); grid on; ylabel('v_c [m/s]'); xlabel('t [s]'); legend('x','y','z','norm');
title(tl,'Vehicle, end-effector, and estimator contact velocity'); linkaxes(ax,'x');
applyTimeXLim(ax,global_xlim);

%% Figure 2 - Velocity projection diagnostics
figure('Name','Normal tuning: velocity projection');
tl=tiledlayout(5,1,'TileSpacing','compact','Padding','compact');
ax=[]; ax(end+1)=nexttile; plot(t,vcNorm,'LineWidth',2.25); grid on; ylabel('||v_c||');
ax(end+1)=nexttile; plot(t,gV,'LineWidth',1.95); grid on; ylabel('g_v'); legend(candidateLabels('\epsilon_v',epsilon_v_candidates),'Location','best');
ax(end+1)=nexttile; plot(t,thetaProjDeg,'LineWidth',1.95); grid on; ylabel('\theta_{proj} [deg]');
ax(end+1)=nexttile; plot(t,fcBar,'LineWidth',1.8); grid on; ylabel('f_c / ||f_c||'); legend('x','y','z');
ax(end+1)=nexttile; plot(t,qGSelectedUnit,'LineWidth',1.8); grid on; ylabel('q_g / ||q_g||'); xlabel('t [s]'); legend('x','y','z');
title(tl,'Velocity-dependent force projection'); linkaxes(ax,'x');
applyTimeXLim(ax,global_xlim);

%% Figure 3 - Normal and curvature estimator
figure('Name','Normal tuning: estimator');
tl=tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
ax=[]; ax(end+1)=nexttile; plot(t,nHat,'LineWidth',1.95); grid on; ylabel('nHat'); legend('x','y','z');
ax(end+1)=nexttile; plot(t,nDot,'LineWidth',1.95); grid on; ylabel('nHatDot'); legend('x','y','z');
ax(end+1)=nexttile; plot(t,nDotNorm,'k','LineWidth',2.25); grid on; ylabel('||nHatDot||');
ax(end+1)=nexttile; plot(t,kappaHat,'LineWidth',2.4); hold on; plot(t,kappaOfflineRaw,'--','LineWidth',1.65); grid on; ylabel('\kappa [1/m]'); xlabel('t [s]'); legend('firmware','offline raw');
title(tl,'Normal and curvature estimator'); linkaxes(ax,'x');
applyTimeXLim(ax,global_xlim);

%% Figure 4 - Force sanity check
figure('Name','Normal tuning: force sanity');
axForce=axes; plot(axForce,t,forceDesired,'LineWidth',2.4); hold(axForce,'on'); plot(axForce,t,fnHat,'LineWidth',2.4); grid(axForce,'on');
xlabel('t [s]'); ylabel('normal force [N]'); legend('F_d','f_n hat'); title('Desired and estimated normal force');
applyTimeXLim(axForce,global_xlim);

%% Figure 5 - Normal force and tangential velocity tracking
figure('Name','Normal tuning: contact-frame command tracking');
tl=tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
ax=[];
ax(end+1)=nexttile; plot(t,forceDesired,'LineWidth',2.4); hold on; plot(t,fnHat,'LineWidth',2.4); grid on;
ylabel('f_n [N]'); legend('desired','f hat_c dot n','Location','best');
ax(end+1)=nexttile; plot(t,u1Des,'LineWidth',2.1); hold on; plot(t,vT1Actual,'LineWidth',2.1); grid on;
ylabel('v_{t1} [m/s]'); legend('desired u_1','actual v_c dot t_1','Location','best');
ax(end+1)=nexttile; plot(t,u2Des,'LineWidth',2.1); hold on; plot(t,vT2Actual,'LineWidth',2.1); grid on;
ylabel('v_{t2} [m/s]'); xlabel('t [s]'); legend('desired u_2','actual v_c dot t_2','Location','best');
title(tl,'Normal-force and contact-frame tangential-velocity tracking'); linkaxes(ax,'x');
applyTimeXLim(ax,global_xlim);

%% Figure 6 - Shadow velocity modulation (not connected to control)
figure('Name','Normal tuning: shadow velocity modulation');
tl=tiledlayout(4,1,'TileSpacing','compact','Padding','compact');
ax=[]; ax(end+1)=nexttile; plot(t,kappaHat,'LineWidth',2.25); grid on; ylabel('\kappa hat');
ax(end+1)=nexttile; plot(t,alphaStarShadow,'LineWidth',1.95); grid on; ylabel('\alpha_* shadow'); legend(candidateLabels('aBarN',a_bar_n_candidates),'Location','best');
ax(end+1)=nexttile; plot(t,[vtDesSpeed,vtDesSpeedOrthonormal],'LineWidth',2.1); grid on; ylabel('nominal [m/s]'); legend('||u1 t1+u2 t2||','sqrt(u1^2+u2^2)');
ax(end+1)=nexttile; plot(t,vtModShadowSpeed,'LineWidth',1.95); grid on; ylabel('shadow [m/s]'); xlabel('t [s]');
title(tl,'Offline-only shadow modulation'); linkaxes(ax,'x');
applyTimeXLim(ax,global_xlim);

%% Figure 7 - 3D geometry
figure('Name','Normal tuning: 3D geometry');
plot3(eePos(:,1),eePos(:,2),eePos(:,3),'Color',[0.70 0.70 0.70],'LineWidth',3.3); hold on;
valid=find(all(isfinite(eePos),2)&all(isfinite(nHat),2));
idx=valid(1:max(1,normal_quiver_stride):end);
quiver3(eePos(idx,1),eePos(idx,2),eePos(idx,3),nHat(idx,1),nHat(idx,2),nHat(idx,3),normal_quiver_scale,'r','LineWidth',1.8);
if show_contact_frame_quivers
    quiver3(eePos(idx,1),eePos(idx,2),eePos(idx,3),t1(idx,1),t1(idx,2),t1(idx,3),normal_quiver_scale,'b'); %#ok<UNRCH>
    quiver3(eePos(idx,1),eePos(idx,2),eePos(idx,3),t2(idx,1),t2(idx,2),t2(idx,3),normal_quiver_scale,'g');
end
if ~isempty(valid)
    scatter3(eePos(valid(1),1),eePos(valid(1),2),eePos(valid(1),3),70,'g','filled');
    scatter3(eePos(valid(end),1),eePos(valid(end),2),eePos(valid(end),3),70,'m','filled');
end
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]'); view(3);
title('Measured EE trajectory and sparse estimated normals');

%% Helpers
function x=safeColumn(T,name)
name=string(name); vars=string(T.Properties.VariableNames);
idx=find(strcmpi(vars,name),1);
if isempty(idx), x=nan(height(T),1); else, x=double(T{:,idx}); end
x=x(:);
end

function xyz=columnsXYZ(T,prefix)
xyz=[safeColumn(T,string(prefix)+"X"),safeColumn(T,string(prefix)+"Y"),safeColumn(T,string(prefix)+"Z")];
end

function [unit,magnitude]=safeNormalizeRows(x,threshold)
magnitude=sqrt(sum(x.^2,2)); unit=nan(size(x));
valid=all(isfinite(x),2)&isfinite(magnitude)&magnitude>threshold;
unit(valid,:)=x(valid,:)./magnitude(valid);
end

function labels=candidateLabels(symbol,values)
labels=arrayfun(@(x)sprintf('%s=%.4g',symbol,x),values,'UniformOutput',false);
end

function applyTimeXLim(ax,limits)
if isempty(limits), return; end
validateattributes(limits,{'numeric'},{'vector','numel',2,'finite','increasing'}, ...
    mfilename,'global_xlim');
for k=1:numel(ax)
    xlim(ax(k),limits);
end
end

function addTimeHighlight(ax,interval,color)
if isempty(interval), return; end
for k=1:numel(ax)
    holdWasOn=ishold(ax(k));
    hold(ax(k),'on');
    yLimits=ylim(ax(k));
    region=patch(ax(k), ...
        [interval(1) interval(2) interval(2) interval(1)], ...
        [yLimits(1) yLimits(1) yLimits(2) yLimits(2)], ...
        color,'FaceAlpha',0.12,'EdgeColor','none', ...
        'HandleVisibility','off');
    uistack(region,'bottom');
    if ~holdWasOn, hold(ax(k),'off'); end
end
end

function idx=timeSpacedIndices(t,validMask,period)
candidates=find(validMask(:)&isfinite(t(:)));
idx=zeros(0,1);
nextTime=-inf;
for k=1:numel(candidates)
    candidate=candidates(k);
    if t(candidate)>=nextTime
        idx(end+1,1)=candidate; %#ok<AGROW>
        nextTime=t(candidate)+period;
    end
end
end
