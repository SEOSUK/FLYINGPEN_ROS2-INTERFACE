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
defaultDir = fullfile(getenv("HOME"),"hitl_ws","src","flying_pen","bag","logging", "good", "1009_1exp");
[file,path] = uigetfile({"wind*.csv","Normal-estimator CSV";"*.csv","All CSV"}, ...
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
fRaw = columnsXYZ(T,"fp_normal_fRaw");      % raw MOB force (newer CSV files)
fBar = columnsXYZ(T,"fp_normal_fBar");      % firmware thrust-effectiveness-corrected force
fnHat = safeColumn(T,"fp_normal_fnHat");
nHat = columnsXYZ(T,"fp_normal_nHat");
nDot = columnsXYZ(T,"fp_normal_nDot");      % signal actually used by firmware kappa estimator
kappaHat = safeColumn(T,"fp_normal_kappaHat");
t1 = columnsXYZ(T,"fp_normal_t1");
t2 = columnsXYZ(T,"fp_normal_t2");
u1Des = safeColumn(T,"u1_des");
u2Des = safeColumn(T,"u2_des");
forceDesired = safeColumn(T,"forceDesired");
mobTorqueRaw = columnsXYZ(T,"fp_normal_mobTorque");
etaFirmware = safeColumn(T,"fp_normal_etaHat");
batteryVoltage = safeColumn(T,"fp_normal_batteryVoltage");
mobInputForce = columnsXYZ(T,"fp_normal_mobInputForce");
mobInputTorqueBody = columnsXYZ(T,"fp_normal_mobInputTorqueBody");

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

%% Shared normal-estimation figure data
overview_xlim = [ ] ;          % [s], e.g. [80 140]; [] uses the full recording
overview_epsilon_v = 0.0009;         % [(m/s)^2], epsilon_v used for overview q_g
overview_highlight_time_1 = [ ];  % [s], pale orange; [] disables interval 1
overview_highlight_time_2 = [ ];       % [s], pale blue; [] disables interval 2
overview_highlight_time_3 = [ ];       % [s], pale green; [] disables interval 3
overview_highlight_time_4 = [ ];       % [s], pale purple; [] disables interval 4
overview_highlight_time_5 = [ ];       % [s], pale red; [] disables interval 5
overview_normal_arrow_scale = 0.125;  % [m], XY normal-arrow length
overview_normal_arrow_width = 2.4;    % XY normal-arrow line width
overview_normal_arrow_period = 2.0;   % [s], draw one XY normal arrow per interval

% Known surface geometry shared by Figure 0 true normals and Figure 0.5.
geometry_fit_xlim = [40 80];          % [s], interval containing one complete surface sweep
geometry_plane_length = 0.25;          % [m], each planar section
geometry_turn_angle_deg = 30.0;        % [deg], each circular turn
geometry_curve_radius = 0.580;         % [m], known physical curvature radius
geometry_initial_tangent_deg = 120.0;  % [deg], lower plane tangent from +X in X-Z
validateattributes(overview_normal_arrow_period,{'numeric'}, ...
    {'scalar','finite','positive'},mfilename,'overview_normal_arrow_period');
overviewHighlightTimes={overview_highlight_time_1,overview_highlight_time_2, ...
    overview_highlight_time_3,overview_highlight_time_4,overview_highlight_time_5};
for highlightNumber=1:numel(overviewHighlightTimes)
    if ~isempty(overviewHighlightTimes{highlightNumber})
        validateattributes(overviewHighlightTimes{highlightNumber},{'numeric'}, ...
            {'vector','numel',2,'finite','increasing'},mfilename, ...
            sprintf('overview_highlight_time_%d',highlightNumber));
    end
end

geometryFit=locateKnownFiveSectionGeometry(t,eePos,geometry_fit_xlim, ...
    geometry_plane_length,geometry_turn_angle_deg,geometry_curve_radius, ...
    geometry_initial_tangent_deg);
trueNormal=geometryFit.trueNormal;
fprintf(['Located known surface: origin = [%.4f %.4f %.4f] m, R = %.4f m, ' ...
    'initial tangent = %.2f deg, RMS = %.4f m, sweep = %.2f-%.2f s\n'], ...
    geometryFit.origin,geometryFit.radius,geometryFit.initialTangentDeg,geometryFit.rms, ...
    geometryFit.sweepTime);

if overview_epsilon_v <= 0
    qGOverview=fcBar;
else
    overviewProjectionScale=sum(vc.*fcBar,2)./(vc2+overview_epsilon_v);
    qGOverview=fcBar-vc.*overviewProjectionScale;
end
[qGOverview,~]=safeNormalizeRows(qGOverview,small_vector_threshold);

%% Figure 0.1 - Detailed normal-estimation overview
figure(2); clf;
set(gcf,'Name','Normal tuning: detailed overview','Position',[80 40 1550 920]);
tl=tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
axOverviewTime=gobjects(18,1);

leftTop=tiledlayout(tl,3,1,'TileSpacing','compact','Padding','compact');
leftTop.Layout.Tile=1;
axOverviewTime(1)=nexttile(leftTop); plot(t,u1Des,'--','LineWidth',1.8); hold on; plot(t,vT1Actual,'LineWidth',2.1); grid on;
ylabel('v_{t1}'); ylim([-0.2 0.2]); title('t_1 velocity'); legend('desired','actual','Location','best');
axOverviewTime(2)=nexttile(leftTop); plot(t,u2Des,'--','LineWidth',1.8); hold on; plot(t,vT2Actual,'LineWidth',2.1); grid on;
ylabel('v_{t2}'); ylim([-0.2 0.2]); title('t_2 velocity'); legend('desired','actual','Location','best');
axOverviewTime(3)=nexttile(leftTop); plot(t,forceDesired,'--','LineWidth',1.8); hold on; plot(t,fnHat,'LineWidth',2.4); grid on;
ylabel('f_n [N]'); ylim([0 0.04]); title('Normal force'); legend('desired','f hat_c dot n','Location','best');

leftBottom=tiledlayout(tl,3,1,'TileSpacing','compact','Padding','compact');
leftBottom.Layout.Tile=6;
axisNames={'x','y','z'};
for component=1:3
    axOverviewTime(3+component)=nexttile(leftBottom);
    plot(t,fcBar(:,component),'--','LineWidth',1.8); hold on;
    plot(t,qGOverview(:,component),'LineWidth',1.95); grid on;
    ylabel(axisNames{component}); ylim([-1 1]);
    if component==1, legend('q_f','q_g','Location','best'); end
end
title(leftBottom,sprintf('q_f vs. q_g (epsilon_v = %.4g)',overview_epsilon_v));

middleTop=tiledlayout(tl,3,1,'TileSpacing','compact','Padding','compact');
middleTop.Layout.Tile=2;
for component=1:3
    axOverviewTime(6+component)=nexttile(middleTop);
    plot(t,nHat(:,component),'LineWidth',1.95); hold on;
    plot(t,trueNormal(:,component),'--','LineWidth',1.8); grid on;
    ylabel(['n_' axisNames{component}]); ylim([-1 1]);
    if component==1, legend('estimated','true geometry','Location','best'); end
end
title(middleTop,'Estimated normal vs. fitted-geometry true normal');

middleBottom=tiledlayout(tl,3,1,'TileSpacing','compact','Padding','compact');
middleBottom.Layout.Tile=5;
for component=1:3
    axOverviewTime(9+component)=nexttile(middleBottom);
    plot(t,nDot(:,component),'LineWidth',1.95); grid on;
    ylabel(['dn_' axisNames{component} ' [1/s]']); ylim([-0.4 0.4]);
end
title(middleBottom,'Normal-dot estimate');

rightTop=tiledlayout(tl,3,1,'TileSpacing','compact','Padding','compact');
rightTop.Layout.Tile=3;
axOverviewTime(13)=nexttile(rightTop);
plot(t,vcNorm,'LineWidth',2.1); grid on; ylabel('|v_c|'); ylim([0 0.2]);
axOverviewTime(14)=nexttile(rightTop);
plot(t,nDotNorm,'LineWidth',2.1); grid on; ylabel('|n hat dot|'); ylim([0 0.4]);
axOverviewTime(15)=nexttile(rightTop);
plot(t,abs(kappaHat),'LineWidth',2.4); grid on; ylabel('|kappa hat|'); ylim([0 4]);
title(rightTop,'Estimator magnitudes');

rightBottom=tiledlayout(tl,3,1,'TileSpacing','compact','Padding','compact');
rightBottom.Layout.Tile=4;
attitudeNames={'roll','pitch','yaw'};
for component=1:3
    axOverviewTime(15+component)=nexttile(rightBottom);
    plot(t,attitude(:,component),'LineWidth',1.95); grid on;
    ylabel([attitudeNames{component} ' [deg]']);
    if component<=2
        ylim([-5 5]);
    else
        ylim([-10 110]);
    end
end
title(rightBottom,'Attitude');

if all(cellfun(@isempty,overviewHighlightTimes))
    overviewTitle='Normal Estimation Overview | highlight: off';
else
    intervalLabels=cell(1,numel(overviewHighlightTimes));
    for highlightNumber=1:numel(overviewHighlightTimes)
        interval=overviewHighlightTimes{highlightNumber};
        if isempty(interval)
            intervalLabels{highlightNumber}=sprintf('%d: off',highlightNumber);
        else
            intervalLabels{highlightNumber}=sprintf('%d: %.2f-%.2f s', ...
                highlightNumber,interval(1),interval(2));
        end
    end
    overviewTitle=sprintf('Normal Estimation Overview | %s', ...
        strjoin(intervalLabels,' | '));
end
title(tl,overviewTitle); xlabel(tl,'t [s]');
linkaxes(axOverviewTime,'x');
applyTimeXLim(axOverviewTime,overview_xlim);
addTimeHighlight(axOverviewTime,overview_highlight_time_1,[1.00 0.75 0.30]);
addTimeHighlight(axOverviewTime,overview_highlight_time_2,[0.35 0.65 1.00]);
addTimeHighlight(axOverviewTime,overview_highlight_time_3,[0.45 0.80 0.45]);
addTimeHighlight(axOverviewTime,overview_highlight_time_4,[0.75 0.55 0.95]);
addTimeHighlight(axOverviewTime,overview_highlight_time_5,[1.00 0.45 0.45]);

%% Figure 0 - Compact normal-estimation overview
boom_geometry = [0.07, 0.0, 0.019]; % [m], body-frame boom-tip offset [x y z]
offline_eta_gamma = 20.0;            % eta adaptation gain
offline_eta_rho = 0.1;               % eta regularization
offline_eta_initial = 1.0;           % initial eta estimate

compact_overview_xlim = [44 100];       % [s]; [] uses the full recording
compact_velocity_ylim = [-0.2 0.2; ... % t1 velocity [m/s]
                         -0.2 0.2; ... % t2 velocity [m/s]
                          0.0 0.04];    % normal force [N]
compact_normal_ylim = [-1 1; ...       % normal x
                       -1 1; ...       % normal y
                       -1 1];          % normal z
compact_force_ylim = [-0.06 0.06; ...  % force x [N]
                      -0.06 0.06; ...  % force y [N]
                      -0.06 0.06];     % force z [N]
compact_unit_force_ylim = [-1.2 1.2; ... % normalized force x
                           -1.2 1.2; ... % normalized force y
                           -1.2 1.2];    % normalized force z

% Up to five highlighted intervals; [] disables the corresponding interval.
overview_highlight_time_1 = [ ];       % [s], pale orange
overview_highlight_time_2 = [ ];       % [s], pale blue
overview_highlight_time_3 = [ ];       % [s], pale green
overview_highlight_time_4 = [ ];       % [s], pale purple
overview_highlight_time_5 = [ ];       % [s], pale red
compactHighlightTimes={overview_highlight_time_1,overview_highlight_time_2, ...
    overview_highlight_time_3,overview_highlight_time_4,overview_highlight_time_5};
for highlightNumber=1:numel(compactHighlightTimes)
    if ~isempty(compactHighlightTimes{highlightNumber})
        validateattributes(compactHighlightTimes{highlightNumber},{'numeric'}, ...
            {'vector','numel',2,'finite','increasing'},mfilename, ...
            sprintf('overview_highlight_time_%d',highlightNumber));
    end
end

[etaOffline,fBarOffline,tauBarOffline,fcHatOffline]=offlineForceRecovery( ...
    t,attitude,fRaw,mobTorqueRaw,mobInputForce,mobInputTorqueBody, ...
    boom_geometry,offline_eta_gamma,offline_eta_rho,offline_eta_initial);
[fRawUnit,~]=safeNormalizeRows(fRaw,small_vector_threshold);
[fBarUnit,~]=safeNormalizeRows(fBar,small_vector_threshold);
[fcHatUnit,~]=safeNormalizeRows(fcHat,small_vector_threshold);
[fBarOfflineUnit,~]=safeNormalizeRows(fBarOffline,small_vector_threshold);
[fcHatOfflineUnit,~]=safeNormalizeRows(fcHatOffline,small_vector_threshold);
lastEta=find(isfinite(etaOffline),1,'last');
if ~isempty(lastEta)
    fprintf('Offline force recovery: %d/%d finite samples; final eta = %.5f\n', ...
        nnz(all(isfinite(fcHatOffline),2)),height(T),etaOffline(lastEta));
end

figure(1); clf;
set(gcf,'Name','Normal estimation: compact overview','Position',[100 45 1550 920]);
tlNormalCompact=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
axNormalCompact=gobjects(12,1);

% Row 1, column 1: t1/t2 velocity tracking and normal force.
tlNormalCompactVelocity=tiledlayout(tlNormalCompact,3,1, ...
    'TileSpacing','compact','Padding','compact');
tlNormalCompactVelocity.Layout.Tile=1;
axNormalCompact(1)=nexttile(tlNormalCompactVelocity);
plot(t,u1Des,'--','LineWidth',1.8); hold on;
plot(t,vT1Actual,'LineWidth',2.1); grid on;
ylabel('v_{t1} [m/s]'); ylim(compact_velocity_ylim(1,:));
legend('desired','actual','Location','best');
axNormalCompact(2)=nexttile(tlNormalCompactVelocity);
plot(t,u2Des,'--','LineWidth',1.8); hold on;
plot(t,vT2Actual,'LineWidth',2.1); grid on;
ylabel('v_{t2} [m/s]'); ylim(compact_velocity_ylim(2,:));
legend('desired','actual','Location','best');
axNormalCompact(3)=nexttile(tlNormalCompactVelocity);
plot(t,forceDesired,'--','LineWidth',1.8); hold on;
plot(t,fnHat,'LineWidth',2.4); grid on;
ylabel('f_n [N]'); ylim(compact_velocity_ylim(3,:));
legend('desired','estimated','Location','best');
title(tlNormalCompactVelocity,'Tangential velocity and normal force');

% Row 2, column 1: true and estimated normal components.
tlNormalCompactEstimate=tiledlayout(tlNormalCompact,3,1, ...
    'TileSpacing','compact','Padding','compact');
tlNormalCompactEstimate.Layout.Tile=3;
for component=1:3
    axNormalCompact(3+component)=nexttile(tlNormalCompactEstimate);
    plot(t,trueNormal(:,component),'k--','LineWidth',1.8); hold on;
    plot(t,nHat(:,component),'LineWidth',2.0); grid on;
    ylabel(sprintf('n_%s',axisNames{component})); ylim(compact_normal_ylim(component,:));
    if component==1
        legend('true','estimated','Location','best');
    end
end
title(tlNormalCompactEstimate,'Normal: true vs. estimated');

% Row 1, column 2: raw, bar, and reconstructed contact-force components.
tlNormalCompactForce=tiledlayout(tlNormalCompact,3,1, ...
    'TileSpacing','compact','Padding','compact');
tlNormalCompactForce.Layout.Tile=2;
for component=1:3
    axNormalCompact(6+component)=nexttile(tlNormalCompactForce);
    plot(t,fRaw(:,component),'LineWidth',1.5); hold on;
    plot(t,fBar(:,component),'LineWidth',1.8);
    plot(t,fcHat(:,component),'LineWidth',2.0);
    plot(t,fBarOffline(:,component),'--','LineWidth',1.8);
    plot(t,fcHatOffline(:,component),'--','LineWidth',2.0); grid on;
    ylabel(sprintf('f_%s [N]',axisNames{component})); ylim(compact_force_ylim(component,:));
    if component==1
        legend('f raw','f bar firmware','hat_c firmware', ...
            'f bar offline','hat_c offline','Location','best');
    end
end
title(tlNormalCompactForce,'Force: firmware vs. offline MOB recovery');

% Row 2, column 2: normalized force directions and true normal.
tlNormalCompactUnit=tiledlayout(tlNormalCompact,3,1, ...
    'TileSpacing','compact','Padding','compact');
tlNormalCompactUnit.Layout.Tile=4;
for component=1:3
    axNormalCompact(9+component)=nexttile(tlNormalCompactUnit);
    plot(t,fRawUnit(:,component),'LineWidth',1.5); hold on;
    plot(t,fBarUnit(:,component),'LineWidth',1.8);
    plot(t,fcHatUnit(:,component),'LineWidth',2.0);
    plot(t,fBarOfflineUnit(:,component),'--','LineWidth',1.8);
    plot(t,fcHatOfflineUnit(:,component),'--','LineWidth',2.0);
    plot(t,trueNormal(:,component),'k--','LineWidth',1.9); grid on;
    ylabel(sprintf('unit %s',axisNames{component})); ylim(compact_unit_force_ylim(component,:));
    if component==1
        legend('f raw normalized','f bar firmware normalized', ...
            'hat_c firmware normalized','f bar offline normalized', ...
            'hat_c offline normalized','true normal','Location','best');
    end
end
title(tlNormalCompactUnit,'Normalized force vs. true normal');

title(tlNormalCompact,'Normal Estimation Overview: Velocity, Force, and Normal');
xlabel(tlNormalCompact,'t [s]');
linkaxes(axNormalCompact,'x');
applyTimeXLim(axNormalCompact,compact_overview_xlim);
for axisNumber=1:numel(axNormalCompact)
    if ~ismember(axisNumber,[6 12])
        axNormalCompact(axisNumber).XTickLabel=[];
    end
end
addTimeHighlight(axNormalCompact,overview_highlight_time_1,[1.00 0.75 0.30]);
addTimeHighlight(axNormalCompact,overview_highlight_time_2,[0.35 0.65 1.00]);
addTimeHighlight(axNormalCompact,overview_highlight_time_3,[0.45 0.80 0.45]);
addTimeHighlight(axNormalCompact,overview_highlight_time_4,[0.75 0.55 0.95]);
addTimeHighlight(axNormalCompact,overview_highlight_time_5,[1.00 0.45 0.45]);

%% Figure 0.05 - MOB recovery diagnostics
figure('Name','MOB recovery: eta, battery, and torque','Position',[150 80 1100 800]);
tlMobRecovery=tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
axMobRecovery=gobjects(3,1);
axMobRecovery(1)=nexttile(tlMobRecovery);
plot(t,etaFirmware,'--','LineWidth',1.7); hold on;
plot(t,etaOffline,'LineWidth',2.0); grid on;
ylabel('\eta'); legend('firmware','offline','Location','best');
title('Thrust-effectiveness estimate');
axMobRecovery(2)=nexttile(tlMobRecovery);
plot(t,batteryVoltage,'LineWidth',1.8); grid on;
ylabel('V_{bat} [V]'); title('Battery voltage');
axMobRecovery(3)=nexttile(tlMobRecovery);
plot(t,mobTorqueRaw,'LineWidth',1.6); hold on;
plot(t,tauBarOffline,'--','LineWidth',1.6); grid on;
ylabel('\tau_{MOB} [N m]'); xlabel('t [s]');
legend('raw \tau_x','raw \tau_y','raw \tau_z', ...
    'offline bar \tau_x','offline bar \tau_y','offline bar \tau_z', ...
    'Location','best');
title('Raw and offline-corrected MOB torque');
linkaxes(axMobRecovery,'x');
applyTimeXLim(axMobRecovery,compact_overview_xlim);
addTimeHighlight(axMobRecovery,overview_highlight_time_1,[1.00 0.75 0.30]);
addTimeHighlight(axMobRecovery,overview_highlight_time_2,[0.35 0.65 1.00]);
addTimeHighlight(axMobRecovery,overview_highlight_time_3,[0.45 0.80 0.45]);
addTimeHighlight(axMobRecovery,overview_highlight_time_4,[0.75 0.55 0.95]);
addTimeHighlight(axMobRecovery,overview_highlight_time_5,[1.00 0.45 0.45]);

%% Figure 0.25 - End-effector position
figure('Name','End-effector position','Position',[160 90 1050 760]);
tlEePosition=tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
axEePosition=gobjects(3,1);
for component=1:3
    axEePosition(component)=nexttile(tlEePosition);
    plot(t,eeCmd(:,component),'--','LineWidth',1.7); hold on;
    plot(t,eePos(:,component),'LineWidth',2.1);
    grid on; ylabel(sprintf('EE %s [m]',axisNames{component}));
    legend('command','measured','Location','best');
end
xlabel(tlEePosition,'t [s]');
title(tlEePosition,'End-effector position: command vs. measured');
linkaxes(axEePosition,'x');
applyTimeXLim(axEePosition,overview_xlim);
addTimeHighlight(axEePosition,overview_highlight_time_1,[1.00 0.75 0.30]);
addTimeHighlight(axEePosition,overview_highlight_time_2,[0.35 0.65 1.00]);
addTimeHighlight(axEePosition,overview_highlight_time_3,[0.45 0.80 0.45]);

%% Figure 0.3 - Force vectors and normalized directions
force_compare_fraw_lpf_hz = 0.5;  % [Hz], fRaw LPF cutoff; <=0 bypasses the filter
force_compare_fbar_lpf_hz = 0.5;  % [Hz], fBar LPF cutoff; <=0 bypasses the filter
force_compare_xlim = [20 80];  % [s], e.g. [40 60]; [] uses the full recording
fRawForceCompare=firstOrderLowPassRows(t,fRaw,force_compare_fraw_lpf_hz);
fBarForceCompare=firstOrderLowPassRows(t,fBar,force_compare_fbar_lpf_hz);
[fRawForceCompareUnit,~]=safeNormalizeRows(fRawForceCompare,small_vector_threshold);
[fBarForceCompareUnit,~]=safeNormalizeRows(fBarForceCompare,small_vector_threshold);
figure('Name','Force vectors: raw, bar, hat, and true normal','Position',[180 60 1450 850]);
tlForceCompare=tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
tlForceRaw=tiledlayout(tlForceCompare,3,1,'TileSpacing','compact','Padding','compact');
tlForceRaw.Layout.Tile=1;
tlForceUnit=tiledlayout(tlForceCompare,3,1,'TileSpacing','compact','Padding','compact');
tlForceUnit.Layout.Tile=2;
axForceCompare=gobjects(6,1);
for component=1:3
    axForceCompare(component)=nexttile(tlForceRaw);
    plot(t,fRawForceCompare(:,component),'LineWidth',1.6); hold on;
    plot(t,fBarForceCompare(:,component),'LineWidth',1.8);
    plot(t,fcHat(:,component),'LineWidth',2.0); grid on;
    ylabel(sprintf('f_%s [N]',axisNames{component})); ylim([-0.06 0.06]);
    if component==1, legend('f raw LPF','fbar LPF','fhat_c','Location','best'); end

    axForceCompare(3+component)=nexttile(tlForceUnit);
    plot(t,fRawForceCompareUnit(:,component),'LineWidth',1.6); hold on;
    plot(t,fBarForceCompareUnit(:,component),'LineWidth',1.8);
    plot(t,fcHatUnit(:,component),'LineWidth',2.1);
    plot(t,trueNormal(:,component),'k--','LineWidth',1.9); grid on;
    ylabel(sprintf('unit %s',axisNames{component})); ylim([-1.2 1.2]);
    if component==1
        legend('f raw LPF / ||f raw LPF||','fbar LPF / ||fbar LPF||', ...
            'fhat_c / ||fhat_c||','true normal','Location','best');
    end
end
title(tlForceRaw,'Force components'); xlabel(tlForceRaw,'t [s]');
title(tlForceUnit,'Normalized force directions and true normal'); xlabel(tlForceUnit,'t [s]');
linkaxes(axForceCompare,'x');
applyTimeXLim(axForceCompare,force_compare_xlim);
addTimeHighlight(axForceCompare,overview_highlight_time_1,[1.00 0.75 0.30]);
addTimeHighlight(axForceCompare,overview_highlight_time_2,[0.35 0.65 1.00]);
addTimeHighlight(axForceCompare,overview_highlight_time_3,[0.45 0.80 0.45]);

%% Figure 0.5 - EE 3-D trajectory and fitted surface
% Figure 0.5 display parameters
overview_3d_time = overview_xlim;     % [s], defaults to the overview window
overview_3d_xlim = [];                % [m], [] chooses a tight automatic limit
overview_3d_ylim = [];                % [m], [] uses surface width plus margin
overview_3d_zlim = [];                % [m], [] uses MATLAB automatic limits
overview_3d_surface_y_half_width = 0.20; % [m], surface half-width about fitted origin_y
overview_3d_axis_margin = 0.03;        % [m], automatic-limit margin
overview_3d_normal_period = 2.0;       % [s], interval between normal arrows
overview_3d_normal_length = 0.125;     % [m], normal-arrow total length
overview_3d_arrow_shaft_radius = 0.006; % [m], RViz-style arrow shaft radius
overview_3d_arrow_head_radius = 0.014;  % [m], RViz-style arrow head radius
overview_3d_arrow_head_fraction = 0.28; % [-], head length / total arrow length

% Separate rotatable 3-D window; default camera looks onto the X-Z plane.
surfaceYLimits=geometryFit.origin(2)+[-1 1]*overview_3d_surface_y_half_width;
figOverview3D=figure('Name','Normal tuning: EE 3-D trajectory and estimated normal');
axOverview3D=axes(figOverview3D);
trajectoryWindowMask=isfinite(t);
if ~isempty(overview_3d_time)
    trajectoryWindowMask=trajectoryWindowMask&t>=overview_3d_time(1)&t<=overview_3d_time(2);
end

% Sweep the fitted plane-curve-plane-curve-plane centerline through Y.
surfaceX=repmat(geometryFit.centerline(:,1),1,2);
surfaceY=repmat(surfaceYLimits,size(geometryFit.centerline,1),1);
surfaceZ=repmat(geometryFit.centerline(:,2),1,2);
surf(axOverview3D,surfaceX,surfaceY,surfaceZ, ...
    'FaceColor',[0.45 0.70 0.90],'FaceAlpha',0.22, ...
    'EdgeColor','none','HandleVisibility','off');
hold(axOverview3D,'on');

plot3(axOverview3D,eePos(trajectoryWindowMask,1),eePos(trajectoryWindowMask,2), ...
    eePos(trajectoryWindowMask,3), ...
    'Color',[0.25 0.25 0.25],'LineWidth',2.1);
validTrajectoryMask=trajectoryWindowMask&all(isfinite(eePos),2)&all(isfinite(nHat),2);
idxTrajectory=timeSpacedIndices(t,validTrajectoryMask,overview_3d_normal_period);
drawRvizArrows3D(axOverview3D,eePos(idxTrajectory,:), ...
    overview_3d_normal_length*nHat(idxTrajectory,:),[0.35 0.35 0.35], ...
    overview_3d_arrow_shaft_radius,overview_3d_arrow_head_radius, ...
    overview_3d_arrow_head_fraction);
highlightColors={[0.95 0.45 0.10],[0.10 0.45 0.95],[0.20 0.65 0.25]};
for highlightNumber=1:numel(overviewHighlightTimes)
    interval=overviewHighlightTimes{highlightNumber};
    if isempty(interval), continue; end
    highlightMask=trajectoryWindowMask&t>=interval(1)&t<=interval(2)& ...
        all(isfinite(eePos),2)&all(isfinite(nHat),2);
    highlightIdx=find(highlightMask);
    plot3(axOverview3D,eePos(highlightIdx,1),eePos(highlightIdx,2),eePos(highlightIdx,3), ...
        'Color',highlightColors{highlightNumber},'LineWidth',3.0);
    highlightArrowIdx=timeSpacedIndices(t,highlightMask,overview_3d_normal_period);
    drawRvizArrows3D(axOverview3D,eePos(highlightArrowIdx,:), ...
        overview_3d_normal_length*nHat(highlightArrowIdx,:), ...
        highlightColors{highlightNumber},overview_3d_arrow_shaft_radius, ...
        overview_3d_arrow_head_radius,overview_3d_arrow_head_fraction);
end
grid(axOverview3D,'on'); axis(axOverview3D,'equal'); axis(axOverview3D,'vis3d');
xlabel(axOverview3D,'EE x [m]'); ylabel(axOverview3D,'EE y [m]'); zlabel(axOverview3D,'EE z [m]');
if ~isempty(overview_3d_xlim)
    xlim(axOverview3D,overview_3d_xlim);
elseif any(validTrajectoryMask)
    xGeometry=[eePos(validTrajectoryMask,1); ...
        eePos(validTrajectoryMask,1)+overview_3d_normal_length*nHat(validTrajectoryMask,1)];
    xlim(axOverview3D,[min(xGeometry)-overview_3d_axis_margin, ...
        max(xGeometry)+overview_3d_axis_margin]);
end
if isempty(overview_3d_ylim)
    ylim(axOverview3D,surfaceYLimits+[-1 1]*overview_3d_axis_margin);
else
    ylim(axOverview3D,overview_3d_ylim);
end
if ~isempty(overview_3d_zlim)
    zlim(axOverview3D,overview_3d_zlim);
end
view(axOverview3D,0,0);
rotate3d(figOverview3D,'on');
title(axOverview3D,sprintf( ...
    'EE trajectory, fitted surface, and estimated normal (R = %.3f m)',geometryFit.radius));

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
function [etaHat,fBar,tauBar,fHatC]=offlineForceRecovery( ...
    t,attitudeDeg,fRaw,tauRaw,fInputWorld,tauInputBody,boomBody,gamma,rho,etaInitial)
% Firmware-equivalent eta correction and point-contact wrench reconstruction.
n=numel(t);
etaHat=nan(n,1); fBar=nan(n,3); tauBar=nan(n,3); fHatC=nan(n,3);
eta=max(etaInitial,1.0e-6);
for k=1:n
    if k==1
        dt=0;
    else
        dt=t(k)-t(k-1);
    end
    valid=isfinite(dt)&&dt>=0&&all(isfinite(attitudeDeg(k,:)))&& ...
        all(isfinite(fRaw(k,:)))&&all(isfinite(tauRaw(k,:)))&& ...
        all(isfinite(fInputWorld(k,:)))&&all(isfinite(tauInputBody(k,:)));
    if ~valid
        etaHat(k)=eta;
        continue;
    end

    roll=deg2rad(attitudeDeg(k,1));
    pitch=deg2rad(attitudeDeg(k,2));
    yaw=deg2rad(attitudeDeg(k,3));
    cr=cos(roll); sr=sin(roll); cp=cos(pitch); sp=sin(pitch);
    cy=cos(yaw); sy=sin(yaw);
    rotationBodyToWorld=[ ...
        cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr; ...
        sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr; ...
        -sp,   cp*sr,          cp*cr];
    boomWorld=(rotationBodyToWorld*boomBody(:)).';
    tauInputWorld=(rotationBodyToWorld*tauInputBody(k,:).').';

    yEta=cross(boomWorld,fInputWorld(k,:))-tauInputWorld;
    epsilonTau=cross(boomWorld,fRaw(k,:))-tauRaw(k,:);
    epsilonEta=epsilonTau-yEta*(eta-1.0);
    denominator=max(rho,1.0e-9)+dot(yEta,yEta);
    eta=max(1.0e-6,eta+dt*max(gamma,0)*dot(yEta,epsilonEta)/denominator);

    fBar(k,:)=fRaw(k,:)-(eta-1.0)*fInputWorld(k,:);
    tauBar(k,:)=tauRaw(k,:)-(eta-1.0)*tauInputWorld;
    boomNormSquared=dot(boomWorld,boomWorld);
    if boomNormSquared>1.0e-12
        parallelForce=boomWorld*(dot(boomWorld,fBar(k,:))/boomNormSquared);
        transverseForce=-cross(boomWorld,tauBar(k,:))/boomNormSquared;
        fHatC(k,:)=parallelForce+transverseForce;
    else
        fHatC(k,:)=fBar(k,:);
    end
    etaHat(k)=eta;
end
end

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

function filtered=firstOrderLowPassRows(t,signal,cutoffHz)
% Causal first-order LPF for irregularly sampled row-vector signals.
if cutoffHz<=0
    filtered=signal;
    return;
end
validateattributes(cutoffHz,{'numeric'},{'scalar','finite','positive'}, ...
    mfilename,'force_compare_fbar_lpf_hz');
filtered=nan(size(signal));
state=nan(1,size(signal,2));
for sample=1:size(signal,1)
    finiteComponent=isfinite(signal(sample,:));
    initialize=finiteComponent&~isfinite(state);
    state(initialize)=signal(sample,initialize);
    if sample>1
        dt=t(sample)-t(sample-1);
        if isfinite(dt)&&dt>0
            alpha=1-exp(-2*pi*cutoffHz*dt);
            update=finiteComponent&isfinite(state);
            state(update)=state(update)+alpha.*(signal(sample,update)-state(update));
        end
    end
    filtered(sample,finiteComponent)=state(finiteComponent);
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

function fit=locateKnownFiveSectionGeometry(t,eePos,timeWindow,planeLength, ...
    turnAngleDeg,curveRadius,initialTangentDeg)
% Estimate translation only for a known plane-arc-plane-arc-plane geometry.
sampleValid=isfinite(t)&all(isfinite(eePos),2);
fitValid=sampleValid;
if ~isempty(timeWindow)
    fitValid=fitValid&t>=timeWindow(1)&t<=timeWindow(2);
end
windowIdx=find(fitValid);
if numel(windowIdx)<20
    error('Not enough finite EE samples in overview_xlim to fit the surface.');
end

zWindow=eePos(windowIdx,3);
zMin=min(zWindow); zMax=max(zWindow); zSpan=zMax-zMin;
lowThreshold=zMin+0.04*zSpan;
highThreshold=zMax-0.04*zSpan;
firstHigh=find(zWindow>=highThreshold,1,'first');
lastLowBeforeHigh=[];
if ~isempty(firstHigh)
    lastLowBeforeHigh=find(zWindow(1:firstHigh)<=lowThreshold,1,'last');
end
firstLow=find(zWindow<=lowThreshold,1,'first');
lastHighBeforeLow=[];
if ~isempty(firstLow)
    lastHighBeforeLow=find(zWindow(1:firstLow)>=highThreshold,1,'last');
end

if ~isempty(lastLowBeforeHigh)
    sweepLocal=lastLowBeforeHigh:firstHigh;
elseif ~isempty(lastHighBeforeLow)
    sweepLocal=fliplr(lastHighBeforeLow:firstLow);
else
    error('Could not detect a complete low-to-high or high-to-low sweep in overview_xlim.');
end
sweepIdx=windowIdx(sweepLocal);

% Spatial binning prevents long dwell periods from dominating the fit.
zSweep=eePos(sweepIdx,3);
binEdges=linspace(min(zSweep),max(zSweep),81);
fitPoints=zeros(0,2);
for binNumber=1:numel(binEdges)-1
    inBin=zSweep>=binEdges(binNumber)&zSweep<binEdges(binNumber+1);
    if binNumber==numel(binEdges)-1
        inBin=inBin|zSweep==binEdges(end);
    end
    if any(inBin)
        fitPoints(end+1,:)=[median(eePos(sweepIdx(inBin),1)), ... %#ok<AGROW>
            median(eePos(sweepIdx(inBin),3))];
    end
end
pathDistance=[0;cumsum(vecnorm(diff(fitPoints,1,1),2,2))];
u=pathDistance/pathDistance(end);
initialTheta=deg2rad(initialTangentDeg);
localQ=[0 0 initialTheta curveRadius];
localPoints=evaluateFiveSection(localQ,u,planeLength,turnAngleDeg);
denseU=linspace(0,1,501)';
localCenterline=evaluateFiveSection(localQ,denseU,planeLength,turnAngleDeg);
originGuess=median(fitPoints-localPoints,1);
originXZ=fminsearch(@(origin)translationOnlyGeometryCost( ...
    origin,fitPoints,localCenterline),originGuess,optimset('Display','off'));
q=[originXZ initialTheta curveRadius];

[centerline,tangentAngle]=evaluateFiveSection(q,denseU,planeLength,turnAngleDeg);
denseNormal=[-sin(tangentAngle),zeros(size(tangentAngle)),cos(tangentAngle)];
fitDistance=zeros(size(fitPoints,1),1);
for pointNumber=1:size(fitPoints,1)
    fitDistance(pointNumber)=sqrt(min(sum((centerline-fitPoints(pointNumber,:)).^2,2)));
end

trueNormal=nan(size(eePos));
queryIdx=find(sampleValid);
for sampleNumber=1:numel(queryIdx)
    sample=queryIdx(sampleNumber);
    [~,nearest]=min(sum((centerline-eePos(sample,[1 3])).^2,2));
    trueNormal(sample,:)=denseNormal(nearest,:);
end

fit.origin=[q(1),median(eePos(sweepIdx,2)),q(2)];
fit.radius=q(4);
fit.initialTangentDeg=rad2deg(q(3));
fit.rms=sqrt(mean(fitDistance.^2));
fit.centerline=centerline;
fit.trueNormal=trueNormal;
fit.sweepTime=[t(sweepIdx(1)) t(sweepIdx(end))];
end

function cost=translationOnlyGeometryCost(origin,points,localCenterline)
centerline=localCenterline+origin;
distance=zeros(size(points,1),1);
for pointNumber=1:size(points,1)
    distance(pointNumber)=sqrt(min(sum((centerline-points(pointNumber,:)).^2,2)));
end
clipDistance=0.08;
cost=mean(min(distance,clipDistance).^2);
end

function [points,tangentAngle]=evaluateFiveSection(q,u,planeLength,turnAngleDeg)
origin=q(1:2); initialTheta=q(3); radius=q(4);
turn=-abs(deg2rad(turnAngleDeg));
segmentLengths=[planeLength,radius*abs(turn),planeLength, ...
    radius*abs(turn),planeLength];
boundaries=[0 cumsum(segmentLengths)];
arcLength=u(:)*boundaries(end);
points=nan(numel(u),2); tangentAngle=nan(numel(u),1);
position=origin; theta=initialTheta;
for segment=1:5
    if segment<5
        selected=arcLength>=boundaries(segment)&arcLength<boundaries(segment+1);
    else
        selected=arcLength>=boundaries(segment)&arcLength<=boundaries(segment+1)+eps;
    end
    localLength=arcLength(selected)-boundaries(segment);
    if mod(segment,2)==1
        points(selected,:)=position+localLength.*[cos(theta) sin(theta)];
        tangentAngle(selected)=theta;
        position=position+planeLength*[cos(theta) sin(theta)];
    else
        curvature=sign(turn)/radius;
        localTurn=curvature*localLength;
        points(selected,1)=position(1)+(sin(theta+localTurn)-sin(theta))/curvature;
        points(selected,2)=position(2)+(-cos(theta+localTurn)+cos(theta))/curvature;
        tangentAngle(selected)=theta+localTurn;
        position=position+[ ...
            (sin(theta+turn)-sin(theta))/curvature, ...
            (-cos(theta+turn)+cos(theta))/curvature];
        theta=theta+turn;
    end
end
end

function drawRvizArrows3D(ax,origins,vectors,color,shaftRadius,headRadius,headFraction)
% Draw solid cylinder-and-cone arrows similar to RViz Marker arrows.
if isempty(origins), return; end
theta=linspace(0,2*pi,12);
headFraction=min(max(headFraction,0.05),0.90);
for arrowNumber=1:size(origins,1)
    origin=origins(arrowNumber,:);
    vector=vectors(arrowNumber,:);
    arrowLength=norm(vector);
    if ~all(isfinite([origin vector])) || arrowLength<=eps
        continue;
    end

    direction=vector/arrowLength;
    reference=[0 0 1];
    if abs(dot(direction,reference))>0.90
        reference=[0 1 0];
    end
    radial1=cross(direction,reference);
    radial1=radial1/norm(radial1);
    radial2=cross(direction,radial1);
    circle=cos(theta(:))*radial1+sin(theta(:))*radial2;

    headLength=headFraction*arrowLength;
    shaftLength=arrowLength-headLength;
    shaftEnd=origin+shaftLength*direction;
    tip=origin+vector;

    shaftRing=shaftRadius*circle;
    surf(ax,[origin(1)+shaftRing(:,1)';shaftEnd(1)+shaftRing(:,1)'], ...
        [origin(2)+shaftRing(:,2)';shaftEnd(2)+shaftRing(:,2)'], ...
        [origin(3)+shaftRing(:,3)';shaftEnd(3)+shaftRing(:,3)'], ...
        'FaceColor',color,'EdgeColor','none','HandleVisibility','off');

    headRing=headRadius*circle;
    surf(ax,[shaftEnd(1)+headRing(:,1)';repmat(tip(1),1,numel(theta))], ...
        [shaftEnd(2)+headRing(:,2)';repmat(tip(2),1,numel(theta))], ...
        [shaftEnd(3)+headRing(:,3)';repmat(tip(3),1,numel(theta))], ...
        'FaceColor',color,'EdgeColor','none','HandleVisibility','off');
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
