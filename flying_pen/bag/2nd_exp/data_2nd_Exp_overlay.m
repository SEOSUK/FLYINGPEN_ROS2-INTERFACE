%% Second-experiment path-aligned overlay analysis
% Multiple runs are aligned by normalized XY arc length, not by time.
clear; close all; clc;

%% User settings
path_xlim=[0 99];                    % normalized fitted-loop progress [%]
time_xlim=[0 97.5];                     % [s], time-aligned overview
geometry_fit_xlim=[0 100.6];
geometry_large_radius=0.420; geometry_small_radius=0.280;
geometry_plane_length_guess=1.0;
trajectory_contact_end_time=76.0;     % [s], alpha-OFF trajectory leaves the surface
alpha_off_first_low_curvature_delay=2.0; % [s], delay first gentle-curve interval
align_path_by_ground_truth=true;       % use ON run's full path length as common scale
alpha_off_path_shift=3.5;              % [% point], positive shifts OFF to the right
alpha_off_contact_loss_path=90.399;    % [%], discard OFF data after this point except force
normal_arrow_path_period=1.6;         % shared overview/crop arrow spacing [% path]
normal_arrow_record_stagger=0.3;     % ON/OFF arrow phase offset [fraction of spacing]
normal_arrow_length=0.10;             % [m]
crop_arrow_period_scale=0.48;         % denser relative spacing in the zoomed crop
crop_positive_y_limit=1.3;            % crop upper EE-y limit [m]
path_trajectory_positive_xlim=1.4;    % path-aligned trajectory positive x-axis limit [m]
a_bar_n=0.005; small=1.0e-8;

%% Portable multi-file loading
dataDir=activeScriptDirectory();
[selectedFiles,selectedPath]=uigetfile( ...
    {'alpha*.csv','Normal-estimator CSV files (*.csv)';'*.csv','All CSV files'}, ...
    'Select second-experiment CSV files',dataDir,'MultiSelect','on');
if isequal(selectedFiles,0), return; end
if ischar(selectedFiles)||isstring(selectedFiles), selectedFiles=cellstr(selectedFiles); end
selectedFiles=sort(selectedFiles);

records=struct([]);
for fileIndex=1:numel(selectedFiles)
    csvFile=fullfile(selectedPath,selectedFiles{fileIndex});
    fprintf('Loading %s\n',csvFile);
    T=readtable(csvFile,detectImportOptions(csvFile,'Delimiter',','));
    r=prepareRecord(T,selectedFiles{fileIndex},geometry_fit_xlim, ...
        geometry_small_radius,geometry_large_radius,geometry_plane_length_guess, ...
        trajectory_contact_end_time,alpha_off_first_low_curvature_delay,a_bar_n,small);
    if fileIndex==1, records=r; else, records(fileIndex)=r; end
    fprintf(['  %s | path length %.3f m | lap boundary %.1f%% | fit RMS %.2f mm | ' ...
        'loop %.2f-%.2f s\n'],r.label,r.pathLength,r.lapBoundaryPercent, ...
        1e3*r.geometry.rms,r.geometry.loopTime);
end
if align_path_by_ground_truth
    records=alignPathByReferenceDistance(records,alpha_off_path_shift);
    fprintf('Path progress uses fitted two-lap ON reference length.\n');
end
records=trimAlphaOffAfterContactLoss(records,alpha_off_contact_loss_path);
colors=recordColors(records);
labels=cellfun(@experimentLabel,{records.label},'UniformOutput',false);

%% 0. Path-aligned overview panel
plot_window_width=680;                % shared width for every plot window [px]
subplot_row_height=115;               % shared height of one tiled subplot row [px]
path_window_origin=[40 60];              % window group lower-left [px]
path_overview_window_size=[plot_window_width 3*subplot_row_height];
path_overview_window_gap=[20 45];        % [horizontal vertical] gap [px]

plotOverviewPanel(records,labels,colors,path_xlim,path_window_origin, ...
    path_overview_window_size,path_overview_window_gap, ...
    normal_arrow_path_period,crop_arrow_period_scale,normal_arrow_record_stagger,normal_arrow_length,'path', ...
    geometry_small_radius,geometry_large_radius,crop_positive_y_limit, ...
    path_trajectory_positive_xlim);

%% 1. Time-aligned overview panel
time_window_origin=[100 120];            % window group lower-left [px]
time_overview_window_size=[plot_window_width 3*subplot_row_height];
time_overview_window_gap=[20 45];        % [horizontal vertical] gap [px]

plotOverviewPanel(records,labels,colors,time_xlim,time_window_origin, ...
    time_overview_window_size,time_overview_window_gap, ...
    1.0,crop_arrow_period_scale,normal_arrow_record_stagger,normal_arrow_length,'time', ...
    geometry_small_radius,geometry_large_radius,crop_positive_y_limit,[]);

%% Local functions
function plotOverviewPanel(records,labels,colors,xLimits,windowOrigin,windowSize,windowGap, ...
    arrowPeriod,cropArrowScale,arrowStagger,arrowLength,alignmentMode,smallRadius,largeRadius, ...
    cropPositiveYLimit,trajectoryPositiveXLimit)
windowPositions=overviewWindowPositions(windowOrigin,windowSize,windowGap);
windowNames={'trajectory','normal vector','normal rate','force / curvature / velocity', ...
    'trajectory crop: second high curvature'};
panels=gobjects(5,1);
for k=1:5
    fig=figure('Name',sprintf('2nd experiment: %s-aligned %s',alignmentMode,windowNames{k}), ...
        'Color','w','Position',windowPositions(k,:));
    panels(k)=uipanel(fig,'Units','normalized','Position',[0 0 1 1], ...
        'BackgroundColor','w','BorderType','none');
end

% Top left: physical XY trajectories and path-spaced estimated normals.
ax=axes(panels(1)); hold(ax,'on');
plotOrder=recordPlotOrder(records);
xyCenter=trajectoryCenter(records); % [EE x, EE y]
allPlotX=[]; allPlotY=[];
[meanX,meanY]=meanFirstTrialTrajectory(records,xyCenter);
plot(ax,meanX,meanY,'Color',[0.78 0.78 0.78],'LineWidth',4.0, ...
    'HandleVisibility','off');
% Draw both runs lightly in their own colors above the mean trajectory.
for plotIndex=1:numel(plotOrder)
    k=plotOrder(plotIndex);
    r=records(k); coordinate=alignmentCoordinate(r,alignmentMode);
    valid=isfinite(coordinate)&coordinate>=xLimits(1)&coordinate<=xLimits(2)& ...
        all(isfinite(r.eePos(:,1:2)),2);
    firstTrial=isfinite(r.pathPercent)&r.pathPercent<=r.lapBoundaryPercent;
    if isAlphaOff(r.label)
        % Outside the dedicated crop, stop the EE-vs-normal trajectory at
        % contact loss. trimAlphaOffAfterContactLoss() marks nHat invalid
        % there while preserving eePos for the crop continuation.
        trajectoryValid=valid&all(isfinite(r.nHat(:,1:2)),2);
    else
        trajectoryValid=valid&firstTrial;
    end
    % Keep the physical trajectory within the same displayed coordinate
    % range.  Previously the path-aligned trace ignored xLimits, leaving
    % out-of-range segments visible (most noticeably in quadrant II).
    centeredX=r.eePos(trajectoryValid,2)-xyCenter(2);
    centeredY=r.eePos(trajectoryValid,1)-xyCenter(1);
    allPlotX=[allPlotX;centeredX]; %#ok<AGROW>
    allPlotY=[allPlotY;centeredY]; %#ok<AGROW>
    trajectoryColor=colors(k,:);
    plot(ax,centeredX,centeredY, ...
        'Color',trajectoryColor, ...
        'LineWidth',1.7, ...
        'DisplayName',labels{k});
end
% Curvature overlays and arrows follow the same OFF-then-ON layer order.
for plotIndex=1:numel(plotOrder)
    k=plotOrder(plotIndex);
    r=records(k); c=colors(k,:); coordinate=alignmentCoordinate(r,alignmentMode);
    valid=isfinite(coordinate)&coordinate>=xLimits(1)&coordinate<=xLimits(2)& ...
        all(isfinite(r.eePos(:,1:2)),2)&isfinite(r.pathPercent)& ...
        r.pathPercent<=r.lapBoundaryPercent;
    [highCurvature,lowCurvature]=curvatureMasks(r.trueCurvature,smallRadius,largeRadius);
    [highColor,lowColor]=curvatureColors();
    plotMaskedTrajectory(ax,r.eePos(:,2)-xyCenter(2),r.eePos(:,1)-xyCenter(1), ...
        valid&highCurvature,highColor);
    plotMaskedTrajectory(ax,r.eePos(:,2)-xyCenter(2),r.eePos(:,1)-xyCenter(1), ...
        valid&lowCurvature,lowColor);
    arrowOffset=mod(k-1,2)*arrowStagger*arrowPeriod;
    arrows=pathSpacedIndices(coordinate,valid&isfinite(r.trueCurvature)& ...
        all(isfinite(r.nHat(:,1:2)),2),arrowPeriod,arrowOffset);
    quiver(ax,r.eePos(arrows,2)-xyCenter(2),r.eePos(arrows,1)-xyCenter(1), ...
        arrowLength*r.nHat(arrows,2), ...
        arrowLength*r.nHat(arrows,1),0,'Color',c,'LineWidth',2.0, ...
        'MaxHeadSize',0.7,'HandleVisibility','off');
end
[highColor,lowColor]=curvatureColors();
plot(ax,nan,nan,'Color',highColor,'LineWidth',4.0,'DisplayName','high curvature');
plot(ax,nan,nan,'Color',lowColor,'LineWidth',4.0,'DisplayName','low curvature');
axis(ax,'equal'); grid(ax,'on'); box(ax,'on'); xlabel(ax,'EE y [m]'); ylabel(ax,'EE x [m]');
title(ax,'XY Trajectory and Estimated Normal'); showRecordLegend(ax,numel(records));
[symmetricXLim,symmetricYLim]=symmetricTrajectoryLimits(allPlotX,allPlotY);
arrowMargin=1.3*arrowLength;
symmetricXLim=symmetricXLim+[-arrowMargin arrowMargin];
symmetricYLim=symmetricYLim+[-arrowMargin arrowMargin];
if ~isempty(trajectoryPositiveXLimit)
    symmetricXLim(2)=trajectoryPositiveXLimit;
end
xlim(ax,symmetricXLim); ylim(ax,symmetricYLim);

% Separate spatial crop around the second high-curvature section.
cropAx=axes(panels(5)); hold(cropAx,'on');
plot(cropAx,meanX,meanY,'Color',[0.78 0.78 0.78],'LineWidth',4.0, ...
    'HandleVisibility','off');
cropPlotOrder=fliplr(plotOrder); % ON first, OFF last/on top in the crop.
cropArrowPeriod=cropArrowScale*arrowPeriod;
offPhase=0;
offIndex=find(cellfun(@isAlphaOff,{records.label}),1);
if ~isempty(offIndex)
    offValid=isfinite(records(offIndex).pathPercent)& ...
        all(isfinite(records(offIndex).nHat(:,1:2)),2);
    offLast=find(offValid,1,'last');
    if ~isempty(offLast)
        offPhase=mod(records(offIndex).pathPercent(offLast),cropArrowPeriod);
    end
end
for plotIndex=1:numel(cropPlotOrder)
    k=cropPlotOrder(plotIndex); r=records(k); c=colors(k,:);
    coordinate=alignmentCoordinate(r,alignmentMode);
    valid=isfinite(coordinate)&coordinate>=xLimits(1)&coordinate<=xLimits(2)& ...
        all(isfinite(r.eePos(:,1:2)),2);
    trajectoryColor=c;
    plot(cropAx,r.eePos(valid,2)-xyCenter(2),r.eePos(valid,1)-xyCenter(1), ...
        'Color',trajectoryColor,'LineWidth',1.7,'DisplayName',labels{k});
    [highCurvature,lowCurvature]=curvatureMasks(r.trueCurvature,smallRadius,largeRadius);
    plotMaskedTrajectory(cropAx,r.eePos(:,2)-xyCenter(2),r.eePos(:,1)-xyCenter(1), ...
        valid&highCurvature,highColor);
    plotMaskedTrajectory(cropAx,r.eePos(:,2)-xyCenter(2),r.eePos(:,1)-xyCenter(1), ...
        valid&lowCurvature,lowColor);
    % Crop arrows use path progress for both runs, so their spacing is identical.
    secondTrial=isfinite(r.pathPercent)&r.pathPercent>=r.lapBoundaryPercent;
    cropArrowCoordinate=r.pathPercent;
    cropArrowValid=valid&secondTrial&all(isfinite(r.nHat(:,1:2)),2);
    if isAlphaOff(r.label)
        % Use the phase position previously occupied by ON.
        arrowOffset=mod(offPhase-arrowStagger*cropArrowPeriod,cropArrowPeriod);
    else
        % Use the phase position previously occupied by OFF.
        arrowOffset=offPhase;
        % If the displayed second lap ends before completing the lower
        % branch, fill only that missing tail with the corresponding first-
        % lap samples. Map them onto the second-lap path coordinate so the
        % arrow grid remains continuous and evenly spaced.
        firstTrial=isfinite(r.pathPercent)&r.pathPercent<r.lapBoundaryPercent;
        mappedFirstTrialPath=r.pathPercent+r.lapBoundaryPercent;
        secondTrialPath=r.pathPercent(cropArrowValid);
        if ~isempty(secondTrialPath)
            lastSecondTrialPath=max(secondTrialPath);
            previousTrialFill=valid&firstTrial&all(isfinite(r.nHat(:,1:2)),2)& ...
                mappedFirstTrialPath>lastSecondTrialPath;
            cropArrowCoordinate(previousTrialFill)=mappedFirstTrialPath(previousTrialFill);
            cropArrowValid=cropArrowValid|previousTrialFill;
        end
    end
    arrows=pathSpacedIndices(cropArrowCoordinate,cropArrowValid,cropArrowPeriod,arrowOffset);
    arrowNormals=r.nHat(arrows,1:2);
    if isAlphaOff(r.label)&&~isempty(arrows)
        % Add exactly one post-contact-loss arrow. Its origin spacing and
        % held direction both come from the final displayed pre-loss arrow.
        lastArrow=arrows(end);
        lastPreLossSample=find(cropArrowValid,1,'last');
        continuationTargets=r.pathPercent(lastArrow)+cropArrowPeriod;
        afterContact=valid&secondTrial&isfinite(r.pathPercent)& ...
            r.pathPercent>r.pathPercent(lastPreLossSample);
        continuationCandidates=find(afterContact);
        if ~isempty(continuationCandidates)
            continuationArrows=zeros(0,1);
            continuationPath=r.pathPercent(continuationCandidates);
            for continuationIndex=1:numel(continuationTargets)
                [distance,nearest]=min(abs(continuationPath- ...
                    continuationTargets(continuationIndex)));
                if distance<=0.5*cropArrowPeriod
                    continuationArrows(end+1,1)=continuationCandidates(nearest); %#ok<AGROW>
                end
            end
            continuationArrows=unique(continuationArrows,'stable');
            if ~isempty(continuationArrows)
                heldNormal=repmat(r.nHat(lastArrow,1:2),numel(continuationArrows),1);
                arrows=[arrows;continuationArrows]; %#ok<AGROW>
                arrowNormals=[arrowNormals;heldNormal]; %#ok<AGROW>
            end
        end
    end
    % Draw each record's arrows as one quiver object so MATLAB applies an
    % identical shaft and arrowhead geometry to the continuation arrow.
    quiver(cropAx,r.eePos(arrows,2)-xyCenter(2),r.eePos(arrows,1)-xyCenter(1), ...
        arrowLength*arrowNormals(:,2),arrowLength*arrowNormals(:,1),0, ...
        'Color',c,'LineWidth',2.0,'MaxHeadSize',0.7,'HandleVisibility','off');
end
plot(cropAx,nan,nan,'Color',highColor,'LineWidth',4.0,'DisplayName','high curvature');
plot(cropAx,nan,nan,'Color',lowColor,'LineWidth',4.0,'DisplayName','low curvature');
axis(cropAx,'equal'); grid(cropAx,'on'); box(cropAx,'on');
xlabel(cropAx,'EE y [m]'); ylabel(cropAx,'EE x [m]');
title(cropAx,'XY Trajectory and Estimated Normal: 2nd High-Curvature Crop');
cropLimits=secondHighCurvatureCropLimits(records,alignmentMode,xLimits,xyCenter, ...
    smallRadius,largeRadius,arrowLength,windowSize(1)/windowSize(2));
if ~isempty(cropLimits)
    cropLimits(2)=min(cropLimits(2),cropPositiveYLimit);
    xlim(cropAx,cropLimits(1:2)); ylim(cropAx,cropLimits(3:4));
end
showRecordLegend(cropAx,numel(records));

% Bottom left: true/estimated normal components.
layout=tiledlayout(panels(2),3,1,'TileSpacing','compact','Padding','compact');
normalAxes=gobjects(3,1); componentNames={'x','y','z'};
for component=1:3
    normalAxes(component)=nexttile(layout); hold(normalAxes(component),'on');
    % Ground-truth baselines are drawn first so all measurements stay above them.
    for k=1:numel(records)
        r=records(k); coordinate=alignmentCoordinate(r,alignmentMode);
        trueNormal=alignedTrueNormal(r,alignmentMode);
        plot(normalAxes(component),coordinate,trueNormal(:,component),'--', ...
            'Color',[0.76 0.76 0.76],'LineWidth',3.0,'HandleVisibility','off');
    end
    for plotIndex=1:numel(plotOrder)
        k=plotOrder(plotIndex); r=records(k); c=colors(k,:);
        coordinate=alignmentCoordinate(r,alignmentMode);
        plot(normalAxes(component),coordinate,r.nHat(:,component),'-', ...
            'Color',c,'LineWidth',1.7,'DisplayName',labels{k});
    end
    grid(normalAxes(component),'on'); box(normalAxes(component),'on');
    ylabel(normalAxes(component),sprintf('n_%s [-]',componentNames{component}));
    ylim(normalAxes(component),[-1 1]);
end
title(normalAxes(1),'Normal Vector');
showRecordLegend(normalAxes(1),numel(records));
finishAlignedAxes(normalAxes,xLimits,alignmentMode);
addCurvatureShading(normalAxes,records(1),alignmentMode,smallRadius,largeRadius);

% Top right: estimated normal-rate components.
layout=tiledlayout(panels(3),3,1,'TileSpacing','compact','Padding','compact');
rateAxes=gobjects(3,1);
for component=1:3
    rateAxes(component)=nexttile(layout); hold(rateAxes(component),'on');
    for plotIndex=1:numel(plotOrder)
        k=plotOrder(plotIndex); coordinate=alignmentCoordinate(records(k),alignmentMode);
        plot(rateAxes(component),coordinate,records(k).nDotFiltered(:,component), ...
            'Color',colors(k,:),'LineWidth',1.7,'DisplayName',labels{k});
    end
    grid(rateAxes(component),'on'); box(rateAxes(component),'on');
    ylabel(rateAxes(component),sprintf('dn_%s/dt [1/s]',componentNames{component}));
    ylim(rateAxes(component),[-0.6 0.6]);
end
title(rateAxes(1),'Estimated Normal Rate');
showRecordLegend(rateAxes(1),numel(records));
finishAlignedAxes(rateAxes,xLimits,alignmentMode);
addCurvatureShading(rateAxes,records(1),alignmentMode,smallRadius,largeRadius);

% Bottom right: force, curvature, and separate OFF/ON velocities.
layout=tiledlayout(panels(4),6,1,'TileSpacing','compact','Padding','compact');
metricAxes=gobjects(6,1);
for j=1:6, metricAxes(j)=nexttile(layout); hold(metricAxes(j),'on'); end
% Draw all desired/reference/command curves first (bottom layer).
referenceIndex=find(~cellfun(@isAlphaOff,{records.label}),1);
if isempty(referenceIndex), referenceIndex=1; end
referenceRecord=records(referenceIndex);
referenceCoordinate=alignmentCoordinate(referenceRecord,alignmentMode);
plot(metricAxes(2),referenceCoordinate,alignedTrueCurvature(referenceRecord,alignmentMode), ...
    '--','Color',[0 0 0],'LineWidth',1.9,'HandleVisibility','off');
for k=1:numel(records)
    r=records(k); coordinate=alignmentCoordinate(r,alignmentMode);
    plot(metricAxes(1),coordinate,1e3*r.forceDesired,'--','Color',[0 0 0], ...
        'LineWidth',1.9,'HandleVisibility','off');
    if isAlphaOff(r.label)
        velocityAxis=metricAxes(3);
    else
        velocityAxis=metricAxes(4);
    end
    plot(velocityAxis,coordinate,r.initialT1Cmd,'--','Color',[0.55 0.55 0.55], ...
        'LineWidth',1.8,'HandleVisibility','off');
    if ~isAlphaOff(r.label)
        plot(metricAxes(4),coordinate,r.finalT1Cmd,'-','Color',[0.30 0.30 0.30], ...
            'LineWidth',2.0,'HandleVisibility','off');
    end
end
% Measurements: OFF first, ON last, so ON remains visually on top.
for plotIndex=1:numel(plotOrder)
    k=plotOrder(plotIndex); r=records(k); c=colors(k,:);
    coordinate=alignmentCoordinate(r,alignmentMode);
    plot(metricAxes(1),coordinate,1e3*r.fnHatFiltered,'Color',c, ...
        'LineWidth',1.7,'DisplayName',labels{k});
    if ~isAlphaOff(r.label)
        plot(metricAxes(2),coordinate,r.kappaHatFiltered,'Color',c, ...
            'LineWidth',1.6,'DisplayName',labels{k});
    end
    if isAlphaOff(r.label), velocityAxis=metricAxes(3); else, velocityAxis=metricAxes(4); end
    plot(velocityAxis,coordinate,r.measuredT1VelocityFiltered,'-', ...
        'Color',c,'LineWidth',1.8, ...
        'DisplayName',labels{k});
end
% Error metrics continue directly below the four response panels.
angularPlotOrder=fliplr(plotOrder); % ON first, OFF last/on top.
for plotIndex=1:numel(angularPlotOrder)
    k=angularPlotOrder(plotIndex); r=records(k);
    coordinate=alignmentCoordinate(r,alignmentMode);
    normalAngularError=normalAngleErrorDeg(alignedTrueNormal(r,alignmentMode),r.nHat);
    plot(metricAxes(5),coordinate,normalAngularError,'Color',colors(k,:), ...
        'LineWidth',1.8,'DisplayName',labels{k});
end
for plotIndex=1:numel(plotOrder)
    k=plotOrder(plotIndex); r=records(k);
    coordinate=alignmentCoordinate(r,alignmentMode);
    forceControlError=1e3*abs(r.fnHatFiltered-r.forceDesired);
    plot(metricAxes(6),coordinate,forceControlError,'Color',colors(k,:), ...
        'LineWidth',1.8,'DisplayName',labels{k});
end
ylabel(metricAxes(1),'$\hat{F}_n\ [\mathrm{mN}]$','Interpreter','latex');
title(metricAxes(1),'Normal Force Response');
ylabel(metricAxes(2),'$\kappa\ [\mathrm{m}^{-1}]$','Interpreter','latex');
title(metricAxes(2),'Directional Curvature');
ylabel(metricAxes(3),'$v_{t_1}\ [\mathrm{m/s}]$','Interpreter','latex');
title(metricAxes(3),'Unmodulated Traversal');
ylabel(metricAxes(4),'$v_{t_1}\ [\mathrm{m/s}]$','Interpreter','latex');
title(metricAxes(4),'Geometry-Responsive Traversal');
ylabel(metricAxes(5),'$e_n\ [\mathrm{deg}]$','Interpreter','latex');
title(metricAxes(5),'Normal Estimation Error');
ylabel(metricAxes(6), ...
    '$|f_d-\hat{\mathbf{n}}^{\mathsf{T}}\hat{\mathbf{f}}_c|\ [\mathrm{mN}]$', ...
    'Interpreter','latex');
title(metricAxes(6),'Normal Force Tracking Error');
ylim(metricAxes(1),[-5 40]); ylim(metricAxes(2),[0 5]);
set(metricAxes(3:4),'YLim',[0 0.2]);
ylim(metricAxes(5),[0 45]);
for j=1:6, grid(metricAxes(j),'on'); box(metricAxes(j),'on'); end
showRecordLegend(metricAxes(1),numel(records));
showRecordLegend(metricAxes(3),numel(records));
showRecordLegend(metricAxes(4),numel(records));
finishAlignedAxes(metricAxes,xLimits,alignmentMode);
addCurvatureShading(metricAxes,records(1),alignmentMode,smallRadius,largeRadius);
end

function errorDeg=normalAngleErrorDeg(reference,estimate)
referenceNorm=vecnorm(reference,2,2); estimateNorm=vecnorm(estimate,2,2);
cosine=sum(reference.*estimate,2)./(referenceNorm.*estimateNorm);
valid=all(isfinite(reference),2)&all(isfinite(estimate),2)& ...
    referenceNorm>1e-12&estimateNorm>1e-12;
cosine=min(1,max(-1,cosine)); errorDeg=nan(size(cosine));
errorDeg(valid)=acosd(cosine(valid));
end

function finishAlignedAxes(ax,xLimits,alignmentMode)
if strcmp(alignmentMode,'path'), xText='Path progress [%]'; else, xText='Time [s]'; end
xlabel(ax(end),xText); set(ax(1:end-1),'XTickLabel',[]);
linkaxes(ax,'x'); set(ax,'XLim',xLimits);
end

function r=prepareRecord(T,label,fitWindow,smallRadius,largeRadius,planeLength, ...
    contactEndTime,alphaOffLowDelay,aBarN,small)
r.label=label; r.t=safeColumn(T,"t_sec");
if all(~isfinite(r.t)), r.t=(0:height(T)-1).'; end
first=find(isfinite(r.t),1); if ~isempty(first), r.t=r.t-r.t(first); end
r.eePos=columnsXYZ(T,"fp_normal_eePos"); r.vc=columnsXYZ(T,"fp_normal_vc");
r.t1=columnsXYZ(T,"fp_normal_t1");
r.fcHat=columnsXYZ(T,"fp_normal_fcHat"); r.nHat=columnsXYZ(T,"fp_normal_nHat");
r.nDot=columnsXYZ(T,"fp_normal_nDot"); r.kappaHat=safeColumn(T,"fp_normal_kappaHat");
u1=safeColumn(T,"u1_des"); u2=safeColumn(T,"u2_des");
r.forceDesired=safeColumn(T,"forceDesired"); r.fnHat=safeColumn(T,"fp_normal_fnHat");
denominator=abs(r.kappaHat).*hypot(u1,u2).^2; valid=isfinite(denominator)&denominator>small;
r.alphaGN=nan(height(T),1); r.alphaGN(valid)=sqrt(aBarN./denominator(valid));
r.alphaStar=min(1,r.alphaGN); r.alphaStar(~valid)=1;
r.fnHatFiltered=zeroPhaseLowPass(r.fnHat,r.t,1.0);
r.nDotFiltered=zeroPhaseLowPass(r.nDot,r.t,1.0);
r.kappaHatFiltered=zeroPhaseLowPass(r.kappaHat,r.t,1.0);
r.alphaStarFiltered=zeroPhaseLowPass(r.alphaStar,r.t,1.0);
r.initialT1Cmd=u1; r.finalT1Cmd=r.alphaStar.*u1;
r.geometry=fitTwoCylinderSurface(r.t,r.eePos,fitWindow,smallRadius,largeRadius,planeLength);
r.trueNormal=r.geometry.trueNormal; r.trueCurvature=r.geometry.trueCurvature;
afterContact=isAlphaOff(label)&r.t>contactEndTime;
r.trueNormal(afterContact,:)=nan; r.trueCurvature(afterContact)=nan;
if isAlphaOff(label)&&alphaOffLowDelay>0
    lowMask=isfinite(r.trueCurvature)&abs(r.trueCurvature-1/largeRadius)<0.08;
    firstLowStart=find(lowMask&[true;~lowMask(1:end-1)],1);
    if ~isempty(firstLowStart)
        delayedPart=lowMask&r.t<r.t(firstLowStart)+alphaOffLowDelay;
        r.trueCurvature(delayedPart)=0;
    end
end
r.trueT1=[-r.trueNormal(:,2),r.trueNormal(:,1),zeros(height(T),1)];
opposite=sum(r.trueT1.*r.t1,2)<0; r.trueT1(opposite,:)=-r.trueT1(opposite,:);
r.measuredT1Velocity=sum(r.vc.*r.trueT1,2);
r.measuredT1VelocityFiltered=zeroPhaseLowPass(r.measuredT1Velocity,r.t,1.0);
pathLoopTime=r.geometry.loopTime;
if isAlphaOff(label)
    % Continue the XY-distance coordinate after contact loss so post-loss
    % signals remain visible; ground truth itself is still cut at loss time.
    finiteTime=r.t(isfinite(r.t));
    if ~isempty(finiteTime), pathLoopTime(2)=max(finiteTime); end
end
[r.pathPercent,r.pathLength,r.lapBoundaryPercent]= ...
    normalizedLoopPath(r.t,r.eePos,pathLoopTime);
end

function records=trimAlphaOffAfterContactLoss(records,contactLossPath)
% Preserve coordinates, the EE trajectory, normal-force signals, and the
% commanded OFF velocity, but hide measured/estimated signals after loss.
fieldsToTrim={'vc','t1','fcHat','nHat','nDot','kappaHat', ...
    'alphaGN','alphaStar','nDotFiltered','kappaHatFiltered','alphaStarFiltered', ...
    'finalT1Cmd','trueNormal','trueCurvature','trueT1', ...
    'measuredT1Velocity','measuredT1VelocityFiltered', ...
    'alignedTrueNormal','alignedTrueCurvature'};
for recordIndex=1:numel(records)
    if ~isAlphaOff(records(recordIndex).label), continue; end
    discard=isfinite(records(recordIndex).pathPercent)& ...
        records(recordIndex).pathPercent>contactLossPath;
    for fieldIndex=1:numel(fieldsToTrim)
        fieldName=fieldsToTrim{fieldIndex};
        if ~isfield(records(recordIndex),fieldName), continue; end
        value=records(recordIndex).(fieldName);
        if size(value,1)==numel(discard)
            value(discard,:)=nan;
            records(recordIndex).(fieldName)=value;
        end
    end
end
end

function [progress,totalLength,lapBoundary]=normalizedLoopPath(t,eePos,loopTime)
progress=nan(size(t)); valid=isfinite(t)&all(isfinite(eePos(:,1:2)),2)& ...
    t>=loopTime(1)&t<=loopTime(2); idx=find(valid); totalLength=0; lapBoundary=50;
if numel(idx)<2, return; end
s=[0;cumsum(vecnorm(diff(eePos(idx,1:2),1,1),2,2))]; totalLength=s(end);
if totalLength>eps
    progress(idx)=100*s/totalLength;
    % The first return to the starting XY point separates lap 1 and lap 2.
    candidates=find(s>=0.30*totalLength&s<=0.70*totalLength);
    if ~isempty(candidates)
        returnDistance=vecnorm(eePos(idx(candidates),1:2)-eePos(idx(1),1:2),2,2);
        [~,nearest]=min(returnDistance);
        lapBoundary=progress(idx(candidates(nearest)));
    end
end
end

function records=alignPathByReferenceDistance(records,alphaOffPathShift)
% A fitted surface lap is geometric and independent of the selected log.
% Two complete ON laps define 100%; an early OFF contact loss stays below 100%.
onIndex=find(~cellfun(@isAlphaOff,{records.label}),1);
if isempty(onIndex), geometryIndex=1; else, geometryIndex=onIndex; end
centerline=records(geometryIndex).geometry.centerline;
oneLapLength=sum(vecnorm(diff([centerline;centerline(1,:)],1,1),2,2));
referenceLength=2*oneLapLength;
if ~isfinite(referenceLength)||referenceLength<=eps, return; end
for recordIndex=1:numel(records)
    scale=records(recordIndex).pathLength/referenceLength;
    records(recordIndex).pathPercent=records(recordIndex).pathPercent*scale;
    records(recordIndex).lapBoundaryPercent=records(recordIndex).lapBoundaryPercent*scale;
end
if isempty(onIndex), referenceIndex=1; else, referenceIndex=onIndex; end
[referenceProgress,uniqueIndex]=unique(records(referenceIndex).pathPercent,'stable');
validReference=isfinite(referenceProgress)& ...
    all(isfinite(records(referenceIndex).trueNormal(uniqueIndex,:)),2)& ...
    isfinite(records(referenceIndex).trueCurvature(uniqueIndex));
referenceProgress=referenceProgress(validReference);
referenceNormal=records(referenceIndex).trueNormal(uniqueIndex(validReference),:);
referenceCurvature=records(referenceIndex).trueCurvature(uniqueIndex(validReference));
if ~isempty(onIndex)
    for recordIndex=1:numel(records)
        if ~isAlphaOff(records(recordIndex).label), continue; end
        [pathScale,pathShift]=bestTrueNormalAffine(referenceProgress,referenceNormal, ...
            records(recordIndex),alphaOffPathShift);
        records(recordIndex).pathPercent= ...
            pathScale*records(recordIndex).pathPercent+pathShift;
        records(recordIndex).lapBoundaryPercent= ...
            pathScale*records(recordIndex).lapBoundaryPercent+pathShift;
        fprintf('  %s | true-normal path alignment: scale %.5f, shift %+.3f%%\n', ...
            records(recordIndex).label,pathScale,pathShift);
    end
elseif isAlphaOff(records(1).label)
    % With OFF only, retain the user shift while keeping the two-lap scale.
    records(1).pathPercent=records(1).pathPercent+alphaOffPathShift;
    records(1).lapBoundaryPercent=records(1).lapBoundaryPercent+alphaOffPathShift;
end
for recordIndex=1:numel(records)
    query=records(recordIndex).pathPercent;
    records(recordIndex).alignedTrueNormal=nan(size(records(recordIndex).trueNormal));
    records(recordIndex).alignedTrueCurvature=nan(size(records(recordIndex).trueCurvature));
    inside=isfinite(query)&query>=referenceProgress(1)&query<=referenceProgress(end);
    records(recordIndex).alignedTrueNormal(inside,:)=interp1( ...
        referenceProgress,referenceNormal,query(inside),'linear');
    records(recordIndex).alignedTrueCurvature(inside)=interp1( ...
        referenceProgress,referenceCurvature,query(inside),'previous');
end
end

function [bestScale,bestShift]=bestTrueNormalAffine(referenceProgress,referenceNormal,target,shiftGuess)
valid=find(isfinite(target.pathPercent)&all(isfinite(target.trueNormal),2));
if numel(valid)<20, bestScale=1; bestShift=shiftGuess; return; end
valid=valid(round(linspace(1,numel(valid),min(700,numel(valid)))));
targetProgress=target.pathPercent(valid); targetNormal=target.trueNormal(valid,:);
targetNormal=targetNormal./max(vecnorm(targetNormal,2,2),eps);
referenceNormal=referenceNormal./max(vecnorm(referenceNormal,2,2),eps);
bestCost=inf; bestScale=1; bestShift=shiftGuess;
for scale=0.97:0.002:1.03
    for shift=shiftGuess-5:0.05:shiftGuess+5
        query=scale*targetProgress+shift;
        inside=query>=referenceProgress(1)&query<=referenceProgress(end);
        if sum(inside)<20, continue; end
        expected=interp1(referenceProgress,referenceNormal,query(inside),'linear');
        cosine=sum(expected.*targetNormal(inside,:),2)./max(vecnorm(expected,2,2),eps);
        cost=mean(1-min(1,max(-1,cosine)));
        if cost<bestCost, bestCost=cost; bestScale=scale; bestShift=shift; end
    end
end
end

function records=alignPathByGroundTruth(records)
% Align lap 1 and lap 2 independently using each run's detected return point.
reference=records(1);
referenceBounds=[0 reference.lapBoundaryPercent;reference.lapBoundaryPercent 100];
for recordIndex=2:numel(records)
    target=records(recordIndex);
    targetBounds=[0 target.lapBoundaryPercent;target.lapBoundaryPercent 100];
    for lap=1:2
        referenceLower=referenceBounds(lap,1); referenceUpper=referenceBounds(lap,2);
        targetLower=targetBounds(lap,1); targetUpper=targetBounds(lap,2);
        referenceValid=find(reference.pathPercent>=referenceLower&reference.pathPercent<=referenceUpper& ...
            all(isfinite(reference.trueNormal),2)&isfinite(reference.trueCurvature));
        targetValid=find(target.pathPercent>=targetLower&target.pathPercent<=targetUpper& ...
            all(isfinite(target.trueNormal),2)&isfinite(target.trueCurvature));
        if numel(referenceValid)<20||numel(targetValid)<20, continue; end
        referenceKnots=referenceValid(round(linspace(1,numel(referenceValid),min(240,numel(referenceValid)))));
        targetKnots=targetValid(round(linspace(1,numel(targetValid),min(240,numel(targetValid)))));
        referenceNormal=reference.trueNormal(referenceKnots,:);
        targetNormal=target.trueNormal(targetKnots,:);
        referenceNormal=referenceNormal./max(vecnorm(referenceNormal,2,2),eps);
        targetNormal=targetNormal./max(vecnorm(targetNormal,2,2),eps);
        referenceCurvature=reference.trueCurvature(referenceKnots);
        targetCurvature=target.trueCurvature(targetKnots);
        referenceProgress=reference.pathPercent(referenceKnots);
        targetProgress=target.pathPercent(targetKnots);
        normalCost=1-max(-1,min(1,referenceNormal*targetNormal.'));
        curvatureScale=max([referenceCurvature;targetCurvature;1]);
        curvatureCost=abs(referenceCurvature-targetCurvature.')/curvatureScale;
        referenceLocal=(referenceProgress-referenceLower)/(referenceUpper-referenceLower);
        targetLocal=(targetProgress-targetLower)/(targetUpper-targetLower);
        progressCost=abs(referenceLocal-targetLocal.');
        [referencePath,targetPath]=dynamicTimeWarpPath( ...
            normalCost+0.45*curvatureCost+0.35*progressCost);
        mappedProgress=nan(numel(targetKnots),1);
        for knot=1:numel(targetKnots)
            matches=referencePath(targetPath==knot);
            if ~isempty(matches), mappedProgress(knot)=mean(referenceProgress(matches)); end
        end
        validMap=isfinite(mappedProgress);
        [rawProgress,uniqueIndex]=unique(targetProgress(validMap),'stable');
        mappedProgress=mappedProgress(validMap); mappedProgress=mappedProgress(uniqueIndex);
        if numel(rawProgress)>=2
            inside=isfinite(target.pathPercent)&target.pathPercent>=targetLower& ...
                target.pathPercent<=targetUpper;
            aligned=interp1(rawProgress,mappedProgress,target.pathPercent(inside),'linear','extrap');
            records(recordIndex).pathPercent(inside)= ...
                min(referenceUpper,max(referenceLower,aligned));
        end
    end
end
end

function [rowPath,columnPath]=dynamicTimeWarpPath(cost)
[rowCount,columnCount]=size(cost); accumulated=inf(rowCount,columnCount);
predecessor=zeros(rowCount,columnCount,'uint8'); accumulated(1,1)=cost(1,1);
for row=1:rowCount
    for column=1:columnCount
        if row==1&&column==1, continue; end
        candidates=[inf inf inf]; % diagonal, vertical, horizontal
        if row>1&&column>1, candidates(1)=accumulated(row-1,column-1); end
        if row>1, candidates(2)=accumulated(row-1,column); end
        if column>1, candidates(3)=accumulated(row,column-1); end
        [best,step]=min(candidates);
        accumulated(row,column)=cost(row,column)+best; predecessor(row,column)=step;
    end
end
row=rowCount; column=columnCount; rowPath=row; columnPath=column;
while row>1||column>1
    step=predecessor(row,column);
    if step==1, row=row-1; column=column-1;
    elseif step==2, row=row-1;
    else, column=column-1;
    end
    rowPath(end+1,1)=row; columnPath(end+1,1)=column; %#ok<AGROW>
end
rowPath=flipud(rowPath); columnPath=flipud(columnPath);
end

function idx=pathSpacedIndices(path,valid,period,offset)
% Select samples nearest to an absolute, evenly spaced coordinate grid.
% Alternating offsets keep coincident ON/OFF normal arrows distinguishable.
candidates=find(valid&isfinite(path)); idx=zeros(0,1);
if isempty(candidates)||~isfinite(period)||period<=0, return; end
candidatePath=path(candidates);
firstTarget=ceil((min(candidatePath)-offset)/period)*period+offset;
lastTarget=floor((max(candidatePath)-offset)/period)*period+offset;
targets=(firstTarget:period:lastTarget).';
for k=1:numel(targets)
    [~,nearest]=min(abs(candidatePath-targets(k)));
    idx(end+1,1)=candidates(nearest); %#ok<AGROW>
end
idx=unique(idx,'stable');
end

function [highMask,lowMask]=curvatureMasks(curvature,smallRadius,largeRadius)
highMask=isfinite(curvature)&abs(curvature-1/smallRadius)<0.08;
lowMask=isfinite(curvature)&abs(curvature-1/largeRadius)<0.08;
end

function plotMaskedTrajectory(ax,x,y,mask,color)
xMasked=x; yMasked=y; xMasked(~mask)=nan; yMasked(~mask)=nan;
plot(ax,xMasked,yMasked,'Color',color,'LineWidth',4.0,'HandleVisibility','off');
end

function addCurvatureShading(axesHandles,record,alignmentMode,smallRadius,largeRadius)
coordinate=alignmentCoordinate(record,alignmentMode);
[highMask,lowMask]=curvatureMasks(record.trueCurvature,smallRadius,largeRadius);
[highColor,lowColor]=curvatureColors();
masks={highMask,lowMask}; regionColors={highColor,lowColor};
for axisNumber=1:numel(axesHandles)
    ax=axesHandles(axisNumber); yLimits=ylim(ax); holdWasOn=ishold(ax); hold(ax,'on');
    for curvatureType=1:2
        intervals=maskToIntervals(coordinate,masks{curvatureType});
        for intervalNumber=1:size(intervals,1)
            interval=intervals(intervalNumber,:);
            region=patch(ax,[interval(1) interval(2) interval(2) interval(1)], ...
                [yLimits(1) yLimits(1) yLimits(2) yLimits(2)],regionColors{curvatureType}, ...
                'FaceAlpha',0.12,'EdgeColor','none','HandleVisibility','off');
            uistack(region,'bottom');
        end
    end
    if ~holdWasOn, hold(ax,'off'); end
end
end

function [highColor,lowColor]=curvatureColors()
% Kept separate from the blue/red-orange ON/OFF record palette.
highColor=[0.10 0.62 0.42]; % green: tight/high-curvature section
lowColor=[0.55 0.42 0.78];  % purple: gentle/low-curvature section
end

function intervals=maskToIntervals(coordinate,mask)
valid=mask(:)&isfinite(coordinate(:));
starts=find(valid&[true;~valid(1:end-1)]);
stops=find(valid&[~valid(2:end);true]);
intervals=[coordinate(starts) coordinate(stops)];
end

function coordinate=alignmentCoordinate(record,alignmentMode)
if strcmp(alignmentMode,'path'), coordinate=record.pathPercent; else, coordinate=record.t; end
end

function positions=overviewWindowPositions(origin,windowSize,gap)
% Preserve the former quadrant arrangement, but put each panel in its own figure.
x=origin(1); y=origin(2); width=windowSize(1); height=windowSize(2);
sixRowHeight=2*height;
leftX=x;
middleX=leftX+width+gap(1);
rightX=middleX+width+gap(1);
positions=[leftX,  y+height+gap(2),width,height; ...
           leftX,  y,              width,height; ...
           middleX,y+sixRowHeight+gap(2),width,height; ...
           middleX,y,                    width,sixRowHeight; ...
           rightX, y+height+gap(2),width,height];
end

function center=trajectoryCenter(records)
onIndex=find(~cellfun(@isAlphaOff,{records.label}),1);
if isempty(onIndex), onIndex=1; end
centerline=records(onIndex).geometry.centerline;
center=(min(centerline,[],1)+max(centerline,[],1))/2;
end

function [xLimits,yLimits]=symmetricTrajectoryLimits(x,y)
x=x(isfinite(x)); y=y(isfinite(y));
if isempty(x), xHalf=1; else, xHalf=max(abs(x)); end
if isempty(y), yHalf=1; else, yHalf=max(abs(y)); end
xHalf=max(0.05,1.06*xHalf); yHalf=max(0.05,1.06*yHalf);
xLimits=[-xHalf xHalf]; yLimits=[-yHalf yHalf];
end

function [meanX,meanY]=meanFirstTrialTrajectory(records,center)
% Normalize each first lap independently before averaging.  This avoids a
% path-alignment shift pulling the mean away from the physical closed loop.
query=linspace(0,1,500).';
xSamples=nan(numel(query),numel(records)); ySamples=xSamples;
for k=1:numel(records)
    coordinate=records(k).pathPercent;
    valid=isfinite(coordinate)&coordinate<=records(k).lapBoundaryPercent& ...
        all(isfinite(records(k).eePos(:,1:2)),2);
    if sum(valid)<2, continue; end
    firstProgress=min(coordinate(valid));
    localProgress=(coordinate-firstProgress)/ ...
        max(records(k).lapBoundaryPercent-firstProgress,eps);
    [coordinateUnique,uniqueIndex]=unique(localProgress(valid),'stable');
    points=records(k).eePos(valid,1:2); points=points(uniqueIndex,:);
    if numel(coordinateUnique)<2, continue; end
    inside=query>=coordinateUnique(1)&query<=coordinateUnique(end);
    xSamples(inside,k)=interp1(coordinateUnique,points(:,2)-center(2),query(inside),'linear');
    ySamples(inside,k)=interp1(coordinateUnique,points(:,1)-center(1),query(inside),'linear');
end
meanX=mean(xSamples,2,'omitnan'); meanY=mean(ySamples,2,'omitnan');
meanX(sum(isfinite(xSamples),2)<2)=nan;
meanY(sum(isfinite(ySamples),2)<2)=nan;
finite=find(isfinite(meanX)&isfinite(meanY));
if ~isempty(finite)
    meanX(end+1,1)=meanX(finite(1));
    meanY(end+1,1)=meanY(finite(1));
end
end

function limits=secondHighCurvatureCropLimits(records,alignmentMode,xLimits,center, ...
    smallRadius,largeRadius,arrowLength,targetAspect)
limits=[];
onIndex=find(~cellfun(@isAlphaOff,{records.label}),1);
if isempty(onIndex), onIndex=1; end
r=records(onIndex); coordinate=alignmentCoordinate(r,alignmentMode);
[highMask,~]=curvatureMasks(r.trueCurvature,smallRadius,largeRadius);
highMask=highMask&isfinite(coordinate)&coordinate>=xLimits(1)&coordinate<=xLimits(2);
intervals=maskToIntervals(coordinate,highMask);
if size(intervals,1)<2, return; end
section=intervals(2,:);
inside=coordinate>=min(section)&coordinate<=max(section)& ...
    all(isfinite(r.eePos(:,1:2)),2);
x=r.eePos(inside,2)-center(2); y=r.eePos(inside,1)-center(1);
if isempty(x), return; end
xMargin=max(1.5*arrowLength,0.25*(max(x)-min(x)));
yMargin=max(1.5*arrowLength,0.25*(max(y)-min(y)));
limits=[min(x)-xMargin,max(x)+xMargin,min(y)-yMargin,max(y)+yMargin];
% Expand only (never crop) to approximately match the figure aspect ratio.
xCenter=mean(limits(1:2)); yCenter=mean(limits(3:4));
width=diff(limits(1:2)); height=diff(limits(3:4));
if width/height<targetAspect
    width=targetAspect*height;
else
    height=width/targetAspect;
end
limits=[xCenter-width/2,xCenter+width/2,yCenter-height/2,yCenter+height/2];
end

function order=recordPlotOrder(records)
% MATLAB paints later graphics on top: OFF first, ON/other records last.
isOff=cellfun(@isAlphaOff,{records.label});
order=[find(isOff),find(~isOff)];
end

function colors=recordColors(records)
% MATLAB default colors: modulation ON red-orange, OFF blue.
matlabBlue=[0.0000 0.4470 0.7410];
matlabRed =[0.8500 0.3250 0.0980];
fallback=[0.4660 0.6740 0.1880; 0.4940 0.1840 0.5560; 0.3010 0.7450 0.9330];
colors=zeros(numel(records),3); fallbackIndex=1;
for k=1:numel(records)
    if isAlphaOff(records(k).label)
        colors(k,:)=matlabBlue;
    elseif ~isempty(regexpi(records(k).label,'alpha1t','once'))
        colors(k,:)=matlabRed;
    else
        colors(k,:)=fallback(mod(fallbackIndex-1,size(fallback,1))+1,:);
        fallbackIndex=fallbackIndex+1;
    end
end
end

function value=alignedTrueNormal(record,alignmentMode)
% Plot each log's independently fitted ground truth for alignment checks.
value=record.trueNormal;
end

function value=alignedTrueCurvature(record,alignmentMode)
% Plot each log's independently fitted ground truth for alignment checks.
value=record.trueCurvature;
end

function color=lightenColor(color,amount)
color=(1-amount)*color+amount*[1 1 1];
end

function tf=isAlphaOff(fileName)
tf=~isempty(regexpi(fileName,'alphaX','once'));
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

function label=experimentLabel(fileName)
[~,rawLabel]=fileparts(fileName);
if ~isempty(regexpi(rawLabel,'alpha1t','once'))
    label='$\alpha^*$ ON';
elseif ~isempty(regexpi(rawLabel,'alphaX','once'))
    label='$\alpha^*$ OFF';
else
    label=regexprep(rawLabel,'_flyingpen_normal_.*$','');
    label=strrep(label,'_',' ');
end
end

function showRecordLegend(ax,recordCount)
if recordCount>1
    legend(ax,'Location','best','Interpreter','latex');
else
    legend(ax,'off');
end
end
function x=safeColumn(T,name)
vars=string(T.Properties.VariableNames); idx=find(strcmpi(vars,string(name)),1);
if isempty(idx), x=nan(height(T),1); else, x=double(T{:,idx}); end
x=x(:);
end
function xyz=columnsXYZ(T,prefix)
xyz=[safeColumn(T,string(prefix)+"X"),safeColumn(T,string(prefix)+"Y"),safeColumn(T,string(prefix)+"Z")];
end

function fit=fitTwoCylinderSurface(t,eePos,timeWindow,smallRadius,largeRadius,planeLengthGuess)
sampleValid=isfinite(t)&all(isfinite(eePos),2); fitValid=sampleValid;
if ~isempty(timeWindow), fitValid=fitValid&t>=timeWindow(1)&t<=timeWindow(2); end
windowIdx=find(fitValid); if numel(windowIdx)<20, error('Not enough finite EE samples.'); end
rawWindow=eePos(windowIdx,1:2); [loopStart,loopEnd]=longestClosedLoop(rawWindow,0.06,2.0);
fitIdx=windowIdx(loopStart:loopEnd); raw=eePos(fitIdx,1:2);
s=[0;cumsum(vecnorm(diff(raw,1,1),2,2))]; [sUnique,uniqueIdx]=unique(s,'stable'); raw=raw(uniqueIdx,:);
sQuery=linspace(0,sUnique(end),240)';
points=[interp1(sUnique,raw(:,1),sQuery),interp1(sUnique,raw(:,2),sQuery)];
xMid=median(points(:,1)); yMin=min(points(:,2)); yMax=max(points(:,2));
q0a=[xMid,yMin+smallRadius,xMid,yMax-largeRadius];
q0b=[xMid,yMax-smallRadius,xMid,yMin+largeRadius];
options=optimset('Display','off','MaxFunEvals',6000,'MaxIter',2500,'TolX',1e-9,'TolFun',1e-10);
cost=@(q)twoCylinderFitCost(q,points,smallRadius,largeRadius,planeLengthGuess);
qa=fminsearch(cost,q0a,options); qb=fminsearch(cost,q0b,options);
if cost(qb)<cost(qa), q=qb; else, q=qa; end
[centerline,denseCurvature]=twoCylinderBoundary(q,smallRadius,largeRadius,720);
idx=find(sampleValid); trueNormal=nan(size(eePos)); trueCurvature=nan(size(t));
previous=[centerline(end-1,:);centerline;centerline(2,:)];
tangent=previous(3:end,:)-previous(1:end-2,:);
outward=[tangent(:,2),-tangent(:,1)]; outward=outward./max(vecnorm(outward,2,2),eps);
distances=nan(numel(idx),1);
for k=1:numel(idx)
    [distances(k),nearest]=min(vecnorm(centerline-eePos(idx(k),1:2),2,2));
    if distances(k)<=0.06
        trueNormal(idx(k),:)=[outward(nearest,:),0]; trueCurvature(idx(k))=denseCurvature(nearest);
    end
end
fit.rms=sqrt(mean(distances(ismember(idx,fitIdx)).^2,'omitnan'));
fit.loopTime=[t(fitIdx(1)) t(fitIdx(end))]; fit.trueNormal=trueNormal;
fit.trueCurvature=trueCurvature; fit.centerline=centerline;
end

function [bestStart,bestEnd]=longestClosedLoop(points,closureTolerance,minLoopLength)
s=[0;cumsum(vecnorm(diff(points,1,1),2,2))]; bestStart=1; bestEnd=size(points,1); bestLength=-inf;
stride=max(1,floor(size(points,1)/1200));
for startIndex=1:stride:size(points,1)
    eligible=find(s-s(startIndex)>=minLoopLength&vecnorm(points-points(startIndex,:),2,2)<=closureTolerance);
    if isempty(eligible), continue; end
    [candidateLength,which]=max(s(eligible)-s(startIndex));
    if candidateLength>bestLength
        bestLength=candidateLength; bestStart=startIndex; bestEnd=eligible(which);
    end
end
if bestLength<minLoopLength, bestStart=1; bestEnd=size(points,1); end
end

function cost=twoCylinderFitCost(q,points,smallRadius,largeRadius,planeLengthGuess)
if any(~isfinite(q))||norm(q(3:4)-q(1:2))<=largeRadius-smallRadius
    cost=1e3+sum(q.^2); return;
end
[boundary,~]=twoCylinderBoundary(q,smallRadius,largeRadius,360);
dPoint=min(pairwiseDistance(points,boundary),[],2); dBoundary=min(pairwiseDistance(boundary,points),[],2);
centerDistance=norm(q(3:4)-q(1:2)); tangentLength=sqrt(max(0,centerDistance^2-(largeRadius-smallRadius)^2));
cost=mean(min(dPoint,0.08).^2)+0.5*mean(min(dBoundary,0.08).^2)+ ...
    0.002*(tangentLength-planeLengthGuess)^2;
end

function [boundary,curvature]=twoCylinderBoundary(q,smallRadius,largeRadius,count)
angles=linspace(0,2*pi,ceil(count/2)+1)'; angles(end)=[];
cloud=[q(1:2)+smallRadius*[cos(angles),sin(angles)]; ...
       q(3:4)+largeRadius*[cos(angles),sin(angles)]];
hull=convhull(cloud(:,1),cloud(:,2),'Simplify',true); hull(end)=[]; polygon=cloud(hull,:);
s=[0;cumsum(vecnorm(diff([polygon;polygon(1,:)],1,1),2,2))];
sQuery=linspace(0,s(end),count+1)'; sQuery(end)=[]; closed=[polygon;polygon(1,:)];
boundary=[interp1(s,closed(:,1),sQuery),interp1(s,closed(:,2),sQuery)];
smallResidual=abs(vecnorm(boundary-q(1:2),2,2)-smallRadius);
largeResidual=abs(vecnorm(boundary-q(3:4),2,2)-largeRadius);
curvature=zeros(count,1); curvature(smallResidual<2e-3)=1/smallRadius;
curvature(largeResidual<2e-3)=1/largeRadius;
end

function distance=pairwiseDistance(a,b)
delta=reshape(a,[],1,2)-reshape(b,1,[],2); distance=sqrt(sum(delta.^2,3));
end

function folder=activeScriptDirectory()
folder='';
try
    activeFile=matlab.desktop.editor.getActiveFilename;
    if ~isempty(activeFile), folder=fileparts(activeFile); end
catch
end
if isempty(folder)
    thisFile=mfilename('fullpath'); if ~isempty(thisFile), folder=fileparts(thisFile); end
end
if isempty(folder), folder=pwd; end
end
