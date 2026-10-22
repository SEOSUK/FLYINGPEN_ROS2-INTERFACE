%% Second-experiment trajectory-fitted normal / curvature analysis
% Read-only analysis. The selected recording is fitted independently.
clear; close all; clc;

%% User settings
plot_xlim=[0 98]; geometry_fit_xlim=[0 98];
geometry_large_radius=0.420; geometry_small_radius=0.280;
geometry_plane_length_guess=1.0; normal_arrow_period=2.0;
normal_arrow_length=0.10; surface_height_half_span=0.20;
a_bar_n=0.005; small=1.0e-8;

%% Portable single-file loading
dataDir=activeScriptDirectory();
fprintf('CSV selection folder: %s\n',dataDir);
[selectedFile,selectedPath]=uigetfile( ...
    {'alpha*.csv','Normal-estimator CSV files (*.csv)';'*.csv','All CSV files'}, ...
    'Select a second-experiment CSV file',dataDir);
if isequal(selectedFile,0), return; end
selectedFiles=cellstr(selectedFile);

records=struct([]);
for fileIndex=1:numel(selectedFiles)
    csvFile=fullfile(selectedPath,selectedFiles{fileIndex});
    fprintf('Loading %s\n',csvFile);
    T=readtable(csvFile,detectImportOptions(csvFile,'Delimiter',','));
    r=prepareRecord(T,selectedFiles{fileIndex},geometry_fit_xlim, ...
        geometry_small_radius,geometry_large_radius, ...
        geometry_plane_length_guess,a_bar_n,small);
    if fileIndex==1, records=r; else, records(fileIndex)=r; end
    g=r.geometry;
    fprintf(['  %s | Rsmall=%.3f m, Rlarge=%.3f m (fixed), centers=%.4f m, ' ...
        'tangent=%.4f m, axis=%.2f deg, fit RMS=%.4f m, loop=%.2f-%.2f s\n'],r.label, ...
        g.smallRadius,g.largeRadius,g.centerDistance,g.tangentLength, ...
        g.axisAngleDeg,g.rms,g.loopTime);
    fprintf('    centers: small=[%.4f %.4f], large=[%.4f %.4f] m\n', ...
        g.smallCenter,g.largeCenter);
end
nRecords=numel(records); colors=lines(nRecords);
labels=cellfun(@experimentLabel,{records.label},'UniformOutput',false);

%% Overview
figure('Name','2nd experiment: fitted-geometry overview','Color','w','Position',[80 50 1500 880]);
overview=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
axOverview=gobjects(4,1);
axOverview(1)=nexttile(overview); hold(axOverview(1),'on');
axOverview(2)=nexttile(overview); hold(axOverview(2),'on');
axOverview(3)=nexttile(overview); hold(axOverview(3),'on');
axOverview(4)=nexttile(overview); hold(axOverview(4),'on');
for k=1:nRecords
    r=records(k); c=colors(k,:);
    plot(axOverview(1),r.t,normalAngleErrorDeg(r.trueNormal,r.nHat), ...
        'Color',c,'LineWidth',1.8,'DisplayName',labels{k});
    plot(axOverview(2),r.t,r.nDotNorm,'Color',c,'LineWidth',1.8,'DisplayName',labels{k});
    plot(axOverview(3),r.t,r.trueCurvature,'--','Color',c,'LineWidth',1.4,'HandleVisibility','off');
    plot(axOverview(3),r.t,r.kappaHat,'Color',c,'LineWidth',1.9,'DisplayName',labels{k});
    plot(axOverview(4),r.t,r.alphaStar,'Color',c,'LineWidth',1.9,'DisplayName',labels{k});
end
decorateAxis(axOverview(1),'error [deg]','Normal estimation error');
decorateAxis(axOverview(2),'|dn/dt| [1/s]','Normal-rate magnitude');
decorateAxis(axOverview(3),'curvature [1/m]','Curvature: dashed true, solid estimated');
decorateAxis(axOverview(4),'\alpha^*','Velocity modulation');
xlabel(overview,'t [s]'); linkaxes(axOverview,'x'); applyTimeXLim(axOverview,plot_xlim);
addCurvatureShading(axOverview,records(1).t,records(1).trueCurvature, ...
    geometry_small_radius,geometry_large_radius);

%% Detailed diagnostics
figure('Name','2nd experiment: fitted-geometry details','Color','w','Position',[110 40 1600 950]);
detail=tiledlayout(4,2,'TileSpacing','compact','Padding','compact');
timeAxes=gobjects(7,1);
for k=1:4, timeAxes(k)=nexttile(detail); hold(timeAxes(k),'on'); end
for k=1:nRecords
    r=records(k); c=colors(k,:);
    plot(timeAxes(1),r.t,normalAngleErrorDeg(r.trueNormal,r.nHat),'Color',c,'LineWidth',1.7,'DisplayName',labels{k});
    plot(timeAxes(2),r.t,normalAngleErrorDeg(r.trueNormal,r.forceDirection),'Color',c,'LineWidth',1.7,'DisplayName',labels{k});
    plot(timeAxes(3),r.t,r.nDotNorm,'Color',c,'LineWidth',1.7,'DisplayName',labels{k});
    plot(timeAxes(4),r.t,vecnorm(r.fcHat,2,2),'Color',c,'LineWidth',1.7,'DisplayName',labels{k});
end
decorateAxis(timeAxes(1),'normal error [deg]','True vs. estimated normal');
decorateAxis(timeAxes(2),'direction error [deg]','True normal vs. force direction');
decorateAxis(timeAxes(3),'|dn/dt| [1/s]','Normal-rate magnitude');
decorateAxis(timeAxes(4),'|f_c| [N]','Contact-force magnitude');

axXY=nexttile(detail); hold(axXY,'on');
for k=1:nRecords
    r=records(k); c=colors(k,:);
    plot(axXY,r.eePos(:,1),r.eePos(:,2),'Color',c,'LineWidth',1.8,'DisplayName',labels{k});
    valid=all(isfinite(r.eePos(:,1:2)),2)&all(isfinite(r.nHat(:,1:2)),2);
    idx=timeSpacedIndices(r.t,valid,normal_arrow_period);
    quiver(axXY,r.eePos(idx,1),r.eePos(idx,2),normal_arrow_length*r.nHat(idx,1), ...
        normal_arrow_length*r.nHat(idx,2),0,'Color',c,'LineWidth',1.2, ...
        'MaxHeadSize',0.7,'HandleVisibility','off');
end
axis(axXY,'equal'); grid(axXY,'on'); xlabel(axXY,'EE x [m]'); ylabel(axXY,'EE y [m]');
title(axXY,'XY trajectory and estimated normals'); legend(axXY,'Location','best','Interpreter','none');

for k=5:7, timeAxes(k)=nexttile(detail); hold(timeAxes(k),'on'); end
for k=1:nRecords
    r=records(k); c=colors(k,:);
    plot(timeAxes(5),r.t,r.vcNorm,'Color',c,'LineWidth',1.7,'DisplayName',labels{k});
    plot(timeAxes(6),r.t,r.alphaStar,'Color',c,'LineWidth',1.8,'DisplayName',labels{k});
    plot(timeAxes(7),r.t,r.trueCurvature,'--','Color',c,'LineWidth',1.3,'HandleVisibility','off');
    plot(timeAxes(7),r.t,r.kappaHat,'Color',c,'LineWidth',1.8,'DisplayName',labels{k});
end
decorateAxis(timeAxes(5),'|v_c| [m/s]','Contact-speed magnitude');
decorateAxis(timeAxes(6),'\alpha^*','Velocity modulation');
decorateAxis(timeAxes(7),'curvature [1/m]','True (dashed) vs. estimated curvature');
xlabel(detail,'t [s]'); linkaxes(timeAxes,'x'); applyTimeXLim(timeAxes,plot_xlim);
addCurvatureShading(timeAxes,records(1).t,records(1).trueCurvature, ...
    geometry_small_radius,geometry_large_radius);

%% Independently fitted surfaces and true-normal fields
figure('Name','2nd experiment: fitted surfaces','Color','w','Position',[150 80 1200 820]);
axSurface=axes; hold(axSurface,'on');
for k=1:nRecords
    r=records(k); g=r.geometry; c=colors(k,:); centerline=g.centerline;
    zLow=g.origin(3)-surface_height_half_span; zHigh=g.origin(3)+surface_height_half_span;
    surf(axSurface,[centerline(:,1),centerline(:,1)],[centerline(:,2),centerline(:,2)], ...
        [zLow*ones(size(centerline,1),1),zHigh*ones(size(centerline,1),1)], ...
        'FaceColor',c,'FaceAlpha',0.12,'EdgeColor','none','HandleVisibility','off');
    plot3(axSurface,r.eePos(:,1),r.eePos(:,2),r.eePos(:,3),'Color',c, ...
        'LineWidth',1.8,'DisplayName',labels{k});
    valid=all(isfinite(r.eePos),2)&all(isfinite(r.trueNormal),2);
    idx=timeSpacedIndices(r.t,valid,normal_arrow_period);
    quiver3(axSurface,r.eePos(idx,1),r.eePos(idx,2),r.eePos(idx,3), ...
        normal_arrow_length*r.trueNormal(idx,1),normal_arrow_length*r.trueNormal(idx,2), ...
        normal_arrow_length*r.trueNormal(idx,3),0,'Color',c,'LineWidth',1.3, ...
        'MaxHeadSize',0.7,'HandleVisibility','off');
end
axis(axSurface,'equal'); grid(axSurface,'on'); view(axSurface,3);
xlabel(axSurface,'x [m]'); ylabel(axSurface,'y [m]'); zlabel(axSurface,'z [m]');
title(axSurface,'Independent surface fits, EE trajectories, and true normals');
legend(axSurface,'Location','best','Interpreter','none');

%% Helpers
function r=prepareRecord(T,label,fitWindow,smallRadius,largeRadius,planeLength,aBarN,small)
r.label=label; r.t=safeColumn(T,"t_sec");
if all(~isfinite(r.t)), r.t=(0:height(T)-1).'; end
r.eePos=columnsXYZ(T,"fp_normal_eePos"); r.vc=columnsXYZ(T,"fp_normal_vc");
r.fcHat=columnsXYZ(T,"fp_normal_fcHat"); r.nHat=columnsXYZ(T,"fp_normal_nHat");
r.nDot=columnsXYZ(T,"fp_normal_nDot"); r.kappaHat=safeColumn(T,"fp_normal_kappaHat");
u1=safeColumn(T,"u1_des"); u2=safeColumn(T,"u2_des");
[r.forceDirection,~]=safeNormalizeRows(r.fcHat,small);
r.vcNorm=vecnorm(r.vc,2,2); r.nDotNorm=vecnorm(r.nDot,2,2);
denominator=abs(r.kappaHat).*hypot(u1,u2).^2; valid=isfinite(denominator)&denominator>small;
r.alphaGN=nan(height(T),1); r.alphaGN(valid)=sqrt(aBarN./denominator(valid));
r.alphaStar=min(1,r.alphaGN); r.alphaStar(~valid)=1;
r.geometry=fitTwoCylinderSurface(r.t,r.eePos,fitWindow,smallRadius,largeRadius,planeLength);
r.trueNormal=r.geometry.trueNormal; r.trueCurvature=r.geometry.trueCurvature;
end

function label=experimentLabel(fileName)
[~,label]=fileparts(fileName); label=regexprep(label,'_flyingpen_normal_.*$','');
label=strrep(label,'_',' ');
end

function decorateAxis(ax,yText,titleText)
grid(ax,'on'); ylabel(ax,yText); title(ax,titleText);
legend(ax,'Location','best','Interpreter','none');
end

function x=safeColumn(T,name)
vars=string(T.Properties.VariableNames); idx=find(strcmpi(vars,string(name)),1);
if isempty(idx), x=nan(height(T),1); else, x=double(T{:,idx}); end
x=x(:);
end

function xyz=columnsXYZ(T,prefix)
xyz=[safeColumn(T,string(prefix)+"X"),safeColumn(T,string(prefix)+"Y"),safeColumn(T,string(prefix)+"Z")];
end

function [unit,magnitude]=safeNormalizeRows(x,threshold)
magnitude=vecnorm(x,2,2); unit=nan(size(x)); valid=all(isfinite(x),2)&magnitude>threshold;
unit(valid,:)=x(valid,:)./magnitude(valid);
end

function errorDeg=normalAngleErrorDeg(reference,estimate)
cosine=sum(reference.*estimate,2)./(vecnorm(reference,2,2).*vecnorm(estimate,2,2));
cosine=min(1,max(-1,cosine)); errorDeg=acosd(cosine);
end

function applyTimeXLim(ax,limits)
if isempty(limits), return; end
validateattributes(limits,{'numeric'},{'vector','numel',2,'finite','increasing'});
for k=1:numel(ax), xlim(ax(k),limits); end
end

function addCurvatureShading(axesHandles,t,trueCurvature,smallRadius,largeRadius)
% Repeated portions of the same physical cylinder always receive one colour.
highMask=abs(trueCurvature-1/smallRadius)<0.08;
lowMask=abs(trueCurvature-1/largeRadius)<0.08;
masks={highMask,lowMask}; colors={[1.00 0.68 0.28],[0.35 0.65 1.00]};
for axisNumber=1:numel(axesHandles)
    ax=axesHandles(axisNumber); holdWasOn=ishold(ax); hold(ax,'on'); yLimits=ylim(ax);
    for curvatureType=1:2
        intervals=maskToIntervals(t,masks{curvatureType});
        for intervalNumber=1:size(intervals,1)
            interval=intervals(intervalNumber,:);
            region=patch(ax,[interval(1) interval(2) interval(2) interval(1)], ...
                [yLimits(1) yLimits(1) yLimits(2) yLimits(2)],colors{curvatureType}, ...
                'FaceAlpha',0.13,'EdgeColor','none','HandleVisibility','off');
            uistack(region,'bottom');
        end
    end
    if ~holdWasOn, hold(ax,'off'); end
end
end

function intervals=maskToIntervals(t,mask)
valid=mask(:)&isfinite(t(:)); starts=find(valid&[true;~valid(1:end-1)]);
stops=find(valid&[~valid(2:end);true]); intervals=[t(starts) t(stops)];
end

function idx=timeSpacedIndices(t,validMask,period)
valid=find(validMask); idx=zeros(0,1); if isempty(valid), return; end
idx=valid(1); lastTime=t(valid(1));
for k=2:numel(valid)
    if isfinite(t(valid(k)))&&t(valid(k))-lastTime>=period
        idx(end+1,1)=valid(k); lastTime=t(valid(k)); %#ok<AGROW>
    end
end
end

function fit=fitTwoCylinderSurface(t,eePos,timeWindow,smallRadius,largeRadius,planeLengthGuess)
sampleValid=isfinite(t)&all(isfinite(eePos),2); fitValid=sampleValid;
if ~isempty(timeWindow), fitValid=fitValid&t>=timeWindow(1)&t<=timeWindow(2); end
windowIdx=find(fitValid); if numel(windowIdx)<20, error('Not enough finite EE samples for geometry fit.'); end
rawWindow=eePos(windowIdx,1:2);
[loopStart,loopEnd]=longestClosedLoop(rawWindow,0.06,2.0);
fitIdx=windowIdx(loopStart:loopEnd); raw=eePos(fitIdx,1:2);
s=[0;cumsum(vecnorm(diff(raw,1,1),2,2))];
[sUnique,uniqueIdx]=unique(s,'stable'); raw=raw(uniqueIdx,:);
if sUnique(end)<=eps, error('EE trajectory has insufficient motion for geometry fit.'); end
sQuery=linspace(0,sUnique(end),240)';
points=[interp1(sUnique,raw(:,1),sQuery),interp1(sUnique,raw(:,2),sQuery)];
xMid=median(points(:,1)); yMin=min(points(:,2)); yMax=max(points(:,2));
q0Lower=[xMid,yMin+smallRadius,xMid,yMax-largeRadius];
q0Upper=[xMid,yMax-smallRadius,xMid,yMin+largeRadius];
options=optimset('Display','off','MaxFunEvals',6000,'MaxIter',2500,'TolX',1e-9,'TolFun',1e-10);
cost=@(q)twoCylinderFitCost(q,points,smallRadius,largeRadius,planeLengthGuess);
qLower=fminsearch(cost,q0Lower,options); qUpper=fminsearch(cost,q0Upper,options);
if cost(qUpper)<cost(qLower), q=qUpper; else, q=qLower; end
[centerline,denseCurvature]=twoCylinderBoundary(q,smallRadius,largeRadius,720);
idx=find(sampleValid); trueNormal=nan(size(eePos)); trueCurvature=nan(size(t));
previous=[centerline(end-1,:);centerline;centerline(2,:)]; tangent=previous(3:end,:)-previous(1:end-2,:);
outward=[tangent(:,2),-tangent(:,1)]; outward=outward./max(vecnorm(outward,2,2),eps);
distances=nan(numel(idx),1);
for k=1:numel(idx)
    [distances(k),nearest]=min(vecnorm(centerline-eePos(idx(k),1:2),2,2));
    if distances(k)<=0.06
        trueNormal(idx(k),:)=[outward(nearest,:),0];
        trueCurvature(idx(k))=denseCurvature(nearest);
    end
end
centerDistance=norm(q(3:4)-q(1:2));
fit.origin=[mean([q(1),q(3)]),mean([q(2),q(4)]),median(eePos(windowIdx,3))];
fit.smallCenter=q(1:2); fit.largeCenter=q(3:4);
fit.smallRadius=smallRadius; fit.largeRadius=largeRadius;
fit.centerDistance=centerDistance;
fit.axisAngleDeg=atan2d(q(4)-q(2),q(3)-q(1));
fit.tangentLength=sqrt(max(0,centerDistance^2-(largeRadius-smallRadius)^2));
fit.rms=sqrt(mean(distances(ismember(idx,fitIdx)).^2,'omitnan'));
fit.loopTime=[t(fitIdx(1)) t(fitIdx(end))];
fit.trueNormal=trueNormal;
fit.trueCurvature=trueCurvature; fit.centerline=centerline;
end

function [bestStart,bestEnd]=longestClosedLoop(points,closureTolerance,minLoopLength)
% Locate the longest path segment whose endpoints return to the same place.
% This rejects approach/departure tails without relying on traversal speed.
s=[0;cumsum(vecnorm(diff(points,1,1),2,2))];
bestStart=1; bestEnd=size(points,1); bestLength=-inf;
stride=max(1,floor(size(points,1)/1200));
candidateStarts=1:stride:size(points,1);
for startIndex=candidateStarts
    eligible=find(s-s(startIndex)>=minLoopLength & ...
        vecnorm(points-points(startIndex,:),2,2)<=closureTolerance);
    if isempty(eligible), continue; end
    [candidateLength,which]=max(s(eligible)-s(startIndex));
    if candidateLength>bestLength
        bestLength=candidateLength; bestStart=startIndex; bestEnd=eligible(which);
    end
end
if bestLength<minLoopLength
    warning('No closed EE loop detected; fitting the full configured interval.');
    bestStart=1; bestEnd=size(points,1);
end
end

function cost=twoCylinderFitCost(q,points,smallRadius,largeRadius,planeLengthGuess)
if any(~isfinite(q))||norm(q(3:4)-q(1:2))<=largeRadius-smallRadius
    cost=1e3+sum(q.^2); return;
end
[boundary,~]=twoCylinderBoundary(q,smallRadius,largeRadius,360);
dPoint=min(pairwiseDistance(points,boundary),[],2); dBoundary=min(pairwiseDistance(boundary,points),[],2);
centerDistance=norm(q(3:4)-q(1:2));
tangentLength=sqrt(max(0,centerDistance^2-(largeRadius-smallRadius)^2));
cost=mean(min(dPoint,0.08).^2)+0.5*mean(min(dBoundary,0.08).^2)+ ...
    0.002*(tangentLength-planeLengthGuess)^2;
end

function [boundary,curvature]=twoCylinderBoundary(q,smallRadius,largeRadius,count)
angles=linspace(0,2*pi,ceil(count/2)+1)'; angles(end)=[];
smallCircle=q(1:2)+smallRadius*[cos(angles),sin(angles)];
largeCircle=q(3:4)+largeRadius*[cos(angles),sin(angles)];
cloud=[smallCircle;largeCircle]; hull=convhull(cloud(:,1),cloud(:,2),'Simplify',true);
hull(end)=[]; polygon=cloud(hull,:); s=[0;cumsum(vecnorm(diff([polygon;polygon(1,:)],1,1),2,2))];
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
    thisFile=mfilename('fullpath');
    if ~isempty(thisFile), folder=fileparts(thisFile); end
end
if isempty(folder), folder=pwd; end
end
