function fig = photo2pc_gui(cloudFile, imgFile)
%PHOTO2PC_GUI  Tabbed visual pipeline for photo <-> point-cloud alignment.
%
%   photo2pc_gui                       -- open the GUI, pick files via dialog
%   photo2pc_gui(cloudFile, imgFile)
%and 
%   Walks through every stage of photo2pc_align and shows it on screen.
%   Each tab gets the full window so the diagnostics are large and
%   readable:
%      1. Point cloud (3-D)
%      2. Photo + intrinsics
%      3. Plane fit (3-D)
%      4. Synthetic orthographic render
%      5. Equalised pair  (synth | photo)
%      6. Detected feature keypoints
%      7. All raw matches
%      8. RANSAC inliers + PnP reprojection
%      9. Photo crop showing overlap with the cloud
%     10. Original cloud  vs.  photo-recoloured cloud  (synced rotation)
%
%   No files are written to disk until you press "Save Results".

if nargin < 1, cloudFile = ''; end
if nargin < 2, imgFile   = ''; end

% Close any previous instances of this GUI (and any orphaned helpers
% it might have spawned) so windows don't stack across re-launches.
closeStaleWindows();

S = initState();
S.cloudFile = char(cloudFile);
S.imgFile   = char(imgFile);

fig = figure('Name','photo2pc_gui  —  pipeline visualizer', ...
    'NumberTitle','off','Color', S.theme.bg, ...
    'MenuBar','none','ToolBar','figure', ...
    'CloseRequestFcn',@(src,~)onClose(src));
% Adaptive sizing: place the figure to take ~90 % of the screen but
% never exceed 1700 x 950 px.  WindowState='maximized' takes over on
% platforms that support it; on those that don't, this fallback keeps
% the GUI usable on small and large monitors alike.
try
    scr = get(0,'ScreenSize');         % [left bottom width height]
    w = min(1700, scr(3) * 0.92);
    h = min(950,  scr(4) * 0.88);
    fig.Position = [max(1,(scr(3)-w)/2), max(40,(scr(4)-h)/2), w, h];
catch
end
try, fig.WindowState = 'maximized'; catch, end

S = buildUI(fig, S);
guidata(fig, S);
refreshHeader(fig);
end

% =====================================================================
%   STATE
% =====================================================================
function S = initState()
S.cloudFile = '';
S.imgFile   = '';
S.maxCloudPts = 8e5;
S.autoRenderWidth = 1500;
S.displayMaxImageWidth = 2200;   % preview only; full-res data stays intact
S.displayMaxCloudPoints = 5e5;   % cap for pcshow rendering only; full cloud kept for save/return
S.busy      = false;
% Manual mode: when on, the auto-only stages (5/6/7 — equalise pair,
% feature detection, raw matches) are skipped and stage 8 opens the
% photo<->synth picker directly.  Stages 1-4 still run because the
% picker needs the plane fit and the synthetic ortho render.
S.manualMode = false;

% Dark-theme colour palette (single source of truth).
S.theme = struct( ...
    'bg',      [0.13 0.14 0.17], ...  % figure background
    'panel',   [0.18 0.19 0.23], ...  % header / description / tab panels
    'text',    [0.92 0.93 0.96], ...  % primary text on dark
    'subtext', [0.62 0.64 0.70], ...  % dim placeholder / skipped text
    'accent',  [0.42 0.72 1.00], ...  % info / status messages
    'success', [0.30 0.62 0.42], ...  % Run / primary action buttons
    'warn',    [0.62 0.32 0.30], ...  % Cancel / destructive
    'input',   [0.22 0.24 0.29], ...  % edit boxes / dropdowns / neutral btns
    'axBg',    [0.10 0.11 0.14], ...  % axes (plot) background
    'border',  [0.32 0.34 0.40]);     % subtle panel borders

S.pc   = []; S.pcF = []; S.xyz = []; S.rgb = [];
S.img  = []; S.H = 0; S.W = 0;
S.imgForMatch = [];   % photo with non-face pixels masked out (optional)
S.faceMask    = [];   % face mask in photo pixels, for visualisation
S.candidateFaceMask = [];
S.autoFaceInfo = struct('status','','nRaw',0,'nInliers',0,'areaPct',0);
S.intr = []; S.K = [];

S.plane = struct('c',[],'u',[],'v',[],'n',[]);

S.synth = []; S.synthMask = []; S.HsynW = [0 0]; S.scl = 1;
S.gRs = []; S.gS = [];
% Per-detector feature pairs produced by autoFaceMaskInline on the
% UNMASKED photo gray.  stage_features reuses these whenever no face
% mask gets applied (i.e. the matching gray is identical to that input),
% avoiding a second SIFT/KAZE/ORB run on the same image.
S.cachedFeatures = struct('valid', false);
S.ortho  = struct('uLo',0,'vLo',0,'dx',1);

S.feat   = struct('pr',zeros(0,2),'ps',zeros(0,2));
S.matches= struct('pr',zeros(0,2),'ps',zeros(0,2));
S.inliers= struct('pr',zeros(0,2),'ps',zeros(0,2));

S.imPts  = zeros(0,2);
S.wPts   = zeros(0,3);
S.pose   = [];
S.R_cw   = []; S.t_cw = [];
S.rms    = NaN;

S.cropBox = [];          % [x y w h] in photo pixels
S.projCropMask = [];
S.selectedCropMask = [];
S.selectedCropSource = 'projection';
S.coloredCloud = [];
S.cropRefined = false;    % one automatic post-crop refinement pass

S.viewLink = [];         % linkprop handle for synced rotation

S.stage  = 0;
S.stageNames = {
    'Loaded point cloud (3-D)'
    'Loaded photo + intrinsics'
    'Plane fit (dominant face plane)'
    'Synthetic orthographic render'
    'Illumination-equalised pair  (synth | photo)'
    'Detected feature keypoints'
    'All raw feature matches'
    'RANSAC inliers + PnP reprojection'
    'Photo crop  (original | overlap)'
    'Cloud comparison  (original | recoloured, synced)'
};
% Visible tabs are MERGED:
%   tab 1: stages 1 + 2 (cloud + photo)              -> 2 tiles
%   tab 2: stages 3 + 4 + 5 (plane / synth / eq.)    -> 4 tiles (eq pair is split)
%   tab 3..7: stages 6..10 one-to-one
% S.stageTab maps each of the 10 stages to its 1-based tab index.
S.stageTab = [1 1 2 2 2 3 4 5 6 7];
S.tabShort = {
    '1. Inputs'
    '2. Plane / Synth / Equalised'
    '3. Features'
    '4. Matches'
    '5. PnP reproj'
    '6. Photo crop'
    '7. Cloud compare'
};
end

% =====================================================================
%   UI CONSTRUCTION
% =====================================================================
function S = buildUI(fig, S)
T = S.theme;          % shorthand for the theme palette
pad = 0.005;
hdrH = 0.085;
S.h.hdrPanel = uipanel('Parent',fig,'Units','normalized', ...
    'Position',[pad 1-hdrH-pad 1-2*pad hdrH], ...
    'BackgroundColor', T.panel, 'BorderType','line', ...
    'HighlightColor', T.border, 'ForegroundColor', T.text);

uicontrol(S.h.hdrPanel,'Style','text', ...
    'Units','normalized','Position',[0.005 0.55 0.06 0.4], ...
    'String','Cloud:','HorizontalAlignment','right','FontSize',10, ...
    'BackgroundColor', T.panel, 'ForegroundColor', T.text);
S.h.cloudEdit = uicontrol(S.h.hdrPanel,'Style','edit', ...
    'Units','normalized','Position',[0.07 0.55 0.30 0.4], ...
    'String',S.cloudFile,'HorizontalAlignment','left','FontSize',10, ...
    'BackgroundColor', T.input, 'ForegroundColor', T.text);
uicontrol(S.h.hdrPanel,'Style','pushbutton', ...
    'Units','normalized','Position',[0.375 0.55 0.05 0.4], ...
    'String','Browse','FontSize',9, ...
    'BackgroundColor', T.input, 'ForegroundColor', T.text, ...
    'Callback',@(~,~)browseCloud(fig));

uicontrol(S.h.hdrPanel,'Style','text', ...
    'Units','normalized','Position',[0.005 0.10 0.06 0.4], ...
    'String','Photo:','HorizontalAlignment','right','FontSize',10, ...
    'BackgroundColor', T.panel, 'ForegroundColor', T.text);
S.h.imgEdit = uicontrol(S.h.hdrPanel,'Style','edit', ...
    'Units','normalized','Position',[0.07 0.10 0.30 0.4], ...
    'String',S.imgFile,'HorizontalAlignment','left','FontSize',10, ...
    'BackgroundColor', T.input, 'ForegroundColor', T.text);
uicontrol(S.h.hdrPanel,'Style','pushbutton', ...
    'Units','normalized','Position',[0.375 0.10 0.05 0.4], ...
    'String','Browse','FontSize',9, ...
    'BackgroundColor', T.input, 'ForegroundColor', T.text, ...
    'Callback',@(~,~)browseImg(fig));

% Status banner on the top half; manual-mode toggle on the bottom half.
% Manual mode skips the auto feature stages and drops the user straight
% into the photo<->synth picker after the plane/ortho render is built.
S.h.statusLbl = uicontrol(S.h.hdrPanel,'Style','text', ...
    'Units','normalized','Position',[0.44 0.55 0.40 0.40], ...
    'String','Idle. Pick files and click Run All.', ...
    'HorizontalAlignment','left','FontSize',10, ...
    'ForegroundColor', T.accent, ...
    'BackgroundColor', T.panel);
S.h.manualToggle = uicontrol(S.h.hdrPanel,'Style','checkbox', ...
    'Units','normalized','Position',[0.44 0.10 0.40 0.40], ...
    'String',['Manual mode  (skip auto feature matching;  ', ...
              'pick correspondences on photo <-> synth ortho)'], ...
    'Value', S.manualMode, ...
    'HorizontalAlignment','left','FontSize',10, ...
    'ForegroundColor', T.text, ...
    'BackgroundColor', T.panel, ...
    'Callback',@(src,~)onManualToggle(fig, src));

S.h.runBtn = uicontrol(S.h.hdrPanel,'Style','pushbutton', ...
    'Units','normalized','Position',[0.85 0.55 0.07 0.40], ...
    'String','Run All','FontSize',10,'FontWeight','bold', ...
    'BackgroundColor', T.success, 'ForegroundColor', T.text, ...
    'Callback',@(~,~)onRunAll(fig));
S.h.stepBtn = uicontrol(S.h.hdrPanel,'Style','pushbutton', ...
    'Units','normalized','Position',[0.925 0.55 0.07 0.40], ...
    'String','Step >','FontSize',10, ...
    'BackgroundColor', T.input, 'ForegroundColor', T.text, ...
    'Callback',@(~,~)onStep(fig));
S.h.resetBtn = uicontrol(S.h.hdrPanel,'Style','pushbutton', ...
    'Units','normalized','Position',[0.85 0.10 0.07 0.40], ...
    'String','Reset','FontSize',10, ...
    'BackgroundColor', T.input, 'ForegroundColor', T.text, ...
    'Callback',@(~,~)onReset(fig));
S.h.saveBtn = uicontrol(S.h.hdrPanel,'Style','pushbutton', ...
    'Units','normalized','Position',[0.925 0.10 0.07 0.40], ...
    'String','Save Results','FontSize',10,'FontWeight','bold', ...
    'BackgroundColor', [0.25 0.42 0.65], 'ForegroundColor', T.text, ...
    'Callback',@(~,~)onSave(fig));
S.h.wb = [];   % progress dialog handle (created on demand)

% ---- tab group below the header
nStages = numel(S.stageNames);     % 10
nTabs   = numel(S.tabShort);        % 7 (merged)
S.h.tabGroup = uitabgroup('Parent',fig, 'Units','normalized', ...
    'Position',[pad pad 1-2*pad 1-hdrH-3*pad]);

% Indexing convention:
%   S.tab  : 1 entry per STAGE (10).  Multiple stages can share the same
%            uitab handle when they live on the same merged tab.
%   S.ax   : 1 entry per STAGE (10).  Each stage's primary axes.
%   S.axR  : 1 entry per STAGE (10).  Optional second axes for stages
%            that draw a side-by-side pair (5, 9, 10).
%   S.descLbl : 1 entry per TAB (7).  Description banner.  Per-stage
%            text is accumulated on the descLbl's UserData so a merged
%            tab shows all of its stages' descriptions stacked together.
S.tab     = gobjects(nStages, 1);
S.ax      = gobjects(nStages, 1);
S.axR     = gobjects(nStages, 1);
S.descLbl = gobjects(nTabs, 1);

% Layout (normalized to the tab area).  Description occupies the full-
% width bottom band; axes fill the rest.
descPos = [0.012 0.015 0.976 0.260];
FULL    = [0.012 0.295 0.976 0.680];
LFULL   = [0.012 0.295 0.485 0.680];
RFULL   = [0.503 0.295 0.485 0.680];
TL2x2   = [0.012 0.638 0.485 0.337];
TR2x2   = [0.503 0.638 0.485 0.337];
BL2x2   = [0.012 0.295 0.485 0.337];
BR2x2   = [0.503 0.295 0.485 0.337];

% Per-stage axes positions (within their assigned tab).
% Stage 5 also gets a second axes (axR) so the equalised pair can occupy
% two tiles instead of being a side-by-side composite in one tile.
stagePos  = cell(nStages, 1);
stagePosR = cell(nStages, 1);
stagePos{1}  = LFULL;
stagePos{2}  = RFULL;
stagePos{3}  = TL2x2;
stagePos{4}  = TR2x2;
stagePos{5}  = BL2x2;   stagePosR{5}  = BR2x2;
stagePos{6}  = FULL;
stagePos{7}  = FULL;
stagePos{8}  = FULL;
stagePos{9}  = LFULL;   stagePosR{9}  = RFULL;
stagePos{10} = LFULL;   stagePosR{10} = RFULL;

% Create the visible tabs and one description banner per tab.
tabHandles = gobjects(nTabs, 1);
for tb = 1:nTabs
    tabHandles(tb) = uitab(S.h.tabGroup, 'Title', S.tabShort{tb});
    try, tabHandles(tb).BackgroundColor = T.bg; catch, end
    S.descLbl(tb) = makeDescPanel(tabHandles(tb), descPos, T);
    S.descLbl(tb).UserData = struct();   % accumulates per-stage strings
end

% Assign each stage to its tab and create its axes.
for k = 1:nStages
    S.tab(k) = tabHandles(S.stageTab(k));
    S.ax(k)  = axes('Parent', S.tab(k), 'Units','normalized', ...
        'Position', stagePos{k}, 'Box','on');
    pendingPlaceholder(S.ax(k));
    if ~isempty(stagePosR{k})
        S.axR(k) = axes('Parent', S.tab(k), 'Units','normalized', ...
            'Position', stagePosR{k}, 'Box','on');
        pendingPlaceholder(S.axR(k));
    end
end
end

function h = makeDescPanel(parent, pos, T)
% Multi-line description panel below the plot.  uicontrol 'text' is
% the most portable and reads cleanly at any screen size.
h = uicontrol('Parent', parent, 'Style','text', 'Units','normalized', ...
    'Position', pos, ...
    'String', '(description appears after this stage runs)', ...
    'HorizontalAlignment','left', ...
    'FontSize', 11, ...
    'ForegroundColor', T.text, ...
    'BackgroundColor', T.panel);
end

function pendingPlaceholder(ax)
axis(ax,'off');
set(ax, 'Color', [0.10 0.11 0.14]);   % dark-theme axes background
text(ax, 0.5, 0.5, '(pending — click Run All or Step >)', ...
    'Units','normalized', 'HorizontalAlignment','center', ...
    'Color',[0.62 0.64 0.70], 'FontSize',12);
end

function S = ensureSplitAxes(S, k)
% Repair older in-memory GUI state where a stage was created as a
% single-axis tab but later code expects left/right axes.
if isgraphics(S.ax(k)) && isgraphics(S.axR(k))
    return;
end
axLeft  = [0.012 0.295 0.485 0.68];
axRight = [0.503 0.295 0.485 0.68];
if ~isgraphics(S.ax(k))
    S.ax(k) = axes('Parent',S.tab(k), 'Units','normalized', ...
        'Position', axLeft, 'Box','on');
else
    set(S.ax(k), 'Units','normalized', 'Position', axLeft, 'Box','on');
end
if ~isgraphics(S.axR(k))
    S.axR(k) = axes('Parent',S.tab(k), 'Units','normalized', ...
        'Position', axRight, 'Box','on');
    pendingPlaceholder(S.axR(k));
else
    set(S.axR(k), 'Units','normalized', 'Position', axRight, 'Box','on');
end
end

function refreshHeader(fig)
S = guidata(fig);
S.h.cloudEdit.String = S.cloudFile;
S.h.imgEdit.String   = S.imgFile;
guidata(fig, S);
end

% =====================================================================
%   CALLBACKS
% =====================================================================
function onManualToggle(fig, src)
% Mode is a top-level pipeline choice.  Mixing modes mid-run leaves
% stale fields from the previous mode (matches/inliers/faceMask), so
% changing the toggle always resets the pipeline - with a confirmation
% prompt when there's prior-run output that would be discarded.
S = guidata(fig);
prev = S.manualMode;
desired = logical(src.Value);
if prev == desired, return; end

if S.busy
    src.Value = prev;
    setStatus(fig, ['Cannot change mode while the pipeline is ', ...
        'running.  Wait for it to finish, then toggle.'], [0.70 0.30 0]);
    return;
end

hasWork = S.stage > 0 || ~isempty(S.pose) || ~isempty(S.imPts);
if hasWork
    ans_ = questdlg( ...
        ['Switching between manual and auto mode resets the ', ...
         'pipeline.  Your current pose, picks and tab visualisations ', ...
         'will be discarded.  Continue?'], ...
        'Switching mode resets pipeline', ...
        'Switch & reset', 'Cancel', 'Cancel');
    if ~strcmp(ans_, 'Switch & reset')
        src.Value = prev;
        return;
    end
end

% onReset reads the checkbox value and preserves it across the reset,
% so manualMode ends up at `desired` automatically.
onReset(fig);
if desired
    setStatus(fig, ['Manual mode ON: pipeline reset.  ', ...
        'Click Run All to start.'], [0.30 0.30 0.70]);
else
    setStatus(fig, ['Auto mode ON: pipeline reset.  ', ...
        'Click Run All to start.'], [0.30 0.30 0.70]);
end
end

function browseCloud(fig)
S = guidata(fig);
[f,d] = uigetfile({'*.mat;*.ply','Point cloud (.mat, .ply)'; ...
    '*.*','All files'},'Pick point cloud', S.cloudFile);
if isequal(f,0), return; end
S.cloudFile = fullfile(d,f);
guidata(fig, S); refreshHeader(fig);
end

function browseImg(fig)
S = guidata(fig);
[f,d] = uigetfile({'*.jpg;*.jpeg;*.png;*.tif;*.tiff','Images'; ...
    '*.*','All files'},'Pick photo', S.imgFile);
if isequal(f,0), return; end
S.imgFile = fullfile(d,f);
guidata(fig, S); refreshHeader(fig);
end

function onRunAll(fig)
S = guidata(fig);
if S.busy, return; end
S.busy = true; guidata(fig, S);
setBusy(fig, true);
try
    S.cloudFile = strtrim(S.h.cloudEdit.String);
    S.imgFile   = strtrim(S.h.imgEdit.String);
    if isempty(S.cloudFile) || isempty(S.imgFile)
        setStatus(fig, 'Pick both a cloud and a photo first.', [0.6 0 0]);
        S.busy = false; guidata(fig, S);
        setBusy(fig, false);
        return;
    end
    guidata(fig, S);
    % Seed once per full run so the feature detectors and RANSAC stages
    % produce identical output on identical input.
    try, rng(0, 'twister'); catch, end

    nStages = numel(guidata(fig).stageNames);
    ensureWaitbar(fig, 0, 'Running pipeline: starting...');
    drawnow;

    for idx = 1:nStages
        S2 = guidata(fig);
        ensureWaitbar(fig, (idx-1)/nStages, ...
            sprintf('Stage %d/%d: %s', idx, nStages, S2.stageNames{idx}));
        runStage(fig, idx);
        S2 = guidata(fig); S2.h.tabGroup.SelectedTab = S2.tab(idx); drawnow;
        ensureWaitbar(fig, idx/nStages, ...
            sprintf('Stage %d/%d: %s — done', idx, nStages, S2.stageNames{idx}));
    end
    S = guidata(fig);
    setStatus(fig, sprintf('Done.  RMS = %.2f px on %d inliers.', ...
        S.rms, size(S.imPts,1)), [0 0.4 0]);
catch ME
    setStatus(fig, ['ERROR: ' ME.message], [0.7 0 0]);
    disp(getReport(ME,'extended'));
end
dismissWaitbar(fig);
setBusy(fig, false);
S = guidata(fig); S.busy = false; guidata(fig, S);
end

function onStep(fig)
S = guidata(fig);
if S.busy, return; end
S.busy = true; guidata(fig, S);
setBusy(fig, true);
try
    S.cloudFile = strtrim(S.h.cloudEdit.String);
    S.imgFile   = strtrim(S.h.imgEdit.String);
    if isempty(S.cloudFile) || isempty(S.imgFile)
        setStatus(fig, 'Pick both a cloud and a photo first.', [0.6 0 0]);
        S.busy = false; guidata(fig, S);
        setBusy(fig, false);
        return;
    end
    if S.stage == 0
        try, rng(0, 'twister'); catch, end
    end
    nStages = numel(S.stageNames);
    next = S.stage + 1;
    if next > nStages
        setStatus(fig, 'Pipeline complete. Press Reset to restart.', [0 0.4 0]);
    else
        guidata(fig, S);
        S2 = guidata(fig);
        ensureWaitbar(fig, (next-1)/nStages, ...
            sprintf('Stage %d/%d: %s', next, nStages, S2.stageNames{next}));
        runStage(fig, next);
        ensureWaitbar(fig, next/nStages, ...
            sprintf('Stage %d/%d: %s — done', next, nStages, S2.stageNames{next}));
        S2 = guidata(fig); S2.h.tabGroup.SelectedTab = S2.tab(next); drawnow;
    end
catch ME
    setStatus(fig, ['ERROR: ' ME.message], [0.7 0 0]);
    disp(getReport(ME,'extended'));
end
dismissWaitbar(fig);
setBusy(fig, false);
S = guidata(fig); S.busy = false; guidata(fig, S);
end

function markSkipped(fig, k)
S = guidata(fig);
T = S.theme;
ax = S.ax(k);
if isgraphics(ax)
    cla(ax,'reset'); axis(ax,'off'); set(ax,'Color', T.bg);
    text(ax, 0.5, 0.5, '(skipped)', ...
        'Units','normalized','HorizontalAlignment','center', ...
        'Color', T.subtext, 'FontSize',13, 'FontAngle','italic');
end
if isgraphics(S.axR(k))
    cla(S.axR(k),'reset'); axis(S.axR(k),'off');
    set(S.axR(k),'Color', T.bg);
end
% Drop any stale description this stage left from a previous run so
% the merged-tab banner no longer shows it.
tb = S.stageTab(k);
if tb >= 1 && tb <= numel(S.descLbl) && isgraphics(S.descLbl(tb))
    ud = S.descLbl(tb).UserData;
    if isstruct(ud)
        f = sprintf('stage%d', k);
        if isfield(ud, f)
            ud = rmfield(ud, f);
            S.descLbl(tb).UserData = ud;
            S.descLbl(tb).String   = renderTabDesc(ud, S.stageTab, tb);
        end
    end
end
end

function onReset(fig)
S = guidata(fig);
% drop any progress dialog still on screen
dismissWaitbar(fig);
S = guidata(fig);
cf = S.cloudFile; im = S.imgFile;
% break any existing linkprop so old handles can be deleted
try, delete(S.viewLink); catch, end
% wipe per-stage axes back to pending placeholders
nStages = numel(S.stageNames);
for k = 1:nStages
    if isgraphics(S.ax(k)),  cla(S.ax(k),'reset');  pendingPlaceholder(S.ax(k));  end
    if isgraphics(S.axR(k)), cla(S.axR(k),'reset'); pendingPlaceholder(S.axR(k)); end
end
% wipe accumulated description text on each merged-tab banner
for tb = 1:numel(S.descLbl)
    if isgraphics(S.descLbl(tb))
        S.descLbl(tb).UserData = struct();
        S.descLbl(tb).String   = '(description appears after this stage runs)';
    end
end
% reset state, but keep UI handles + tabs + axes + descLbl + waitbar
oldHandles = S.h; oldTabs = S.tab; oldAx = S.ax;
oldAxR    = S.axR; oldDescLbl = S.descLbl;
S = initState();
S.cloudFile = cf; S.imgFile = im;
S.h = oldHandles; S.tab = oldTabs; S.ax = oldAx; S.axR = oldAxR;
S.descLbl = oldDescLbl;
S.h.wb = [];
% Preserve the manual-mode checkbox value across reset (it's a user
% preference, not pipeline output).
if isfield(S.h, 'manualToggle') && isgraphics(S.h.manualToggle)
    S.manualMode = logical(S.h.manualToggle.Value);
end
guidata(fig, S);
refreshHeader(fig);
setStatus(fig, 'Reset. Click Run All to start.', [0.2 0.2 0.5]);
end

function onSave(fig)
S = guidata(fig);
if isempty(S.imPts) || isempty(S.pose)
    setStatus(fig, 'Nothing to save yet. Run the pipeline first.', [0.6 0 0]);
    return;
end
if isempty(S.coloredCloud) || isempty(S.cropBox)
    setStatus(fig, ['Save needs stages 9 (crop) and 10 (recolor) to ', ...
        'have completed.  Re-run the pipeline.'], [0.6 0 0]);
    return;
end
[~,stem] = fileparts(S.imgFile);
defaultDir = fileparts(S.imgFile);
if isempty(defaultDir), defaultDir = pwd; end
[matFile, matDir] = uiputfile({'*.mat','MAT alignment'}, ...
    'Save alignment as', fullfile(defaultDir, [stem '_p2pc_alignment.mat']));
if isequal(matFile,0), return; end

setStatus(fig, 'Building pixel <-> cloud maps...', [0.1 0.1 0.5]);
drawnow;
maps = buildCropToCloudMapsInline(S);

% Output paths (all next to the chosen .mat).
matPath        = fullfile(matDir, matFile);
overlayPngPath = fullfile(matDir, [stem '_p2pc_overlay.png']);
croppedPngPath = fullfile(matDir, [stem '_p2pc_cropped.png']);
plyPath        = fullfile(matDir, [stem '_p2pc_colored_cloud.ply']);

% -------------------------------------------------------------------
% Saved-struct layout for downstream geologic-segmentation work
% -------------------------------------------------------------------
% alignment.croppedImage          uint8  Hc x Wc x 3   tunnel-face crop
% alignment.croppedFaceMask       logical Hc x Wc      manual face mask
%                                                      cropped to bbox,
%                                                      [] if no mask was
%                                                      drawn
% alignment.croppedProjMask       logical Hc x Wc      projection support
%                                                      mask cropped to
%                                                      bbox (which pixels
%                                                      have any cloud
%                                                      point in front)
% alignment.cropBox               [x0 y0 w h]          crop bbox in
%                                                      ORIGINAL-photo
%                                                      pixel coords,
%                                                      1-based
%
% alignment.coloredCloudPoints    single N x 3         photo-recoloured
% alignment.coloredCloudColors    uint8  N x 3         cloud (full
%                                                      resolution; PLY
%                                                      sidecar has the
%                                                      same data)
%
% alignment.pixelToPointIdx       uint32 Hc x Wc       front-most cloud
%                                                      point index per
%                                                      cropped-image
%                                                      pixel; 0 = no
%                                                      point.  Indexes
%                                                      into coloredCloud*
% alignment.pointToPixel          single N x 2         (u, v) in
%                                                      cropped-image
%                                                      coords for each
%                                                      cloud point;
%                                                      NaN if outside
%                                                      the crop
% alignment.pixelToPointXYZ       single Hc x Wc x 3   convenience: the
%                                                      3-D point under
%                                                      each pixel
%                                                      (NaN where no
%                                                      point hits)
%
% alignment.intrinsics, .K        camera intrinsics
% alignment.worldPose             rigidtform3d (camera-in-world)
% alignment.R_cw, .t_cw           world->camera rotation + translation
% alignment.fullImageSize         [H W] of the ORIGINAL photo
% alignment.imagePoints,          PnP correspondences used (after
%   .worldPoints, .reprojRMS      inliers/refinement)
% -------------------------------------------------------------------
%
% TYPICAL USES
%
%   Pixel -> point (paint geologic regions on the cropped image, then
%   colour the matching cloud points):
%       L = imread('my_geology_labels.png');           % Hc x Wc uint8
%       newColors = alignment.coloredCloudColors;       % start from photo
%       for lbl = unique(L(:)).'
%           if lbl == 0, continue; end
%           pix = find(L == lbl);
%           pts = alignment.pixelToPointIdx(pix);
%           pts = pts(pts > 0);
%           newColors(pts, :) = repmat(colorForLabel(lbl), numel(pts), 1);
%       end
%       pc = pointCloud(double(alignment.coloredCloudPoints), ...
%           'Color', newColors);
%       pcwrite(pc, 'my_geology_overlay.ply');
%
%   Point -> pixel (highlight a 3-D region on the 2-D crop):
%       uv = alignment.pointToPixel(myPointIds, :);
%       valid = all(~isnan(uv), 2);
%       imshow(alignment.croppedImage); hold on;
%       plot(uv(valid,1), uv(valid,2), 'r.');
% -------------------------------------------------------------------

alignment = struct( ...
    'status','solved', ...
    'method', ternary(S.manualMode, 'gui-manual-picker', ...
        'gui-auto-ortho-SIFT'), ...
    'imageFile', S.imgFile, ...
    'cloudFile', S.cloudFile, ...
    'manualMode', S.manualMode, ...
    'intrinsics', maps.intrinsics, ...
    'K', maps.K, ...
    'worldPose', maps.worldPose, ...
    'R_cw', maps.R_cw, ...
    't_cw', maps.t_cw, ...
    'fullImageSize', maps.fullImageSize, ...
    'imagePoints', maps.imagePoints, ...
    'worldPoints', maps.worldPoints, ...
    'reprojRMS', maps.reprojRMS, ...
    'cropBox', maps.cropBox, ...
    'croppedImage', maps.croppedImage, ...
    'croppedFaceMask', maps.croppedFaceMask, ...
    'croppedProjMask', maps.croppedProjMask, ...
    'coloredCloudPoints', maps.coloredCloudPoints, ...
    'coloredCloudColors', maps.coloredCloudColors, ...
    'pixelToPointIdx', maps.pixelToPointIdx, ...
    'pointToPixel', maps.pointToPixel, ...
    'pixelToPointXYZ', maps.pixelToPointXYZ, ...
    'faceMask', S.faceMask, ...
    'projectionCropMask', S.projCropMask, ...
    'selectedCropMask', S.selectedCropMask, ...
    'selectedCropSource', S.selectedCropSource); %#ok<NASGU>

setStatus(fig, 'Writing output files...', [0.1 0.1 0.5]);
drawnow;
save(matPath, 'alignment', '-v7.3');

written = {matFile};
try
    exportgraphics(S.ax(8), overlayPngPath, 'Resolution',150);
    written{end+1} = [stem '_p2pc_overlay.png'];
catch
end
try
    imwrite(maps.croppedImage, croppedPngPath);
    written{end+1} = [stem '_p2pc_cropped.png'];
catch ME
    warning('photo2pc_gui:saveCrop', ...
        'Could not write cropped PNG: %s', ME.message);
end
try
    pcwrite(S.coloredCloud, plyPath, 'Encoding','binary');
    written{end+1} = [stem '_p2pc_colored_cloud.ply'];
catch ME
    warning('photo2pc_gui:savePly', ...
        'Could not write colored cloud PLY: %s', ME.message);
end

setStatus(fig, sprintf('Saved %d files in %s: %s', numel(written), ...
    matDir, strjoin(written, ', ')), [0 0.4 0]);
end

% =====================================================================
%   STAGE RUNNER
% =====================================================================
function runStage(fig, k)
S = guidata(fig);
% Manual mode: the equalise / feature-detect / raw-match stages exist
% only to feed the auto solver, so skip them when the user has chosen
% to pick correspondences by hand.  stage_solve detects the mode and
% opens the photo<->synth picker directly.
if S.manualMode && (k == 5 || k == 6 || k == 7)
    setStatus(fig, sprintf('Stage %d/%d: %s - skipped (manual mode).', ...
        k, numel(S.stageNames), S.stageNames{k}), [0.40 0.40 0.50]);
    markSkipped(fig, k);       % mutates axes + descLbl handles in place
    S.stage = k;
    guidata(fig, S);
    drawnow;
    return;
end
setStatus(fig, sprintf('Stage %d/%d: %s ...', k, numel(S.stageNames), ...
    S.stageNames{k}), [0.1 0.1 0.5]);
drawnow;
switch k
    case 1,  S = stage_loadCloud(S, fig);
    case 2,  S = stage_loadPhoto(S, fig);
    case 3,  S = stage_planefit(S, fig);
    case 4,  S = stage_ortho(S, fig);
    case 5,  S = stage_equalize(S, fig);
    case 6,  S = stage_features(S, fig);
    case 7,  S = stage_matches(S, fig);
    case 8,  S = stage_solve(S, fig);
    case 9
        S = stage_crop(S, fig);
        if shouldRefineFromProjectionCrop(S)
            S = refineAlignmentFromProjectionCrop(S, fig);
            S = stage_crop(S, fig);
        end
    case 10, S = stage_recolor(S, fig);
end
S.stage = k;
guidata(fig, S);
drawnow;
end

% =====================================================================
%   STAGES
% =====================================================================
function S = stage_loadCloud(S, ~)
S.pc  = loadCloudInline(S.cloudFile);
S.pcF = S.pc;
if S.pc.Count > S.maxCloudPts
    S.pcF = pcdownsample(S.pc,'random',S.maxCloudPts/S.pc.Count);
end
S.xyz = double(S.pcF.Location);
S.rgb = S.pcF.Color;
if isempty(S.rgb), S.rgb = uint8(repmat(200,size(S.xyz,1),3)); end

ax = S.ax(1); cla(ax,'reset'); axis(ax,'on'); set(ax,'Color', S.theme.axBg, ...
    'XColor', S.theme.text, 'YColor', S.theme.text, 'ZColor', S.theme.text, ...
    'GridColor', S.theme.border);
nShow = min(size(S.xyz,1), 3e5);
ridx = unique(randi(size(S.xyz,1),[nShow 1]));
scatter3(ax, S.xyz(ridx,1),S.xyz(ridx,2),S.xyz(ridx,3), 6, ...
    double(S.rgb(ridx,:))/255, '.');
axis(ax,'equal','vis3d'); grid(ax,'on'); rotate3d(ax,'on');
xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
title(ax,'1. Point cloud (3-D)');
themeAxes(ax, S.theme);
setDesc(S, 1, {
    'INPUT POINT CLOUD.  Loaded from the .mat or .ply file and shown with its original scanner RGB.  A random subset is drawn for interactive performance; drag inside the axes to rotate.'
    ''
    sprintf('Total points: %s     |     Displayed: %s (random subsample)', ...
        commaInt(S.pc.Count), commaInt(numel(ridx)))
    });
end

function S = stage_loadPhoto(S, ~)
S.img = imread(S.imgFile);
if size(S.img,3)==1, S.img = repmat(S.img,1,1,3); end
S.H = size(S.img,1); S.W = size(S.img,2);

S.intr = resolveIntrinsicsInline(S.imgFile, S.W, S.H);
S.K    = intrMatrixInline(S.intr);

ax = S.ax(2); cla(ax,'reset');
imshowPhotoPreviewInline(ax, S.img, S.W, S.H, S.displayMaxImageWidth);
title(ax,'2. Input photo + camera intrinsics');
themeAxes(ax, S.theme);
setDesc(S, 2, {
    'INPUT PHOTO + CAMERA INTRINSICS.  The RGB image whose pose we will recover.  Focal length and principal point come from EXIF when available; otherwise the focal length is estimated from the image dimensions.'
    ''
    sprintf('Photo: %d x %d px     |     Focal length: %.0f px     |     Principal point: (%.0f, %.0f)', ...
        S.W, S.H, S.K(1,1), S.K(1,3), S.K(2,3))
    sprintf('Source: %s', shortPath(S.imgFile))
    });
end

function S = stage_manualFaceCrop(S, fig)
% Review the automatic face mask after the equalised pair has been
% prepared.  The user can accept it, redraw/edit it, or skip masking.
S.h.tabGroup.SelectedTab = S.tab(6);
try, figure(fig); catch, end
drawnow;

ax = S.ax(6); cla(ax,'reset');
mask = S.candidateFaceMask;
if isempty(mask) || ~any(mask(:))
    mask = [];
end
showFaceCropReview(S, ax, mask);
themeAxes(ax, S.theme);

fig.Pointer = 'arrow';
dismissWaitbar(fig);
S = guidata(fig);
T = S.theme;
panel = uipanel('Parent', S.tab(6), 'Units','normalized', ...
    'Position', [0.012 0.015 0.976 0.260], ...
    'BorderType','line', 'HighlightColor', T.border, ...
    'BackgroundColor', T.panel, 'ForegroundColor', T.text);
uicontrol(panel, 'Style','text', 'Units','normalized', ...
    'Position',[0.02 0.55 0.96 0.40], ...
    'HorizontalAlignment','left', ...
    'BackgroundColor', T.panel, 'ForegroundColor', T.text, 'FontSize',10, ...
    'String','Review the proposed tunnel-face crop before feature detection. Accept it, edit/redraw it, or skip masking.');
uicontrol(panel, 'Style','text', 'Units','normalized', ...
    'Position',[0.02 0.30 0.96 0.20], ...
    'HorizontalAlignment','left', ...
    'BackgroundColor', T.panel, 'ForegroundColor', T.subtext, 'FontSize',9, ...
    'String','Edit uses brush strokes: blue adds face pixels, red removes pixels, then Finish continues.');

setappdata(fig, 'faceCropChoice', '');
uicontrol(panel, 'Style','pushbutton', 'Units','normalized', ...
    'Position',[0.02 0.05 0.18 0.22], 'String','Accept crop', ...
    'FontSize',10, 'FontWeight','bold', ...
    'Enable', ternary(isempty(mask), 'off', 'on'), ...
    'BackgroundColor', T.success, 'ForegroundColor', T.text, ...
    'Callback', @(~,~)pickChoice(fig,'accept'));
uicontrol(panel, 'Style','pushbutton', 'Units','normalized', ...
    'Position',[0.22 0.05 0.18 0.22], 'String','Brush edit', ...
    'FontSize',10, 'BackgroundColor', T.input, 'ForegroundColor', T.text, ...
    'Callback', @(~,~)pickChoice(fig,'edit'));
uicontrol(panel, 'Style','pushbutton', 'Units','normalized', ...
    'Position',[0.42 0.05 0.18 0.22], 'String','Skip mask', ...
    'FontSize',10, 'BackgroundColor', T.warn, 'ForegroundColor', T.text, ...
    'Callback', @(~,~)pickChoice(fig,'skip'));

setStatus(fig, 'Stage 6: review the proposed face crop, then accept, edit, or skip.', ...
    [0.10 0.10 0.50]);
drawnow;
uiwait(fig);
choice = getappdata(fig, 'faceCropChoice');
try, delete(panel); catch, end

if strcmp(choice, 'edit')
    mask = editFaceMaskInline(S, ax, mask);
elseif strcmp(choice, 'skip') || isempty(choice)
    mask = [];
end

if ~isempty(mask) && any(mask(:))
    S = applyFaceMaskForMatching(S, mask);
    showFaceCropReview(S, ax, mask);
else
    S.imgForMatch = [];
    S.faceMask = [];
    S.gRs = adaptiveEqualizeInline(downsamplePhotoGrayInline(S.img, S.scl));
    cla(ax,'reset'); imshow(S.img, 'Parent', ax);
    title(ax, '6. Face crop skipped');
    themeAxes(ax, S.theme);
    setDesc(S, 6, {
        'FACE CROP SKIPPED.  Feature detection will use the full photo.  The post-alignment Photo crop tab still uses the projected 3-D cloud to compute the final overlap crop.'
        });
end
fig.Pointer = 'watch';
return;
%{

title(ax, '3. Manual face crop  —  choose an option below');
themeAxes(ax, S.theme);

% Build an inline action panel inside the tab's description area.
% Stash the choice on the figure via setappdata so the button
% callbacks (which run on the event loop) can communicate with us.
setappdata(fig, 'faceCropChoice', '');

T = S.theme;
actionPanel = uipanel('Parent', S.tab(3), 'Units','normalized', ...
    'Position', [0.012 0.015 0.976 0.260], ...
    'BorderType','line', 'HighlightColor', T.border, ...
    'BackgroundColor', T.panel, 'ForegroundColor', T.text);

uicontrol(actionPanel, 'Style','text', 'Units','normalized', ...
    'Position',[0.02 0.55 0.96 0.40], ...
    'HorizontalAlignment','left', ...
    'BackgroundColor', T.panel, 'ForegroundColor', T.text, 'FontSize',10, ...
    'String','OPTIONAL: outline the tunnel face to exclude tunnel walls, lighting and equipment from feature matching.');
uicontrol(actionPanel, 'Style','text', 'Units','normalized', ...
    'Position',[0.02 0.30 0.96 0.20], ...
    'HorizontalAlignment','left', ...
    'BackgroundColor', T.panel, 'ForegroundColor', T.subtext, 'FontSize',9, ...
    'String','Skip is fine for most photos; the auto pipeline usually works without it.');

btnSkip = uicontrol(actionPanel, 'Style','pushbutton', ...
    'Units','normalized', 'Position',[0.02 0.05 0.20 0.22], ...
    'String','Skip  (use full photo)','FontSize',10, ...
    'BackgroundColor', T.input, 'ForegroundColor', T.text, ...
    'Callback', @(~,~)pickChoice(fig,'skip'));
btnDraw = uicontrol(actionPanel, 'Style','pushbutton', ...
    'Units','normalized', 'Position',[0.24 0.05 0.20 0.22], ...
    'String','Draw polygon','FontSize',10, ...
    'FontWeight','bold', ...
    'BackgroundColor', T.success, 'ForegroundColor', T.text, ...
    'Callback', @(~,~)pickChoice(fig,'draw'));     %#ok<NASGU>

setStatus(fig, ['Stage 3: pick "Skip" or "Draw polygon" below the photo.'], ...
    [0.10 0.10 0.50]);
drawnow;

uiwait(fig);                          % wait for either button
choice = getappdata(fig, 'faceCropChoice');
try, delete(actionPanel); catch, end

if isempty(choice) || strcmp(choice,'skip')
    S.imgForMatch = []; S.faceMask = [];
    setDesc(S, 3, {
        'MANUAL FACE CROP (SKIPPED):  Using the full photo for'
        'feature matching.  This is the default workflow - the auto'
        'pipeline is usually good enough without manual cropping.'
        ''
        'To use this step, run again and click "Draw polygon".'
        });
    fig.Pointer = 'watch';
    return;
end

% --- DRAW POLYGON ----------------------------------------------------
title(ax, '3. Manual face crop  —  click vertices, double-click to finish');
themeAxes(ax, S.theme);
setStatus(fig, ['Click around the tunnel face to add vertices.  ', ...
    'Drag to reposition.  Double-click the last point (or press Enter) ', ...
    'to finish.'], [0.10 0.10 0.50]);
drawnow;

mask = [];
poly = [];
try
    poly = drawpolygon(ax, 'Color',[0.2 1 0.2], 'LineWidth',2, ...
        'FaceAlpha',0.15);
catch ME
    try
        mask = roipoly(S.img);
    catch
        warning('photo2pc_gui:drawTool', ...
            'No polygon tool available (%s).  Skipping crop.', ME.message);
    end
end

if ~isempty(poly) && isobject(poly) && isvalid(poly)
    pts = poly.Position;
    if size(pts,1) >= 3
        mask = createMask(poly, S.img);
    end
    try, delete(poly); catch, end
end

if isempty(mask) || ~any(mask(:))
    S.imgForMatch = []; S.faceMask = [];
    setDesc(S, 3, {
        'MANUAL FACE CROP (NO POLYGON):  No valid polygon was drawn.'
        'Continuing with the full photo.'
        });
    fig.Pointer = 'watch';
    return;
end

% Build the masked photo for matching.  S.img is left untouched so
% projection overlay and recoloured cloud still use original pixels.
masked = S.img;
m3 = repmat(mask, 1, 1, size(S.img,3));
masked(~m3) = 0;
S.imgForMatch = masked;
S.faceMask    = mask;

cla(ax,'reset');
imshow(masked,'Parent',ax); hold(ax,'on');
try
    Bd = bwboundaries(mask, 'noholes');
    for k = 1:numel(Bd)
        plot(ax, Bd{k}(:,2), Bd{k}(:,1), 'Color',[0.2 1 0.2], 'LineWidth',2);
    end
catch
end
hold(ax,'off');
title(ax, '3. Manual face crop applied');
themeAxes(ax, S.theme);

pct = 100 * nnz(mask) / numel(mask);
setDesc(S, 3, {
    'MANUAL FACE CROP APPLIED:  Pixels outside the polygon are zeroed'
    'so SIFT / KAZE / ORB only respond to the face area.  The original'
    'photo is still used for the projection overlay and the final'
    'recoloured cloud, so no information is lost.'
    ''
    sprintf('Face pixels kept: %s of %s  (%.1f%% of the photo)', ...
        commaInt(nnz(mask)), commaInt(numel(mask)), pct)
    });
fig.Pointer = 'watch';
%}
end

function pickChoice(fig, choice)
% Button callback for the face-crop action panel: records the user's
% pick and resumes the paused pipeline.
setappdata(fig, 'faceCropChoice', choice);
uiresume(fig);
end

function S = stage_planefit(S, ~)
c = mean(S.xyz,1);
[~,~,V] = svd(S.xyz - c, "econ");
S.plane.c = c;
u = V(:,1).'/norm(V(:,1));
v = V(:,2).'/norm(V(:,2));
n = V(:,3).'/norm(V(:,3));
% Force consistent orientation: v points down in world, right-handed.
[u, v, n] = orientPlaneAxesInline(u, v, n);
% Resolve the +/- sign of n (front vs back of face) by trying both
% orientations and keeping the one whose synth ortho has more SIFT
% matches against the photo.  Use the FULL cloud (not the downsampled
% S.xyz) so the small test render is dense enough for SIFT.
if ~isempty(S.img)
    xyzFull = double(S.pc.Location);
    rgbFull = S.pc.Color;
    if isempty(rgbFull)
        rgbFull = uint8(repmat(200,size(xyzFull,1),3));
    end
    [u, v, n, oinfo] = chooseFrontOrientationInline( ...
        u, v, n, xyzFull, rgbFull, S.img); %#ok<NASGU>
end
S.plane.u = u; S.plane.v = v; S.plane.n = n;

ax = S.ax(3); cla(ax,'reset'); axis(ax,'on'); set(ax,'Color', S.theme.axBg, ...
    'XColor', S.theme.text, 'YColor', S.theme.text, 'ZColor', S.theme.text, ...
    'GridColor', S.theme.border);
nShow = min(size(S.xyz,1), 3e5);
ridx = unique(randi(size(S.xyz,1),[nShow 1]));
scatter3(ax, S.xyz(ridx,1),S.xyz(ridx,2),S.xyz(ridx,3), 6, ...
    double(S.rgb(ridx,:))/255, '.');
hold(ax,'on');
d = S.xyz - c;
up = d * S.plane.u.';  vp = d * S.plane.v.';
% Vector-form prctile -> one sort per axis instead of two.
uB = prctile(up, [1 99]); uLo = uB(1); uHi = uB(2);
vB = prctile(vp, [1 99]); vLo = vB(1); vHi = vB(2);
corners = c + [uLo vLo; uHi vLo; uHi vHi; uLo vHi] * [S.plane.u; S.plane.v];
patch(ax, corners(:,1), corners(:,2), corners(:,3), [0.4 0.7 1.0], ...
    'FaceAlpha',0.18, 'EdgeColor',[0.1 0.3 0.7], 'LineWidth',1.5);
quiver3(ax, c(1),c(2),c(3), S.plane.n(1),S.plane.n(2),S.plane.n(3), ...
    norm([uHi-uLo, vHi-vLo])*0.4, 'LineWidth',2.5, 'Color',[0.85 0.1 0.1], ...
    'MaxHeadSize',0.5);
hold(ax,'off');
axis(ax,'equal','vis3d'); grid(ax,'on'); rotate3d(ax,'on');
view(ax, S.plane.n);
xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');
title(ax,'3. Plane fit (dominant face plane)');
themeAxes(ax, S.theme);
setDesc(S, 3, {
    'PLANE FIT.  SVD of the centred cloud locates the dominant plane (blue patch) and its outward normal (red arrow), which becomes the viewing direction for the next ortho render.  Axes are oriented so v points down in world Z; the sign of n is resolved by trying both directions and keeping the one that produces more SIFT matches against the photo.'
    ''
    sprintf('Plane normal n = [%.3f, %.3f, %.3f]', S.plane.n)
    });
end

function S = stage_ortho(S, ~)
c = S.plane.c; u = S.plane.u; v = S.plane.v; n = S.plane.n;
d = S.xyz - c;
up = d*u.'; vp = d*v.'; wp = d*n.';

% Vector-form prctile -> one sort per axis instead of two.
uB = prctile(up, [0.5 99.5]); uLo = uB(1); uHi = uB(2);
vB = prctile(vp, [0.5 99.5]); vLo = vB(1); vHi = vB(2);
Wsyn = max(400, round(S.autoRenderWidth));
dx   = (uHi-uLo)/Wsyn;
Hsyn = max(50, round((vHi-vLo)/dx));
ix = floor((up - uLo)/dx) + 1;
iy = floor((vp - vLo)/dx) + 1;
keep = ix>=1 & ix<=Wsyn & iy>=1 & iy<=Hsyn;
ix = ix(keep); iy = iy(keep);
rgbK = S.rgb(keep,:); wD = wp(keep);

[~,ord] = sort(wD,'descend');
ix = ix(ord); iy = iy(ord); rgbK = rgbK(ord,:);
lin = sub2ind([Hsyn Wsyn], iy, ix);

synth = zeros(Hsyn,Wsyn,3,'uint8');
for ch = 1:3
    Cc = zeros(Hsyn,Wsyn,'uint8');
    Cc(lin) = rgbK(:,ch);
    synth(:,:,ch) = Cc;
end
valid = any(synth>0,3);
synthMask = valid;
try
    synthMask = imclose(synthMask, strel('disk',5));
    synthMask = imfill(synthMask, 'holes');
    synthMask = imdilate(synthMask, strel('disk',8));
catch
end

try
    dil = imdilate(synth, strel('disk',3));
    for ch = 1:3
        O = synth(:,:,ch); D = dil(:,:,ch);
        O(~valid) = D(~valid);
        synth(:,:,ch) = O;
    end
catch
end

S.synth = synth;
S.synthMask = synthMask;
S.HsynW = [Hsyn Wsyn];
S.ortho.uLo = uLo; S.ortho.vLo = vLo; S.ortho.dx = dx;

ax = S.ax(4); cla(ax,'reset');
imshow(synth,'Parent',ax);
title(ax,'4. Synthetic orthographic render');
themeAxes(ax, S.theme);
setDesc(S, 4, {
    'ORTHOGRAPHIC RENDER.  The cloud is projected onto its dominant plane along the surface normal, giving a synthetic image of the face as seen straight-on.  Small gaps are dilation-filled so the feature detectors see continuous regions.  This synthetic image is what the photo will be matched against.'
    ''
    sprintf('Synth size: %d x %d px     |     Pixel size: %.4f cloud-units / px', Wsyn, Hsyn, dx)
    });
end

function S = stage_equalize(S, fig)
% Prepare the equalised pair from the full photo.  In autoThenManual
% mode, also compute a candidate face mask for the next review stage;
% do not apply it until the user confirms or edits it.
S.candidateFaceMask = [];
S.scl = S.HsynW(2) / S.W;

% Compute the CLAHE'd photo gray and synth gray ONCE here, then pass
% both into autoFaceMaskInline so that helper doesn't redo the same
% rgb2gray + CLAHE work internally.
gRFullEq = adaptiveEqualizeInline(downsamplePhotoGrayInline(S.img, S.scl));
S.gS     = adaptiveEqualizeInline(rgb2gray(S.synth));

if ~isempty(S.synthMask)
    [mask, info, featCache] = autoFaceMaskInline( ...
        S.img, S.synth, S.synthMask, gRFullEq, S.gS);
    S.autoFaceInfo = info;
    S.cachedFeatures = featCache;
    if ~isempty(mask) && any(mask(:))
        S.candidateFaceMask = mask;
        S.faceMask = mask;
    end
else
    S.autoFaceInfo.status = 'missing synthetic face mask';
    S.cachedFeatures = struct('valid', false);
end

% Fallback: if the automatic face-mask detector failed (or there was no
% synthetic support to match against), open the manual brush editor.
% A user-drawn mask invalidates the cached features (autoFaceMaskInline
% ran on the unmasked photo), so we discard the cache here and let
% stage_features redetect on the now-masked gray.
if isempty(S.faceMask)
    if isgraphics(fig)
        setStatus(fig, 'Auto face crop failed - opening manual brush editor...', [0.7 0.3 0]);
        drawnow;
    end
    drawnMask = manualFaceMaskBrushInline(S);
    if ~isempty(drawnMask) && any(drawnMask(:))
        S.candidateFaceMask = drawnMask;
        S.faceMask          = drawnMask;
        S.cachedFeatures    = struct('valid', false);
    end
end

if isempty(S.faceMask)
    S.faceMask = [];
end
S.imgForMatch = [];
if isempty(S.faceMask)
    % No mask -> matching gray = unmasked-equalised gray (already cached).
    S.gRs = gRFullEq;
else
    % Masked matching gray: zero non-face BEFORE CLAHE so the equaliser
    % doesn't amplify noise in the suppressed regions.  This branch
    % can't reuse gRFullEq directly because masking must happen first.
    gRFullSmall = downsamplePhotoGrayInline(S.img, S.scl);
    maskSmall = imresize(S.faceMask, size(gRFullSmall), 'nearest') > 0;
    gRFullSmall(~maskSmall) = 0;
    S.gRs = adaptiveEqualizeInline(gRFullSmall);
end
gRsPreview = gRFullEq;

% Stage 5 now occupies TWO tiles on the merged tab (bottom-left and
% bottom-right of the 2x2 grid): equalised synth on the left, equalised
% photo on the right.  Drawing them as separate axes (instead of one
% concatenated combo) lets MATLAB pick the right aspect for each.
axL = S.ax(5); cla(axL,'reset');
imshow(S.gS, 'Parent', axL);
title(axL, '5. Equalised synth (CLAHE)');
themeAxes(axL, S.theme);

axR = S.axR(5); cla(axR,'reset');
imshow(gRsPreview, 'Parent', axR);
title(axR, '5. Equalised photo (CLAHE)');
themeAxes(axR, S.theme);
if ~isempty(S.faceMask)
    hold(axR,'on');
    showMaskOverlayScaledInline(axR, S.faceMask, S.scl, 0, ...
        [0.2 1.0 0.2], 0.22);
    plotMaskBoundaryScaledInline(axR, S.faceMask, S.scl, 0, ...
        [0.2 1.0 0.2], 2.0);
    hold(axR,'off');
end
maskLines = {
    'ILLUMINATION EQUALISATION.  Cloud-scan colours are typically much dimmer than phone-photo colours, so we apply CLAHE to both images and the detectors compare gradient STRUCTURE rather than absolute brightness.  The photo is also downscaled to the synth''s width so feature scales line up.'
    ''
    sprintf('Synth (CLAHE):  %d x %d px     |     Photo (CLAHE, \\downarrow %.2fx):  %d x %d px', ...
        size(S.gS,2), size(S.gS,1), S.scl, size(gRsPreview,2), size(gRsPreview,1))
    };
if ~isempty(S.candidateFaceMask)
    maskLines = [maskLines; {
        ''
        sprintf('PROPOSED FACE MASK: %.1f%% of the photo (%d raw matches, %d homography inliers).', ...
            S.autoFaceInfo.areaPct, S.autoFaceInfo.nRaw, S.autoFaceInfo.nInliers)
        'Green overlay on the full photo preview shows the crop used for feature matching.'
        }];
else
    maskLines = [maskLines; {
        ''
        sprintf('PROPOSED FACE MASK: unavailable (%s).  The next tab can still draw one manually.', ...
            S.autoFaceInfo.status)
        }];
end
setDesc(S, 5, maskLines);
end

function S = stage_features(S, ~)
% Reuse the per-detector features autoFaceMaskInline already computed,
% provided the matching gray hasn't been changed by a face mask.  When
% a mask IS active, S.gRs differs from the gray autoFaceMaskInline saw,
% so we must redetect; same applies after refineAlignmentFromProjectionCrop
% (which sets a fresh mask).
useCached = isempty(S.faceMask) && isstruct(S.cachedFeatures) ...
    && isfield(S.cachedFeatures, 'valid') && S.cachedFeatures.valid;
if useCached
    c = S.cachedFeatures;
    prSIFT = c.prSIFT; psSIFT = c.psSIFT; fRS = c.fRS; fSS = c.fSS;
    prKAZE = c.prKAZE; psKAZE = c.psKAZE; fRK = c.fRK; fSK = c.fSK;
    prORB  = c.prORB;  psORB  = c.psORB;  fRO = c.fRO; fSO = c.fSO;
else
    gR = S.gRs; gS = S.gS;
    [prSIFT, psSIFT, fRS, fSS] = featPair(gR, gS, 'SIFT');
    [prKAZE, psKAZE, fRK, fSK] = featPair(gR, gS, 'KAZE');
    [prORB,  psORB,  fRO, fSO] = featPair(gR, gS, 'ORB');
end

S.feat.pr = [prSIFT; prKAZE; prORB];
S.feat.ps = [psSIFT; psKAZE; psORB];

ax = S.ax(6); cla(ax,'reset');
combo = makePair(S.gS, S.gRs);
imshow(combo,'Parent',ax); hold(ax,'on');
offset = size(S.gS,2) + 6;
plotPts(ax, fSS, [0 0],       'g.');
plotPts(ax, fSK, [0 0],       'y.');
plotPts(ax, fSO, [0 0],       'c.');
plotPts(ax, fRS, [offset 0],  'g.');
plotPts(ax, fRK, [offset 0],  'y.');
plotPts(ax, fRO, [offset 0],  'c.');
hold(ax,'off');
nR = safeCount(fRS)+safeCount(fRK)+safeCount(fRO);
nS = safeCount(fSS)+safeCount(fSK)+safeCount(fSO);
title(ax,'6. Detected feature keypoints');
themeAxes(ax, S.theme);
setDesc(S, 6, {
    'FEATURE DETECTION.  SIFT, KAZE and ORB are run independently on both images and the keypoint sets are unioned.  Each detector responds to different local structure, so the union is far more robust on synthetic ortho renders than any single detector alone.'
    ''
    sprintf('GREEN = SIFT   |   YELLOW = KAZE   |   CYAN = ORB     Keypoints  synth: %s     photo: %s', ...
        commaInt(nS), commaInt(nR))
    });
end

function S = stage_matches(S, ~)
S.matches.pr = S.feat.pr;
S.matches.ps = S.feat.ps;

ax = S.ax(7); cla(ax,'reset');
combo = makePair(S.gS, S.gRs);
imshow(combo,'Parent',ax); hold(ax,'on');
offset = size(S.gS,2) + 6;
nM = size(S.matches.pr,1);
showIdx = 1:max(1,floor(nM/800)):nM;
for ii = showIdx
    plot(ax, [S.matches.ps(ii,1), S.matches.pr(ii,1)+offset], ...
             [S.matches.ps(ii,2), S.matches.pr(ii,2)], ...
             '-', 'Color',[1 0.5 0 0.3], 'LineWidth',0.5);
end
plot(ax, S.matches.ps(:,1), S.matches.ps(:,2),'g+','MarkerSize',4);
plot(ax, S.matches.pr(:,1)+offset, S.matches.pr(:,2),'g+','MarkerSize',4);
hold(ax,'off');
title(ax,'7. All raw feature matches');
themeAxes(ax, S.theme);
setDesc(S, 7, {
    'RAW MATCHES.  Descriptors are matched between photo and synth for each detector independently, then the three sets are unioned.  Orange lines link matched pairs (a subset is drawn to keep the plot legible).  Many of these will be wrong — the next step uses RANSAC to filter for geometric consistency.'
    ''
    sprintf('Combined raw matches: %s    (showing %d lines)', commaInt(nM), numel(showIdx))
    });
end

function S = stage_solve(S, fig)
% Seed RNG so RANSAC inside estimateGeometricTransform2D and
% estworldpose is reproducible across runs.
try, rng(0, 'twister'); catch, end

% Manual-mode shortcut: skip every auto computation and open the
% photo<->synth picker directly.  The picker uses S.plane (from
% stage_planefit) and S.synth + S.ortho (from stage_ortho), both of
% which have already run by this point.
if isfield(S, 'manualMode') && S.manualMode
    % Tunnel-face crop step.  Try the auto detector to get a seed
    % mask; then open the brush editor so the user can confirm or
    % redraw.  The resulting mask is used by the picker to dim
    % non-face regions of the photo so the user only picks on the
    % tunnel face, not walls/roof/floor.  If the auto detector and
    % the user both produce nothing, the picker shows the full photo.
    setStatus(fig, 'Manual mode: opening tunnel-face crop editor...', ...
        [0.30 0.30 0.70]);
    drawnow;
    seedMask = [];
    try
        [seedMask, ~, ~] = autoFaceMaskInline( ...
            S.img, S.synth, S.synthMask, [], []);
    catch
    end
    if ~isempty(seedMask) && any(seedMask(:))
        S.candidateFaceMask = seedMask;
    end
    drawnMask = manualFaceMaskBrushInline(S);
    if ~isempty(drawnMask) && any(drawnMask(:))
        S.faceMask = drawnMask;
        setStatus(fig, sprintf(['Face mask set (%.1f%% of photo).  ', ...
            'Opening photo<->synth picker...'], ...
            100*nnz(S.faceMask)/numel(S.faceMask)), [0.30 0.30 0.70]);
    else
        S.faceMask = [];
        setStatus(fig, ['No face mask (full photo).  ', ...
            'Opening photo<->synth picker...'], [0.50 0.40 0.30]);
    end
    drawnow;
    needManual = true;
    reason = 'manual mode';
    [pose, inIdx, ok] = deal([], [], false); %#ok<ASGLU>
    imPts = zeros(0,2); wPts = zeros(0,3); %#ok<NASGU>
    S.inliers.pr = zeros(0,2); S.inliers.ps = zeros(0,2);
else
% Auto path: RANSAC over the unioned detector matches, then lift the
% surviving synth coords to 3-D via the analytic ortho map and solve
% PnP.  If too few inliers survive (or PnP fails), drop into the
% photo<->synth manual picker.
pr = S.matches.pr; ps = S.matches.ps;
try
    [~,inl] = estimateGeometricTransform2D(pr, ps, ...
        'projective','MaxNumTrials',8000,'MaxDistance',6, 'Confidence',99);
    pr = pr(inl,:); ps = ps(inl,:);
catch
    try
        [~,inl] = estimateGeometricTransform(pr, ps, ...
            'projective','MaxNumTrials',8000,'MaxDistance',6);
        pr = pr(inl,:); ps = ps(inl,:);
    catch
    end
end
S.inliers.pr = pr; S.inliers.ps = ps;
setStatus(fig, sprintf(['RANSAC kept %d / %d matches as inliers.  ', ...
    'Solving PnP...'], size(pr,1), size(S.matches.pr,1)), [0.1 0.1 0.5]);
drawnow;

uC = S.ortho.uLo + (ps(:,1) - 0.5)*S.ortho.dx;
vC = S.ortho.vLo + (ps(:,2) - 0.5)*S.ortho.dx;
planePts = S.plane.c + uC * S.plane.u + vC * S.plane.v;
try
    kdt = KDTreeSearcher(S.xyz);
    nnI = knnsearch(kdt, planePts);
catch
    nnI = zeros(size(planePts,1),1);
    for ii=1:size(planePts,1)
        d2 = sum((S.xyz - planePts(ii,:)).^2, 2);
        [~,nnI(ii)] = min(d2);
    end
end
wPts  = S.xyz(nnI,:);
imPts = pr / S.scl;

% --------------------------------------------------------------------
% AUTO PnP + multi-criteria sanity check.
%
% A weak PnP can succeed on a handful of accidentally-self-consistent
% bad matches and return a "pose" that re-colours random patches of the
% image onto the cloud.  Just checking inlier count is not enough.
% We REJECT the auto result (and fall through to the manual picker) if
% ANY of these fail:
%   * fewer than 15 inliers from RANSAC,
%   * reprojection RMS on the inliers > 15 px,
%   * recovered camera is unreasonably far from / inside the cloud,
%   * camera optical axis does not face the cloud centroid (dot > 0.3),
%   * fewer than 10 % of cloud points project into the photo.
% These thresholds are loose enough to keep any plausibly-correct pose
% and tight enough to drop the random-looking ones the user reported.
% --------------------------------------------------------------------
needManual = false;
reason = '';
[pose, inIdx, ok] = deal([], [], false);
if size(imPts, 1) < 6
    needManual = true;
    reason = sprintf('only %d matched pairs (need >=6)', size(imPts, 1));
else
    [pose, inIdx, ok] = solvePnPInline(imPts, wPts, S.intr);
    if ~ok || isempty(inIdx)
        needManual = true; reason = 'estworldpose failed';
    elseif numel(inIdx) < 15
        needManual = true;
        reason = sprintf('only %d PnP inliers (need >=15)', numel(inIdx));
    else
        ipIn = imPts(inIdx, :); wpIn = wPts(inIdx, :);
        try, pose = bundleAdjustmentMotion(wpIn, ipIn, pose, S.intr); catch, end
        [valid, vReason] = isPoseLikelyValidInline( ...
            pose, S.intr, S.K, S.xyz, S.W, S.H, ipIn, wpIn);
        if ~valid
            needManual = true;
            reason = vReason;
        end
    end
end
end   % end of auto-path else (manual-mode shortcut closed above)

if needManual
    % During the post-crop refinement pass, do NOT open another picker -
    % throw so refineAlignmentFromProjectionCrop reverts to the pose
    % the user already accepted in the first pass.
    if isfield(S, 'cropRefined') && S.cropRefined
        error('Refinement rejected (%s) - keeping pre-refinement pose.', reason);
    end
    if strcmp(reason, 'manual mode')
        setStatus(fig, ['Manual mode: opening photo<->synth picker...'], ...
            [0.30 0.30 0.70]);
    else
        setStatus(fig, sprintf(['Auto alignment rejected (%s) - opening ', ...
            'manual photo<->synth picker...'], reason), [0.7 0.3 0]);
    end
    drawnow;
    [imPts, wPts, cancelled] = manualPhotoSynthPickerInline(S);
    if cancelled || size(imPts,1) < 6
        error('Manual picker cancelled or fewer than 6 pairs picked (got %d).', size(imPts,1));
    end
    [pose, inIdx, ok] = solvePnPInline(imPts, wPts, S.intr);
    if ~ok || isempty(inIdx) || numel(inIdx) < min(6, size(imPts,1))
        % Trust the user-vetted picks: re-solve with a huge RANSAC
        % threshold so estworldpose accepts every pick.
        [pose, inIdx, ok] = solvePnPInlineLoose(imPts, wPts, S.intr);
    end
    if ~ok, error('PnP solver failed even on the manual picks.'); end
    ipIn = imPts(inIdx, :); wpIn = wPts(inIdx, :);
    try, pose = bundleAdjustmentMotion(wpIn, ipIn, pose, S.intr); catch, end
end

S.pose = pose;
[S.R_cw, S.t_cw] = poseToExtrinsicsInline(pose);
[uv,front] = projectPtsInline(wpIn, S.R_cw, S.t_cw, S.K);
e = hypot(uv(front,1)-ipIn(front,1), uv(front,2)-ipIn(front,2));
S.rms = sqrt(mean(e.^2));
S.imPts = ipIn; S.wPts = wpIn;
imPts = ipIn; wPts = wpIn; %#ok<NASGU>     % keep local names consistent

ax = S.ax(8); cla(ax,'reset');
imshowPhotoPreviewInline(ax, S.img, S.W, S.H, S.displayMaxImageWidth); hold(ax,'on');
[uvA,fr] = projectPtsInline(S.xyz, S.R_cw, S.t_cw, S.K);
inb = fr & uvA(:,1)>=1 & uvA(:,1)<=S.W & uvA(:,2)>=1 & uvA(:,2)<=S.H;
nA = nnz(inb);
idx = find(inb);
keep = idx(1:max(1,floor(nA/4e5)):end);
scatter(ax, uvA(keep,1), uvA(keep,2), 4, double(S.rgb(keep,:))/255, '.');
plot(ax, S.imPts(:,1), S.imPts(:,2), 'g+', 'MarkerSize',8, 'LineWidth',1.2);
[uvC,~] = projectPtsInline(S.wPts, S.R_cw, S.t_cw, S.K);
plot(ax, uvC(:,1), uvC(:,2), 'ro', 'MarkerSize',6, 'LineWidth',1);
nP = size(S.imPts,1);
showIdx = 1:max(1,floor(nP/200)):nP;
for ii = showIdx
    plot(ax, [S.imPts(ii,1) uvC(ii,1)], [S.imPts(ii,2) uvC(ii,2)], ...
        'y-', 'LineWidth',0.5);
end
hold(ax,'off');
title(ax,'8. RANSAC inliers + PnP reprojection');
themeAxes(ax, S.theme);
setDesc(S, 8, {
    'GEOMETRIC SOLVE.  Projective RANSAC filters the raw matches by fitting a homography (the face is approximately planar).  Surviving inliers are lifted to 3-D via the analytic ortho mapping; estworldpose then estimates the camera pose, refined by bundle adjustment.  The overlay shows the projected cloud over the photo.'
    ''
    'Coloured dots = projected cloud     Green +  = matched photo pixel     Red o  = reprojected world point     Yellow line = residual'
    ''
    sprintf('PnP inliers: %d     |     Reprojection RMS: %.2f px', nP, S.rms)
    });
end

function S = stage_crop(S, ~)
% Use the projected cloud to find the photo region that actually
% overlaps the scanned face, then show the original photo (with the
% overlap bbox drawn) next to the cropped overlap region.
% Density-based mask: a pixel is "face" if many cloud points project
% there. That filters out sparse outliers near the photo borders.
[uvA,fr] = projectPtsInline(S.xyz, S.R_cw, S.t_cw, S.K);
ix = round(uvA(:,1)); iy = round(uvA(:,2));
inb = fr & ix>=1 & ix<=S.W & iy>=1 & iy<=S.H;

if ~any(inb)
    cropBox = [1 1 S.W S.H];
    mask = false(S.H, S.W);
else
    % Bin projections into a coarse grid. Face areas accumulate hundreds
    % of projected points per cell; scattered outliers along the edges
    % accumulate only a few - so a percentile threshold on nonzero
    % cells isolates the face cleanly.
    binSize = 40;
    nXbin = ceil(S.W/binSize);
    nYbin = ceil(S.H/binSize);
    binX = ceil(ix(inb)/binSize);
    binY = ceil(iy(inb)/binSize);
    binX = min(max(binX,1), nXbin);
    binY = min(max(binY,1), nYbin);
    binCounts = accumarray([binY binX], 1, [nYbin nXbin]);

    nz = binCounts(binCounts > 0);
    if isempty(nz)
        binMask = false(nYbin, nXbin);
    else
        % Keep a deliberately generous density support.  The crop must
        % never cut into the tunnel face, so tolerate more wall/roof/floor
        % margin instead of using a tight threshold near the rim.
        binMask = binCounts >= max(3, 0.15 * max(nz));
    end

    % keep only the largest connected component of bins (drops islands)
    try
        cc = bwconncomp(binMask);
        if cc.NumObjects > 1
            sz = cellfun(@numel, cc.PixelIdxList);
            [~,bigK] = max(sz);
            nm = false(size(binMask));
            nm(cc.PixelIdxList{bigK}) = true;
            binMask = nm;
        end
    catch
    end

    % Upsample the coarse projection support, but keep its real outline.
    % Indentations in the projection overlay are useful diagnostics, so
    % only fill holes/drop islands instead of smoothing the outer edge.
    mask = smoothProjectionCropMaskInline(binMask, S.H, S.W, binSize);

    if ~any(binMask(:))
        cropBox = [1 1 S.W S.H];
    else
        [byy, bxx] = find(binMask);
        pad = max(80, round(0.04 * max(S.W, S.H)));
        x0 = max(1,   (min(bxx)-1)*binSize + 1 - pad);
        y0 = max(1,   (min(byy)-1)*binSize + 1 - pad);
        x1 = min(S.W, max(bxx)*binSize + pad);
        y1 = min(S.H, max(byy)*binSize + pad);
        cropBox = [x0 y0 (x1-x0+1) (y1-y0+1)];
    end
end
maskProj = logical(mask);
if any(maskProj(:))
    cropBox = bboxFromVisibleOverlayInline(maskProj, S.W, S.H, ...
        max(30, round(0.015 * max(S.W,S.H))));
end
S.cropBox = cropBox;
S.projCropMask = maskProj;
S.selectedCropMask = maskProj;
S.selectedCropSource = 'projection';
cropped = S.img(cropBox(2):cropBox(2)+cropBox(4)-1, ...
                cropBox(1):cropBox(1)+cropBox(3)-1, :);
croppedMask = maskProj(cropBox(2):cropBox(2)+cropBox(4)-1, ...
                       cropBox(1):cropBox(1)+cropBox(3)-1);
S = ensureSplitAxes(S, 9);

drawStage9CropViews(S, cropped, croppedMask);
end

function drawStage9CropViews(S, cropped, croppedMask)
maskProj = S.projCropMask;
cropBox = S.cropBox;

% Left: original photo with the projection-derived crop overlay.
axL = S.ax(9); cla(axL,'reset');
overlay = makePhotoOverlayPreviewInline(S.img, S.W, S.H, ...
    {maskProj}, {[0 255 0]}, 0.28, ...
    S.displayMaxImageWidth);
imshowPhotoPreviewInline(axL, overlay, S.W, S.H, Inf); hold(axL,'on');
plotMaskBoundaryInline(axL, maskProj, [0.1 1.0 0.25], 2.0);
xlim(axL, [1 S.W]);
ylim(axL, [1 S.H]);
hold(axL,'off');
title(axL, 'ORIGINAL photo (green=projection crop)');
themeAxes(axL, S.theme);

% Other side: original pixels clipped to the projection overlay.
axR = S.axR(9); cla(axR,'reset');
imshowMaskedCropPreviewInline(axR, cropped, croppedMask, cropBox, S.displayMaxImageWidth);
title(axR, 'CROPPED to the projection crop region');
themeAxes(axR, S.theme);

pctArea = 100 * cropBox(3) * cropBox(4) / (S.W * S.H);
setDesc(S, 9, {
    'OVERLAP REGION.  After PnP the cloud is projected onto the photo and the pixel projection densities are converted into a filled support mask.  The green overlay outlines this projection-derived crop region, which is also reused for the automatic refinement feature-matching pass.  Left: full photo with overlay.  Right: photo clipped to the overlay.'
    ''
    sprintf('Photo: %d x %d     |     Crop: %d x %d  (%.1f%% of photo)', ...
        S.W, S.H, cropBox(3), cropBox(4), pctArea)
    });
end

function tf = shouldRefineFromProjectionCrop(S)
tf = false;
if S.cropRefined
    return;
end
% In manual mode the pose already came from user-vetted picks; an
% automatic refinement pass would re-open the picker (because the auto
% path is disabled), which is not what the user wants.
if isfield(S, 'manualMode') && S.manualMode
    return;
end
if isempty(S.projCropMask) || ~any(S.projCropMask(:))
    return;
end
if isempty(S.pose) || isempty(S.R_cw) || isempty(S.t_cw)
    return;
end
areaPct = 100 * nnz(S.projCropMask) / numel(S.projCropMask);
tf = areaPct > 3 && areaPct < 95;
end

function S = refineAlignmentFromProjectionCrop(S, fig)
% One automatic second pass: use the post-solve projection crop mask to
% suppress non-face image content, then redo feature matching and PnP.
backup = S;
backup.cropRefined = true;
try
    setStatus(fig, 'Refining alignment once using the projection crop mask...', [0.1 0.1 0.5]);
    drawnow;
    S.cropRefined = true;
    S = prepareMatchingFromMaskInline(S, S.projCropMask);
    S = stage_features(S, fig);
    S = stage_matches(S, fig);
    S = stage_solve(S, fig);
    S.cropRefined = true;
    setStatus(fig, sprintf('Crop-refined alignment complete. RMS = %.2f px.', S.rms), [0 0.4 0]);
catch ME
    S = backup;
    setStatus(fig, ['Crop-refinement skipped: ' ME.message], [0.7 0.3 0]);
end
drawnow;
end

function S = prepareMatchingFromMaskInline(S, mask)
S.faceMask = logical(mask);
S.imgForMatch = [];
if isempty(S.scl) || ~isfinite(S.scl) || S.scl <= 0
    S.scl = S.HsynW(2) / S.W;
end
g = downsamplePhotoGrayInline(S.img, S.scl);
maskSmall = imresize(S.faceMask, size(g), 'nearest') > 0;
g(~maskSmall) = 0;
S.gRs = adaptiveEqualizeInline(g);
if isempty(S.gS)
    S.gS = adaptiveEqualizeInline(rgb2gray(S.synth));
end
end

function S = stage_recolor(S, fig)
% If the plane normal isn't already set (e.g. defensive against earlier
% stage failures), compute it now via SVD and disambiguate the sign
% using the recovered camera position (PnP put the camera on the FRONT
% side of the face by construction) so the view direction matches the
% actual face orientation.
if isempty(S.plane.n) || numel(S.plane.n) ~= 3
    c0 = mean(S.xyz, 1);
    [~,~,V0] = svd(S.xyz - c0, "econ");
    u0 = V0(:,1).'/norm(V0(:,1));
    v0 = V0(:,2).'/norm(V0(:,2));
    n0 = V0(:,3).'/norm(V0(:,3));
    [u0, v0, n0] = orientPlaneAxesInline(u0, v0, n0);
    if ~isempty(S.R_cw) && ~isempty(S.t_cw)
        try
            Cw = -S.R_cw.' * S.t_cw(:);
            if dot(n0(:), Cw(:) - c0(:)) < 0
                n0 = -n0;
                u0 = -u0;          % keep {u0, v0, n0} right-handed
            end
        catch
        end
    end
    S.plane.c = c0; S.plane.u = u0; S.plane.v = v0; S.plane.n = n0;
end
Xall = double(S.pc.Location);
[uvA,fr] = projectPtsInline(Xall, S.R_cw, S.t_cw, S.K);
ix = round(uvA(:,1)); iy = round(uvA(:,2));
inb = fr & ix>=1 & ix<=S.W & iy>=1 & iy<=S.H;
col = S.pc.Color;
if isempty(col), col = uint8(repmat(200,size(Xall,1),3)); end
lin = sub2ind([S.H S.W], iy(inb), ix(inb));
Rc=S.img(:,:,1); Gc=S.img(:,:,2); Bc=S.img(:,:,3);
col(inb,1)=Rc(lin); col(inb,2)=Gc(lin); col(inb,3)=Bc(lin);
S.coloredCloud = pointCloud(Xall,'Color',col);

% Build a pointCloud for the ORIGINAL colours (so we can hand both
% sides off to pcshow, which renders solid surfaces much better than
% scatter3 - especially for ~1.9M-point clouds).
origCol = S.pc.Color;
if isempty(origCol), origCol = uint8(repmat(200,size(Xall,1),3)); end
pcOrig = pointCloud(Xall, 'Color', origCol);

% pcshow uses native point rendering; MarkerSize bumped up so the
% face looks solid instead of pixel-sparse.  Background dark to make
% the photo colours pop.
markerSize = 30;
bg = [0.05 0.05 0.07];
initialView = initialCloudViewDirectionInline(S, Xall);

% Display-only downsampling: pcshow on a 1.9M-point cloud spends 1-3 s
% per panel just on the first render.  We hand pcshow a random subset
% (cap = S.displayMaxCloudPoints) but keep the FULL coloured cloud on
% S.coloredCloud for save/return - so no data is lost, only rendering
% load.  MarkerSize stays large so the face still looks solid.
N        = size(Xall, 1);
cap      = S.displayMaxCloudPoints;
if isfinite(cap) && N > cap
    ridx       = randperm(N, cap);
    pcOrigDisp = pointCloud(Xall(ridx,:), 'Color', origCol(ridx,:));
    pcReDisp   = pointCloud(Xall(ridx,:), 'Color', col(ridx,:));
    dispNote   = sprintf('Display subsample: %s / %s points  (full cloud kept on S.coloredCloud)', ...
        commaInt(numel(ridx)), commaInt(N));
else
    pcOrigDisp = pcOrig;
    pcReDisp   = S.coloredCloud;
    dispNote   = sprintf('Displaying all %s points', commaInt(N));
end

% Left: original cloud
axL = S.ax(10); cla(axL,'reset');
pcshow(pcOrigDisp, 'Parent', axL, ...
    'MarkerSize', markerSize, 'BackgroundColor', bg, ...
    'VerticalAxis','Z', 'VerticalAxisDir','Up');
xlabel(axL,'X'); ylabel(axL,'Y'); zlabel(axL,'Z');
title(axL, 'ORIGINAL scanner colours', 'Color', [1 1 1]);
view(axL, initialView); axis(axL,'vis3d');

% Right: photo-recoloured cloud
axR = S.axR(10); cla(axR,'reset');
pcshow(pcReDisp, 'Parent', axR, ...
    'MarkerSize', markerSize, 'BackgroundColor', bg, ...
    'VerticalAxis','Z', 'VerticalAxisDir','Up');
xlabel(axR,'X'); ylabel(axR,'Y'); zlabel(axR,'Z');
title(axR, 'PHOTO-RECOLOURED', 'Color', [1 1 1]);
view(axR, initialView); axis(axR,'vis3d');

setDesc(S, 10, {
    'FINAL RESULT.  Each cloud point is projected to the photo and sampled from that pixel, giving the cloud the photo''s colour and texture.  LEFT shows the original scanner colours, RIGHT shows the photo-recoloured cloud.  Camera, axis limits and view angle are LINKED  -  drag inside either side to rotate or zoom and both follow in lock-step.'
    ''
    sprintf('Points recoloured: %s / %s  (in-bounds projections)     |     Reprojection RMS = %.2f px on %d PnP inliers', ...
        commaInt(nnz(inb)), commaInt(size(Xall,1)), S.rms, size(S.imPts,1))
    dispNote
    });

% pcshow may reset camera properties on its first call - establish
% the linkprop AFTER both axes are fully set up.
try, delete(S.viewLink); catch, end
S.viewLink = linkprop([axL, axR], ...
    {'CameraPosition','CameraTarget','CameraUpVector','CameraViewAngle', ...
     'XLim','YLim','ZLim'});
setStatus(fig, sprintf(['Rotation/zoom of both 3-D views is LINKED. ', ...
    'Use the rotate or zoom tool on either side; both follow.   ', ...
    'RMS = %.2f px on %d inliers.'], S.rms, size(S.imPts,1)), [0 0.4 0]);
end

% =====================================================================
%   SMALL UI / FEATURE HELPERS
% =====================================================================
function combo = makePair(left, right)
h1 = size(left,1); h2 = size(right,1); h = max(h1,h2);
pad1 = uint8(zeros(h - h1, size(left,2)));
pad2 = uint8(zeros(h - h2, size(right,2)));
gap  = uint8(255*ones(h, 6));
combo = [[left; pad1], gap, [right; pad2]];
end

function imshowPhotoPreviewInline(ax, img, W, H, maxW)
% Display-only downsampling for large photos.  Axes coordinates remain
% in original pixel units, so overlays/projected points still line up.
if nargin < 5 || isempty(maxW), maxW = Inf; end
imgDisp = img;
if isfinite(maxW) && W > maxW
    try
        imgDisp = imresize(img, maxW / W);
    catch
    end
end
imshow(imgDisp, 'Parent', ax, 'XData', [1 W], 'YData', [1 H]);
axis(ax, 'image');
end

function g = downsamplePhotoGrayInline(img, scale)
% For matching/equalization, resize the RGB image first and convert to
% grayscale after.  This avoids building a full-resolution grayscale copy.
try
    g = rgb2gray(imresize(img, scale));
catch
    g = imresize(rgb2gray(img), scale);
end
end

function imshowMaskedCropPreviewInline(ax, img, mask, cropBox, maxW)
% Show original pixels clipped to the selected crop overlay.  Pixels
% outside the selected overlay are transparent, so this is not just a
% rectangular bounding-box preview.
if nargin < 5 || isempty(maxW), maxW = Inf; end
w = cropBox(3);
h = cropBox(4);
imgDisp = img;
maskDisp = mask;
if isfinite(maxW) && w > maxW
    try
        s = maxW / w;
        imgDisp = imresize(img, s);
        maskDisp = imresize(mask, [size(imgDisp,1) size(imgDisp,2)], 'nearest') > 0;
    catch
    end
end
x0 = cropBox(1);
y0 = cropBox(2);
imshow(imgDisp, 'Parent', ax, ...
    'XData', [x0, x0 + w - 1], 'YData', [y0, y0 + h - 1]);
try
    set(findobj(ax, 'Type','image'), 'AlphaData', double(maskDisp));
catch
end
axis(ax, 'image');
set(ax, 'Color', [0.10 0.11 0.14]);
end

function overlay = makePhotoOverlayPreviewInline(img, W, H, masks, colors, alphas, maxW)
% Build tinted previews at display resolution only.  This avoids copying
% and blending huge full-resolution RGB arrays just to paint the GUI.
if nargin < 7 || isempty(maxW), maxW = Inf; end
scale = 1;
if isfinite(maxW) && W > maxW
    scale = maxW / W;
end
try
    overlay = imresize(img, scale);
catch
    overlay = img;
    scale = 1;
end
hD = size(overlay,1);
wD = size(overlay,2);
for ii = 1:numel(masks)
    mask = masks{ii};
    if isempty(mask) || ~any(mask(:)), continue; end
    try
        if scale ~= 1 || size(mask,1) ~= hD || size(mask,2) ~= wD
            mask = imresize(mask, [hD wD], 'nearest') > 0;
        end
        alpha = alphas(ii);
        color = colors{ii};
        for ch = 1:3
            C = overlay(:,:,ch);
            C(mask) = uint8((1-alpha) * double(C(mask)) + alpha * color(ch));
            overlay(:,:,ch) = C;
        end
    catch
    end
end
% Preserve the intended full-image aspect if imresize rounded a pixel.
if H > 0 && abs((hD / wD) - (H / W)) > 0.01
    try, overlay = imresize(overlay, [round(wD * H / W), wD]); catch, end
end
end

function mask = smoothProjectionCropMaskInline(binMask, H, W, binSize)
mask = false(H,W);
if isempty(binMask) || ~any(binMask(:)), return; end
try
    mask = imresize(binMask, [H W], 'nearest') > 0;
catch
    mask = logical(kron(binMask, true(binSize)));
    mask = mask(1:min(end,H), 1:min(end,W));
    if size(mask,1) < H, mask(end+1:H,:) = false; end
    if size(mask,2) < W, mask(:,end+1:W) = false; end
end
try
    mask = imfill(mask, 'holes');
    cc = bwconncomp(mask);
    if cc.NumObjects > 1
        sz = cellfun(@numel, cc.PixelIdxList);
        [~,bigK] = max(sz);
        keep = false(size(mask));
        keep(cc.PixelIdxList{bigK}) = true;
        mask = keep;
    end
catch
end
end

function mask = smoothCropMaskInline(mask, H, W)
if isempty(mask) || ~any(mask(:))
    mask = false(H,W);
    return;
end
mask = logical(mask);
try
    rClose = max(12, round(min(H,W)/65));
    rOpen = max(4, round(min(H,W)/220));
    mask = imclose(mask, strel('disk', rClose));
    mask = imfill(mask, 'holes');
    mask = imopen(mask, strel('disk', rOpen));
    mask = imclose(mask, strel('disk', max(6, round(rClose/2))));
    mask = imfill(mask, 'holes');
catch
end
try
    cc = bwconncomp(mask);
    if cc.NumObjects > 1
        sz = cellfun(@numel, cc.PixelIdxList);
        [~,bigK] = max(sz);
        keep = false(size(mask));
        keep(cc.PixelIdxList{bigK}) = true;
        mask = keep;
    end
catch
end
mask = smoothOuterBoundaryInline(mask, H, W, max(11, round(min(H,W)/140)));
end

function mask = smoothOuterBoundaryInline(mask, H, W, win)
% Convert the largest mask boundary to a smoothed polygon and refill it.
% This removes pixel stair-steps from the displayed crop outline while
% keeping the mask filled and hole-free.
if isempty(mask) || ~any(mask(:)), return; end
try
    Bd = bwboundaries(mask, 'noholes');
    if isempty(Bd), return; end
    lens = cellfun(@(b) size(b,1), Bd);
    [~, idx] = max(lens);
    b = Bd{idx};
    if size(b,1) < 20, return; end
    if mod(win,2) == 0, win = win + 1; end
    win = min(win, 2*floor((size(b,1)-1)/2)+1);
    ker = ones(win,1) / win;
    pad = floor(win/2);
    yy = b(:,1); xx = b(:,2);
    yyPad = [yy(end-pad+1:end); yy; yy(1:pad)];
    xxPad = [xx(end-pad+1:end); xx; xx(1:pad)];
    yyS = conv(yyPad, ker, 'same');
    xxS = conv(xxPad, ker, 'same');
    yyS = yyS(pad+1:end-pad);
    xxS = xxS(pad+1:end-pad);
    try
        tol = max(1.5, min(H,W)/900);
        p = reducepoly([xxS yyS], tol);
        xxS = p(:,1); yyS = p(:,2);
    catch
    end
    xxS = min(max(xxS,1), W);
    yyS = min(max(yyS,1), H);
    sm = poly2mask(xxS, yyS, H, W);
    sm = imfill(sm, 'holes');
    if nnz(sm) > 0
        mask = sm;
    end
catch
end
end

function bbox = bboxFromMaskInline(mask, W, H, pad)
if isempty(mask) || ~any(mask(:))
    bbox = [1 1 W H];
    return;
end
[yy, xx] = find(mask);
x0 = max(1, min(xx) - pad);
y0 = max(1, min(yy) - pad);
x1 = min(W, max(xx) + pad);
y1 = min(H, max(yy) + pad);
bbox = [x0 y0 (x1-x0+1) (y1-y0+1)];
end

function bbox = bboxFromVisibleOverlayInline(mask, W, H, pad)
% Match the crop to the selected overlay that is actually drawn.  This
% ignores tiny/hidden mask fragments that can otherwise make the crop
% look like the full original photo.
if isempty(mask) || ~any(mask(:))
    bbox = [1 1 W H];
    return;
end
try
    Bd = bwboundaries(mask, 'noholes');
    if ~isempty(Bd)
        lens = cellfun(@(b) size(b,1), Bd);
        [~, idx] = max(lens);
        b = Bd{idx};
        yy = b(:,1);
        xx = b(:,2);
        x0 = max(1, floor(min(xx)) - pad);
        y0 = max(1, floor(min(yy)) - pad);
        x1 = min(W, ceil(max(xx)) + pad);
        y1 = min(H, ceil(max(yy)) + pad);
        bbox = [x0 y0 (x1-x0+1) (y1-y0+1)];
        return;
    end
catch
end
bbox = bboxFromMaskInline(mask, W, H, pad);
end

function plotMaskBoundaryInline(ax, mask, color, width)
if isempty(mask) || ~any(mask(:)), return; end
try
    Bd = bwboundaries(mask, 'noholes');
    for k = 1:numel(Bd)
        plot(ax, Bd{k}(:,2), Bd{k}(:,1), 'Color', color, 'LineWidth', width);
    end
catch
end
end

function plotMaskBoundaryScaledInline(ax, mask, scl, xOffset, color, width)
if isempty(mask) || ~any(mask(:)), return; end
try
    Bd = bwboundaries(mask, 'noholes');
    for k = 1:numel(Bd)
        x = Bd{k}(:,2) * scl + xOffset;
        y = Bd{k}(:,1) * scl;
        plot(ax, x, y, 'Color', color, 'LineWidth', width);
    end
catch
end
end

function showMaskOverlayScaledInline(ax, mask, scl, xOffset, color, alpha)
if isempty(mask) || ~any(mask(:)), return; end
try
    m = imresize(mask, scl, 'nearest');
    h = size(m,1);
    w = size(m,2);
    tint = zeros(h, w, 3);
    tint(:,:,1) = color(1);
    tint(:,:,2) = color(2);
    tint(:,:,3) = color(3);
    image(ax, [xOffset + 1, xOffset + w], [1, h], tint, ...
        'AlphaData', alpha * double(m));
catch
end
end

function viewDir = initialCloudViewDirectionInline(S, X)
% Front view of the face for the final 3-D preview.
%
% MATLAB's view([x y z]) expects a vector pointing FROM the camera
% target TO the viewer position - so to put the viewer where the photo
% camera was, we need (Cw - tgt), not (tgt - Cw).  The previous version
% had this sign reversed, which is why the preview always opened from
% behind the face.
%
% The recovered camera pose is the most reliable front cue: PnP put
% the camera where the photo was actually taken, which is by definition
% the front of the face.  S.plane.n is used only as a fallback - and in
% auto/autoThenManual modes chooseFrontOrientation has already resolved
% its sign by matching against the photo (which is the same disambiguation
% the synth ortho render uses).
viewDir = [];
try
    if ~isempty(S.R_cw) && ~isempty(S.t_cw)
        Cw  = -S.R_cw.' * S.t_cw(:);
        tgt = S.plane.c(:);
        if isempty(tgt) || any(~isfinite(tgt))
            tgt = mean(X, 1).';
        end
        viewDir = (Cw - tgt).';
    end
catch
    viewDir = [];
end
if isempty(viewDir) || numel(viewDir) ~= 3 || any(~isfinite(viewDir)) || norm(viewDir) < eps
    % Fallback: front-facing plane normal from chooseFrontOrientation.
    viewDir = S.plane.n;
end
if isempty(viewDir) || numel(viewDir) ~= 3 || any(~isfinite(viewDir)) || norm(viewDir) < eps
    viewDir = [1 1 1];
end
viewDir = viewDir ./ norm(viewDir);
end

function [mask, info, featCache] = autoFaceMaskInline(img, synth, synthMask, gREqCached, gSEqCached)
mask = [];
info = struct('status','not attempted','nRaw',0,'nInliers',0,'areaPct',0);
featCache = struct('valid', false);
if nargin < 4, gREqCached = []; end
if nargin < 5, gSEqCached = []; end
if isempty(img) || isempty(synth) || isempty(synthMask) || ~any(synthMask(:))
    info.status = 'missing synthetic face mask';
    return;
end
try
    H = size(img,1); W = size(img,2);
    scl = size(synth,2) / W;
    % Reuse the cached CLAHE'd photo gray when it was built at this same
    % scale; otherwise compute fresh.  Saves a full-photo equalisation.
    if ~isempty(gREqCached) && size(gREqCached,2) == size(synth,2)
        gR = gREqCached;
    else
        gR = adaptiveEqualizeInline(downsamplePhotoGrayInline(img, scl));
    end
    % Same for the synth: stage_equalize already computes the CLAHE'd
    % synth gray for the matching pair, so accept it as an override.
    if ~isempty(gSEqCached) && isequal(size(gSEqCached), [size(synth,1), size(synth,2)])
        gS = gSEqCached;
    else
        gS = adaptiveEqualizeInline(rgb2gray(synth));
    end

    [prSIFT, psSIFT, fRS, fSS] = featPair(gR, gS, 'SIFT');
    [prKAZE, psKAZE, fRK, fSK] = featPair(gR, gS, 'KAZE');
    [prORB,  psORB,  fRO, fSO] = featPair(gR, gS, 'ORB');
    pr = [prSIFT; prKAZE; prORB];
    ps = [psSIFT; psKAZE; psORB];
    % Per-detector features describe the UNMASKED photo at the synth
    % scale - stage_features can reuse them whenever no face mask gets
    % applied to S.gRs (the matching gray would be identical to gR here).
    featCache = struct('valid', true, ...
        'prSIFT', prSIFT, 'psSIFT', psSIFT, 'fRS', fRS, 'fSS', fSS, ...
        'prKAZE', prKAZE, 'psKAZE', psKAZE, 'fRK', fRK, 'fSK', fSK, ...
        'prORB',  prORB,  'psORB',  psORB,  'fRO', fRO, 'fSO', fSO);
    info.nRaw = size(pr,1);
    if info.nRaw < 12
        info.status = 'too few full-photo matches';
        return;
    end

    prOrig = pr / scl;
    try
        [tform, inl] = estimateGeometricTransform2D(ps, prOrig, ...
            'projective','MaxNumTrials',10000,'MaxDistance',18, ...
            'Confidence',99);
    catch
        [tform, inl] = estimateGeometricTransform(ps, prOrig, ...
            'projective','MaxNumTrials',10000,'MaxDistance',18);
    end
    info.nInliers = nnz(inl);
    if info.nInliers < 20
        info.status = 'too few homography inliers';
        return;
    end
    psIn = ps(inl,:);
    prIn = prOrig(inl,:);
    [tx, ty] = transformPointsForward(tform, psIn(:,1), psIn(:,2));
    info.medianErr = median(hypot(tx - prIn(:,1), ty - prIn(:,2)));
    if info.medianErr > 18
        info.status = sprintf('high homography residual %.1f px', info.medianErr);
        return;
    end

    Rout = imref2d([H W]);
    synthForWarp = logical(synthMask);
    try
        synthForWarp = imdilate(synthForWarp, ...
            strel('disk', max(8, round(min(size(synthForWarp))/35))));
    catch
    end
    try
        warped = imwarp(synthForWarp, tform, 'nearest', 'OutputView', Rout);
    catch
        warped = imwarp(synthForWarp, tform, 'OutputView', Rout);
        warped = warped > 0.5;
    end

    % CRITICAL: run the geometric sanity checks on the RAW warped mask
    % (BEFORE cleanFaceMaskInline).  The cleanup step runs imdilate with
    % radius ~min(H,W)/55, which is large enough to fatten a thin
    % diagonal wedge into a chunky blob that would otherwise pass an
    % aspect-ratio check.  Doing the checks first means we reject the
    % underlying degenerate-homography mask before any morphology hides
    % its shape.
    warpedRaw = logical(warped);
    if ~any(warpedRaw(:))
        mask = [];
        info.status = 'warp produced an empty mask';
        return;
    end
    info.areaPctRaw = 100 * nnz(warpedRaw) / numel(warpedRaw);
    if info.areaPctRaw < 4 || info.areaPctRaw > 90
        mask = [];
        info.status = sprintf('unreasonable raw mask area %.1f%%', info.areaPctRaw);
        return;
    end

    % --- Geometric sanity on the RAW warped mask -------------------
    % A tunnel face is roughly equiaxed and solid.  A degenerate
    % homography typically produces a long thin wedge or a scattered
    % blob.  Reject masks that are either ELONGATED (bounding-box
    % aspect ratio >3:1), SPARSE (less than half-filled w.r.t. their
    % convex hull), or have an aspect that differs dramatically from
    % the source synth mask (a true projection preserves the rough
    % shape up to perspective foreshortening).
    [yIdx, xIdx] = find(warpedRaw);
    bbW = max(xIdx) - min(xIdx) + 1;
    bbH = max(yIdx) - min(yIdx) + 1;
    aspectMask = max(bbW, bbH) / max(1, min(bbW, bbH));
    info.aspect = aspectMask;
    if aspectMask > 3.0
        mask = [];
        info.status = sprintf('mask too elongated (aspect %.1f:1)', aspectMask);
        return;
    end
    try
        ch = bwconvhull(warpedRaw);
        solidity = nnz(warpedRaw) / max(1, nnz(ch));
        info.solidity = solidity;
        if solidity < 0.50
            mask = [];
            info.status = sprintf('mask too sparse (solidity %.2f)', solidity);
            return;
        end
    catch
    end
    try
        [sy, sx] = find(synthMask);
        if ~isempty(sy)
            sBW = max(sx) - min(sx) + 1;
            sBH = max(sy) - min(sy) + 1;
            aspectSynth = max(sBW, sBH) / max(1, min(sBW, sBH));
            info.aspectSynth = aspectSynth;
            ratioMismatch = max(aspectMask, aspectSynth) ...
                          / max(1, min(aspectMask, aspectSynth));
            info.aspectRatioMismatch = ratioMismatch;
            if ratioMismatch > 2.5
                mask = [];
                info.status = sprintf( ...
                    'mask aspect %.1f vs synth aspect %.1f (ratio %.1f>2.5)', ...
                    aspectMask, aspectSynth, ratioMismatch);
                return;
            end
        end
    catch
    end

    % Only NOW do the morphological cleanup (fill holes, close gaps,
    % dilate to give the matcher some margin around the face).
    mask = cleanFaceMaskInline(warpedRaw, H, W);
    info.areaPct = 100 * nnz(mask) / numel(mask);
    if info.areaPct < 8 || info.areaPct > 85
        mask = [];
        info.status = sprintf('cleaned mask has unreasonable area %.1f%%', info.areaPct);
        return;
    end
    info.status = 'ok';
catch ME
    mask = [];
    info.status = ME.message;
end
end

function S = applyFaceMaskForMatching(S, mask)
S = prepareMatchingFromMaskInline(S, mask);
end

function showFaceCropReview(S, ax, mask)
cla(ax,'reset');
if isempty(mask) || ~any(mask(:))
    imshow(S.img, 'Parent', ax);
    title(ax, '6. Face crop review - no automatic proposal');
    setDesc(S, 6, {
        'FACE CROP REVIEW.  No reliable automatic crop was found.  Click Edit polygon to draw the tunnel face manually, or Skip mask to continue with the full photo.'
        });
    return;
end

overlay = S.img;
R = overlay(:,:,1); G = overlay(:,:,2); B = overlay(:,:,3);
alpha = 0.30;
R(mask) = uint8((1-alpha)*double(R(mask)));
G(mask) = uint8((1-alpha)*double(G(mask)) + alpha*255);
B(mask) = uint8((1-alpha)*double(B(mask)));
overlay(:,:,1) = R; overlay(:,:,2) = G; overlay(:,:,3) = B;
imshow(overlay,'Parent',ax); hold(ax,'on');
try
    Bd = bwboundaries(mask, 'noholes');
    for k = 1:numel(Bd)
        plot(ax, Bd{k}(:,2), Bd{k}(:,1), 'Color',[0.2 1 0.2], 'LineWidth',2);
    end
catch
end
hold(ax,'off');
title(ax, '6. Face crop review - green = proposed face');
setDesc(S, 6, {
    'FACE CROP REVIEW.  Green shows the proposed tunnel-face region.  Accept it only if the whole face is included and wall/roof/floor spillover is acceptable; use Edit polygon if the left/right boundary needs correction.'
    ''
    sprintf('Face pixels proposed: %s of %s  (%.1f%% of the photo)     |     Full-photo matches: %d     |     Homography inliers: %d', ...
        commaInt(nnz(mask)), commaInt(numel(mask)), 100*nnz(mask)/numel(mask), ...
        S.autoFaceInfo.nRaw, S.autoFaceInfo.nInliers)
    });
end

function mask = manualFaceMaskBrushInline(S)
%MANUALFACEMASKBRUSHINLINE  Modal CLOSED-REGION mask editor.
%
%   Opened as a fallback when the automatic face-mask detector fails.
%   Returns a logical mask in S.img pixel coordinates, or [] if the
%   user cancels.
%
%   Workflow: click-and-DRAG to outline a region; on release the
%   outline is closed and its INTERIOR is filled into the mask.  The
%   tool you picked stays active so you can keep adding more regions
%   without re-clicking - the editor only returns to the caller when
%   Finish (or Cancel) is pressed.
%
%   Tools:
%     +Add region     each freehand outline is OR'd into the mask.
%     -Remove region  each freehand outline is cleared from the mask.
%     Expand border   imdilate the mask by ~min(H,W)/120 px (one step).
%     Shrink border   imerode  the mask by the same radius (one step).
%     Undo            revert the last edit (up to 30 deep).
%     Reset           clear the mask.
%     Finish          accept the current mask and return.
%     Cancel          discard everything and return [].
%
%   Implementation note: the editor uses CUSTOM figure-level mouse
%   handlers (WindowButtonDownFcn / WindowButtonMotionFcn /
%   WindowButtonUpFcn) - NOT images.roi.Freehand + draw().  The
%   blocking draw() call did not reliably return when the active ROI
%   was deleted mid-wait by a tool-switch button click, leaving the
%   loop stuck and breaking subsequent strokes.  With direct mouse
%   handlers there is no blocking call to interrupt: tool buttons
%   simply set regionMode, and the next stroke (whether Add or
%   Remove) reads regionMode at mouse-down time.
mask = false(S.H, S.W);
if isfield(S, 'candidateFaceMask') && ~isempty(S.candidateFaceMask) ...
        && any(S.candidateFaceMask(:))
    mask = logical(S.candidateFaceMask);
end

T = S.theme;
BG = T.bg;  PANEL = T.panel;  TXT = T.text;  INPUT = T.input;
SUCCESS = T.success;  WARN = T.warn;
ADD_COL    = [0.12 0.38 0.85];
REMOVE_COL = [1.00 0.20 0.20];

fig = figure('Name','Manual tunnel-face region editor', ...
    'NumberTitle','off','Color', BG, ...
    'WindowStyle','normal','MenuBar','none','ToolBar','figure', ...
    'CloseRequestFcn',@(~,~)finishCb(true));
try, fig.WindowState = 'maximized'; catch, end

ax = axes('Parent',fig, 'Units','normalized', ...
    'Position',[0.02 0.20 0.96 0.76], 'Box','on');
hImg = imshow(S.img, 'Parent', ax);
hold(ax,'on');
greenPlane = zeros(S.H, S.W, 3, 'uint8');
greenPlane(:,:,2) = 255;
greenPlane(:,:,1) = 51;
greenPlane(:,:,3) = 51;
hOverlay  = image(ax, greenPlane, 'AlphaData', double(mask) * 0.30);
% Critical: HitTest off + PickableParts none MUST be set at construction
% time and not via a (silently-failing) try/catch.  Otherwise the green
% boundary line of the previously-drawn mask intercepts mouse clicks
% before they reach the next images.roi.Freehand - which is exactly why
% Add only worked on the first stroke and Remove never worked at all.
hBoundary = plot(ax, NaN, NaN, ...
    'Color',[0.2 1 0.2], 'LineWidth', 2, ...
    'HitTest','off', 'PickableParts','none');
set([hImg hOverlay], 'HitTest','off','PickableParts','none');
hold(ax,'off');
title(ax,'Manual tunnel-face region editor', 'Color', TXT);
themeAxes(ax, T);
try, disableDefaultInteractivity(ax); catch, end

msgLbl = uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.02 0.115 0.96 0.05], ...
    'FontSize',11,'HorizontalAlignment','left', ...
    'BackgroundColor', PANEL, 'ForegroundColor', TXT);

% State - declared BEFORE button callbacks/handlers can fire.
regionMode  = 'idle';            % 'add' | 'remove' | 'idle'
isDrawing   = false;
strokePath  = zeros(0, 2);       % live (x,y) points of in-progress outline
hStrokeLine = gobjects(0);       % live preview polyline
undoStack   = {};
cancelled   = false;

% Buttons in a single row across the bottom.  Direct (single-level)
% anonymous callbacks - no nested closures.
btnH = 0.06; btnY = 0.025; gap = 0.008; btnW = 0.115; x0 = 0.02;
posOf = @(i) [x0 + (i-1)*(btnW+gap), btnY, btnW, btnH];
hAddBtn = uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position', posOf(1), 'String','+ Add region','FontSize',10, ...
    'BackgroundColor', ADD_COL, 'ForegroundColor', TXT, ...
    'Callback', @(~,~) setMode('add'));
hRemoveBtn = uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position', posOf(2), 'String','- Remove region','FontSize',10, ...
    'BackgroundColor', REMOVE_COL, 'ForegroundColor', TXT, ...
    'Callback', @(~,~) setMode('remove'));
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position', posOf(3), 'String','Expand border (+)','FontSize',10, ...
    'BackgroundColor', INPUT, 'ForegroundColor', TXT, ...
    'Callback', @(~,~) idleAndRun(@() growShrink(+1)));
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position', posOf(4), 'String','Shrink border (-)','FontSize',10, ...
    'BackgroundColor', INPUT, 'ForegroundColor', TXT, ...
    'Callback', @(~,~) idleAndRun(@() growShrink(-1)));
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position', posOf(5), 'String','Undo','FontSize',10, ...
    'BackgroundColor', INPUT, 'ForegroundColor', TXT, ...
    'Callback', @(~,~) idleAndRun(@undoLast));
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position', posOf(6), 'String','Reset','FontSize',10, ...
    'BackgroundColor', INPUT, 'ForegroundColor', TXT, ...
    'Callback', @(~,~) idleAndRun(@resetMask));
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position', posOf(7), 'String','Finish','FontSize',10, ...
    'BackgroundColor', SUCCESS, 'ForegroundColor', TXT, ...
    'Callback', @(~,~) idleAndRun(@() finishCb(false)));
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position', posOf(8), 'String','Cancel / skip mask','FontSize',10, ...
    'BackgroundColor', WARN, 'ForegroundColor', TXT, ...
    'Callback', @(~,~) idleAndRun(@() finishCb(true)));

% Figure-level mouse handlers drive the freehand drawing.  No blocking
% calls - tool buttons can switch regionMode at any time between
% strokes and the next stroke picks up the new mode at mouse-down.
fig.WindowButtonDownFcn   = @(~,~) onMouseDown();
fig.WindowButtonMotionFcn = @(~,~) onMouseMove();
fig.WindowButtonUpFcn     = @(~,~) onMouseUp();

highlightActiveTool();
setMsg();

uiwait(fig);
if isvalid(fig)
    fig.WindowButtonDownFcn   = '';
    fig.WindowButtonMotionFcn = '';
    fig.WindowButtonUpFcn     = '';
    if ~isempty(hStrokeLine) && isvalid(hStrokeLine)
        try, delete(hStrokeLine); catch, end
    end
    delete(fig);
end
if cancelled, mask = []; end

% ===================== UI helpers =====================================
    function setMsg(extra)
        if ~isgraphics(msgLbl), return; end
        if nargin < 1, extra = ''; end
        pct = 100 * nnz(mask) / max(1, numel(mask));
        modeStr = 'IDLE  -  click +Add region or -Remove region to start';
        if strcmp(regionMode, 'add')
            modeStr = 'ADD region (click-DRAG to outline; release = fill)';
        elseif strcmp(regionMode, 'remove')
            modeStr = 'REMOVE region (click-DRAG to outline; release = cut)';
        end
        base = sprintf( ...
            'Mask: %.1f%% of photo.  Mode: %s.  Outlines stay sticky - draw as many regions as you want.', ...
            pct, modeStr);
        if isempty(extra)
            msgLbl.String = base;
        else
            msgLbl.String = sprintf('%s    [%s]', base, extra);
        end
    end

    function highlightActiveTool()
        addOn = strcmp(regionMode, 'add');
        rmOn  = strcmp(regionMode, 'remove');
        if addOn, addW = 'bold'; else, addW = 'normal'; end
        if rmOn,  rmW  = 'bold'; else, rmW  = 'normal'; end
        try, set(hAddBtn,    'FontWeight', addW); catch, end
        try, set(hRemoveBtn, 'FontWeight', rmW);  catch, end
    end

    function refreshOverlay()
        try, set(hOverlay, 'AlphaData', double(mask) * 0.30); catch, end
        try
            B = bwboundaries(mask, 'noholes');
            if isempty(B)
                hBoundary.XData = NaN; hBoundary.YData = NaN;
            else
                xs = []; ys = [];
                for kk = 1:numel(B)
                    xs = [xs; B{kk}(:,2); NaN]; %#ok<AGROW>
                    ys = [ys; B{kk}(:,1); NaN]; %#ok<AGROW>
                end
                hBoundary.XData = xs; hBoundary.YData = ys;
            end
        catch
        end
        drawnow;     % flush so the user actually SEES the change
    end

% ===================== region drawing =================================
    function setMode(newMode)
        % Switch into Add or Remove mode.  No blocking interaction call
        % to interrupt - the next mouse-down on the axes reads
        % regionMode and starts a stroke of the matching colour.
        regionMode = newMode;
        cancelInProgressStroke();
        highlightActiveTool();
        setMsg();
    end

    function idleAndRun(actionFn)
        % Used by all non-Add/Remove buttons: drop out of any active
        % drawing mode, cancel an in-progress outline, run the action.
        regionMode = 'idle';
        cancelInProgressStroke();
        highlightActiveTool();
        try, actionFn(); catch, end
        setMsg();
    end

    function cancelInProgressStroke()
        if isDrawing
            isDrawing = false;
        end
        if ~isempty(hStrokeLine) && isvalid(hStrokeLine)
            try, delete(hStrokeLine); catch, end
        end
        hStrokeLine = gobjects(0);
        strokePath  = zeros(0, 2);
    end

    function onMouseDown()
        if cancelled || ~isvalid(fig), return; end
        if ~strcmp(regionMode,'add') && ~strcmp(regionMode,'remove'), return; end
        if ~mouseOnImageAxes(), return; end
        cp = ax.CurrentPoint;
        isDrawing  = true;
        strokePath = [cp(1,1), cp(1,2)];
        if strcmp(regionMode,'add'), col = ADD_COL; else, col = REMOVE_COL; end
        if ~isempty(hStrokeLine) && isvalid(hStrokeLine)
            try, delete(hStrokeLine); catch, end
        end
        hold(ax,'on');
        hStrokeLine = plot(ax, strokePath(:,1), strokePath(:,2), ...
            '-', 'Color', col, 'LineWidth', 2, ...
            'HitTest','off', 'PickableParts','none');
        hold(ax,'off');
    end

    function onMouseMove()
        if ~isDrawing, return; end
        cp = ax.CurrentPoint;
        strokePath(end+1,:) = [cp(1,1), cp(1,2)]; %#ok<AGROW>
        if ~isempty(hStrokeLine) && isvalid(hStrokeLine)
            try
                set(hStrokeLine, 'XData', strokePath(:,1), 'YData', strokePath(:,2));
            catch
            end
        end
    end

    function onMouseUp()
        if ~isDrawing, return; end
        isDrawing = false;
        if ~isempty(hStrokeLine) && isvalid(hStrokeLine)
            try, delete(hStrokeLine); catch, end
        end
        hStrokeLine = gobjects(0);
        sp = strokePath;
        strokePath = zeros(0, 2);
        if size(sp, 1) < 3, return; end
        try
            stroke = poly2mask(sp(:,1), sp(:,2), S.H, S.W);
        catch
            stroke = [];
        end
        if isempty(stroke) || ~any(stroke(:)), return; end
        pushUndo();
        before = nnz(mask);
        if strcmp(regionMode,'add')
            mask = mask | stroke;
            setMsg(sprintf('Added %d px to the mask.', nnz(mask) - before));
        elseif strcmp(regionMode,'remove')
            mask = mask & ~stroke;
            removed = before - nnz(mask);
            if removed == 0
                setMsg('Remove had no effect - the drawn region didn''t cover any mask pixels.');
            else
                setMsg(sprintf('Removed %d px from the mask.', removed));
            end
        end
        refreshOverlay();
    end

    function tf = mouseOnImageAxes()
        % Only accept mouse-down events that landed on our axes.  Without
        % this check, every click on a button or the figure background
        % would start a phantom stroke.
        try
            co = fig.CurrentObject;
            if isempty(co), tf = false; return; end
            aco = ancestor(co, 'axes');
            tf = ~isempty(aco) && isequal(aco, ax);
        catch
            tf = false;
        end
    end

% ===================== other tools ====================================
    function pushUndo()
        undoStack{end+1} = mask; %#ok<AGROW>
        if numel(undoStack) > 30, undoStack(1) = []; end
    end

    function growShrink(dir)
        if ~any(mask(:)), return; end
        pushUndo();
        try
            r = max(8, round(min(S.H, S.W)/120));
            se = strel('disk', r);
            if dir > 0
                mask = imdilate(mask, se);
                mask = imfill(mask, 'holes');
            else
                mask = imerode(mask, se);
            end
        catch
        end
        refreshOverlay();
    end

    function undoLast()
        if ~isempty(undoStack)
            mask = undoStack{end};
            undoStack(end) = [];
            refreshOverlay();
        end
    end

    function resetMask()
        pushUndo();
        mask = false(S.H, S.W);
        refreshOverlay();
    end

    function finishCb(isCancel)
        cancelled = isCancel;
        uiresume(fig);
    end
end

function [imPts, wPts, cancelled] = manualPhotoSynthPickerInline(S)
%MANUALPHOTOSYNTHPICKERINLINE  Modal photo<->synth correspondence picker.
%   Used as a fallback when automatic feature matching produces too few
%   PnP inliers.  The user clicks alternating features on the photo and
%   the synth-ortho render; synth coords are lifted to 3-D via the same
%   analytic ortho map the auto path uses (S.ortho + S.plane + nearest
%   cloud point), so this fallback feeds estworldpose the same kind of
%   data the auto path would have.
imPts = zeros(0, 2);
wPts  = zeros(0, 3);
cancelled = true;

T = S.theme;
BG = T.bg;  PANEL = T.panel;  TXT = T.text;  INPUT = T.input;
SUCCESS = T.success;  WARN = T.warn;

fig = figure('Name','Manual photo <-> synth correspondence picker', ...
    'NumberTitle','off','Color', BG, ...
    'WindowStyle','normal','MenuBar','none','ToolBar','figure', ...
    'CloseRequestFcn',@(~,~)finishCb(true));
try, fig.WindowState = 'maximized'; catch, end

axI = axes('Parent',fig, 'Units','normalized', ...
    'Position',[0.02 0.18 0.46 0.78], 'Box','on');
drawPhotoWithMask();
axI.ButtonDownFcn = @(~,~)onImg();

axS = axes('Parent',fig, 'Units','normalized', ...
    'Position',[0.51 0.18 0.46 0.78], 'Box','on');
imshow(S.synth,'Parent',axS);
title(axS,'SYNTH ORTHO  -  click the matching feature here', 'Color', TXT);
themeAxes(axS, T);
set(findobj(axS,'Type','image'),'HitTest','off','PickableParts','none');
try, disableDefaultInteractivity(axS); catch, end
axS.ButtonDownFcn = @(~,~)onSynth();

msgLbl = uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[0.02 0.10 0.96 0.05], ...
    'FontSize',11,'HorizontalAlignment','left', ...
    'BackgroundColor', PANEL, 'ForegroundColor', TXT);

btnW = 0.13; btnH = 0.06; btnY = 0.025; gap = 0.01; x0 = 0.02;
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x0+0*(btnW+gap) btnY btnW btnH], ...
    'String','Flip view','FontSize',10, ...
    'BackgroundColor', INPUT, 'ForegroundColor', TXT, ...
    'TooltipString',['Re-render the synth ortho from the opposite side ', ...
        'of the plane (use this if the synth shows the BACK of the ', ...
        'face instead of the front).  Any pairs you have picked will ', ...
        'be cleared because the synth pixel coordinates change.'], ...
    'Callback',@(~,~)flipView());
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x0+1*(btnW+gap) btnY btnW btnH], ...
    'String','Undo last','FontSize',10, ...
    'BackgroundColor', INPUT, 'ForegroundColor', TXT, ...
    'Callback',@(~,~)undoLast());
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x0+2*(btnW+gap) btnY btnW btnH], ...
    'String','Finish & solve','FontSize',10,'FontWeight','bold', ...
    'BackgroundColor', SUCCESS, 'ForegroundColor', TXT, ...
    'Callback',@(~,~)acceptCb());
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[x0+3*(btnW+gap) btnY btnW btnH], ...
    'String','Cancel','FontSize',10, ...
    'BackgroundColor', WARN, 'ForegroundColor', TXT, ...
    'Callback',@(~,~)finishCb(true));

mode       = 'image';   % 'image' (photo first) or 'synth' (synth second)
pendingPix = [];
imgPts     = zeros(0,2);
synPts     = zeros(0,2);
hImgMarks  = gobjects(0);
hImgLabels = gobjects(0);
hSynMarks  = gobjects(0);
hSynLabels = gobjects(0);
% Bright, saturated palette chosen to stay readable on both real
% photos and the (mostly dark) synth ortho.  Pair k uses palette row
% ((k-1) mod size) + 1, so undo/redo realigns colours automatically
% and a pair always has the same colour on photo and synth.
pairPalette = [ ...
    1.00 0.25 0.25;   % red
    0.25 1.00 0.25;   % lime
    1.00 0.85 0.10;   % gold
    0.20 0.85 1.00;   % sky blue
    1.00 0.35 1.00;   % magenta
    1.00 0.60 0.10;   % orange
    0.55 1.00 0.30;   % spring green
    0.45 0.65 1.00;   % azure
    1.00 0.45 0.75;   % hot pink
    0.85 1.00 0.35;   % chartreuse
    0.30 1.00 0.85;   % aquamarine
    1.00 0.80 0.55];  % peach
setMsg();
uiwait(fig);
if isvalid(fig), delete(fig); end

    function setMsg()
        if ~isgraphics(msgLbl), return; end
        n = size(imgPts,1);
        if strcmp(mode, 'image')
            msgLbl.String = sprintf( ...
                'Pairs: %d (need >=6).  Click the next feature in the PHOTO (left).', n);
        else
            msgLbl.String = sprintf( ...
                'Pairs: %d (need >=6).  Click the SAME feature in the SYNTH (right).', n);
        end
    end

    function drawPhotoWithMask()
        % Draw the photo on the left axes.  If a tunnel-face mask is
        % set, dim non-face regions and outline the face in green so
        % the user can see which area to pick from.  Picks outside the
        % mask are still allowed; the visual is a guide only.
        cla(axI,'reset');
        if isfield(S,'faceMask') && ~isempty(S.faceMask) && any(S.faceMask(:))
            dimmed = S.img;
            outside = ~S.faceMask;
            for ch_ = 1:3
                Cc_ = dimmed(:,:,ch_);
                Cc_(outside) = uint8(0.30 * double(Cc_(outside)));
                dimmed(:,:,ch_) = Cc_;
            end
            imshow(dimmed,'Parent',axI); hold(axI,'on');
            try
                Bd = bwboundaries(S.faceMask, 'noholes');
                for kk = 1:numel(Bd)
                    plot(axI, Bd{kk}(:,2), Bd{kk}(:,1), ...
                        'Color',[0.2 1 0.2], 'LineWidth',2, ...
                        'HitTest','off','PickableParts','none');
                end
            catch
            end
            hold(axI,'off');
            title(axI, ['PHOTO  -  click a feature on the GREEN-OUTLINED ', ...
                'tunnel face first'], 'Color', TXT);
        else
            imshow(S.img,'Parent',axI);
            title(axI,'PHOTO  -  click a feature here first', 'Color', TXT);
        end
        themeAxes(axI, T);
        set(findobj(axI,'Type','image'),'HitTest','off','PickableParts','none');
        try, disableDefaultInteractivity(axI); catch, end
    end

    function onImg()
        if ~strcmp(mode, 'image'), return; end
        cp = axI.CurrentPoint;
        x = min(max(cp(1,1),1), S.W);
        y = min(max(cp(1,2),1), S.H);
        pendingPix = [x y];
        k = size(imgPts,1) + 1;     % index of the pair being started
        col = pairColor(k);
        hold(axI,'on');
        hImgMarks(end+1) = plot(axI, x, y, '+', ...
            'Color', col, 'MarkerSize',14, 'LineWidth',2);
        hImgLabels(end+1) = text(axI, x, y, sprintf('  %d', k), ...
            'Color', col, 'FontWeight','bold');
        hold(axI,'off');
        mode = 'synth';
        setMsg();
    end

    function onSynth()
        if ~strcmp(mode, 'synth') || isempty(pendingPix), return; end
        cp = axS.CurrentPoint;
        Hs = size(S.synth,1); Ws = size(S.synth,2);
        x = min(max(cp(1,1),1), Ws);
        y = min(max(cp(1,2),1), Hs);
        imgPts(end+1,:) = pendingPix; %#ok<AGROW>
        synPts(end+1,:) = [x y];      %#ok<AGROW>
        k = size(imgPts,1);          % index of the pair just completed
        col = pairColor(k);
        hold(axS,'on');
        hSynMarks(end+1) = plot(axS, x, y, '+', ...
            'Color', col, 'MarkerSize',14, 'LineWidth',2);
        hSynLabels(end+1) = text(axS, x, y, sprintf('  %d', k), ...
            'Color', col, 'FontWeight','bold');
        hold(axS,'off');
        pendingPix = [];
        mode = 'image';
        setMsg();
    end

    function c = pairColor(k)
        c = pairPalette(mod(k-1, size(pairPalette,1)) + 1, :);
    end

    function undoLast()
        if ~isempty(pendingPix)
            pendingPix = [];
            delLast(hImgMarks);  hImgMarks  = hImgMarks(1:end-1);
            delLast(hImgLabels); hImgLabels = hImgLabels(1:end-1);
            mode = 'image';
        elseif ~isempty(imgPts)
            imgPts(end,:) = []; synPts(end,:) = [];
            delLast(hImgMarks);  hImgMarks  = hImgMarks(1:end-1);
            delLast(hImgLabels); hImgLabels = hImgLabels(1:end-1);
            delLast(hSynMarks);  hSynMarks  = hSynMarks(1:end-1);
            delLast(hSynLabels); hSynLabels = hSynLabels(1:end-1);
            mode = 'image';
        end
        setMsg();
    end

    function acceptCb()
        if size(imgPts,1) < 6
            msgbox(sprintf('Need at least 6 pairs (have %d).', size(imgPts,1)), ...
                'More pairs needed','warn','modal');
            return;
        end
        % Lift synth pixels to 3-D via the analytic ortho map, then snap
        % each plane point to the nearest actual cloud point.
        psx = synPts(:,1); psy = synPts(:,2);
        uC = S.ortho.uLo + (psx - 0.5)*S.ortho.dx;
        vC = S.ortho.vLo + (psy - 0.5)*S.ortho.dx;
        planePts = S.plane.c + uC * S.plane.u + vC * S.plane.v;
        try
            kdt = KDTreeSearcher(S.xyz);
            nnI = knnsearch(kdt, planePts);
        catch
            nnI = zeros(size(planePts,1),1);
            for ii=1:size(planePts,1)
                d2 = sum((S.xyz - planePts(ii,:)).^2, 2);
                [~,nnI(ii)] = min(d2);
            end
        end
        imPts = imgPts;
        wPts  = S.xyz(nnI,:);
        cancelled = false;
        uiresume(fig);
    end

    function finishCb(isCancel)
        cancelled = isCancel;
        if isCancel
            imPts = zeros(0,2);
            wPts  = zeros(0,3);
        end
        uiresume(fig);
    end

    function flipView()
        % Re-render the synth ortho from the opposite side of the
        % dominant plane.  Negating both n and u keeps the {u,v,n}
        % frame right-handed (v still points "down" in world Z), so
        % the synth comes out mirrored left-right rather than
        % upside-down.  All existing picks are discarded because their
        % synth pixel coordinates no longer point at the same
        % world feature.
        if size(imgPts,1) > 0 || ~isempty(pendingPix)
            ans_ = questdlg( ...
                sprintf(['Flipping the view re-renders the synth ortho ', ...
                    'from the opposite side of the plane.  Your %d ', ...
                    'picked pair(s) will be discarded.  Continue?'], ...
                    size(imgPts,1)), ...
                'Confirm flip view', 'Flip', 'Cancel', 'Cancel');
            if ~strcmp(ans_, 'Flip'), return; end
        end
        S.plane.n = -S.plane.n;
        S.plane.u = -S.plane.u;
        [newSynth, newSynthMask, newULo, newVLo, newDx] = ...
            renderOrthoForFlipInline(S);
        S.synth     = newSynth;
        S.synthMask = newSynthMask;
        S.HsynW     = [size(newSynth,1) size(newSynth,2)];
        S.ortho.uLo = newULo;
        S.ortho.vLo = newVLo;
        S.ortho.dx  = newDx;
        clearAllPairs();
        cla(axS,'reset');
        imshow(S.synth,'Parent',axS);
        title(axS,'SYNTH ORTHO (flipped)  -  click the matching feature here', ...
            'Color', TXT);
        themeAxes(axS, T);
        set(findobj(axS,'Type','image'),'HitTest','off','PickableParts','none');
        try, disableDefaultInteractivity(axS); catch, end
        axS.ButtonDownFcn = @(~,~)onSynth();
        setMsg();
    end

    function clearAllPairs()
        pendingPix = [];
        imgPts     = zeros(0,2);
        synPts     = zeros(0,2);
        delAll(hImgMarks);  hImgMarks  = gobjects(0);
        delAll(hImgLabels); hImgLabels = gobjects(0);
        delAll(hSynMarks);  hSynMarks  = gobjects(0);
        delAll(hSynLabels); hSynLabels = gobjects(0);
        mode = 'image';
    end

    function delAll(harr)
        for h_ = harr(:).'
            if isgraphics(h_), delete(h_); end
        end
    end

    function delLast(harr)
        if ~isempty(harr) && isgraphics(harr(end)), delete(harr(end)); end
    end
end

function mask = editFaceMaskInline(S, ax, initialMask)
fig = ancestor(ax, 'figure');
mask = logical(initialMask);
if isempty(mask), mask = false(S.H, S.W); end
setStatus(fig, ['Brush edit: blue strokes ADD tunnel face, red strokes ', ...
    'REMOVE wall/roof/floor. Click Finish when the overlay is correct.'], ...
    [0.10 0.10 0.50]);
showBrushEditOverlay(S, ax, mask);

T = S.theme;
panel = uipanel('Parent', S.tab(6), 'Units','normalized', ...
    'Position', [0.012 0.015 0.976 0.260], ...
    'BorderType','line', 'HighlightColor', T.border, ...
    'BackgroundColor', T.panel, 'ForegroundColor', T.text);
uicontrol(panel, 'Style','text', 'Units','normalized', ...
    'Position',[0.02 0.55 0.96 0.40], ...
    'HorizontalAlignment','left', ...
    'BackgroundColor', T.panel, 'ForegroundColor', T.text, 'FontSize',10, ...
    'String','Paint corrections on the image. Draw a freehand patch; releasing the mouse applies it immediately.');
uicontrol(panel, 'Style','text', 'Units','normalized', ...
    'Position',[0.02 0.30 0.96 0.20], ...
    'HorizontalAlignment','left', ...
    'BackgroundColor', T.panel, 'ForegroundColor', T.subtext, 'FontSize',9, ...
    'String','Blue brush expands the face mask. Red brush removes wrongly included wall, roof, floor, equipment, or background.');

setappdata(fig, 'brushChoice', '');
uicontrol(panel, 'Style','pushbutton', 'Units','normalized', ...
    'Position',[0.02 0.05 0.18 0.22], 'String','Blue add', ...
    'FontSize',10, 'FontWeight','bold', ...
    'BackgroundColor', [0.12 0.38 0.85], 'ForegroundColor', T.text, ...
    'Callback', @(~,~)pickBrushChoice(fig,'add'));
uicontrol(panel, 'Style','pushbutton', 'Units','normalized', ...
    'Position',[0.22 0.05 0.18 0.22], 'String','Red remove', ...
    'FontSize',10, 'FontWeight','bold', ...
    'BackgroundColor', T.warn, 'ForegroundColor', T.text, ...
    'Callback', @(~,~)pickBrushChoice(fig,'remove'));
uicontrol(panel, 'Style','pushbutton', 'Units','normalized', ...
    'Position',[0.42 0.05 0.14 0.22], 'String','Undo', ...
    'FontSize',10, 'BackgroundColor', T.input, 'ForegroundColor', T.text, ...
    'Callback', @(~,~)pickBrushChoice(fig,'undo'));
uicontrol(panel, 'Style','pushbutton', 'Units','normalized', ...
    'Position',[0.58 0.05 0.14 0.22], 'String','Finish', ...
    'FontSize',10, 'BackgroundColor', T.success, 'ForegroundColor', T.text, ...
    'Callback', @(~,~)pickBrushChoice(fig,'finish'));
uicontrol(panel, 'Style','pushbutton', 'Units','normalized', ...
    'Position',[0.74 0.05 0.14 0.22], 'String','Cancel edit', ...
    'FontSize',10, 'BackgroundColor', T.input, 'ForegroundColor', T.text, ...
    'Callback', @(~,~)pickBrushChoice(fig,'cancel'));

undoStack = {};
while true
    setappdata(fig, 'brushChoice', '');
    uiwait(fig);
    choice = getappdata(fig, 'brushChoice');
    if strcmp(choice, 'finish')
        break;
    elseif strcmp(choice, 'cancel') || isempty(choice)
        mask = logical(initialMask);
        break;
    elseif strcmp(choice, 'undo')
        if ~isempty(undoStack)
            mask = undoStack{end};
            undoStack(end) = [];
            showBrushEditOverlay(S, ax, mask);
        end
    elseif strcmp(choice, 'add') || strcmp(choice, 'remove')
        stroke = brushStrokeMaskInline(S, ax, choice);
        if ~isempty(stroke) && any(stroke(:))
            undoStack{end+1} = mask; %#ok<AGROW>
            if strcmp(choice, 'add')
                mask = mask | stroke;
            else
                mask(stroke) = false;
            end
            showBrushEditOverlay(S, ax, mask);
        end
    end
end
try, delete(panel); catch, end
if isempty(mask) || ~any(mask(:)), mask = []; end
end

function stroke = brushStrokeMaskInline(S, ax, choice)
stroke = [];
col = [0.1 0.45 1.0];
if strcmp(choice, 'remove'), col = [1.0 0.15 0.10]; end
try
    roi = drawfreehand(ax, 'Color', col, 'LineWidth', 2, ...
        'FaceAlpha', 0.20);
    stroke = createMask(roi, S.img);
    try, delete(roi); catch, end
catch
    try
        stroke = roipoly(S.img);
    catch
    end
end
end

function showBrushEditOverlay(S, ax, mask)
cla(ax,'reset');
overlay = S.img;
if ~isempty(mask) && any(mask(:))
    R = overlay(:,:,1); G = overlay(:,:,2); B = overlay(:,:,3);
    alpha = 0.30;
    R(mask) = uint8((1-alpha)*double(R(mask)));
    G(mask) = uint8((1-alpha)*double(G(mask)) + alpha*255);
    B(mask) = uint8((1-alpha)*double(B(mask)));
    overlay(:,:,1) = R; overlay(:,:,2) = G; overlay(:,:,3) = B;
end
imshow(overlay,'Parent',ax); hold(ax,'on');
try
    Bd = bwboundaries(mask, 'noholes');
    for k = 1:numel(Bd)
        plot(ax, Bd{k}(:,2), Bd{k}(:,1), 'Color',[0.2 1 0.2], 'LineWidth',2);
    end
catch
end
hold(ax,'off');
title(ax, '6. Brush edit - green = current face mask');
themeAxes(ax, S.theme);
end

function pickBrushChoice(fig, choice)
setappdata(fig, 'brushChoice', choice);
uiresume(fig);
end

function mask = cleanFaceMaskInline(mask, H, W)
if isempty(mask) || ~any(mask(:))
    mask = false(H,W);
    return;
end
try
    mask = imfill(mask, 'holes');
    mask = imclose(mask, strel('disk', max(6, round(min(H,W)/70))));
    mask = imdilate(mask, strel('disk', max(8, round(min(H,W)/55))));
catch
end
try
    cc = bwconncomp(mask);
    if cc.NumObjects > 1
        sz = cellfun(@numel, cc.PixelIdxList);
        [~,bigK] = max(sz);
        keep = false(size(mask));
        keep(cc.PixelIdxList{bigK}) = true;
        mask = keep;
    end
catch
end
end

function showAutoFaceMask(S, ~, mask)
try
    ax = S.ax(3); cla(ax,'reset');
    overlay = S.img;
    R = overlay(:,:,1); G = overlay(:,:,2); B = overlay(:,:,3);
    alpha = 0.30;
    R(mask) = uint8((1-alpha)*double(R(mask)));
    G(mask) = uint8((1-alpha)*double(G(mask)) + alpha*255);
    B(mask) = uint8((1-alpha)*double(B(mask)));
    overlay(:,:,1) = R; overlay(:,:,2) = G; overlay(:,:,3) = B;
    imshow(overlay,'Parent',ax); hold(ax,'on');
    try
        Bd = bwboundaries(mask, 'noholes');
        for k = 1:numel(Bd)
            plot(ax, Bd{k}(:,2), Bd{k}(:,1), 'Color',[0.2 1 0.2], 'LineWidth',2);
        end
    catch
    end
    hold(ax,'off');
    title(ax, '3. Automatic face segmentation applied');
    themeAxes(ax, S.theme);
    setDesc(S, 3, {
        'AUTO FACE SEGMENTATION APPLIED.  The synthetic face support was matched to the full photo and warped into photo pixels.  Only the green region is used for feature matching; the original photo is still used for overlays and recolouring.'
        ''
        sprintf('Face pixels kept: %s of %s  (%.1f%% of the photo)     |     Full-photo matches: %d     |     Homography inliers: %d', ...
            commaInt(nnz(mask)), commaInt(numel(mask)), ...
            100 * nnz(mask) / numel(mask), ...
            S.autoFaceInfo.nRaw, S.autoFaceInfo.nInliers)
        });
catch
end
end

function [pr, ps, fR, fS] = featPair(gR, gS, kind)
pr = zeros(0,2); ps = zeros(0,2); fR = []; fS = [];
try
    switch lower(kind)
        case 'sift'
            fR = detectSIFTFeatures(gR, 'ContrastThreshold', 0.0067);
            fS = detectSIFTFeatures(gS, 'ContrastThreshold', 0.0067);
            minN = 20;
        case 'kaze'
            fR = detectKAZEFeatures(gR, 'Threshold', 0.0001);
            fS = detectKAZEFeatures(gS, 'Threshold', 0.0001);
            minN = 20;
        case 'orb'
            fR = detectORBFeatures(gR);
            fS = detectORBFeatures(gS);
            minN = 50;
        otherwise
            return;
    end
    if safeCount(fR) < minN || safeCount(fS) < minN, return; end
    [dR,vR] = extractFeatures(gR, fR);
    [dS,vS] = extractFeatures(gS, fS);
    m = matchFeatures(dR,dS,'Unique',true,'MaxRatio',0.95, ...
        'MatchThreshold',60);
    if isempty(m), return; end
    pr = double(vR(m(:,1)).Location);
    ps = double(vS(m(:,2)).Location);
catch
end
end

function plotPts(ax, F, off, spec)
if isempty(F) || F.Count==0, return; end
L = F.Location;
plot(ax, L(:,1)+off(1), L(:,2)+off(2), spec, 'MarkerSize',3);
end

function n = safeCount(F)
if isempty(F), n = 0; else, n = F.Count; end
end

function themeAxes(ax, T)
% Apply dark-theme colours to an axes after a plot has been drawn.
% Safe to call on imshow, scatter3, pcshow, or empty axes.
if ~isgraphics(ax), return; end
try, set(ax, 'XColor', T.text, 'YColor', T.text, 'GridColor', T.border); catch, end
try, ax.ZColor = T.text; catch, end
try, ax.Title.Color = T.text; catch, end
end

function s = commaInt(n)
% Format an integer with thousands separators (e.g. 1,889,700).
s = sprintf('%.0f', n);
% reverse, insert commas every 3 chars, reverse back
sr = fliplr(s);
out = '';
for i = 1:numel(sr)
    out(end+1) = sr(i); %#ok<AGROW>
    if mod(i,3)==0 && i < numel(sr) && ~strcmp(sr(i+1),'-')
        out(end+1) = ','; %#ok<AGROW>
    end
end
s = fliplr(out);
end

function setDesc(S, k, lines)
% Set description text for stage k.  Because tabs may be MERGED (e.g.
% stages 3+4+5 share one tab in the new layout), this function stores
% each stage's text on the tab's descLbl UserData, then re-renders the
% banner with every stage's text on that tab stacked in stage order.
% That way each tile's description appears as it gets ready, and a
% merged tab shows all of its tiles' descriptions side by side without
% any tile overwriting another.
if k < 1 || k > numel(S.stageTab), return; end
tb = S.stageTab(k);
if tb < 1 || tb > numel(S.descLbl), return; end
if ~isgraphics(S.descLbl(tb)), return; end
if ischar(lines) || isstring(lines), lines = {char(lines)}; end

ud = S.descLbl(tb).UserData;
if ~isstruct(ud), ud = struct(); end
ud.(sprintf('stage%d', k)) = lines;
S.descLbl(tb).UserData = ud;
S.descLbl(tb).String   = renderTabDesc(ud, S.stageTab, tb);
end

function out = renderTabDesc(ud, stageTab, tb)
% Concatenate per-stage description cells stored in `ud` for the
% stages assigned to tab `tb`.  Stages are listed in stage-number order
% with a blank-line separator between them.
out = {};
stagesInTab = find(stageTab == tb);
for j = 1:numel(stagesInTab)
    s = stagesInTab(j);
    f = sprintf('stage%d', s);
    if isfield(ud, f) && ~isempty(ud.(f))
        if ~isempty(out)
            out{end+1, 1} = ''; %#ok<AGROW>
        end
        out = [out; ud.(f)(:)]; %#ok<AGROW>
    end
end
if isempty(out)
    out = {'(description appears after this stage runs)'};
end
end

function setStatus(fig, msg, color)
S = guidata(fig);
S.h.statusLbl.String = msg;
if nargin >= 3, S.h.statusLbl.ForegroundColor = color; end
end

function closeStaleWindows()
% Close any previous photo2pc_gui main window, manual picker, or
% progress dialog still hanging around from a prior run.
names = { ...
    'photo2pc_gui  —  pipeline visualizer', ...
    'photo2pc_gui — pipeline visualizer', ...
    'Photo <-> Point-Cloud manual picking', ...
    'photo2pc_gui progress'};
for k = 1:numel(names)
    h = findall(0, 'Type','figure', 'Name', names{k});
    for ii = 1:numel(h)
        try, delete(h(ii)); catch, end
    end
end
end

function onClose(fig)
% Also clean up any progress dialog we may have spawned.
try
    S = guidata(fig);
    if isstruct(S) && isfield(S,'h') && isfield(S.h,'wb') ...
            && ~isempty(S.h.wb) && isvalid(S.h.wb)
        delete(S.h.wb);
    end
catch
end
delete(fig);
end

function setBusy(fig, busy)
% Grey out the action buttons while the pipeline is running so the
% user can't accidentally launch another run on top of one in flight.
S = guidata(fig);
state = 'on'; if busy, state = 'off'; end
btnNames = {'runBtn','stepBtn','resetBtn','saveBtn'};
for k = 1:numel(btnNames)
    if isfield(S.h, btnNames{k}) && isgraphics(S.h.(btnNames{k}))
        S.h.(btnNames{k}).Enable = state;
    end
end
if isgraphics(S.h.cloudEdit),  S.h.cloudEdit.Enable  = state; end
if isgraphics(S.h.imgEdit),    S.h.imgEdit.Enable    = state; end
% Update pointer for extra visual feedback.
try, fig.Pointer = ternary(busy, 'watch', 'arrow'); catch, end
end

function out = ternary(cond, a, b)
if cond, out = a; else, out = b; end
end

function ensureWaitbar(fig, frac, msg)
% Create/update a waitbar that lives across all stages.  If the user
% closed it, recreate it so progress stays visible.  The waitbar is
% themed to match the GUI's dark palette.
S = guidata(fig);
T = S.theme;
need = ~isfield(S.h,'wb') || isempty(S.h.wb) || ~isvalid(S.h.wb);
if need
    S.h.wb = waitbar(frac, msg, 'Name','photo2pc_gui progress', ...
        'CreateCancelBtn','');
    themeWaitbar(S.h.wb, T);
    guidata(fig, S);
else
    waitbar(frac, S.h.wb, msg);
    themeWaitbar(S.h.wb, T);    % re-apply in case waitbar reset colours
end
end

function themeWaitbar(h, T)
% Restyle a waitbar's children so the dialog matches the dark theme.
if ~isgraphics(h), return; end
try, set(h, 'Color', T.panel); catch, end
% all axes (the progress-bar background + bar) and text labels
kids = findall(h);
for k = kids(:).'
    try
        ty = get(k, 'Type');
        switch ty
            case 'axes'
                set(k, 'Color', T.input, 'XColor', T.text, ...
                    'YColor', T.text, 'GridColor', T.border);
                if ~isempty(k.Title), k.Title.Color = T.text; end
            case 'text'
                set(k, 'Color', T.text);
            case 'patch'
                % the actual progress bar - tint to accent so it pops
                set(k, 'FaceColor', T.accent, 'EdgeColor', T.accent);
            case 'uicontrol'
                style = get(k, 'Style');
                if strcmp(style, 'text')
                    set(k, 'BackgroundColor', T.panel, 'ForegroundColor', T.text);
                end
        end
    catch
    end
end
end

function dismissWaitbar(fig)
S = guidata(fig);
if isfield(S.h,'wb') && ~isempty(S.h.wb) && isvalid(S.h.wb)
    try, delete(S.h.wb); catch, end
end
S.h.wb = [];
guidata(fig, S);
end

function s = shortPath(p)
[d,f,e] = fileparts(p);
[~,dShort] = fileparts(d);
if isempty(dShort)
    s = [f e];
else
    s = [dShort filesep f e];
end
end

% =====================================================================
%   INLINE PIPELINE HELPERS  (so the GUI is self-contained)
% =====================================================================
function pc = loadCloudInline(path)
[~,~,ext] = fileparts(char(path));
ext = lower(ext);
if strcmp(ext,'.ply')
    pc = pcread(char(path)); return;
end
if ~strcmp(ext,'.mat')
    error('photo2pc_gui:badExt','Unsupported cloud extension: %s', ext);
end
S = load(char(path));
fns = fieldnames(S);
for k = 1:numel(fns)
    if isa(S.(fns{k}),'pointCloud'), pc = S.(fns{k}); return; end
end
candXYZ = {'facePts','xyz','points','XYZ','pts','vertices','V'};
candRGB = {'faceRGB','rgb','colors','color','RGB','C'};
XYZ = []; RGB = [];
for k = 1:numel(candXYZ)
    if isfield(S,candXYZ{k}), XYZ = S.(candXYZ{k}); break; end
end
for k = 1:numel(candRGB)
    if isfield(S,candRGB{k}), RGB = S.(candRGB{k}); break; end
end
if isempty(XYZ)
    best = 0; pick = '';
    for k = 1:numel(fns)
        v = S.(fns{k});
        if isnumeric(v) && (size(v,1)==3 || size(v,2)==3) && numel(v)>best
            best = numel(v); pick = fns{k};
        end
    end
    if ~isempty(pick), XYZ = S.(pick); end
end
if isempty(XYZ), error('photo2pc_gui:noXYZ','No XYZ array in %s', path); end
if size(XYZ,1)==3 && size(XYZ,2)~=3, XYZ = XYZ.'; end
XYZ = double(XYZ);
if ~isempty(RGB)
    if size(RGB,1)==3 && size(RGB,2)~=3, RGB = RGB.'; end
    if isfloat(RGB) && max(RGB(:)) <= 1.0+1e-6
        RGB = uint8(round(RGB*255));
    end
    RGB = uint8(RGB);
    pc = pointCloud(XYZ,'Color',RGB);
else
    pc = pointCloud(XYZ);
end
end

function intr = resolveIntrinsicsInline(imgFile, W, H)
f = focalFromEXIFInline(imgFile, W);
if isempty(f) || ~isfinite(f) || f<=0
    f = 1.2*max(W,H);
end
intr = cameraIntrinsics([f f], [W/2 H/2], [H W]);
end

function f = focalFromEXIFInline(imgFile, W)
f = [];
try
    info = imfinfo(imgFile);
    if isfield(info,'DigitalCamera')
        dc = info.DigitalCamera;
        if isfield(dc,'FocalLengthIn35mmFilm') && ~isempty(dc.FocalLengthIn35mmFilm) && dc.FocalLengthIn35mmFilm>0
            f = double(dc.FocalLengthIn35mmFilm)/36 * W;
        elseif isfield(dc,'FocalLength') && isfield(dc,'FocalPlaneXResolution') && dc.FocalPlaneXResolution>0
            f = double(dc.FocalLength)*double(dc.FocalPlaneXResolution);
        end
    end
catch
end
end

function K = intrMatrixInline(intr)
try
    K = intr.K;
    if ~isnumeric(K) || ~isequal(size(K),[3 3])
        K = intr.IntrinsicMatrix.';
    end
catch
    K = intr.IntrinsicMatrix.';
end
end

function [pose, inIdx, ok] = solvePnPInline(imPts, wPts, intr, maxReprojErr)
if nargin < 4 || isempty(maxReprojErr), maxReprojErr = 8; end
ok = false; pose = []; inIdx = [];
try
    [pose, inIdx, st] = estworldpose(imPts, wPts, intr, ...
        'MaxReprojectionError',maxReprojErr,'Confidence',99,'MaxNumTrials',5000);
    ok = (st == 0);
catch
    try
        [Rcw, tcw, inIdx, st] = estimateWorldCameraPose(imPts, wPts, intr, ...
            'MaxReprojectionError',maxReprojErr,'Confidence',99,'MaxNumTrials',5000); %#ok<ASGLU>
        ok = (st == 0);
        if ok
            Rwc = Rcw.'; twc = -Rwc * tcw(:);
            try
                pose = rigidtform3d(Rwc, twc.');
            catch
                pose = rigid3d(Rwc, twc.');
            end
        end
    catch
    end
end
end

function [pose, inIdx, ok] = solvePnPInlineLoose(imPts, wPts, intr)
% Used after manual point-picking: human-vetted picks, so loosen the
% RANSAC threshold so estworldpose accepts every pick.
[pose, inIdx, ok] = solvePnPInline(imPts, wPts, intr, 1e6);
end

function [ok, reason] = isPoseLikelyValidInline(pose, intr, K, xyz, W, H, imPts, wPts)
%ISPOSELIKELYVALIDINLINE  Multi-criteria sanity check on a PnP solution.
%   Returns ok = true only if every check passes; otherwise the caller
%   should treat the alignment as a failure and fall back to manual
%   correspondences.  `reason` gives a short human-readable cause when
%   ok = false.  Tuned to ACCEPT any plausibly-correct pose and REJECT
%   the "random matches happened to be self-consistent" case where a
%   wrong pose nominally passes PnP.
ok = false; reason = '';
try
    if isempty(pose) || isempty(imPts) || size(imPts,1) < 6
        reason = 'no pose / not enough pairs'; return;
    end
    [R_cw, t_cw] = poseToExtrinsicsInline(pose);

    % --- 1. Reprojection RMS on the supplied pairs ---------------------
    [uv, front] = projectPtsInline(wPts, R_cw, t_cw, K);
    if nnz(front) < 6
        reason = 'too few pairs in front of the camera'; return;
    end
    e = hypot(uv(front,1)-imPts(front,1), uv(front,2)-imPts(front,2));
    rms = sqrt(mean(e.^2));
    if ~isfinite(rms) || rms > 15
        reason = sprintf('reprojection RMS = %.1f px (>15)', rms); return;
    end

    % --- 2. Camera position within a reasonable distance of the cloud --
    Cw  = -R_cw.' * t_cw(:);
    ctr = mean(xyz, 1).';
    span = max(max(xyz, [], 1) - min(xyz, [], 1));
    if ~isfinite(span) || span <= 0
        reason = 'degenerate cloud'; return;
    end
    dist = norm(Cw - ctr);
    if ~isfinite(dist) || dist > 50*span
        reason = sprintf('camera %.1fx cloud span away (>50x)', dist/span); return;
    end
    if dist < 0.01*span
        reason = 'camera too close to / inside cloud'; return;
    end

    % --- 3. Optical axis points TOWARD the cloud (not away) -----------
    % R_cw maps world->camera, so R_cw(3,:).' is the camera Z-axis
    % expressed in world coords (the direction looked at).  Compare
    % with the world-vector from camera position to cloud centroid.
    opt = R_cw(3,:).';
    aim = ctr - Cw; aimN = norm(aim);
    if aimN < eps
        reason = 'camera at cloud centroid'; return;
    end
    aim = aim / aimN;
    cosLook = dot(opt, aim);
    if ~isfinite(cosLook) || cosLook < 0.30
        reason = sprintf('camera not facing cloud (cos=%.2f)', cosLook); return;
    end

    % --- 4. Cloud coverage: enough of the cloud must land in the image -
    [uvAll, frAll] = projectPtsInline(xyz, R_cw, t_cw, K);
    inside = frAll & uvAll(:,1) >= 1 & uvAll(:,1) <= W ...
                   & uvAll(:,2) >= 1 & uvAll(:,2) <= H;
    pct = nnz(inside) / max(1, size(xyz,1));
    if pct < 0.10
        reason = sprintf('only %.1f%% of cloud projects into the photo (<10%%)', 100*pct);
        return;
    end

    ok = true;
catch ME
    reason = ['sanity-check error: ' ME.message];
end
end

function [R_cw, t_cw] = poseToExtrinsicsInline(pose)
try, R = pose.R; catch, R = pose.Rotation; end
try, t = pose.Translation; catch, t = pose.T(4,1:3); end
R_cw = R.'; t_cw = -R_cw * t(:);
end

function [uv, front] = projectPtsInline(X, R_cw, t_cw, K)
Xc = (R_cw*X.' + t_cw).';
front = Xc(:,3) > 1e-6;
z = Xc(:,3); z(~front) = 1;
u = K(1,1)*Xc(:,1)./z + K(1,3);
v = K(2,2)*Xc(:,2)./z + K(2,3);
uv = [u v];
end

function [u, v, n] = orientPlaneAxesInline(u, v, n)
% Force the SVD-derived principal axes into a consistent orientation:
%   * v points "down" in world Z (upright synth render)
%   * the {u, v, n} frame is right-handed (det = +1)
% Without this, SVD's sign ambiguity makes the synth render randomly
% flipped top-down or rotated 180 between runs.  The SIGN of n is
% still ambiguous - use chooseFrontOrientationInline if you have the
% photo available.
if v(3) > 0, v = -v; end
if det([u(:).'; v(:).'; n(:).']) < 0, u = -u; end
end

function [u, v, n, info] = chooseFrontOrientationInline(u, v, n, xyz, rgb, img)
% Resolve the +/- ambiguity in SVD's plane normal by rendering the
% cloud orthographically from BOTH n directions at a small test
% resolution, matching against the photo, and picking whichever
% orientation produces more SIFT matches that survive a projective
% geometric check (= the side the camera was looking at).
info = struct('nFront', 0, 'nBack', 0, 'flipped', false);
testW = 1200;
gP = adaptiveEqualizeInline(downsamplePhotoGrayInline(img, testW/size(img,2)));
try
    fP = detectSIFTFeatures(gP, 'ContrastThreshold', 0.005);
catch
    return;
end
if isempty(fP) || fP.Count < 20, return; end
[dP, vP] = extractFeatures(gP, fP);
nA = countMatchesQuickInline( u,  v,  n, xyz, rgb, dP, vP, testW);
nB = countMatchesQuickInline(-u,  v, -n, xyz, rgb, dP, vP, testW);
info.nFront = nA; info.nBack = nB;
if nB > nA, u = -u; n = -n; info.flipped = true; end
end

function nMatches = countMatchesQuickInline(u, v, n, xyz, rgb, dPhoto, vPhoto, testW)
nMatches = 0;
c = mean(xyz,1);
d = xyz - c;
up = d * u.';   vp = d * v.';   wp_ = d * n.';
% Vector-form prctile -> one sort per axis instead of two.
uB = prctile(up, [0.5 99.5]); uLo = uB(1); uHi = uB(2);
vB = prctile(vp, [0.5 99.5]); vLo = vB(1); vHi = vB(2);
if uHi <= uLo || vHi <= vLo, return; end
dx = (uHi - uLo) / testW;
Hs = max(20, round((vHi - vLo)/dx));
ix = floor((up - uLo)/dx) + 1;
iy = floor((vp - vLo)/dx) + 1;
keep = ix>=1 & ix<=testW & iy>=1 & iy<=Hs;
ix = ix(keep); iy = iy(keep); rgbK = rgb(keep,:); wD = wp_(keep);
[~, ord] = sort(wD, 'descend');
ix = ix(ord); iy = iy(ord); rgbK = rgbK(ord,:);
lin = sub2ind([Hs testW], iy, ix);
synth = zeros(Hs, testW, 3, 'uint8');
for ch = 1:3
    Cc = zeros(Hs, testW, 'uint8');
    Cc(lin) = rgbK(:,ch);
    synth(:,:,ch) = Cc;
end
gS = adaptiveEqualizeInline(rgb2gray(synth));
try
    fS = detectSIFTFeatures(gS, 'ContrastThreshold', 0.005);
catch
    return;
end
if isempty(fS) || fS.Count < 20, return; end
[dS, vS] = extractFeatures(gS, fS);
m = matchFeatures(dPhoto, dS, 'Unique', true, 'MaxRatio', 0.95);
if isempty(m), return; end
pP = double(vPhoto(m(:,1)).Location);
pS = double(vS(m(:,2)).Location);
% Score: raw match count + 10x projective-inlier count.  Correctly
% oriented synth has lots of geometrically consistent matches; a
% mirrored synth has scattered, mostly inconsistent matches.
nRaw = size(m, 1);
nInl = 0;
if size(pP,1) >= 8
    try
        [~, inl] = estimateGeometricTransform2D(pP, pS, ...
            'projective','MaxNumTrials',3000,'MaxDistance',20, ...
            'Confidence',90);
        nInl = nnz(inl);
    catch
    end
end
nMatches = nRaw + 10*nInl;
end

function g = adaptiveEqualizeInline(g)
try, g = adapthisteq(g, 'ClipLimit', 0.01, 'NumTiles', [8 8]); return; catch, end
try, g = histeq(g); return; catch, end
try, g = imadjust(g); catch, end
end

function [synth, synthMask, uLo, vLo, dx] = renderOrthoForFlipInline(S)
% Render the synthetic ortho image used by the manual picker.  Mirrors
% the geometry in stage_ortho but takes everything as arguments so the
% picker's Flip-view button can re-render with an updated plane normal
% without going through the full pipeline.
c = S.plane.c; u = S.plane.u; v = S.plane.v;
d = S.xyz - c;
up = d * u.'; vp = d * v.'; wp = d * S.plane.n.';
uB = prctile(up, [0.5 99.5]); uLo = uB(1); uHi = uB(2);
vB = prctile(vp, [0.5 99.5]); vLo = vB(1); vHi = vB(2);
Wsyn = max(400, round(S.autoRenderWidth));
dx   = (uHi - uLo) / Wsyn;
Hsyn = max(50, round((vHi - vLo)/dx));
ix = floor((up - uLo)/dx) + 1;
iy = floor((vp - vLo)/dx) + 1;
keep = ix>=1 & ix<=Wsyn & iy>=1 & iy<=Hsyn;
ix = ix(keep); iy = iy(keep);
rgbK = S.rgb(keep,:); wD = wp(keep);
[~, ord] = sort(wD, 'descend');
ix = ix(ord); iy = iy(ord); rgbK = rgbK(ord,:);
lin = sub2ind([Hsyn Wsyn], iy, ix);
synth = zeros(Hsyn, Wsyn, 3, 'uint8');
for ch = 1:3
    Cc = zeros(Hsyn, Wsyn, 'uint8');
    Cc(lin) = rgbK(:,ch);
    synth(:,:,ch) = Cc;
end
valid = any(synth>0, 3);
synthMask = valid;
try
    synthMask = imclose(synthMask, strel('disk',5));
    synthMask = imfill(synthMask, 'holes');
    synthMask = imdilate(synthMask, strel('disk',8));
catch
end
try
    dil = imdilate(synth, strel('disk',3));
    for ch = 1:3
        O = synth(:,:,ch); D = dil(:,:,ch);
        O(~valid) = D(~valid);
        synth(:,:,ch) = O;
    end
catch
end
end

function out = buildCropToCloudMapsInline(S)
%BUILDCROPTOCLOUDMAPSINLINE  Maps between cropped-image pixels and the
%photo-recoloured cloud, packaged for downstream geologic-segmentation
%work.
%
%   out = buildCropToCloudMapsInline(S)
%
%   The returned struct lets future code go in either direction:
%
%     PIXEL -> POINT  (e.g. paint a region on the cropped image, then
%                      colour the corresponding cloud points)
%       k = out.pixelToPointIdx(py, px);     % 0 if no point under pixel
%       if k > 0
%           xyz = out.coloredCloudPoints(k, :);
%       end
%
%     POINT -> PIXEL  (e.g. find where each cloud point appears in the
%                      cropped image, to highlight a 3-D region in 2-D)
%       uv = out.pointToPixel(k, :);         % NaN if not in the crop
%
%   out.pixelToPointIdx is z-buffered (front-most point per pixel) and
%   indexes into out.coloredCloudPoints, which is a flat N x 3 single
%   array.  out.coloredCloudColors is the matching N x 3 uint8 RGB.
%
%   To re-project a future 3-D annotation back to the cropped image
%   without using the maps, use out.intrinsics + out.R_cw + out.t_cw +
%   out.cropBox - the same conventions photo2pc_gui uses internally
%   (R_cw maps world->camera, t_cw is the translation in camera frame).

out = struct();
cb = S.cropBox;
if isempty(cb) || numel(cb) < 4 || cb(3) <= 0 || cb(4) <= 0
    cb = [1 1 S.W S.H];
end
x0 = cb(1); y0 = cb(2); Wc = cb(3); Hc = cb(4);

% --- cropped image + co-registered masks ---------------------------
out.cropBox      = cb;
out.croppedImage = S.img(y0:y0+Hc-1, x0:x0+Wc-1, :);
if ~isempty(S.faceMask)
    out.croppedFaceMask = S.faceMask(y0:y0+Hc-1, x0:x0+Wc-1);
else
    out.croppedFaceMask = [];
end
if ~isempty(S.projCropMask)
    out.croppedProjMask = S.projCropMask(y0:y0+Hc-1, x0:x0+Wc-1);
else
    out.croppedProjMask = [];
end

% --- colored cloud arrays (full cloud, FRONT-WHEN-FULL count N) ----
if ~isempty(S.coloredCloud)
    Xfull = double(S.coloredCloud.Location);
    Cfull = S.coloredCloud.Color;
else
    Xfull = double(S.pc.Location);
    Cfull = S.pc.Color;
end
if isempty(Cfull)
    Cfull = uint8(repmat(200, size(Xfull,1), 3));
end
N = size(Xfull, 1);
out.coloredCloudPoints = single(Xfull);
out.coloredCloudColors = uint8(Cfull);

% --- project every cloud point into the cropped image --------------
[uv, fr] = projectPtsInline(Xfull, S.R_cw, S.t_cw, S.K);
Xc = (S.R_cw * Xfull.' + S.t_cw).';
depth = Xc(:,3);
uCrop = uv(:,1) - x0 + 1;          % cropped-image coords, 1-based
vCrop = uv(:,2) - y0 + 1;
ix = round(uCrop);
iy = round(vCrop);
inCrop = fr & ix >= 1 & ix <= Wc & iy >= 1 & iy <= Hc;

% Inverse map: pointToPixel(k, :) = [u v] in cropped-image coords, or
% NaN if the point is behind the camera or outside the crop.
pointToPixel = nan(N, 2, 'single');
pointToPixel(inCrop, 1) = single(uCrop(inCrop));
pointToPixel(inCrop, 2) = single(vCrop(inCrop));
out.pointToPixel = pointToPixel;

% Forward map: pixelToPointIdx(py, px) = index of the front-most cloud
% point that projects to that pixel, or 0 if no point.  Vectorised
% z-buffer: sort by descending depth so MATLAB's last-write-wins on
% duplicate pixels keeps the smallest depth (nearest point).
pixelToPointIdx = zeros(Hc, Wc, 'uint32');
id = find(inCrop);
if ~isempty(id)
    [~, ord] = sort(depth(id), 'descend');
    idSorted  = id(ord);
    linIdx    = sub2ind([Hc Wc], iy(idSorted), ix(idSorted));
    pixelToPointIdx(linIdx) = uint32(idSorted);
end
out.pixelToPointIdx = pixelToPointIdx;

% Convenience: per-pixel XYZ (NaN where no point hits the pixel).
pixelToPointXYZ = nan(Hc, Wc, 3, 'single');
msk = pixelToPointIdx > 0;
if any(msk(:))
    src = pixelToPointIdx(msk);
    Xs  = single(Xfull);
    for ch = 1:3
        Cc = nan(Hc, Wc, 'single');
        Cc(msk) = Xs(src, ch);
        pixelToPointXYZ(:,:,ch) = Cc;
    end
end
out.pixelToPointXYZ = pixelToPointXYZ;

% --- geometry needed to re-project new 3-D annotations later -------
out.intrinsics = S.intr;
out.K          = S.K;
out.worldPose  = S.pose;
out.R_cw       = S.R_cw;
out.t_cw       = S.t_cw;
out.fullImageSize = [S.H S.W];     % (H, W) of the original photo
out.reprojRMS  = S.rms;
out.imagePoints = S.imPts;
out.worldPoints = S.wPts;
end
