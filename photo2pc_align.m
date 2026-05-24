function result = photo2pc_align(cloudFile, imgFile, varargin)
%PHOTO2PC_ALIGN  Register an RGB photo to a colored 3-D point cloud.
%
%   result = photo2pc_align(cloudFile, imgFile)
%   result = photo2pc_align(cloudFile, imgFile, Name, Value, ...)
%
%   cloudFile may be a .ply or a .mat. For .mat the loader auto-detects:
%     - a pointCloud / pcshow-compatible object stored as any variable, OR
%     - two arrays: XYZ (3xN or Nx3, numeric) + RGB (3xN or Nx3, uint8 or
%       0..1 double). Common variable name pairs are recognised
%       (facePts/faceRGB, xyz/rgb, points/colors, etc.).
%
%   Name-Value options
%     'Mode'           'auto' | 'manual' | 'autoThenManual' (default)
%     'FocalLengthPx'  Focal length in pixels (fx=fy). Overrides EXIF.
%     'FocalLengthMM' + 'SensorWidthMM'  : alternative way to set focal.
%     'Intrinsics'     A cameraIntrinsics object (overrides everything).
%     'Correspondences' struct('imagePoints',Nx2,'worldPoints',Nx3) to
%                      skip detection and solve directly.
%     'MaxCloudPoints' Cap for render/display speed (default 8e5).
%     'AutoRenderWidth' Synthetic render width in px (default 1500).
%     'ComputePixelMap' Build per-pixel -> 3-D point map (default true).
%     'OutputDir'      Where to save results (default = image folder).
%     'SaveResults'    If true, writes <stem>_p2pc_alignment.mat and
%                      <stem>_p2pc_overlay.png to OutputDir.  Default
%                      is FALSE - everything is returned in the result
%                      struct and nothing is written.
%     'Verbose'        Logical (default true).
%
%   result fields (see in-line comments below).

% ---------------------------------------------------------------- args
p = inputParser;
p.addRequired ('cloudFile', @(s)ischar(s)||isstring(s));
p.addRequired ('imgFile',   @(s)ischar(s)||isstring(s));
p.addParameter('Mode','autoThenManual', @(s)ischar(s)||isstring(s));
p.addParameter('FocalLengthPx',[], @(x)isempty(x)||isscalar(x));
p.addParameter('FocalLengthMM',[], @(x)isempty(x)||isscalar(x));
p.addParameter('SensorWidthMM',[], @(x)isempty(x)||isscalar(x));
p.addParameter('Intrinsics',[]);
p.addParameter('Correspondences',[]);
p.addParameter('MaxCloudPoints',8e5, @isscalar);
p.addParameter('AutoRenderWidth',1500, @isscalar);
p.addParameter('ComputePixelMap',true, @islogical);
p.addParameter('OutputDir','', @(s)ischar(s)||isstring(s));
p.addParameter('SaveResults',false, @islogical);
p.addParameter('Verbose',true, @islogical);
p.parse(cloudFile,imgFile,varargin{:});
o = p.Results;
say = @(varargin) o.Verbose && fprintf([varargin{1} '\n'], varargin{2:end});

result = struct('status','failed','method','','intrinsics',[], ...
    'worldPose',[],'R_cw',[],'t_cw',[],'imagePoints',[], ...
    'worldPoints',[],'reprojRMS',NaN,'files',struct('mat','','png',''));

% ---------------------------------------------------------------- load
if exist(cloudFile,'file')~=2
    error('photo2pc_align:noCloud','Cloud file not found: %s',cloudFile);
end
if exist(imgFile,'file')~=2
    error('photo2pc_align:noIMG','Image not found: %s',imgFile);
end
pc  = loadCloud(cloudFile, say);
img = imread(imgFile);
if size(img,3)==1, img = repmat(img,1,1,3); end
H = size(img,1); W = size(img,2);
say('Loaded cloud (%d pts) and image (%d x %d).', pc.Count, W, H);

% downsample for speed (the FULL cloud is kept in 'pc' for recolouring)
pcF = pc;
if pc.Count > o.MaxCloudPoints
    pcF = pcdownsample(pc,'random',o.MaxCloudPoints/pc.Count);
    say('Downsampled cloud to %d pts for processing.', pcF.Count);
end
xyz = double(pcF.Location);
rgb = pcF.Color;
if isempty(rgb), rgb = uint8(repmat(200,size(xyz,1),3)); end

% ---------------------------------------------------------------- K
intr = resolveIntrinsics();
result.intrinsics = intr;
K = intrMatrix(intr);
say('Intrinsics: f=%.1f px, pp=(%.1f, %.1f).', K(1,1), K(1,3), K(2,3));

% ---------------------------------------------------- correspond + solve
% The auto pipeline can fail four ways: (1) too few raw matches,
% (2) PnP rejects the matches outright, (3) RANSAC keeps so few inliers
% that the fit is meaningless, or (4) the fit looks numerically valid
% but explains the data poorly (huge reprojection RMS) - typically when
% SIFT locked onto the wrong region of the face.  Any of these in
% 'autothenmanual' mode drops through to the manual picker so the user
% can rescue the alignment instead of being left with status='failed'.
mode = lower(string(o.Mode));
imPts = []; wPts = []; meth = '';
worldPose = []; inIdx = []; pnpOk = false; nOrig = 0;
autoFailReason = '';

if ~isempty(o.Correspondences)
    imPts = double(o.Correspondences.imagePoints);
    wPts  = double(o.Correspondences.worldPoints);
    meth  = 'supplied';
    nOrig = size(imPts,1);
    [worldPose, inIdx, pnpOk] = solvePnP(imPts, wPts, intr, 8);
elseif mode=="auto" || mode=="autothenmanual"
    [imPtsA, wPtsA] = autoCorrespond();
    nA = size(imPtsA,1);
    if nA < 6
        autoFailReason = sprintf('only %d raw correspondences', nA);
    else
        [poseA, inA, okA] = solvePnP(imPtsA, wPtsA, intr, 8);
        if ~okA
            autoFailReason = 'PnP rejected the auto matches';
        elseif numel(inA) < max(8, ceil(0.25*nA))
            autoFailReason = sprintf('only %d / %d PnP inliers', numel(inA), nA);
        else
            [Rtmp, Ttmp] = poseToExtrinsics(poseA);
            rmsA = quickRMS(imPtsA(inA,:), wPtsA(inA,:), Rtmp, Ttmp, K);
            rmsLimit = max(20, 0.01*max(W,H));
            if ~isfinite(rmsA) || rmsA > rmsLimit
                autoFailReason = sprintf( ...
                    'reprojection RMS = %.1f px > limit %.1f px', ...
                    rmsA, rmsLimit);
            else
                imPts = imPtsA;  wPts = wPtsA;
                nOrig = nA;
                worldPose = poseA;  inIdx = inA;  pnpOk = true;
                meth = 'auto-ortho-SIFT';
                say('Auto attempt OK: %d/%d inliers, RMS = %.2f px.', ...
                    numel(inA), nA, rmsA);
            end
        end
    end
    if ~pnpOk
        say('Auto attempt failed: %s.', autoFailReason);
    end
end

% Fall back to manual whenever the auto/supplied path did not produce a
% trustworthy pose AND the mode allows manual picking.
if ~pnpOk && (mode=="manual" || mode=="autothenmanual")
    if mode == "autothenmanual"
        say('Falling back to manual correspondence picking.');
    end
    [imPtsM, wPtsM, cancelled] = manualCorrespond();
    if cancelled, result.status='cancelled'; return; end
    if size(imPtsM,1) < 6
        say('Not enough correspondences (need >=6, got %d).', size(imPtsM,1));
        return;
    end
    imPts = imPtsM;  wPts = wPtsM;  meth = 'manual';
    nOrig = size(imPts,1);
    % Manual picks are vetted by a human, so we trust every point.
    % Loosen the RANSAC threshold and add a direct (non-RANSAC)
    % fallback so the solver doesn't reject good picks just because
    % they are 30-50 px noisy on a 4080-wide photo.
    [worldPose, inIdx, pnpOk] = solvePnP(imPts, wPts, intr, 60);
    if ~pnpOk || isempty(inIdx) || numel(inIdx) < min(6, nOrig)
        [worldPose, inIdx, pnpOk] = solvePnP(imPts, wPts, intr, 1e6);
    end
end

if ~pnpOk
    if ~isempty(autoFailReason)
        say('PnP failed (auto: %s).', autoFailReason);
    else
        say('PnP solver failed (status from estworldpose).');
    end
    return;
end
if isempty(imPts) || size(imPts,1) < 6
    say('Not enough correspondences (need >=6, got %d).', size(imPts,1));
    return;
end
imPts = imPts(inIdx,:);  wPts = wPts(inIdx,:);
say('PnP: %d / %d inliers (%s).', numel(inIdx), nOrig, meth);

% refine (motion-only bundle adjustment), tolerate version differences
try
    worldPose = bundleAdjustmentMotion(wPts, imPts, worldPose, intr);
    say('Refined pose with bundleAdjustmentMotion.');
catch ME
    say('Refinement skipped (%s).', ME.message);
end

% ---------------------------------------------------------- extrinsics
[R_cw, t_cw] = poseToExtrinsics(worldPose);

% reprojection error
[uv,front] = projectPts(wPts, R_cw, t_cw, K);
e = hypot(uv(front,1)-imPts(front,1), uv(front,2)-imPts(front,2));
rms = sqrt(mean(e.^2));

result.status='solved';  result.method=meth;
result.worldPose=worldPose;  result.R_cw=R_cw;  result.t_cw=t_cw;
result.imagePoints=imPts;    result.worldPoints=wPts;
result.reprojRMS=rms;
say('Reprojection RMS = %.2f px on %d inliers.', rms, size(imPts,1));

% ---------------------------------------------------------- outputs
result.coloredCloud = recolorCloud();   % photo colours onto full cloud

if o.ComputePixelMap
    [~,idxMap] = renderPerspective(R_cw,t_cw, H, W);
    result.pixelToPointIdx = idxMap;     % H x W, 0 = no point
    result.pixelXYZsource  = xyz;        % rows indexed by idx
    PM = nan(H,W,3,'single');  msk = idxMap>0;
    xyzS = single(xyz);
    for ch = 1:3
        Cc = nan(H,W,'single');
        Cc(msk) = xyzS(idxMap(msk),ch);
        PM(:,:,ch) = Cc;
    end
    result.pixelToPointXYZ = PM;
    say('Built per-pixel 3-D map (%d/%d px hit a point).', nnz(msk), H*W);
end

% Writes only happen when the caller explicitly asks. Default is to
% return everything in the result struct and write nothing.
if o.SaveResults
    outdir = char(o.OutputDir);
    if isempty(outdir), outdir = fileparts(imgFile); end
    if isempty(outdir), outdir = pwd; end
    [~,stem] = fileparts(imgFile);
    pngPath = fullfile(outdir, [stem '_p2pc_overlay.png']);
    matPath = fullfile(outdir, [stem '_p2pc_alignment.mat']);

    qcOverlay(pngPath);
    result.files.png = pngPath;

    alignment = result;
    if isfield(alignment,'pixelToPointXYZ')
        alignment = rmfield(alignment,'pixelToPointXYZ');
    end
    save(matPath,'alignment','-v7.3');
    result.files.mat = matPath;
    say('Saved: %s  and  %s', matPath, pngPath);
end

% =====================================================================
%  NESTED HELPERS  (share workspace with the main function)
% =====================================================================

    function intr = resolveIntrinsics()
        if ~isempty(o.Intrinsics) && isa(o.Intrinsics,'cameraIntrinsics')
            intr = o.Intrinsics; return;
        end
        f = [];
        if ~isempty(o.FocalLengthPx)
            f = o.FocalLengthPx;
        elseif ~isempty(o.FocalLengthMM) && ~isempty(o.SensorWidthMM)
            f = o.FocalLengthMM / o.SensorWidthMM * W;
        else
            f = focalFromEXIF(imgFile, W);
        end
        if isempty(f) || ~isfinite(f) || f<=0
            f = 1.2*max(W,H);
            warning('photo2pc_align:guessFocal', ...
                ['No focal length available; guessing f=%.0f px. ', ...
                 'Pass ''FocalLengthPx'' for accuracy.'], f);
        end
        intr = cameraIntrinsics([f f], [W/2 H/2], [H W]);
    end

    % ===================== AUTO CORRESPONDENCE ========================
    % Fit a plane to the cloud (works very well for ~planar tunnel
    % faces), render an orthographic colour image of the face along
    % the plane normal, SIFT-match that image to the photo, then lift
    % matched ortho pixels back to 3-D via the analytic ortho mapping.
    function [ip, wp] = autoCorrespond()
        ip = []; wp = [];
        % Seed RNG so RANSAC inside estimateGeometricTransform2D and
        % estworldpose is reproducible across runs.
        try, rng(0, 'twister'); catch, end
        % Resolve synth working width once - used by both the test
        % render in chooseFrontOrientation and the main ortho render.
        Wsyn = max(400, round(o.AutoRenderWidth));
        try
            % --- 1. dominant plane via SVD of mean-centred cloud
            c   = mean(xyz,1);
            [~,~,V] = svd(xyz - c, "econ");
            u   = V(:,1).'; u = u / norm(u);
            v   = V(:,2).'; v = v / norm(v);
            nrm = V(:,3).'; nrm = nrm/norm(nrm);
            % Orient so v points "down" in world Z (upright ortho render)
            % and the {u,v,nrm} frame is right-handed.
            [u, v, nrm] = orientPlaneAxes(u, v, nrm);
            % SVD doesn't constrain the SIGN of nrm.  Test both
            % directions and keep whichever produces more SIFT
            % matches against the photo - that's the front of the
            % face (the side the camera was looking at).  Use the FULL
            % cloud (not the downsampled pcF) so the small test render
            % is dense enough for SIFT.
            xyzFull = double(pc.Location);
            rgbFull = pc.Color;
            if isempty(rgbFull)
                rgbFull = uint8(repmat(200,size(xyzFull,1),3));
            end
            [u, v, nrm, oinfo] = chooseFrontOrientation(u, v, nrm, xyzFull, rgbFull, img);
            if oinfo.flipped
                say('Auto: orientation FLIPPED (front=%d back=%d matches).', ...
                    oinfo.nFront, oinfo.nBack);
            else
                say('Auto: orientation kept (front=%d back=%d matches).', ...
                    oinfo.nFront, oinfo.nBack);
            end
            % --- 2. ortho coordinates of every point
            d   = xyz - c;
            up  = d * u.';                 % N x 1
            vp  = d * v.';                 % N x 1
            wp_ = d * nrm.';               % depth along normal

            % crop tails (robust extents).  Vector-form prctile sorts the
            % input only ONCE per axis instead of twice.
            uB = prctile(up, [0.5 99.5]); uLo = uB(1); uHi = uB(2);
            vB = prctile(vp, [0.5 99.5]); vLo = vB(1); vHi = vB(2);
            uExt = uHi - uLo;  vExt = vHi - vLo;
            if uExt<=0 || vExt<=0
                say('Auto: degenerate plane extents, skipping.');
                return;
            end

            % --- 3. render to a moderate-resolution ortho image
            dx   = uExt / Wsyn;
            Hsyn = max(50, round(vExt/dx));
            ix   = floor((up - uLo)/dx) + 1;       % 1..Wsyn
            iy   = floor((vp - vLo)/dx) + 1;       % 1..Hsyn
            keep = ix>=1 & ix<=Wsyn & iy>=1 & iy<=Hsyn;
            ix = ix(keep); iy = iy(keep);
            rgbK = rgb(keep,:);  wDepth = wp_(keep);

            % near-side first via z-buffer.  Sort descending so MATLAB's
            % indexed-assignment last-write-wins on duplicate pixels keeps
            % the smallest depth (the nearest point) - no explicit loop.
            lin = sub2ind([Hsyn Wsyn], iy, ix);
            idxBuf = zeros(Hsyn,Wsyn);
            [~,ord]      = sort(wDepth,'descend');
            lin2         = lin(ord);
            idxKeep      = find(keep);
            idxBuf(lin2) = idxKeep(ord);
            valid = idxBuf>0;
            synth = zeros(Hsyn,Wsyn,3,'uint8');
            if ~any(valid(:))
                say('Auto: ortho render is empty.');
                return;
            end
            srcIdx = idxBuf(valid);
            for ch=1:3
                Cch = zeros(Hsyn,Wsyn,'uint8');
                Cch(valid) = rgb(srcIdx,ch);
                synth(:,:,ch) = Cch;
            end
            % fill small holes so SIFT has continuous regions
            synth = fillHoles(synth, valid, 3);

            % --- 4. SIFT match: real photo vs synthetic ortho.
            % Downscale photo BEFORE rgb2gray (smaller image to process)
            % and equalise both with CLAHE so detectors compare gradient
            % structure rather than absolute brightness.
            scl = Wsyn / W;
            gRs = adaptiveEqualize(rgb2gray(imresize(img, scl)));
            gS  = adaptiveEqualize(rgb2gray(synth));

            % Detect features on multiple detectors. Synthetic
            % ortho-renders look "different" from real photos so a
            % single detector often under-matches; we union SIFT, KAZE
            % and (when available) ORB descriptor matches before RANSAC.
            [pr, ps] = multiDetectorMatch(gRs, gS, say);
            if isempty(pr) || size(pr,1) < 8
                say('Auto: only %d raw matches across all detectors.', size(pr,1));
                return;
            end
            say('Auto: combined raw matches = %d', size(pr,1));

            % geometric verification with a projective fit (planar
            % scene viewed under perspective -> homography is the
            % correct model; affine is too strict).
            inlierCount = 0;
            try
                [~,inl] = estimateGeometricTransform2D(pr, ps, ...
                    'projective','MaxNumTrials',8000,'MaxDistance',6, ...
                    'Confidence',99);
                pr = pr(inl,:); ps = ps(inl,:); inlierCount = size(pr,1);
            catch
                try
                    [~,inl] = estimateGeometricTransform(pr, ps, ...
                        'projective','MaxNumTrials',8000,'MaxDistance',6);
                    pr = pr(inl,:); ps = ps(inl,:); inlierCount = size(pr,1);
                catch
                end
            end
            if inlierCount < 8
                % last resort: try affine on the raw set, looser tol
                try
                    [~,inl] = estimateGeometricTransform2D(pr, ps, ...
                        'affine','MaxNumTrials',8000,'MaxDistance',8);
                    pr = pr(inl,:); ps = ps(inl,:); inlierCount = size(pr,1);
                catch
                end
            end
            if inlierCount < 8
                say('Auto: only %d inliers after geometric check.', inlierCount);
                return;
            end

            % --- 5. lift synth pixels back to 3-D via analytic ortho map.
            % We don't snap to the rendered z-buffered idxBuf (which is
            % sparse) - we re-solve directly: given pixel (psx,psy),
            % uCoord = uLo + (psx-0.5)*dx ; vCoord = vLo + (psy-0.5)*dx
            % then the 3-D point on the plane is c + uCoord*u + vCoord*v.
            % We then find the nearest cloud point for robustness.
            psx = ps(:,1); psy = ps(:,2);
            uC  = uLo + (psx - 0.5)*dx;
            vC  = vLo + (psy - 0.5)*dx;
            planePts = c + uC*u + vC*v;       % Nx3 on the plane

            % snap to nearest actual cloud point so the 3-D coord is real
            try
                kdt = KDTreeSearcher(xyz);
                nnI = knnsearch(kdt, planePts);
            catch
                nnI = zeros(size(planePts,1),1);
                for ii=1:size(planePts,1)
                    d2 = sum((xyz - planePts(ii,:)).^2, 2);
                    [~,nnI(ii)] = min(d2);
                end
            end
            wp = xyz(nnI,:);

            % rescale matched photo coords (downsized) back to original
            ip = pr / scl;
            say('Auto: %d candidate correspondences after verification.', size(ip,1));
        catch ME
            say('Auto correspond error: %s', ME.message);
            ip = []; wp = [];
        end
    end

    % ===================== MANUAL CORRESPONDENCE ======================
    % Classic figure + axes UI. Crucially we do NOT use rotate3d here:
    % rotate3d puts the entire figure into a "mode" that silently
    % suppresses every ButtonDownFcn / WindowButtonDownFcn callback,
    % so clicks never reach our picker. Instead we use the per-axes
    % enableDefaultInteractivity / disableDefaultInteractivity API
    % (which leaves ButtonDownFcn working) and attach the pick
    % handlers directly on the axes.
    function [ip, wp, cancelled] = manualCorrespond()
        ip = zeros(0,2); wp = zeros(0,3); cancelled = true;

        S.mode = 'idle';        % 'idle' | 'image' | 'cloud'
        S.pendingPix = [];
        S.hImgMarks  = gobjects(0);
        S.hCloudMarks= gobjects(0);
        S.hImgLabels = gobjects(0);
        S.hCloudLabels=gobjects(0);

        % Dark-theme palette (kept in sync with photo2pc_gui's S.theme)
        BG     = [0.13 0.14 0.17];
        PANEL  = [0.18 0.19 0.23];
        TXT    = [0.92 0.93 0.96];
        INPUT  = [0.22 0.24 0.29];
        SUCCESS= [0.30 0.62 0.42];
        WARN   = [0.62 0.32 0.30];
        AXBG   = [0.10 0.11 0.14];
        BORDER = [0.32 0.34 0.40];

        fig = figure('Name','Photo <-> Point-Cloud manual picking', ...
            'NumberTitle','off','Color', BG, ...
            'WindowStyle','normal','MenuBar','none','ToolBar','figure', ...
            'CloseRequestFcn',@(~,~)finish(true));
        try
            fig.WindowState = 'maximized';
        catch
        end

        % --- image axes (left)
        axI = subplot('Position',[0.03 0.18 0.46 0.78], 'Parent', fig);
        imshow(img,'Parent',axI);
        title(axI,'PHOTO', 'Color', TXT);
        set(axI, 'Color', AXBG, 'XColor', TXT, 'YColor', TXT);
        hImg = findobj(axI,'Type','image');
        set(hImg,'HitTest','off','PickableParts','none');
        try, disableDefaultInteractivity(axI); catch, end
        axI.ButtonDownFcn = @(~,~)onImg();

        % --- cloud axes (right)
        axP = subplot('Position',[0.51 0.18 0.46 0.78], 'Parent', fig);
        scatter3(axP, xyz(:,1),xyz(:,2),xyz(:,3), 6, ...
            double(rgb)/255, '.', ...
            'HitTest','off','PickableParts','none');
        axis(axP,'equal','vis3d'); grid(axP,'on');
        set(axP, 'Color', AXBG, ...
            'XColor', TXT, 'YColor', TXT, 'ZColor', TXT, ...
            'GridColor', BORDER);
        title(axP,'POINT CLOUD  (drag to rotate; click to pick when armed)', ...
            'Color', TXT);
        try, enableDefaultInteractivity(axP); catch, end
        axP.ButtonDownFcn = @(~,~)onCloud();
        faceView();

        % --- control panel
        msgLbl = uicontrol(fig,'Style','text','Units','normalized', ...
            'Position',[0.03 0.10 0.94 0.05], ...
            'FontSize',11,'HorizontalAlignment','left', ...
            'BackgroundColor', PANEL, 'ForegroundColor', TXT);

        btnW = 0.13; btnH = 0.06; btnY = 0.025; gap = 0.01;
        x0 = 0.03;
        uicontrol(fig,'Style','pushbutton','Units','normalized', ...
            'Position',[x0+0*(btnW+gap) btnY btnW btnH], ...
            'String','1) Pick PHOTO point','FontSize',10, ...
            'BackgroundColor', INPUT, 'ForegroundColor', TXT, ...
            'Callback',@(~,~)arm('image'));
        uicontrol(fig,'Style','pushbutton','Units','normalized', ...
            'Position',[x0+1*(btnW+gap) btnY btnW btnH], ...
            'String','2) Pick 3-D point','FontSize',10, ...
            'BackgroundColor', INPUT, 'ForegroundColor', TXT, ...
            'Callback',@(~,~)arm('cloud'));
        uicontrol(fig,'Style','pushbutton','Units','normalized', ...
            'Position',[x0+2*(btnW+gap) btnY btnW btnH], ...
            'String','Undo last','FontSize',10, ...
            'BackgroundColor', INPUT, 'ForegroundColor', TXT, ...
            'Callback',@(~,~)undo());
        uicontrol(fig,'Style','pushbutton','Units','normalized', ...
            'Position',[x0+3*(btnW+gap) btnY btnW btnH], ...
            'String','Reset view','FontSize',10, ...
            'BackgroundColor', INPUT, 'ForegroundColor', TXT, ...
            'Callback',@(~,~)faceView());
        uicontrol(fig,'Style','pushbutton','Units','normalized', ...
            'Position',[x0+4*(btnW+gap) btnY btnW btnH], ...
            'String','Solve & Accept','FontSize',10, ...
            'FontWeight','bold', ...
            'BackgroundColor', SUCCESS, 'ForegroundColor', TXT, ...
            'Callback',@(~,~)accept());
        uicontrol(fig,'Style','pushbutton','Units','normalized', ...
            'Position',[x0+5*(btnW+gap) btnY btnW btnH], ...
            'String','Cancel','FontSize',10, ...
            'BackgroundColor', WARN, 'ForegroundColor', TXT, ...
            'Callback',@(~,~)finish(true));

        % Pick handlers are attached at the axes level (see above);
        % we don't use a figure-level WindowButtonDownFcn because that
        % is what rotate3d / zoom would suppress.

        setMsg();
        uiwait(fig);
        if isvalid(fig), delete(fig); end

        % ---------- helpers ----------
        function faceView()
            try
                cc = mean(xyz,1);
                [~,~,V] = svd(xyz - cc, "econ");
                uF = V(:,1).'; uF = uF/norm(uF);
                vF = V(:,2).'; vF = vF/norm(vF);
                nF = V(:,3).'; nF = nF/norm(nF);
                [~, ~, nF] = orientPlaneAxes(uF, vF, nF);
                if abs(nF(3))>0.9, up=[0 1 0]; else, up=[0 0 1]; end
                view(axP, nF);
                axP.CameraUpVector = up;
                axis(axP,'equal','vis3d');
            catch
            end
        end

        function setMsg()
            switch S.mode
                case 'image'
                    s = 'ARMED: click the feature in the PHOTO (left).';
                case 'cloud'
                    s = 'ARMED: click the SAME point in the CLOUD (right). Rotation is paused so the click registers as a pick.';
                otherwise
                    s = 'Drag the cloud to rotate. Then click "1) Pick PHOTO point".';
            end
            msgLbl.String = sprintf('Pairs: %d / 6 minimum     |     %s', size(ip,1), s);
        end

        function arm(which)
            S.mode = which;
            % While armed for a CLOUD pick, suspend drag-to-rotate so a
            % single click goes straight to ButtonDownFcn. While armed
            % for an IMAGE pick (or idle), leave cloud interactions on
            % so the user can rotate freely between picks.
            try
                if strcmp(which,'cloud')
                    disableDefaultInteractivity(axP);
                else
                    enableDefaultInteractivity(axP);
                end
            catch
            end
            setMsg();
        end

        function onImg()
            if ~strcmp(S.mode,'image'), return; end
            cp = axI.CurrentPoint;
            x = min(max(cp(1,1),1), W);
            y = min(max(cp(1,2),1), H);
            S.pendingPix = [x y];
            hold(axI,'on');
            S.hImgMarks(end+1) = plot(axI, x, y, '+', ...
                'Color',[0 1 0], 'MarkerSize',14, 'LineWidth',2);
            S.hImgLabels(end+1) = text(axI, x, y, ...
                sprintf('  %d', size(ip,1)+1), ...
                'Color',[0 1 0], 'FontWeight','bold');
            hold(axI,'off');
            arm('cloud');
        end

        function onCloud()
            if ~strcmp(S.mode,'cloud') || isempty(S.pendingPix), return; end
            cp = axP.CurrentPoint;  % [near; far] in world coords
            a = cp(1,:);  b = cp(2,:);
            dv = b - a;  dv = dv / norm(dv);
            d  = xyz - a;
            perp = d - (d*dv.') * dv;
            [~,k] = min(sum(perp.^2, 2));

            ip(end+1,:) = S.pendingPix; %#ok<AGROW>
            wp(end+1,:) = xyz(k,:);     %#ok<AGROW>

            hold(axP,'on');
            S.hCloudMarks(end+1) = plot3(axP, xyz(k,1), xyz(k,2), xyz(k,3), ...
                'o', 'Color',[0 1 0], 'MarkerSize',11, 'LineWidth',2);
            S.hCloudLabels(end+1) = text(axP, xyz(k,1), xyz(k,2), xyz(k,3), ...
                sprintf('  %d', size(ip,1)), ...
                'Color',[0 1 0], 'FontWeight','bold');
            hold(axP,'off');
            S.pendingPix = [];
            arm('image');           % auto-advance: next click is on the photo
        end

        function undo()
            % undo the most recent change: a pending pixel, or the last pair
            if ~isempty(S.pendingPix)
                S.pendingPix = [];
                if ~isempty(S.hImgMarks)
                    delTrailing(S.hImgMarks); S.hImgMarks(end)=[];
                end
                if ~isempty(S.hImgLabels)
                    delTrailing(S.hImgLabels); S.hImgLabels(end)=[];
                end
            elseif ~isempty(ip)
                ip(end,:)=[]; wp(end,:)=[];
                if ~isempty(S.hImgMarks)
                    delTrailing(S.hImgMarks); S.hImgMarks(end)=[];
                end
                if ~isempty(S.hImgLabels)
                    delTrailing(S.hImgLabels); S.hImgLabels(end)=[];
                end
                if ~isempty(S.hCloudMarks)
                    delTrailing(S.hCloudMarks); S.hCloudMarks(end)=[];
                end
                if ~isempty(S.hCloudLabels)
                    delTrailing(S.hCloudLabels); S.hCloudLabels(end)=[];
                end
            end
            arm('image'); setMsg();
        end

        function accept()
            if size(ip,1) >= 6
                finish(false);
            else
                msgbox(sprintf('Need at least 6 pairs (have %d).', size(ip,1)), ...
                    'More pairs needed','warn','modal');
            end
        end

        function finish(isCancel)
            cancelled = isCancel;
            if isCancel
                ip = zeros(0,2); wp = zeros(0,3);
            end
            uiresume(fig);
        end
    end

    % ===================== QC OVERLAY =================================
    function qcOverlay(pngPath)
        fg = figure('Visible','off','Color','w','Name','p2pc QC');
        ax = axes(fg); imshow(img,'Parent',ax); hold(ax,'on');
        [uvA,fr] = projectPts(xyz, R_cw, t_cw, K);
        inb = fr & uvA(:,1)>=1 & uvA(:,1)<=W & uvA(:,2)>=1 & uvA(:,2)<=H;
        scatter(ax, uvA(inb,1), uvA(inb,2), 2, double(rgb(inb,:))/255, '.');
        plot(ax, imPts(:,1), imPts(:,2), 'g+', 'MarkerSize',10, 'LineWidth',1.5);
        [uvC,~] = projectPts(wPts, R_cw, t_cw, K);
        plot(ax, uvC(:,1), uvC(:,2), 'ro', 'MarkerSize',8, 'LineWidth',1);
        for ii=1:size(imPts,1)
            plot(ax,[imPts(ii,1) uvC(ii,1)],[imPts(ii,2) uvC(ii,2)], ...
                'y-','LineWidth',1);
        end
        title(ax, sprintf(['Photo + projected cloud  |  RMS = %.2f px  ', ...
            '|  %d inliers'], rms, size(imPts,1)));
        hold(ax,'off');
        try exportgraphics(ax, pngPath, 'Resolution',150); catch, end
        close(fg);
    end

    function cc = recolorCloud()
        Xall = double(pc.Location);
        [uvA,fr] = projectPts(Xall, R_cw, t_cw, K);
        ix=round(uvA(:,1)); iy=round(uvA(:,2));
        inb = fr & ix>=1 & ix<=W & iy>=1 & iy<=H;
        col = pc.Color;
        if isempty(col), col = uint8(repmat(200,size(Xall,1),3)); end
        lin = sub2ind([H W], iy(inb), ix(inb));
        Rc=img(:,:,1); Gc=img(:,:,2); Bc=img(:,:,3);
        col(inb,1)=Rc(lin); col(inb,2)=Gc(lin); col(inb,3)=Bc(lin);
        cc = pointCloud(Xall,'Color',col);
    end

    function [im, idxMap] = renderPerspective(Rcw, tcw, Hr, Wr)
        [uv,front] = projectPts(xyz, Rcw, tcw, K);
        ix = round(uv(:,1)); iy = round(uv(:,2));
        inb = front & ix>=1 & ix<=Wr & iy>=1 & iy<=Hr;
        Xc  = (Rcw*xyz.' + tcw).';
        depth = Xc(:,3);
        im = zeros(Hr,Wr,3,'uint8');
        idxMap = zeros(Hr,Wr);
        id = find(inb);
        if ~isempty(id)
            % Sort by descending depth so last-write-wins on duplicate
            % pixels keeps the nearest cloud point - vectorized z-buffer.
            [~, ord] = sort(depth(id), 'descend');
            idSort   = id(ord);
            linIdx   = sub2ind([Hr Wr], iy(idSort), ix(idSort));
            idxMap(linIdx) = idSort;
            msk = idxMap > 0;
            src = idxMap(msk);
            for ch = 1:3
                Cc = zeros(Hr, Wr, 'uint8');
                Cc(msk) = rgb(src, ch);
                im(:,:,ch) = Cc;
            end
        end
    end

end  % ===================== local functions ==========================

% ---------------------------------------------------------- LOADER
function pc = loadCloud(path, say)
    [~,~,ext] = fileparts(char(path));
    ext = lower(ext);
    if strcmp(ext,'.ply')
        pc = pcread(char(path));
        return;
    end
    if ~strcmp(ext,'.mat')
        error('photo2pc_align:badExt','Unsupported cloud file extension: %s', ext);
    end
    S = load(char(path));
    % 1) any pointCloud-like object?
    fns = fieldnames(S);
    for k = 1:numel(fns)
        v = S.(fns{k});
        if isa(v,'pointCloud')
            pc = v; say('loadCloud: using pointCloud variable "%s".', fns{k}); return;
        end
    end
    % 2) explicit XYZ + RGB pairs
    candidatesXYZ = {'facePts','xyz','points','XYZ','pts','vertices','V'};
    candidatesRGB = {'faceRGB','rgb','colors','color','RGB','C'};
    XYZ = []; RGB = [];
    for k = 1:numel(candidatesXYZ)
        if isfield(S,candidatesXYZ{k}), XYZ = S.(candidatesXYZ{k}); break; end
    end
    for k = 1:numel(candidatesRGB)
        if isfield(S,candidatesRGB{k}), RGB = S.(candidatesRGB{k}); break; end
    end
    % 3) last-ditch: pick the biggest numeric 3xN/Nx3 arrays
    if isempty(XYZ)
        best = 0; pick = '';
        for k = 1:numel(fns)
            v = S.(fns{k});
            if isnumeric(v) && (size(v,1)==3 || size(v,2)==3) && numel(v)>best
                best = numel(v); pick = fns{k};
            end
        end
        if ~isempty(pick), XYZ = S.(pick); say('loadCloud: using "%s" as XYZ.', pick); end
    end
    if isempty(XYZ)
        error('photo2pc_align:noXYZ','Could not find XYZ array in %s', path);
    end
    if size(XYZ,1)==3 && size(XYZ,2)~=3, XYZ = XYZ.'; end
    XYZ = double(XYZ);
    if ~isempty(RGB)
        if size(RGB,1)==3 && size(RGB,2)~=3, RGB = RGB.'; end
        if isa(RGB,'double') || isa(RGB,'single')
            if max(RGB(:)) <= 1.0 + 1e-6
                RGB = uint8(round(RGB*255));
            else
                RGB = uint8(RGB);
            end
        end
        RGB = uint8(RGB);
        pc = pointCloud(XYZ,'Color',RGB);
    else
        pc = pointCloud(XYZ);
    end
end

function f = focalFromEXIF(imgFile, W)
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

function K = intrMatrix(intr)
% works on cameraIntrinsics from any reasonable MATLAB release
try
    K = intr.K;            % R2022b+
    if ~isnumeric(K) || ~isequal(size(K),[3 3])
        K = intr.IntrinsicMatrix.';
    end
catch
    K = intr.IntrinsicMatrix.';
end
end

function [pose, inIdx, ok] = solvePnP(imPts, wPts, intr, maxReprojErr)
if nargin < 4, maxReprojErr = 8; end
ok = false; pose = []; inIdx = [];
try
    [pose, inIdx, st] = estworldpose(imPts, wPts, intr, ...
        'MaxReprojectionError',maxReprojErr,'Confidence',99,'MaxNumTrials',5000);
    ok = (st == 0);
catch
    try
        [Rcw, tcw, inIdx, st] = estimateWorldCameraPose(imPts, wPts, intr, ...
            'MaxReprojectionError',maxReprojErr,'Confidence',99,'MaxNumTrials',5000);
        ok = (st == 0);
        if ok
            Rwc = Rcw.';
            twc = -Rwc * tcw(:);
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

function [R_cw, t_cw] = poseToExtrinsics(pose)
% works for both rigidtform3d (R is world->? convention)
try
    R = pose.R;            % rigidtform3d: world->camera? no -- it's the
                           % rotation of the camera in the world frame.
catch
    R = pose.Rotation;
end
try
    t = pose.Translation;
catch
    t = pose.T(4,1:3);
end
% camera-in-world: Pose.R is R_wc, Pose.Translation is camera position
R_cw = R.';
t_cw = -R_cw * t(:);
end

function [uv, front] = projectPts(X, R_cw, t_cw, K)
Xc = (R_cw*X.' + t_cw).';
front = Xc(:,3) > 1e-6;
z = Xc(:,3); z(~front) = 1;
u = K(1,1)*Xc(:,1)./z + K(1,3);
v = K(2,2)*Xc(:,2)./z + K(2,3);
uv = [u v];
end

function r = quickRMS(ip, wp, R_cw, t_cw, K)
% Reprojection RMS used to sanity-check an auto-mode PnP fit before
% accepting it.  Points that fall behind the camera are excluded.
[uv, fr] = projectPts(wp, R_cw, t_cw, K);
if ~any(fr), r = inf; return; end
e = hypot(uv(fr,1)-ip(fr,1), uv(fr,2)-ip(fr,2));
if isempty(e), r = inf; else, r = sqrt(mean(e.^2)); end
end

function [pr, ps] = multiDetectorMatch(gR, gS, say)
% Run SIFT, KAZE, and ORB on both images independently; concatenate
% the per-detector matched coordinates. Each detector finds different
% local structure, and a cloud-render-to-photo match benefits a lot
% from union. Outliers are filtered downstream by RANSAC.
pr = zeros(0,2); ps = zeros(0,2);

% --- SIFT (low MetricThreshold to find dim features in render) ---
try
    fR = detectSIFTFeatures(gR, 'ContrastThreshold', 0.0067);
    fS = detectSIFTFeatures(gS, 'ContrastThreshold', 0.0067);
    if fR.Count >= 20 && fS.Count >= 20
        [dR,vR] = extractFeatures(gR,fR);
        [dS,vS] = extractFeatures(gS,fS);
        m = matchFeatures(dR,dS,'Unique',true,'MaxRatio',0.95, ...
            'MatchThreshold',60);
        if ~isempty(m)
            pr = [pr; double(vR(m(:,1)).Location)];
            ps = [ps; double(vS(m(:,2)).Location)];
            say('   SIFT  matches=%d  (features R=%d S=%d)', ...
                size(m,1), fR.Count, fS.Count);
        end
    else
        say('   SIFT  features too few (R=%d S=%d)', fR.Count, fS.Count);
    end
catch ME
    say('   SIFT  skipped (%s)', ME.message);
end

% --- KAZE (often picks up where SIFT misses on synthetic renders) ---
try
    fR = detectKAZEFeatures(gR, 'Threshold', 0.0001);
    fS = detectKAZEFeatures(gS, 'Threshold', 0.0001);
    if fR.Count >= 20 && fS.Count >= 20
        [dR,vR] = extractFeatures(gR,fR);
        [dS,vS] = extractFeatures(gS,fS);
        m = matchFeatures(dR,dS,'Unique',true,'MaxRatio',0.95, ...
            'MatchThreshold',60);
        if ~isempty(m)
            pr = [pr; double(vR(m(:,1)).Location)];
            ps = [ps; double(vS(m(:,2)).Location)];
            say('   KAZE  matches=%d  (features R=%d S=%d)', ...
                size(m,1), fR.Count, fS.Count);
        end
    else
        say('   KAZE  features too few (R=%d S=%d)', fR.Count, fS.Count);
    end
catch ME
    say('   KAZE  skipped (%s)', ME.message);
end

% --- ORB ---
try
    fR = detectORBFeatures(gR);
    fS = detectORBFeatures(gS);
    if fR.Count >= 50 && fS.Count >= 50
        [dR,vR] = extractFeatures(gR,fR);
        [dS,vS] = extractFeatures(gS,fS);
        m = matchFeatures(dR,dS,'Unique',true,'MaxRatio',0.95, ...
            'MatchThreshold',80);
        if ~isempty(m)
            pr = [pr; double(vR(m(:,1)).Location)];
            ps = [ps; double(vS(m(:,2)).Location)];
            say('   ORB   matches=%d  (features R=%d S=%d)', ...
                size(m,1), fR.Count, fS.Count);
        end
    else
        say('   ORB   features too few (R=%d S=%d)', fR.Count, fS.Count);
    end
catch ME
    say('   ORB   skipped (%s)', ME.message);
end
end

function im = fillHoles(im, valid, radius)
% propagate colours into small gaps so SIFT has continuous regions
if nargin < 3, radius = 1; end
if ~any(~valid(:)), return; end
orig = im;
try
    dil = imdilate(im, strel('disk',radius));
catch
    return;            % no IPT: leave the holes
end
for ch = 1:3
    O = orig(:,:,ch);
    D = dil(:,:,ch);
    O(~valid) = D(~valid);
    im(:,:,ch) = O;
end
end

function g = adaptiveEqualize(g)
% CLAHE-style contrast equalization with safe fallback to histeq, then
% to a plain imadjust if neither is available. Makes SIFT robust to
% brightness/contrast mismatch between dim scan colours and a bright
% phone photo.
try
    g = adapthisteq(g, 'ClipLimit', 0.01, 'NumTiles', [8 8]);
    return;
catch
end
try
    g = histeq(g);
    return;
catch
end
try
    g = imadjust(g);
catch
end
end

function [u, v, n] = orientPlaneAxes(u, v, n)
% Force the SVD-derived principal axes into a consistent orientation:
%   * v points "down" in world Z (so the synth ortho render is upright
%     under the conventional iy = (vp - vLo)/dx mapping where small vp
%     lands at the top row)
%   * the {u, v, n} frame is right-handed (det = +1)
% SVD only constrains the AXES of these vectors, not their signs, so
% without this normalisation the synth render can come out flipped
% top-down or mirrored from run to run.  The SIGN of n is still
% ambiguous after this — use chooseFrontOrientation to resolve it.
if v(3) > 0, v = -v; end
if det([u(:).'; v(:).'; n(:).']) < 0, u = -u; end
end

function [u, v, n, info] = chooseFrontOrientation(u, v, n, xyz, rgb, img)
% SVD's plane normal n has arbitrary sign.  If our synth is rendered
% with n pointing INTO the face (i.e. we're looking from behind the
% rock), it comes out as the mirror image of what the camera saw, and
% although SIFT can still match across a mirror, the lifted 3-D points
% land on the wrong side of the plane and PnP returns a flipped pose.
%
% This routine renders the cloud orthographically from BOTH n
% directions at a small test resolution, runs quick SIFT matching
% against the photo, and returns whichever orientation produced more
% raw matches.  The companion flip on u keeps {u,v,n} right-handed.
info = struct('nFront', 0, 'nBack', 0, 'flipped', false);
testW = 1200;
% Downscale BEFORE rgb2gray to skip gray-converting full-res pixels.
gP = adaptiveEqualize(rgb2gray(imresize(img, testW/size(img,2))));
fP = detectSIFTFeatures(gP, 'ContrastThreshold', 0.005);
if isempty(fP) || fP.Count < 20
    return;            % can't reliably score; leave orientation as-is
end
[dP, vP] = extractFeatures(gP, fP);
nA = countMatchesQuick( u,  v,  n, xyz, rgb, dP, vP, testW);
nB = countMatchesQuick(-u,  v, -n, xyz, rgb, dP, vP, testW);
info.nFront = nA; info.nBack = nB;
if nB > nA
    u = -u; n = -n;
    info.flipped = true;
end
end

function [nMatches, vPhoto, vSynth] = countMatchesQuick(u, v, n, xyz, rgb, dPhoto, vPhotoIn, testW)
% Render a small orthographic synth, detect SIFT with a low contrast
% threshold (so we get plenty of candidates), and count matches
% against the precomputed photo descriptors.  Returns the matched
% locations so the caller can optionally geometric-verify.
nMatches = 0; vPhoto = []; vSynth = [];
c = mean(xyz,1);
d = xyz - c;
up = d * u.';   vp = d * v.';   wp_ = d * n.';
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
gS = adaptiveEqualize(rgb2gray(synth));
try
    fS = detectSIFTFeatures(gS, 'ContrastThreshold', 0.005);
catch
    return;
end
if isempty(fS) || fS.Count < 20, return; end
[dS, vS] = extractFeatures(gS, fS);
m = matchFeatures(dPhoto, dS, 'Unique', true, 'MaxRatio', 0.95);
if isempty(m), return; end
vPhoto = double(vPhotoIn(m(:,1)).Location);
vSynth = double(vS(m(:,2)).Location);
% Geometric verification: count matches that survive a projective fit
% (correct orientation -> coherent transform with many inliers).
% Fall back to raw match count if RANSAC can't find a consensus.
nRaw = size(m, 1);
nInl = 0;
if size(vPhoto,1) >= 8
    try
        [~, inl] = estimateGeometricTransform2D(vPhoto, vSynth, ...
            'projective','MaxNumTrials',3000,'MaxDistance',20, ...
            'Confidence',90);
        nInl = nnz(inl);
    catch
    end
end
% Combine raw + inlier counts so we have a usable score even when
% geometric verification can't lock in.
nMatches = nRaw + 10*nInl;
end

function delTrailing(harr)
if ~isempty(harr) && isgraphics(harr(end)), delete(harr(end)); end
end
