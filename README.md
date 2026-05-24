# Photo to Point-Cloud Alignment

MATLAB tools for registering an RGB photo to a coloured 3-D point cloud
of a **tunnel face**, recovering the camera pose, and producing a
photo-recoloured cloud plus a per-pixel link back to the cloud — built
to support downstream geologic segmentation work on the cropped 2-D
image and re-projection of those annotations onto the 3-D cloud.

---

## What's in here

| File | What it does |
|---|---|
| [`photo2pc_align.m`](photo2pc_align.m) | Headless API. One function call solves the alignment and returns a struct (auto, manual, or auto-then-manual mode). |
| [`photo2pc_gui.m`](photo2pc_gui.m) | Interactive pipeline visualiser. Walks every stage on screen (load → plane fit → ortho render → feature match → PnP → recolour) with tabs for each diagnostic. |

---

## Features

- **Two run modes** selectable from the GUI:
  - **Auto** — orthographic render of the cloud is matched to the photo with SIFT + KAZE + ORB, RANSAC-filtered, then PnP.
  - **Manual** — skips feature detection entirely. Opens a photo↔synth picker where you click matching points by hand. Each pair is drawn in its own bright colour on both images so pairs are easy to track.
- **Tunnel-face crop step** before manual picking — auto-seeded face mask that you can confirm or repaint with a brush editor (add/remove regions, expand/shrink borders). The picker dims non-face areas so you only pick face features.
- **Flip view** button in the manual picker re-renders the synth ortho from the opposite side of the plane in one click — useful when the SVD normal lands on the wrong face.
- **Switching mode resets the pipeline** (with a confirmation if you have unsaved work) so you can't end up with stale fields from a previous run.

---

## Requirements

- **MATLAB R2022b** or newer (older releases will mostly work; the code has fallbacks for the camera-pose API rename `estimateWorldCameraPose` → `estworldpose`).
- **Toolboxes:**
  - Computer Vision Toolbox (`pointCloud`, `pcread`, `pcwrite`, `estworldpose`, `bundleAdjustmentMotion`, `cameraIntrinsics`, feature detectors)
  - Image Processing Toolbox (`adapthisteq`, `imclose`, `imdilate`, `bwboundaries`, `bwconvhull`)
  - Statistics and Machine Learning Toolbox (`KDTreeSearcher`, `knnsearch`, `prctile`)

The code degrades gracefully if some helpers are missing (e.g. ORB on
very old releases) — it just uses fewer detectors.

---

## Quick start

### GUI (recommended)

```matlab
photo2pc_gui                                  % file dialog
photo2pc_gui('path/to/cloud.ply', 'path/to/photo.jpg')
```

Tick **Manual mode** in the header if you want to skip auto feature
matching and pick correspondences by hand.

Then: **Run All** → review each tab → **Save Results**.

### Headless

```matlab
result = photo2pc_align('cloud.ply', 'photo.jpg');                 % auto-then-manual
result = photo2pc_align('cloud.ply', 'photo.jpg', 'Mode','manual'); % manual only
result = photo2pc_align('cloud.ply', 'photo.jpg', ...
    'FocalLengthPx', 4500, 'SaveResults', true);
```

The cloud may be a `.ply` or a `.mat` containing a `pointCloud`, or
two arrays `XYZ` (N×3) and `RGB` (N×3 uint8).

---

## Output files

Clicking **Save Results** in the GUI (or passing `'SaveResults', true`
to `photo2pc_align`) writes four files next to your photo:

| File | Contents |
|---|---|
| `<stem>_p2pc_alignment.mat` | Comprehensive struct (see below). |
| `<stem>_p2pc_overlay.png`  | Photo with the projected cloud overlaid + residual lines. |
| `<stem>_p2pc_cropped.png`  | The tunnel-face crop as a standalone image. |
| `<stem>_p2pc_colored_cloud.ply` | Photo-recoloured point cloud (binary PLY). |

### The alignment struct

The `.mat` is built for **downstream geologic-segmentation work**.
Every field needed to go from 2-D image coordinates to 3-D cloud points
(and back) is in there:

| Field | Type | Meaning |
|---|---|---|
| `croppedImage` | `uint8` Hc × Wc × 3 | The cropped tunnel-face image. |
| `croppedFaceMask` | `logical` Hc × Wc | Manually-drawn face mask cropped to the bbox (empty if no mask was drawn). |
| `croppedProjMask` | `logical` Hc × Wc | Projection support mask (which pixels have a cloud point in front). |
| `cropBox` | `[x0 y0 w h]` | Bbox in **original-photo** pixel coords, 1-based. |
| `coloredCloudPoints` | `single` N × 3 | Flat array of photo-recoloured cloud positions. |
| `coloredCloudColors` | `uint8` N × 3 | Matching RGB. |
| `pixelToPointIdx` | `uint32` Hc × Wc | Front-most cloud-point index per cropped pixel (z-buffered; **0 = no point**). |
| `pointToPixel` | `single` N × 2 | `(u, v)` in cropped-image coords for each cloud point, **NaN** if outside the crop. |
| `pixelToPointXYZ` | `single` Hc × Wc × 3 | Convenience — the 3-D point under each pixel (NaN where no point hits). |
| `intrinsics`, `K` | camera intrinsics |
| `worldPose`, `R_cw`, `t_cw` | World → camera transform (R_cw maps world → camera). |
| `fullImageSize` | `[H W]` | Of the original photo, for un-cropping. |
| `imagePoints`, `worldPoints`, `reprojRMS` | PnP correspondences and quality. |
| `imageFile`, `cloudFile`, `manualMode`, `method` | Provenance. |

### Typical downstream patterns

**Image segmentation → cloud highlight** (paint regions on the cropped
image, then colour the matching cloud points):

```matlab
A = load('photo_p2pc_alignment.mat').alignment;
L = imread('my_geology_labels.png');             % Hc x Wc uint8 label map
newColors = A.coloredCloudColors;                % start from photo colours
for lbl = unique(L(:)).'
    if lbl == 0, continue; end
    pix = find(L == lbl);
    pts = A.pixelToPointIdx(pix);
    pts = pts(pts > 0);
    newColors(pts, :) = repmat(colorForLabel(lbl), numel(pts), 1);
end
pc = pointCloud(double(A.coloredCloudPoints), 'Color', newColors);
pcwrite(pc, 'my_geology_overlay.ply');
```

**3-D region → image highlight** (you've labelled some cloud points
and want to see where they fall in the cropped image):

```matlab
A = load('photo_p2pc_alignment.mat').alignment;
uv = A.pointToPixel(myPointIds, :);
valid = all(~isnan(uv), 2);
imshow(A.croppedImage); hold on;
plot(uv(valid, 1), uv(valid, 2), 'r.');
```

---

## Pipeline tabs (GUI)

1. Loaded point cloud (3-D)
2. Loaded photo + intrinsics
3. Plane fit (dominant face plane)
4. Synthetic orthographic render
5. Illumination-equalised pair *(skipped in manual mode)*
6. Detected feature keypoints *(skipped in manual mode)*
7. All raw feature matches *(skipped in manual mode)*
8. RANSAC inliers + PnP reprojection *(in manual mode, opens the picker)*
9. Photo crop (original | overlap)
10. Cloud comparison (original | recoloured, synced rotation)

---

## Notes

- A sample point cloud (`face.mat`, ~14 MB) and a sample tunnel-face
  photo (`test.jpg`) are committed so you can run the GUI immediately
  without bringing your own data. Larger sample sets (`sample*/`
  folders and other stand-alone `*.ply`) stay local via `.gitignore`.
- The first time you run the auto pipeline on a new cloud it can take
  10-30 s on the synth render + feature detection. Manual mode runs in
  a few seconds because it skips that work.
- If the auto pipeline keeps producing wrong poses, switch to manual
  mode — it almost always works.
