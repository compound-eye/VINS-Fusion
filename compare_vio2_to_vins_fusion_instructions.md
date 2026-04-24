# Comparing VIO2 vs VINS-Fusion on CE Data

How to run end-to-end pipeline to benchmark our in-house VIO2 against VINS-Fusion on the
same CSI datasets. The comparison is fair by construction: VINS-Fusion is
fed the exact pixels VIO2 processed, not a re-implemented image pipeline.

**Final artifacts:**
- HTML regression comparison dashboard (EPE, CDFs, bad-segment counts) at
  `<output-root>/index.html`.

Commands are run in one of two containers, labeled on every stage heading:

- **ce_build_env** — Compoundeye monorepo container
- **vins-fusion-noetic** — VINS-Fusion container

Data moves between containers through the shared host filesystem.

---

## Pipeline at a glance

```
┌────────────────────────┐   ┌────────────────────────┐   ┌────────────────────────┐
│ 1. VIO2 recon          │──▶│ 2. Export repo-        │──▶│ 3. Create              │
│    (+ debug imgs)      │   │    agnostic data       │   │    ROS bag             │
│    ce_build_env        │   │    ce_build_env        │   │    vins-fusion-noetic  │
└────────────────────────┘   └────────────────────────┘   └────────────────────────┘
           │                                                          │
           │ BodyPath.OdometryPose                                    ▼
           │ (VIO2 baseline)                              ┌────────────────────────┐
           │                                              │ 4. Run VINS            │
           │                                              │    + convert           │
           │                                              │    vins-fusion-noetic  │
           │                                              └────────────────────────┘
           │                                                          │
           ▼                                                          │
┌──────────────────────────────────────────────┐                      │
│ 5. VIO debug viewer (multi-root comparison)  │◀─────────────────────┤
│    ce_build_env                              │                      │
└──────────────────────────────────────────────┘                      │
┌──────────────────────────────────────────────┐                      │
│ 6. Regression dashboard (compare_vio_runs.py)│◀─────────────────────┘
│    ce_build_env                              │
└──────────────────────────────────────────────┘
```

---

## Prerequisites

- CE monorepo checked out and buildable. `reconstruction_batch` is the main
  binary; build once via `env/build/build.sh -- reconstruction_batch`.
- VINS-Fusion Docker image built. From the VINS-Fusion repo root (on the host):
  ```bash
  docker build --tag ros:vins-fusion -f docker/Dockerfile .
  ```
- Scenario JSON listing the datasets used to benchmark at
  `data_json/odometry_rgb_ground_eval_small.json`. You do not have to use
  the same dataset split. This is simply the one that was used for the original
  comparison.

---

## Stage 1 — VIO2 reconstructions with debug images (ce_build_env)

Make sure you've built reconstruction_batch:
```bash
./env/build/build.sh reconstruction_batch
```

Then run reconstruction using `scripts/analysis/reconstruction_test_runner.py`
with the `-Sdebug_img_interval=1` flag. This flag ensures all images
after being run through vio2's image processing pipeline are saved, so we can
feed the same images to any open-source method.

For which dataset split to run reconstruction on, the one used originally
is at `data_json/odometry_rgb_ground_eval_small.json`.

### Outputs consumed downstream (per dataset, under `--recon-output`)

```
<dataset_id>/
├── BodyPath.OdometryPose           ← VIO2 baseline trajectory (Stage 5, 6)
├── reconstruction.log
├── calibration/
│   ├── left_cam_imu_offset.lua     ← imu0_from_cam0_rect (Stage 2)
│   └── rectified_calibration0.lua  ← rectified intrinsics + baseline (Stage 2)
├── debug/
│   ├── _rgb.left/000000.png …      ← Stage 2 input
│   └── _rgb.right/000000.png …     ← stereo only
└── …
summary.json                         ← consumed by compare_vio_runs.py (Stage 6)
```

---

## Stage 2 — Export repo-agnostic dataset (ce_build_env)

Reads `debug/_rgb.{left,right}/` + `calibration/` + the raw dataset's
`CsiFrameInfo` (for IMU) produces a dataset any VIO repo can consume
without CE monorepo dependencies.

**Script:** `/src/py/dataset/export_repo_agnostic_dataset_from_recon.py`

The original exported datasets can be found here:
`/host/mnt/nas3/user/ajones/datasets/csi_datasets_for_opensource_repos`

They share the same subfolder structure as their original csi dataset
directories.

```
<scenario>/raw/<dataset_id>/
├── images/
│   ├── cam0/000000.png …
│   └── cam1/000000.png …      (stereo only)
├── frame_timestamps.csv       columns: frame_id, timestamp_s
├── imu_data.csv               columns: timestamp_s, acc_xyz, gyro_xyz
└── data.bag                   (written in Stage 3)
```

### Single-dataset invocation (full export)

```bash
python3 /src/py/dataset/export_repo_agnostic_dataset_from_recon.py \
    --dataset-dir <path/to/csi_dataset_dir> \
    --recon-dir   <path/to/recon_dir> \
    --output-dir  <path/to/output_dir> \
```


### Why this stage exists

CE's image pipeline uses a non-standard Division polynomial distortion model
plus dataset-specific rectification parameters (`rectified_height` from
`PairSettings0.lua`, `image_dim_scale` from `reconstruction_settings.lua`).
Re-implementing it in Python risks subtle pixel drift. Reusing reconstruction
debug output guarantees pixel-identical frames reach VINS-Fusion at zero extra
implementation cost.

### Calibration notes

- **`body_T_cam0` = `imu0_from_cam0_rect`** read directly from
  `left_cam_imu_offset.lua`. VINS-Fusion's "body" frame is the IMU — *not*
  VIO2's body (which is `cam0_rect`).
- **`body_T_cam1` = `imu0_from_cam0_rect · [I | [baseline, 0, 0]ᵀ]`** — after
  stereo rectification, cam1 differs from cam0 only by a horizontal baseline.

---

## Stage 3 — Create ROS bags (vins-fusion-noetic)

**Script:** `py/exported_to_rosbag.py` (in the VINS-Fusion repo).

Reads the PNGs and CSVs from Stage 2 and writes `data.bag` in place. Auto-
detects mono vs stereo from whether `images/cam1/` exists.

Topics produced:

- `/cam0/image_raw` — `sensor_msgs/Image`, `mono8`
- `/cam1/image_raw` — `sensor_msgs/Image`, `mono8` (stereo only)
- `/imu0`          — `sensor_msgs/Imu`, accel m/s², gyro rad/s

### Batch (all datasets in the scenario JSON)
Running in batch-mode assumes that the subfolder structure of the exported
datasets root directory matches the subfolder structure of the original csi dataset
directories.
```bash
python3 py/exported_to_rosbag.py \
    <path/to/exported_datasets_root_dir> \
    --scenario-json data_json/odometry_rgb_ground_eval_small.json
```

### Single Dataset

Below is example command for creating ros bag for a single exported csi dataset:
```bash
python3 py/exported_to_rosbag.py \
    <path/to/single/exported/csi_dir> \
    <path/to/single/exported/csi_dir/data.bag> \
```

---

## Stage 4 - Generate VINS-Fusion YAML Config Files for CE Datasets (vins-fusion-noetic)

### Batch Mode

```bash
python3 py/generate_ce_config.py \
    --dataset-root <path/to/exported_datasets_root_dir> \
    --scenario-json data_json/odometry_rgb_ground_eval_small.json

### Single Dataset

```bash
python3 py/generate_ce_config.py \
    --dataset-dir <path/to/single/exported/csi_dir>
```
```
```

## Stage 5 — Run VINS-Fusion + convert to `BodyPath.OdometryPose` (vins-fusion-noetic)

**Script:** `py/run_vins_batch.py`.

For each dataset in the scenario JSON, the script:

1. Patches the VINS config YAML with the run-specific `output_path`.
2. Starts `roscore` + `vins_node` with the patched config.
3. Plays `data.bag`.
4. Converts `vio.csv` → `BodyPath.OdometryPose` (Avro) via the self-contained
   `ce_pose.py` — no CE monorepo dependencies required inside the VINS
   container.
5. Writes a minimal `reconstruction.log` so `eval_vio.py` accepts the run.
6. Writes `summary.json` listing successful datasets.

### Stereo + IMU

```bash
python3 py/run_vins_batch.py \
    --scenario-json data_json/odometry_rgb_ground_eval_small.json \
    --dataset-root  <path/to/exported_datasets_root_dir> \
    --config-root   config/compound_eye \
    --save-dir      <path/to/save/dir> \
    --config-name   stereo_imu_config.yaml
```

### Mono left + IMU

```bash
python3 py/run_vins_batch.py \
    --scenario-json data_json/odometry_rgb_ground_eval_small.json \
    --dataset-root  <path/to/exported_datasets_root_dir> \
    --config-root   config/compound_eye \
    --save-dir      <path/to/save/dir> \
    --config-name   mono_left_imu_config.yaml
```

`--dataset-root` and `--config-root` both consume the same
`<scenario>/raw/<dataset_id>` relative structure, which mirrors the CE csi dataset
layout.

### Coordinate transform (summarized)

VINS writes `vins_world_from_imu` in VINS world frame (X-right, Y-forward,
Z-up). `BodyPath.OdometryPose` expects `vio2_world_from_cam0_rect` in VIO2
world frame (X-right, Y-down, Z-forward). `ce_pose.py` applies:

```
vio2_world_from_cam0_rect = VIO2WORLD_FROM_VINSWORLD
                             · vins_world_from_imu
                             · imu_from_cam0_rect
```

where `VIO2WORLD_FROM_VINSWORLD` is +90° about X.

---

## Stage 6 — Comparing VIO2 to VINS-Fusion Via Regression Dashboard (ce_build_env)

**Script:** `/src/py/vio/compare_vio_runs.py`. Runs EPE + bad-segment +
CDF evaluation on each labeled run, then emits a single comparison dashboard.

The root directories passed for vio2 and vins-fusion must contain the
`reconstructions` subdirectory, which contains the outputs for each dataset.

```bash
./env/build/run.sh python3 py/vio/compare_vio_runs.py \
    --runs \
        "vio2:<path/to/vio2/recon_root_dir>" \
        "vins-fusion:<path/to/vins-fusion/output_root_dir>" \
    --output-root <path/to/save/output> \
```

Open `<output-root>/index.html` to view side-by-side metrics.

---
