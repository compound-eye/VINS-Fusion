#!/usr/bin/env python3
"""Generate VINS-Fusion config files from calibration.json in exported dataset folders.

Reads the repo-agnostic calibration.json written by export_from_recon_debug.py
and writes VINS-Fusion-specific config YAMLs into this repo's config/ directory,
mirroring the dataset folder structure.

Example (single dataset):
    python3 py/generate_ce_config.py \
        --dataset-dir /host/mnt/nas3/user/ajones/datasets/VINS-Fusion/agx/20251031_SF/raw/251106-212953

Example (batch via scenario JSON):
    python3 py/generate_ce_config.py \
        --dataset-root /host/mnt/nas3/user/ajones/datasets/VINS-Fusion \
        --scenario-json /path/to/odometry_rgb_ground_eval_small.json
"""
import argparse
import json
import pathlib

import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
CONFIG_ROOT = REPO_ROOT / "config" / "compound_eye"


# IMU noise values from vio2/common/sensors_config.hpp (continuous-time densities)
_IMU_PARAMS = """\
acc_n: 0.002
gyr_n: 0.0001
acc_w: 0.0002
gyr_w: 0.00001
g_norm: 9.81007"""

# Solver/feature tracking params
_SOLVER_PARAMS = """\
max_cnt: 200
min_dist: 25
freq: 20
F_threshold: 1.0
show_track: 0
flow_back: 1

max_solver_time: 0.04
max_num_iterations: 8
keyframe_parallax: 10.0

multiple_thread: 1
estimate_td: 0
td: 0.0"""


def _format_body_t_cam(name: str, T: np.ndarray) -> str:
    """Format a 4x4 transform as a VINS-Fusion !!opencv-matrix block."""
    d = T.flatten()
    rows = []
    for i in range(4):
        vals = ", ".join(f"{d[i * 4 + j]:.10g}" for j in range(4))
        prefix = "   data: [" if i == 0 else "          "
        suffix = "]" if i == 3 else ","
        rows.append(f"{prefix}{vals}{suffix}")
    return f"""\
{name}: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
""" + "\n".join(
        rows
    )


def _dataset_rel_path(dataset_dir: pathlib.Path) -> str:
    """Extract relative dataset path by splitting on 'datasets/'."""
    dataset_dir_str = str(dataset_dir)
    if "datasets/" in dataset_dir_str:
        return dataset_dir_str.split("datasets/", 1)[1]
    raise RuntimeError(
        f"Dataset dir {dataset_dir} does not contain 'datasets/' in its path. "
        f"Cannot derive config directory structure."
    )


def write_camera_pinhole_yaml(
    camera_index: int,
    cal: dict,
    config_dir: pathlib.Path,
    overwrite: bool,
) -> pathlib.Path:
    """Write a cam<N>_pinhole.yaml for VINS-Fusion."""
    config_dir.mkdir(parents=True, exist_ok=True)
    out_path = config_dir / f"cam{camera_index}_pinhole.yaml"

    if out_path.exists() and not overwrite:
        print(f"  Skipping {out_path.name} (exists, use --overwrite)")
        return out_path

    content = f"""%YAML:1.0
---
model_type: PINHOLE
camera_name: cam{camera_index}
image_width: {cal['image_width']}
image_height: {cal['image_height']}
distortion_parameters:
   k1: 0.0
   k2: 0.0
   p1: 0.0
   p2: 0.0
projection_parameters:
   fx: {cal['fx']:.6f}
   fy: {cal['fy']:.6f}
   cx: {cal['cx']:.6f}
   cy: {cal['cy']:.6f}
"""
    out_path.write_text(content)
    print(f"  Wrote {out_path.name}")
    return out_path


def write_vins_config(
    config_name: str,
    num_cams: int,
    cal: dict,
    body_t_cam0: np.ndarray,
    body_t_cam1: np.ndarray,
    config_dir: pathlib.Path,
    overwrite: bool,
    output_path: str = "",
) -> pathlib.Path:
    """Write a VINS-Fusion config YAML (mono or stereo+IMU)."""
    config_dir.mkdir(parents=True, exist_ok=True)
    out_path = config_dir / config_name

    if out_path.exists() and not overwrite:
        print(f"  Skipping {out_path.name} (exists, use --overwrite)")
        return out_path

    lines = ["%YAML:1.0", ""]
    lines.append("imu: 1")
    lines.append(f"num_of_cam: {num_cams}")
    lines.append("")
    lines.append('imu_topic: "/imu0"')
    lines.append('image0_topic: "/cam0/image_raw"')

    if num_cams == 1:
        lines.append("")
        lines.append('cam0_calib: "cam0_pinhole.yaml"')
    else:
        lines.append('image1_topic: "/cam1/image_raw"')
        lines.append("")
        lines.append('cam0_calib: "cam0_pinhole.yaml"')
        lines.append('cam1_calib: "cam1_pinhole.yaml"')

    lines.append(f"image_width: {cal['image_width']}")
    lines.append(f"image_height: {cal['image_height']}")
    lines.append("")

    lines.append("estimate_extrinsic: 1   # refine around initial guess")
    lines.append(_format_body_t_cam("body_T_cam0", body_t_cam0))
    if num_cams == 2:
        lines.append("")
        lines.append(_format_body_t_cam("body_T_cam1", body_t_cam1))
    lines.append("")

    lines.append(_IMU_PARAMS)
    lines.append("")
    lines.append(_SOLVER_PARAMS)
    lines.append("")
    lines.append(f'output_path: "{output_path}"')
    lines.append("")

    out_path.write_text("\n".join(lines))
    print(f"  Wrote {out_path.name}")
    return out_path


def generate_for_dataset(dataset_dir: pathlib.Path, overwrite: bool):
    """Generate VINS-Fusion configs for a single dataset."""
    cal_path = dataset_dir / "calibration.json"
    if not cal_path.exists():
        print(f"Warning: {cal_path} not found, skipping")
        return

    with open(cal_path) as f:
        cal = json.load(f)

    rel_path = _dataset_rel_path(dataset_dir)
    config_dir = CONFIG_ROOT / rel_path
    print(f"Dataset {dataset_dir.name} -> {config_dir}")

    body_t_cam0 = np.array(cal["imu_from_cam0_rect"])
    body_t_cam1 = np.array(cal["imu_from_cam1_rect"])

    has_cam0 = (dataset_dir / "images" / "cam0").exists()
    has_cam1 = (dataset_dir / "images" / "cam1").exists()

    if has_cam0:
        write_camera_pinhole_yaml(0, cal, config_dir, overwrite)
    if has_cam1:
        write_camera_pinhole_yaml(1, cal, config_dir, overwrite)

    if has_cam0:
        write_vins_config(
            "mono_left_imu_config.yaml", 1, cal,
            body_t_cam0, body_t_cam1, config_dir, overwrite,
        )
    if has_cam1:
        write_vins_config(
            "mono_right_imu_config.yaml", 1, cal,
            body_t_cam1, body_t_cam0, config_dir, overwrite,
        )
    if has_cam0 and has_cam1:
        write_vins_config(
            "stereo_imu_config.yaml", 2, cal,
            body_t_cam0, body_t_cam1, config_dir, overwrite,
        )


def main():
    parser = argparse.ArgumentParser(
        description="Generate VINS-Fusion config files from calibration.json."
    )
    parser.add_argument(
        "--dataset-dir", type=pathlib.Path,
        help="Single exported dataset directory containing calibration.json.",
    )
    parser.add_argument(
        "--dataset-root", type=pathlib.Path,
        help="Root of exported datasets (for batch mode with --scenario-json).",
    )
    parser.add_argument(
        "--scenario-json", type=pathlib.Path,
        help="Scenario JSON for batch mode.",
    )
    parser.add_argument(
        "--overwrite", action="store_true",
        help="Overwrite existing config files.",
    )
    args = parser.parse_args()

    print(f"Config output root: {CONFIG_ROOT}")

    if args.scenario_json:
        if not args.dataset_root:
            parser.error("--dataset-root required with --scenario-json")
        with open(args.scenario_json) as f:
            scenario = json.load(f)
        for group in scenario.get("dataset_groups", []):
            for ds in group.get("datasets", []):
                ds_path = ds["id"]
                if "datasets/" in ds_path:
                    rel_path = ds_path.split("datasets/", 1)[1]
                else:
                    rel_path = pathlib.Path(ds_path).name
                dataset_dir = args.dataset_root / rel_path
                generate_for_dataset(dataset_dir, args.overwrite)
    elif args.dataset_dir:
        generate_for_dataset(args.dataset_dir, args.overwrite)
    else:
        parser.error("Provide either --dataset-dir or --scenario-json")


if __name__ == "__main__":
    main()
