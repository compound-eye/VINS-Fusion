#!/usr/bin/env python3
"""Run VINS-Fusion on all datasets in a scenario JSON.

Iterates over datasets, starts roscore + VINS-Fusion for each, plays the rosbag,
converts vio.csv to BodyPath.OdometryPose, and creates fake recon dirs compatible
with evaluate_vio2.py.

Run inside the VINS-Fusion noetic container.
Prerequisites: pip3 install fastavro pyquaternion

Example:
    python3 py/run_vins_batch.py \
        --scenario-json /host/mnt/nas3/.../odometry_rgb_ground_eval_small.json \
        --dataset-root /host/mnt/nas3/user/ajones/datasets/VINS-Fusion \
        --config-root config/compound_eye \
        --save-dir /host/mnt/nas3/user/ajones/output/VINS-Fusion/runs/20260403
"""
import argparse
import json
import pathlib
import re
import shutil
import subprocess
import time

import numpy as np
# Extract rotation as quaternion
from scipy.spatial.transform import Rotation

from ce_pose import Pose, VIO2WORLD_FROM_VINSWORLD, convert_vio_csv


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run VINS-Fusion on all datasets in a scenario JSON."
    )
    parser.add_argument("--scenario-json", type=pathlib.Path, required=True,
                        help="Path to scenario JSON file")
    parser.add_argument("--dataset-root", type=pathlib.Path, required=True,
                        help="Root of exported datasets (contains data.bag per dataset)")
    parser.add_argument("--config-root", type=pathlib.Path, required=True,
                        help="Root of VINS configs (e.g. config/compound_eye)")
    parser.add_argument("--save-dir", type=pathlib.Path, required=True,
                        help="Root output dir for logs + vio.csv")
    parser.add_argument("--config-name", default="stereo_imu_config.yaml",
                        help="Config filename to use (default: stereo_imu_config.yaml)")
    parser.add_argument("--playback-rate", type=float, default=1.0,
                        help="Rosbag playback rate (default: 1.0)")
    return parser.parse_args()


def datasets_from_scenario_json(json_path: pathlib.Path):
    """Yield (csi_dir, dataset_id) for each dataset in the scenario JSON."""
    with open(json_path) as f:
        data = json.load(f)
    for group in data["dataset_groups"]:
        raw_root = group["raw_root"]
        for dataset in group["datasets"]:
            dataset_id = dataset["id"]
            csi_dir = f"{raw_root}/{dataset_id}"
            yield csi_dir, dataset_id


def kill_proc(proc, name):
    """Kill a subprocess and wait for it to exit."""
    if proc is None or proc.poll() is not None:
        return
    print(f"  Stopping {name} (pid {proc.pid})...")
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        print(f"  Force killing {name}...")
        proc.kill()
        proc.wait()


def patch_output_path(src_config: pathlib.Path, dst_config: pathlib.Path,
                      output_path: str):
    """Copy config YAML, replacing the output_path line."""
    lines = src_config.read_text().splitlines()
    with open(dst_config, "w") as f:
        for line in lines:
            if line.startswith("output_path:"):
                f.write(f'output_path: "{output_path}"\n')
            else:
                f.write(line + "\n")


def read_body_t_cam0_from_config(config_path: pathlib.Path) -> Pose:
    """Parse body_T_cam0 from a VINS-Fusion config YAML.

    Reads the !!opencv-matrix data for body_T_cam0 and returns a Pose.
    """
    text = config_path.read_text()
    # Find the body_T_cam0 data line(s)
    match = re.search(r"body_T_cam0:.*?data:\s*\[([^\]]+)\]", text, re.DOTALL)
    if match is None:
        raise RuntimeError(f"Could not find body_T_cam0 in {config_path}")
    values = [float(x.strip()) for x in match.group(1).split(",")]
    if len(values) != 16:
        raise RuntimeError(f"Expected 16 values for body_T_cam0, got {len(values)}")
    T = np.array(values).reshape(4, 4)
    q_xyzw = Rotation.from_matrix(T[:3, :3]).as_quat()  # [x, y, z, w]
    q_wxyz = [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]
    return Pose(q=q_wxyz, t=T[:3, 3].tolist())


def write_fake_reconstruction_log(save_dir: pathlib.Path, csi_dir: str):
    """Write a minimal reconstruction.log that evaluate_vio2.py can parse."""
    log_path = save_dir / "reconstruction.log"
    log_path.write_text(
        f"reconstruction_batch --csi-dir={csi_dir}\n"
        f"exit status: 0\n"
    )


def run_dataset(bag_path: pathlib.Path, config_path: pathlib.Path,
                run_save_dir: pathlib.Path, playback_rate: float,
                csi_dir: str):
    """Run VINS-Fusion on a single dataset and convert output."""
    run_save_dir.mkdir(parents=True, exist_ok=True)

    # Remove stale vio.csv from previous runs
    # stale_vio = run_save_dir / "vio.csv"
    # if stale_vio.exists():
    #     stale_vio.unlink()

    # Patch config with correct output_path
    patched_config = run_save_dir / "config.yaml"
    patch_output_path(config_path, patched_config, str(run_save_dir) + "/")

    # Copy pinhole yamls to save dir so VINS can find them
    for pinhole in config_path.parent.glob("cam*_pinhole.yaml"):
        shutil.copy2(pinhole, run_save_dir / pinhole.name)

    roscore_proc = None
    vins_proc = None

    try:
        # Start roscore
        roscore_log = open(run_save_dir / "roscore.log", "w")
        roscore_proc = subprocess.Popen(
            ["roscore"],
            stdout=roscore_log, stderr=subprocess.STDOUT
        )
        time.sleep(2)

        # Start VINS-Fusion
        vins_log = open(run_save_dir / "rosrun.log", "w")
        vins_proc = subprocess.Popen(
            ["rosrun", "vins", "vins_node", str(patched_config)],
            stdout=vins_log, stderr=subprocess.STDOUT
        )
        time.sleep(2)

        # Play rosbag (blocks until done, output visible in terminal)
        subprocess.run(
            ["rosbag", "play", str(bag_path),
             "--rate", str(playback_rate)],
        )

        # Wait for VINS to finish processing remaining data
        print("  Waiting for VINS to finish processing...")
        time.sleep(5)

    finally:
        kill_proc(vins_proc, "vins_node")
        kill_proc(roscore_proc, "roscore")
        roscore_log.close()
        vins_log.close()

    # Check vio.csv result
    vio_csv = run_save_dir / "vio.csv"
    if not vio_csv.exists():
        print(f"  WARNING: vio.csv not found in {run_save_dir}")
        return False

    n_lines = sum(1 for _ in open(vio_csv))
    print(f"  vio.csv: {n_lines} lines")

    # Convert vio.csv to BodyPath.OdometryPose
    print("  Converting vio.csv to BodyPath.OdometryPose...")
    imu_from_cam_rect = read_body_t_cam0_from_config(patched_config)
    output_avro = run_save_dir / "BodyPath.OdometryPose"
    n_poses = convert_vio_csv(vio_csv, output_avro, imu_from_cam_rect)
    print(f"  Wrote {n_poses} poses to {output_avro}")

    # Create fake reconstruction.log
    write_fake_reconstruction_log(run_save_dir, csi_dir)
    print(f"  Wrote reconstruction.log")

    return True


def main():
    args = parse_args()

    print(f"Scenario JSON: {args.scenario_json}")
    print(f"Dataset root:  {args.dataset_root}")
    print(f"Config root:   {args.config_root}")
    print(f"Save dir:      {args.save_dir}")
    print(f"Config name:   {args.config_name}")
    print()

    # Collect all dataset info for summary.json
    summary_reconstructions = []

    for csi_dir, dataset_id in datasets_from_scenario_json(args.scenario_json):
        # Derive rel_path (everything after "datasets/")
        if "datasets/" not in csi_dir:
            print(f"SKIP {dataset_id}: csi_dir has no 'datasets/' in path: {csi_dir}")
            print()
            continue
        rel_path = csi_dir.split("datasets/", 1)[1]

        bag_path = args.dataset_root / rel_path / "data.bag"
        config_path = args.config_root / rel_path / args.config_name
        run_save_dir = args.save_dir / dataset_id

        if not bag_path.exists():
            print(f"SKIP {dataset_id}: bag not found at {bag_path}")
            print()
            continue
        if not config_path.exists():
            print(f"SKIP {dataset_id}: config not found at {config_path}")
            print()
            continue

        print(f"{'=' * 60}")
        print(f"Dataset: {dataset_id}")
        print(f"  bag:    {bag_path}")
        print(f"  config: {config_path}")
        print(f"  save:   {run_save_dir}")
        print(f"{'=' * 60}")

        success = run_dataset(bag_path, config_path, run_save_dir,
                              args.playback_rate, csi_dir)

        if success:
            summary_reconstructions.append({
                "test_case": {"csi_dir": csi_dir},
                "reconstruction_dir": str(run_save_dir),
            })
        print()

    # Write summary.json for evaluate_vio2.py
    summary_path = args.save_dir / "summary.json"
    summary = {"reconstructions": summary_reconstructions}
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"Wrote {summary_path} ({len(summary_reconstructions)} datasets)")
    print(f"All datasets processed. Results in {args.save_dir}")


if __name__ == "__main__":
    main()
