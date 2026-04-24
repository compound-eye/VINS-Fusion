#!/usr/bin/env python3
"""Convert exported CSI images and IMU CSV to a ROS bag for VINS-Fusion.

Reads the output of export_from_recon_debug.py and writes a ROS bag.
Auto-detects available cameras from images/cam0/ and images/cam1/ subdirectories.

Topics written:
  /cam0/image_raw  -- sensor_msgs/Image, mono8 (if images/cam0/ exists)
  /cam1/image_raw  -- sensor_msgs/Image, mono8 (if images/cam1/ exists)
  /imu0            -- sensor_msgs/Imu, accel m/s^2, gyro rad/s

Run inside the VINS-Fusion noetic container (has rosbag/rospy/sensor_msgs).

Single dataset example:
    python3 py/exported_to_rosbag.py \\
        /path/to/exported_dataset \\
        /path/to/output.bag

Batch mode (from scenario JSON):
    python3 py/exported_to_rosbag.py \\
        /host/mnt/nas3/user/ajones/datasets/VINS-Fusion \\
        --scenario-json /path/to/odometry_rgb_ground_eval_small.json
"""
import argparse
import csv
import json
import pathlib

import cv2
import rosbag
import rospy
from sensor_msgs.msg import Image, Imu


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert exported CSI images + IMU CSV to ROS bag for VINS-Fusion."
    )
    parser.add_argument("exported_dir", type=pathlib.Path,
                        help="Exported dataset directory (single mode) or "
                        "root of all exported datasets (batch mode with --scenario-json)")
    parser.add_argument("output_bag", type=pathlib.Path, nargs="?", default=None,
                        help="Output .bag file path (single mode only)")
    parser.add_argument("--scenario-json", type=pathlib.Path, default=None,
                        help="Scenario JSON file for batch mode. Creates data.bag "
                        "in each dataset's exported directory.")
    parser.add_argument("--imu-topic", default="/imu0",
                        help="ROS IMU topic name (default: /imu0)")
    parser.add_argument("--start-frame", type=int, default=0,
                        help="First frame index to include (default: 0)")
    parser.add_argument("--stop-frame", type=int, default=None,
                        help="Stop frame index, exclusive (default: all)")
    args = parser.parse_args()

    if args.scenario_json is None and args.output_bag is None:
        parser.error("output_bag is required in single dataset mode "
                     "(when --scenario-json is not used)")

    return args


def read_frame_timestamps(exported_dir: pathlib.Path):
    """Returns list of (frame_id, timestamp_s) sorted by frame_id."""
    rows = []
    with open(exported_dir / "frame_timestamps.csv") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append((int(row["frame_id"]), float(row["timestamp_s"])))
    rows.sort(key=lambda r: r[0])
    return rows


def read_imu_data(exported_dir: pathlib.Path):
    """Returns list of dicts sorted by timestamp_s."""
    rows = []
    with open(exported_dir / "imu_data.csv") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                "timestamp_s": float(row["timestamp_s"]),
                "acc_x": float(row["acc_x"]),
                "acc_y": float(row["acc_y"]),
                "acc_z": float(row["acc_z"]),
                "gyro_x": float(row["gyro_x"]),
                "gyro_y": float(row["gyro_y"]),
                "gyro_z": float(row["gyro_z"]),
            })
    rows.sort(key=lambda r: r["timestamp_s"])
    return rows


def find_cameras(exported_dir: pathlib.Path):
    """Detect available cameras from images/cam<N>/ subdirectories.

    Returns dict mapping camera index to (images_dir, topic_name).
    """
    cameras = {}
    images_root = exported_dir / "images"
    for cam_idx in [0, 1]:
        cam_dir = images_root / f"cam{cam_idx}"
        if cam_dir.exists():
            cameras[cam_idx] = (cam_dir, f"/cam{cam_idx}/image_raw")
    if not cameras:
        raise FileNotFoundError(
            f"No images/cam0/ or images/cam1/ found in {exported_dir}"
        )
    return cameras


def make_image_msg(img_path: pathlib.Path, timestamp_s: float,
                   frame_id: str) -> Image:
    img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError(f"Failed to read image: {img_path}")

    msg = Image()
    msg.header.stamp = rospy.Time.from_sec(timestamp_s)
    msg.header.frame_id = frame_id
    msg.height = img.shape[0]
    msg.width = img.shape[1]
    msg.encoding = "mono8"
    msg.is_bigendian = 0
    msg.step = img.shape[1]
    msg.data = img.tobytes()
    return msg


def make_imu_msg(row: dict) -> Imu:
    msg = Imu()
    msg.header.stamp = rospy.Time.from_sec(row["timestamp_s"])
    msg.header.frame_id = "imu0"

    msg.linear_acceleration.x = row["acc_x"]
    msg.linear_acceleration.y = row["acc_y"]
    msg.linear_acceleration.z = row["acc_z"]

    msg.angular_velocity.x = row["gyro_x"]
    msg.angular_velocity.y = row["gyro_y"]
    msg.angular_velocity.z = row["gyro_z"]

    # Covariances unknown — set to -1 to indicate not available
    msg.orientation_covariance[0] = -1.0
    msg.linear_acceleration_covariance[0] = -1.0
    msg.angular_velocity_covariance[0] = -1.0

    return msg


def create_rosbag(exported_dir: pathlib.Path, output_bag: pathlib.Path,
                  imu_topic: str, start_frame: int, stop_frame):
    """Create a rosbag from a single exported dataset directory."""
    cameras = find_cameras(exported_dir)
    cam_names = {0: "left", 1: "right"}
    for cam_idx, (cam_dir, topic) in cameras.items():
        print(f"  Found cam{cam_idx} ({cam_names[cam_idx]}): {cam_dir} -> {topic}")

    all_frame_timestamps = read_frame_timestamps(exported_dir)
    print(f"  {len(all_frame_timestamps)} total frames")

    # Slice to [start_frame, stop_frame)
    stop = stop_frame if stop_frame is not None else len(all_frame_timestamps)
    frame_timestamps = all_frame_timestamps[start_frame:stop]
    if not frame_timestamps:
        raise ValueError(f"No frames in range [{start_frame}, {stop})")
    print(f"  Using frames [{start_frame}, {stop}) = {len(frame_timestamps)} frames")

    all_imu_data = read_imu_data(exported_dir)
    t_start = frame_timestamps[0][1]
    t_end = frame_timestamps[-1][1]
    imu_data = [r for r in all_imu_data if t_start <= r["timestamp_s"] <= t_end]
    print(f"  {len(imu_data)} IMU samples in [{t_start:.3f}, {t_end:.3f}]")

    # Build sorted image path lists per camera
    cam_img_paths = {}
    for cam_idx, (cam_dir, topic) in cameras.items():
        all_paths = sorted(cam_dir.glob("*.png"), key=lambda p: int(p.stem))
        cam_img_paths[cam_idx] = all_paths[start_frame:stop]

    # Merge all cameras and IMU into a single time-sorted stream
    events = []
    for cam_idx, (cam_dir, topic) in cameras.items():
        for (frame_id, ts), img_path in zip(frame_timestamps, cam_img_paths[cam_idx]):
            events.append(("image", ts, img_path, cam_idx, topic))
    for row in imu_data:
        events.append(("imu", row["timestamp_s"], row, None, None))
    events.sort(key=lambda e: e[1])

    output_bag.parent.mkdir(parents=True, exist_ok=True)
    print(f"  Writing bag: {output_bag}")
    n_images = {cam_idx: 0 for cam_idx in cameras}
    n_imu = 0

    with rosbag.Bag(str(output_bag), "w") as bag:
        for kind, ts, data, cam_idx, topic in events:
            if kind == "image":
                msg = make_image_msg(data, ts, f"cam{cam_idx}")
                bag.write(topic, msg, msg.header.stamp)
                n_images[cam_idx] += 1
                total_imgs = sum(n_images.values())
                if total_imgs % 1000 == 0:
                    print(f"    {total_imgs} images, {n_imu} IMU samples written...")
            else:
                msg = make_imu_msg(data)
                bag.write(imu_topic, msg, msg.header.stamp)
                n_imu += 1

    for cam_idx, count in n_images.items():
        print(f"  cam{cam_idx}: {count} images")
    print(f"  IMU: {n_imu} samples")
    print(f"  Done: {output_bag}")


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


def main():
    args = parse_args()

    if args.scenario_json is not None:
        # Batch mode: iterate over datasets in scenario JSON
        dataset_root = args.exported_dir
        print(f"Batch mode: scenario JSON = {args.scenario_json}")
        print(f"Dataset root: {dataset_root}")
        print()

        for csi_dir, dataset_id in datasets_from_scenario_json(args.scenario_json):
            # Derive rel_path from csi_dir (everything after "datasets/")
            if "datasets/" in csi_dir:
                rel_path = csi_dir.split("datasets/", 1)[1]
            else:
                raise RuntimeError(
                    f"csi_dir has unexpected format (no 'datasets/' found): {csi_dir}"
                )

            exported_dir = dataset_root / rel_path
            if not exported_dir.exists():
                print(f"SKIP {rel_path}: exported dir not found at {exported_dir}")
                print()
                continue

            output_bag = exported_dir / "data.bag"
            print(f"=== {rel_path} ===")
            create_rosbag(exported_dir, output_bag, args.imu_topic,
                          args.start_frame, args.stop_frame)
            print()

        print("Batch complete.")
    else:
        # Single dataset mode
        create_rosbag(args.exported_dir, args.output_bag, args.imu_topic,
                      args.start_frame, args.stop_frame)
        print(f"\nVerify with: rosbag info {args.output_bag}")


if __name__ == "__main__":
    main()
