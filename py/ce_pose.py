"""Minimal Pose and OdometryPose classes for writing BodyPath.OdometryPose Avro files.

Copied from /src/py/utilities/pose.py and /src/py/vio/pose.py to avoid depending
on the full ce_build_env. Only includes what's needed for Avro serialization and
pose composition.

Dependencies: numpy, pyquaternion, fastavro
"""
import math

import fastavro
import numpy as np
from pyquaternion import Quaternion


class Pose:
    """Rigid body transform (rotation + translation)."""

    AVRO_SCHEMA = {
        "type": "record",
        "name": "Pose",
        "fields": [
            {
                "name": "q",
                "type": {
                    "type": "record",
                    "name": "Quaternion_double",
                    "fields": [
                        {"name": "value", "type": {"type": "array", "items": "double"}}
                    ],
                },
            },
            {
                "name": "t",
                "type": {
                    "type": "record",
                    "name": "Matrix_3_1_double",
                    "fields": [
                        {"name": "value", "type": {"type": "array", "items": "double"}}
                    ],
                },
            },
        ],
    }

    def __init__(self, q=None, t=None):
        """Create a Pose.

        Args:
            q: Quaternion as [w, x, y, z] list/array, or pyquaternion.Quaternion.
            t: Translation as [x, y, z] list/array.
        """
        if q is None:
            self.q = Quaternion()
        else:
            self.q = Quaternion(q).normalised
        if t is None:
            self.t = np.array([0, 0, 0], dtype=np.float64)
        else:
            self.t = np.asarray(t, dtype=np.float64)

    def __mul__(self, o):
        new_q = (self.q * o.q).normalised
        new_t = self.t + self.q.rotate(o.t)
        return Pose(new_q.q, new_t)

    def to_avro_obj(self):
        return dict(
            q=dict(value=self.q.elements.tolist()),
            t=dict(value=self.t.tolist()),
        )


class OdometryPose:
    """Pose with metadata from odometry, for Avro serialization."""

    AVRO_SCHEMA = {
        "type": "record",
        "name": "OdometryPose",
        "fields": [
            {"name": "frame_id", "type": "int", "default": -1},
            {"name": "t_sec", "type": "double"},
            {"name": "pose", "type": Pose.AVRO_SCHEMA.copy()},
            {
                "name": "cov",
                "type": {
                    "type": "record",
                    "name": "Matrix_6_6_double",
                    "fields": [
                        {"name": "value", "type": {"type": "array", "items": "double"}}
                    ],
                },
                "default": {"value": [0] * 36},
            },
            {"name": "final_estimate", "type": "boolean", "default": True},
        ],
    }

    def __init__(self, frame_id, t_sec, pose):
        self.frame_id = frame_id
        self.t_sec = t_sec
        self.pose = pose

    def to_avro_obj(self):
        return dict(
            frame_id=self.frame_id,
            t_sec=self.t_sec,
            pose=self.pose.to_avro_obj(),
            cov=dict(value=[0] * 36),
            final_estimate=True,
        )


# VIO2 world from VINS world: +90° rotation about X axis.
# Maps VINS world (X-right, Y-forward, Z-up) to VIO2 world (X-right, Y-down, Z-forward).
VIO2WORLD_FROM_VINSWORLD = Pose(
    q=[math.cos(math.pi / 4), math.sin(math.pi / 4), 0, 0],
    t=[0, 0, 0],
)


def convert_vio_csv(vio_csv_path, output_path, imu_from_cam_rect):
    """Convert VINS-Fusion vio.csv to BodyPath.OdometryPose Avro file.

    Args:
        vio_csv_path: Path to vio.csv
        output_path: Path to write BodyPath.OdometryPose
        imu_from_cam_rect: Pose, the body_T_cam0 from VINS config (= imu0_from_cam0_rect)
    """
    data = np.loadtxt(str(vio_csv_path), delimiter=",", usecols=range(11))
    if data.ndim == 1:
        data = data[np.newaxis, :]

    t_sec = data[:, 0] * 1e-9
    tx, ty, tz = data[:, 1], data[:, 2], data[:, 3]
    qw, qx, qy, qz = data[:, 4], data[:, 5], data[:, 6], data[:, 7]

    records = []
    for i in range(len(data)):
        vins_world_from_imu = Pose(
            q=[qw[i], qx[i], qy[i], qz[i]], t=[tx[i], ty[i], tz[i]]
        )
        vio2_world_from_camera = (
            VIO2WORLD_FROM_VINSWORLD * vins_world_from_imu * imu_from_cam_rect
        )
        op = OdometryPose(
            frame_id=i, t_sec=float(t_sec[i]), pose=vio2_world_from_camera
        )
        records.append(op.to_avro_obj())

    output_path = str(output_path)
    with open(output_path, "wb") as f:
        fastavro.writer(f, OdometryPose.AVRO_SCHEMA, records)

    return len(records)
