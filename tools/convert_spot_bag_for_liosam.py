#!/usr/bin/env python3
"""
Convert the Spot sim rosbag into a LIO-SAM-compatible bag.

Fixes applied:
  /imu          - rewrite header.stamp to the DB receive time (epoch-aligned with the
                  lidar), and reconstruct angular_velocity from the orientation stream
                  (the source gyro is all-zeros).
  /point_cloud  - re-emit as a Velodyne-style cloud (x,y,z,intensity,ring,time) so
                  imageProjection accepts it. `ring` is synthesized by binning the
                  elevation angle into N_SCAN rows; `time` from the azimuth sweep.

Usage:
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 tools/convert_spot_bag_for_liosam.py \
      /home/nikolas/Downloads/spot_lio_data_L2/spot_lio_data \
      /home/nikolas/Downloads/spot_lio_data_L2/spot_lio_data_liosam
"""
import sys
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2, PointField, Imu
from std_msgs.msg import Header

# ---- sensor model used to synthesize ring/time ----------------------------
N_SCAN = 64          # number of synthetic rings (must match config params.yaml)
ELEV_MIN_DEG = -45.0  # Ouster OS0 vertical FOV lower bound
ELEV_MAX_DEG = 45.0   # Ouster OS0 vertical FOV upper bound
SCAN_PERIOD = 0.1     # seconds per revolution (used to synthesize per-point time)


def quat_to_rotvec(q_prev, q_curr):
    """Body-frame rotation vector taking q_prev -> q_curr (xyzw inputs)."""
    # relative rotation dq = inv(q_prev) * q_curr
    x0, y0, z0, w0 = q_prev
    x1, y1, z1, w1 = q_curr
    # inv(q_prev) = conjugate (unit quaternion)
    ix, iy, iz, iw = -x0, -y0, -z0, w0
    # quaternion product (ix,iy,iz,iw) * (x1,y1,z1,w1)
    dw = iw * w1 - ix * x1 - iy * y1 - iz * z1
    dx = iw * x1 + ix * w1 + iy * z1 - iz * y1
    dy = iw * y1 - ix * z1 + iy * w1 + iz * x1
    dz = iw * z1 + ix * y1 - iy * x1 + iz * w1
    n = np.sqrt(dx * dx + dy * dy + dz * dz)
    dw = max(-1.0, min(1.0, dw))
    angle = 2.0 * np.arctan2(n, dw)
    if angle > np.pi:
        angle -= 2.0 * np.pi
    if n < 1e-12:
        return np.zeros(3)
    axis = np.array([dx, dy, dz]) / n
    return axis * angle


def build_pc_fields():
    f = []
    f.append(PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1))
    f.append(PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1))
    f.append(PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1))
    f.append(PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1))
    f.append(PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1))
    f.append(PointField(name='time', offset=20, datatype=PointField.FLOAT32, count=1))
    return f


OUT_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
    'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
    'offsets': [0, 4, 8, 12, 16, 20],
    'itemsize': 24,
})


def convert_cloud(msg: PointCloud2) -> PointCloud2:
    # source layout is tightly packed x,y,z float32 (point_step 12)
    xyz = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3).astype(np.float64)
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    rng = np.sqrt(x * x + y * y + z * z)
    safe = np.maximum(rng, 1e-9)
    elev = np.degrees(np.arcsin(np.clip(z / safe, -1.0, 1.0)))
    az = np.degrees(np.arctan2(y, x))  # [-180,180]

    # ring: bin elevation into N_SCAN rows
    frac = (elev - ELEV_MIN_DEG) / (ELEV_MAX_DEG - ELEV_MIN_DEG)
    ring = np.clip((frac * N_SCAN).astype(np.int32), 0, N_SCAN - 1).astype(np.uint16)
    # time: position within the azimuth sweep
    t = ((az + 180.0) / 360.0) * SCAN_PERIOD

    out = np.zeros(len(xyz), dtype=OUT_DTYPE)
    out['x'] = x
    out['y'] = y
    out['z'] = z
    out['intensity'] = np.minimum(rng, 255.0)
    out['ring'] = ring
    out['time'] = t

    new = PointCloud2()
    new.header = msg.header
    new.height = 1
    new.width = len(xyz)
    new.fields = build_pc_fields()
    new.is_bigendian = False
    new.point_step = OUT_DTYPE.itemsize
    new.row_step = OUT_DTYPE.itemsize * len(xyz)
    new.is_dense = True
    new.data = out.tobytes()
    return new


def main():
    in_uri, out_uri = sys.argv[1], sys.argv[2]

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=in_uri, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=out_uri, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(
        name='/imu', type='sensor_msgs/msg/Imu', serialization_format='cdr'))
    writer.create_topic(rosbag2_py.TopicMetadata(
        name='/point_cloud', type='sensor_msgs/msg/PointCloud2', serialization_format='cdr'))

    prev_q = None
    prev_t = None
    n_imu = n_pc = 0
    while reader.has_next():
        topic, data, t = reader.read_next()

        if topic == '/imu':
            imu = deserialize_message(data, Imu)
            # epoch-align the header stamp with the recorded (DB) time
            imu.header.stamp.sec = int(t // 1_000_000_000)
            imu.header.stamp.nanosec = int(t % 1_000_000_000)
            # reconstruct angular velocity from orientation differences
            q = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
            t_sec = t * 1e-9
            if prev_q is not None and (t_sec - prev_t) > 1e-6:
                omega = quat_to_rotvec(prev_q, q) / (t_sec - prev_t)
                imu.angular_velocity.x = float(omega[0])
                imu.angular_velocity.y = float(omega[1])
                imu.angular_velocity.z = float(omega[2])
            prev_q, prev_t = q, t_sec
            writer.write('/imu', serialize_message(imu), t)
            n_imu += 1

        elif topic == '/point_cloud':
            pc = deserialize_message(data, PointCloud2)
            writer.write('/point_cloud', serialize_message(convert_cloud(pc)), t)
            n_pc += 1
            if n_pc % 500 == 0:
                print(f"  clouds: {n_pc}", flush=True)

    print(f"Done. imu={n_imu} clouds={n_pc}")


if __name__ == '__main__':
    main()
