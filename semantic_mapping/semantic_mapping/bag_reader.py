"""Read LiDAR frames and camera images from a ROS2 sqlite3 bag without needing ROS running."""
import sqlite3
import struct
import numpy as np
from pathlib import Path
from typing import Iterator, Tuple, Optional
import cv2


def _cdr_string(data: bytes, offset: int) -> Tuple[str, int]:
    length = struct.unpack_from("<I", data, offset)[0]
    offset += 4
    return data[offset:offset + length - 1].decode("utf-8", errors="ignore"), offset + length


def _align(offset: int, size: int) -> int:
    return (offset + size - 1) & ~(size - 1)


def parse_pointcloud2(data: bytes) -> Optional[np.ndarray]:
    """Return Nx4 array (x, y, z, intensity) from a CDR-serialized PointCloud2."""
    try:
        off = 4  # skip CDR header
        # header
        off += 4 + 4  # seq, stamp.sec
        off += 4       # stamp.nanosec
        frame_len = struct.unpack_from("<I", data, off)[0]; off += 4 + frame_len
        off = _align(off, 4)
        height, width = struct.unpack_from("<II", data, off); off += 8
        # fields: count
        nfields = struct.unpack_from("<I", data, off)[0]; off += 4
        field_info = {}
        for _ in range(nfields):
            name_len = struct.unpack_from("<I", data, off)[0]; off += 4
            name = data[off:off + name_len - 1].decode(); off += name_len
            offset_f, dtype, count = struct.unpack_from("<IBI", data, off); off += 9
            field_info[name] = (offset_f, dtype, count)
        is_bigendian = data[off]; off += 1
        point_step, row_step = struct.unpack_from("<II", data, off); off += 8
        data_len = struct.unpack_from("<I", data, off)[0]; off += 4
        raw = data[off:off + data_len]

        n_pts = height * width
        if n_pts == 0 or point_step == 0:
            return None

        x_off = field_info.get("x", (0,))[0]
        y_off = field_info.get("y", (4,))[0]
        z_off = field_info.get("z", (8,))[0]
        i_off = field_info.get("intensity", (None,))[0]

        pts = np.frombuffer(raw, dtype=np.uint8).reshape(n_pts, point_step)
        xyz = np.column_stack([
            pts[:, x_off:x_off+4].view(np.float32).flatten(),
            pts[:, y_off:y_off+4].view(np.float32).flatten(),
            pts[:, z_off:z_off+4].view(np.float32).flatten(),
        ])
        if i_off is not None:
            intensity = pts[:, i_off:i_off+4].view(np.float32).flatten()
        else:
            intensity = np.zeros(n_pts, dtype=np.float32)

        valid = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 0.1)
        return np.column_stack([xyz[valid], intensity[valid]])
    except Exception:
        return None


def parse_image(data: bytes) -> Optional[np.ndarray]:
    """Return HxWx3 BGR image from CDR-serialized sensor_msgs/Image."""
    try:
        off = 4  # CDR header
        off += 4 + 4 + 4  # seq, stamp
        frame_len = struct.unpack_from("<I", data, off)[0]; off += 4 + frame_len
        off = _align(off, 4)
        height, width = struct.unpack_from("<II", data, off); off += 8
        enc_len = struct.unpack_from("<I", data, off)[0]; off += 4
        encoding = data[off:off + enc_len - 1].decode(); off += enc_len
        off += 1  # is_bigendian
        step = struct.unpack_from("<I", data, off)[0]; off += 4
        data_len = struct.unpack_from("<I", data, off)[0]; off += 4
        raw = np.frombuffer(data[off:off + data_len], dtype=np.uint8)
        if "bgr8" in encoding or "rgb8" in encoding:
            img = raw.reshape(height, width, 3)
            if "rgb8" in encoding:
                img = img[:, :, ::-1]
            return img
        elif "mono8" in encoding or "8UC1" in encoding:
            return cv2.cvtColor(raw.reshape(height, width), cv2.COLOR_GRAY2BGR)
        elif "16UC1" in encoding or "mono16" in encoding:
            img16 = raw.view(np.uint16).reshape(height, width)
            img8 = (img16 >> 8).astype(np.uint8)
            return cv2.cvtColor(img8, cv2.COLOR_GRAY2BGR)
        return None
    except Exception:
        return None


class BagReader:
    def __init__(self, bag_path: str, lidar_topic: str, camera_topic: str = ""):
        # Handle both .db3 and directory bags
        p = Path(bag_path)
        if p.is_dir():
            candidates = list(p.glob("*.db3"))
            self.db_path = str(candidates[0]) if candidates else bag_path
        else:
            self.db_path = bag_path

        self.lidar_topic = lidar_topic
        self.camera_topic = camera_topic
        self._conn = None

    def _open(self):
        if self._conn is None:
            self._conn = sqlite3.connect(self.db_path)

    def _topic_id(self, topic: str) -> Optional[int]:
        self._open()
        row = self._conn.execute(
            "SELECT id FROM topics WHERE name=?", (topic,)
        ).fetchone()
        return row[0] if row else None

    def lidar_frames(self, every_n: int = 10, max_frames: int = 500) -> Iterator[Tuple[float, np.ndarray]]:
        """Yield (timestamp_sec, Nx4_array) for every_n-th LiDAR frame."""
        tid = self._topic_id(self.lidar_topic)
        if tid is None:
            raise ValueError(f"Topic {self.lidar_topic} not found in bag")
        self._open()
        count = 0
        for i, (ts_ns, data) in enumerate(self._conn.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)
        )):
            if i % every_n != 0:
                continue
            pts = parse_pointcloud2(bytes(data))
            if pts is not None and len(pts) > 100:
                yield ts_ns / 1e9, pts
                count += 1
                if count >= max_frames:
                    break

    def camera_frames(self, every_n: int = 10, max_frames: int = 500) -> Iterator[Tuple[float, np.ndarray]]:
        """Yield (timestamp_sec, BGR_image) for every_n-th camera frame."""
        if not self.camera_topic:
            return
        tid = self._topic_id(self.camera_topic)
        if tid is None:
            return
        self._open()
        count = 0
        for i, (ts_ns, data) in enumerate(self._conn.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)
        )):
            if i % every_n != 0:
                continue
            img = parse_image(bytes(data))
            if img is not None:
                yield ts_ns / 1e9, img
                count += 1
                if count >= max_frames:
                    break

    def list_topics(self):
        self._open()
        return [r[0] for r in self._conn.execute("SELECT name FROM topics")]
