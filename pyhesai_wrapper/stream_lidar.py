from typing import Generator
from pyhesai_wrapper.hesai_lidar import LidarPointCloudFrame, HesaiLidar

def stream_lidar_left() -> Generator[LidarPointCloudFrame | None, None, None]:
    lidar = HesaiLidar(use_right_lidar=False)
    lidar.start()
    try:
        while True:
            yield lidar.get_next()
    finally:
        lidar.stop()

def stream_lidar_left_blocking() -> Generator[LidarPointCloudFrame, None, None]:
    lidar = HesaiLidar(use_right_lidar=False)
    lidar.start()
    try:
        yield from lidar.read_stream()
    finally:
        lidar.stop()

def stream_lidar_right() -> Generator[LidarPointCloudFrame | None, None, None]:
    lidar = HesaiLidar(use_right_lidar=True)
    lidar.start()
    try:
        while True:
            yield lidar.get_next()
    finally:
        lidar.stop()

def stream_lidar_right_blocking() -> Generator[LidarPointCloudFrame, None, None]:
    lidar = HesaiLidar(use_right_lidar=True)
    lidar.start()
    try:
        yield from lidar.read_stream()
    finally:
        lidar.stop()
