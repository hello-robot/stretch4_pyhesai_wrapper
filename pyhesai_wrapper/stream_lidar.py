from typing import Generator
from pyhesai_wrapper.hesai_lidar import LidarPointCloudFrame, HesaiLidar
from itertools import chain

def stream_left_lidar() -> Generator[LidarPointCloudFrame | None, None, None]:
    lidar = HesaiLidar(use_right_lidar=False)
    lidar.start()
    try:
        while True:
            yield lidar.get_next()
    finally:
        lidar.stop()

def stream_left_lidar_blocking() -> Generator[LidarPointCloudFrame, None, None]:
    lidar = HesaiLidar(use_right_lidar=False)
    lidar.start()
    try:
        yield from lidar.read_stream()
    finally:
        lidar.stop()

def stream_right_lidar() -> Generator[LidarPointCloudFrame | None, None, None]:
    lidar = HesaiLidar(use_right_lidar=True)
    lidar.start()
    try:
        while True:
            yield lidar.get_next()
    finally:
        lidar.stop()

def stream_right_lidar_blocking() -> Generator[LidarPointCloudFrame, None, None]:
    lidar = HesaiLidar(use_right_lidar=True)
    lidar.start()
    try:
        yield from lidar.read_stream()
    finally:
        lidar.stop()

def stream_left_right_lidar() -> Generator[tuple[LidarPointCloudFrame | None,LidarPointCloudFrame | None], None, None]:
    yield from zip(stream_left_lidar(), stream_right_lidar())