import queue
import threading
from typing import Generator
from stretch4_pyhesai_wrapper.hesai_lidar import LidarPointCloudFrame, HesaiLidar
from itertools import chain

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

def stream_lidar_both() -> Generator[tuple[LidarPointCloudFrame | None,LidarPointCloudFrame | None], None, None]:
    right = HesaiLidar(use_right_lidar=True, queue_size=3)
    left = HesaiLidar(use_right_lidar=False, queue_size=3)

    pair_queue = queue.Queue(maxsize=3)
    recv_lock = threading.Lock()
    slop = 0.06
    def recv(msg, name):
        with recv_lock:
            other_queue = right.frame_queue if name == 'left' else left.frame_queue
            with other_queue.mutex:
                frames = list(other_queue.queue)
            delta = lambda f: abs(f.timestamp[0] - msg.timestamp[0])
            of = min(frames, key=delta, default=None)
            if of is None or delta(of) >= slop:
                return
            left_frame = msg if name == 'left' else of
            right_frame = of if name == 'left' else msg
            if pair_queue.full():
                pair_queue.get_nowait()
            pair_queue.put_nowait((left_frame, right_frame))

    right.registerCallback(recv, right.side)
    left.registerCallback(recv, left.side)
    right.start()
    left.start()
    try:
        while True:
            try:
                # timeout to be reponsive to keyboard interrupts
                yield pair_queue.get(timeout=0.5)
            except queue.Empty:
                continue
    finally:
        right.stop()
        left.stop()
