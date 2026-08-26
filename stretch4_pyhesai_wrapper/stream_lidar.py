import queue
import threading
import time
from typing import Generator
from stretch4_pyhesai_wrapper.hesai_lidar import LidarPointCloudFrame, HesaiLidar


PAIR_TIMEOUT_S = 0.5 # How long stream_lidar_both() waits before yielding None
_POLL_S = 0.1


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

def stream_lidar_both(timeout: float | None = PAIR_TIMEOUT_S) -> Generator[tuple[LidarPointCloudFrame, LidarPointCloudFrame] | None, None, None]:
    right = HesaiLidar(use_right_lidar=True, queue_size=3)
    left = HesaiLidar(use_right_lidar=False, queue_size=3)

    last_pair = {'left': float('-inf'), 'right': float('-inf')}
    pair_queue = queue.Queue(maxsize=3)
    recv_lock = threading.Lock()
    slop = 0.06
    def recv(msg, name):
        with recv_lock:
            # never reuse a frame already emitted, and never emit out of order
            if msg.frame_start_timestamp <= last_pair[name]:
                return
            other = 'right' if name == 'left' else 'left'
            other_queue = right.frame_queue if name == 'left' else left.frame_queue
            with other_queue.mutex:
                frames = list(other_queue.queue)
            frames = [f for f in frames if f.frame_start_timestamp > last_pair[other]]
            delta = lambda f: abs(f.frame_start_timestamp - msg.frame_start_timestamp)
            of = min(frames, key=delta, default=None)
            if of is None or delta(of) >= slop:
                return
            last_pair[name] = msg.frame_start_timestamp
            last_pair[other] = of.frame_start_timestamp
            left_frame = msg if name == 'left' else of
            right_frame = of if name == 'left' else msg
            try:
                pair_queue.put_nowait((left_frame, right_frame))
            except queue.Full:
                try:
                    pair_queue.get_nowait()
                except queue.Empty:
                    pass
                pair_queue.put_nowait((left_frame, right_frame))

    right.registerCallback(recv, right.side)
    left.registerCallback(recv, left.side)
    try:
        right.start()
        left.start()
        wait = _POLL_S if timeout is None else min(_POLL_S, timeout)
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            try:
                pair = pair_queue.get(timeout=wait)
            except queue.Empty:
                if deadline is None or time.monotonic() < deadline:
                    continue
                deadline = time.monotonic() + timeout
                yield None
                continue
            if deadline is not None:
                deadline = time.monotonic() + timeout
            yield pair
    finally:
        right.stop()
        left.stop()
