import time

from .pyhesai_wrapper_cpp import * # Import everything from the C++ wrapper

from dataclasses import dataclass
import logging
import queue
import os
import numpy as np
import yaml

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.yaml")

with open(CONFIG_PATH, "r") as f:
    _config_str = os.path.expandvars(f.read())
    CONFIG = yaml.safe_load(_config_str)

@dataclass
class LidarPointCloudFrame:
    points: np.ndarray # x, y, z
    intensity: np.ndarray # intensity
    timestamp: np.ndarray # timestamps corresponding to each point
    timestamp_system: float 
    confidence: np.ndarray
    ring: np.ndarray
    name: str|None = None
    # Start of sweep, seconds, on whichever clock use_timestamp_type selects.
    # Same quantity the ROS driver puts in header.stamp
    frame_start_timestamp: float = 0.0

    @staticmethod
    def from_named_numpy_array(pc_array: np.ndarray, lidar_name:str|None=None,
                               frame_start_timestamp: float = 0.0) -> 'LidarPointCloudFrame':
        """
        The logic for unpacking the lidar point cloud assumes the point cloud is from a Hesai lidar.
        """
        points = np.empty((pc_array.size, 3), dtype=np.float32)
        points[:, 0] = pc_array['x']
        points[:, 1] = pc_array['y']
        points[:, 2] = pc_array['z']
        
        return LidarPointCloudFrame(
            points=points,
            intensity=pc_array['intensity'],
            timestamp=pc_array['timestamp'],
            timestamp_system=time.time(),
            confidence=pc_array['confidence'],
            ring=pc_array['ring'],
            name=lidar_name,
            frame_start_timestamp=frame_start_timestamp
        )


class HesaiLidar():
    FULL_SWEEP_RATIO = 0.99

    def __init__(self, use_right_lidar=False, queue_size=2, quiet=True):
        self.quiet = quiet

        logging.basicConfig(level=logging.INFO)

        self.side = "right" if use_right_lidar else "left"
        self.cfg = CONFIG[f"{self.side}_lidar"]

        self.ip = self.cfg["ip"]
        self.ptc_port = self.cfg["ptc_port"]
        self.correction_file_path = self.cfg["correction_file"]

        self.frame_queue = queue.Queue(maxsize=queue_size)
        self.on_msg_cbs = []

        self.connected = False
        self.started = False

        self._full_points = 0
        self.partial_frames_dropped = 0

    def registerCallback(self, callback, *args):
        """Expects a callback that can be called
        method(frame: LidarPointCloudFrame, *args)
        """
        self.on_msg_cbs.append((callback, args))

    def connect(self):
        # Instatiate PyRSDriver. quiet= suppresses the SDK's version banner,
        # which the constructor prints straight to std::cout.
        self.lidar_driver = HesaiLidarSdk_XYZICRT(quiet=self.quiet)

        param = DriverParam()

        # --- Parameters from test.cc ---
        param.log_Target = 0x00 if self.quiet else 0x01 # HESAI_LOG_TARGET_NONE / HESAI_LOG_TARGET_CONSOLE from libhesai's logger.h
        param.use_gpu = False
        param.input_param.source_type = SourceType.DATA_FROM_LIDAR
        param.input_param.device_ip_address = self.ip
        param.input_param.udp_port = self.cfg["udp_port"]
        param.input_param.use_ptc_connected = self.cfg["ptc_connected"]
        param.input_param.ptc_port = self.ptc_port
        param.input_param.correction_file_path = self.correction_file_path
        
        self._full_points = 0

        def pointcloud_callback(frame):
            # Partial sweeps, e.g. like the first one after Start()
            # are reported as a N<115200 frame whose frame_start_timestamp
            # is the first packet's time rather than the azimuth-0 boundary.
            # Drop these partial sweeps.
            full, n = self._full_points, frame.points_num
            self._full_points = max(full, n)
            if full == 0:
                logging.info("%s lidar: dropped the partial sweep in progress at startup "
                              "(%d points)", self.side, n)
                return
            if n < self.FULL_SWEEP_RATIO * full:
                self.partial_frames_dropped += 1
                logging.warning("%s lidar: dropped a partial sweep (%d of %d points) - "
                                "packets lost or delayed", self.side, n, full)
                return

            pc_array = np.array(frame.points)
            lidar_frame = LidarPointCloudFrame.from_named_numpy_array(pc_array, self.side, frame.frame_start_timestamp)
            try:
                self.frame_queue.put_nowait(lidar_frame)
            except queue.Full:
                # Queue is full: drop the oldest frame to make space. This
                # thread is the only producer, so the retried put cannot fail.
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
                self.frame_queue.put_nowait(lidar_frame)

            # a failing callback must not kill the SDK's C++ callback thread
            for (cb, args) in self.on_msg_cbs:
                try:
                    cb(lidar_frame, *args)
                except Exception:
                    logging.exception("on_msg callback %r failed", cb)

        logging.debug(f"Initializing SDK for Lidar at {param.input_param.device_ip_address}...")
        if not self.lidar_driver.Init(param):
            logging.error("SDK Init failed. Check connection and correction file path.")
            raise RuntimeError("SDK Init failed. Check connection and correction file path.")

        self.lidar_driver.RegRecvCallback(pointcloud_callback)

        self.connected = True

    def start(self):
        if not self.connected:
            self.connect()
        self.lidar_driver.Start()
        self.started = True

    def stop(self):
        if hasattr(self, 'lidar_driver') and self.lidar_driver is not None:
            self.lidar_driver.Stop()
            self.lidar_driver = None
        self.started = False
        self.connected = False

    def get_next(self) -> LidarPointCloudFrame | None:
        try:
            return self.frame_queue.get_nowait()
        except queue.Empty:
            return None

    def read_stream(self):
        """Yields a generator with LidarPointCloudFrame."""
        while self.started:
            try:
                yield self.frame_queue.get(timeout=1.0)
            except queue.Empty:
                continue


    def __del__(self):
        if hasattr(self, 'started') and self.started:
            self.stop()
            
    def download_calibration(self, save_path:str|None=None):
        import os
        from .pyhesai_wrapper_cpp import download_calibration_bytes
        

        if save_path is None:
            save_path = self.correction_file_path
            
        logging.info(f"Downloading calibration from {self.ip}:{self.ptc_port}...")
        calib_bytes = download_calibration_bytes(self.ip, self.ptc_port)
        
        # Ensure directory exists
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        
        with open(save_path, "wb") as f:
            f.write(calib_bytes)
        print(f"Calibration successfully saved to {save_path}")
