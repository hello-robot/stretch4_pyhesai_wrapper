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

    @staticmethod
    def from_named_numpy_array(pc_array: np.ndarray) -> 'LidarPointCloudFrame':
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
            ring=pc_array['ring']
        )


class HesaiLidar():

    def __init__(self, use_right_lidar=False):

        logging.basicConfig(level=logging.INFO)

        side = "right" if use_right_lidar else "left"
        self.cfg = CONFIG[f"{side}_lidar"]

        self.ip = self.cfg["ip"]
        self.ptc_port = self.cfg["ptc_port"]
        self.correction_file_path = self.cfg["correction_file"]

        self.frame_queue = queue.Queue(maxsize=2)

        self.connected = False
        self.started = False
    
    def connect(self):
        # Instatiate PyRSDriver
        self.lidar_driver = HesaiLidarSdk_XYZICRT()

        param = DriverParam()

        # --- Parameters from test.cc ---
        param.use_gpu = False
        param.input_param.source_type = SourceType.DATA_FROM_LIDAR
        param.input_param.device_ip_address = self.ip
        param.input_param.udp_port = self.cfg["udp_port"]
        param.input_param.use_ptc_connected = self.cfg["ptc_connected"]
        param.input_param.ptc_port = self.ptc_port
        param.input_param.correction_file_path = self.correction_file_path
        
        def pointcloud_callback(frame):
            pc_array = np.array(frame.points)
            lidar_frame = LidarPointCloudFrame.from_named_numpy_array(pc_array)
            try:
                # If the queue is full, remove the oldest frame to make space
                if self.frame_queue.full():
                    self.frame_queue.get_nowait()
                self.frame_queue.put_nowait(lidar_frame)
            except queue.Empty:
                pass
            except queue.Full:
                pass

        logging.info(f"Initializing SDK for Lidar at {param.input_param.device_ip_address}...")
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
        self.lidar_driver.Stop()
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
