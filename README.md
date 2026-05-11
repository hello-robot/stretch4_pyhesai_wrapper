# pyhesai_wrapper

This repository holds code that is intended to provide a Python interface to the Hesai JT128 hemispherical LiDAR.

# How to Build and Run

### Prerequisites:
* A C++17 compiler (like g++).
* cmake (version 3.14 or higher, e.g., `sudo apt install cmake`).
* Python 3.12+ and pip (or uv).
* Git (for cloning the Hesai SDK).
* The Hesai SDK's system dependencies: `libpcap-dev`, `libssl-dev` (e.g., `sudo apt install libpcap-dev libssl-dev`).

### Setup:

   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   ```

The build process is now fully automated. Simply run:

```bash
pip install .
```

This will:

1. Read pyproject.toml
1. Use scikit-build to run CMakeLists.txt.
1. CMake will find pybind11, the SDK headers, and the SDK libraries.
1. It will compile pybind_hesai_sdk.cpp and link it against all the .a and .so files.
1. It will create a Python module file (e.g., pyhesai_wrapper_cpp.cpython-310-x86_64-linux-gnu.so) and install it into your Python environment.
1. If the build is successful, the pyhesai_wrapper module is now installed and available to all Python scripts in your environment.

### Use in your python code

```python
from pyhesai_wrapper import stream_lidar_left

def main():
    # Continually yields LidarPointCloudFrame objects as long as it's running
    for frame in stream_lidar_left():
        if frame is not None:
            print(f"Points shape: {frame.points.shape}, timestamp: {frame.timestamp}")

if __name__ == "__main__":
    main()
```

#### The `LidarPointCloudFrame` Dataclass
When you fetch points using `lidar.get_next()` or via the streaming generators, the system returns a `LidarPointCloudFrame` object (or `None` if no new data is available yet). The properties of this object are:
* `points`: A NumPy array of shape `(N, 3)` containing the X, Y, and Z Cartesian coordinates of the captured points (`dtype=float32`).
* `intensity`: A NumPy 1D array of shape `(N,)` containing the return intensity values (`dtype=uint8`).
* `timestamp`: A NumPy 1D array of shape `(N,)` containing the microsecond tick timestamps for each point (`dtype=float64`).
* `confidence`: A NumPy 1D array of shape `(N,)` containing the confidence values (`dtype=uint8`).
* `ring`: A NumPy 1D array of shape `(N,)` containing the laser ring IDs (`dtype=uint16`).

### Tools:

#### Live Lidar test (`tools/stretch_lidar_show.py`):
1. Edit `pyhesai_wrapper/config.yaml` to configure your lidar settings:
   - Update `device_ip_address` to match your lidar's IP (default: `192.168.1.201`)
   - Update `correction_file_path` to point to your lidar's correction file
   - Optionally update other parameters like `udp_port`, `ptc_port`, etc.
2. Make sure your machine is on the same network as the lidar.
3. Run the script:
   ```bash
   stretch_lidar_show --left
   ```
   or
   ```bash
   stretch_lidar_show --right
   ```
4. You should see point cloud data streaming from the lidar. Press Ctrl-C to stop.
5. You may also visualize the points in rerun using the `--rerun` flag:
   ```bash
   stretch_lidar_show --left --rerun
   ```

#### Download calibration (`tools/REx_hesai_download_calibration.py`):
1. Edit `pyhesai_wrapper/config.yaml` to configure your lidar settings:
   - Update `device_ip_address` to match your lidar's IP (default: `192.168.1.201`)
   - Update `ptc_port` to match your lidar's PTC port (default: `9347`)
2. Make sure your machine is on the same network as the lidar.
3. Run the script:
   ```bash
   REx_hesai_download_calibration --left
   ```
   or
   ```bash
   REx_hesai_download_calibration --right
   ```
4. You should see calibration data being downloaded from the lidar to the  `$HELLO_FLEET_PATH/$HELLO_FLEET_ID/calibration_hesais`directory.