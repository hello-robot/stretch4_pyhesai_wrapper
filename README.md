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

Both Left and Right Lidars:
```python

from pyhesai_wrapper import stream_left_right_lidar

for left, right in stream_left_right_lidar():
   if left is not None:
      print(f"Points shape: {left.points.shape}, timestamp: {left.timestamp}")
   if right is not None:
      print(f"Points shape: {right.points.shape}, timestamp: {right.timestamp}")
```

Left Lidar:

```python
from pyhesai_wrapper import stream_left_lidar

for frame in stream_left_lidar():
   if frame is not None:
      print(f"Points shape: {frame.points.shape}, timestamp: {frame.timestamp}")
```

Right Lidar:

```python
from pyhesai_wrapper import stream_right_lidar

for frame in stream_right_lidar():
   if frame is not None:
      print(f"Points shape: {frame.points.shape}, timestamp: {frame.timestamp}")
```

Alternatively, you can poll the next frame using `next()`:

```python
left, right  = stream_left_right_lidar()
left_frame = next(left)
right_frame = next(right)
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
   stretch_lidar_show
   stretch_lidar_show --cluster_high_intensity
   stretch_lidar_show --left
   stretch_lidar_show --right
   ```

   > Note: You can  cluster and display the Euclidean distance to high intensity points by passing the `--cluster_high_intensity` flag
   
4. You should see point cloud data streaming from the lidar. Press Ctrl-C to stop.

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

#### PTC getters/setters (`pyhesai_wrapper/ptc_client.py`):

SDK-backed JT128 PTC client for return mode, point-cloud filter, PTP lock offset, diagnostics, and reachability checks.

```python
from pyhesai_wrapper.ptc_client import (
    FILTER_STRONG,
    get_point_cloud_config,
    get_return_mode,
    set_filter_type,
    set_return_mode,
    get_ptp_lock_offset_us,
    ptc_reachable,
)

if ptc_reachable('192.168.1.201'):
    print(get_return_mode('192.168.1.201'))
    set_return_mode('192.168.1.201', 2)
    set_filter_type('192.168.1.201', FILTER_STRONG)  # ultra_precise unchanged
    print(get_point_cloud_config('192.168.1.201'))
```

#### Show configuration (`REx_hesai_show_config`):

To view complete lidar information, return mode, spin rate, PTP status, and point cloud settings:

```bash
# Show config/status for both lidars
REx_hesai_show_config

# Show config/status for a specific lidar
REx_hesai_show_config --left
REx_hesai_show_config --right
```

This retrieves the serial number, model, hardware and software versions, build ID, MAC address, return mode, spin rate, lock offset, ultra-precise mode, noise filter type, PTP status, and active PTP master offset (if PTP is synchronized).

#### Modify configuration (`REx_hesai_set_config`):

> [!WARNING]
> Modifying the LiDAR hardware configuration can disrupt the normal operation of your robot. Be cautious when using this utility.

An interactive tool to adjust hardware settings on a specific lidar:

```bash
# Configure left lidar
REx_hesai_set_config --left

# Configure right lidar
REx_hesai_set_config --right
```

After accepting the warning, you can select from the interactive options:
* **10** - Set Return Mode (0 to 5)
* **11** - Set Spin Speed (600 or 1200 RPM)
* **12** - Set PTP Lock Offset (1 to 1000 us)
* **13** - Set Noise Filter Type (0 to 2)

Each setting operation performs a baseline GET, followed by the SET command, and finishes with a readback verification GET to guarantee that the hardware successfully applied the modification.

#### Standalone PTC bench test:

You can run the standalone PTC test menu directly:

```bash
python3 test/ptc_test.py --left
```

