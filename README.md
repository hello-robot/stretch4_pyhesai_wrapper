# stretch4_pyhesai_wrapper

This repository holds code that provides a Python interface to the Hesai JT128 hemispherical LiDAR.

This package can be installed by: 
```
python3 -m pip install -U hello-robot-stretch4-pyhesai-wrapper
```

## Details

A generator pattern to stream both left and right LiDARs. Internally, a synchronizer ensures left and right frames are within 60ms of each other. Frame pairs are not guaranteed to return at 10hz. There can be degraded rate or even drop out for multiple seconds at a time, so **please implement a watchdog for safety applications**. You can 1) run a watchdog thread, or 2) the generator yields `None` if no pair has arrived for `timeout` seconds (0.5s by default), and keeps yielding `None` every `timeout` until the lidars recover.
```python

from stretch4_pyhesai_wrapper import stream_lidar_both

for pair in stream_lidar_both():
   if pair is None:
      robot.omnibase.hard_stop(); robot.push_command()
      continue  # degraded or dropped out - stop the robot here, don't reuse the last pair
   left, right = pair
   print(f"Points shape: {left.points.shape}, timestamp: {left.timestamp}")
   print(f"Points shape: {right.points.shape}, timestamp: {right.timestamp}")
```

Left Lidar:

```python
from stretch4_pyhesai_wrapper import stream_lidar_left

for frame in stream_lidar_left():
   if frame is not None:
      print(f"Points shape: {frame.points.shape}, timestamp: {frame.timestamp}")
```

> Note: all three generators can yield `None`, but for different reasons. `stream_lidar_left()` and `stream_lidar_right()` are non-blocking and yield `None` whenever no new frame is available *yet*, which is the normal case between sweeps — use `stream_lidar_left_blocking()` / `stream_lidar_right_blocking()` to block until a frame arrives and never receive `None`. `stream_lidar_both()` blocks until a synchronized pair is available and yields `None` only under **degradation or dropout** — no pair for `timeout` seconds (0.5s by default). That `None` is not routine: it means the pair stream has stalled, so handle it as a fault rather than skipping past it. Pass `stream_lidar_both(timeout=None)` to block indefinitely and never receive `None`.

Right Lidar:

```python
from stretch4_pyhesai_wrapper import stream_lidar_right

for frame in stream_lidar_right():
   if frame is not None:
      print(f"Points shape: {frame.points.shape}, timestamp: {frame.timestamp}")
```

Alternatively, you can poll the next frame using `next()`:

```python
pairs = stream_lidar_both()
pair = next(pairs)          # None if the stream is degraded or dropped out
if pair is not None:
   left_frame, right_frame = pair
```

### The `LidarPointCloudFrame` Dataclass
When you fetch points using `lidar.get_next()` or via the streaming generators, the system returns a `LidarPointCloudFrame` object (or `None` if no new data is available yet). The properties of this object are:
* `points`: A NumPy array of shape `(N, 3)` containing the X, Y, and Z Cartesian coordinates of the captured points (`dtype=float32`).
* `intensity`: A NumPy 1D array of shape `(N,)` containing the return intensity values (`dtype=uint8`).
* `timestamp`: A NumPy 1D array of shape `(N,)` containing the microsecond tick timestamps for each point (`dtype=float64`).
* `confidence`: A NumPy 1D array of shape `(N,)` containing the confidence values (`dtype=uint8`).
* `ring`: A NumPy 1D array of shape `(N,)` containing the laser ring IDs (`dtype=uint16`).
* `frame_start_timestamp`: in seconds, This is the timestamp of the first packet the SDK sees (which clock depends on `use_timestamp_type`)

### Tools:

#### Live Lidar test (`tools/stretch_lidar_show.py`):
1. Edit `stretch4_pyhesai_wrapper/config.yaml` to configure your lidar settings:
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
1. Edit `stretch4_pyhesai_wrapper/config.yaml` to configure your lidar settings:
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

#### PTC getters/setters (`stretch4_pyhesai_wrapper/ptc_client.py`):

SDK-backed JT128 PTC client for return mode, point-cloud filter, PTP lock offset, diagnostics, and reachability checks.

```python
from stretch4_pyhesai_wrapper.ptc_client import (
    FILTER_STRONG,
    FILTER_STRONGEST,
    POINT_CLOUD_MODE_MAPPING,
    get_point_cloud_config,
    get_point_cloud_mode,
    get_return_mode,
    is_new_firmware_supported,
    set_filter_type,
    set_point_cloud_mode,
    set_return_mode,
    get_ptp_lock_offset_us,
    ptc_reachable,
)

ip = '192.168.1.201'
if ptc_reachable(ip):
    print(get_return_mode(ip))
    set_return_mode(ip, 2)
    set_filter_type(ip, FILTER_STRONG)  # ultra_precise unchanged
    print(get_point_cloud_config(ip))

    # Strongest filter (3) and POINT_CLOUD_MODE need FW
    # 15.AF.B0.00.02.Y / 1.b.0028 / 2.b.0692
    if is_new_firmware_supported(ip):
        set_filter_type(ip, FILTER_STRONGEST)
        set_point_cloud_mode(ip, POINT_CLOUD_MODE_MAPPING)  # 0 general, 1 mapping, 2 mapping+ground
        print(get_point_cloud_mode(ip))
```

Noise filter levels: `0` disabled, `1` medium, `2` strong, `3` strongest (new FW only).

`is_new_firmware_supported()` is the single firmware gate used by Strongest filter and POINT_CLOUD_MODE. Per Hesai, it requires all three inventory patches at or above `15.AF.B0.00.02.Y` / `1.b.0028` / `2.b.0692` (wrapper fields `hardware_version`, `software_version`, `fpga_version`). When a newer mass-production firmware ships, re-check Hesai’s version naming (especially if APP moves past `…02.Z` / to `…03.X`) and update that function.
#### Show configuration (`REx_hesai_show_config`):

To view complete lidar information, return mode, spin rate, PTP status, and point cloud settings:

```bash
# Show config/status for both lidars
REx_hesai_show_config

# Show config/status for a specific lidar
REx_hesai_show_config --left
REx_hesai_show_config --right
```

This retrieves the serial number, model, hardware and software versions, build ID, MAC address, whether new-FW features are supported, return mode, spin rate, lock offset, ultra-precise mode, noise filter type, point-cloud mode (when supported), PTP status, and active PTP master offset (if PTP is synchronized).

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
* **13** - Set Noise Filter Type (0 to 3; `3` / strongest requires new FW)
* **14** - Set Point Cloud Mode (0 to 2; requires new FW)

Each setting operation performs a baseline GET, followed by the SET command, and finishes with a readback verification GET to guarantee that the hardware successfully applied the modification.

#### Upgrade firmware (`REx_hesai_upgrade_firmware`):

> [!WARNING]
> Do not power off the lidar during upgrade. The unit reboots after a successful transfer. Upgrade one lidar at a time.

Uploads a **Hesai-provided** JT128 firmware patch via PTC Upgrade Safe Image (`0x83`) and prints transfer progress. The firmware file is not shipped in this repo; obtain it from Hesai.

```bash
# Right lidar (interactive confirm)
REx_hesai_upgrade_firmware --right --firmware /path/to/JT128_upgrade.patch

# Left lidar, skip confirm prompt
REx_hesai_upgrade_firmware --left --firmware /path/to/JT128_upgrade.patch -y

# Explicit IP
REx_hesai_upgrade_firmware 192.168.1.201 --firmware /path/to/JT128_upgrade.patch
```

Optional flags: `--timeout` (PTC connect timeout, default 30s), `--reboot-wait` (wait for lidar to return after transfer, default 120s), `-y` / `--yes` (skip confirmation).

The tool prints inventory versions before upload, streams `Progress: xx.x%`, waits for reboot, then prints versions again.
The version after upgrade might not show all the version 
```
Versions after upgrade:
  [after]
  Hardware Version:     15.AF.B0.00.02.Y0
  Software/Firmware:    1.b.0028
  FPGA Version:         
  Build/Signature ID:   0x00000000
```
You can run `REx_hesai_show_config` and check the inventory info

```
  INVENTORY INFO
  ------------------------------------------------------------------
  Model:                JT128
  Serial Number:        JT3AC9509338CB50
  MAC Address:          ec:9f:0d:02:f1:cd
  Calibration/Mfg Date: 2025-03-05
  Hardware Version:     15.AF.B0.00.02.Y0
  Software/Firmware:    1.b.0028
  FPGA Version:         2.b.0692
  Build/Signature ID:   0x791C2330
  New FW Features:      supported
```
#### Standalone PTC bench test:

You can run the standalone PTC test menu directly:

```bash
python3 test/ptc_test.py --left
```

## Building

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
1. If the build is successful, the stretch4_pyhesai_wrapper module is now installed and available to all Python scripts in your environment.