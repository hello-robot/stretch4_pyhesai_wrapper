import argparse
import time
import numpy as np

from pyhesai_wrapper import HesaiLidar

from stretch4_urdf import get_urdf_from_robot_params, get_transform
urdf_contents = get_urdf_from_robot_params(apply_calibration=True)



def stretch_show_lidar(use_left: bool, use_right: bool, use_rerun: bool = True, no_transform: bool = False):
    import signal
    import os

    def sigint_handler(signum, frame):
        print("\nStopping LiDARs...")
        os._exit(0)

    signal.signal(signal.SIGINT, sigint_handler)
    signal.signal(signal.SIGTERM, sigint_handler)

    if use_rerun:
        import rerun as rr
        rr.init("stretch_show_lidar", spawn=True)

    if not no_transform:
        print("Transforming point clouds to base_footprint frame using calibrated URDF.")

    lidars = []

    T_left, T_right = None, None
    
    if use_left:
        print("Connecting to Left LiDAR...")
        hesai_left = HesaiLidar(use_right_lidar=False)
        hesai_left.start()
        lidars.append(("lidar/left", hesai_left))
        T_left = get_transform(urdf_contents, "lidar_left_link", "base_footprint")
        
    if use_right:
        print("Connecting to Right LiDAR...")
        hesai_right = HesaiLidar(use_right_lidar=True)
        hesai_right.start()
        lidars.append(("lidar/right", hesai_right))
        T_right = get_transform(urdf_contents, "lidar_right_link", "base_footprint")

    try:
        while True:
            for name, lidar in lidars:
                frame = lidar.get_next()
                if frame is not None:
                    points = frame.points
                    if not no_transform:
                        T = T_left if name == "lidar/left" else T_right
                        if T is not None and points is not None and points.shape[0] > 0:
                            ones = np.ones((points.shape[0], 1), dtype=points.dtype)
                            pts_hom = np.hstack([points, ones])
                            points = (pts_hom @ T.T)[:, :3]

                    if points is not None and points.shape[0] > 0:
                        high_intensity_mask = frame.intensity > 240
                        high_intensity_points = points[high_intensity_mask]
                    else:
                        high_intensity_points = np.zeros((0, 3), dtype=np.float32)

                    if use_rerun:
                        rr.log(f"{name}/points", rr.Points3D(positions=points))
                        rr.log(f"{name}/high_intensity", rr.Points3D(positions=high_intensity_points,radii=0.005))
                    else:
                        print(f"{name}: {points.shape=}, high_intensity={high_intensity_points.shape[0]}, {frame.timestamp=}")
                else:
                    if not use_rerun:
                        pass # avoid spamming if printing text
            time.sleep(1/10)
    except KeyboardInterrupt:
        print("\nStopping LiDARs...")
        os._exit(0)

def main():
    parser = argparse.ArgumentParser(
        description="Test connection to Hesai LiDAR. By default, point clouds are transformed to the base_footprint frame (using the calibrated URDF if possible) unless --no-transform is specified."
    )
    parser.add_argument("--left", action="store_true", help="Connect to the left LiDAR only")
    parser.add_argument("--right", action="store_true", help="Connect to the right LiDAR only")
    parser.add_argument("--print", action="store_false", help="Plot points to rerun")
    parser.add_argument("--no-transform", action="store_true", help="Skip transforming point clouds to the base_footprint frame")
    
    args = parser.parse_args()
    
    use_left = args.left
    use_right = args.right
    
    if not use_left and not use_right:
        use_left = True
        use_right = True
        
    stretch_show_lidar(use_left, use_right, use_rerun=args.print, no_transform=args.no_transform)

if __name__ == "__main__":
    main()