import argparse
import time

from stretch4_pyhesai_wrapper import HesaiLidar

def stretch_show_lidar(use_left: bool, use_right: bool, use_rerun:bool=True):
    if use_rerun:
        import rerun as rr
        rr.init("stretch_show_lidar", spawn=True)

    lidars = []
    
    if use_left:
        print("Connecting to Left LiDAR...")
        hesai_left = HesaiLidar(use_right_lidar=False)
        hesai_left.start()
        lidars.append(("lidar/left", hesai_left))
        
    if use_right:
        print("Connecting to Right LiDAR...")
        hesai_right = HesaiLidar(use_right_lidar=True)
        hesai_right.start()
        lidars.append(("lidar/right", hesai_right))

    try:
        while True:
            for name, lidar in lidars:
                frame = lidar.get_next()
                if frame is not None:
                    if use_rerun:
                        rr.log(f"{name}/points", rr.Points3D(positions=frame.points))
                    else:
                        print(f"{name}: {frame.points.shape=}, {frame.timestamp=}")
                else:
                    if not use_rerun:
                        pass # avoid spamming if printing text
            time.sleep(1/10)
    except KeyboardInterrupt:
        print("Stopping LiDARs...")
        for name, lidar in lidars:
            lidar.stop()

def main():
    parser = argparse.ArgumentParser(description="Test connection to Hesai LiDAR")
    parser.add_argument("--left", action="store_true", help="Connect to the left LiDAR only")
    parser.add_argument("--right", action="store_true", help="Connect to the right LiDAR only")
    parser.add_argument("--print", action="store_false", help="Plot points to rerun")
    
    args = parser.parse_args()
    
    use_left = args.left
    use_right = args.right
    
    if not use_left and not use_right:
        use_left = True
        use_right = True
        
    stretch_show_lidar(use_left, use_right, use_rerun=args.print)

if __name__ == "__main__":
    main()