import argparse
from pyhesai_wrapper import HesaiLidar

def REx_hesai_download_calibration(use_right_lidar:bool):
    hesai = HesaiLidar(use_right_lidar=use_right_lidar)

    proceed = input(f"""
Downloading calibration file to {hesai.correction_file_path}, proceed? This will overwrite an existing file. [y/N]:  
""")
    if proceed.lower() != "y":
        return print("Aborting...")
    
    print(f"Connecting to {'Right' if use_right_lidar else 'Left'} LiDAR...")
    hesai.download_calibration()

def main():
    parser = argparse.ArgumentParser(description="Download calibration from Hesai LiDARs")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--left", action="store_true", help="Download from the left LiDAR")
    group.add_argument("--right", action="store_true", help="Download from the right LiDAR")
    
    args = parser.parse_args()
    
    use_right_lidar = args.right

    REx_hesai_download_calibration(use_right_lidar)

if __name__ == "__main__":
    main()
