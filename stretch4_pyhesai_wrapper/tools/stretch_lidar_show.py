import argparse
import time
import numpy as np

from stretch4_pyhesai_wrapper import HesaiLidar, stream_lidar_left, stream_lidar_right, stream_lidar_left_right, LidarPointCloudFrame


from stretch4_urdf import get_urdf_from_robot_params, get_transform
urdf_contents = get_urdf_from_robot_params(apply_calibration=True)


def stretch_show_lidar(use_left: bool, use_right: bool, use_rerun: bool = True, no_transform: bool = False, cluster_high_intensity: bool = False):
    import signal
    import os

    if use_rerun:
        import rerun as rr
        rr.init("stretch_show_lidar", spawn=True)

    if not no_transform:
        print("Transforming point clouds to base_footprint frame using calibrated URDF.")


    T_left = get_transform(urdf_contents, "lidar_left_link", "base_footprint")
    T_right = get_transform(urdf_contents, "lidar_right_link", "base_footprint")

    def _handle_frame(frame:LidarPointCloudFrame):

            if frame is not None:
                # Keep local points for sensor-relative distance/range calculation
                points_local = frame.points
                points = points_local.copy() if points_local is not None else None

                if not no_transform:
                    T = T_left if frame.name == "left" else T_right
                    if T is not None and points is not None and points.shape[0] > 0:
                        ones = np.ones((points.shape[0], 1), dtype=points.dtype)
                        pts_hom = np.hstack([points, ones])
                        points = (pts_hom @ T.T)[:, :3]

                if cluster_high_intensity and points_local is not None and points_local.shape[0] > 0:
                    high_intensity_mask = frame.intensity > 240
                    high_intensity_points_local = points_local[high_intensity_mask]
                    centroids_local = []
                    centroid_labels = []
                    
                    if high_intensity_points_local.shape[0] > 0:
                        from sklearn.cluster import DBSCAN
                        db = DBSCAN(eps=0.15, min_samples=4).fit(high_intensity_points_local)
                        labels = db.labels_
                        
                        unique_labels = set(labels)
                        for label in unique_labels:
                            if label == -1:
                                continue
                            cluster_pts_local = high_intensity_points_local[labels == label]
                            centroid_local = np.mean(cluster_pts_local, axis=0)
                            centroids_local.append(centroid_local)
                            
                            # Distance is the true range from physical sensor center (0, 0, 0)
                            dist = np.linalg.norm(centroid_local)
                            centroid_labels.append(f"{dist:.2f}m")
                    
                    centroids_local = np.array(centroids_local, dtype=np.float32) if len(centroids_local) > 0 else np.zeros((0, 3), dtype=np.float32)
                    
                    high_intensity_points = high_intensity_points_local.copy()
                    centroids = centroids_local.copy()
                    
                    # Apply transformation for correct 3D visualization alignment
                    if not no_transform:
                        T = T_left if frame.name == "left" else T_right
                        if T is not None:
                            if high_intensity_points.shape[0] > 0:
                                ones = np.ones((high_intensity_points.shape[0], 1), dtype=high_intensity_points.dtype)
                                pts_hom = np.hstack([high_intensity_points, ones])
                                high_intensity_points = (pts_hom @ T.T)[:, :3]
                            
                            if centroids.shape[0] > 0:
                                ones = np.ones((centroids.shape[0], 1), dtype=centroids.dtype)
                                pts_hom = np.hstack([centroids, ones])
                                centroids = (pts_hom @ T.T)[:, :3]
                elif points_local is not None and points_local.shape[0] > 0:
                    high_intensity_mask = frame.intensity > 240
                    high_intensity_points = points_local[high_intensity_mask]
                    centroids = None
                    
                    if not no_transform:
                        T = T_left if frame.name == "left" else T_right
                        if T is not None and high_intensity_points.shape[0] > 0:
                            ones = np.ones((high_intensity_points.shape[0], 1), dtype=high_intensity_points.dtype)
                            pts_hom = np.hstack([high_intensity_points, ones])
                            high_intensity_points = (pts_hom @ T.T)[:, :3]
                else:
                    high_intensity_points = np.zeros((0, 3), dtype=np.float32)
                    centroids = None

                if use_rerun:
                    rr.log(f"{frame.name}/points", rr.Points3D(positions=points))
                    if cluster_high_intensity:
                        if high_intensity_points is not None:
                            rr.log(f"{frame.name}/high_intensity", rr.Points3D(positions=high_intensity_points, radii=0.005))
                        if centroids is not None and centroids.shape[0] > 0:
                            rr.log(f"{frame.name}/centroids", rr.Points3D(positions=centroids, labels=centroid_labels, radii=0.005))
                        else:
                            rr.log(f"{frame.name}/centroids", rr.Points3D(positions=np.zeros((0, 3), dtype=np.float32)))
                    else:
                        rr.log(f"{frame.name}/high_intensity", rr.Points3D(positions=high_intensity_points, radii=0.005))
                else:
                    if cluster_high_intensity and centroids is not None:
                        print(f"{frame.name}: {points.shape=}, high_intensity={high_intensity_points.shape[0]}, clusters={centroids.shape[0]}, {frame.timestamp=}")
                    else:
                        print(f"{frame.name}: {points.shape=}, high_intensity={high_intensity_points.shape[0]}, {frame.timestamp=}")
            else:
                if not use_rerun:
                    pass # avoid spamming if printing text

    try:

        if use_left and use_right:
            stream = stream_lidar_left_right()
            for left, right in stream:
                _handle_frame(left)
                _handle_frame(right)
                
        elif use_left:
            for frame in stream_lidar_left():
                _handle_frame(frame)
    
        elif use_right:
            for frame in stream_lidar_right():
                _handle_frame(frame)

        else:
            raise ValueError("Must specify at least one of `use_left` or `use_right`")
        

    except KeyboardInterrupt:
        print("\nStopping LiDARs...")

def main():
    parser = argparse.ArgumentParser(
        description="Test connection to Hesai LiDAR. By default, point clouds are transformed to the base_footprint frame (using the calibrated URDF if possible) unless --no-transform is specified."
    )
    parser.add_argument("--left", action="store_true", help="Connect to the left LiDAR only")
    parser.add_argument("--right", action="store_true", help="Connect to the right LiDAR only")
    parser.add_argument("--print", action="store_false", help="Plot points to rerun")
    parser.add_argument("--no-transform", action="store_true", help="Skip transforming point clouds to the base_footprint frame")
    parser.add_argument("--cluster_high_intensity", action="store_true", help="Cluster high intensity points (>240) and label centroids with their distance")
    
    args = parser.parse_args()
    
    use_left = args.left
    use_right = args.right
    
    if not use_left and not use_right:
        use_left = True
        use_right = True
        
    stretch_show_lidar(use_left, use_right, use_rerun=args.print, no_transform=args.no_transform, cluster_high_intensity=args.cluster_high_intensity)

if __name__ == "__main__":
    main()