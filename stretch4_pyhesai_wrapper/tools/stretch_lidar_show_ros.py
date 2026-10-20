import os
import signal
import subprocess
import time


def lidar_topics_publishing(topics, timeout_s: float = 5.0) -> bool:
    """Return True if every topic in `topics` receives at least one message within `timeout_s`."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = rclpy.create_node("stretch_lidar_show_topic_check")
    received = {topic: False for topic in topics}

    def _make_callback(topic):
        def _callback(_msg):
            received[topic] = True
        return _callback

    for topic in topics:
        node.create_subscription(PointCloud2, topic, _make_callback(topic), qos_profile_sensor_data)

    deadline = time.time() + timeout_s
    while time.time() < deadline and not all(received.values()):
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    for topic, ok in received.items():
        print(f"{topic}: {'publishing' if ok else 'not publishing'}")
    return all(received.values())


def stretch_show_lidar_ros(use_left: bool, use_right: bool):
    try:
        from ament_index_python.packages import get_package_share_directory
        from stretch_core.vision.vision_topics import VisionTopics
    except ImportError as e:
        print(f"Could not import ROS 2 packages ({e}). Did you source ~/ament_ws/install/setup.bash?")
        return

    topics = []
    if use_left:
        topics.append(VisionTopics.lidar_points("left"))
    if use_right:
        topics.append(VisionTopics.lidar_points("right"))

    if lidar_topics_publishing(topics):
        rviz_config_path = os.path.join(get_package_share_directory("stretch_core"), "rviz", "lidars.rviz")
        commands = [["rviz2", "-d", rviz_config_path]]
        print("Launching RViz...")
    else:
        commands = [
            ["ros2", "launch", "stretch_core", "stretch_driver.launch.py"],
            ["ros2", "launch", "stretch_core", "dual_hesai.launch.py", "use_rviz:=true"],
        ]
        print("LiDAR topics are not publishing. Launching the stretch driver and dual Hesai LiDAR driver.")
        print("Please wait a minute while they launch; RViz will open when ready. Press Ctrl+C to quit.")

    # Each launch gets its own process group so the nodes it spawns are stopped along with it
    processes = [subprocess.Popen(cmd, start_new_session=True) for cmd in commands]

    def _stop_processes(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _stop_processes)
    signal.signal(signal.SIGHUP, _stop_processes)

    try:
        for p in processes:
            p.wait()
    except KeyboardInterrupt:
        print("\nStopping ROS 2 processes...")
    finally:
        for p in processes:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in processes:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
