import rclpy
from rclpy.node import Node
from mavsdk import System
from mavsdk.mocap import VisionPositionEstimate, PositionBody, AngleBody, Covariance
from nav_msgs.msg import Odometry
import asyncio
import time


def odom2vispos(msg: Odometry) -> VisionPositionEstimate:
    return VisionPositionEstimate(
        time_usec=int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1e3),
        position_body=PositionBody(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
        angle_body=AngleBody(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z),
        pose_covariance=Covariance(msg.pose.covariance)
    )


class Drone(System):
    mavloop = asyncio.new_event_loop()

    def __init__(self, address=None):
        super().__init__()
        self.async_do(self.connect(address))
        print("Connected to drone")

    def async_do(self, future):
        self.mavloop.run_until_complete(future)

    def set_vio(self, msg: Odometry):
        self.async_do(self.mocap.set_vision_position_estimate(odom2vispos(msg)))


"""
Launch realsense topic and then launch rtabmap
$ ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true
$ ros2 param set /camera/camera depth_module.emitter_enabled 0
$ ros2 launch rtabmap_examples realsense_d435i_stereo.launch.py

Run listener
$ ros2 run suave_rtabmap rtabmap_listener
"""


class RTABMapListener(Node):
    loop = asyncio.new_event_loop()
    drone = Drone()
    t = time.time()
    f = open("/home/patrick/Data/rtabmaps/%s.txt" % t, "w+")

    # writer = rosbag2_py.SequentialWriter()
    # storage_options = rosbag2_py._storage.StorageOptions(
    #     uri='big_synthetic_bag',
    #     storage_id='mcap')
    # converter_options = rosbag2_py._storage.ConverterOptions('', '')
    # writer.open(storage_options, converter_options)
    # topic_info = rosbag2_py._storage.TopicMetadata(
    #     name='synthetic',
    #     type='example_interfaces/msg/Int32',
    #     serialization_format='cdr')
    # writer.create_topic(topic_info)

    def __init__(self):
        super().__init__('rtabmap_listener')
        self.rtabmap_sub = self.create_subscription(
            msg_type=Odometry,
            topic='/odom',
            callback=self.rtabmap_callback,
            qos_profile=10
        )

    def rtabmap_callback(self, msg):
        print(msg)
        self.drone.set_vio(msg)
        self.f.write(msg)

        # self.writer.write(
        #     'synthetic',
        #     serialize_message(msg),
        #     time_stamp.nanoseconds)


def main():
    print("[main] rtabmaps_listener.py")
    rclpy.init()
    rtabmap_listener = RTABMapListener()
    rclpy.spin(rtabmap_listener)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
