import sys
import os
import rclpy
from rclpy.node import Node
from mavsdk import System
from mavsdk.mocap import VisionPositionEstimate, PositionBody, AngleBody, Covariance
from nav_msgs.msg import Odometry
import asyncio
import time
from datetime import datetime


def odom2vispos(msg: Odometry) -> VisionPositionEstimate:
    return VisionPositionEstimate(
        time_usec=int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1e3),
        position_body=PositionBody(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
        angle_body=AngleBody(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z),
        pose_covariance=Covariance([float('nan')])
    )


class Drone(System):
    mavloop = asyncio.new_event_loop()
    t = time.time()
    f = open("/home/suave/Data/rtabmaps/POSITION_%s.csv" % t, "w")
    f.write("time_usec,x_m,y_m,z_m,roll_rad,pitch_rad,yaw_rad\n")

    def __init__(self, address=None):
        super().__init__()
        print("Attempting to connect to drone")
        self.async_do(self.connect(system_address=f"serial://{address}:57600"))
        print("Connected to drone")

    def async_do(self, future):
        self.mavloop.run_until_complete(future)

    def set_vio(self, msg: Odometry):
        vispos = odom2vispos(msg)
        self.f.write(f"{vispos.time_usec},{vispos.position_body.x_m},{vispos.position_body.y_m},{vispos.position_body.z_m},{vispos.angle_body.roll_rad},{vispos.angle_body.pitch_rad},{vispos.angle_body.yaw_rad}\n")
        print(f"POS: {vispos.position_body}")
        self.async_do(self.mocap.set_vision_position_estimate(vispos))


"""
# Launch realsense topic and then launch rtabmap
ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true

ros2 param set /camera/camera depth_module.emitter_enabled 0
ros2 launch rtabmap_examples realsense_d435i_stereo.launch.py

# Build listener
ros
colcon build
source ./install/setup.bash
ros2 run suave_rtabmap rtabmap_listener
"""


class RTABMapListener(Node):
    loop = asyncio.new_event_loop()
    drone = Drone("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0019BWS-if00-port0")
    t = time.time()

    def __init__(self):
        super().__init__('rtabmap_listener')
        self.rtabmap_sub = self.create_subscription(
            msg_type=Odometry,
            topic='/odom',
            callback=self.rtabmap_callback,
            qos_profile=10
        )

    def rtabmap_callback(self, msg):
        now = datetime.now()
        timestr = now.strftime("%m-%d %H:%M:%S")
        print(f"[{timestr}]")
        try:
            self.drone.set_vio(msg)
        except:
            print("Failed to set VIO")
   
def main():
    print("[main] rtabmaps_listener.py")
    rclpy.init()
    rtabmap_listener = RTABMapListener()
    try:
        rclpy.spin(rtabmap_listener)
    except KeyboardInterrupt:
        print("Closing drone file")
        rtabmap_listener.drone.f.close()
        try:
            sys.exit(130)
        except SystemExit:
            os._exit(130)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
