import sys
import os
import rclpy
from rclpy.node import Node
from mavsdk import System
import mavsdk.mocap as mocap
from nav_msgs.msg import Odometry
import asyncio
import time
from datetime import datetime
from math import (atan2, asin, sin, cos, pi)

def odom2vispos(msg: Odometry, gam) -> mocap.VisionPositionEstimate:
    q = msg.pose.pose.orientation
    yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
    pitch = asin(-2.0*(q.x*q.z - q.w*q.y))
    roll = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)
    initial_pitch = -26*pi/180
    xcam = -msg.pose.pose.position.y
    ycam = -msg.pose.pose.position.x
    zcam = -msg.pose.pose.position.z
    xned = xcam*cos(gam) - ycam*sin(gam)
    yned = xcam*sin(gam) + ycam*cos(gam)
    zned = zcam
    rollbody = yaw
    pitchbody = -pitch-initial_pitch
    yawbody = -roll-pi/2+gam
    while yawbody > 2*pi: yawbody -= 2*pi
    while yawbody < 0: yawbody += 2*pi
    return mocap.VisionPositionEstimate(
        time_usec=int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1e3),
        position_body=mocap.PositionBody(xned, yned, zned),
        angle_body=mocap.AngleBody(rollbody, pitchbody, yawbody),
        pose_covariance=mocap.Covariance([float('nan')])
    )

def odom2odom(msg: Odometry, vispos: mocap.VisionPositionEstimate) -> mocap.Odometry:
    roll = vispos.angle_body.roll_rad
    pitch = vispos.angle_body.pitch_rad
    yaw = vispos.angle_body.yaw_rad
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    q = mocap.Quaternion(qw, qx, qy, qz)
    return mocap.Odometry(
        time_usec=int(msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1e3),
        frame_id=mocap.Odometry.MavFrame.MOCAP_NED,
        position_body=vispos.position_body,
        q=q,
        speed_body=mocap.SpeedBody(msg.twist.twist.linear.x, -msg.twist.twist.linear.y, -msg.twist.twist.linear.z),
        angular_velocity_body=mocap.AngularVelocityBody(msg.twist.twist.angular.x, -msg.twist.twist.angular.y, -msg.twist.twist.angular.z),
        pose_covariance=mocap.Covariance([float('nan')]),
        velocity_covariance=mocap.Covariance([float('nan')])
    )


class Drone(System):
    mavloop = asyncio.new_event_loop()
    t = time.time()
    tstr = time.strftime("%Y_%m_%d-%H%M%S")
    fname = "/home/suave/Data/rtabmaps/ODOM_%s.csv" % tstr
    f = open(fname, "w")
    f.write("time_usec,x_m,y_m,z_m,roll_rad,pitch_rad,yaw_rad,xvel_m_s,yvel_m_s,zvel_m_s,roll_rad_s,pitch_rad_s,yaw_rad_s\n")
    init_heading = None

    def __init__(self, address=None):
        super().__init__()
        print(f"Attempting to connect to drone: {address}")
        self.async_do(self.connect(system_address=f"serial://{address}:57600"))
        print("Connected to drone")
        print("Asking for initial heading")
        self.async_do(self.wait_for_init_heading())
        print(f"Initial heading = {self.init_heading*180/pi}")

    def async_do(self, future):
        self.mavloop.run_until_complete(future)

    async def wait_for_init_heading(self):
        heading = float(input(f"Input Heading: "))
        self.init_heading = heading*pi/180
        return
        async for heading in self.telemetry.heading():
            self.init_heading = heading.heading_deg * pi / 180
            return
        print("-- ERROR: Could not find pos in telemetry")

    async def getpos(self):
        async for posvelned in self.telemetry.position_velocity_ned():
            return posvelned.position
        print("-- ERROR: Could not find pos in telemetry")

    def set_vio(self, msg: Odometry) -> mocap.VisionPositionEstimate:
        vispos = odom2vispos(msg, self.init_heading)
        self.async_do(self.mocap.set_vision_position_estimate(vispos))
        return vispos

    def set_odom(self, msg: Odometry, vispos: mocap.Odometry):
        odom = odom2odom(msg, vispos)
        self.async_do(self.mocap.set_odometry(odom))
        return odom


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
    drone = Drone("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10L1BX5-if00-port0")
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
        vispos = None
        odom = None
        try:
            vispos = self.drone.set_vio(msg)
        except Exception as e:
            print("Failed to set VIO")
            print(e)
        try:
            if vispos is None:
                print("Invalid vispos, skipping odom")
                return
            odom = self.drone.set_odom(msg, vispos)
        except Exception as e:
            print("Failed to set ODOM")
            print(e)
        if odom is not None:
            self.drone.f.write(f"{vispos.time_usec},{vispos.position_body.x_m},{vispos.position_body.y_m},{vispos.position_body.z_m},{vispos.angle_body.roll_rad},{vispos.angle_body.pitch_rad},{vispos.angle_body.yaw_rad},{odom.speed_body.x_m_s},{odom.speed_body.y_m_s},{odom.speed_body.z_m_s},{odom.angular_velocity_body.roll_rad_s},{odom.angular_velocity_body.pitch_rad_s},{odom.angular_velocity_body.yaw_rad_s}\n")
            print(f"sec:{msg.header.stamp.sec}, frame_id: {msg.header.frame_id}, child_frame_id: {msg.child_frame_id}, init_heading: {self.drone.init_heading*180/pi}")
            print(f"POSITION: {vispos.position_body.x_m}, {vispos.position_body.y_m}, {vispos.position_body.z_m}")
            print(f"ANGLE: {vispos.angle_body.roll_rad*180/pi}, {vispos.angle_body.pitch_rad*180/pi}, {vispos.angle_body.yaw_rad*180/pi}")
            print(f"VELOCITY: {odom.speed_body.x_m_s}, {odom.speed_body.y_m_s}, {odom.speed_body.z_m_s}")
            print(f"TWIST: {odom.angular_velocity_body.roll_rad_s}, {odom.angular_velocity_body.pitch_rad_s}, {odom.angular_velocity_body.yaw_rad_s}") # in FRD
            print()
        else:
            print("ODOM IS NONE")
            
   
def main():
    print("[main] rtabmaps_listener.py")
    rclpy.init()
    rtabmap_listener = RTABMapListener()
    try:
        rclpy.spin(rtabmap_listener)
    except KeyboardInterrupt:
        print(f"Closing drone file: {rtabmap_listener.drone.fname}")
        rtabmap_listener.drone.f.close()
        try:
            sys.exit(130)
        except SystemExit:
            os._exit(130)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
