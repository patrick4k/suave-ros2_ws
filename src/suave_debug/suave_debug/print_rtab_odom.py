import sys
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from datetime import datetime
from math import (atan2, asin, sin, cos, pi)
import mavsdk.mocap as mocap
from mavsdk import System
import asyncio

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
    init_heading = None
    mavloop = asyncio.new_event_loop()

    def async_do(self, future):
        self.mavloop.run_until_complete(future)

    async def wait_for_init_heading(self):
        async for heading in self.telemetry.heading():
            self.init_heading = heading.heading_deg * pi / 180
            return
        print("-- ERROR: Could not find pos in telemetry")

    def __init__(self, address=None):
        super().__init__()
        print(f"Attempting to connect to drone: {address}")
        self.async_do(self.connect(system_address=f"serial://{address}:57600"))
        print("Connected to drone")
        print("Asking for initial heading")
        self.async_do(self.wait_for_init_heading())
        print(f"Initial heading = {self.init_heading}")


class RTABMapListener(Node):
    drone = Drone("/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10L1BX5-if00-port0")

    def __init__(self):
        super().__init__('rtabmap_listener')
        self.rtabmap_sub = self.create_subscription(
            msg_type=Odometry,
            topic='/odom',
            callback=self.rtabmap_callback,
            qos_profile=10
        )

    def rtabmap_callback(self, msg):
        vispos = odom2vispos(msg, self.drone.init_heading)
        odom = odom2odom(msg, vispos)
        q = msg.pose.pose.orientation
        print(f"sec:{msg.header.stamp.sec}, frame_id: {msg.header.frame_id}, child_frame_id: {msg.child_frame_id}, init_heading: {self.drone.init_heading*180/pi}")
        print(f"POSITION: {vispos.position_body.x_m}, {vispos.position_body.y_m}, {vispos.position_body.z_m}")
        print(f"ANGLE: {vispos.angle_body.roll_rad*180/pi}, {vispos.angle_body.pitch_rad*180/pi}, {vispos.angle_body.yaw_rad*180/pi}")
        print(f"VELOCITY: {odom.speed_body.x_m_s},{odom.speed_body.y_m_s}, {odom.speed_body.z_m_s}")
        print(f"TWIST: {odom.angular_velocity_body.roll_rad_s}, {odom.angular_velocity_body.pitch_rad_s}, {odom.angular_velocity_body.yaw_rad_s}") # in FRD
        print()
   
def main():
    print("[main] print_rtab_odom.py")
    rclpy.init()
    rtabmap_listener = RTABMapListener()
    try:
        rclpy.spin(rtabmap_listener)
    except KeyboardInterrupt:
        print("Closing print_rtab_odom")
        try:
            sys.exit(130)
        except SystemExit:
            os._exit(130)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
