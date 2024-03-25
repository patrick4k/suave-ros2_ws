from mavsdk import System
from mavsdk.mocap import VisionPositionEstimate, PositionBody, AngleBody, Covariance
from nav_msgs.msg import Odometry
import asyncio


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

    def async_do(self, future):
        self.mavloop.run_until_complete(future)

    def set_vio(self, msg: Odometry):
        self.async_do(self.mocap.set_vision_position_estimate(odom2vispos(msg)))
