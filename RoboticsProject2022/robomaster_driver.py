from typing import Any
import time

import rclpy
import rclpy.executors
import rclpy.logging

from robomaster_ros.client import RoboMasterROS

class ControllerNode(RoboMasterROS):
    def __init__(self):
        super.__init__('controller_node')
        
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    node = RoboMasterROS(executor=executor)
    ep_chassis = node.ep_robot.chassis
    #ep_chassis.move(x=2, y=0, z=0, xy_speed=0.7).wait_for_completed()
    time.sleep(3)
    if not node.disconnection.done():
        try:
            rclpy.spin_until_future_complete(node, node.disconnection, executor=executor)
        except KeyboardInterrupt:
            pass
    time.sleep(0.1)
    node.stop()
    time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()
