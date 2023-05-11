import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

COLLISION_DISTANCE_THRESHOLD = 1.0
SPEED_INCREMENT = 0.005
MAXIMUM_SPEED = 0.2
MAX_LASER_READING = 10
COLLISION_SECTOR_START = 2/5
COLLISION_SECTOR_END = 3/5
TURNING_ANGULAR_SPEED = 0.2

class MotionNode(Node):

    def __init__(self):
        super().__init__('MotionNode')
        self.collision_subscriber = self.create_subscription(
            LaserScan,
            '/demo/laser/out',
            self.laser_callback,
            10)
        self.motion_publisher = self.create_publisher(
            Twist,
            '/demo/cmd_vel',
            10)

        # Keep track of current motion to accelerate evenly
        self.current_motion = 0.01
        # Initial velocity
        init_vel_cmd = Twist()
        init_vel_cmd.linear.x = 0.01
        self.motion_publisher.publish(init_vel_cmd)

    def laser_callback(self, laser_msg):
        laser_readings = np.array(laser_msg.ranges)
        laser_readings[laser_readings == float('inf')] = MAX_LASER_READING
        if np.any(laser_readings[int(len(laser_readings)* COLLISION_SECTOR_START):int(len(laser_readings)* COLLISION_SECTOR_END)] < COLLISION_DISTANCE_THRESHOLD):
            right_space_sum = np.sum(laser_readings[:int(len(laser_readings)/2)])
            left_space_sum = np.sum(laser_readings[int(len(laser_readings)/2):])
            turn_vel_cmd = Twist()
            turn_vel_cmd.linear.x = self.current_motion
            turn_vel_cmd.angular.z = TURNING_ANGULAR_SPEED if left_space_sum > right_space_sum else -TURNING_ANGULAR_SPEED
            self.motion_publisher.publish(turn_vel_cmd)
        else:
            lin_vel_cmd = Twist()
            lin_vel_cmd.linear.x = self.current_motion if self.current_motion >= MAXIMUM_SPEED else self.current_motion + SPEED_INCREMENT
            lin_vel_cmd.angular.z = 0.0
            self.motion_publisher.publish(lin_vel_cmd)


def main(args=None):

    rclpy.init(args=args)
    motion_node = MotionNode()
    rclpy.spin(motion_node)
    motion_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()