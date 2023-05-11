import rclpy
import math
import pandas as pd
import os
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion

ERROR_TOLERANCE = 0.2
STOP_ZONES = [
    {
        'x' : [2.95, 3.05],
        'y' : [-1.6, 0.4],
        'enabled' : True
    },
    {
        'x' : [-0.05, 0.05],
        'y' : [-4.55, -1.6],
        'enabled' : True
    },
    {
        'x' : [7.45, 7.55],
        'y' : [-1.6, 0.4],
        'enabled' : True
    }
]
SAVE_INTERVAL_SECONDS = 30.0


class LocalizationNode(Node):

    def __init__(self):
        super().__init__('LocalizationNode')
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/demo/odom',
            self.odom_callback,
            10)
        self.motion_subscriber = self.create_subscription(
            Twist,
            '/demo/cmd_vel',
            self.motion_callback,
            10)
        self.stop_zone_publisher = self.create_publisher(
            Bool,
            '/demo/stop_zone',
            10
        )
        self.calculated_pose = {
            'position' : {
                'x' : 0.0,
                'y' : 0.0,
                'z' : 0.0
            },
            'orientation' : {
                'roll' : 0.0,
                'pitch' : 0.0,
                'yaw' : 0.0
            }
        }
        self.previous_time = self.get_clock().now()
        self.dead_reckoning_measures = []
        self.next_save_time = self.get_clock().now() + rclpy.time.Duration(seconds=SAVE_INTERVAL_SECONDS)
        
    def odom_callback(self, msg):
        roll, pitch, yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w])
        x, y, z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        x_calc, y_calc, z_calc, yaw_calc = self.calculated_pose["position"]["x"], self.calculated_pose["position"]["y"], self.calculated_pose["position"]["z"], self.calculated_pose["orientation"]["yaw"]
        time = self.get_clock().now().nanoseconds/1e9
        self.dead_reckoning_measures.append({
            'time' : time,
            'x_calc' : x_calc, 'x' : x,
            'y_calc' : y_calc, 'y' : y,
            'z_calc' : z_calc, 'z' : z,
            'yaw_calc' : yaw_calc, 'yaw' : yaw
        })
        if self.get_clock().now() >= self.next_save_time:
            self.save_measures()
            self.next_save_time += rclpy.time.Duration(seconds=SAVE_INTERVAL_SECONDS)
        self.check_if_in_stop_zone(x, y)
        if abs(x - x_calc) > ERROR_TOLERANCE:
            print(f"Error x: {x}, {x_calc}")
            self.calculated_pose["position"]["x"] = x
        if abs(y - y_calc) > ERROR_TOLERANCE:
            print(f"Error y: {y}, {y_calc}")
            self.calculated_pose["position"]["y"] = y
        if abs(z - z_calc) > ERROR_TOLERANCE:
            print(f"Error z: {z}, {z_calc}")
            self.calculated_pose["position"]["z"] = z
        if abs(yaw - yaw_calc) > ERROR_TOLERANCE:
            print(f"Error yaw: {yaw}, {yaw_calc}")
            self.calculated_pose["orientation"]["yaw"] = yaw


    def motion_callback(self, msg):
        time_now = self.get_clock().now()
        elapsed_time = time_now - self.previous_time
        elapsed_time_in_seconds = elapsed_time.nanoseconds / 1e9
        yaw_angle = self.calculated_pose["orientation"]["yaw"]
        d_x = elapsed_time_in_seconds * msg.linear.x * math.cos(yaw_angle)
        d_y = elapsed_time_in_seconds * msg.linear.y * math.sin(yaw_angle)
        d_yaw = elapsed_time_in_seconds * msg.angular.z
        self.calculated_pose["position"]["x"] += d_x
        self.calculated_pose["position"]["y"] += d_y
        self.calculated_pose["orientation"]["yaw"] += d_yaw
        self.previous_time = time_now
        self.check_if_in_stop_zone(self.calculated_pose["position"]["x"], self.calculated_pose["position"]["y"])

    def check_if_in_stop_zone(self, x, y):
        for stop_zone in STOP_ZONES:
            if stop_zone['x'][0] < x and stop_zone['x'][1] > x and stop_zone['y'][0] < y and stop_zone['y'][1] > y:
                if stop_zone['enabled']:
                    self.measured_time = 0
                    is_stop_zone = Bool()
                    is_stop_zone.data = True
                    # Avoid stop zones affecting the robot multiple times per passing
                    stop_zone['enabled'] = False
                    self.stop_zone_publisher.publish(is_stop_zone)
                else:
                    is_stop_zone = Bool()
                    is_stop_zone.data = False
                    self.stop_zone_publisher.publish(is_stop_zone)

    def save_measures(self):
        dead_reckoning_df = pd.DataFrame(self.dead_reckoning_measures)
        print(f"Working dir: {os.getcwd()}")
        dead_reckoning_df.to_csv("dead_reckoning_measures.csv")

def main(args=None):

    rclpy.init(args=args)
    localization_node = LocalizationNode()
    rclpy.spin(localization_node)
    localization_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()