import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

ERROR_TOLERANCE = 0.2


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
        
    def odom_callback(self, msg):
        roll, pitch, yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w])
        x, y, z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
        x_calc, y_calc, z_calc, yaw_calc = self.calculated_pose["position"]["x"], self.calculated_pose["position"]["y"], self.calculated_pose["position"]["z"], self.calculated_pose["orientation"]["yaw"]

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
        d_x = elapsed_time_in_seconds * msg.linear.x * math.cos(self.calculated_pose["orientation"]["yaw"])
        d_y = elapsed_time_in_seconds * msg.linear.x * math.sin(self.calculated_pose["orientation"]["yaw"])
        d_yaw = elapsed_time_in_seconds * msg.angular.z
        self.calculated_pose["position"]["x"] += d_x
        print(f"Adjusted x with {d_x}")
        self.calculated_pose["position"]["y"] += d_y
        self.calculated_pose["orientation"]["yaw"] += d_yaw
        self.previous_time = time_now


def main(args=None):

    rclpy.init(args=args)
    localization_node = LocalizationNode()
    rclpy.spin(localization_node)
    localization_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()