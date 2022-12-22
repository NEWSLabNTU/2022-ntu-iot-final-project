import sys
import rclpy
import time
from rclpy.node import Node
import re
from . import qos

# Import the message types in the format:
# from package_name.msg import Message1, Message2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from autoware_control_msgs.msg import Control
from carla_msgs.msg import CarlaEgoVehicleControl

# Define a ROS node as a class
class Agent(Node):
    n_ticks = 0

    def __init__(self):
        super().__init__("iot_agent")
        self.logger = self.get_logger()

        lidar_topic_name = self.wait_for_lidar_topic()

        #  Create a publisher
        self.control_pub = self.create_publisher(
            Control,  # Message type. Try to discover by `ros2 interface`.
            "/carla/vehicle/hero/ackermann_cmd",  # Please change this
            qos.best_effort(),  # QoS
        )

        # Subscribe to Lidar point cloud data
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            lidar_topic_name,
            self.lidar_pcd_callback,
            qos.best_effort(),
        )

        # Subscribe to odometry data
        self.odometry_sub = self.create_subscription(
            Odometry,
            "/carla/vehicle/hero/odometry",
            self.odometry_callback,
            qos.best_effort(),
        )

        # Subscribe to ticks
        self.tick_sub = self.create_subscription(
            Empty,
            "/carla/tick",
            self.tick_callback,
            qos.best_effort(),
        )

        # Create a timer that periodically scans available topics
        timer_period = 0.1  # in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def tick_callback(self, _msg):
        # Please refer to
        # https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_msgs/
        msg = Control()

        time = self.get_clock().now().to_msg()
        msg.stamp = time
        msg.control_time = time

        msg.lateral.stamp = time
        msg.lateral.control_time = time
        msg.lateral.steering_tire_angle = 0.0
        msg.lateral.steering_tire_rotation_rate = 0.0
        msg.lateral.is_defined_steering_tire_rotation_rate = False

        msg.longitudinal.stamp = time
        msg.longitudinal.control_time = time
        msg.longitudinal.velocity = 1.0
        msg.longitudinal.acceleration = 0.0
        msg.longitudinal.jerk = 0.0
        msg.longitudinal.is_defined_acceleration = True
        msg.longitudinal.is_defined_jerk = False

        self.control_pub.publish(msg)
        self.n_ticks += 1

    def lidar_pcd_callback(self, msg):
        # Please refer to
        # https://docs.ros2.org/galactic/api/sensor_msgs/msg/PointCloud2.html
        # self.get_logger().info('PCD fields: "%s"' % msg.fields)
        pass

    def odometry_callback(self, msg):
        # Please refer to https://docs.ros2.org/galactic/api/nav_msgs/msg/Odometry.html
        # self.get_logger().info('Pose: "%s"' % msg.pose)
        pass

    def wait_for_lidar_topic(self):
        while True:

            def find_lidar_topic(tup):
                name, types = tup
                cond1 = (
                    re.match(r"/carla/sensor/id_\d*/semantic_pointcloud", name)
                    is not None
                )
                cond2 = "sensor_msgs/msg/PointCloud2" in types
                return cond1 and cond2

            time.sleep(3)
            topics = self.get_topic_names_and_types()
            try:
                lidar_topic_name, _ = next(filter(find_lidar_topic, topics))
                break
            except StopIteration:
                self.logger.warn(
                    "Unable to find the lidar topic. Retrying...", once=True
                )
                pass

        self.logger.info(f"Found lidar topic {lidar_topic_name}")
        return lidar_topic_name

    def timer_callback(self):
        pass


def main():
    rclpy.init(args=sys.argv)
    agent = Agent()

    try:
        rclpy.spin(agent)  # It must be called. Otherwise the node halts.
    except KeyboardInterrupt:
        pass

    agent.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
