import rclpy
from rclpy.node import Node
import copy
from geometry_msgs.msg import (
    TwistWithCovariance,
    PoseWithCovarianceStamped,
    PoseWithCovariance,
    Pose,
    Twist,
    Transform,
    TransformStamped,
    Quaternion,
)
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from autoware_adapi_v1_msgs.msg import LocalizationInitializationState
from rclpy.qos import QoSReliabilityPolicy, QoSProfile, QoSHistoryPolicy
import numpy as np
import json
import math


class GnssLocator(Node):
    def __init__(self):
        super().__init__("gnss_locator")
        self.declare_parameter("angle", rclpy.Parameter.Type.DOUBLE)

        self.gnss_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/sensing/gnss/pose_with_covariance",
            self.gnss_listener_callback,
            1,
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            "/sensing/imu/imu_data",
            self.imu_listener_callback,
            10,
        )
        self.kinematic_publisher = self.create_publisher(
            Odometry,
            "/localization/kinematic_state",
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )
        self.tf_publisher = self.create_publisher(
            TFMessage,
            "/tf",
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )
        self.init_publisher = self.create_publisher(
            LocalizationInitializationState,
            "/localization/initialization_state",
            QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )

        yaw_degrees = self.get_parameter("angle").value
        assert yaw_degrees <= 180.0 and yaw_degrees >= -180.0
        self.quaternion = yaw_degrees_to_quaternion(yaw_degrees)
        self.gnss_msg = None
        self.imu_msg = None
        self.out_odom = None
        self.out_trans = None

        # self.covariance = list(0.0 for _ in range(36))
        # self.orientation = np.array([0,0,-0.999136,0.0415587]).astype(np.float64) TODO
        # self.vector = np.array([0, 0, 0]).astype(np.float64)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def gnss_listener_callback(self, msg):
        self.get_logger().info("gnss")
        self.gnss_msg = msg
        self.update_output()

    def imu_listener_callback(self, msg):
        self.get_logger().info("imu")
        self.imu_msg = msg
        self.update_output()

    def timer_callback(self):
        if self.out_odom is not None:
            self.kinematic_publisher.publish(self.out_odom)

        if self.out_trans is not None:
            self.tf_publisher.publish(self.out_trans)

        # Publish a mocked initialized state
        if self.gnss_msg is not None:
            msg = LocalizationInitializationState()
            msg.stamp = self.get_clock().now().to_msg()
            msg.state = 3
            self.init_publisher.publish(msg)

    def update_output(self):
        if self.gnss_msg is None:
            return

        stamp = self.get_clock().now().to_msg()

        # assign linear and angular vector to twist
        twist = Twist()
        if self.imu_msg is not None:
            twist.linear = self.imu_msg.linear_acceleration
            twist.angular = self.imu_msg.angular_velocity

        # assign all field to odometry msg
        twist_w_cov = TwistWithCovariance()
        twist_w_cov.twist = twist
        twist_w_cov.covariance = list(0.0 for _ in range(36))

        pose = copy.deepcopy(self.gnss_msg.pose.pose)
        pose.orientation = self.quaternion

        pose_w_cov = PoseWithCovariance()
        pose_w_cov.pose = pose
        pose_w_cov.covariance = list(0.0 for _ in range(36))

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose = pose_w_cov
        odom.twist = twist_w_cov

        # assign all field to TransformStamped
        transform = Transform()
        gnss_pos = self.gnss_msg.pose.pose.position
        transform.translation.x = gnss_pos.x
        transform.translation.y = gnss_pos.y
        transform.translation.z = gnss_pos.z
        transform.rotation = self.quaternion

        trans = TransformStamped()
        trans.header.stamp = stamp
        trans.header.frame_id = "map"
        trans.child_frame_id = "base_link"
        trans.transform = transform

        tf = TFMessage()
        tf.transforms = [trans]

        self.out_trans = tf
        self.out_odom = odom


def yaw_degrees_to_quaternion(degrees: float):
    rads = math.radians(degrees)
    result = Quaternion()
    quaternion_z = math.sin(rads / 2)
    quaternion_w = math.cos(rads / 2)
    result.x, result.y, result.z, result.w = (
        0.0,
        0.0,
        quaternion_z,
        quaternion_w,
    )
    return result


def main():
    rclpy.init()
    gnss_locator = GnssLocator()

    rclpy.spin(gnss_locator)
    gnss_locator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
