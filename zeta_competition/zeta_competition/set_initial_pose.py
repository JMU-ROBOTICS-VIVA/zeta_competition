#!/usr/bin/env python
"""Node that will publish a single PoseWithCovarianceStamped

"""
import rclpy
import rclpy.node
from rclpy.task import Future


from geometry_msgs.msg import PoseWithCovarianceStamped
from jmu_ros2_util import transformations


class InitialPoseNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('set_initial_pose')
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                              'initialpose', 10)
        self.declare_parameter('x', 0)
        self.declare_parameter('y', 0)
        self.declare_parameter('theta', 0)
        self.declare_parameter('pos_variance', .25)
        self.declare_parameter('angle_variance', 0.07)

        self.timer = self.create_timer(.1, self.timer_callback)
        self.future = Future()

    def create_pose_w_covariance_stamped(self, x, y, theta, pos_variance, angle_variance):
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y

        quaternion = transformations.quaternion_from_euler(0, 0, theta, 'rxyz')
        pose.pose.pose.orientation.x = quaternion[0]
        pose.pose.pose.orientation.y = quaternion[1]
        pose.pose.pose.orientation.z = quaternion[2]
        pose.pose.pose.orientation.w = quaternion[3]

        pose.pose.covariance[0] = pos_variance
        pose.pose.covariance[7] = pos_variance
        pose.pose.covariance[35] = angle_variance
        return pose


    def timer_callback(self):
        if self.count_subscribers('initialpose') > 0:  
            x = self.get_parameter('x').get_parameter_value().double_value
            print(x)
            y = self.get_parameter('y').get_parameter_value().double_value
            theta = self.get_parameter('theta').get_parameter_value().double_value
            pos_var = self.get_parameter('pos_variance').get_parameter_value().double_value
            angle_var = self.get_parameter('angle_variance').get_parameter_value().double_value
            pose = self.create_pose_w_covariance_stamped(x, y, theta, pos_var, angle_var)
            self.pose_pub.publish(pose)
            self.future.set_result(None)
            self.timer.cancel()


def main():
    rclpy.init()
    node = InitialPoseNode()
    rclpy.spin_until_future_complete(node, node.future)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
