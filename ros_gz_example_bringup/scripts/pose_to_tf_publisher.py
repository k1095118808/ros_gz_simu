#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
# from tf_transformations import euler_from_quaternion # if needed for debugging

class PoseToTfNode(Node):
    def __init__(self):
        super().__init__('pose_to_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.world_frame_ = self.declare_parameter('world_frame', 'world').get_parameter_value().string_value
        self.child_frame_ = self.declare_parameter('child_frame', 'explorer_ds1/base_link').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/explorer_ds1/pose', # Default topic name, ensure it matches remapping in launch
            self.pose_callback,
            10)
        self.get_logger().info(f"Publishing TF from '{self.world_frame_}' to '{self.child_frame_}'")

    def pose_callback(self, msg: PoseStamped):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp # Use sim time from PoseStamped header
        t.header.frame_id = self.world_frame_
        t.child_frame_id = self.child_frame_

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PoseToTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()