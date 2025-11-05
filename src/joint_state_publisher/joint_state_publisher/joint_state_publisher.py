#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_msgs.msg import TFMessage
import tf_transformations
from sensor_msgs.msg import JointState
import math

class ArmAnglesPublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.angles_pub = self.create_publisher(JointState, '/gt_joint_states', 10)

        self.timer = self.create_timer(0.05, self.publish_joint_states)
        self.declare_parameter('tag_names', ["cabin", "boom", "arm", "bucket"])
        tag_names = self.get_parameter('tag_names').get_parameter_value().string_array_value
        self.kinematic_chain = [(tag_names[i], tag_names[i-1]) for i in range(1, len(tag_names))]
        self.ref = tag_names[0]
        self.js_names = tag_names[1:]
    
    def publish_joint_states(self,):
        js_msg = JointState()
        js_msg.name = self.js_names
        js_msg.position = []
        js_msg.header.frame_id = "world"
        for child, parent in self.kinematic_chain:
            yaw = float('nan')
            if self.tf_buffer.can_transform(parent, child, rclpy.time.Time()):
                transform = self.tf_buffer.lookup_transform(child, parent, rclpy.time.Time())
                js_msg.header.stamp = transform.header.stamp
                q = transform.transform.rotation
                quaternion = [q.x, q.y, q.z, q.w]
                _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
                yaw = yaw * 180.0 / math.pi
                print(f"Transform from {parent} to {child}: Yaw = {yaw:.2f} degrees")
                js_msg.position.append(yaw)
            else:
                self.get_logger().warn(f"Could not transform from {parent} to {child} at time {js_msg.header.stamp}")
                return
            # js_msg.position.append(yaw)
        self.angles_pub.publish(js_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = ArmAnglesPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()