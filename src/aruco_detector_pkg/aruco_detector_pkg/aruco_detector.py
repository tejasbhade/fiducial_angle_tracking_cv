#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import  TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf_transformations
from dt_apriltags import Detector
from tf2_ros import TransformBroadcaster


class RelativeTranslationPose(Node):
    def __init__(self):
        super().__init__("Relative_Translation")

        # Parameter to toggle detection display
        self.declare_parameter('display_detections', True)
        self.declare_parameter('tag_ids', [9, 12, 20, 10])
        self.declare_parameter('tag_names', ["cabin", "boom", "arm", "bucket"])
        self.display_detections = self.get_parameter('display_detections').get_parameter_value().bool_value
        self.tag_ids = self.get_parameter('tag_ids').get_parameter_value().integer_array_value
        tag_names = self.get_parameter('tag_names').get_parameter_value().string_array_value
        self.tags = dict(zip(self.tag_ids, tag_names))
        self.bridge = CvBridge()

        # dt_apriltags detector 
        self.detector = Detector(families='tag36h11', quad_decimate=1, quad_sigma=0.8,refine_edges=1, debug=0)

        # Camera intrinsics
        self.fx, self.fy = 1449.776780, 1455.984946
        self.cx, self.cy = 970.356269, 596.122599
        self.camera_params = (self.fx, self.fy, self.cx, self.cy)
        self.camera_matrix = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]])
        self.dist_coeffs = np.zeros(4)        

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.image_sub = self.create_subscription(Image, '/flir_camera/image_raw', self.image_callback, 10)
        if self.display_detections:
            self.annotated_image_pub = self.create_publisher(Image, '/apriltag_detector/annotated_image', 10)

    def publish_transform(self, translation, quaternion, parent_frame, child_frame, stamp):
        """Publish a transform between two frames"""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        
        t.transform.translation.x = float(translation[0])
        t.transform.translation.y = float(translation[1])
        t.transform.translation.z = float(translation[2])
        
        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])
        
        self.tf_broadcaster.sendTransform(t)

    def image_callback(self, msg):
        # img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        img = cv2.cvtColor(raw, cv2.COLOR_BayerRG2BGR)
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(gray, estimate_tag_pose=True, camera_params=self.camera_params, tag_size=0.3)

        poses = {}
        centers = {}
        stamp = msg.header.stamp

        self.get_logger().info(f"Detected {len(detections)} tags")
        self.get_logger().info(f"Tag IDs: {[det.tag_id for det in detections]}")

        for det in detections:
            transformation_matrix = np.eye(4)
            transformation_matrix[:3, :3] = det.pose_R
            transformation_matrix[:3, 3] = det.pose_t.flatten()
            quaternion = tf_transformations.quaternion_from_matrix(transformation_matrix)
            
            poses[det.tag_id] = (quaternion, det.pose_t)
            centers[det.tag_id] = tuple(map(int, det.center))

            assert(det.tag_id in self.tags), f"Detected unknown tag ID: {det.tag_id}"

            tag_frame_name = self.tags[det.tag_id]
            self.publish_transform(
                det.pose_t.flatten(), 
                quaternion, 
                "camera_link",
                tag_frame_name,
                stamp
            )

        def get_relative_euler_deg(parent_id, child_id):
            if parent_id in poses and child_id in poses:
                q_parent, _ = poses[parent_id]
                q_child, _ = poses[child_id]
                
                q_parent_inv = tf_transformations.quaternion_conjugate(q_parent)
                q_relative = tf_transformations.quaternion_multiply(q_parent_inv, q_child)
                
                euler_rad = tf_transformations.euler_from_quaternion(q_relative)
                return np.rad2deg(euler_rad)
            return None
        
        if self.display_detections:
            rel_chain = [(self.tag_ids[i], self.tag_ids[i-1]) for i in range(1, len( self.tag_ids))]

            relative_angles = {
                ch_id: get_relative_euler_deg(parent_id, ch_id) for ch_id, parent_id in rel_chain 
            }

            annotated_img = img.copy()

            for tag_id, center_pos in centers.items():
                cv2.circle(annotated_img, center_pos, 5, (0, 255, 0), -1)
                cv2.putText(annotated_img, f"ID: {tag_id}", center_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                if tag_id in poses:
                    quaternion, tvec = poses[tag_id]
                    self.draw_axes_from_quaternion(annotated_img, quaternion, tvec, self.camera_matrix, self.dist_coeffs)
                if tag_id in relative_angles:
                    try:
                        roll, pitch, yaw = relative_angles[tag_id]
                        angle_text = f"Yaw: {yaw:.1f} deg"
                        text_pos = (center_pos[0] + 15, center_pos[1] - 15)
                        cv2.putText(annotated_img, angle_text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    except TypeError:
                        self.get_logger().warn(f"Could not compute relative angles for tag ID {tag_id}")
    
            self.annotated_image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8"))

    def draw_axes_from_quaternion(self, image, quaternion, tvec, camera_matrix, dist_coeffs, axis_length=0.15):
        # Convert quaternion to rotation matrix
        rot_mat = tf_transformations.quaternion_matrix(quaternion)[:3, :3]
        rvec, _ = cv2.Rodrigues(rot_mat)

        # Define axis points in 3D
        axis_points_3d = np.float32([
            [0, 0, 0],  # origin
            [axis_length, 0, 0],  
            [0, axis_length, 0],  
            [0, 0, axis_length]  
        ]).reshape(-1, 3)

        
        axis_points_2d, _ = cv2.projectPoints(axis_points_3d, rvec, tvec, camera_matrix, dist_coeffs)
        axis_points_2d = np.int32(axis_points_2d).reshape(-1, 2)
        origin = tuple(axis_points_2d[0])

        
        cv2.arrowedLine(image, origin, tuple(axis_points_2d[1]), (0, 0, 255), 3, tipLength=0.2)  # X - Red
        cv2.arrowedLine(image, origin, tuple(axis_points_2d[2]), (0, 255, 0), 3, tipLength=0.2)  # Y - Green
        cv2.arrowedLine(image, origin, tuple(axis_points_2d[3]), (255, 0, 0), 3, tipLength=0.2)  # Z - Blue

      
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(image, 'X', tuple(axis_points_2d[1]), font, 0.35, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(image, 'Y', tuple(axis_points_2d[2]), font, 0.35, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(image, 'Z', tuple(axis_points_2d[3]), font, 0.35, (255, 0, 0), 2, cv2.LINE_AA)

        return image

def main(args=None):
    rclpy.init(args=args)
    node = RelativeTranslationPose()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
