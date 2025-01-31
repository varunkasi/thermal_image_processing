# System python packages.
from collections import deque
import cv2
import multiprocessing as mp
import numpy as np

# ROS2 standard python packages.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

# Custom python packages from other ROS2 packages.
from straps_msgs.msg import (
    BodyPose2DBBox,
    BodyPose2DPose,
    BodyPose2DDetection,
)

# This ROS2 package.
from thermal_image_processing.processing_algorithm import ProcessingAlgorithm

# Main ROS2 node.
class AlgorithmNode(Node):
    def __init__(self):
        super().__init__('algorithm_node')
        
        self.algorithm = ProcessingAlgorithm()
        
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            CompressedImage,
            '/iphone1/regular_view/arframe_image_resize/compressed',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.image_queue = deque()
        self.image_queue_lock = mp.Lock()
        
        self.pub_body_pose_2d = self.create_publisher(
            BodyPose2DDetection, 
            '/orin1/body_pose_2d', 
            10)
        
        self.timer = self.create_timer(0.2, self.image_consumer_callback)

    def image_callback(self, msg):
        # Buffer the opencv image.
        with self.image_queue_lock:
            self.image_queue.append(msg)
        
        self.get_logger().info("Image received. ")

    def awesome_image_processing(self, cv_image):
        return self.algorithm(cv_image)

    def publish_results(self, msg, detections, keypoints):
        # Get the results for body pose detection.
        # Later, we need to figure out how to process the values if multiple people are detected in the frame
        body_pose_2d_bbox = BodyPose2DBBox()
        body_pose_2d_bbox.left = float(detections[0])    # x1
        body_pose_2d_bbox.top = float(detections[1])     # y1
        body_pose_2d_bbox.right = float(detections[2])   # x2
        body_pose_2d_bbox.bottom = float(detections[3])  # y3
        body_pose_2d_bbox.score = float(detections[4])   # confidence score
        
        poses = []
        
        for _ in range(17):
            body_pose_2d_pose = BodyPose2DPose()
            body_pose_2d_pose.x = float(keypoints[0][0][_])  # x
            body_pose_2d_pose.y = float(keypoints[0][1][_])  # y
            body_pose_2d_pose.score = float(keypoints[0][2][_])  # confidence score
            poses.append(body_pose_2d_pose)
            
        body_pose_2d_detection = BodyPose2DDetection()
        body_pose_2d_detection.header.stamp = self.get_clock().now().to_msg()
        body_pose_2d_detection.input_image_timestamp = msg.header.stamp
        body_pose_2d_detection.bbox = body_pose_2d_bbox
        body_pose_2d_detection.poses = poses
        
        self.pub_body_pose_2d.publish(body_pose_2d_detection)

    def consume_image(self, msg):
        # Decode the compressed image.
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        
        # === Do stuff about cv_image. ===
        detections, keypoints = self.awesome_image_processing(cv_image)
        
        # === Publish the results. ===
        self.publish_results(msg, detections, keypoints)

    def image_consumer_callback(self):
        msg = None
        
        with self.image_queue_lock:
            if len(self.image_queue) > 0:
                msg = self.image_queue.popleft()
                self.image_queue.clear()
          
        if msg is not None:
            self.consume_image(msg)
            self.get_logger().info("Image consumed (inference). ")

def main(args=None):
    rclpy.init(args=args)
    
    image_subscriber = AlgorithmNode()
    image_subscriber.get_logger().info("algorithm_node started. ")
    
    # Start spinning.
    rclpy.spin(image_subscriber)
    
    # The user has pressed Ctrl+C or the ROS2 system has requested the node to shut down.
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # This is NOT the entry point of the program.
    # The entry point is the main function, which gets registered in the setup.py file.
    main()
