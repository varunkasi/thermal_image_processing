from ultralytics import YOLO
import cv2
import os
import re
import numpy as np
from ament_index_python.packages import get_package_share_directory

class ProcessingAlgorithm(object):
    def __init__(self):
        super().__init__()
        # Get the path to the model file using ROS2's resource finding
        package_share_dir = get_package_share_directory('thermal_image_processing')
        model_path = os.path.join(package_share_dir, 'resource', 'models', 'yolo11x-pose.pt')
        self.model = YOLO(model_path)
        
    def __call__(self, image):
        results = self.model.predict(image, save=False)
        # Get detections
        detections = []
        keypoints = []
        for result in results:
            for box in result.boxes:
                if box.cls == 0:  # person class
                    # Get box coordinates and confidence in format [x1, y1, x2, y2, conf]
                    box_data = np.concatenate([box.xyxy.cpu().numpy()[0], box.conf.cpu().numpy()[None]])
                    detections.append(box_data)
                    keypoints.append(result.keypoints.cpu().numpy())
        return detections, keypoints
