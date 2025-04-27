import cv2
import numpy as np
import math
#rclpy provides the canonical Python API for interacting with ROS 2.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

CAMERA_ID = 1

class WebcamProcessor(Node):
    def __init__(self):
        super().__init__('webcam_processor')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'webcam/image', 10)
        self.bbox_pub = self.create_publisher(String, 'webcam/bounding_boxes', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  

        self.webcam_stream = cv2.VideoCapture(CAMERA_ID)
        if not self.webcam_stream.isOpened():
            self.get_logger().error(f"Error: Could not open webcam with CAMERA_ID '{CAMERA_ID}'")
        else:
            self.get_logger().info("Webcam stream initialized.")

    def timer_callback(self):
        ret, frame = self.webcam_stream.read()
        if not ret:
            self.get_logger().error("Error: Could not read frame from webcam.")
            return

        processed_frame, all_boxes, largest_box = self.process_frame_with_bounding_box(frame)

        image_msg = self.bridge.cv2_to_imgmsg(processed_frame, encoding="bgr8")
        self.image_pub.publish(image_msg)

        if largest_box:
            self.bbox_pub.publish(String(data=str(largest_box)))
            self.get_logger().info(f"Largest Box: {largest_box}")

    def process_frame_with_bounding_box(self, frame, nms_threshold=0.4):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur_frame = cv2.GaussianBlur(gray_frame, (1, 1), 0)

        _, maxVal, _, _ = cv2.minMaxLoc(blur_frame)
        threshold_value = maxVal * 0.85  
        _, thresh_frame = cv2.threshold(blur_frame, threshold_value, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresh_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            boxes.append([x, y, x + w, y + h])

        if len(boxes) == 0:
            return frame, [], {}

        boxes = np.array(boxes)

        indices = cv2.dnn.NMSBoxes(
            bboxes=boxes.tolist(),
            scores=[1.0] * len(boxes),  
            score_threshold=0.0,
            nms_threshold=nms_threshold
        )

        all_boxes = []
        largest_area = 0
        largest_box = {}

        for i in indices:
            i = i[0] 
            x1, y1, x2, y2 = boxes[i]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), 1)
            box_dict = {"x1": x1, "y1": y1, "x2": x2, "y2": y2}
            all_boxes.append(box_dict)

            width = x2 - x1
            height = y2 - y1
            area = width * height
            if area > largest_area:
                largest_area = area
                x_center = x1 + width // 2
                y_center = y1 + height // 2
                largest_box = {
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "x_center": x_center,
                    "y_center": y_center
                }

        if largest_box:
            frame_height, frame_width = frame.shape[:2]
            bottom_center_x = frame_width / 2
            bottom_center_y = frame_height
            dx = largest_box["x_center"] - bottom_center_x
            dy = bottom_center_y - largest_box["y_center"]
            angle_radians = math.atan2(dx, dy)
            angle_degrees = angle_radians * 180 / math.pi

            largest_box["dx"] = dx
            largest_box["dy"] = dy
            largest_box["angle_degrees"] = angle_degrees

        return frame, all_boxes, largest_box

    def destroy_node(self):
        if self.webcam_stream.isOpened():
            self.webcam_stream.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
