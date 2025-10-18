import cv2
import numpy as np
import math

CAMERA_ID = 1

def get_webcam_stream():
    stream = cv2.VideoCapture(CAMERA_ID)
    if not stream.isOpened():
        print(f"Error: Could not open webcam. Check if CAMERA_ID '{CAMERA_ID}' is correct.")
        return None
    return stream

def process_frame_with_bounding_box(frame, nms_threshold=0.4):

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)

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
        x1, y1, x2, y2 = boxes[i]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), 1)

        box_dict = {
            "x1": x1,
            "y1": y1,
            "x2": x2,
            "y2": y2
        }
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

        dx = largest_box["x_center"] - bottom_center_x

        CAMERA_HORIZONTAL_FOV_DEGREES = 57

        degrees_per_pixel = CAMERA_HORIZONTAL_FOV_DEGREES / frame_width

        # Final real-world angle you should rotate
        angle_degrees = dx * degrees_per_pixel

        largest_box["dx"] = dx
        largest_box["angle_degrees"] = angle_degrees

    return frame, all_boxes, largest_box

def main():
    cv2.namedWindow('Live Stream with Bounding Boxes', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Live Stream with Bounding Boxes', 800, 600)

    webcam_stream = get_webcam_stream()
    if webcam_stream is None:
        return

    while True:
        ret, frame = webcam_stream.read()
        if not ret:
            print("Error: Could not read frame from webcam.")
            break

        processed_frame, all_boxes, largest_box = process_frame_with_bounding_box(frame)

        if largest_box:
            print("Largest Box:", largest_box)

        cv2.imshow('Live Stream with Bounding Boxes', processed_frame)

        # exit on ESC (27) or Backspace (8)
        key = cv2.waitKey(20)
        if key in [27, 8]:
            break

    webcam_stream.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()