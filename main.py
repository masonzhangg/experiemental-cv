import smbus2
import time
import cv2
import cvzone
import cvzone.FPS as FPS
import numpy as np
from ultralytics import YOLO

class LidarLite:
    def __init__(self, address=0x62):
        self.address = address
        self.bus = smbus2.SMBus(1)

    def read_distance(self):
        self.bus.write_byte(self.address, 0x00)
        time.sleep(0.1)

        distance = self.bus.read_i2c_block_data(self.address, 0x8f, 2)
        distance = (distance[0] << 8) + distance[1]
        return distance

    def close(self):
        self.bus.close()

OBSTACLE_PENALTY = -100
TARGET_DIMENSIONS = (640, 480)
model = YOLO('yolov10n.pt')

SMALL_OBJECT_AREA_THRESHOLD = 1000
MEDIUM_OBJECT_AREA_THRESHOLD = 4000
LARGE_OBJECT_AREA_THRESHOLD = 9000

def get_lidar_data(lidar):
    return lidar.read_distance()

def process_lidar_data(lidar_scan):
    left_sector = np.mean(lidar_scan[30:150])
    right_sector = np.mean(lidar_scan[210:330])
    return left_sector, right_sector

def decide_direction(lidar_scan):
    left_avg, right_avg = process_lidar_data(lidar_scan)
    if left_avg > right_avg:
        return 'left'
    else:
        return 'right'

def adjust_behavior(terrain_type):
    if terrain_type == "loose_sand":
        print("Adjusting for loose sand: Lower speed, higher torque")
    elif terrain_type == "small_rock":
        print("Traversing small rocks: Reducing speed, increasing traction")
    elif terrain_type == "bedrock":
        print("Stable on bedrock, moving at normal speed")
    elif terrain_type == "outcrop":
        print("Outcrop detected: rerouting")

def is_obstacle(object_class):
    obstacle_classes = ['small_object', 'medium_object', 'large_object']
    return object_class in obstacle_classes

def can_go_over_object(area):
    return area < SMALL_OBJECT_AREA_THRESHOLD

def YOLOv10_detect_objects(frame):
    results = model(frame)
    detections = []
    for result in results:
        for *box, conf, cls in result.boxes.data:
            x1, y1, x2, y2 = map(int, box)
            detections.append({
                'x': x1,
                'y': y1,
                'width': x2 - x1,
                'height': y2 - y1,
                'area': (x2 - x1) * (y2 - y1),
                'class': int(cls),
            })
    return detections

def calculate_distance(vector):
    return np.linalg.norm(vector)

def calculate_angle(vector):
    return np.arctan2(vector[1], vector[0]) * 180 / np.pi

def OpenVLA_compute_vector(obstacle_position):
    return np.array([obstacle_position[0], obstacle_position[1], 1])

def convert_to_homogeneous(coordinates):
    return np.array([coordinates[0], coordinates[1], 1])

def assign_obstacle_value():
    return OBSTACLE_PENALTY

def capture_frame_from_camera():
    cap = cvzone.Camera(0)
    if not cap.isOpened():
        return None, None
    
    return cap.read(), cap

def detect_obstacles(frame, lidar_scan, fps_counter):
    resized_frame = cv2.resize(frame, TARGET_DIMENSIONS)
    detected_objects = YOLOv10_detect_objects(resized_frame)
    
    total_score = 0
    left_avg, right_avg = process_lidar_data(lidar_scan)

    for obj in detected_objects:
        if is_obstacle(obj['class']):
            x, y, w, h, area = obj['x'], obj['y'], obj['width'], obj['height'], obj['area']
            cvzone.cornerRect(frame, (x, y, w, h), colorR=(0, 0, 255), thickness=2)
            
            if can_go_over_object(area):
                cvzone.putTextRect(frame, "Can go over!", (x, y - 10), scale=0.5, color=(0, 255, 0), thickness=2)
            else:
                cvzone.putTextRect(frame, "Cannot go over!", (x, y - 10), scale=0.5, color=(0, 0, 255), thickness=2)

            if left_avg < 30 and right_avg < 30:
                print("Obstacle too close on both sides, stopping")
                break
            elif left_avg > right_avg:
                print("More space on the left, turning left")
            else:
                print("More space on the right, turning right")

            total_score += assign_obstacle_value()

            # Add distance and angle display
            distance_to_obstacle = calculate_distance([x + w // 2, y + h // 2])
            angle_to_obstacle = calculate_angle([x + w // 2, y + h // 2])

            cv2.putText(frame, f"Dist: {distance_to_obstacle:.2f}", (x, y - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.putText(frame, f"Angle: {angle_to_obstacle:.2f}", (x, y - 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    fps_counter.update(frame, pos=(10, 40))
    return frame, total_score

def main():
    lidar = LidarLite()
    frame, cap = capture_frame_from_camera()
    if frame is None:
        return
    
    fps_counter = FPS()

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        lidar_scan = get_lidar_data(lidar)
        processed_frame, total_score = detect_obstacles(frame, lidar_scan, fps_counter)

        cv2.imshow('Obstacle Detection', processed_frame)

        if total_score < 0:
            print("Obstacle detected, avoid!")
        else:
            print("Safe path ahead!")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    lidar.close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
