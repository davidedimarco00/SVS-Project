import carla
import cv2
import numpy as np
import queue
import threading
import json
from ultralytics import YOLO 
from Networking.MQTTclient import MQTTclient
import logging
logging.getLogger("ultralytics").setLevel(logging.ERROR)

class CameraManager:
    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        self.camera = None
        self.frame_queue = queue.Queue()
        self.model = YOLO('YOLO/yolov8s.pt')
        self.stop_display = False
        self.detected_pedestrian = False
        self.high_confidence_pedestrian = False
        self.display_thread = threading.Thread(target=self.display_camera, daemon=True)
        self.mqtt_client = MQTTclient(broker="61b4907635634e6fb0da78cfb5cf349d.s1.eu.hivemq.cloud", port=8883,
            username="ego_vehicle", 
            password="Tesista08+")
        self.mqtt_client.subscribe("vehicle/pedestrian")

    def spawn_camera(self):
        blueprint_library = self.world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('fov', '90')
        camera_transform = carla.Transform(carla.Location(x=0.5, y=0, z=1.5))
        self.camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
        self.camera.listen(self.process_image)

    def process_image(self, image):
        img = np.reshape(np.array(image.raw_data), (image.height, image.width, 4))
        img = img[:, :, :3]
        img = img.astype(np.uint8)
        img = self.detect_pedestrian(img, image.frame)
        self.frame_queue.put(img)

    def detect_pedestrian(self, image, frame_number):
        results = self.model(image)[0]
        self.detected_pedestrian = False
        self.high_confidence_pedestrian = False

        for box in results.boxes.data:
            x1, y1, x2, y2, conf, cls = box.cpu().numpy()
            if int(cls) == 0:  # Classe "person" (pedone)
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                cv2.rectangle(image, (x1, y1), (x2, y2), (255, 255, 255), 2)
                cv2.putText(image, f"Pedestrian {conf:.2f}", (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                self.detected_pedestrian = True
                if conf > 0.60:
                    self.high_confidence_pedestrian = True
                message = self.format_mqtt_message(x1, y1, float(conf), frame_number)
                self.mqtt_client.publish("vehicle/pedestrian", message)
        return image
    
    def format_mqtt_message(self, x, y, confidence, frame_number):
        action = "No action"
        if self.high_confidence_pedestrian:
            action = "Braking"
        elif self.detected_pedestrian:
            action = "Slowing down"

        message = {
            "event": "Pedestrian Detection",
            "action": action,
            "confidence": confidence
        }
        return json.dumps(message)

    def display_camera(self):
        while not self.stop_display:
            if not self.frame_queue.empty():
                img = self.frame_queue.get()
                cv2.imshow("Pedestrian Detection Camera", img)
            if cv2.waitKey(1) == 27:  # ESC per chiudere
                self.stop_display = True
                break
        cv2.destroyAllWindows()

