import carla
import math
import time
import pygame
from Networking.MQTTclient import MQTTclient
import json

class RadarManager:
    def __init__(self, vehicle, world):
        self.vehicle = vehicle
        self.world = world
        self.radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
        self.radar_lateral_bp = self.world.get_blueprint_library().find('sensor.other.radar')
        self.radar_left = None
        self.radar_right = None
        self.radar_lateral = None
        self.mqtt_client = MQTTclient(broker="61b4907635634e6fb0da78cfb5cf349d.s1.eu.hivemq.cloud", port=8883,
            username="ego_vehicle", 
            password="Tesista08+")
        self.mqtt_client.subscribe("vehicle/bsd")
        self.beep_sound = pygame.mixer.Sound("assets/bsd_sound.wav")
        self.avoiding_collision = False

    def spawn_radars(self):
        self.radar_bp.set_attribute('horizontal_fov', '35')  
        self.radar_bp.set_attribute('vertical_fov', '10')  
        self.radar_bp.set_attribute('range', '7')
        self.radar_lateral_bp.set_attribute('horizontal_fov', '120')  
        self.radar_lateral_bp.set_attribute('vertical_fov', '20')  
        self.radar_lateral_bp.set_attribute('range', '3')
        #rear-left
        radar_left_transform = carla.Transform(
            carla.Location(x=-2.0, y=-0.8, z=0.7),
            carla.Rotation(yaw=200)
        )
        #rear-right
        radar_right_transform = carla.Transform(
            carla.Location(x=-2.0, y= 0.8, z=0.7),
            carla.Rotation(yaw=160)
        )
        #lateral right
        radar_lateral_transform = carla.Transform(
            carla.Location(x=0.0, y= 0.8, z=0.7),
            carla.Rotation(yaw=90)
        )
        #associate callback
        self.radar_left = self.world.spawn_actor(
            self.radar_bp, radar_left_transform, attach_to=self.vehicle
        )
        self.radar_left.listen(self.radar_callback_left)
        self.radar_right = self.world.spawn_actor(
            self.radar_bp, radar_right_transform, attach_to=self.vehicle
        )
        self.radar_right.listen(self.radar_callback_right)
        self.radar_lateral = self.world.spawn_actor(
            self.radar_lateral_bp, radar_lateral_transform, attach_to=self.vehicle
        )
        self.radar_lateral.listen(self.radar_callback_lateral)
        print("Radars spawn correctly")

    def get_avoiding_collision(self):
        return self.avoiding_collision
    
    def format_mqtt_message(self, side, x, y, velocity):
        message = {
            "event": "Blind Spot Detection",
            "side": side,
            "coordinates": {"x": round(x, 2), "y": round(y, 2)},
            "velocity": round(velocity, 2),
            "timestamp": time.time()
        }
        return json.dumps(message)

    def destroy_radars(self):
        if self.radar_left is not None:
            self.radar_left.stop()
            self.radar_left.destroy()
            self.radar_left = None
        if self.radar_right is not None:
            self.radar_right.stop()
            self.radar_right.destroy()
            self.radar_right = None
        print("Radar distrutti.")

    def radar_callback_left(self, radar_data):
        sensor_transform = self.radar_left.get_transform()
        sensor_location = sensor_transform.location
        forward_vec = sensor_transform.get_forward_vector()

        # Debug: disegna una freccia rossa che indica la direzione del radar
        self.world.debug.draw_line(
            sensor_location,
            sensor_location + 5 * forward_vec,
            thickness=0.05,
            color=carla.Color(255, 0, 0),
            life_time=0.1
        )

        for detection in radar_data:
            distance = detection.depth
            azimuth = detection.azimuth
            altitude = detection.altitude
            object_velocity = abs(detection.velocity)
            x = distance * math.cos(altitude) * math.cos(azimuth)
            y = distance * math.cos(altitude) * math.sin(azimuth)
            z = distance * math.sin(altitude)

            #debug detection
            detection_world = sensor_transform.transform(
                carla.Location(x=x, y=y, z=z)
            )
            self.world.debug.draw_point(
                detection_world,
                size=0.15,
                color=carla.Color(255, 255, 0),
                life_time=0.2
            )


            blind_spot_left = (0 < x < 10) and (-3.0 < y < -0.5)

            if blind_spot_left:
                message = {
                    "event": "Blind Spot Detection",
                    "side": "LEFT",
                    "coordinates": {"x": round(x,2), "y": round(y,2)},
                    "velocity": round(object_velocity,2),
                    "timestamp": time.time()
                }
                #print(message)
                message = self.format_mqtt_message("LEFT", x, y, object_velocity)
                self.mqtt_client.publish("vehicle/bsd", message)
                self.beep_sound.play()


    def radar_callback_right(self, radar_data):
        sensor_transform = self.radar_right.get_transform()
        sensor_location = sensor_transform.location
        forward_vec = sensor_transform.get_forward_vector()
        #debug
        self.world.debug.draw_line(
            sensor_location,
            sensor_location + 5 * forward_vec,
            thickness=0.05,
            color=carla.Color(255, 0, 0),
            life_time=0.1
        )
        for detection in radar_data:
            distance = detection.depth
            azimuth = detection.azimuth
            altitude = detection.altitude
            object_velocity = abs(detection.velocity)
            x = distance * math.cos(altitude) * math.cos(azimuth)
            y = distance * math.cos(altitude) * math.sin(azimuth)
            z = distance * math.sin(altitude)
            detection_world = sensor_transform.transform(
                carla.Location(x=x, y=y, z=z)
            )
            #detection point debug in yellow
            self.world.debug.draw_point(
                detection_world,
                size=0.15,
                color=carla.Color(255, 255, 0),
                life_time=0.2
            )
            blind_spot_right = (0 < x < 10) and (0.5 < y < 3.0)

            if blind_spot_right:
                message = {
                    "event": "Blind Spot Detection",
                    "side": "RIGHT",
                    "coordinates": {"x": round(x,2), "y": round(y,2)},
                    "velocity": round(object_velocity,2),
                    "timestamp": time.time()
                }
                #print(message)
                message = self.format_mqtt_message("RIGHT", x, y, object_velocity)
                self.mqtt_client.publish("vehicle/bsd", message)
                self.beep_sound.play()



    def radar_callback_lateral(self, radar_data):
        sensor_transform = self.radar_lateral.get_transform()

        sensor_location = sensor_transform.location
        forward_vec = sensor_transform.get_forward_vector()
        #red ray
        self.world.debug.draw_line(
            sensor_location,
            sensor_location + 5 * forward_vec,
            thickness=0.05,
            color=carla.Color(255, 0, 0),
            life_time=0.1
        )
        for detection in radar_data:
            distance = detection.depth
            azimuth = detection.azimuth
            altitude = detection.altitude
            object_velocity = abs(detection.velocity)
            x = distance * math.cos(altitude) * math.cos(azimuth)
            y = distance * math.cos(altitude) * math.sin(azimuth)
            z = distance * math.sin(altitude)
            # Disegno un punto nel mondo (giallo) per debug
            detection_world = sensor_transform.transform(
                carla.Location(x=x, y=y, z=z)
            )
            self.world.debug.draw_point(
                detection_world,
                size=0.15,
                color=carla.Color(255, 255, 0),
                life_time=0.2
            )

            lateral_right_spot = (-1 < x < 3.5) and (0 < y < 2)
            try:
                control = self.vehicle.get_control()
            except:
                return
                #print("Unable to retrieve control, vehicle destroyed")
            if lateral_right_spot:
                if control.steer > 0.1: # Sta girando a destra
                    self.avoiding_collision = True
                    control.throttle = 0.1  # Rallenta
                    control.brake = 0.5  # Aggiungi frenata leggera
                    control.steer = -0.1  # Sterza a sinistra gradualmente
                    self.vehicle.apply_control(control)
                    time.sleep(1)
                    #come back to the original trajectory
                    control.steer = 0.0
                    control.brake = 0.0
                    self.vehicle.apply_control(control)
                    self.avoiding_collision = False
                #mqtt message format
                message_json = self.format_mqtt_message("LATERAL_RIGHT", x, y, object_velocity)
                #Publish the information
                self.mqtt_client.publish("vehicle/bsd", message_json)
                #emit beep
                self.beep_sound.play()
                #print(message_json)