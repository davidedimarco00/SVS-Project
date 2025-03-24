import carla
import math, time, cv2
import numpy as np
import random
import threading


class EnvironmentManager:
    def __init__(self, world):
        self.world = world
        self.vehicles = []
        self.sensors = []
        self.pedestrians = []
        self.bicycles = []

    def spawn_vehicle(self, model='vehicle.tesla.model3', spawn_index=0, offset_x=None, offset_y=None, offset_z=None):
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.find(model)
        spawn_point = self.world.get_map().get_spawn_points()[spawn_index]
        if (offset_x is not None):
            spawn_point.location.x += offset_x
        if (offset_y is not None):
            spawn_point.location.y += offset_y
        if (offset_z is not None):
            spawn_point.location.z += offset_z
        vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
        if vehicle:
            self.vehicles.append(vehicle)
        return vehicle

    def spawn_pedestrian(self, model='walker.pedestrian.0001'):
        blueprint_library = self.world.get_blueprint_library()
        pedestrian_bp = blueprint_library.find(model)
        controller_bp = blueprint_library.find('controller.ai.walker')
        trans = carla.Transform()
        trans.location = self.world.get_random_location_from_navigation()
        trans.location.z += 1
        pedestrian = self.world.spawn_actor(pedestrian_bp, trans)
        if pedestrian:
            self.pedestrians.append(pedestrian)
            print(f"Spawned pedestrian at {trans.location}")
            self.move_spectator_to(trans, self.world.get_spectator())
            walker_controller = self.world.spawn_actor(controller_bp, carla.Transform(), pedestrian)
            self.world.wait_for_tick()
            walker_controller.start()
            walker_controller.go_to_location(self.world.get_random_location_from_navigation())
            walker_controller.set_max_speed(1.5)
        return pedestrian

    def spawn_pedestrian_at_location(self, location, rotation=None, model=None):
        blueprint_library = self.world.get_blueprint_library()
        if model is None:
            walker_bps = blueprint_library.filter("walker.pedestrian.*")
            pedestrian_bp = random.choice(walker_bps)
        else:
            pedestrian_bp = blueprint_library.find(model)
        if rotation is None:
            rotation = carla.Rotation(0, 0, 0)
        transform = carla.Transform(location, rotation)
        pedestrian = self.world.try_spawn_actor(pedestrian_bp, transform)
        if pedestrian is not None:
            self.pedestrians.append(pedestrian)
        return pedestrian


    def spawn_bicycle_at_location(self, location, rotation=None, model=None):
        blueprint_library = self.world.get_blueprint_library()
        if model is None:
            bicycle_bps = blueprint_library.filter("vehicle.bh.crossbike")
            bicycle_bp = random.choice(bicycle_bps)
        else:
            bicycle_bp = blueprint_library.find(model)

        if rotation is None:
            rotation = carla.Rotation(0, 0, 0)
        transform = carla.Transform(location, rotation)

        bicycle = self.world.try_spawn_actor(bicycle_bp, transform)
        if bicycle is not None:
            self.bicycles.append(bicycle)
        return bicycle

    def spawn_radar(self, attach_to, position):
        radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
        if position == "lateral":
            radar_bp.set_attribute('horizontal_fov', '150')
            radar_bp.set_attribute('vertical_fov', '10')
            radar_bp.set_attribute('range', '3')
        else:
            radar_bp.set_attribute('horizontal_fov', '55')
            radar_bp.set_attribute('vertical_fov', '10')
            radar_bp.set_attribute('points_per_second', '10000')
            radar_bp.set_attribute('range', '7')

        if position == "lateral":
            transform = carla.Transform(
                carla.Location(x=0.0, y=0.9, z=0.7),
                carla.Rotation(pitch=0.0, yaw=90)
            )
        else:
            transform = carla.Transform(
                carla.Location(x=-2.0, y=0.9 if position == "right" else -0.8, z=0.7),
                carla.Rotation(pitch=0.0, yaw=160 if position == "right" else 200, roll=0.0),
            )
        radar = self.world.spawn_actor(radar_bp, transform, attach_to=attach_to)
        if radar:
            self.sensors.append(radar)
        return radar

    def spawn_camera(self, attach_to, transform=None):
        if transform is None:
            transform = carla.Transform(carla.Location(x=-5.0, y=0.0, z=2.5))

        camera_manager = CameraManager(self.world, attach_to)
        camera_manager.spawn_camera()
        camera_manager.display_thread.start()
        if camera_manager.camera:
            self.sensors.append(camera_manager.camera)
        return camera_manager

    def move_spectator_to(self, transform, spectator, distance=18.0, x=0, y=0, z=4, yaw=0, pitch=-30, roll=0):
        back_location = transform.location - transform.get_forward_vector() * distance
        back_location.x += x
        back_location.y += y
        back_location.z += z
        transform.rotation.yaw += yaw
        transform.rotation.pitch = pitch
        transform.rotation.roll = roll
        spectator_transform = carla.Transform(back_location, transform.rotation)
        spectator.set_transform(spectator_transform)

    def spawn_follower_in_blind_spot(self, ego_vehicle, side='right', backward_offset=15.0, lateral_offset=2.0):
        blueprint_library = self.world.get_blueprint_library()
        follower_bp = blueprint_library.find('vehicle.audi.tt')
        ego_transform = ego_vehicle.get_transform()
        spawn_location = ego_transform.location
        spawn_location -= ego_transform.get_forward_vector() * backward_offset

        if side == 'right':
            spawn_location += ego_transform.get_right_vector() * lateral_offset
        else:
            spawn_location -= ego_transform.get_right_vector() * lateral_offset

        spawn_location.z = ego_transform.location.z + 1.0
        follower_transform = carla.Transform(spawn_location, ego_transform.rotation)
        follower = self.world.spawn_actor(follower_bp, follower_transform)
        if follower:
            self.vehicles.append(follower)
        time.sleep(1)
        return follower

    def draw_path(self, waypoints, color):
        for i in range(len(waypoints) - 1):
            start = None
            end = None
            if isinstance(waypoints[i], carla.Waypoint):
                start = waypoints[i].transform.location
                end   = waypoints[i+1].transform.location
            elif isinstance(waypoints[i], carla.Transform):
                start = waypoints[i].location
                end   = waypoints[i+1].location
            else:
                raise TypeError("Waypoints devono essere carla.Waypoint o carla.Transform")

            self.world.debug.draw_point(
                location=start,
                size=0.1,
                color=color,
                life_time=60.0,
                persistent_lines=True
            )

    def cleanup(self):
        for sensor in self.sensors:
            sensor.destroy()
        for vehicle in self.vehicles:
            vehicle.destroy()
        for pedestrian in self.pedestrians:
            pedestrian.destroy()
        for bicycle in self.bicycles:
            bicycle.destroy()
        self.sensors.clear()
        self.vehicles.clear()
        self.pedestrians.clear()
        self.bicycles.clear()
        print("Environment cleaned up.")
