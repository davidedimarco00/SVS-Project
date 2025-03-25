from Managers.RadarManager import RadarManager
import carla
import time
import random
import numpy as np
from agents.navigation.basic_agent import BasicAgent
class CollisionSensor:
    def __init__(self, vehicle):
        self.world = vehicle.get_world()
        self.blueprint = self.world.get_blueprint_library().find('sensor.other.collision')
        self.collision_sensor = None
        self.vehicle = vehicle
        self.collision_detected = False
        self.collision_sensor = self.world.spawn_actor(
            self.blueprint,
            carla.Transform(),
            attach_to=self.vehicle
        )
        self.collision_sensor.listen(lambda event: self._on_collision(event))
    def _on_collision(self, event):
        self.collision_detected = True
        other_actor = event.other_actor
        print(f"[COLLISION] Collision with: {other_actor.type_id}")
    def destroy(self):
        if self.collision_sensor is not None:
            self.collision_sensor.stop()
            self.collision_sensor.destroy()
            self.collision_sensor = None

class TestBicycleOvertaking:
    def __init__(self, world, env_manager):
        self.world = world
        self.env_manager = env_manager
        self.ego_vehicle = None
        self.collision_sensor = None
        self.bicycle = None
        self.radar = None

    def test_bicycle_overtaking(self):
        print("Starting random destination test...")
        spawn_index = 1
        self.ego_vehicle = self.env_manager.spawn_vehicle(spawn_index=spawn_index)
        if not self.ego_vehicle:
            print("Failed to spawn ego vehicle.")
            return
        print("Ego vehicle spawned at:", self.ego_vehicle.get_location())
        self.collision_sensor = CollisionSensor(self.ego_vehicle) #attach the collision sensor
        spectator = self.world.get_spectator()
        self.env_manager.move_spectator_to(self.ego_vehicle.get_transform(), spectator)

        #set a destination
        random_destination = carla.Location(x=-52.186733, y=42.565128, z=0.000000)
        print("Random destination chosen:", random_destination)
        agent = BasicAgent(self.ego_vehicle, target_speed=20)
        agent.set_destination(random_destination)
        agent.ignore_vehicles(True)
        #spawn the bicycle at right
        ego_transform = self.ego_vehicle.get_transform()
        bicycle_spawn_location = carla.Location(
            ego_transform.location.x - ego_transform.get_forward_vector().x * 5.0 + ego_transform.get_right_vector().x * 4.0,
            ego_transform.location.y - ego_transform.get_forward_vector().y * 5.0 + ego_transform.get_right_vector().y * 1.45 ,
            ego_transform.location.z
        )
        self.bicycle = self.env_manager.spawn_bicycle_at_location(bicycle_spawn_location)
        if not self.bicycle:
            print("Failed to spawn bicycle.")
        else:
            print("Bicycle spawned at:", self.bicycle.get_location())
        #attach the right radar ensure that cover all the right side
        self.radar_right = self.env_manager.spawn_radar(self.ego_vehicle, "lateral")
        radar_manager_lateral = RadarManager(self.ego_vehicle, self.world)
        radar_manager_lateral.spawn_radars()
        time.sleep(2)
        if self.bicycle:
            print("[INFO]Bicycle starts moving...")
            for _ in range(15):
                self.bicycle.apply_control(carla.VehicleControl(throttle=0.6))
                time.sleep(0.1)
        time.sleep(2)
        start_time = time.time()
        while time.time() - start_time < 10:
            if not self.ego_vehicle:
                break
            if self.bicycle:
                self.bicycle.apply_control(carla.VehicleControl(throttle=0.9))
            if not radar_manager_lateral.avoiding_collision:
                control = agent.run_step()
                self.ego_vehicle.apply_control(control)
            time.sleep(0.1)
        print("Test completed.")

        if self.collision_sensor:
            self.collision_sensor.destroy()
        self.env_manager.cleanup()
        return not self.collision_sensor.collision_detected