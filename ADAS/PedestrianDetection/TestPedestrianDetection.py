import carla
import random
import time
import threading
import math

from Managers.CameraManager import CameraManager

class CollisionSensor:
    """
    Sensor collision, when a collision occurs the collision_detected flag is set to True.
    """
    def __init__(self, ego_vehicle):
        self.world = ego_vehicle.get_world()
        self.blueprint = self.world.get_blueprint_library().find('sensor.other.collision')
        self.collision_sensor = None
        self.ego_vehicle = ego_vehicle
        self.collision_detected = False
        self.collision_sensor = self.world.spawn_actor(
            self.blueprint,
            carla.Transform(),
            attach_to=self.ego_vehicle
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


class TestPedestrianDetection:
    def __init__(self, world, env_manager, scenario):
        self.world = world
        self.env_manager = env_manager
        self.scenario = scenario
        self.vehicle = None
        self.camera_manager = None
        self.collision_sensor = None
        self.spectator = self.world.get_spectator()
        self.stop_display = False
        self.walker = None
        self.left_walkers = []
        self.right_walkers = []

    def run_test(self):
        #vehicle spawn
        self.vehicle = self.env_manager.spawn_vehicle(
            model='vehicle.tesla.model3',
            spawn_index=0,
            offset_x=-20.0
        )
        if not self.vehicle:
            print("[ERRORE] Impossibile spawnare il veicolo.")
            return False
        time.sleep(1.0)
        self.env_manager.move_spectator_to(self.vehicle.get_transform(), self.spectator)

        #spawn camera + display
        self.camera_manager = CameraManager(self.world, self.vehicle)
        self.camera_manager.spawn_camera()
        self.camera_manager.display_thread.start()
        self.collision_sensor = CollisionSensor(self.vehicle)

        #start scenario
        if self.scenario == 1:
            self.run_scenario_1()
        elif self.scenario == 2:
            self.run_scenario_2()
        return not self.collision_sensor.collision_detected

    def run_scenario_1(self):

        print("[INFO] Running Scenario 1 (Tunnel of Pedestrians).")
        #create a tunnel of pedestrians to simulate crowd
        num_walkers_per_side = 6
        distance_between_walkers = 4.0
        lateral_offset = 4.0
        initial_offset = 10.0

        vehicle_transform = self.vehicle.get_transform()
        vehicle_x = vehicle_transform.location.x
        vehicle_y = vehicle_transform.location.y
        vehicle_z = vehicle_transform.location.z

        for i in range(num_walkers_per_side):
            walker_x = vehicle_x + initial_offset + i * distance_between_walkers

            # Left side
            left_location = carla.Location(
                x=walker_x,
                y=vehicle_y - lateral_offset,
                z=vehicle_z + 1.0
            )
            # Right side
            right_location = carla.Location(
                x=walker_x,
                y=vehicle_y + lateral_offset,
                z=vehicle_z + 1.0
            )


            left_walker = self.env_manager.spawn_pedestrian_at_location(
                location=left_location, rotation=None, model=None
            )
            if left_walker:
                self.left_walkers.append(left_walker)

            right_walker = self.env_manager.spawn_pedestrian_at_location(
                location=right_location, rotation=None, model=None
            )
            if right_walker:
                self.right_walkers.append(right_walker)
        print("[INFO] Pedestrians are waiting on the roadside.")

        # 4) Start the vehicle-control thread
        vehicle_control_thread = threading.Thread(
            target=self.vehicle_control_loop_scenario_1, daemon=True
        )
        vehicle_control_thread.start()

        wait_time = 0.5
        start_time = time.time()
        while time.time() - start_time < wait_time:
            self.world.tick()

        #pedestrians runs
        print("[INFO] Gradual random crossing begins.")
        self.launch_walkers_randomly()

        #scenario runs for 10 seconds
        crossing_time = 10.0
        crossing_start = time.time()
        while time.time() - crossing_start < crossing_time:
            self.world.tick()
        print("[INFO] Scenario 1 completed: most pedestrians have crossed.")

    def launch_walkers_randomly(self):
        """Launch the left- and right-side walkers at random times."""
        # Left side => direction +y
        for walker in self.left_walkers:
            t = threading.Thread(
                target=self.launch_single_walker, args=(walker, +1), daemon=True
            )
            t.start()

        # Right side => direction -y
        for walker in self.right_walkers:
            t = threading.Thread(
                target=self.launch_single_walker, args=(walker, -1), daemon=True
            )
            t.start()

    def launch_single_walker(self, walker, direction_sign):
        delay = random.uniform(0, 5)
        time.sleep(delay)
        walker.apply_control(
            carla.WalkerControl(
                speed=1.5,
                direction=carla.Vector3D(0, direction_sign, 0)
            )
        )

    def vehicle_control_loop_scenario_1(self):
        """Check if any pedestrian is in the vehicle path; brake/slow if needed."""
        while not self.stop_display:
            if not (self.camera_manager.detected_pedestrian or
                    self.camera_manager.high_confidence_pedestrian):
                # No pedestrians detected => drive normally
                self.vehicle.apply_control(
                    carla.VehicleControl(throttle=0.7, brake=0.0)
                )
            else:
                # At least one pedestrian is detected
                ped_in_path = False

                # Instead of searching the entire world, we can just iterate
                # over env_manager's pedestrians if you like:
                for ped in self.env_manager.pedestrians:
                    if self.is_pedestrian_in_path(self.vehicle, ped, max_distance=20.0, max_lateral=2.0):
                        ped_in_path = True
                        break

                if ped_in_path:
                    if self.camera_manager.high_confidence_pedestrian:
                        print("[ALERT] High-confidence pedestrian IN PATH -> Full brake!")
                        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                    else:
                        print("[WARNING] Pedestrian in path -> Slowing down.")
                        self.vehicle.apply_control(carla.VehicleControl(throttle=0.2, brake=0.0))
                else:
                    # Pedestrians are seen but not in path
                    self.vehicle.apply_control(
                        carla.VehicleControl(throttle=0.7, brake=0.0)
                    )
            time.sleep(0.1)

    def is_pedestrian_in_path(self, vehicle, pedestrian, max_distance=20.0, max_lateral=2.0):
        """
        Returns True if the pedestrian is in front of the vehicle (local_x > 0),
        within max_distance, and within max_lateral in Y offset.
        """
        v_transform = vehicle.get_transform()
        p_transform = pedestrian.get_transform()

        dx = p_transform.location.x - v_transform.location.x
        dy = p_transform.location.y - v_transform.location.y

        # Vehicle yaw in radians
        yaw = v_transform.rotation.yaw * math.pi / 180.0

        # Transform (dx, dy) into vehicle local coordinates
        local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
        local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)

        if local_x > 0 and local_x < max_distance and abs(local_y) < max_lateral:
            return True
        return False








    def run_scenario_2(self):
        print("[INFO] Running Scenario 2...")
        # 1) Spawn pedone a un certo offset dal veicolo
        vehicle_tf = self.vehicle.get_transform()
        walker_location = carla.Location(
            x=vehicle_tf.location.x + 40,
            y=vehicle_tf.location.y - 15,
            z=1
        )
        self.walker = self.env_manager.spawn_pedestrian_at_location(walker_location)

        # 2) Thread di controllo veicolo
        vehicle_control_thread = threading.Thread(
            target=self.vehicle_control_loop_scenario_2,
            daemon=True
        )
        vehicle_control_thread.start()
        start_t = time.time()
        while time.time() - start_t < 0.5:
            self.world.tick()

        print("[INFO] Pedestrian now starts crossing!")
        if self.walker:
            self.walker.apply_control(
                carla.WalkerControl(
                    speed=1.7,
                    direction=carla.Vector3D(0, 2, 0)
                )
            )

        crossing_dur = 7.0
        start_t = time.time()
        while time.time() - start_t < crossing_dur:
            self.world.tick()

        print("[INFO] Pedestrian crossing completed for Scenario 2.")


    def compute_safety_distance(self, vehicle, deceleration=7.0):
        """Calcola la distanza di arresto considerando solo la velocità orizzontale."""
        velocity = vehicle.get_velocity().x
        velocity_km = velocity * 3.6

        safety_distance = (velocity_km /10)**2
        #print(f"Speed m/s: {velocity:.2f}, Speed km/h: {velocity_km:.2f}, Safety distance: {safety_distance:.2f}")
        return safety_distance

    def vehicle_control_loop_scenario_2(self):
        while not self.stop_display:
            safety_distance = self.compute_safety_distance(self.vehicle)
            if self.camera_manager and self.camera_manager.high_confidence_pedestrian and self.is_pedestrian_in_path(self.vehicle, self.walker, safety_distance+2, max_lateral=6.0):
                print("[ALERT] High-confidence Pedestrian in PATH -> Full brake.")
                self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
            elif self.camera_manager and self.camera_manager.detected_pedestrian and self.is_pedestrian_in_path(self.vehicle, self.walker, safety_distance+2, max_lateral=6.0):
                print("[WARNING] Pedestrian detected IN PATH -> Slowing down.")
                self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.2))
            else:
                self.vehicle.apply_control(carla.VehicleControl(throttle=0.7, brake=0.0))
            time.sleep(0.1)

    def cleanup(self):
        print("[INFO] Manual cleanup requested.")
        self.stop_display = True
        if self.camera_manager:
            self.camera_manager.stop_display = True
            if self.camera_manager.camera is not None:
                self.camera_manager.camera.destroy()
        if self.collision_sensor:
            self.collision_sensor.destroy()
        self.env_manager.cleanup()
        print("[INFO] Cleanup completato.")
