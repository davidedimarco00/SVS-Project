import random
import carla
import time
import threading
import math

from Managers.RouteManager import RouteManager

class TestBlindSpot:
    def __init__(self, world, env_manager):
        self.world = world
        self.env_manager = env_manager

    def test_dynamic_blind_spot(self, vehicle):
        follower_right = self.env_manager.spawn_follower_in_blind_spot(vehicle, "right")
        threading.Thread(target=self.move_forward, args=(follower_right, 20.0)).start()
        time.sleep(2)
        threading.Thread(target=self.move_forward, args=(vehicle, 25.0)).start()
        time.sleep(5)
        threading.Thread(target=self.move_backward, args=(vehicle, 25.0)).start()
        time.sleep(10)

    def test_static_blind_spot(self, vehicle):
        #simulate the static detection of object (ego vehicle is static)
        follower_right = self.env_manager.spawn_follower_in_blind_spot(vehicle, "right")
        follower_left = self.env_manager.spawn_follower_in_blind_spot(vehicle, "left")
        time.sleep(3)
        print("Static Blind Spot Test: Veicoli generati e posizionati.")

    def test_overtake_scenario(self, vehicle):
        #simulate a vehicle that overtake from left side
        follower_left = self.env_manager.spawn_follower_in_blind_spot(vehicle, "left", backward_offset=20.0)
        time.sleep(2)
        #start overtake
        threading.Thread(target=self.move_forward, args=(follower_left, 45.0)).start()
        time.sleep(5)
        print("Overtake Scenario Test: Sorpasso completato.")

    def test_traffic_scenario(self, vehicle):
        followers = []
        offsets = [10, 15, 20, 25]  #distances from the ego vehicle
        for offset in offsets:
            followers.append(self.env_manager.spawn_follower_in_blind_spot(vehicle, "left", backward_offset=offset+15))
        for i, follower in enumerate(followers):
            threading.Thread(target=self.delayed_move_forward, args=(follower, 50, i * 5)).start()
        time.sleep(len(followers) * 5 + 10)# aspetto

    def delayed_move_forward(self, vehicle, distance, delay):
        time.sleep(delay)
        self.move_forward(vehicle, distance)

    def test_sudden_braking_in_blind_spot(self, vehicle):
        follower = self.env_manager.spawn_follower_in_blind_spot(vehicle, "right", backward_offset=15.0)
        time.sleep(2)
        threading.Thread(target=self.move_forward, args=(follower, 10.0)).start()
        time.sleep(3)
        follower.set_target_velocity(carla.Vector3D(0, 0, 0))  # Frenata improvvisa
        time.sleep(2)
        print("Sudden Braking Test: follower stopped in blind spot")


    def test_dynamic_ego(self, ego_vehicle):
        carla_map = self.world.get_map()
        ego_start_waypoint = carla_map.get_waypoint(ego_vehicle.get_location())
        #generate waypoint
        ego_waypoints = [ego_start_waypoint]
        for _ in range(400):
            next_wp = ego_waypoints[-1].next(5.0)[0]
            ego_waypoints.append(next_wp)

        #generate left side line -3.5 respect to ego vehicle and spawn the follower
        left_waypoints = self.generate_parallel_waypoints(ego_waypoints, offset=-3.5)
        follower_spawn_transform = left_waypoints[0]
        follower_spawn_transform.location.z += 0.01 #safety for collision with terrain
        blueprint_library = self.world.get_blueprint_library()
        follower_bp = blueprint_library.find('vehicle.audi.tt')
        follower_left = self.world.try_spawn_actor(follower_bp, follower_spawn_transform)
        if not follower_left:
            raise RuntimeError("Impossible spawn follower")
        #draw paths
        self.env_manager.draw_path(ego_waypoints, carla.Color(0, 255, 0))  # verde
        self.env_manager.draw_path(left_waypoints, carla.Color(0, 0, 255)) # blu
        target_speed = 10.0 #36kmh
        ego_thread = threading.Thread(
            target=self.follow_waypoints,
            args=(ego_vehicle, ego_waypoints, target_speed),
            daemon=True
        )
        follower_left_thread = threading.Thread(
            target=self.follow_leader,
            args=(follower_left, ego_vehicle, left_waypoints, 5.0, target_speed),
            daemon=True
        )

        ego_thread.start()
        follower_left_thread.start()
        print("[INFO] Test Dynamic Ego")

    def move_forward(self, vehicle, distance):
        velocity = 3.0
        duration = distance / abs(velocity)
        forward_vec = vehicle.get_transform().get_forward_vector()
        vehicle.set_target_velocity(carla.Vector3D(forward_vec.x * velocity, forward_vec.y * velocity, 0))
        time.sleep(duration)
        vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))

    def move_backward(self, vehicle, distance):
        velocity = -3.0
        duration = distance / abs(velocity)
        forward_vec = vehicle.get_transform().get_forward_vector()
        vehicle.set_target_velocity(carla.Vector3D(forward_vec.x * velocity, forward_vec.y * velocity, 0))
        time.sleep(duration)
        vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))

    def generate_parallel_waypoints(self, waypoints, offset):
        parallel_waypoints = []
        for wp in waypoints:
            location = wp.transform.location
            right_vector = wp.transform.get_right_vector()

            parallel_location = carla.Location(
                location.x + offset * right_vector.x,
                location.y + offset * right_vector.y,
                location.z
            )
            parallel_transform = carla.Transform(parallel_location, wp.transform.rotation)
            parallel_waypoints.append(parallel_transform)
        return parallel_waypoints


    def follow_waypoints(self, vehicle, waypoints, target_speed=20.0):
        for wp in waypoints:
            if isinstance(wp, carla.Waypoint):
                location = wp.transform.location
            else:
                location = wp.location
            while True:
                vehicle_location = vehicle.get_location()
                distance = vehicle_location.distance(location)

                control = carla.VehicleControl()
                vector = location - vehicle_location
                yaw = math.atan2(vector.y, vector.x)
                current_yaw = math.radians(vehicle.get_transform().rotation.yaw)

                #angle difference
                angle_diff = (yaw - current_yaw + math.pi) % (2 * math.pi) - math.pi
                steer = max(-1.0, min(1.0, angle_diff))

                #actual velocity
                velocity = vehicle.get_velocity()
                current_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

                #throttle and braking
                if current_speed < target_speed:
                    control.throttle = 0.5
                    control.brake = 0.0
                elif current_speed > target_speed:
                    control.throttle = 0.0
                    control.brake = 0.5
                else:
                    control.throttle = 0.2

                control.steer = steer
                vehicle.apply_control(control)

                if distance < 1.0:
                    break
                time.sleep(0.1)

    def follow_leader(self, follower, leader, waypoints, target_distance=5.0, max_speed=20.0):
        for wp in waypoints:
            target_loc = wp.location if isinstance(wp, carla.Transform) else wp.transform.location
            while follower.get_location().distance(target_loc) >= 1.0:
                floc = follower.get_location()
                lloc = leader.get_location()
                fvel = follower.get_velocity()
                lvel = leader.get_velocity()
                #distance and compute speed
                dist_to_leader = floc.distance(lloc)
                leader_speed = math.sqrt(lvel.x**2 + lvel.y**2 + lvel.z**2)
                follower_speed = math.sqrt(fvel.x**2 + fvel.y**2 + fvel.z**2)

                speed_ref = min(max_speed, leader_speed)
                control = carla.VehicleControl()
                distance_error = dist_to_leader - target_distance

                #logic of control
                if distance_error > 2.0:
                    control.throttle = min(1.0, 0.5 + 0.1 * distance_error)
                elif distance_error < -2.0:
                    control.brake = min(1.0, 0.5 - 0.1 * distance_error)
                else:
                    if follower_speed < speed_ref:
                        control.throttle = 0.3
                        control.brake = 0.0
                    else:
                        control.throttle = 0.0
                        control.brake = 0.3

                direction = target_loc - floc
                target_yaw = math.atan2(direction.y, direction.x)
                current_yaw = math.radians(follower.get_transform().rotation.yaw)
                angle_diff = (target_yaw - current_yaw + math.pi) % (2 * math.pi) - math.pi
                control.steer = max(-1.0, min(1.0, angle_diff))

                follower.apply_control(control)
                time.sleep(0.05)

