import carla
import math, time, cv2
import numpy as np
import random
from agents.navigation.basic_agent import BasicAgent


class RouteManager:
    def __init__(self, world, vehicle):
        self.world = world
        self.vehicle = vehicle
        self.agent = BasicAgent(vehicle)
        self.agent.ignore_traffic_lights(active=True)
        self.destination = None
        self.waypoints = []
    
    def set_destination(self, destination):
        self.destination = destination
        self.agent.set_destination(destination)
    
    def draw_path(self, world, route):
        for waypoint, _ in route:  # tuple (waypoint, info)
            world.debug.draw_point(
                waypoint.transform.location,
                size=0.1,
                color=carla.Color(0, 255, 0),
                life_time=120
            )
    
    def follow_route(self):
        self.waypoints = []
        route = self.agent._local_planner.get_plan()
        self.draw_path(self.world, route)
        last_waypoint = None
        while True:
            control = self.agent.run_step()
            control.throttle = min(control.throttle, 0.5)
            self.vehicle.apply_control(control)
            current_location = self.vehicle.get_location()
            current_waypoint = self.world.get_map().get_waypoint(current_location)
            if last_waypoint is None or current_location.distance(last_waypoint.transform.location) > 2.0:
                self.waypoints.append(current_waypoint)
                last_waypoint = current_waypoint
            if current_location.distance(self.destination) < 2.0:
                brake_control = carla.VehicleControl(throttle=0.0, brake=1.0)
                self.vehicle.apply_control(brake_control)
                print("Destination reached!")
                break

            self.world.tick()