import carla
import math, time, cv2
import numpy as np
import random
import threading
from agents.navigation.basic_agent import BasicAgent
import pygame


# Useful functions
def connect_to_simulator(host='localhost', port=2000):
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    return client

def move_spectator_to(transform, spectator, distance=20.0, x=0, y=0, z=4, yaw=0, pitch=-20, roll=0):
    back_location = transform.location - transform.get_forward_vector() * distance
    back_location.x += x
    back_location.y += y
    back_location.z += z
    transform.rotation.yaw += yaw
    transform.rotation.pitch = pitch
    transform.rotation.roll = roll
    spectator_transform = carla.Transform(back_location, transform.rotation)
    spectator.set_transform(spectator_transform)

def spawn_ego_vehicle(world, spawn_index=0):
    blueprint_library = world.get_blueprint_library()
    ego_vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
    spawn_point = world.get_map().get_spawn_points()[spawn_index]
    ego_vehicle = world.spawn_actor(ego_vehicle_bp, spawn_point)
    return ego_vehicle


def spawn_radar(world, attach_to, position):
    radar_bp = world.get_blueprint_library().find('sensor.other.radar')
    radar_bp.set_attribute('horizontal_fov', '20')
    radar_bp.set_attribute('vertical_fov', '10')
    radar_bp.set_attribute('points_per_second', '10000')
    radar_bp.set_attribute('range', '15')
    
    if position == "right":
        transform = carla.Transform(
            carla.Location(x=-2.0, y=0.9, z=0.7),
            carla.Rotation(pitch=0.0, yaw=120, roll=0.0)
        )
    else:
        transform = carla.Transform(
            carla.Location(x=-2.0, y=-0.8, z=0.7),
            carla.Rotation(pitch=0.0, yaw=220.0, roll=0.0)
        )
    
    radar = world.spawn_actor(radar_bp, transform, attach_to=attach_to)
    return radar

def radar_callback(radar_data, world, sensor, ego_vehicle):
    sensor_transform = sensor.get_transform()
    
    # Otteniamo i vettori della trasformazione
    forward_vec = sensor_transform.get_forward_vector()
    right_vec = sensor_transform.get_right_vector()
    up_vec = sensor_transform.get_up_vector()
    sensor_location = sensor_transform.location
    axis_length = 5
    
    # Disegno degli assi del sensore per debug
    world.debug.draw_line(
        sensor_location,
        sensor_location + axis_length * forward_vec,
        thickness=0.03,
        color=carla.Color(255, 0, 0),
        life_time=0.06
    )
    world.debug.draw_line(
        sensor_location,
        sensor_location + axis_length * right_vec,
        thickness=0.03,
        color=carla.Color(0, 255, 0),
        life_time=0.06
    )
    
    x_min, x_max = -0.0, 6.0   # Da 6m dietro al veicolo fino a poco oltre il sensore
    y_min, y_max = -1.0, 1.5   # Da poco dentro il veicolo fino a 4m lateralmente
    z_min, z_max = -0.5, 2.0   # Da poco sotto il sensore fino a 2m sopra
    
    has_object_in_blind_spot = False

    radar_transform = radar_data.transform  # Posizione e orientamento del radar
    for detection in radar_data:
        # Conversione coordinate polari -> cartesiane
        azimuth = detection.azimuth  # Angolo orizzontale
        altitude = detection.altitude  # Angolo verticale
        distance = detection.depth  # Distanza dell'oggetto rilevato

        # Conversione in coordinate 3D rispetto al sensore
        x = distance * math.cos(altitude) * math.cos(azimuth)
        y = distance * math.cos(altitude) * math.sin(azimuth)
        z = distance * math.sin(altitude)

        # Trasformazione nel sistema di coordinate globale
        detection_point = radar_transform.transform(carla.Location(x=x, y=y, z=z))

        # Ottenere le coordinate rispetto al veicolo
        rel_x = detection_point.x - ego_vehicle.get_transform().location.x
        rel_y = detection_point.y - ego_vehicle.get_transform().location.y
        rel_z = detection_point.z - ego_vehicle.get_transform().location.z

        # **Definizione della zona cieca**
        blind_spot_left = (-6 < rel_x < -2) and (1 < rel_y < 3)  # Lato sinistro
        blind_spot_right = (-6 < rel_x < -2) and (-3 < rel_y < -1)  # Lato destro

        if blind_spot_left or blind_spot_right:
            has_object_in_blind_spot = True
            world.debug.draw_point(
                detection_point,
                size=0.15,  
                color=carla.Color(255, 255, 0),  # Giallo per oggetti nella zona cieca
                life_time=0.2  
            )
            beep_sound.play()
            print("[BLIND SPOT WARNING] Oggetto rilevato nella zona cieca!")

        else:
            has_object_in_blind_spot = False
            print("Nessun oggetto rilevato nella zona cieca.")


def spawn_follower_vehicle(world, ego_vehicle, distance=5.0):
    blueprint_library = world.get_blueprint_library()
    follower_bp = blueprint_library.find('vehicle.audi.tt')
    # Calculate the spawn point for the follower
    ego_transform = ego_vehicle.get_transform()
    follower_location = ego_transform.location - ego_transform.get_forward_vector() * distance
    follower_location.z = ego_transform.location.z+0.1  # Maintain the same height
    follower_transform = carla.Transform(follower_location, ego_transform.rotation)
    # Spawn the follower vehicle
    follower = world.spawn_actor(follower_bp, follower_transform)
    return follower

def draw_on_screen(world, transform, content="O", color=carla.Color(0, 255, 0), life_time=20):
    world.debug.draw_string(transform.location, content, color=color, life_time=life_time)

def draw_path(world, route):
    for waypoint, _ in route:  # Route contains tuples
        world.debug.draw_point(
            waypoint.transform.location,
            size=0.1,
            color=carla.Color(0, 255, 0),
            life_time=120
        )

def spawn_follower_in_blind_spot(world, ego_vehicle):
    blueprint_library = world.get_blueprint_library()
    follower_bp = blueprint_library.find('vehicle.audi.tt')

    ego_transform = ego_vehicle.get_transform()

    # Offset per il blind spot
    backward_offset = 15.0  
    lateral_offset  = 3.0  

    # Calcolo della posizione di spawn
    spawn_location = ego_transform.location
    spawn_location -= ego_transform.get_forward_vector() * backward_offset
    spawn_location += ego_transform.get_right_vector() * lateral_offset
    spawn_location.z = ego_transform.location.z + 1.0  # Mantiene la stessa altezza

    follower_transform = carla.Transform(spawn_location, ego_transform.rotation)

    # Spawna il veicolo
    follower = world.spawn_actor(follower_bp, follower_transform)

    # Avvia il movimento all'indietro in un thread separato
    threading.Thread(target=move_backward, args=(follower, 20.0)).start()
    time.sleep(3)

    return follower

def move_backward(vehicle, distance):
    """
    Muove il veicolo in retromarcia per la distanza specificata.
    """
    velocity = 3.0  # Velocità negativa per andare in retromarcia
    duration = distance / abs(velocity)  # Tempo necessario per percorrere la distanza

    # Imposta la velocità all'indietro
    forward_vec = vehicle.get_transform().get_forward_vector()
    vehicle.set_target_velocity(carla.Vector3D(forward_vec.x * velocity, forward_vec.y * velocity, 0))

    time.sleep(duration)  # Aspetta che il veicolo percorra la distanza

    # Ferma il veicolo
    vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))

def main():
    # Connect to the simulator
    client = connect_to_simulator("localhost", 2000)
    world = client.get_world()
    spectator = world.get_spectator()
    carla_map = world.get_map()

    # Spawn the ego vehicle
    ego_vehicle = spawn_ego_vehicle(world, spawn_index=1)
    print(f"Ego vehicle spawned: {ego_vehicle.type_id}")
    #attach sensors to the ego vehicle
    radar = spawn_radar(world, attach_to=ego_vehicle, position="right")
    radar.listen(lambda radar_data: radar_callback(radar_data, world, radar, ego_vehicle))

    #left
    #radar_left = spawn_radar(world, attach_to=ego_vehicle, position="left")
    #radar_left.listen(lambda radar_data: radar_callback(radar_data, world, radar_left))

    time.sleep(1)
    # Spawn the follower vehicle 10 meters behind the ego vehicle
    #follower = spawn_follower_vehicle(world, ego_vehicle, distance=10.0)
    follower = spawn_follower_in_blind_spot(world, ego_vehicle)
    print(f"Follower vehicle spawned: {follower.type_id}")

    # Create agents for both vehicles
    #ego_agent = BasicAgent(ego_vehicle)
    #follower_agent = BasicAgent(follower)
    #ego_agent.ignore_traffic_lights(active=True)
    #follower_agent.ignore_traffic_lights(active=True)

    # Set a random destination for the ego vehicle
    destination = random.choice(carla_map.get_spawn_points()).location
    #ego_agent.set_destination(destination)

    #print(f"Route calculated to destination: {destination}")

    # Get and draw the planned route
    #route = ego_agent._local_planner.get_plan()
    #draw_path(world, route)

    try:
        while True:
            # Controllo dell'ego vehicle
            #control = ego_agent.run_step()
            #control.throttle = min(control.throttle, 0.5)  # Limita l'accelerazione per stabilità
            #ego_vehicle.apply_control(control)
           
            
          

            # Calcola una posizione leggermente dietro l'ego vehicle per il follower
            #ego_transform = ego_vehicle.get_transform()
            #follower_destination = ego_transform.location + ego_transform.get_right_vector() * 2.0

            # Imposta la destinazione del follower
            #follower_agent.set_destination(follower_destination)
            #follower_control = follower_agent.run_step()
            #follower_control.throttle = min(follower_control.throttle, 1.0)  # Limita l'accelerazione per stabilità
            #follower.apply_control(follower_control)

            # Aggiorna la vista dello spettatore per seguire l'ego vehicle
            #move_spectator_to(ego_vehicle.get_transform(), spectator)

            world.tick()

            # Controlla se l'ego vehicle ha raggiunto la destinazione
            if ego_vehicle.get_location().distance(destination) < 2.0:
                print("Destinazione raggiunta!")
                break
    finally:
        # Destroy the vehicles
        ego_vehicle.destroy()
        follower.destroy()
        radar.destroy()
        print("Vehicles destroyed!")

if __name__ == '__main__':
    #logger = Logger("RoutePlanner")
    #logger.log("Start Simulation....")
    pygame.init()
    pygame.mixer.init()
    global beep_sound
    beep_sound = pygame.mixer.Sound("beep.wav")  # Assicurati di avere un file beep.wav nella directory

    main()
