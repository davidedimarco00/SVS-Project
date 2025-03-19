import carla
import time
import pygame

from Managers.EnvironmentManager import EnvironmentManager
from Managers.RadarManager import RadarManager
from Managers.RouteManager import RouteManager
from BlindSpotDetection.TestBlindSpot import TestBlindSpot
from BicycleOvertaking.TestBicycleOvertaking import TestBicycleOvertaking
from PedestrianDetection.TestPedestrianDetection import TestPedestrianDetection


def select_macro_scenario():
    """Function to select the macro test category."""
    print("\n===== Select a Macro Test Category =====")
    print("1. Blind Spot Detection (BSD)")
    print("2. Pedestrian Detection")
    print("3. Bicycle Overtaking")
    print("0. Exit")

    while True:
        choice = input("Enter the number of the test category: ")
        if choice in ["0", "1", "2", "3"]:
            return choice
        print("Invalid choice. Please enter a valid number from the list.")


def select_bsd_scenario():
    """Function to select the specific BSD test type."""
    print("\n===== Select a BSD Test Type =====")
    print("1. Static BSD Test")
    print("2. Dynamic BSD Test")
    print("0. Back to Main Menu")

    while True:
        choice = input("Enter the number of the BSD test type: ")
        if choice in ["0", "1", "2"]:
            return choice
        print("Invalid choice. Please enter a valid number from the list.")


def select_specific_bsd_test(is_dynamic):
    """Function to select the specific BSD test within the chosen category."""
    print("\n===== Select a Specific BSD Test =====")
    tests = {
        "1": "Overtake Scenario",
        "2": "Traffic Scenario",
        "3": "Sudden Braking in Blind Spot",
        "0": "Back to BSD Menu"
    }
    for key, value in tests.items():
        print(f"{key}. {value}")

    while True:
        choice = input("Enter the number of the specific test: ")
        if choice in tests.keys():
            return choice
        print("Invalid choice. Please enter a valid number from the list.")


def select_specific_detection_test():
    """Function to select the specific Pedestrian Detection test within the chosen category."""
    print("\n===== Select a Specific Pedestrian Test =====")
    tests = {
        "1": "Standard Detection Scenario",
        "2": "Advanced Detection Scenario",
        "0": "Back to BSD Menu"
    }
    for key, value in tests.items():
        print(f"{key}. {value}")

    while True:
        choice = input("Enter the number of the specific test: ")
        if choice in tests.keys():
            return choice
        print("Invalid choice. Please enter a valid number from the list.")


def run_bsd_test(world, env_manager, is_dynamic):
    test_blind_spot = TestBlindSpot(world, env_manager)
    spectator = world.get_spectator()
    if not is_dynamic:  # case not dynamic
        ego_vehicle = env_manager.spawn_vehicle(offset_x=8.0)
        time.sleep(1)  # take time for spawning
        spectator = world.get_spectator()
        env_manager.move_spectator_to(ego_vehicle.get_transform(), spectator)
        radar_manager = RadarManager(ego_vehicle, world)
        radar_manager.spawn_radars()
        while True:
            test_choice = select_specific_bsd_test(is_dynamic)
            if test_choice == "0":
                break
            elif test_choice == "1":
                test_blind_spot.test_overtake_scenario(ego_vehicle)
            elif test_choice == "2":
                test_blind_spot.test_traffic_scenario(ego_vehicle)
            elif test_choice == "3":
                test_blind_spot.test_sudden_braking_in_blind_spot(ego_vehicle)
    else:  # case dynamic with follower
        ego_vehicle = env_manager.spawn_vehicle(spawn_index=1)
        env_manager.move_spectator_to(ego_vehicle.get_transform(), spectator)
        # setup radar manager and spawn radars
        radar_manager = RadarManager(ego_vehicle, world)
        radar_manager.spawn_radars()
        # start the dynamic test
        test_blind_spot.test_dynamic_ego(ego_vehicle)


def run_pedestrian_test(world, env_manager, scenario):
    test_pedestrian = TestPedestrianDetection(world, env_manager, scenario)
    test_pedestrian.run_test()
    time.sleep(20)  # time to finish the test


def run_bicycle_test(world, env_manager):
    test_bicycle_test = TestBicycleOvertaking(world, env_manager)
    test_bicycle_test.test_bicycle_overtaking()
    time.sleep(20)  # time to finish the test


def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    env_manager = EnvironmentManager(world)  # instantiate the Environment Manager
    try:
        while True:
            macro_choice = select_macro_scenario()
            if macro_choice == "0":
                print("Exiting...")
                break
            elif macro_choice == "1":
                while True:
                    bsd_choice = select_bsd_scenario()
                    if bsd_choice == "0":
                        break
                    elif bsd_choice == "1":
                        client.load_world('Town03')
                        time.sleep(5)
                        run_bsd_test(world, env_manager, is_dynamic=False)
                    elif bsd_choice == "2":
                        client.load_world('Town10HD')
                        time.sleep(5)
                        run_bsd_test(world, env_manager, is_dynamic=True)
            elif macro_choice == "2":
                while True:
                    pd_choice = select_specific_detection_test()
                    if pd_choice == "0":
                        break
                    elif pd_choice == "1":
                        client.load_world('Town10HD')
                        time.sleep(5)
                        run_pedestrian_test(world, env_manager, 1)
                        break
                    elif pd_choice == "2":
                        client.load_world('Town10HD')
                        time.sleep(5)
                        run_pedestrian_test(world, env_manager, 2)
                        break
            elif macro_choice == "3":
                client.load_world('Town10HD')
                time.sleep(5)
                run_bicycle_test(world, env_manager)

    except KeyboardInterrupt:
        env_manager.cleanup()
        print("Cleaning up scenario...")

    finally:
        env_manager.cleanup()
        print("Environment cleaned up. OK")


if __name__ == '__main__':
    pygame.init()
    pygame.mixer.init()
    main()
