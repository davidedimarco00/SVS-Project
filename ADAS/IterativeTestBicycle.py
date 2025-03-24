import time
import carla
import matplotlib.pyplot as plt
import pygame

from Managers.EnvironmentManager import EnvironmentManager
from BicycleOvertaking.TestBicycleOvertaking import TestBicycleOvertaking  # Assicurati che il path dell'import sia corretto
from agents.navigation.basic_agent import BasicAgent

class IterativeTestBicycle:

    def __init__(self, world, env_manager, num_runs=50):
        self.world = world
        self.env_manager = env_manager
        self.num_runs = num_runs

    def run_tests(self):
        successes = 0
        failures = 0
        for i in range(self.num_runs):
            print(f"\n[TEST] START TEST OVERTAKING {i+1}/{self.num_runs}")
            test_bicycle_overtaking = TestBicycleOvertaking(
                world=self.world,
                env_manager=self.env_manager
            )
            test_success = test_bicycle_overtaking.test_bicycle_overtaking()
            if test_success:
                print("[RESULT] No collision -> test success")
                successes += 1
            else:
                print("[RESULT] Collision! -> test failed")
                failures += 1
            test_bicycle_overtaking.env_manager.cleanup()
            time.sleep(1.0)
        success_rate = (successes / self.num_runs) * 100.0
        return success_rate, successes, failures

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    env_manager = EnvironmentManager(world)
    weathers = {
       "CloudyNoon": carla.WeatherParameters.CloudyNoon,
       "MidRainSunset": carla.WeatherParameters.MidRainSunset,
       "HardRainNoon": carla.WeatherParameters.HardRainNoon,
       "ClearSunset": carla.WeatherParameters.ClearSunset,
       "SoftRainNight": carla.WeatherParameters.SoftRainNight,
       "Default": carla.WeatherParameters.Default,
    }
    weather_labels = []
    success_rates = []
    for weather_name, weather in weathers.items():
        world.set_weather(weather)
        print(f"\n[EXECUTING] Test with weather: {weather_name}")
        print("Loading...")
        time.sleep(2)
        tester = IterativeTestBicycle(world, env_manager, num_runs=50)
        success_rate, successes, failures = tester.run_tests()
        print(f"[Final Results] Weather: {weather_name}")
        print(f"  Success Rate: {success_rate:.2f}%\n")
        print(f"  Successes: ", str(successes))
        print(f"  Failures: ", str(failures))
        weather_labels.append(weather_name)
        success_rates.append(success_rate)

    # final graph
    plt.figure()
    x_positions = range(len(weathers))
    plt.bar(x_positions, success_rates)
    plt.xticks(x_positions, weather_labels, rotation=45, ha='right')
    plt.ylabel("Success Rate (%)")
    plt.title("Bicycle Overtaking - Success Rate")
    plt.ylim(0, 100)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    pygame.init()
    pygame.mixer.init()
    main()
