import time
import carla
import matplotlib.pyplot as plt
from Managers.EnvironmentManager import EnvironmentManager
from PedestrianDetection.TestPedestrianDetection import TestPedestrianDetection
from agents.navigation.basic_agent import BasicAgent

class Scenario2Tester:
    """
    Classe che esegue multiple volte lo Scenario 2,
    usando la classe TestPedestrianDetection che crea da sé il CollisionSensor.
    """
    def __init__(self, world, env_manager, num_runs=50):
        self.world = world
        self.env_manager = env_manager
        self.num_runs = num_runs

    def run_tests(self):
        """
        Esegue i test per lo scenario 2 e ritorna la percentuale di successo.
        Restituisce un valore float compreso tra 0 e 100.
        """
        successes = 0

        for i in range(self.num_runs):
            print(f"\n[TEST] Avvio test scenario 2 numero {i+1}/{self.num_runs}")

            # 1) Crea la nuova istanza
            test_pedestrian_detection = TestPedestrianDetection(
                world=self.world,
                env_manager=self.env_manager,
                scenario=2
            )

            # 2) Avvia lo scenario. Restituisce True se NON c'è stata collisione
            test_success = test_pedestrian_detection.run_test()

            # 3) Attendi la durata dello scenario
            scenario_duration = 8.0
            start_time = time.time()
            while time.time() - start_time < scenario_duration:
                self.world.tick()

            if test_success:
                print("[RISULTATO] Nessuna collisione -> Test riuscito.")
                successes += 1
            else:
                print("[RISULTATO] Collisione avvenuta -> Test fallito.")

            # 4) Pulizia risorse
            test_pedestrian_detection.cleanup()

            # Attendi un attimo tra un test e l'altro
            time.sleep(1.0)

        # 5) Calcolo della percentuale di successo
        success_rate = (successes / self.num_runs) * 100.0
        return success_rate

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    env_manager = EnvironmentManager(world)
    weathers = {
       "CloudyNoon": carla.WeatherParameters.CloudyNoon,
        "MidRainSunset": carla.WeatherParameters.MidRainSunset,
        "HardRainNoon" : carla.WeatherParameters.HardRainNoon,
        "ClearSunset" : carla.WeatherParameters.ClearSunset,
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
        tester = Scenario2Tester(world, env_manager, num_runs=2)
        success_rate = tester.run_tests()
        print(f"[RISULTATI FINALI] Weather: {weather_name}")
        print(f"  Success Rate: {success_rate:.2f}%\n")
        weather_labels.append(weather_name)
        success_rates.append(success_rate)

    # ---- GRAFICO FINALE CON MATPLOTLIB ----

    plt.figure()  # Inizializza una nuova figura
    x_positions = range(len(weathers))
    plt.bar(x_positions, success_rates)  # Grafico a barre
    plt.xticks(x_positions, weather_labels, rotation=45, ha='right')
    plt.ylabel("Success Rate (%)")
    plt.title("Pedestrian Detection - Success Rate")
    plt.ylim(0, 100)

    # Mostra il grafico
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
