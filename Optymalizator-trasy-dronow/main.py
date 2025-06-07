import time
import sys
import os

import matplotlib
matplotlib.use("TkAgg")
# Dodaj src do ścieżki Python
current_dir = os.path.dirname(os.path.abspath(__file__))
src_path = os.path.join(current_dir, 'src')
sys.path.insert(0, src_path)

# Importy z naszych modułów
from geometry.point import Point
from map.map_generator import MapGenerator
from pathfinding.drone_pathfinder import DronePathfinder
from visualization.visualizer import DroneMapVisualizer
from shapely.geometry import Polygon

class DroneOptimizer:
    def __init__(self, map_width=1000, map_height=800):
        self.map_width = map_width
        self.map_height = map_height
        self.map_data = None
        self.pathfinder = None
        self.visualizer = None
        self._animation = None

        print("🚁 OPTYMALIZATOR TRAS DLA DRONÓW 🚁")
        print("=" * 50)
        print("Projekt z wykorzystaniem geometrii obliczeniowej:")
        print("• Triangulacja Delaunay")
        print("• Otoczki wypukłe")
        print("• Drzewa zakresowe 2D")
        print("• Algorytm A*")
        print("=" * 50)

    def generate_map(self, building_count=20, forbidden_zones=8, seed=42):
        """Generuje nową mapę"""
        print(f"\n📍 Generowanie mapy {self.map_width}x{self.map_height}...")

        generator = MapGenerator(self.map_width, self.map_height, seed)
        self.map_data = generator.generate_map(building_count, forbidden_zones)

        print(f"✅ Mapa wygenerowana pomyślnie!")
        print(f"   • Budynki (prostokątne): {len(self.map_data.buildings)}")
        print(f"   • Strefy zakazane: {len(self.map_data.forbidden_zones)}")
        print(f"   • Całkowita powierzchnia: {self.map_width * self.map_height} m²")

        # Sprawdź pokrycie mapy przeszkodami
        total_obstacle_area = 0
        for building in self.map_data.buildings:
            total_obstacle_area += building.area()
        for zone in self.map_data.forbidden_zones:
            total_obstacle_area += zone.area()

        coverage = (total_obstacle_area / (self.map_width * self.map_height)) * 100
        print(f"   • Pokrycie przeszkodami: {coverage:.1f}%")

    def initialize_pathfinder(self, safety_margin=10):
        """Inicjalizuje system wyznaczania tras"""
        if not self.map_data:
            raise ValueError("❌ Najpierw wygeneruj mapę!")

        print(f"\n🔧 Inicjalizacja systemu nawigacji (margines: {safety_margin}m)...")
        start_time = time.time()

        self.pathfinder = DronePathfinder(self.map_data, safety_margin)

        init_time = time.time() - start_time
        print(f"✅ System zainicjalizowany w {init_time:.3f} sekund")
        print(f"   • Węzły nawigacyjne: {len(self.pathfinder.navigation_graph)}")
        print(f"   • Bezpieczne punkty: {len(self.pathfinder.safe_points)}")

    def find_path(self, start_x, start_y, end_x, end_y, optimize=True):
        """Wyznacza trasę dla drona z dokładną walidacją"""
        if not self.pathfinder:
            raise ValueError("❌ Najpierw zainicjalizuj pathfinder!")

        start_point = Point(start_x, start_y)
        end_point = Point(end_x, end_y)

        print(f"\n🎯 Wyznaczanie precyzyjnej trasy:")
        print(f"   Start: ({start_x:.1f}, {start_y:.1f})")
        print(f"   Cel:   ({end_x:.1f}, {end_y:.1f})")

        # Sprawdź odległość w linii prostej
        direct_distance = start_point.distance_to(end_point)
        print(f"   Odległość w linii prostej: {direct_distance:.1f} m")

        start_time = time.time()
        path = self.pathfinder.find_path(start_point, end_point)
        computation_time = time.time() - start_time

        if path:
            print(f"✅ Znaleziono bezpieczną trasę!")
            print(f"   • Punkty trasy: {len(path)}")
            print(f"   • Czas obliczeń: {computation_time:.4f} s")

            # Dodatkowa walidacja
            if self.pathfinder.validate_complete_path(path):
                print("✅ Trasa przeszła pełną walidację bezpieczeństwa")
            else:
                print("⚠️  Ostrzeżenie: Trasa może zawierać problemy")

            if optimize:
                print("🔄 Optymalizacja trasy...")
                optimized_path = self.pathfinder.optimize_path(path)

                # Waliduj zoptymalizowaną trasę
                if self.pathfinder.validate_complete_path(optimized_path):
                    print(f"✅ Zoptymalizowano do: {len(optimized_path)} punktów")

                    # Wyświetl statystyki
                    stats = self.pathfinder.get_path_statistics(optimized_path)
                    print(f"   • Długość trasy: {stats['total_distance']:.1f} m")
                    print(f"   • Współczynnik wydłużenia: {stats['total_distance'] / direct_distance:.2f}x")
                    print(f"   • Średnia długość segmentu: {stats['avg_segment_length']:.1f} m")
                    print(f"   • Liczba zakrętów: {stats['num_turns']}")

                    return optimized_path, computation_time, start_point, end_point
                else:
                    print("⚠️  Optymalizacja spowodowała kolizje - używam oryginalnej trasy")
                    return path, computation_time, start_point, end_point
            else:
                return path, computation_time, start_point, end_point
        else:
            print("❌ Nie znaleziono bezpiecznej trasy!")
            return [], computation_time, start_point, end_point

    def visualize(self, path=None, start_point=None, end_point=None,
                  show_animation=False, show_safe_points=False, save_plot=False):
        """Wizualizuje mapę i trasę"""
        if not self.map_data:
            raise ValueError("❌ Najpierw wygeneruj mapę!")

        print("\n📊 Tworzenie wizualizacji...")

        # Zawsze utwórz nowy visualizer
        if self.visualizer:
            self.visualizer.close()
        self.visualizer = DroneMapVisualizer(self.map_data)

        # RYSUJ MAPĘ
        self.visualizer.plot_map(show_safe_points=show_safe_points,
                                 pathfinder=self.pathfinder)

        if path:
            self.visualizer.plot_path(path, start_point, end_point)

            if hasattr(self, '_last_computation_time'):
                self.visualizer.plot_statistics(path, self._last_computation_time,
                                                self.pathfinder)

            if show_animation:
                print("🎬 Uruchamianie animacji lotu...")
                self._animation = self.visualizer.animate_drone_flight(path, interval=600,
                                                                       save_animation=save_plot)

        if save_plot:
            filename = f"drone_map_{int(time.time())}.png"
            self.visualizer.save(filename)
            print(f"💾 Mapa zapisana jako {filename}")

        # Pokaż wykres
        if show_animation:
            import matplotlib.pyplot as plt
            print("▶️ Trwa animacja... Zamknij okno, aby kontynuować.")
            plt.show(block=True)
        else:
            # Upewnij się, że mapa jest wyświetlana
            self.visualizer.show()

    def visualize_landing_zones(self, zone_size=20, min_distance=50):
        """Wizualizuje bezpieczne strefy lądowania"""
        if not self.pathfinder:
            raise ValueError("❌ Najpierw zainicjalizuj pathfinder!")

        print("🔍 Wyszukiwanie i wizualizacja bezpiecznych stref lądowania...")

        # Znajdź bezpieczne strefy lądowania
        safe_zones = self.pathfinder.get_safe_landing_zones(
            landing_zone_size=zone_size,
            min_distance_between=min_distance
        )

        if not safe_zones:
            print("❌ Nie znaleziono bezpiecznych stref lądowania!")
            return []

        # Utwórz nowy visualizer
        if self.visualizer:
            self.visualizer.close()
        self.visualizer = DroneMapVisualizer(self.map_data)

        # Wizualizuj strefy lądowania
        self.visualizer.plot_landing_zones(safe_zones, zone_size)

        # Pokaż wykres
        print("📊 Wyświetlanie mapy ze strefami lądowania...")
        try:
            self.visualizer.show()
            print("✅ Wizualizacja stref lądowania zakończona!")
        except KeyboardInterrupt:
            print("\n⏭️ Wizualizacja przerwana przez użytkownika")
        except Exception as e:
            print(f"❌ Błąd podczas wyświetlania: {e}")

        return safe_zones

    def run_demo(self):
        """Uruchamia demonstrację systemu"""
        print("\n🎮 DEMONSTRACJA OPTYMALIZATORA TRAS DRONÓW")
        print("=" * 60)

        try:
            # Generuj mapę
            self.generate_map(building_count=25, forbidden_zones=10, seed=42)

            # Inicjalizuj pathfinder
            self.initialize_pathfinder(safety_margin=10)

            # Wyznacz przykładową trasę
            start_x, start_y = 80, 80
            end_x, end_y = self.map_width - 80, self.map_height - 80

            path, comp_time, start_point, end_point = self.find_path(
                start_x, start_y, end_x, end_y, optimize=True)

            self._last_computation_time = comp_time

            if path:
                # Wizualizuj z animacją
                self.visualize(path, start_point, end_point,
                               show_animation=True, save_plot=True)

            print("\n🎉 Demonstracja zakończona pomyślnie!")

        except Exception as e:
            print(f"❌ Błąd podczas demonstracji: {e}")
            import traceback
            traceback.print_exc()

    def run_interactive(self):
        """Tryb interaktywny"""
        print("\n🎮 TRYB INTERAKTYWNY")
        print("=" * 40)

        while True:
            print("\nOpcje:")
            print("1. 🗺️  Nowa mapa")
            print("2. 🎯 Wyznacz trasę")
            print("3. 📊 Wizualizuj mapę")
            print("4. 🔍 Znajdź bezpieczne strefy lądowania")
            print("5. 📈 Test wydajności")
            print("6. 🚪 Wyjście")

            try:
                choice = input("\nWybierz opcję (1-6): ").strip()

                if choice == '1':
                    print("\nKonfiguracja mapy:")
                    buildings = int(input("Liczba budynków (10-50) [25]: ") or 25)
                    zones = int(input("Liczba stref zakazanych (3-20) [8]: ") or 8)
                    seed = int(input("Seed (dla powtarzalności) [42]: ") or 42)
                    margin = int(input("Margines bezpieczeństwa (10-50) [25]: ") or 25)

                    self.generate_map(buildings, zones, seed)
                    self.initialize_pathfinder(margin)

                elif choice == '2':
                    if not self.pathfinder:
                        print("❌ Najpierw wygeneruj mapę!")
                        continue

                    print(f"\nWspółrzędne (0-{self.map_width} x 0-{self.map_height}):")
                    start_x = float(input(f"Start X: "))
                    start_y = float(input(f"Start Y: "))
                    end_x = float(input(f"Cel X: "))
                    end_y = float(input(f"Cel Y: "))

                    path, comp_time, start_point, end_point = self.find_path(
                        start_x, start_y, end_x, end_y)

                    self._last_computation_time = comp_time

                    if path:
                        self.visualize(path, start_point, end_point,
                                       show_animation=True)

                elif choice == '3':
                    if not self.map_data:
                        print("❌ Najpierw wygeneruj mapę!")
                        continue

                    print("📊 Tworzenie wizualizacji mapy...")
                    try:
                        self.visualize(show_safe_points=False)
                        print("✅ Wizualizacja zakończona!")
                    except KeyboardInterrupt:
                        print("\n⏭️ Wizualizacja przerwana przez użytkownika")
                    except Exception as e:
                        print(f"❌ Błąd podczas wizualizacji: {e}")
                        import traceback
                        traceback.print_exc()

                elif choice == '4':
                    if not self.pathfinder:
                        print("❌ Najpierw wygeneruj mapę!")
                        continue

                    print("\nKonfiguracja stref lądowania:")
                    try:
                        zone_size = int(input("Rozmiar strefy lądowania (5-50m) [20]: ") or 20)
                        min_distance = int(input("Min. odległość między strefami (10-100m) [50]: ") or 50)

                        # Walidacja parametrów
                        if zone_size < 5 or zone_size > 50:
                            print("⚠️ Rozmiar strefy musi być między 5 a 50m. Używam domyślnej wartości 20m.")
                            zone_size = 20

                        if min_distance < 10 or min_distance > 100:
                            print("⚠️ Minimalna odległość musi być między 10 a 100m. Używam domyślnej wartości 50m.")
                            min_distance = 50

                        safe_zones = self.visualize_landing_zones(zone_size, min_distance)

                        if safe_zones:
                            print(f"✅ Znaleziono {len(safe_zones)} bezpiecznych stref lądowania")
                            print("   Strefy zostały wyświetlone na mapie")
                        else:
                            print("❌ Nie znaleziono bezpiecznych stref lądowania")
                            print("   Spróbuj zmniejszyć rozmiar strefy lub minimalną odległość")

                    except KeyboardInterrupt:
                        print("\n⏭️ Przerwano wyszukiwanie stref lądowania")
                    except ValueError as ve:
                        print(f"❌ Błąd danych wejściowych: {ve}")
                        print("   Używam domyślnych wartości: rozmiar=20m, odległość=50m")
                        try:
                            safe_zones = self.visualize_landing_zones(20, 50)
                        except Exception:
                            print("❌ Nie udało się wyszukać stref z domyślnymi parametrami")
                    except Exception as e:
                        print(f"❌ Błąd podczas wyszukiwania stref: {e}")
                        import traceback
                        traceback.print_exc()

                elif choice == '5':
                    if not self.pathfinder:
                        print("❌ Najpierw wygeneruj mapę!")
                        continue

                    print("📈 Test wydajności - 10 losowych tras...")
                    times = []
                    successful_paths = 0

                    for i in range(10):
                        import random
                        start_x = random.randint(50, self.map_width - 50)
                        start_y = random.randint(50, self.map_height - 50)
                        end_x = random.randint(50, self.map_width - 50)
                        end_y = random.randint(50, self.map_height - 50)

                        path, comp_time, _, _ = self.find_path(start_x, start_y, end_x, end_y)
                        times.append(comp_time)
                        if path:
                            successful_paths += 1

                        print(f"Test {i + 1}/10: {comp_time:.3f}s {'✅' if path else '❌'}")

                    print(f"\n📊 Wyniki testu wydajności:")
                    print(f"Udane trasy: {successful_paths}/10 ({successful_paths * 10}%)")
                    print(f"Średni czas: {sum(times) / len(times):.3f}s")
                    print(f"Najszybszy: {min(times):.3f}s")
                    print(f"Najwolniejszy: {max(times):.3f}s")

                elif choice == '6':
                    print("👋 Do widzenia!")
                    # Zamknij wszystkie visualizery
                    if self.visualizer:
                        self.visualizer.close()
                    break

                else:
                    print("❌ Nieprawidłowa opcja!")

            except ValueError as e:
                print(f"❌ Błąd wprowadzania danych: {e}")
            except KeyboardInterrupt:
                print("\n👋 Przerwano przez użytkownika!")
                break
            except Exception as e:
                print(f"❌ Nieoczekiwany błąd: {e}")
                import traceback
                traceback.print_exc()


def main():
    """Główna funkcja programu"""
    try:
        print("🚁 Optymalizator Tras dla Dronów")
        print("Projekt z geometrii obliczeniowej")
        print("Autorzy:\n Alan Ozga\n Maciej Sadkowski\n Bartosz Szmyd\n")
        print("=" * 50)

        optimizer = DroneOptimizer(map_width=1200, map_height=900)

        # Sprawdź argumenty linii poleceń
        if len(sys.argv) > 1 and sys.argv[1] == '--demo':
            optimizer.run_demo()
        else:
            optimizer.run_interactive()

    except KeyboardInterrupt:
        print("\n👋 Program przerwany przez użytkownika!")
    except Exception as e:
        print(f"❌ Krytyczny błąd: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Zamknij wszystkie figury matplotlib
        import matplotlib.pyplot as plt
        plt.close('all')


if __name__ == "__main__":
    main()