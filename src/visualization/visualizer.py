import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import numpy as np
import time


class DroneMapVisualizer:
    def __init__(self, map_data):
        self.map_data = map_data
        self.fig = None
        self.ax = None
        self.colors = {
            'building': '#8B4513',
            'forbidden': '#FF4444',
            'path': '#0066CC',
            'start': '#00AA00',
            'end': '#AA0000',
            'drone': '#FFD700',
            'safe_points': '#CCCCCC',
            'landing_zones': '#90EE90'
        }
        self._ensure_figure()

    def _ensure_figure(self):
        """Zapewnia że figura i osie są utworzone"""
        if self.fig is None or self.ax is None or not plt.fignum_exists(self.fig.number):
            plt.close('all')  # Zamknij wszystkie poprzednie figury
            self.fig, self.ax = plt.subplots(figsize=(14, 10))
            # Upewnij się, że figura jest widoczna
            self.fig.canvas.draw()
            self.fig.show()

    def plot_map(self, show_safe_points=False, pathfinder=None):
        """Wizualizuje mapę z przeszkodami"""
        self._ensure_figure()
        self.ax.clear()
        self.ax.set_xlim(0, self.map_data.width)
        self.ax.set_ylim(0, self.map_data.height)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_facecolor('#F0F8FF')  # Alice Blue background

        # Rysuj budynki
        for i, building in enumerate(self.map_data.buildings):
            self._draw_polygon(building, self.colors['building'],
                               f'Budynek' if i == 0 else "", alpha=0.8)

        # Rysuj strefy zakazane
        for i, zone in enumerate(self.map_data.forbidden_zones):
            self._draw_polygon(zone, self.colors['forbidden'],
                               f'Strefa zakazana' if i == 0 else "", alpha=0.6)

        # Rysuj bezpieczne punkty nawigacyjne (opcjonalnie)
        if show_safe_points and pathfinder and hasattr(pathfinder, 'safe_points'):
            for point in pathfinder.safe_points[::3]:  # Co trzeci punkt dla czytelności
                self.ax.plot(point.x, point.y, 'o', color=self.colors['safe_points'],
                             markersize=2, alpha=0.5)

        self.ax.set_title('Mapa Dronów - Przeszkody i Strefy Zakazane', fontsize=16, fontweight='bold')
        self.ax.set_xlabel('Współrzędna X [m]', fontsize=12)
        self.ax.set_ylabel('Współrzędna Y [m]', fontsize=12)

        # Dodaj legendę tylko jeśli są elementy
        handles, labels = self.ax.get_legend_handles_labels()
        if handles:
            self.ax.legend(loc='upper right', framealpha=0.9)

    def plot_landing_zones(self, landing_zones, zone_size=20):
        """Wizualizuje bezpieczne strefy lądowania"""
        self._ensure_figure()

        print(f"📊 Wizualizacja {len(landing_zones)} stref lądowania...")

        # Najpierw narysuj mapę
        self.plot_map()

        # Dodaj strefy lądowania
        for i, zone in enumerate(landing_zones):
            # Narysuj kwadrat strefy lądowania
            half_size = zone_size / 2
            landing_square = patches.Rectangle(
                (zone.x - half_size, zone.y - half_size),
                zone_size, zone_size,
                facecolor=self.colors['landing_zones'],
                edgecolor='darkgreen',
                alpha=0.7,
                linewidth=2,
                label='Strefa lądowania' if i == 0 else ""
            )
            self.ax.add_patch(landing_square)

            # Dodaj marker w środku
            self.ax.plot(zone.x, zone.y, 'o',
                         color='darkgreen', markersize=8,
                         markeredgecolor='white', markeredgewidth=2,
                         zorder=10)

            # Dodaj numer strefy (tylko dla pierwszych 20 dla czytelności)
            if i < 20:
                self.ax.annotate(f'{i+1}', (zone.x, zone.y),
                                 xytext=(3, 3), textcoords='offset points',
                                 fontsize=8, fontweight='bold', color='darkgreen',
                                 bbox=dict(boxstyle='round,pad=0.2',
                                           facecolor='white', alpha=0.8))

        # Aktualizuj tytuł
        self.ax.set_title(f'Mapa Dronów - {len(landing_zones)} Bezpiecznych Stref Lądowania ({zone_size}x{zone_size}m)',
                          fontsize=14, fontweight='bold')

        # Dodaj informacje o strefach
        info_text = f"""STREFY LĄDOWANIA:
================
Liczba stref: {len(landing_zones)}
Rozmiar strefy: {zone_size}x{zone_size} m
Minimalna powierzchnia: {zone_size*zone_size} m²

Strefy są rozmieszczone tak, aby:
• Nie kolidować z budynkami
• Nie kolidować ze strefami zakazanymi  
• Zachować bezpieczną odległość
• Umożliwić bezpieczne lądowanie"""

        # Dodaj tekst na wykres
        self.ax.text(0.02, 0.98, info_text, transform=self.ax.transAxes,
                     verticalalignment='top', fontsize=9,
                     bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.9))

        # Aktualizuj legendę
        handles, labels = self.ax.get_legend_handles_labels()
        if handles:
            self.ax.legend(loc='upper right', framealpha=0.9)

    def plot_precise_path(self, path, start_point=None, end_point=None, show_safety_zones=True):
        """Wizualizuje bardzo dokładnie trasę z wszystkimi szczegółami"""
        self._ensure_figure()

        if not path:
            print("Brak trasy do wyświetlenia")
            return

        # Rysuj trasę z bardzo wysoką dokładnością
        x_coords = [p.x for p in path]
        y_coords = [p.y for p in path]

        # Główna linia trasy - grubsza i bardziej widoczna
        self.ax.plot(x_coords, y_coords, color=self.colors['path'],
                     linewidth=6, label='Trasa drona (zoptymalizowana)',
                     alpha=0.9, zorder=10, solid_capstyle='round')

        # Dodaj cień pod trasą dla lepszej widoczności
        self.ax.plot(x_coords, y_coords, color='white',
                     linewidth=8, alpha=0.7, zorder=9)

        # Punkty trasy z numeracją
        for i, point in enumerate(path):
            # Punkt trasy
            self.ax.plot(point.x, point.y, 'o', color='white',
                         markersize=10, zorder=12, markeredgecolor=self.colors['path'],
                         markeredgewidth=2)

            # Numeracja punktów (co 3 punkt dla czytelności)
            if i % 3 == 0 or i == len(path) - 1:
                self.ax.annotate(f'{i}', (point.x, point.y),
                                 xytext=(5, 5), textcoords='offset points',
                                 fontsize=8, fontweight='bold', color='darkblue',
                                 bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8))

        # Strefy bezpieczeństwa wokół trasy (opcjonalnie)
        if show_safety_zones:
            self._draw_safety_corridor(path)

        # Oznacz start i koniec z większymi markerami
        if start_point:
            self.ax.plot(start_point.x, start_point.y, 'o',
                         color=self.colors['start'], markersize=20,
                         label='START', zorder=15, markeredgecolor='white',
                         markeredgewidth=3)
            self.ax.annotate('START', (start_point.x, start_point.y),
                             xytext=(10, 10), textcoords='offset points',
                             fontsize=12, fontweight='bold', color='green',
                             bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgreen', alpha=0.9))

        if end_point:
            self.ax.plot(end_point.x, end_point.y, 's',
                         color=self.colors['end'], markersize=20,
                         label='CEL', zorder=15, markeredgecolor='white',
                         markeredgewidth=3)
            self.ax.annotate('CEL', (end_point.x, end_point.y),
                             xytext=(10, -20), textcoords='offset points',
                             fontsize=12, fontweight='bold', color='red',
                             bbox=dict(boxstyle='round,pad=0.3', facecolor='lightcoral', alpha=0.9))

        # Dodaj strzałki kierunku - więcej i bardziej widoczne
        self._add_detailed_direction_arrows(path)

        # Dodaj informacje o długości segmentów
        self._add_segment_lengths(path)

        # Aktualizuj legendę
        self.ax.legend(loc='upper right', framealpha=0.95, fontsize=10)

    def _draw_safety_corridor(self, path):
        """Rysuje korytarz bezpieczeństwa wokół trasy"""
        if len(path) < 2:
            return

        safety_margin = 15  # Margines bezpieczeństwa

        # Twórz wielokąt korytarza bezpieczeństwa
        left_points = []
        right_points = []

        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]

            # Wektor kierunku
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            length = (dx * dx + dy * dy) ** 0.5

            if length > 0:
                # Wektor prostopadły (obrócony o 90 stopni)
                perp_x = -dy / length * safety_margin
                perp_y = dx / length * safety_margin

                # Punkty po lewej i prawej stronie
                left_points.append((p1.x + perp_x, p1.y + perp_y))
                right_points.append((p1.x - perp_x, p1.y - perp_y))

        # Dodaj ostatni punkt
        if path:
            p_last = path[-1]
            if len(path) > 1:
                p_prev = path[-2]
                dx = p_last.x - p_prev.x
                dy = p_last.y - p_prev.y
                length = (dx * dx + dy * dy) ** 0.5

                if length > 0:
                    perp_x = -dy / length * safety_margin
                    perp_y = dx / length * safety_margin

                    left_points.append((p_last.x + perp_x, p_last.y + perp_y))
                    right_points.append((p_last.x - perp_x, p_last.y - perp_y))

        # Połącz punkty w wielokąt
        corridor_points = left_points + list(reversed(right_points))

        if corridor_points:
            corridor_patch = patches.Polygon(corridor_points, closed=True,
                                             facecolor='lightblue', alpha=0.2,
                                             edgecolor='blue', linewidth=1,
                                             label='Korytarz bezpieczeństwa')
            self.ax.add_patch(corridor_patch)

    def _add_detailed_direction_arrows(self, path):
        """Dodaje szczegółowe strzałki kierunku"""
        if len(path) < 2:
            return

        # Dodaj strzałki co 2-3 segmenty
        arrow_interval = max(1, len(path) // 6)

        for i in range(0, len(path) - 1, arrow_interval):
            dx = path[i + 1].x - path[i].x
            dy = path[i + 1].y - path[i].y

            # Normalizuj długość strzałki
            length = (dx ** 2 + dy ** 2) ** 0.5
            if length > 0:
                scale = min(25, length * 0.4)
                dx_norm = (dx / length) * scale
                dy_norm = (dy / length) * scale

                # Pozycja strzałki (w środku segmentu)
                arrow_x = path[i].x + dx * 0.5
                arrow_y = path[i].y + dy * 0.5

                self.ax.arrow(arrow_x, arrow_y, dx_norm, dy_norm,
                              head_width=12, head_length=8,
                              fc='darkblue', ec='darkblue',
                              alpha=0.8, zorder=11, linewidth=2)

    def _add_segment_lengths(self, path):
        """Dodaje długości segmentów na wykresie"""
        if len(path) < 2:
            return

        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i + 1]

            # Oblicz długość segmentu
            length = p1.distance_to(p2)

            # Pozycja tekstu (środek segmentu)
            mid_x = (p1.x + p2.x) / 2
            mid_y = (p1.y + p2.y) / 2

            # Dodaj tekst z długością (tylko dla dłuższych segmentów)
            if length > 30 and i % 2 == 0:  # Co drugi segment dla czytelności
                self.ax.annotate(f'{length:.1f}m', (mid_x, mid_y),
                                 fontsize=8, ha='center', va='center',
                                 bbox=dict(boxstyle='round,pad=0.2',
                                           facecolor='yellow', alpha=0.7),
                                 zorder=13)

    def plot_path(self, path, start_point=None, end_point=None, show_waypoints=True):
        """Główna metoda wizualizacji - używa precyzyjnej wersji"""
        self.plot_precise_path(path, start_point, end_point, show_safety_zones=True)

    def _draw_polygon(self, polygon, color, label, alpha=0.7):
        """Rysuje wielokąt"""
        points = [(p.x, p.y) for p in polygon.points]
        poly_patch = patches.Polygon(points, closed=True,
                                     facecolor=color, edgecolor='black',
                                     alpha=alpha, linewidth=1.5, label=label)
        self.ax.add_patch(poly_patch)

    def animate_drone_flight(self, path, interval=300, save_animation=False):
        """Animuje lot drona po trasie"""
        self._ensure_figure()

        if not path:
            print("Brak trasy do animacji")
            return None

        plt.ion()  # Tryb interaktywny dla animacji

        # Przygotuj tło (nie czyść mapy!)
        drone_point, = self.ax.plot([], [], 'o', color=self.colors['drone'],
                                    markersize=15, label='Dron', zorder=15,
                                    markeredgecolor='orange', markeredgewidth=2)
        trail_line, = self.ax.plot([], [], color='orange', alpha=0.6,
                                   linewidth=3, zorder=4)

        info_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes,
                                 verticalalignment='top', fontsize=10,
                                 bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        trail_x, trail_y = [], []

        def animate(frame):
            current_pos = path[frame]
            drone_point.set_data([current_pos.x], [current_pos.y])
            trail_x.append(current_pos.x)
            trail_y.append(current_pos.y)
            trail_line.set_data(trail_x, trail_y)

            progress = (frame + 1) / len(path) * 100
            info_text.set_text(f'Lot Drona\nKrok: {frame + 1}/{len(path)}\nPostęp: {progress:.1f}%')
            self.ax.set_title(f'Lot Drona - Krok {frame + 1}/{len(path)}',
                              fontsize=16, fontweight='bold')
            return drone_point, trail_line, info_text

        anim = FuncAnimation(self.fig, animate, frames=len(path),
                             interval=interval, blit=False, repeat=True)

        self.ax.legend(loc='upper right', framealpha=0.9)

        if save_animation:
            filename = f"drone_animation_{int(time.time())}.gif"
            anim.save(filename, writer='pillow', fps=3)
            print(f"Animacja zapisana jako {filename}")

        return anim

    def plot_statistics(self, path, computation_time, pathfinder=None):
        """Wyświetla statystyki trasy"""
        if not path:
            print("Brak trasy do analizy")
            return

        # Oblicz statystyki
        stats = self._calculate_path_statistics(path)

        # Dodaj informacje o pathfinderze
        graph_info = ""
        if pathfinder:
            graph_nodes = len(pathfinder.navigation_graph)
            graph_info = f"Węzły grafu: {graph_nodes}\n"

        # Stwórz tekst ze statystykami
        stats_text = f"""STATYSTYKI TRASY:
================
{graph_info}Długość trasy: {stats['total_distance']:.1f} m
Liczba punktów: {stats['num_waypoints']}
Liczba zakrętów: {stats['num_turns']}
Czas obliczeń: {computation_time:.3f} s

Średnia odległość między punktami: {stats['avg_segment_length']:.1f} m
Najdłuższy segment: {stats['max_segment']:.1f} m
Najkrótszy segment: {stats['min_segment']:.1f} m"""

        print(stats_text)

        # Dodaj tekst na wykres
        self.ax.text(0.02, 0.02, stats_text, transform=self.ax.transAxes,
                     verticalalignment='bottom', fontsize=9,
                     bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))

    def _calculate_path_statistics(self, path):
        """Oblicza szczegółowe statystyki ścieżki"""
        if len(path) < 2:
            return {'total_distance': 0, 'num_waypoints': len(path),
                    'num_turns': 0, 'avg_segment_length': 0,
                    'max_segment': 0, 'min_segment': 0}

        # Oblicz odległości segmentów
        segment_lengths = []
        total_distance = 0

        for i in range(len(path) - 1):
            distance = path[i].distance_to(path[i + 1])
            segment_lengths.append(distance)
            total_distance += distance

        return {
            'total_distance': total_distance,
            'num_waypoints': len(path),
            'num_turns': len(path) - 2,
            'avg_segment_length': total_distance / len(segment_lengths) if segment_lengths else 0,
            'max_segment': max(segment_lengths) if segment_lengths else 0,
            'min_segment': min(segment_lengths) if segment_lengths else 0
        }

    def plot_comparison(self, paths_dict, start_point=None, end_point=None):
        """Porównuje różne trasy na jednym wykresie"""
        self._ensure_figure()
        self.plot_map()

        colors = ['blue', 'red', 'green', 'purple', 'orange']

        for i, (name, path) in enumerate(paths_dict.items()):
            if path:
                color = colors[i % len(colors)]
                x_coords = [p.x for p in path]
                y_coords = [p.y for p in path]

                self.ax.plot(x_coords, y_coords, color=color,
                             linewidth=3, label=f'{name} ({len(path)} punktów)',
                             alpha=0.8)

        # Oznacz start i koniec
        if start_point:
            self.ax.plot(start_point.x, start_point.y, 'go', markersize=12, label='Start')
        if end_point:
            self.ax.plot(end_point.x, end_point.y, 'ro', markersize=12, label='Cel')

        self.ax.legend()
        self.ax.set_title('Porównanie Tras Dronów')

    def show(self):
        """Wyświetla wykres"""
        self._ensure_figure()
        plt.tight_layout()

        # Upewnij się, że okno jest na wierzchu i widoczne
        if self.fig:
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

            # Spróbuj przenieść okno na wierzch
            try:
                manager = self.fig.canvas.manager
                if hasattr(manager, 'window'):
                    manager.window.wm_attributes('-topmost', True)
                    manager.window.wm_attributes('-topmost', False)
            except:
                pass  # Ignoruj błędy związane z window managerem

        # Pokaż wykres w trybie nieblokującym
        plt.show(block=False)
        plt.pause(0.5)  # Daj czas na wyrenderowanie

        print("📊 Mapa wyświetlona. Zamknij okno aby kontynuować lub naciśnij Ctrl+C...")

        try:
            # Czekaj aż użytkownik zamknie okno
            while plt.get_fignums() and self.fig and plt.fignum_exists(self.fig.number):
                plt.pause(0.1)
            print("✅ Okno zostało zamknięte.")
        except KeyboardInterrupt:
            print("\n⏭️ Przerwano wyświetlanie (Ctrl+C)")
            if self.fig:
                plt.close(self.fig)
        except Exception as e:
            print(f"⚠️ Błąd podczas wyświetlania: {e}")
            if self.fig:
                plt.close(self.fig)

    def save(self, filename, dpi=300):
        """Zapisuje wykres do pliku"""
        self._ensure_figure()
        self.fig.savefig(filename, dpi=dpi, bbox_inches='tight', facecolor='white')
        print(f"Wykres zapisany jako {filename}")

    def close(self):
        """Zamyka figurę"""
        if self.fig is not None:
            plt.close(self.fig)
            self.fig = None
            self.ax = None
        # Dodatkowo zamknij wszystkie otwarte figury matplotlib
        plt.close('all')