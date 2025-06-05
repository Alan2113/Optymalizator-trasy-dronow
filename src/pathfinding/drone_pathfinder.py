import heapq
import math
import time
from geometry.point import Point
from geometry.triangulation import DelaunayTriangulation
from pathfinding.range_tree import RangeTree2D


class PathNode:
    def __init__(self, point, g_cost=0, h_cost=0, parent=None):
        self.point = point
        self.g_cost = g_cost  # Koszt od startu
        self.h_cost = h_cost  # Heurystyka do celu
        self.f_cost = g_cost + h_cost  # Całkowity koszt
        self.parent = parent

    def __lt__(self, other):
        return self.f_cost < other.f_cost

    def __eq__(self, other):
        return self.point == other.point


class DronePathfinder:
    def __init__(self, map_data, safety_margin=15):
        self.map_data = map_data
        self.safety_margin = safety_margin
        self.navigation_graph = {}
        self.range_tree = None
        self.safe_points = []
        self._build_navigation_graph()

    def _build_navigation_graph(self):
        """Buduje graf nawigacyjny używając triangulacji Delaunay"""
        print("Budowanie grafu nawigacyjnego...")
        start_time = time.time()

        # Zbierz wszystkie punkty z przeszkód z marginesem bezpieczeństwa
        all_points = []

        # Dodaj punkty z budynków (rozszerzone o margines)
        for building in self.map_data.buildings:
            expanded_points = self._expand_obstacle_points(building)
            all_points.extend(expanded_points)

        # Dodaj punkty z stref zakazanych
        for zone in self.map_data.forbidden_zones:
            expanded_points = self._expand_obstacle_points(zone)
            all_points.extend(expanded_points)

        # Dodaj punkty graniczne mapy
        margin = 30
        boundary_points = [
            Point(margin, margin),
            Point(self.map_data.width - margin, margin),
            Point(self.map_data.width - margin, self.map_data.height - margin),
            Point(margin, self.map_data.height - margin),
            # Dodatkowe punkty na krawędziach
            Point(self.map_data.width // 2, margin),
            Point(self.map_data.width - margin, self.map_data.height // 2),
            Point(self.map_data.width // 2, self.map_data.height - margin),
            Point(margin, self.map_data.height // 2)
        ]
        all_points.extend(boundary_points)

        # Usuń duplikaty
        unique_points = []
        for point in all_points:
            is_duplicate = False
            for existing in unique_points:
                if point.distance_to(existing) < 5:
                    is_duplicate = True
                    break
            if not is_duplicate:
                unique_points.append(point)

        # Wykonaj triangulację Delaunay
        if len(unique_points) >= 3:
            triangulation = DelaunayTriangulation(unique_points)
            triangles = triangulation.triangulate()

            # Buduj graf z centroidów trójkątów
            self._build_graph_from_triangulation(triangles)

        # Buduj drzewo zakresowe dla optymalizacji
        if unique_points:
            self.range_tree = RangeTree2D(unique_points)

        build_time = time.time() - start_time
        print(f"Graf zbudowany w {build_time:.3f} sekund")
        print(f"Liczba węzłów: {len(self.navigation_graph)}")

    def _expand_obstacle_points(self, obstacle):
        """Rozszerza punkty przeszkody o margines bezpieczeństwa"""
        expanded_points = []
        centroid = obstacle.centroid()

        for point in obstacle.points:
            # Oblicz kierunek od centroidu do punktu
            dx = point.x - centroid.x
            dy = point.y - centroid.y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance > 0:
                # Normalizuj i rozszerz
                dx_norm = dx / distance
                dy_norm = dy / distance

                expanded_point = Point(
                    point.x + dx_norm * self.safety_margin,
                    point.y + dy_norm * self.safety_margin
                )

                # Sprawdź granice mapy
                expanded_point.x = max(10, min(self.map_data.width - 10, expanded_point.x))
                expanded_point.y = max(10, min(self.map_data.height - 10, expanded_point.y))

                expanded_points.append(expanded_point)
            else:
                expanded_points.append(point)

        return expanded_points

    def _build_graph_from_triangulation(self, triangles):
        """Buduje graf nawigacyjny z triangulacji"""
        # Stwórz węzły z centroidów trójkątów
        triangle_centers = []

        for triangle in triangles:
            center_x = (triangle.p1.x + triangle.p2.x + triangle.p3.x) / 3
            center_y = (triangle.p1.y + triangle.p2.y + triangle.p3.y) / 3
            center = Point(center_x, center_y)

            # Sprawdź czy centroid nie jest w przeszkodzie
            if not self._point_in_obstacle(center):
                triangle_centers.append(center)
                self.safe_points.append(center)

        # Połącz sąsiednie centra
        for i, center1 in enumerate(triangle_centers):
            self.navigation_graph[center1] = []

            for j, center2 in enumerate(triangle_centers):
                if i != j:
                    distance = center1.distance_to(center2)
                    # Połącz tylko relatywnie bliskie punkty
                    if distance < 150 and self._can_connect(center1, center2):
                        self.navigation_graph[center1].append(center2)

    def _point_in_obstacle(self, point):
        """Sprawdza czy punkt jest w przeszkodzie"""
        for obstacle in self.map_data.get_obstacles():
            if obstacle.contains_point(point):
                return True
        return False

    def _can_connect(self, point1, point2):
        """Sprawdza czy można połączyć dwa punkty linią prostą (bardzo dokładnie)"""
        # Zwiększona dokładność sprawdzania
        distance = point1.distance_to(point2)
        steps = max(int(distance / 2), 20)  # Co 2 jednostki lub minimum 20 kroków

        for i in range(steps + 1):
            t = i / steps
            x = point1.x + t * (point2.x - point1.x)
            y = point1.y + t * (point2.y - point1.y)
            test_point = Point(x, y)

            # Sprawdź wszystkie przeszkody z dodatkowym marginesem
            for obstacle in self.map_data.get_obstacles():
                if obstacle.contains_point(test_point):
                    return False

                # Dodatkowe sprawdzenie - czy punkt jest zbyt blisko krawędzi
                if obstacle.is_point_on_edge(test_point, tolerance=self.safety_margin):
                    return False

        return True

    def _point_in_obstacle_with_margin(self, point):
        """Sprawdza czy punkt jest w przeszkodzie lub zbyt blisko niej"""
        for obstacle in self.map_data.get_obstacles():
            if obstacle.contains_point(point):
                return True

            # Sprawdź odległość od krawędzi przeszkody
            min_distance_to_edge = float('inf')
            for i in range(len(obstacle.points)):
                p1 = obstacle.points[i]
                p2 = obstacle.points[(i + 1) % len(obstacle.points)]

                # Oblicz odległość punktu od odcinka
                distance = self._point_to_segment_distance(point, p1, p2)
                min_distance_to_edge = min(min_distance_to_edge, distance)

            if min_distance_to_edge < self.safety_margin:
                return True

        return False

    def _point_to_segment_distance(self, point, seg_start, seg_end):
        """Oblicza odległość punktu od odcinka"""
        # Wektor odcinka
        dx = seg_end.x - seg_start.x
        dy = seg_end.y - seg_start.y

        if dx == 0 and dy == 0:
            return point.distance_to(seg_start)

        # Parametr t dla projekcji punktu na odcinek
        t = ((point.x - seg_start.x) * dx + (point.y - seg_start.y) * dy) / (dx * dx + dy * dy)

        # Ograniczenie t do [0, 1]
        t = max(0, min(1, t))

        # Punkt na odcinku najbliższy danemu punktowi
        closest_x = seg_start.x + t * dx
        closest_y = seg_start.y + t * dy
        closest_point = Point(closest_x, closest_y)

        return point.distance_to(closest_point)

    def validate_complete_path(self, path):
        """Waliduje całą ścieżkę pod kątem kolizji"""
        if not path or len(path) < 2:
            return True

        print("🔍 Walidacja ścieżki pod kątem kolizji...")

        for i in range(len(path) - 1):
            if not self._can_connect(path[i], path[i + 1]):
                print(f"❌ Kolizja wykryta między punktami {i} i {i + 1}")
                print(f"   Punkt {i}: ({path[i].x:.1f}, {path[i].y:.1f})")
                print(f"   Punkt {i + 1}: ({path[i + 1].x:.1f}, {path[i + 1].y:.1f})")
                return False

        print("✅ Ścieżka jest bezpieczna - brak kolizji")
        return True

    def find_path(self, start, end):
        """Główna metoda wyznaczania trasy A* z walidacją"""
        print(f"Wyznaczanie trasy z {start} do {end}...")

        # Sprawdź czy punkty startowy i końcowy są bezpieczne
        if self._point_in_obstacle_with_margin(start):
            print("❌ Punkt startowy znajduje się w strefie zakazanej lub zbyt blisko przeszkody!")
            return []

        if self._point_in_obstacle_with_margin(end):
            print("❌ Punkt docelowy znajduje się w strefie zakazanej lub zbyt blisko przeszkody!")
            return []

        # Znajdź najbliższe punkty w grafie
        start_node = self._find_nearest_graph_node(start)
        end_node = self._find_nearest_graph_node(end)

        if not start_node or not end_node:
            print("Nie można znaleźć bezpiecznych punktów startowych/końcowych w grafie!")
            return []

        print(f"Start graph node: {start_node}")
        print(f"End graph node: {end_node}")

        # A* algorithm
        open_set = []
        closed_set = set()
        came_from = {}

        start_path_node = PathNode(
            start_node,
            0,
            self._heuristic(start_node, end_node)
        )

        heapq.heappush(open_set, start_path_node)
        g_score = {start_node: 0}

        while open_set:
            current = heapq.heappop(open_set)

            if current.point == end_node:
                # Rekonstruuj ścieżkę
                path = self._reconstruct_path(came_from, current.point)

                # Dodaj start i end jeśli różnią się od węzłów grafu
                final_path = []
                if start != start_node:
                    final_path.append(start)
                final_path.extend(path)
                if end != end_node:
                    final_path.append(end)

                # WALIDACJA ŚCIEŻKI
                if self.validate_complete_path(final_path):
                    print(f"✅ Znaleziono bezpieczną ścieżkę z {len(final_path)} punktami")
                    return final_path
                else:
                    print("❌ Znaleziona ścieżka zawiera kolizje - kontynuowanie wyszukiwania...")
                    continue

            closed_set.add(current.point)

            # Sprawdź sąsiadów
            if current.point in self.navigation_graph:
                for neighbor in self.navigation_graph[current.point]:
                    if neighbor in closed_set:
                        continue

                    tentative_g = g_score.get(current.point, float('inf')) + current.point.distance_to(neighbor)

                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current.point
                        g_score[neighbor] = tentative_g

                        neighbor_node = PathNode(
                            neighbor,
                            tentative_g,
                            self._heuristic(neighbor, end_node)
                        )

                        heapq.heappush(open_set, neighbor_node)

        print("❌ Nie znaleziono bezpiecznej ścieżki!")
        return []

    def _reconstruct_path(self, came_from, current):
        """Rekonstruuje ścieżkę z mapy came_from"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _find_nearest_graph_node(self, point):
        """Znajduje najbliższy węzeł grafu"""
        min_distance = float('inf')
        nearest_node = None

        for graph_point in self.navigation_graph.keys():
            distance = point.distance_to(graph_point)
            if distance < min_distance:
                # Sprawdź czy można połączyć punkt z węzłem grafu
                if distance < 50 or self._can_connect(point, graph_point):
                    min_distance = distance
                    nearest_node = graph_point

        return nearest_node

    def _heuristic(self, point1, point2):
        """Funkcja heurystyczna dla A* (odległość euklidesowa)"""
        return point1.distance_to(point2)

    def optimize_path(self, path):
        """Optymalizuje ścieżkę usuwając niepotrzebne punkty"""
        if len(path) <= 2:
            return path

        optimized = [path[0]]

        i = 0
        while i < len(path) - 1:
            # Znajdź najdalszy punkt, do którego można dojść w linii prostej
            farthest = i + 1
            for j in range(i + 2, len(path)):
                if self._can_connect(path[i], path[j]):
                    farthest = j
                else:
                    break

            optimized.append(path[farthest])
            i = farthest

        # Waliduj zoptymalizowaną ścieżkę
        if self.validate_complete_path(optimized):
            print(f"Ścieżka zoptymalizowana z {len(path)} do {len(optimized)} punktów")
            return optimized
        else:
            print("⚠️ Optymalizacja spowodowała kolizje - zwracam oryginalną ścieżkę")
            return path

    def get_path_statistics(self, path):
        """Oblicza statystyki ścieżki"""
        if len(path) < 2:
            return {
                'total_distance': 0,
                'num_waypoints': len(path),
                'num_turns': 0,
                'avg_segment_length': 0,
                'max_segment': 0,
                'min_segment': 0
            }

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

    def get_safe_landing_zones(self, landing_zone_size=20, min_distance_between=50):
        """
        Znajduje bezpieczne strefy lądowania o określonym rozmiarze

        Args:
            landing_zone_size: Rozmiar strefy lądowania (kwadrat)
            min_distance_between: Minimalna odległość między strefami lądowania
        """
        print(f"🔍 Wyszukiwanie bezpiecznych stref lądowania {landing_zone_size}x{landing_zone_size}m...")

        safe_zones = []
        half_size = zone_size / 2

        # Sprawdź punkty na siatce z odpowiednim odstępem
        step = max(min_distance_between // 2, 10)  # Mniejszy krok dla dokładniejszego wyszukiwania, minimum 10

        for x in range(int(half_size), int(self.map_data.width - half_size), step):
            for y in range(int(half_size), int(self.map_data.height - half_size), step):
                center = Point(x, y)

                # Sprawdź czy całą strefa lądowania jest bezpieczna
                if self._is_landing_zone_safe(center, zone_size):
                    # Sprawdź odległość od innych stref lądowania
                    too_close = False
                    for existing_zone in safe_zones:
                        if center.distance_to(existing_zone) < min_distance_between:
                            too_close = True
                            break

                    if not too_close:
                        safe_zones.append(center)

        print(f"✅ Znaleziono {len(safe_zones)} bezpiecznych stref lądowania")
        return safe_zones

    def _is_landing_zone_safe(self, center, zone_size):
        """
        Sprawdza czy strefa lądowania o zadanym rozmiarze jest bezpieczna

        Args:
            center: Środek strefy lądowania
            zone_size: Rozmiar strefy (kwadrat)
        """
        half_size = zone_size / 2

        # Punkty do sprawdzenia - narożniki i środek każdej krawędzi
        test_points = [
            # Narożniki
            Point(center.x - half_size, center.y - half_size),
            Point(center.x + half_size, center.y - half_size),
            Point(center.x + half_size, center.y + half_size),
            Point(center.x - half_size, center.y + half_size),
            # Środki krawędzi
            Point(center.x, center.y - half_size),
            Point(center.x + half_size, center.y),
            Point(center.x, center.y + half_size),
            Point(center.x - half_size, center.y),
            # Środek
            Point(center.x, center.y)
        ]

        # Sprawdź czy wszystkie punkty testowe są bezpieczne
        for point in test_points:
            # Sprawdź granice mapy
            if (point.x < 0 or point.x > self.map_data.width or
                    point.y < 0 or point.y > self.map_data.height):
                return False

            # Sprawdź kolizje z przeszkodami (z dodatkowym marginesem)
            if self._point_in_obstacle_with_margin(point):
                return False

        # Dodatkowe sprawdzenie - czy żadna przeszkoda nie przecina strefy lądowania
        return self._is_landing_area_clear(center, zone_size)

    def _is_landing_area_clear(self, center, zone_size):
        """
        Sprawdza czy obszar lądowania nie przecina się z żadną przeszkodą
        """
        half_size = zone_size / 2

        # Granice obszaru lądowania
        zone_min_x = center.x - half_size
        zone_max_x = center.x + half_size
        zone_min_y = center.y - half_size
        zone_max_y = center.y + half_size

        # Sprawdź każdą przeszkodę
        for obstacle in self.map_data.get_obstacles():
            # Pobierz bounding box przeszkody
            bbox = obstacle.get_bounding_box()
            obs_min_x, obs_min_y, obs_max_x, obs_max_y = bbox

            # Dodaj margines bezpieczeństwa do przeszkody
            margin = self.safety_margin
            obs_min_x -= margin
            obs_max_x += margin
            obs_min_y -= margin
            obs_max_y += margin

            # Sprawdź czy bounding boxy się przecinają
            if not (zone_max_x <= obs_min_x or obs_max_x <= zone_min_x or
                    zone_max_y <= obs_min_y or obs_max_y <= zone_min_y):

                # Jeśli bounding boxy się przecinają, sprawdź dokładniej
                # Sprawdź czy środek przeszkody jest w strefie lądowania
                obs_center = obstacle.centroid()
                if (zone_min_x <= obs_center.x <= zone_max_x and
                        zone_min_y <= obs_center.y <= zone_max_y):
                    return False

                # Sprawdź czy którykolwiek punkt przeszkody jest w strefie lądowania
                for point in obstacle.points:
                    if (zone_min_x <= point.x <= zone_max_x and
                            zone_min_y <= point.y <= zone_max_y):
                        return False

        return True