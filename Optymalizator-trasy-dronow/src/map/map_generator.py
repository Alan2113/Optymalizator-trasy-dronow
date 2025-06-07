import random
import math
from geometry.point import Point
from geometry.polygon import Polygon
from geometry.convex_hull import ConvexHull
from map.map_data import MapData


class Rectangle:
    def __init__(self, x, y, width, height):
        self.x = x  # Lewy dolny róg
        self.y = y
        self.width = width
        self.height = height
        self.x2 = x + width  # Prawy górny róg
        self.y2 = y + height

    def overlaps(self, other):
        """Sprawdza czy prostokąty się nakładają"""
        return not (self.x2 <= other.x or other.x2 <= self.x or
                    self.y2 <= other.y or other.y2 <= self.y)

    def to_polygon(self):
        """Konwertuje prostokąt na wielokąt"""
        points = [
            Point(self.x, self.y),  # Lewy dolny
            Point(self.x2, self.y),  # Prawy dolny
            Point(self.x2, self.y2),  # Prawy górny
            Point(self.x, self.y2)  # Lewy górny
        ]
        return Polygon(points)

    def get_center(self):
        """Zwraca środek prostokąta"""
        return Point(self.x + self.width / 2, self.y + self.height / 2)


class MapGenerator:
    def __init__(self, width, height, seed=None):
        self.width = width
        self.height = height
        if seed:
            random.seed(seed)

    def generate_map(self, building_count=20, forbidden_zone_count=5):
        """Generuje kompletną mapę"""
        map_data = MapData(self.width, self.height)

        # Generuj budynki jako prostokąty
        buildings = self.generate_rectangular_buildings(building_count)
        for building in buildings:
            map_data.add_building(building)

        # Generuj strefy zakazane (mogą być nieregularne)
        forbidden_zones = self.generate_forbidden_zones(forbidden_zone_count, buildings)
        for zone in forbidden_zones:
            map_data.add_forbidden_zone(zone)

        return map_data

    def generate_rectangular_buildings(self, count):
        """Generuje prostokątne budynki bez nakładania się"""
        buildings = []
        rectangles = []  # Lista prostokątów do sprawdzania kolizji

        # Parametry generowania
        min_size = 30
        max_size = 100
        margin = 20  # Margines od krawędzi mapy
        min_distance = 15  # Minimalna odległość między budynkami
        max_attempts = count * 100  # Maksymalna liczba prób

        attempts = 0

        print(f"Generowanie {count} prostokątnych budynków...")

        while len(buildings) < count and attempts < max_attempts:
            attempts += 1

            # Losowe wymiary budynku
            if random.random() < 0.3:  # 30% szans na kwadrat
                size = random.randint(min_size, max_size)
                width = height = size
            else:  # 70% szans na prostokąt
                width = random.randint(min_size, max_size)
                height = random.randint(min_size, max_size)

            # Losowa pozycja (sprawdzamy czy mieści się w mapie)
            max_x = self.width - width - margin
            max_y = self.height - height - margin

            if max_x <= margin or max_y <= margin:
                continue  # Budynek za duży dla mapy

            x = random.randint(margin, max_x)
            y = random.randint(margin, max_y)

            # Stwórz nowy prostokąt
            new_rect = Rectangle(x, y, width, height)

            # Sprawdź kolizje z istniejącymi budynkami
            collision = False
            for existing_rect in rectangles:
                if self._rectangles_too_close(new_rect, existing_rect, min_distance):
                    collision = True
                    break

            if not collision:
                # Dodaj budynek
                rectangles.append(new_rect)
                building_polygon = new_rect.to_polygon()
                buildings.append(building_polygon)

                if len(buildings) % 5 == 0:
                    print(f"Wygenerowano {len(buildings)}/{count} budynków...")

        print(f"Pomyślnie wygenerowano {len(buildings)} budynków w {attempts} próbach")
        return buildings

    def _rectangles_too_close(self, rect1, rect2, min_distance):
        """Sprawdza czy prostokąty są za blisko siebie"""
        # Rozszerz pierwszy prostokąt o min_distance
        expanded_rect = Rectangle(
            rect1.x - min_distance,
            rect1.y - min_distance,
            rect1.width + 2 * min_distance,
            rect1.height + 2 * min_distance
        )

        return expanded_rect.overlaps(rect2)

    def generate_forbidden_zones(self, count, existing_buildings):
        """Generuje strefy zakazane unikając budynków i innych stref"""
        zones = []
        zone_rects = []  # Lista prostokątów stref do sprawdzania kolizji
        building_rects = []

        # Konwertuj budynki na prostokąty dla sprawdzania kolizji
        for building in existing_buildings:
            bbox = building.get_bounding_box()
            building_rect = Rectangle(bbox[0], bbox[1],
                                      bbox[2] - bbox[0], bbox[3] - bbox[1])
            building_rects.append(building_rect)

        attempts = 0
        max_attempts = count * 100  # WIĘCEJ prób
        min_distance_between_zones = 12  # MNIEJSZY dystans

        margin = 60  # MNIEJSZY margines od krawędzi

        print(f"Generowanie {count} stref zakazanych bez nakładania się...")

        while len(zones) < count and attempts < max_attempts:
            attempts += 1

            center_x = random.uniform(margin, self.width - margin)
            center_y = random.uniform(margin, self.height - margin)

            # MNIEJSZY rozmiar strefy
            points = []
            num_points = random.randint(5, 8)
            base_radius = random.uniform(25, 45)

            for i in range(num_points):
                angle = 2 * math.pi * i / num_points + random.uniform(-0.3, 0.3)
                radius = base_radius * random.uniform(0.6, 1.4)
                x = center_x + radius * math.cos(angle)
                y = center_y + radius * math.sin(angle)
                x = max(40, min(self.width - 40, x))
                y = max(40, min(self.height - 40, y))
                points.append(Point(x, y))

            zone = Polygon(points)
            zone_bbox = zone.get_bounding_box()
            zone_rect = Rectangle(zone_bbox[0], zone_bbox[1],
                                  zone_bbox[2] - zone_bbox[0],
                                  zone_bbox[3] - zone_bbox[1])

            # Sprawdź kolizje z budynkami
            collision_with_buildings = False
            for building_rect in building_rects:
                if self._rectangles_too_close(zone_rect, building_rect, 20):
                    collision_with_buildings = True
                    break

            # Sprawdź kolizje z innymi strefami zakazanymi
            collision_with_zones = False
            for existing_zone_rect in zone_rects:
                if self._rectangles_too_close(zone_rect, existing_zone_rect, min_distance_between_zones):
                    collision_with_zones = True
                    break

            # Dodaj strefę tylko jeśli nie ma kolizji
            if not collision_with_buildings and not collision_with_zones:
                zones.append(zone)
                zone_rects.append(zone_rect)

                if len(zones) % 2 == 0:
                    print(f"Wygenerowano {len(zones)}/{count} stref zakazanych...")

        print(f"Pomyślnie wygenerowano {len(zones)} stref zakazanych w {attempts} próbach")
        return zones

    def generate_rectangular_forbidden_zones(self, count, existing_buildings):
        """Alternatywna metoda: generuje prostokątne strefy zakazane"""
        zones = []
        zone_rects = []
        building_rects = []

        # Konwertuj budynki na prostokąty
        for building in existing_buildings:
            bbox = building.get_bounding_box()
            building_rect = Rectangle(bbox[0], bbox[1],
                                      bbox[2] - bbox[0], bbox[3] - bbox[1])
            building_rects.append(building_rect)

        attempts = 0
        max_attempts = count * 8
        min_distance = 30  # Minimalna odległość między strefami

        print(f"Generowanie {count} prostokątnych stref zakazanych...")

        while len(zones) < count and attempts < max_attempts:
            attempts += 1

            # Losowe wymiary strefy zakazanej
            width = random.randint(60, 120)
            height = random.randint(60, 120)

            # Losowa pozycja
            margin = 50
            max_x = self.width - width - margin
            max_y = self.height - height - margin

            if max_x <= margin or max_y <= margin:
                continue

            x = random.randint(margin, max_x)
            y = random.randint(margin, max_y)

            # Stwórz prostokątną strefę
            zone_rect = Rectangle(x, y, width, height)

            # Sprawdź kolizje z budynkami
            collision_with_buildings = False
            for building_rect in building_rects:
                if self._rectangles_too_close(zone_rect, building_rect, 25):
                    collision_with_buildings = True
                    break

            # Sprawdź kolizje z innymi strefami
            collision_with_zones = False
            for existing_zone_rect in zone_rects:
                if self._rectangles_too_close(zone_rect, existing_zone_rect, min_distance):
                    collision_with_zones = True
                    break

            if not collision_with_buildings and not collision_with_zones:
                # Konwertuj prostokąt na wielokąt
                zone_polygon = zone_rect.to_polygon()
                zones.append(zone_polygon)
                zone_rects.append(zone_rect)

        print(f"Wygenerowano {len(zones)} prostokątnych stref zakazanych")
        return zones

    def generate_circular_forbidden_zones(self, count, existing_buildings):
        """Alternatywna metoda: generuje okrągłe strefy zakazane"""
        zones = []
        zone_circles = []  # Lista (center_x, center_y, radius)
        building_rects = []

        # Konwertuj budynki na prostokąty
        for building in existing_buildings:
            bbox = building.get_bounding_box()
            building_rect = Rectangle(bbox[0], bbox[1],
                                      bbox[2] - bbox[0], bbox[3] - bbox[1])
            building_rects.append(building_rect)

        attempts = 0
        max_attempts = count * 8
        min_distance = 40  # Minimalna odległość między centrami okręgów

        print(f"Generowanie {count} okrągłych stref zakazanych...")

        while len(zones) < count and attempts < max_attempts:
            attempts += 1

            # Losowy promień i pozycja
            radius = random.randint(30, 60)
            margin = radius + 20

            center_x = random.randint(margin, self.width - margin)
            center_y = random.randint(margin, self.height - margin)

            # Sprawdź kolizje z budynkami
            collision_with_buildings = False
            zone_rect = Rectangle(center_x - radius, center_y - radius,
                                  2 * radius, 2 * radius)

            for building_rect in building_rects:
                if self._rectangles_too_close(zone_rect, building_rect, 20):
                    collision_with_buildings = True
                    break

            # Sprawdź kolizje z innymi okręgami
            collision_with_zones = False
            for existing_x, existing_y, existing_radius in zone_circles:
                distance = math.sqrt((center_x - existing_x) ** 2 + (center_y - existing_y) ** 2)
                if distance < (radius + existing_radius + min_distance):
                    collision_with_zones = True
                    break

            if not collision_with_buildings and not collision_with_zones:
                # Stwórz wielokąt aproksymujący okrąg
                points = []
                num_points = 16  # 16 punktów dla gładkiego okręgu

                for i in range(num_points):
                    angle = 2 * math.pi * i / num_points
                    x = center_x + radius * math.cos(angle)
                    y = center_y + radius * math.sin(angle)
                    points.append(Point(x, y))

                zone = Polygon(points)
                zones.append(zone)
                zone_circles.append((center_x, center_y, radius))

        print(f"Wygenerowano {len(zones)} okrągłych stref zakazanych")
        return zones

    def generate_grid_buildings(self, count):
        """Alternatywna metoda: generuje budynki na siatce"""
        buildings = []

        # Parametry siatki
        grid_size = 150  # Rozmiar komórki siatki
        building_margin = 20  # Margines wewnątrz komórki

        # Oblicz liczbę komórek w siatce
        cols = self.width // grid_size
        rows = self.height // grid_size

        # Lista dostępnych komórek
        available_cells = []
        for row in range(rows):
            for col in range(cols):
                available_cells.append((row, col))

        # Losowo wybierz komórki dla budynków
        selected_cells = random.sample(available_cells, min(count, len(available_cells)))

        for row, col in selected_cells:
            # Pozycja komórki
            cell_x = col * grid_size
            cell_y = row * grid_size

            # Rozmiar budynku w komórce
            max_building_size = grid_size - 2 * building_margin

            if random.random() < 0.4:  # 40% szans na kwadrat
                size = random.randint(max_building_size // 2, max_building_size)
                width = height = size
            else:
                width = random.randint(max_building_size // 2, max_building_size)
                height = random.randint(max_building_size // 2, max_building_size)

            # Wycentruj budynek w komórce
            x = cell_x + (grid_size - width) // 2
            y = cell_y + (grid_size - height) // 2

            # Stwórz prostokątny budynek
            rect = Rectangle(x, y, width, height)
            building = rect.to_polygon()
            buildings.append(building)

        print(f"Wygenerowano {len(buildings)} budynków na siatce")
        return buildings

    def generate_map_with_zone_type(self, building_count=20, forbidden_zone_count=5, zone_type="irregular"):
        """Generuje mapę z różnymi typami stref zakazanych"""
        map_data = MapData(self.width, self.height)

        # Generuj prostokątne budynki
        buildings = self.generate_rectangular_buildings(building_count)
        for building in buildings:
            map_data.add_building(building)

        # Generuj strefy zakazane według typu
        if zone_type == "rectangular":
            forbidden_zones = self.generate_rectangular_forbidden_zones(forbidden_zone_count, buildings)
        elif zone_type == "circular":
            forbidden_zones = self.generate_circular_forbidden_zones(forbidden_zone_count, buildings)
        else:  # "irregular"
            forbidden_zones = self.generate_forbidden_zones(forbidden_zone_count, buildings)

        for zone in forbidden_zones:
            map_data.add_forbidden_zone(zone)

        return map_data

    def generate_test_scenario(self, scenario_type="urban"):
        """Generuje predefiniowane scenariusze testowe"""
        if scenario_type == "urban":
            return self.generate_map_with_zone_type(building_count=25, forbidden_zone_count=8, zone_type="irregular")
        elif scenario_type == "sparse":
            return self.generate_map_with_zone_type(building_count=10, forbidden_zone_count=3, zone_type="circular")
        elif scenario_type == "dense":
            return self.generate_map_with_zone_type(building_count=40, forbidden_zone_count=15, zone_type="rectangular")
        elif scenario_type == "grid":
            # Użyj metody siatki dla regularnego układu
            map_data = MapData(self.width, self.height)
            buildings = self.generate_grid_buildings(20)
            for building in buildings:
                map_data.add_building(building)

            forbidden_zones = self.generate_rectangular_forbidden_zones(8, buildings)
            for zone in forbidden_zones:
                map_data.add_forbidden_zone(zone)

            return map_data
        else:
            return self.generate_map_with_zone_type()

    def generate_random_points(self, count):
        """Generuje losowe punkty na mapie"""
        points = []
        for _ in range(count):
            x = random.uniform(0, self.width)
            y = random.uniform(0, self.height)
            points.append(Point(x, y))
        return points
