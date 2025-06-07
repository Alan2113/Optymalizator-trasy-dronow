class MapData:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.buildings = []
        self.forbidden_zones = []
        self.air_corridors = []
        self.safe_points = []

    def add_building(self, building):
        """Dodaje budynek do mapy"""
        self.buildings.append(building)

    def add_forbidden_zone(self, zone):
        """Dodaje strefę zakazaną do mapy"""
        self.forbidden_zones.append(zone)

    def get_obstacles(self):
        """Zwraca wszystkie przeszkody (budynki + strefy zakazane)"""
        return self.buildings + self.forbidden_zones

    def get_all_obstacle_points(self):
        """Zwraca wszystkie punkty z przeszkód"""
        points = []
        for obstacle in self.get_obstacles():
            points.extend(obstacle.points)
        return points

    def is_point_safe(self, point):
        """Sprawdza czy punkt jest bezpieczny (nie w przeszkodzie)"""
        for obstacle in self.get_obstacles():
            if obstacle.contains_point(point):
                return False
        return True

    def get_map_bounds(self):
        """Zwraca granice mapy"""
        return (0, 0, self.width, self.height)
