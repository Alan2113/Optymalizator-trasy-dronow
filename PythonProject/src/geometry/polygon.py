from .point import Point
import math


class Polygon:
    def __init__(self, points):
        self.points = points if isinstance(points[0], Point) else [Point(p[0], p[1]) for p in points]

    def contains_point(self, point):
        """Ray casting algorithm - z zajęć 2"""
        x, y = point.x, point.y
        n = len(self.points)
        inside = False

        p1x, p1y = self.points[0].x, self.points[0].y
        for i in range(1, n + 1):
            p2x, p2y = self.points[i % n].x, self.points[i % n].y
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside

    def area(self):
        """Shoelace formula - z zajęć 2"""
        if len(self.points) < 3:
            return 0

        area = 0
        n = len(self.points)
        for i in range(n):
            j = (i + 1) % n
            area += self.points[i].x * self.points[j].y
            area -= self.points[j].x * self.points[i].y

        return abs(area) / 2

    def get_bounding_box(self):
        """Zwraca prostokąt ograniczający"""
        min_x = min(p.x for p in self.points)
        max_x = max(p.x for p in self.points)
        min_y = min(p.y for p in self.points)
        max_y = max(p.y for p in self.points)
        return (min_x, min_y, max_x, max_y)

    def centroid(self):
        """Oblicza centroid wielokąta"""
        x = sum(p.x for p in self.points) / len(self.points)
        y = sum(p.y for p in self.points) / len(self.points)
        return Point(x, y)

    def is_point_on_edge(self, point, tolerance=1e-6):
        """Sprawdza czy punkt leży na krawędzi wielokąta"""
        for i in range(len(self.points)):
            p1 = self.points[i]
            p2 = self.points[(i + 1) % len(self.points)]

            # Sprawdź czy punkt leży na odcinku p1-p2
            if self._point_on_segment(point, p1, p2, tolerance):
                return True
        return False

    def _point_on_segment(self, point, p1, p2, tolerance):
        """Sprawdza czy punkt leży na odcinku"""
        # Sprawdź czy punkt jest współliniowy z p1 i p2
        cross_product = (point.y - p1.y) * (p2.x - p1.x) - (point.x - p1.x) * (p2.y - p1.y)
        if abs(cross_product) > tolerance:
            return False

        # Sprawdź czy punkt jest między p1 i p2
        dot_product = (point.x - p1.x) * (p2.x - p1.x) + (point.y - p1.y) * (p2.y - p1.y)
        squared_distance = (p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2

        return 0 <= dot_product <= squared_distance
