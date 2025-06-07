from .point import Point
import random
import math


class Triangle:
    def __init__(self, p1, p2, p3):
        self.p1, self.p2, self.p3 = p1, p2, p3
        self.circumcenter, self.circumradius = self._calculate_circumcircle()
    def _calculate_circumcircle(self):
        """Oblicza środek i promień okręgu opisanego"""
        ax, ay = self.p1.x, self.p1.y
        bx, by = self.p2.x, self.p2.y
        cx, cy = self.p3.x, self.p3.y

        d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
        if abs(d) < 1e-10:
            return Point(0, 0), float('inf')

        ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d
        uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d

        center = Point(ux, uy)
        radius = center.distance_to(self.p1)

        return center, radius

    def contains_point_in_circumcircle(self, point):
        """Sprawdza czy punkt jest wewnątrz okręgu opisanego"""
        return self.circumcenter.distance_to(point) < self.circumradius

    def area(self):
        """Oblicza pole trójkąta"""
        return abs((self.p2.x - self.p1.x) * (self.p3.y - self.p1.y) -
                   (self.p3.x - self.p1.x) * (self.p2.y - self.p1.y)) / 2

    def centroid(self):
        """Zwraca centroid (środek ciężkości) trójkąta."""
        x = (self.p1.x + self.p2.x + self.p3.x) / 3
        y = (self.p1.y + self.p2.y + self.p3.y) / 3
        return Point(x, y)



class DelaunayTriangulation:
    def __init__(self, points):
        self.points = points
        self.triangles = []

    def triangulate(self):
        """Bowyer-Watson algorithm - z zajęć 6"""
        if len(self.points) < 3:
            return []

        # Stwórz super-triangle
        super_triangle = self._create_super_triangle()
        self.triangles = [super_triangle]

        # Dodaj punkty jeden po drugim
        for point in self.points:
            bad_triangles = []

            # Znajdź triangles których circumcircle zawiera punkt
            for triangle in self.triangles:
                if triangle.contains_point_in_circumcircle(point):
                    bad_triangles.append(triangle)

            # Znajdź boundary polygon
            polygon = []
            for triangle in bad_triangles:
                edges = [(triangle.p1, triangle.p2), (triangle.p2, triangle.p3), (triangle.p3, triangle.p1)]
                for edge in edges:
                    shared = False
                    for other_triangle in bad_triangles:
                        if other_triangle != triangle:
                            other_edges = [(other_triangle.p1, other_triangle.p2),
                                           (other_triangle.p2, other_triangle.p3),
                                           (other_triangle.p3, other_triangle.p1)]
                            if edge in other_edges or (edge[1], edge[0]) in other_edges:
                                shared = True
                                break
                    if not shared:
                        polygon.append(edge)

            # Usuń bad triangles
            for triangle in bad_triangles:
                self.triangles.remove(triangle)

            # Dodaj nowe triangles
            for edge in polygon:
                new_triangle = Triangle(edge[0], edge[1], point)
                self.triangles.append(new_triangle)

        # Usuń triangles zawierające punkty super-triangle
        super_points = {super_triangle.p1, super_triangle.p2, super_triangle.p3}
        self.triangles = [t for t in self.triangles
                          if not (t.p1 in super_points or t.p2 in super_points or t.p3 in super_points)]

        return self.triangles

    def _create_super_triangle(self):
        """Tworzy super-triangle zawierający wszystkie punkty"""
        min_x = min(p.x for p in self.points)
        max_x = max(p.x for p in self.points)
        min_y = min(p.y for p in self.points)
        max_y = max(p.y for p in self.points)

        dx = max_x - min_x
        dy = max_y - min_y
        delta_max = max(dx, dy)

        mid_x = (min_x + max_x) / 2
        mid_y = (min_y + max_y) / 2

        p1 = Point(mid_x - 20 * delta_max, mid_y - delta_max)
        p2 = Point(mid_x, mid_y + 20 * delta_max)
        p3 = Point(mid_x + 20 * delta_max, mid_y - delta_max)

        return Triangle(p1, p2, p3)
