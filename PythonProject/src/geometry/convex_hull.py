from .point import Point
import math


class ConvexHull:
    @staticmethod
    def graham_scan(points):
        """Graham Scan - z zajęć 3"""
        if len(points) < 3:
            return points

        # Znajdź punkt z najniższą współrzędną y
        start = min(points, key=lambda p: (p.y, p.x))

        # Sortuj punkty według kąta polarnego
        def polar_angle(p):
            dx = p.x - start.x
            dy = p.y - start.y
            return math.atan2(dy, dx)

        sorted_points = sorted([p for p in points if p != start], key=polar_angle)

        # Graham scan
        hull = [start, sorted_points[0]]

        for p in sorted_points[1:]:
            while len(hull) > 1 and ConvexHull._cross_product(hull[-2], hull[-1], p) <= 0:
                hull.pop()
            hull.append(p)

        return hull

    @staticmethod
    def jarvis_march(points):
        """Jarvis March (Gift Wrapping) - z zajęć 3"""
        if len(points) < 3:
            return points

        # Znajdź leftmost point
        leftmost = min(points, key=lambda p: p.x)

        hull = []
        current = leftmost

        while True:
            hull.append(current)
            next_point = points[0]

            for p in points[1:]:
                cross = ConvexHull._cross_product(current, next_point, p)
                if next_point == current or cross > 0 or (cross == 0 and
                                                          current.distance_to(p) > current.distance_to(next_point)):
                    next_point = p

            current = next_point
            if current == leftmost:
                break

        return hull

    @staticmethod
    def _cross_product(o, a, b):
        """Oblicza iloczyn wektorowy"""
        return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
