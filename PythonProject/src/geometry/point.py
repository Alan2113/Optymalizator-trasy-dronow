import math


class Point:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)

    def distance_to(self, other):
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def __eq__(self, other):
        return abs(self.x - other.x) < 1e-9 and abs(self.y - other.y) < 1e-9

    def __repr__(self):
        return f"Point({self.x}, {self.y})"

    def __hash__(self):
        return hash((round(self.x, 6), round(self.y, 6)))

    def translate(self, dx, dy):
        """Translacja punktu o wektor (dx, dy)"""
        return Point(self.x + dx, self.y + dy)

    def rotate(self, angle, center=None):
        """Obrót punktu wokół centrum"""
        if center is None:
            center = Point(0, 0)

        cos_a = math.cos(angle)
        sin_a = math.sin(angle)

        # Translacja do początku układu
        x = self.x - center.x
        y = self.y - center.y

        # Obrót
        new_x = x * cos_a - y * sin_a
        new_y = x * sin_a + y * cos_a

        # Translacja z powrotem
        return Point(new_x + center.x, new_y + center.y)
