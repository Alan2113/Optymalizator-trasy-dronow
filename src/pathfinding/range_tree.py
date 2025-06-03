from geometry.point import Point


class RangeTreeNode:
    def __init__(self, point=None):
        self.point = point
        self.left = None
        self.right = None
        self.y_tree = None


class RangeTree2D:
    def __init__(self, points):
        """Drzewo zakresowe 2D - z zajęć 4"""
        self.points = sorted(points, key=lambda p: p.x) if points else []
        self.root = self._build_x_tree(self.points) if self.points else None

    def _build_x_tree(self, points):
        """Buduje drzewo zakresowe dla współrzędnej X"""
        if not points:
            return None

        if len(points) == 1:
            node = RangeTreeNode(points[0])
            node.y_tree = self._build_y_tree([points[0]])
            return node

        mid = len(points) // 2
        node = RangeTreeNode()
        node.left = self._build_x_tree(points[:mid])
        node.right = self._build_x_tree(points[mid:])
        node.y_tree = self._build_y_tree(points)

        return node

    def _build_y_tree(self, points):
        """Buduje drzewo pomocnicze dla współrzędnej Y"""
        return sorted(points, key=lambda p: p.y)

    def query_range(self, x_min, x_max, y_min, y_max):
        """Wyszukuje punkty w prostokątnym zakresie"""
        if not self.root:
            return []

        result = []
        self._query_x_range(self.root, x_min, x_max, y_min, y_max, result)
        return result

    def _query_x_range(self, node, x_min, x_max, y_min, y_max, result):
        """Rekurencyjne wyszukiwanie w zakresie X"""
        if not node:
            return

        if node.point:  # Liść
            if (x_min <= node.point.x <= x_max and
                    y_min <= node.point.y <= y_max):
                result.append(node.point)
            return

        # Sprawdź które poddrzewa mogą zawierać punkty w zakresie
        if node.left:
            left_points = self._get_all_points(node.left)
            if left_points and max(p.x for p in left_points) >= x_min:
                self._query_x_range(node.left, x_min, x_max, y_min, y_max, result)

        if node.right:
            right_points = self._get_all_points(node.right)
            if right_points and min(p.x for p in right_points) <= x_max:
                self._query_x_range(node.right, x_min, x_max, y_min, y_max, result)

    def _get_all_points(self, node):
        """Pomocnicza funkcja do zbierania wszystkich punktów z poddrzewa"""
        if not node:
            return []
        if node.point:
            return [node.point]

        points = []
        if node.left:
            points.extend(self._get_all_points(node.left))
        if node.right:
            points.extend(self._get_all_points(node.right))
        return points

    def find_nearest_neighbors(self, query_point, radius):
        """Znajduje najbliższych sąsiadów w promieniu"""
        x_min = query_point.x - radius
        x_max = query_point.x + radius
        y_min = query_point.y - radius
        y_max = query_point.y + radius

        candidates = self.query_range(x_min, x_max, y_min, y_max)

        # Filtruj punkty w rzeczywistym promieniu (okrąg, nie kwadrat)
        neighbors = []
        for point in candidates:
            if query_point.distance_to(point) <= radius:
                neighbors.append(point)

        return neighbors
