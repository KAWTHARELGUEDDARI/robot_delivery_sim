class Grid:
    def __init__(self, width, height, obstacles=None):
        self.width = width
        self.height = height
        self.obstacles = obstacles if obstacles else set()

    def is_occupied(self, x, y):
        return (x, y) in self.obstacles

    def in_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height
