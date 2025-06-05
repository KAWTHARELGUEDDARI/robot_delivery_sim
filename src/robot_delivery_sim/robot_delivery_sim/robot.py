class Robot:
    def __init__(self, start_pos):
        self.position = start_pos
        self.path = []

    def set_path(self, path):
        self.path = path

    def move_step(self):
        if self.path:
            self.position = self.path.pop(0)
