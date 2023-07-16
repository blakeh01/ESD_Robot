import numpy as np


class AlignmentPoint:

    def __init__(self, world_pos, surface_normal):
        self.world_pos = world_pos
        self.surface_normal = surface_normal

    def __str__(self):
        return f"pos: {str(self.world_pos)}, facing: {self.surface_normal}"