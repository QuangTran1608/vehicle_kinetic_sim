import math
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

class Position():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def update(self, x=0, y=0):
        self.x = x
        self.y = y

class Orientation():
    def __init__(self, roll=0, pitch=0, yaw=0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def update(self, roll=0, pitch=0, yaw=0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

class BoundingBox():
    # 1-------------2
    # |             |
    # |      o      |
    # |             |
    # 4-------------3
    #
    # AXIS:
    # 
    # ^ y
    # |
    # |
    # |
    # |
    # |
    # .----------------> x
    def __init__(self, pos, orientation, y, x):
        self.x_length = x
        self.y_length = y
        self.pts = self.calc_pts(pos, orientation)

    def calc_pts(self, pos, orientation):
        sin_yaw = math.sin(orientation.yaw)
        cos_yaw = math.cos(orientation.yaw)

        x_length_offset_x = self.x_length * cos_yaw / 2
        y_length_offset_x = self.y_length * sin_yaw / 2

        x_length_offset_y = self.x_length * sin_yaw / 2
        y_length_offset_y = self.y_length * cos_yaw / 2
        pos_1 = Position(x = pos.x - x_length_offset_x - y_length_offset_x, 
                         y = pos.y - x_length_offset_y + y_length_offset_y)
        pos_2 = Position(x = pos.x + x_length_offset_x - y_length_offset_x, 
                         y = pos.y + x_length_offset_y + y_length_offset_y)
        pos_3 = Position(x = pos.x + x_length_offset_x + y_length_offset_x, 
                         y = pos.y + x_length_offset_y - y_length_offset_y)
        pos_4 = Position(x = pos.x - x_length_offset_x + y_length_offset_x, 
                         y = pos.y - x_length_offset_y - y_length_offset_y)
        return pos_1, pos_2, pos_3, pos_4
    
    def update(self, pos, orientation):
        self.pts = self.calc_pts(pos, orientation)

class BoxObject():
    def __init__(self, width, length, pos=Position(0,0), orientation=Orientation(0,0,0)):
        super().__init__()
        self.width = width
        self.length = length
        self.pos = pos
        self.orientation = orientation
        self.box = BoundingBox(pos, orientation, width, length)

    def update_pos(self, pos, orientation):
        self.pos = pos
        self.orientation = orientation
        self.box.update(pos, orientation)

    def unwind_pos(self):
        return np.array([point.x for point in self.box.pts]), \
               np.array([point.y for point in self.box.pts])

    def get_BBox(self):
        return self.unwind_pos()
