import math
from simulator.type import *

class BaseModel():
    def __init__(self, pos=Position(0,0), orientation=Orientation(0,0,0)):
        self.pos = pos
        self.orientation = orientation
        self.velocity = 0
        self.yaw_rate = 0
        # LENGTH is x, WIDTH is y. These are used when calculated 
        self.x_length = 4
        self.y_length = 4
        self.bbox = BoxObject(self.y_length, self.x_length, pos, orientation)

    def update_velocity(self, new_velocity):
        self.velocity = new_velocity

    def update_yaw_rate(self, new_yaw_rate):
        self.yaw_rate = new_yaw_rate

    def update_position(self, delta_time=0.1):
        # Changes in yaw_rate and velocity are not considered
        x_change_rate = self.velocity * math.cos(self.orientation.yaw) * delta_time
        y_change_rate = self.velocity * math.sin(self.orientation.yaw) * delta_time
        self.pos.update(self.pos.x + x_change_rate, self.pos.y + y_change_rate)

        # For simplicity, yaw_rate and orientation uses the same type. This is not true in real life
        self.orientation.update(yaw = self.orientation.yaw + self.yaw_rate * delta_time)
                                
        self.bbox.update_pos(self.pos, self.orientation)
