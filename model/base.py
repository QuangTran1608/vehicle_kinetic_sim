import math
from simulator.type import *


class BaseModel():
    def __init__(self, pos=Position(0,0), orientation=Orientation(0,0,0), delta_time=0.1):
        self.pos = pos
        self.orientation = orientation
        self.velocity = 0
        self.yaw_rate = 0
        self.delta_time = delta_time
        self.x_length = 4
        self.y_length = 4
        self.bbox = BoxObject(self.y_length, self.x_length, self.pos, self.orientation)

    def apply_throttle(self, throttle):
        self.velocity += throttle

    def apply_steer(self, steering_angle):
        self.yaw_rate += steering_angle

    def step(self):
        # Changes in yaw_rate and velocity are not considered
        x_change_rate = self.velocity * math.cos(self.orientation.yaw) * self.delta_time
        y_change_rate = self.velocity * math.sin(self.orientation.yaw) * self.delta_time
        self.pos.update(self.pos.x + x_change_rate, self.pos.y + y_change_rate)

        # For simplicity, yaw_rate and orientation uses the same type. This is not true in real life
        self.orientation.update(yaw = self.orientation.yaw + self.yaw_rate * self.delta_time)
                                
        self.bbox.update_pos(self.pos, self.orientation)