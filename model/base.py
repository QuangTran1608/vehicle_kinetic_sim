import math
from simulator.type import *
import common.maths.functions as func


class BaseModel():
    def __init__(self, position=Position(0,0), rotation=Rotation(0,0,0), delta_time=0.1):
        self.position = position
        self.rotation = rotation
        self.velocity = 0
        self.yaw_rate = 0
        self.delta_time = delta_time
        self.x_length = 4
        self.y_length = 4
        self.bbox = BoxObject(self.y_length, self.x_length, self.position, self.rotation)

    def apply_throttle(self, throttle):
        self.velocity += throttle

    def apply_steer(self, steering_angle):
        self.yaw_rate += steering_angle

    def step(self):
        # Changes in yaw_rate and velocity are not considered
        x_change_rate = self.velocity * math.cos(self.rotation.yaw) * self.delta_time
        y_change_rate = self.velocity * math.sin(self.rotation.yaw) * self.delta_time
        self.position.update(self.position.x + x_change_rate, self.position.y + y_change_rate)

        # For simplicity, yaw_rate and orientation uses the same type. This is not true in real life
        new_yaw = func.norm_to_range(self.rotation.yaw + self.yaw_rate * self.delta_time)
        self.rotation.update(yaw=new_yaw)
                                
        self.bbox.update_pos(self.position, self.rotation)