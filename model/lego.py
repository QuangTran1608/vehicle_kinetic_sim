import math
from .base import BaseModel
from simulator.type import *

class LegoConfig():
    wheelbase= 16.8 #cm
    max_steer= 0.78539 #rad
    axle_width= 13.6 #cm
    tire_width= 1.4 #cm
    tire_height= 5.6 #cm
    max_speed = 58.433623356770156 # cm/s
    length= 22 #cm

# Simulate Lego model base on used API and bicyle model
class LegoModel(BaseModel):
    def __init__(self, config=LegoConfig(), pos=Position(0,0), orientation=Orientation(0,0,0), delta_time=0.1):
        super().__init__(pos, orientation)
        self.physic = config
        self.delta_time = delta_time
        self.x_length = self.physic.length
        self.y_length = self.physic.wheelbase
        self.bbox = BoxObject(self.y_length, self.x_length, self.pos, self.orientation)

    def apply_throttle(self, throttle):
        accel = throttle*self.physic.max_speed - self.velocity
        self.velocity *= 0.98
        self.velocity = min(self.velocity + self.delta_time * accel, self.physic.max_speed)

    def apply_steer(self, steering_angle):
        steering_angle = min(max(steering_angle, -self.physic.max_steer), self.physic.max_steer)
        self.yaw_rate = self.velocity*math.tan(steering_angle) / self.physic.wheelbase
