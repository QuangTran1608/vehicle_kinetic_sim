import math
from .base import BaseModel
from simulator.type import *
import common.maths.functions as func

class X3PlusConfig():
    maxspeed = 70 #cm/s
    maxyawrate = math.pi #rad/s
    x_length = 27.94 #cm
    y_length = 20

# Simulate Lego model base on used API and bicyle model
class X3PlusModel(BaseModel):
    def __init__(self, config=X3PlusConfig(), position=Position(0,0), rotation=Rotation(0,0,0), delta_time=0.1):
        super().__init__(position, rotation)
        self.physic = config
        self.delta_time = delta_time
        self.x_length = self.physic.x_length
        self.y_length = self.physic.y_length
        self.bbox = BoxObject(self.y_length, self.x_length, self.position, self.rotation)

    def get_current_states(self):
        return (self.bbox.position.x, 
                self.bbox.position.y, 
                self.bbox.rotation.yaw, 
                self.velocity, 
                self.yaw_rate)

    def apply_throttle(self, throttle):
        self.velocity = throttle * self.physic.maxspeed

    def apply_steer(self, steering_angle):
        self.yaw_rate = steering_angle * self.physic.maxyawrate

    def simulate_next_step(self, x, y, yaw, velocity, yaw_rate, throttle, steering_angle):
        # For simplicity, yaw_rate and orientation uses the same type. This is not true in real life
        new_velocity = throttle * self.physic.maxspeed
        yaw_rate = steering_angle * self.physic.maxyawrate / 2
        new_yaw = func.norm_to_range(yaw + yaw_rate * self.delta_time)

        x_change_rate = velocity * math.cos(yaw) * self.delta_time/2 + new_velocity * math.cos(yaw) * self.delta_time/2
        y_change_rate = velocity * math.sin(yaw) * self.delta_time/2 + new_velocity * math.sin(yaw) * self.delta_time/2
        new_x = x + x_change_rate
        new_y = y + y_change_rate
        return new_x, new_y, new_yaw, new_velocity, yaw_rate
