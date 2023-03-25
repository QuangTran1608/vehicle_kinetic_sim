import math
from .base import BaseModel
from simulator.type import *
import common.maths.functions as func

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
    def __init__(self, config=LegoConfig(), position=Position(0,0), rotation=Rotation(0,0,0), delta_time=0.1):
        super().__init__(position, rotation)
        self.physic = config
        self.delta_time = delta_time
        self.x_length = self.physic.length
        self.y_length = self.physic.wheelbase
        self.bbox = BoxObject(self.y_length, self.x_length, self.position, self.rotation)

    def get_current_states(self):
        return (self.bbox.position.x, 
                self.bbox.position.y, 
                self.bbox.rotation.yaw, 
                self.velocity, 
                self.yaw_rate)

    def apply_throttle(self, throttle):
        accel = throttle*self.physic.max_speed - self.velocity
        self.velocity *= 0.98
        self.velocity = min(self.velocity + self.delta_time * accel, self.physic.max_speed)

    def apply_steer(self, steering_angle):
        steering_angle = min(max(steering_angle, -self.physic.max_steer), self.physic.max_steer)
        self.yaw_rate = self.velocity*math.tan(steering_angle) / self.physic.wheelbase

    def simulate_next_step(self, x, y, yaw, velocity, yaw_rate, throttle, steering_angle):
        x_change_rate = velocity * math.cos(yaw) * self.delta_time
        y_change_rate = velocity * math.sin(yaw) * self.delta_time
        new_x = x + x_change_rate
        new_y = y + y_change_rate

        # For simplicity, yaw_rate and orientation uses the same type. This is not true in real life
        steering_angle = min(max(steering_angle, -self.physic.max_steer), self.physic.max_steer)
        yaw_rate = velocity*math.tan(steering_angle) / self.physic.wheelbase
        new_yaw = func.norm_to_range(yaw + yaw_rate * self.delta_time)

        accel = throttle*self.physic.max_speed - self.velocity
        velocity *= 0.98
        new_velocity = min(self.velocity + self.delta_time * accel, self.physic.max_speed)
        return new_x, new_y, new_yaw, new_velocity, yaw_rate
