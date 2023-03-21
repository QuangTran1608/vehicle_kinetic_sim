import math
from simulator.type import *

class BaseModel():
    def __init__(self, pos=Position(0,0), orientation=Orientation(0,0,0), delta_time=0.1):
        self.pos = pos
        self.orientation = orientation
        self.velocity = 0
        self.yaw_rate = 0
        self.delta_time = delta_time
        # LENGTH is x, WIDTH is y. These are used when calculated 
        self.x_length = 4
        self.y_length = 4
        self.bbox = BoxObject(self.y_length, self.x_length, pos, orientation)

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

class LegoConfig():
    wheelbase= 16.8 #cm
    max_steer= 0.78539 #rad
    axle_width= 13.6 #cm
    tire_width= 1.4 #cm
    tire_height= 5.6 #cm
    max_speed = 58.433623356770156 # cm/s

# Simulate Lego model base on used API and bicyle model
class LegoModel(BaseModel):
    def __init__(self, config=LegoConfig(), pos=Position(0,0), orientation=Orientation(0,0,0), delta_time=0.1):
        super().__init__(pos, orientation)
        self.physic = config
        self.delta_time = delta_time

    def apply_throttle(self, throttle):
        accel = throttle*self.physic.max_speed - self.velocity
        self.velocity *= 0.98
        self.velocity = min(self.velocity + self.delta_time * accel, self.physic.max_speed)

    def apply_steer(self, steering_angle):
        steering_angle = min(max(steering_angle, -self.physic.max_steer), self.physic.max_steer)
        self.yaw_rate = self.velocity*math.tan(steering_angle) / self.physic.wheelbase
