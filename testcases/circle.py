import numpy as np
import math
from simulator.type import Position, Orientation

def generate_circle(diameter=200):
    perimeter = math.pi * diameter
    number_pts = int(perimeter)
    rad_interval = 2*math.pi/number_pts
    rad_list = np.array([rad_interval * i for i in range(number_pts)])
    inner_diameter = diameter - 20
    outter_diameter = diameter + 20
    x_1 = np.cos(rad_list) * inner_diameter / 2
    x_2 = np.cos(rad_list) * outter_diameter / 2
    y_1 = np.sin(rad_list) * inner_diameter / 2
    y_2 = np.sin(rad_list) * outter_diameter / 2
    checkpoint_x = np.cos(rad_list) * diameter / 2
    checkpoint_y = np.sin(rad_list) * diameter / 2
    return x_1, x_2, y_1, y_2, np.array([checkpoint_x, checkpoint_y])

x_1, x_2, y_1, y_2, checkpoint = generate_circle()
checkpoint_orientation = np.arctan2(checkpoint[1, 1:] - checkpoint[1, 0:-1], 
                                    checkpoint[0, 1:] - checkpoint[0, 0:-1])
spawn_point = Position(checkpoint[0, 0], checkpoint[1, 0])
spawn_ori = Orientation(0, 0, checkpoint_orientation[0])
