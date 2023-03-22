import numpy as np
import math
from simulator.type import Position, Orientation


def generate_circle(diameter=200):
    perimeter = math.pi * diameter
    number_pts = int(perimeter)
    rad_interval = 2*math.pi/number_pts
    rad_list = np.array([rad_interval * i for i in range(number_pts)])
    checkpoint_x = np.cos(rad_list) * diameter / 2
    checkpoint_y = np.sin(rad_list) * diameter / 2
    return np.array([checkpoint_x, checkpoint_y])


checkpoint = generate_circle()
checkpoint_orientation = np.arctan2(checkpoint[1, 1:] - checkpoint[1, 0:-1],
                                    checkpoint[0, 1:] - checkpoint[0, 0:-1])
spawn_point = Position(checkpoint[0, 0], checkpoint[1, 0])
spawn_ori = Orientation(0, 0, checkpoint_orientation[0])
