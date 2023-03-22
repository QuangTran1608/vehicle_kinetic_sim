import numpy as np
import math
from constants import HALF_LANE_WIDTH
from simulator.type import Position, Rotation


def generate_circle(radius=100):
    perimeter = math.pi * radius * 2
    number_pts = int(perimeter)
    rad_interval = 2*math.pi/number_pts
    rad_list = np.array([rad_interval * i for i in range(number_pts)])
    inner_radius = radius - HALF_LANE_WIDTH
    outter_radius = radius + HALF_LANE_WIDTH
    x_1 = np.cos(rad_list) * inner_radius
    x_2 = np.cos(rad_list) * outter_radius
    y_1 = np.sin(rad_list) * inner_radius
    y_2 = np.sin(rad_list) * outter_radius
    checkpoint_x = np.cos(rad_list) * radius
    checkpoint_y = np.sin(rad_list) * radius
    return x_1, x_2, y_1, y_2, np.array([checkpoint_x, checkpoint_y])


x_1, x_2, y_1, y_2, checkpoint = generate_circle()
checkpoint_orientation = np.arctan2(checkpoint[1, 1:] - checkpoint[1, 0:-1],
                                    checkpoint[0, 1:] - checkpoint[0, 0:-1])
spawn_position = Position(checkpoint[0, 0], checkpoint[1, 0])
spawn_rotation = Rotation(0, 0, checkpoint_orientation[0])
