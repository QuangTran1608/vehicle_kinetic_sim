import numpy as np
from constants import HALF_LANE_WIDTH
from simulator.type import Position, Rotation


spawn_position = Position(0, 2)
spawn_rotation = Rotation(0, 0, 0)
x_1 = np.linspace(0, 500, num=2)
x_2 = np.linspace(0, 500, num=2)
y_1 = np.linspace(-HALF_LANE_WIDTH, -HALF_LANE_WIDTH, num=2)
y_2 = np.linspace(+HALF_LANE_WIDTH, +HALF_LANE_WIDTH, num=2)
checkpoint = np.array([
   np.linspace(0, 500, num=50),
   np.linspace(0, 0, num=50)
])
checkpoint_orientation = np.arctan2(checkpoint[1, 1:] - checkpoint[1, 0:-1],
                                    checkpoint[0, 1:] - checkpoint[0, 0:-1])
