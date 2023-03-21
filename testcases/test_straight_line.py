import numpy as np
import math
from simulator.type import Position, Orientation

spawn_point = Position(0, 2)
spawn_ori = Orientation(0, 0, 0.002)
xlim = (-5, 60)
ylim = (-2, 63)
x_1 = np.linspace(0, 50, num=2)
x_2 = np.linspace(0, 50, num=2)
y_1 = np.linspace(0, 0, num=2)
y_2 = np.linspace(4, 4, num=2)
checkpoint = np.array([np.linspace(0, 50, num=50), np.linspace(2, 2, num=50)])
checkpoint_orientation = np.arctan2(checkpoint[1, 1:] - checkpoint[1, 0:-1], 
                                    checkpoint[0, 1:] - checkpoint[0, 0:-1])
