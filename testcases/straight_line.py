import numpy as np
from simulator.type import Position, Orientation

spawn_point = Position(0, 2)
spawn_ori = Orientation(0, 0, 0)
checkpoint = np.array([np.linspace(0, 500, num=50), np.linspace(2, 2, num=50)])
checkpoint_orientation = np.arctan2(checkpoint[1, 1:] - checkpoint[1, 0:-1],
                                    checkpoint[0, 1:] - checkpoint[0, 0:-1])
