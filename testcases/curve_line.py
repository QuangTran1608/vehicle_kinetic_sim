import numpy as np
from simulator.type import Position, Orientation
from common.cubic_spline.cubic_spline_planner import Spline2D

x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0]
ds = 0.1  # [m] distance of each interpolated points

x = [xi * 50 for xi in x]
y = [yi * 50 for yi in y]

sp = Spline2D(x, y)
s = np.arange(0, sp.s[-1], ds)

rx, ry, ryaw = [], [], []
for i_s in s:
    ix, iy = sp.calc_position(i_s)
    rx.append(ix)
    ry.append(iy)
    ryaw.append(sp.calc_yaw(i_s))

checkpoint = np.array([np.asarray(rx), np.asarray(ry)])
checkpoint_orientation = np.asarray(ryaw)
spawn_point = Position(x[0], y[0])
spawn_ori = Orientation(yaw=ryaw[0])
