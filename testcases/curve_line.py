import numpy as np
from constants import HALF_LANE_WIDTH
from simulator.type import Position, Rotation, Transform
from common.cubic_spline.cubic_spline_planner import Spline2D

x = [-2.5, +0.0, +2.5, +5.0]
y = [+0.5, -5.0, +5.0, +6.5]
ds = 0.1  # [m] distance of each interpolated points

x = [xi * 30 for xi in x]
y = [yi * 30 for yi in y]

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
spawn_position = Position(x[0], y[0])
spawn_rotation = Rotation(yaw=ryaw[1])

slen = len(s)
x_1 = np.zeros(slen)
x_2 = np.zeros(slen)
y_1 = np.zeros(slen)
y_2 = np.zeros(slen)
for i in range(slen):
    trans = Transform(Position(rx[i], ry[i]), Rotation(yaw=ryaw[i]))
    ltrans = trans.translate(0.0, -HALF_LANE_WIDTH, in_place=False)
    rtrans = trans.translate(0.0, +HALF_LANE_WIDTH, in_place=False)
    x_1[i] = ltrans.position.x
    x_2[i] = rtrans.position.x
    y_1[i] = ltrans.position.y
    y_2[i] = rtrans.position.y
