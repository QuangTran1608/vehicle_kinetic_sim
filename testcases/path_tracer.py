import math
import numpy as np
from model import LegoConfig as lc
from constants import HALF_LANE_WIDTH
from simulator.type import Position, Rotation, Transform

TEST_DATA = "test_data.txt"
DRIVE_DIR = -1


def split(line):
    translation_tbl = str.maketrans('', '', '() ')
    numbers = line.translate(translation_tbl).strip().split(',')
    numbers = [int(number) for number in numbers]
    timestamp = numbers[0:1]        # (time) sec
    pitch_roll_yaw = numbers[1:4]   # (x, y, z) deg
    gyroscope = numbers[4:7]        # (x, y, z) deg/s
    accelerations = numbers[7:10]   # (x, y, z) cm/s^2
    steer_encoder = numbers[10:14]  # (speed_pct, rel_pos, abs_pos, pwm)
    drive_encoder = numbers[14:18]  # (speed_pct, rel_pos, abs_pos, pwm)
    return timestamp, pitch_roll_yaw, gyroscope, accelerations, \
        steer_encoder, drive_encoder


def drive_decode(prev, curr):
    return DRIVE_DIR * (curr[1] - prev[1]) * lc.tire_height * math.pi / 360


def next_position(x, y, yaw, distance):
    nx = x + distance * math.cos(yaw)
    ny = y + distance * math.sin(yaw)
    return nx, ny


ryaw, rx, ry = [], [], []
with open(TEST_DATA, 'r') as f:
    line = f.readline()
    assert line

    p_timestamp, p_pitch_roll_yaw, _, _, _, p_drive_encoder = split(line)
    p_yaw = np.radians(p_pitch_roll_yaw[2])

    ryaw.append(p_yaw)
    rx.append(0.)
    ry.append(0.)

    skip = False
    while True:
        skip = ~skip
        if skip:
            continue

        line = f.readline()
        if not line:
            break

        timestamp, pitch_roll_yaw, _, _, _, drive_encoder = split(line)
        yaw = np.radians(pitch_roll_yaw[2])

        distance = drive_decode(p_drive_encoder, drive_encoder)
        nx, ny = next_position(rx[-1], ry[-1], yaw, distance)

        ryaw.append(yaw)
        rx.append(nx)
        ry.append(ny)

        p_pitch_roll_yaw = pitch_roll_yaw
        p_drive_encoder = drive_encoder

checkpoint = np.array([np.asarray(rx), np.asarray(ry)])
checkpoint_orientation = np.asarray(ryaw)
spawn_position = Position(rx[0], ry[0])
spawn_rotation = Rotation(yaw=ryaw[0])

slen = len(ryaw)
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
