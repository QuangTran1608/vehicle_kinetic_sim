from hub import motion, sound
from projects.mpy_robot_tools.rc import RCReceiver, R_STICK_VER, L_STICK_HOR, SETTING1, L_TRIGGER
from projects.mpy_robot_tools.helpers import PBMotor as Motor
from projects.mpy_robot_tools.helpers import Port, wait
from math import pi as PI
from time import time as now

rcv = RCReceiver(name="robot")

steer_motor = Motor(Port.A)
drive_motor = Motor(Port.B)

raw_steer_motor = steer_motor.control.motor
raw_drive_motor = drive_motor.control.motor

STEER_DIR = -1
DRIVE_DIR = -1

STEER_MOTOR_GEAR = 12  # teeth
STEER_OUTPUT_GEAR = 20  # teeth

CONST_SPEED = 10  # cm/s
WHEEL_DIAMETER = 5.6  # cm
DRIVE_MOTOR_GEAR = 20  # teeth
DRIVE_OUTPUT_GEAR = 28  # teeth
ANGULAR_SPEED = 360 * DRIVE_OUTPUT_GEAR * CONST_SPEED / (DRIVE_MOTOR_GEAR * WHEEL_DIAMETER * PI)  # deg/s

DATA_LOG = "test_data_{}.txt".format(now())
while 1:
    if rcv.is_connected():
        steer_target, speed_target, trim, thumb = rcv.controller_state(L_STICK_HOR, R_STICK_VER, SETTING1, L_TRIGGER)
    else:
        steer_target, speed_target, trim, thumb = (0, 0, 0, 0)

    steer_motor.track_target(STEER_DIR * steer_target * STEER_OUTPUT_GEAR / STEER_MOTOR_GEAR + trim)
    # drive_motor.dc(DRIVE_DIR * speed_target)
    sign = 1 if 0 < speed_target else (-1 if speed_target < 0 else 0)
    drive_motor.run(DRIVE_DIR * ANGULAR_SPEED * sign)
    if 0 != sign:
        yaw, pitch, roll = motion.yaw_pitch_roll()
        pitch_roll_yaw = [pitch, yaw, roll]     # [x, y, z] deg
        rates = motion.gyroscope()              # [x, y, z] deg/s
        accelerations = motion.accelerometer()  # [x, y, z] cm/s^2
        steer_encoder = raw_steer_motor.get()   # [speed_pct, rel_pos, abs_pos, pwm]
        drive_encoder = raw_drive_motor.get()   # [speed_pct, rel_pos, abs_pos, pwm]
        with open(DATA_LOG, 'a') as f:
            f.write(f"{pitch_roll_yaw},{accelerations},{steer_encoder},{drive_encoder}\n")

    if thumb > 50:
        sound.beep(400, 25)
        wait(20)
