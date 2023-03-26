from projects.mpy_robot_tools.rc import R_STICK_VER, L_STICK_HOR, SETTING1, L_TRIGGER
from projects.mpy_robot_tools.rc import RCReceiver
from projects.mpy_robot_tools.helpers import PBMotor
from projects.mpy_robot_tools.helpers import Port
from hub import motion, sound, button, USB_VCP
from math import pi as PI
from time import time as now

END_MSG_SYMBOL = "<<\n"
HUB_TIMESTAMP = "H01"
CAM_RECORDER_STOP = "H02"

rcv = RCReceiver(name="robot")
jetson_com_port = USB_VCP(0)

steer_motor = PBMotor(Port.A)
drive_motor = PBMotor(Port.B)

raw_steer_motor = steer_motor.control.motor
raw_drive_motor = drive_motor.control.motor

steer_abs_angle = raw_steer_motor.get()[2]
steer_motor.reset_angle(0)
steer_motor.run_target(120, -steer_abs_angle, wait=True)

drive_abs_angle = raw_drive_motor.get()[2]
drive_motor.reset_angle(0)
drive_motor.run_target(120, -drive_abs_angle, wait=True)

steer_motor.reset_angle(0)
drive_motor.reset_angle(0)

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
while not button.center.is_pressed():
    if rcv.is_connected():
        steer_target, speed_target, trim, thumb = rcv.controller_state(L_STICK_HOR, R_STICK_VER, SETTING1, L_TRIGGER)
    else:
        steer_target, speed_target, trim, thumb = (0, 0, 0, 0)

    trim = 0  # we don't need this, since our steering is mechanically calibrated

    steer_motor.run_target(120, STEER_DIR * steer_target * STEER_OUTPUT_GEAR / STEER_MOTOR_GEAR + trim, wait=False)
    # drive_motor.dc(DRIVE_DIR * speed_target)
    sign = 1 if 0 < speed_target else (-1 if speed_target < 0 else 0)
    drive_motor.run(DRIVE_DIR * ANGULAR_SPEED * sign)

    timestamp = now()
    if 0 != sign:
        yaw, pitch, roll = motion.yaw_pitch_roll()
        pitch_roll_yaw = (pitch, roll, yaw)             # (x, y, z) deg
        gyroscope = motion.gyroscope()                  # (x, y, z) deg/s
        accelerations = motion.accelerometer()          # (x, y, z) cm/s^2
        steer_encoder = tuple(raw_steer_motor.get())    # (speed_pct, rel_pos, abs_pos, pwm)
        drive_encoder = tuple(raw_drive_motor.get())    # (speed_pct, rel_pos, abs_pos, pwm)
        with open(DATA_LOG, 'a') as f:
            f.write("({}),{},{},{},{},{}\n".format(
                timestamp, pitch_roll_yaw, gyroscope, accelerations, steer_encoder, drive_encoder)
            )

    if jetson_com_port:
        jetson_com_port.write("{}{}{}".format(HUB_TIMESTAMP, timestamp, END_MSG_SYMBOL))

    if thumb > 50:
        sound.beep(400, 25)

if jetson_com_port:
    jetson_com_port.write("{}{}".format(CAM_RECORDER_STOP, END_MSG_SYMBOL))
