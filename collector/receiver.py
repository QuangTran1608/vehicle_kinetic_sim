from hub import motion, port, sound
from projects.mpy_robot_tools.rc import RCReceiver, R_STICK_VER, L_STICK_HOR, SETTING1, L_TRIGGER
from projects.mpy_robot_tools.helpers import PBMotor as Motor
from projects.mpy_robot_tools.helpers import Port, wait
from math import pi

rcv = RCReceiver(name="robot")

steer_motor = Motor(Port.A)
drive_motor = Motor(Port.B)

raw_steer_motor = steer_motor.control.motor
raw_drive_motor = drive_motor.control.motor

# speed_pct, rel_pos, abs_pos, pwm
raw_steer_motor.mode([(1, 0), (2, 0), (3, 0), (0, 0)])
raw_drive_motor.mode([(1, 0), (2, 0), (3, 0), (0, 0)])

STEER_DIR = -1
DRIVE_DIR = -1

STEER_MOTOR_GEAR = 12  # teeth
STEER_OUTPUT_GEAR = 20  # teeth

CONST_SPEED = 10  # cm/s
WHEEL_DIAMETER = 5.6  # cm
DRIVE_MOTOR_GEAR = 20  # teeth
DRIVE_OUTPUT_GEAR = 28  # teeth
ANGULAR_SPEED = 360 * DRIVE_OUTPUT_GEAR * CONST_SPEED / (DRIVE_MOTOR_GEAR * WHEEL_DIAMETER * pi)  # deg/s

while 1:
    if rcv.is_connected():
        steer_target, speed_target, trim, thumb = rcv.controller_state(L_STICK_HOR, R_STICK_VER, SETTING1, L_TRIGGER)
    else:
        steer_target, speed_target, trim, thumb = (0, 0, 0, 0)

    steer_motor.track_target(STEER_DIR * steer_target * (STEER_OUTPUT_GEAR / STEER_MOTOR_GEAR) + trim)
    # drive_motor.dc(DRIVE_DIR * speed_target)
    sign = 1 if 0 < speed_target else (-1 if speed_target < 0 else 0)
    drive_motor.run(DRIVE_DIR * ANGULAR_SPEED * sign)
    if 0 != sign:
        yaw, _, _ = motion.yaw_pitch_roll()
        with open("test_data.txt", 'a') as f:
            f.write(f"{yaw} {raw_steer_motor.get()} {raw_drive_motor.get()}\n")

    if thumb > 50:
        sound.beep(400, 25)
        wait(20)

