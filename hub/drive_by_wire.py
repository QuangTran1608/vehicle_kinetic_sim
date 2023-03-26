from projects.mpy_robot_tools.helpers import (
    PBMotor, PBUltrasonicSensor, Port
)
from hub import button, motion, USB_VCP
from math import pi as PI
import time

END_HUB_MSG_SYMBOL = "<<\n"
HUB_TIMESTAMP = "H01"
HUB_TERMINATE = "H02"
HUB_DYNAMICS = "H03"
HUB_USS_DIST = "H04"

END_JET_MSG_SYMBOL = "<<"
JET_CTRLS = "J1"

REF_SPEED = 10  # cm/s
ALERT_DIST = 50  # cm
SAFETY_DIST = 5.  # cm
DEFAULT_TOLERANCE = 3.  # cm
TOLERANCE_DIST = DEFAULT_TOLERANCE

STEER_DIR = -1
DRIVE_DIR = -1

STEER_MOTOR_GEAR = 12  # teeth
STEER_OUTPUT_GEAR = 20  # teeth

WHEEL_DIAMETER = 5.6  # cm
DRIVE_MOTOR_GEAR = 20  # teeth
DRIVE_OUTPUT_GEAR = 28  # teeth


def calc_motor_angular_speed(speed):
    return 360 * DRIVE_OUTPUT_GEAR * speed / (DRIVE_MOTOR_GEAR * WHEEL_DIAMETER * PI)  # deg/s


def drive_decode(prev, curr):
    return DRIVE_DIR * (curr[1] - prev[1]) * WHEEL_DIAMETER * PI / 360


def main():
    jetson_com_port = USB_VCP(0)
    if not jetson_com_port.isconnected():
        print("Could not connect Jetson over COM port!")
        return

    steer_motor = PBMotor(Port.A)
    drive_motor = PBMotor(Port.B)
    uss_sensor = PBUltrasonicSensor(Port.C)

    raw_steer_motor = steer_motor.control.motor
    raw_drive_motor = drive_motor.control.motor

    raw_steer_motor = steer_motor.control.motor
    steer_abs_angle = raw_steer_motor.get()[2]
    steer_motor.reset_angle(0)
    steer_motor.run_target(120, -steer_abs_angle, wait=True)

    steer_motor.reset_angle(0)
    drive_motor.reset_angle(0)

    p_drive_encoder = None
    p_timestamp = None

    while not button.center.is_pressed():
        timestamp = time.ticks_ms()
        if not p_timestamp:
            # skip first iteration
            p_drive_encoder = tuple(raw_drive_motor.get())
            p_timestamp = timestamp
            continue

        dt = time.ticks_diff(timestamp, p_timestamp)  # milliseconds
        dt /= 1000.  # seconds

        # receive controls from Jetson
        msg_in = None
        try:
            # Read 10 bytes from buffer
            buffer = jetson_com_port.read(10)
            # Decode message
            msg_in = str(buffer.decode('UTF-8'))
            print("Received data: " + str(msg_in))
        except Exception as ex:
            print(ex)

        if msg_in and msg_in.endswith(END_JET_MSG_SYMBOL):
            if msg_in.startswith(JET_CTRLS):
                steering = int(msg_in[2:5])
                throttle = int(msg_in[5:8])
                steer_motor.dc(steering)
                drive_motor.dc(throttle)

        # send car dynamics to Jetson
        yaw, _, _ = motion.yaw_pitch_roll()
        drive_encoder = tuple(raw_drive_motor.get())  # (speed_pct, rel_pos, abs_pos, pwm)
        travel_dist = drive_decode(p_drive_encoder, drive_encoder)
        velocity = travel_dist / dt  # cm/s
        jetson_com_port.write("{}{},{}{}".format(
            HUB_DYNAMICS, yaw, velocity, END_HUB_MSG_SYMBOL
        ))

        obstacle_dist = uss_sensor.distance() / 10.  # cm
        if obstacle_dist < ALERT_DIST:
            jetson_com_port.write("{}{}{}".format(
                HUB_USS_DIST, obstacle_dist, END_HUB_MSG_SYMBOL
            ))

        p_drive_encoder = drive_encoder
        p_timestamp = timestamp

    jetson_com_port.write("{}{}".format(
        HUB_TERMINATE, END_HUB_MSG_SYMBOL
    ))


if __name__ == '__main__':
    main()
