from projects.mpy_robot_tools.helpers import (
    PBMotor, PBUltrasonicSensor, Port
)
from hub import button, motion, USB_VCP
import time

END_MSG_SYMBOL = "<<\n"
HUB_TIMESTAMP = "H01"
HUB_TERMINATE = "H02"
JETSON_CTRLS = "J01"

REF_SPEED = 10  # cm/s
SAFETY_DIST = 5.  # cm
DEFAULT_TOLERANCE = 3.  # cm
TOLERANCE_DIST = DEFAULT_TOLERANCE


def main():
    jetson_com_port = USB_VCP(0)
    if not jetson_com_port.isconnected():
        print("Could not connect Jetson over COM port!")
        return

    steer_motor = PBMotor(Port.A)
    drive_motor = PBMotor(Port.B)
    uss_sensor = PBUltrasonicSensor(Port.C)

    raw_steer_motor = steer_motor.control.motor
    steer_abs_angle = raw_steer_motor.get()[2]
    steer_motor.reset_angle(0)
    steer_motor.run_target(120, -steer_abs_angle, wait=True)

    steer_motor.reset_angle(0)
    drive_motor.reset_angle(0)

    p_t = None
    while not button.center.is_pressed():
        now = time.time_ns()
        if not p_t:
            # skip first iteration
            p_t = now
            continue

        dt = (now - p_t) / 1000000  # in milliseconds
        p_t = now

        msg_in = None
        try:
            # Read 10 bytes from buffer
            buffer = jetson_com_port.read(10)
            # Decode message
            msg_in = str(buffer.decode('UTF-8'))
            print("Received data: " + str(msg_in))
        except Exception as ex:
            print(ex)

        timestamp = now()

        yaw, pitch, roll = motion.yaw_pitch_roll()
        pitch_roll_yaw = (pitch, roll, yaw)     # (x, y, z) deg
        gyroscope = motion.gyroscope()          # (x, y, z) deg/s
        accelerations = motion.accelerometer()  # (x, y, z) cm/s^2

        obstacle_dist = uss_sensor.distance()

        jetson_com_port.write("{}{}{}".format(
            HUB_TIMESTAMP, timestamp, END_MSG_SYMBOL
        ))

    jetson_com_port.write("{}{}".format(
        HUB_TERMINATE, END_MSG_SYMBOL
    ))


if __name__ == '__main__':
    main()
