import cv2
import numpy as np

from serial import Serial
from math import pi as PI

MSG_HEADER_LEN = 3
MSG_FOOTER_LEN = 3

END_HUB_MSG_SYMBOL = b"<<\n"
HUB_TIMESTAMP = b"H01"
HUB_TERMINATE = b"H02"
HUB_DYNAMICS = b"H03"
HUB_USS_DIST = b"H04"

END_JET_MSG_SYMBOL = "<<"
JET_CTRLS = "J1"

WHEEL_DIAMETER = 5.6  # cm
DRIVE_MOTOR_GEAR = 20  # teeth
DRIVE_OUTPUT_GEAR = 28  # teeth
MAX_RPM = 155 * (DRIVE_MOTOR_GEAR / DRIVE_OUTPUT_GEAR)
MAX_SPEED = (MAX_RPM / 60) * WHEEL_DIAMETER * PI  # cm/s
DEFAULT_SPEED = 10.  # cm/s
TARGET_SPEED = DEFAULT_SPEED


class Logger:
    def __init__(self):
        self.f = open('log.txt', 'w')

    def write(self, msg):
        self.f.write(msg)

    def __del__(self):
        self.f.close()


log = Logger()


def decode_hub_msg(msg_in):
    msg_out = msg_in.decode("utf-8")
    # log.write(msg_out)
    msg_out = msg_in.strip()[MSG_HEADER_LEN:(len(msg_in) - MSG_FOOTER_LEN)]
    # log.write(msg_out)
    return str(msg_out)


def main():
    try:
        hub_com_port = Serial('/dev/ttyACM0', 115200, timeout=0.050)
    except Exception as ex:
        log.write(ex)
        hub_com_port = Serial('/dev/ttyACM1', 115200, timeout=0.050)

    if not hub_com_port.is_open:
        log.write("Could not connect Lego hub on COM port!")
        return

    camset = "nvarguscamerasrc ! nvvidconv ! video/x-raw, width=640, height=480, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
    video_capture = cv2.VideoCapture(camset, cv2.CAP_GSTREAMER)
    if not (video_capture and video_capture.isOpened()):
        log.write("Could not open camera!")
        return

    while True:
        ret, frame = video_capture.read()
        if ret:
            # TODO: process video frame here (asynchronously ???)
            pass

        # receive data from Lego hub
        msg_in = None
        while hub_com_port.in_waiting:
            msg_in = hub_com_port.readline()
            hub_com_port.reset_input_buffer()

        if not msg_in:
            continue

        if not msg_in.endswith(END_HUB_MSG_SYMBOL):
            continue

        if msg_in.startswith(HUB_TERMINATE):
            log.write("Stop everything")
            cv2.destroyAllWindows()
            video_capture.release()
            video_capture = None

        elif msg_in.startswith(HUB_DYNAMICS):
            log.write("Receive car dynamics")
            dynamics = decode_hub_msg(msg_in)
            log.write(dynamics)

            sep_index = dynamics.find(',')
            if -1 == sep_index:
                log.write("Error: Separator not found")
                continue

            yaw = np.radians(int(dynamics[2:sep_index]))  # rad
            log.write("Yaw: {}".format(yaw))

            velocity = float(dynamics[sep_index+1:])  # cm/s
            log.write("Velocity: {}".format(velocity))

        elif msg_in.startswith(HUB_USS_DIST):
            log.write("Receive USS distance")
            obstacle_dist = decode_hub_msg(msg_in)
            log.write(obstacle_dist)

            obstacle_dist = float(obstacle_dist)  # cm

        # TODO: make decision
        steering = 0
        throttle = 0

        # from percentage of max speed to power
        throttle = int(100 * throttle / MAX_SPEED)

        # send controls to Lego hub
        steering_str = str(steering).zfill(3)  # -99 <= steering <= 99  (degrees)
        throttle_str = str(throttle).zfill(3)  # -99 <= throttle <= 99  (percentage)
        ctrls = JET_CTRLS + steering_str + throttle_str + END_JET_MSG_SYMBOL
        hub_com_port.write(ctrls.encode("utf-8"))


if __name__ == '__main__':
    main()
