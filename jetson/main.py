import os
import cv2

from time import time as now
from time import sleep
from serial import Serial
from hub.drive_by_wire import (
    HUB_TIMESTAMP
)


def main():
    hub_com_port = Serial('/dev/ttyACM0', 115200, timeout=0.050)
    if not hub_com_port.is_open:
        print("Could not connect Lego hub on COM port!")
        return

    video_capture = cv2.VideoCapture(0)
    if not (video_capture and video_capture.isOpened()):
        print("Could not open camera!")
        return

    while True:
        sleep(0.02)

        ret, frame = video_capture.read()

        msg_in = None
        while hub_com_port.in_waiting:
            msg_in = hub_com_port.readline().strip()
            print(msg_in.decode("utf-8"))
            hub_com_port.reset_input_buffer()

        len_msg_in = len(msg_in)
        if not (0 < len_msg_in):
            continue

        if msg_in.startswith(HUB_TIMESTAMP):
            ts_begin = len(HUB_TIMESTAMP)
            ts_end = len_msg_in
            while not msg_in[ts_end - 1].isdigit():
                ts_end = ts_end - 1

            if not (ts_begin < ts_end):
                continue

            hub_timestamp = int(msg_in[ts_begin:ts_end])

    cv2.destroyAllWindows()
    video_capture.release()


if __name__ == '__main__':
    main()
