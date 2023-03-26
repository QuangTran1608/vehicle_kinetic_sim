import os
import cv2

from time import time as now
from time import sleep
from serial import Serial

END_MSG_SYMBOL = b"<<\n"
HUB_TIMESTAMP = b"H01"
CAM_RECORDER_START = b"H02"
CAM_RECORDER_STOP = b"H03"


def main():
    hub_com_port = Serial('/dev/ttyACM0', 115200, timeout=0.050)
    if not hub_com_port.is_open:
        print("Could not connect Lego hub on COM port!")
        return

    start_time = int(now() * 10**6)
    record_dir = "/dev/camera_frames_{}".format(start_time)

    os.makedirs(record_dir, exist_ok=True)
    video_capture = None

    while True:
        sleep(0.02)

        msg_in = None
        while hub_com_port.in_waiting:
            msg_in = hub_com_port.readline()
            hub_com_port.reset_input_buffer()

        if not msg_in:
            continue

        if not msg_in.endswith(END_MSG_SYMBOL):
            continue

        msg_in = msg_in.decode("utf-8")
        print(msg_in)

        msg_in = msg_in.strip()[2:len(msg_in) - len(END_MSG_SYMBOL)]
        len_msg_in = len(msg_in)
        if not (0 < len_msg_in):
            continue

        if msg_in.startswith(CAM_RECORDER_START):
            video_capture = cv2.VideoCapture(0)

        elif msg_in.startswith(CAM_RECORDER_STOP):
            cv2.destroyAllWindows()
            video_capture.release()
            video_capture = None

        elif msg_in.startswith(HUB_TIMESTAMP):
            ts_begin = len(HUB_TIMESTAMP)
            ts_end = len_msg_in
            while not msg_in[ts_end - 1].isdigit():
                ts_end = ts_end - 1

            if not (ts_begin < ts_end):
                continue

            timestamp = msg_in[ts_begin:ts_end]
            if not (video_capture and video_capture.isOpened()):
                continue

            ret, frame = video_capture.read()
            if not ret:
                continue

            basename = timestamp
            base_path = os.path.join(record_dir, basename)
            cv2.imwrite('{}.jpg'.format(base_path), frame)


if __name__ == '__main__':
    main()
