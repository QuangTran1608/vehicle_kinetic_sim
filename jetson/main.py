import os
import cv2

from time import time as now
from time import sleep
from serial import Serial

MSG_HEADER_LEN = 3
END_MSG_SYMBOL = b"<<\n"
HUB_TIMESTAMP = b"H01"
CAM_RECORDER_START = b"H02"
CAM_RECORDER_STOP = b"H03"

class Logger:
    def __init__(self):
        self.f = open('log.txt', 'w')
    def write(self, msg):
        self.f.write(msg)
    def __del__(self):
        self.f.close()


def main():
    log = Logger()

    try:
        hub_com_port = Serial('/dev/ttyACM0', 115200, timeout=0.050)
    except:
        hub_com_port = Serial('/dev/ttyACM1', 115200, timeout=0.050)

    if not hub_com_port.is_open:
        log.write("Could not connect Lego hub on COM port!")
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

        if msg_in.endswith(END_MSG_SYMBOL):
            msg_in = msg_in.decode("utf-8")
            log.write(msg_in)

            msg_in = msg_in.strip()[MSG_HEADER_LEN:(len(msg_in) - 2)]
            log.write(msg_in)

            len_msg_in = len(msg_in)
            if not (0 < len_msg_in):
                continue

            if msg_in.startswith(CAM_RECORDER_START):
                log.write(CAM_RECORDER_START)
                video_capture = cv2.VideoCapture(0)

            elif msg_in.startswith(CAM_RECORDER_STOP):
                log.write(CAM_RECORDER_STOP)
                cv2.destroyAllWindows()
                video_capture.release()
                video_capture = None

            elif msg_in.startswith(HUB_TIMESTAMP):
                log.write(HUB_TIMESTAMP)
                ts_begin = len(HUB_TIMESTAMP)
                ts_end = len_msg_in
                while not msg_in[ts_end - 1].isdigit():
                    ts_end = ts_end - 1

                log.write('after while')
                if not (ts_begin < ts_end):
                    continue

                log.write('after ts check')
                timestamp = msg_in[ts_begin:ts_end]
                if not (video_capture and video_capture.isOpened()):
                    continue

                log.write('video is opened')
                ret, frame = video_capture.read()
                if not ret:
                    continue

                log.write('frame captured')
                basename = timestamp
                base_path = os.path.join(record_dir, basename)
                cv2.imwrite('{}.jpg'.format(base_path), frame)


if __name__ == '__main__':
    main()
