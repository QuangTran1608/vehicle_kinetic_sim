import os
import cv2

from time import time as now
from time import sleep
from serial import Serial

MSG_HEADER_LEN = 3
MSG_FOOTER_LEN = 3
END_MSG_SYMBOL = b"<<\n"
HUB_TIMESTAMP = b"H01"
CAM_RECORDER_STOP = b"H02"


class Logger:
    def __init__(self):
        self.f = open('log.txt', 'w')

    def write(self, msg):
        self.f.write(msg)

    def __del__(self):
        self.f.close()


log = Logger()


def decode_msg(msg_in):
    msg_out = msg_in.decode("utf-8")
    # log.write(msg_out)
    msg_out = msg_in.strip()[MSG_HEADER_LEN:(len(msg_in) - MSG_FOOTER_LEN)]
    # log.write(msg_out)
    return str(msg_out)


def main():
    try:
        hub_com_port = Serial('/dev/ttyACM0', 115200, timeout=0.050)
    except:
        hub_com_port = Serial('/dev/ttyACM1', 115200, timeout=0.050)

    if not hub_com_port.is_open:
        log.write("Could not connect Lego hub on COM port!")
        return

    camset = "nvarguscamerasrc ! nvvidconv ! video/x-raw, width=640, height=480, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
    video_capture = cv2.VideoCapture(camset, cv2.CAP_GSTREAMER)
    if not (video_capture and video_capture.isOpened()):
        return

    start_time = int(now() * 10**6)
    record_dir = "/dev/camera_frames_{}".format(start_time)
    os.makedirs(record_dir, exist_ok=True)

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

        if msg_in.startswith(CAM_RECORDER_STOP):
            log.write("Stop recording")
            cv2.destroyAllWindows()
            video_capture.release()
            video_capture = None

        elif msg_in.startswith(HUB_TIMESTAMP):
            log.write("Receive timestamp")
            timestamp = decode_msg(msg_in)
            log.write(timestamp)

            if not video_capture:
                continue

            ret, frame = video_capture.read()
            if not ret:
                continue

            log.write('frame captured')
            basename = timestamp
            base_path = os.path.join(record_dir, basename)
            cv2.imwrite('{}.jpg'.format(base_path), frame)


if __name__ == '__main__':
    main()
