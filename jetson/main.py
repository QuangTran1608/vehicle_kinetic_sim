import cv2

from serial import Serial

MSG_HEADER_LEN = 3
MSG_FOOTER_LEN = 3
END_MSG_SYMBOL = b"<<\n"
HUB_TIMESTAMP = b"H01"
HUB_TERMINATE = b"H02"


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
        log.write("Could not open camera!")
        return

    while True:
        ret, frame = video_capture.read()
        if ret:
            # TODO: do something with the video frame here
            pass

        msg_in = None
        while hub_com_port.in_waiting:
            msg_in = hub_com_port.readline()
            hub_com_port.reset_input_buffer()

        if not msg_in:
            continue

        if not msg_in.endswith(END_MSG_SYMBOL):
            continue

        if msg_in.startswith(HUB_TERMINATE):
            log.write("Stop recording")
            msg_in = decode_msg(msg_in)
            cv2.destroyAllWindows()
            video_capture.release()
            video_capture = None

        elif msg_in.startswith(HUB_TIMESTAMP):
            log.write("Receive timestamp")
            msg_in = decode_msg(msg_in)
            ts_begin = len(HUB_TIMESTAMP)
            ts_end = len(msg_in)
            while not msg_in[ts_end - 1].isdigit():
                ts_end = ts_end - 1

            if not (ts_begin < ts_end):
                log.write("Empty timestamp")
                continue

            timestamp = msg_in[ts_begin:ts_end]
            log.write(timestamp)


if __name__ == '__main__':
    main()
