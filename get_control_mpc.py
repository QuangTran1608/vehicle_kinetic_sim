from controller import MPCController
import math
import numpy as np
from ultralytics import YOLO

img_size = (480, 640)

model = YOLO("yolo/best.pt")
mpc = MPCController()
horizon = mpc.horizon

def get_control_mpc(image, velocity):
    results= model.predict(image, device=0)
    try:
        mask = results[0].masks.masks[0, :, :].numpy()
    except:
        return None
    control = mpc.get_control(velocity)
    throttle = control[:horizon]
    steering = control[horizon:]
    return list(zip(throttle, steering))
