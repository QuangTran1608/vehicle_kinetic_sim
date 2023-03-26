import cv2
import math
import numpy as np
from ultralytics import YOLO

img_size = (480, 640)

model = YOLO("yolo/best.pt")

class FuzzyController():
    def __init__(self):
        self.accumulator_size = (480//4, 640//3)
        y_start = img_size[0]//2
        self.accumulator_position = [((0,self.accumulator_size[1]), (y_start, y_start+self.accumulator_size[0])),
                        ((self.accumulator_size[1], 2*self.accumulator_size[1]), (y_start, y_start+self.accumulator_size[0])),
                        ((2*self.accumulator_size[1], img_size[1]), (y_start, y_start+self.accumulator_size[0])),
                        ((0,self.accumulator_size[1]), (y_start+self.accumulator_size[0], img_size[0])),
                        ((self.accumulator_size[1], 2*self.accumulator_size[1]), (y_start+self.accumulator_size[0], img_size[0])),
                        ((2*self.accumulator_size[1], img_size[1]), (y_start+self.accumulator_size[0], img_size[0])),
                        ]
        self.steer_caliberate = np.array([-0.174, 0, 0.174, -0.3, 0, 0.3])
        self.velocity_yaw_rate_constant = 0.04

    def get_control_fuzzy(self, image, velocity):
        results= model.predict(image, device=0)
        try:
            mask = results[0].masks.masks[0, :, :].numpy()
        except:
            return None
        accumulator = np.array([np.sum(results[0].masks.masks[0, :, :].numpy()[x_range[0]:x_range[1], y_range[0]:y_range[1]]) \
                        for x_range, y_range in self.accumulator_position])
        # can use argpartition instead for better performance
        best3_lane_indexes = np.argsort(accumulator)[-3]
        for i, index in enumerate(best3_lane_indexes):
            if accumulator[index] == 0:
                best3_lane_indexes = np.delete(best3_lane_indexes, i)
        steer = self.velocity_yaw_rate_constant * velocity * np.sum(self.steer_caliberate[best3_lane_indexes])
        throttle = math.cos(steer)*1.
        return throttle, steer
