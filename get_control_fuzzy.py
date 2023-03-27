import cv2
import math
import numpy as np
from glob import glob
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
        results= model.predict(image, device='cpu')
        try:
            mask = results[0].masks.masks[0, :, :].numpy()
        except:
            return None
        accumulator = np.array([np.sum(results[0].masks.masks[0, :, :].numpy()[x_range[0]:x_range[1], y_range[0]:y_range[1]]) \
                        for x_range, y_range in self.accumulator_position])
        # can use argpartition instead for better performance
        best3_lane_indexes = list()
        for index in np.argsort(accumulator)[-3:]:
            if accumulator[index] > 0:
                best3_lane_indexes.append(index)
        if len(best3_lane_indexes):
            best3_lane_indexes = np.array(best3_lane_indexes)
            steer = self.velocity_yaw_rate_constant * velocity * np.sum(self.steer_caliberate[best3_lane_indexes])
            throttle = math.cos(steer)*1.
            return [[throttle, steer]]
        else:
            return None

controller = FuzzyController()
img_list = glob('OkwithObs/*.jpg')
print("data len: ",len(img_list))
fourcc = cv2.VideoWriter_fourcc(*'XVID')
# Export video
out = cv2.VideoWriter('fuzzy.avi', fourcc, 10.0, (640, 480))
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 1
for image_dir in img_list:
    # Capture the frames
    image = cv2.imread(image_dir)
    control = controller.get_control_fuzzy(image, 10)
    if control:
        vis = cv2.putText(image, 'steering: ' + str('%02.2f'%(control[0][1]/math.pi*180)) + ' degree', 
                        fontFace=font, fontScale=fontScale, org=(20, 40), color=[255,0,0], 
                        thickness=2, lineType=cv2.LINE_AA)
        out.write(vis)
out.release()
