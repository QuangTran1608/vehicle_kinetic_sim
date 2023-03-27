from controller import MPCController
from model import LegoModel
from glob import glob
import numpy as np
import math
from ultralytics import YOLO
import cv2

img_size = (480, 640)
middle_x = img_size[1] // 2 

model = YOLO("yolo/best.pt")
car_model = LegoModel()
mpc = MPCController(car_model)
horizon = mpc.horizon
y_checkpoint_per_10_cm = [440, 340, 275, 235, 210, 200, 195, 190]
x_pixel_1cm_ratio = [30, 20, 17, 15, 12, 10, 8, 7]

def grab_lane_middle_point(array):
    left_point = None
    right_point = None
    middle_point = None
    for i in range(len(array)):
        if array[i] > 0:
            left_point = i
            break

    for i in range(len(array)-1, -1, -1):
        if array[i] > 0:
            right_point = i
            break
    if left_point and right_point:
        middle_point = (right_point+left_point)//2
    elif left_point:
        middle_point = left_point
    elif right_point:
        middle_point = right_point
    return middle_point

def get_control_mpc(image, velocity):
    results= model.predict(image, device='cpu')
    try:
        resImg = results[0].plot(
            show_conf=True, 
            line_width=1, 
            font_size=1, 
            font='Arial.ttf', 
            pil=False, 
            example='abc')
        mask = results[0].masks.masks[0, :, :].numpy()
    except Exception as e:
        print(e)
        return None, None
    x_list = list()
    y_list = list()
    for i, y in enumerate(y_checkpoint_per_10_cm):
        middle_pixel_point = grab_lane_middle_point(mask[y, :])
        if not middle_pixel_point is None:
            x_abs = (middle_pixel_point - middle_x) / x_pixel_1cm_ratio[i]
            y_abs = (i+1) * 10
            x_list.append(x_abs)
            y_list.append(y_abs)
    try:
        point_collection = np.array([np.array(x_list), np.array(y_list)])
        target_orientation = np.arctan2(point_collection[1, 1:] - point_collection[1, 0:-1],
                                        point_collection[0, 1:] - point_collection[0, 0:-1])
        target_orientation = np.append(target_orientation, target_orientation[-1])
    except Exception as e:
        print(e)
        return None, None
    mpc.set_expected_state(point_collection, target_orientation)
    control = mpc.get_control(velocity=10)
    throttle = control[:horizon]
    steering = control[horizon:]
    return list(zip(throttle, steering)), resImg

img_list = glob('OkwithObs/*.jpg')
print("data len: ",len(img_list))
fourcc = cv2.VideoWriter_fourcc(*'XVID')
# Export video
out = cv2.VideoWriter('proc.avi', fourcc, 10.0, (640, 480))
font = cv2.FONT_HERSHEY_SIMPLEX
fontScale = 1
for image_dir in img_list:
    # Capture the frames
    image = cv2.imread(image_dir)
    control, vis_img = get_control_mpc(image, 10)
    if not vis_img is None:
        vis = cv2.putText(vis_img, 'steering: ' + str('%02.2f'%(control[0][1]/math.pi*180)) + ' degree', 
                        fontFace=font, fontScale=fontScale, org=(20, 40), color=[255,0,0], 
                        thickness=2, lineType=cv2.LINE_AA)
        out.write(vis)
out.release()
    