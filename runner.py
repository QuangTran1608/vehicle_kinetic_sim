import os
import argparse
import importlib 
import math
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

from simulator.type import Position, Orientation
from model.car import BaseModel

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='''Draft as simulation scenario with defined test cases''')

    parser.add_argument('-t', '--testcase', default='test_straight_line',
                        help='capture data of a test case defined in testcases folder')
    parser.add_argument('-c', '--controller', default='base_controller',
                        help='controller module')

    args = parser.parse_args()
    
    testcase_module = f'testcases.{args.testcase}'
    test_data = importlib.import_module(testcase_module, package=None)
    
    controller_module = f'controller.{args.controller}'
    controller = importlib.import_module(controller_module, package=None)
    speed_controller = controller.BaseController()
    yaw_controller = controller.BaseController(0.5)
    
    fig = plt.figure()
    
    model = BaseModel(test_data.spawn_point, test_data.spawn_ori)

    ax = plt.axes(xlim=test_data.xlim, ylim=test_data.ylim)
    line, = ax.plot([], [], lw=3)
    line.set_data([], [])
    delta_time = 1
    def draw_map():
        ax.plot(test_data.x_1, test_data.y_1)
        ax.plot(test_data.x_2, test_data.y_2)
        return line,

    def step(i):
        target_speed = math.sqrt((test_data.checkpoint[0, i] - model.pos.x)**2 + \
                                 (test_data.checkpoint[1, i] - model.pos.y)**2)
        target_yaw = math.atan2(test_data.checkpoint[1, i] - model.pos.y,
                                test_data.checkpoint[0, i] - model.pos.x)
        model.update_velocity(model.velocity + speed_controller.get_control(model.velocity, target_speed))
        model.update_yaw_rate(model.yaw_rate + yaw_controller.get_control(model.yaw_rate, (target_yaw - model.orientation.yaw)/delta_time))
        model.update_position(delta_time)
        x,y = model.bbox.get_BBox()
        x = np.append(x, x[0])
        y = np.append(y, y[0])
        line.set_data(x, y)
        return line,

    anim = FuncAnimation(fig, step, init_func=draw_map, frames=50, interval=100, blit=True, repeat=False)
    anim.save(f'{args.testcase}.gif')
    plt.show()
    