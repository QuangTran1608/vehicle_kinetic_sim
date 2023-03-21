import argparse
import importlib 
from dataclasses import dataclass
import math
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

from simulator.type import Position, Orientation

@dataclass
class Fargs:
    ax: plt.Axes

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='''Draft as simulation scenario with defined test cases''')

    parser.add_argument('-t', '--testcase', default='test_straight_line',
                        help='capture data of a test case defined in testcases folder')
    parser.add_argument('-c', '--controller', default='BaseController',
                        help='controller module')
    parser.add_argument('-m', '--model', default='BaseModel',
                        help='controller module')

    args = parser.parse_args()
    
    testcase_module = f'testcases.{args.testcase}'
    test_data = importlib.import_module(testcase_module)
    
    controller_module = 'controller.base'
    controller = getattr(importlib.import_module(controller_module), args.controller)
    speed_controller = controller(0.1)
    yaw_controller = controller(0.1)
    
    vehicle_model = 'model.car'
    vehicle_model = getattr(importlib.import_module(vehicle_model), args.model)
    model = vehicle_model(pos=test_data.spawn_point,
                          orientation=test_data.spawn_ori)
    fig = plt.figure()
    ax = plt.axes()
    line, = ax.plot([], [], lw=3)
    line.set_data([], [])
    delta_time = 0.1
    ax.set_aspect('equal')
    ax.plot(test_data.x_1, test_data.y_1)
    ax.plot(test_data.x_2, test_data.y_2)
    def step(i, fargs):
        fargs.ax.set_xlim(model.pos.x - 50, model.pos.x + 50)
        fargs.ax.set_ylim(model.pos.y - 50, model.pos.y + 50)
        dist = (test_data.checkpoint[0, :] - model.pos.x)**2 + (test_data.checkpoint[1, :] - model.pos.y)**2
        current_index = min(np.argmin(dist) + 5, test_data.checkpoint.shape[1]-1)
        target_speed = 10
        target_yaw = math.atan2(test_data.checkpoint[1, current_index] - model.pos.y,
                                test_data.checkpoint[0, current_index] - model.pos.x)
        model.apply_throttle(speed_controller.get_control(model.velocity, target_speed))
        model.apply_steer(yaw_controller.get_control(model.yaw_rate, (target_yaw - model.orientation.yaw)/delta_time))
        model.step()
        x,y = model.bbox.get_BBox()
        x = np.append(x, x[0])
        y = np.append(y, y[0])
        line.set_data(x, y)
        return line,

    anim = FuncAnimation(fig, step, fargs=[Fargs(ax=ax)], frames=500, interval=int(1000*delta_time), blit=False, repeat=False)
    anim.save(f'{args.testcase}.gif')
    plt.show()
    