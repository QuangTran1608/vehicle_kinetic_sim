import argparse
from pydoc import locate
from dataclasses import dataclass
import math
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import common.maths.functions as func


class Simulation:
    def __init__(self):
        self.fps = 30.0
        self.dt = 1 / self.fps
        self.interval = self.dt * 10**3
        self.map_size_x = 70
        self.map_size_y = 40
        self.frames = 1000
        self.loop = False


class Car:
    def __init__(self, model, speed_controller, yaw_controller):
        self.model = model
        self.speed_controller = speed_controller
        self.yaw_controller = yaw_controller

    def drive(self, target_speed, yaw_diff):
        throttle = self.speed_controller.get_control(self.model.velocity, target_speed)
        steering = self.yaw_controller.get_control(self.model.yaw_rate, yaw_diff)
        self.model.apply_throttle(throttle)
        self.model.apply_steer(steering)
        self.model.step()


@dataclass
class Fargs:
    ax: plt.Axes
    sim: Simulation
    car: Car
    car_outline: plt.Line2D


def step(frame, fargs):
    # Camera tracks car
    fargs.ax.set_xlim(fargs.car.model.position.x - fargs.sim.map_size_x,
                      fargs.car.model.position.x + fargs.sim.map_size_x)
    fargs.ax.set_ylim(fargs.car.model.position.y - fargs.sim.map_size_y,
                      fargs.car.model.position.y + fargs.sim.map_size_y)

    # Get car's target
    dist = (test_data.checkpoint[0, :] - fargs.car.model.position.x)**2 + \
           (test_data.checkpoint[1, :] - fargs.car.model.position.y)**2
    current_index = min(np.argmin(dist) + 5, test_data.checkpoint.shape[1]-1)
    target_yaw = math.atan2(test_data.checkpoint[1, current_index] - fargs.car.model.position.y,
                            test_data.checkpoint[0, current_index] - fargs.car.model.position.x)
    yaw_diff = func.norm_to_range(target_yaw - fargs.car.model.rotation.yaw)
    target_speed = 10

    # Drive car and draw car
    fargs.car.drive(target_speed, yaw_diff / fargs.sim.dt)
    x, y = fargs.car.model.bbox.get_bbox()
    x = np.append(x, x[0])
    y = np.append(y, y[0])
    fargs.car_outline.set_data(x, y)
    return fargs.car_outline,


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='''Simulation scenario with defined test cases''')
    parser.add_argument('-t', '--testcase', default='straight_line',
                        help='capture data of a test case defined in testcases folder')
    parser.add_argument('-c', '--controller', default='BaseController',
                        help='controller module')
    parser.add_argument('-m', '--model', default='LegoModel',
                        help='controller module')
    args = parser.parse_args()

    test_data = locate(f'testcases.{args.testcase}')

    controller = locate(f'controller.{args.controller}')
    speed_controller = controller(0.1)
    yaw_controller = controller(0.1)

    vehicle_model = locate(f'model.{args.model}')
    model = vehicle_model(position=test_data.spawn_position,
                          rotation=test_data.spawn_rotation)

    sim = Simulation()
    car = Car(model, speed_controller, yaw_controller)

    fig = plt.figure()
    ax = plt.axes()
    ax.set_aspect('equal')

    ax.plot(test_data.x_1, test_data.y_1)
    ax.plot(test_data.x_2, test_data.y_2)
    ax.plot(test_data.checkpoint[0],
            test_data.checkpoint[1],
            linestyle='dashed',
            color='gold')

    empty = ([], [])
    car_outline, = ax.plot(*empty)

    fargs = [Fargs(
        ax=ax,
        sim=sim,
        car=car,
        car_outline=car_outline,
    )]

    anim = FuncAnimation(fig, step, fargs=fargs, frames=sim.frames, interval=sim.interval, repeat=sim.loop)
    # anim.save(f'{args.testcase}.gif', writer='imagemagick', fps=sim.fps)

    plt.grid()
    plt.show()
