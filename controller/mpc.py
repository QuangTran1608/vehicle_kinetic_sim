from controller import BaseController
import math
from common.maths.functions import norm_to_range
from scipy.optimize import minimize
import numpy as np

# for simplicity, everything is list


def dist(point_a, point_b):
    return math.sqrt((point_a[0] - point_b[0])**2 + (point_a[1] - point_b[1])**2)

class Obstacle():
    affect_radius_ratio = 1.5
    amp_affect_radius = 0.5
    amp_collide_radius = 1.

    def __init__(self, center, radius):
        self.center = center
        self.collide_radius = radius
        self.affect_radius = self.affect_radius_ratio * radius

    def cost(self, ego_loc):
        cost = 0
        dist_center_obs = dist(ego_loc, self.center)
        if dist_center_obs < self.affect_radius:
            cost += self.amp_affect_radius*(self.affect_radius - dist_center_obs)
            if dist_center_obs < self.collide_radius:
                cost += self.amp_collide_radius*(self.collide_radius - dist_center_obs) ** 2
        return cost


def cost_2d_target_point(ego_loc, target_point):
    return dist(ego_loc, target_point)**2


def cost_yaw(calculated_yaw, expected_yaw, scaler=2):
    return scaler*math.tan(abs(norm_to_range(expected_yaw-calculated_yaw))/2)


def cost_steering(prev_steer, current_steer, scaler=2):
    return scaler*(current_steer - prev_steer)**2

class MPCController(BaseController):
    def __init__(self, horizon=5, sim_steps=30):
        self.horizon = horizon
        self.sim_steps = sim_steps
        self.bounds = np.array([-1, 1]*self.horizon + [-0.872, 0.872]*self.horizon).reshape(self.horizon*2, 2)

    def calc_cost(self, control):
        current_sim_state = self.current_states
        ret = 0
        for i in range(min(self.horizon, len(self.target_yaw))):
            ret += cost_2d_target_point(current_sim_state[:2], self.target_pos[:, i])
            ret += cost_yaw(current_sim_state[2], self.target_yaw[i])
            current_sim_state = self.model.simulate_next_step(*current_sim_state,
                                                              control[i],
                                                              control[i+self.horizon])
        tmp_steer_prev = control[self.horizon:-1]
        tmp_steer_curr = control[self.horizon+1:]
        ret += np.sum(cost_steering(tmp_steer_prev, tmp_steer_curr))
        return ret

    def set_expected_state(self, target_pos, target_yaw):
        self.target_pos = target_pos[:self.horizon]
        self.target_yaw = target_yaw[:self.horizon]

    def get_control(self, velocity):
        init_control = np.zeros(shape=(self.horizon*2))
        self.current_states = (0, 0, 0, velocity, 0) 
        solution = minimize(self.calc_cost,
                            init_control,
                            method='SLSQP',
                            bounds=self.bounds,
                            tol=1e-5)
        print(solution)
        return solution.x
