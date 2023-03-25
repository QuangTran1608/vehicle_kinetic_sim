from controller import BaseController

class PIDController(BaseController):
    def __init__(self, Kp=1, Ki=1, Kd=1, delta_time=0.1, persist_error=False):
        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd
        self._dt = delta_time
        self.persist_error = persist_error

    def set_expected_state(self, target_state):
        self.target_state = target_state
        if not self.persist_error:
            self._prev_err = 0
            self._d_error = 0
            self._i_error = 0

    def get_control(self, current_state):
        self._update_error(current_state)
        return self._Kp*self._error + \
               self._Kd*self._d_error + \
               self._Ki*self._i_error

    def _update_error(self, current_state):
        self._error = self.target_state - current_state
        self._d_error = (self._error - self._prev_err)/self._dt
        self._prev_err = self._error
        self._i_error += self._error*self._dt

    def update_K(self, Kp=1, Ki=1, Kd=1):
        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd
