class BaseController():
    def __init__(self, Kp=1):
        self._Kp = Kp

    def get_control(self, current_state, target_state):
        error = target_state - current_state
        return self._Kp*error
