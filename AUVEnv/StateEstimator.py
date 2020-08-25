import numpy as np
import os
from .ekf import ekf_estimation, observation
from .FlowField import FlowField
from copy import deepcopy


data_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data/UUVV711b.mat')
target_area = [-10, 21, -10, 10]

def point_in_area(p, area = target_area):
    assert len(p)>1, "state should be at least 2D"
    xmin, xmax, ymin, ymax = area
    if (p[0] < xmax and p[0] >= xmin) and (p[1] < ymax and p[1] >= ymin):
        return True
    return False




class StateTransition:
    def __init__(self):
        self.actions = ['N', 'S', 'E', 'W', 'NE', 'NW', 'SE', 'SW', 'H']
        self.ROTATE_90 = 47
        self.ROTATE_45 = 24
        self.MOVE_FORWARD = 10

    def __getitem__(self, item):
        item = str(item)
        if item == 'N': return self.move_north
        elif item == 'S':return self.move_south
        elif item == 'E':return self.move_east
        elif item == 'W':return self.move_west
        elif item == 'NE':return self.move_north_east
        elif item == 'NW':return self.move_north_west
        elif item == 'SE':return self.move_south_east
        elif item == 'SW':return self.move_south_west
        else:return self.move_nowhere
    def move_north(self, state):
        for _ in range(self.MOVE_FORWARD):
            state.u = np.array([[1], [0]])
            state.update()
        state.u = np.array([[0], [0]])
        state.update()


    def move_south(self, state):
        for _ in range(2 * self.ROTATE_90):
            state.u = np.array([[0], [1]])
            state.update()

        state.u = np.array([[0], [0]])
        self.move_north(state)


    def move_east(self, state):
        for _ in range(self.ROTATE_90):
            state.u = np.array([[0], [1]])
            state.update()

        state.u = np.array([[0], [0]])
        self.move_north(state)


    def move_west(self, state):
        for _ in range(self.ROTATE_90):
            state.u = np.array([[0], [-1]])
            state.update()

        state.u = np.array([[0], [0]])
        self.move_north(state)


    def move_north_east(self, state):
        for _ in range(self.ROTATE_45):
            state.u = np.array([[0], [1]])
            state.update()

        state.u = np.array([[0], [0]])
        self.move_north(state)

    def move_south_east(self, state):
        for _ in range(self.ROTATE_90):
            state.u = np.array([[0], [-1]])
            state.update()

        state.u = np.array([[0], [0]])
        self.move_north(state)


    def move_north_west(self, state):
        for _ in range(self.ROTATE_90 + self.ROTATE_45):
            state.u = np.array([[0], [-1]])
            state.update()

        state.u = np.array([[0], [0]])
        self.move_north(state)

    def move_south_west(self, state):
        for _ in range(self.ROTATE_90 + self.ROTATE_45):
            state.u = np.array([[0], [1]])
            state.update()

        state.u = np.array([[0], [0]])
        self.move_north(state)


    def move_nowhere(self, state):
        state.u = np.array([[0], [0]])
        state.update()


class StateEstimator(StateTransition):

    def __init__(self):
        super(StateEstimator, self).__init__()
        self.xEst = np.zeros((4, 1))
        self.xTrue = np.zeros((4, 1))
        self.u = np.zeros((2, 1))
        self.PEst = np.eye(4)
        self.xDR = np.zeros((4, 1))  # Dead reckoning
        self.FF = FlowField(data_path, target_area)

    def set(self, pos):
        self.xEst[0, 0] = self.xTrue[0, 0] = self.xDR[0, 0] = pos[0]
        self.xEst[1, 0] = self.xTrue[1, 0] = self.xDR[1, 0] = pos[1]



    def update(self):
        self.xTrue, z, self.xDR, ud = observation(self.xTrue, self.xDR, self.u)
        p = self.get_position()
        if point_in_area(p):
            # compute external disturbance
            r = [self.xEst[0, 0], self.xEst[1, 0]]
            u, phi = self.FF.control_input(r)
            ku, kw = 0.001, -0.001
            w = self.xEst[2, 0] + phi + self.xEst[3, 0]
            uF = np.array([[ku * u], [kw * w]])
            # add external force to control input with process noise
            ud += uF
        self.xEst, self.Est = ekf_estimation(self.xEst, self.PEst, z, ud )
        
    def __call__(self, action):
        f = self[action]
        newState = deepcopy(self)
        f(newState)
        # print(newState, action)
        return newState
    def get_position(self):
        return np.array([self.xEst[0, 0], self.xEst[1, 0]])
    def __repr__(self):
        return "x = {:.3f} y = {:.3f}".format(self.xEst[0, 0], self.xEst[1, 0])

if __name__ == '__main__':
    X = StateEstimator()
    for _ in range(20):
        X = X('N')
        print(X)