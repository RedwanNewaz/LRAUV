from copy import deepcopy
import numpy as np

class StateTransition:
    def __init__(self):
        self.pos = np.array([0, 0])
        self.actions = ['N', 'S', 'E', 'W', 'NE', 'NW', 'SE', 'SW', 'H']
    def __getitem__(self, item):
        # print('getting item ', item)
        item = str(item)
        if item == 'N':   return [0, 1]
        elif item == 'S': return [0, -1]
        elif item == 'E': return [1, 0]
        elif item == 'W': return [-1, 0]
        elif item == 'NE':return [1, 1]
        elif item == 'NW':return [-1, 1]
        elif item == 'SE':return [1, -1]
        elif item == 'SW':return [-1, -1]
        elif item == 'H' :return [0, 0]
        else: raise RuntimeError('item not found')

class StateEstimator(StateTransition):
    def __init__(self):
        super(StateEstimator, self).__init__()

    def __add__(self, other):
        self.pos += np.array(other)
        return self

    def set(self, pos):
        self.pos = pos

    def __repr__(self):
        return "x = {:.3f} y = {:.3f}".format(self.pos[0], self.pos[1])

    def __call__(self, action):
        f = self[action]
        newState = deepcopy(self)

        # print(newState, action, f)
        newState = newState + f
        return newState

    def get_position(self):
        return self.pos