from ActionSet import Action, get_psuedo_goal
import numpy as np
from numpy.linalg import norm
from AUVEnv import  point_in_area, goal
from copy import deepcopy

from AUVEnv import StateEstimator



class Simulator(object):
    def __init__(self, pos=np.array([0, 0])):
        self.pos = pos
        self.state = StateEstimator()
        self.state.set(pos)


    def __add__(self, other):
        assert isinstance(other, tuple)
        self.pos += np.array(other)
        return self

    def __repr__(self):
        return "x = {:.3f} y = {:.3f} | gt {}".format(self.pos[0], self.pos[1], goal.tolist())

    def getPossibleActions(self):
        possibleActions = [Action(k) for k in self.state.actions]
        # possibleActions = [Action(k) for k in AVAILABLE_ACTIONS.keys()]
        validActions = []
        for action in possibleActions:
            newState = self.takeAction(action)
            if(point_in_area(newState.pos)):
                validActions.append(action)
        # assert len(validActions) > 0 , "No valid action found"
        return validActions

    def takeAction(self, action):
        newState = deepcopy(self)
        newState.state = newState.state(action)
        newState.pos = newState.state.get_position()
        # newState = newState + AVAILABLE_ACTIONS.get(action.premitive)
        return newState

    def isTerminal(self):
        terminate = np.linalg.norm(self.pos - goal) <= 1.5
        if terminate:
            print('goal found @ ', self)
        return terminate

    def getReward(self):
        p = self.pos
        if point_in_area(p):
            dist = norm(p - goal)  # goal is partially observable
            size = np.exp(dist / 20.0)
            pseudoG = get_psuedo_goal(goal, size)
            return 1.0 / (1.0 + norm(p - pseudoG))
        elif norm(p - goal) < 1:
            return 100
        return False
