import numpy as np

area =[29,0,20,0]

def point_in_area(p):
    xmax,xmin,ymax,ymin=area
    if (p[0]<xmax and p[0]>=xmin) and (p[1]<ymax and p[1]>=ymin):
        return True
    return False

def get_psuedo_goal(goal, size):
    goal_x = np.random.uniform(goal[0] - size, goal[0] + size)
    goal_y = np.random.uniform(goal[1] - size, goal[1] + size)
    pseudo_goal = np.array([goal_x, goal_y])
    return pseudo_goal


class Action():
    def __init__(self, premitive):
        self.premitive = premitive


    def __str__(self):
        return self.premitive

    def __repr__(self):
        return str(self)

    def __eq__(self, other):
        return self.__class__ == other.__class__ and self.premitive == other.premitive

    def __hash__(self):
        return hash(self.premitive)