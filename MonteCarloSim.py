import random
import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt
from numpy.linalg import norm
from treelib import Node, Tree




area =[29,0,20,0]
array_shape = np.array([29, 20])
goal = np.array([20,12])

def get_psuedo_goal(size):
    goal_x = np.random.uniform(goal[0] - size, goal[0] + size)
    goal_y = np.random.uniform(goal[1] - size, goal[1] + size)
    pseudo_goal = np.array([goal_x, goal_y])
    return pseudo_goal

HORIZON = 20
NUM_SAMPLES = 200
DISCOUNT_FACTOR = 0.95

AVAILABLE_ACTIONS = {
    'N': (0, 1),
    'S': (0, -1),
    'E': (1, 0),
    'W':  (-1, 0),
    'NE': (1,1),
    'NW': (-1, 1),
    'SE': (1, -1),
    'SW': (-1, -1),
    'H': (0, 0)
}


def rollout(x,y, n):
    for i in range(n):
        step= random.choice(['N','S', 'E', 'W', 'NE', 'NW', 'SE', 'SW', 'H'])
        dx, dy = AVAILABLE_ACTIONS[step]
        x, y = x + dx , y + dy
        yield [np.array([x,y]), step]

def point_in_area(p, area):
    xmax,xmin,ymax,ymin=area
    if (p[0]<xmax and p[0]>=xmin) and (p[1]<ymax and p[1]>=ymin):
        return True
    return False

def get_avg_reward(traj):
    reward = 0
    for i, pa in enumerate(traj):
        p, a = pa
        step_reward = -100
        if point_in_area(p,area):
            dist = norm(p - goal) # goal is partially observable
            size = np.exp(dist / 20.0)
            pseudoG = get_psuedo_goal(size)
            step_reward = 1.0/(1.0+norm(p - pseudoG) )
        elif norm(p - goal)<1:
            step_reward = 100
        reward += step_reward*DISCOUNT_FACTOR**i
    return reward

def nextstep(p):
    totalreward = {}
    for i in range(NUM_SAMPLES):
        x, y = deepcopy(p)
        traj = list(rollout(x, y, HORIZON))
        totalreward[get_avg_reward(traj)]= traj[0]
    r = max(totalreward.keys())
    s, a = totalreward.get(r)
    return s, a , r

def sub2ind(rows, cols):
    ind = rows*array_shape[1] + cols
    return ind

def ind2sub(ind):
    ind[ind < 0] = -1
    ind[ind >= array_shape[0]*array_shape[1]] = -1
    rows = (ind.astype('int') / array_shape[1])
    cols = ind % array_shape[1]
    return (rows, cols)

class sar(object):
    def __init__(self, s, a = 'H', r = 0, next = None):
        self.s = s
        self.a = a
        self.r = r
        self.next = next
    def __str__(self):
        return "{} {} {}".format(self.s, self.a, self.r)
    def __repr__(self):
        return str(self)

def search(X, Y, s, tree, parent):
    count = 0
    while norm(s-goal)>1:
        s, a, r =nextstep(s)
        node_name = str(sub2ind(s[0], s[1]))
        if(not tree.get_node(node_name)):
            tree.create_node(node_name, node_name, parent=parent, data=sar(s, a, r))
        else:
            old_node = tree.get_node(node_name)
            if(old_node.data.a != a):
                old_node.data.next = sar(s, a, r)
                # print(old_node.data.a, a )
        parent = node_name
        X.append(s[0])
        Y.append(s[1])
        count += 1
        if(count > 500):break


if __name__ == '__main__':
    s=[0,0]
    X, Y = [], []
    tree = Tree()
    node = str(sub2ind(s[0], s[1])) # name
    tree.create_node(node, node,  data=sar(s))
    search(X, Y, s, tree, node)
    # for _ in range(10):
    #     search(X, Y, s, tree, node)
    # tree.show()
    plt.plot(X, Y)
    plt.show()





