import random
import numpy as np
from copy import deepcopy
import matplotlib.pyplot as plt


area =[29,0,20,0]
goal = np.array([20,12])
def rollout(x,y, n):
    traj=[]
    for i in range(n):
        #step= random.choice(['N','S', 'E', 'W', 'NE', 'NW', 'SE', 'SW', 'H'])
        step = random.choice(['N', 'S', 'E', 'W','NE', 'NW', 'SE', 'SW'])
        if step=='N':
            x,y= x, y + 1
        elif step=='S':
            x, y = x, y - 1
        elif step == 'E':
            x, y = x+1, y
        elif step == 'W':
            x, y = x-1, y
        elif step == 'NE':
            x, y = x + 1, y + 1
        elif step == 'NW':
            x, y = x - 1, y + 1
        elif step == 'SE':
            x, y = x + 1, y - 1
        elif step == 'SW':
            x, y = x - 1, y - 1
        #elif step == 'H':
        #    x, y = x, y
        traj.append(np.array([x,y]))
    return traj

def point_in_area(p, area):
    xmax,xmin,ymax,ymin=area
    if (p[0]<xmax and p[0]>=xmin) and (p[1]<ymax and p[1]>=ymin):
        return True
    return False

def dist (p, goal):
    return np.linalg.norm(goal-p)

def get_avg_reward(traj):
    reward = 0
    for p in traj:
        if point_in_area(p,area):
            reward += 1.0/(1.0+dist(p,goal))
        elif dist(p,goal)<1:
            reward += 100
        else:
            reward -= 100
    return reward

def nextstep(p):
    totalreward = {}
    for i in range(2500):
        x, y = deepcopy(p)
        traj = rollout(x, y, 20)
        totalreward[get_avg_reward(traj)]=np.array(traj[0])
    maxreward= max(totalreward.keys())
    return totalreward.get(maxreward)


if __name__ == '__main__':
    p=[0,0]
    x, y = [], []

    for i in range(50):
        p=nextstep(p)
        print (p)
        x.append(p[0])
        y.append(p[1])

    plt.scatter(goal[0],goal[1])
    plt.plot(x,y)
    plt.show()





