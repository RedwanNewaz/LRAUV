import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
from Env import target_area, robot_in_area
from matplotlib.collections import PatchCollection
from random import choice
from pprint import pprint
import math
'''
This idea is borrowed from
https://www.youtube.com/watch?v=789weryntzM
'''
NUM_OBSTACLES = 5
OBSTACLE_RADIUS = 0.5
def get_dist(p, q):
    p, q = np.array(p), np.array(q)
    return np.linalg.norm(p-q)



class Obstacles(object):
    def __init__(self, p ):
        self.r = OBSTACLE_RADIUS
        self.x = p[0]
        self.y = p[1]
        self.color =  np.random.uniform(size=3)
        self.vx = 0.5 * (np.random.rand() - 0.5)
        self.vy = 0.5 * (np.random.rand() - 0.5)


    def get_patch(self):
        """
        :return: matpotlib circle
        """
        obj = Circle((self.x, self.y), self.r)
        obj.set_color(self.color)
        return obj

    @staticmethod
    def rotate(vx, vy, angle):
        ux = vx * math.cos(angle) - vy*math.sin(angle)
        uy = vx * math.sin(angle) + vy*math.cos(angle)
        return (ux, uy)


    def ressolve_collision(self, other):
        ax, ay = self.vx - other.vx, self.vy - other.vy
        dx , dy = other.x - self.x, other.y - self.y
        if(ax* dx + ay* dy >= 0):
            angle = - math.atan2(dy, dx)
            u1 = self.rotate(self.vx, self.vy, angle)
            u2 = self.rotate(other.vx, other.vy, angle)
            v1 = self.rotate(u2[0], u1[1], -angle)
            v2 = self.rotate(u1[0], u2[1], -angle)
            self.vx, self.vy = v1[0], v1[1]
            other.vx, other.vy = v2[0], v2[1]


    def __sub__(self, other):
        return get_dist([self.x, self.y], [other.x, other.y]) < 2*self.r

    def __eq__(self, other):
        return (self.x == other.x) and (self.y == other.y)
    def __ne__(self, other):
        return (self.x != other.x) and (self.y != other.y)

    def update(self, particles):
        for p in particles:
            if(self == p): continue
            if(self - p):
                self.ressolve_collision(p)
        if( (self.x - self.r) <= target_area[0] or (self.x + self.r) > target_area[1] ):
            self.vx *= -1
        if( (self.y - self.r) <= target_area[2] or (self.y + self.r) > target_area[3] ):
            self.vy *= -1
        self.x += self.vx
        self.y += self.vy
        return self.get_patch()

    def __repr__(self):
        return "({:.3f}, {:.3f})".format(self.x, self.y)


def random_particles(r, particles = []):
    x = np.random.uniform(target_area[0] + r, target_area[1] - r)
    y = np.random.uniform(target_area[2] + r, target_area[3] - r)
    collision = False
    q = (x, y)
    for p in particles:
        if(get_dist(p, q)< 2*r):
            collision = True
            break
    if(collision): random_particles(r, particles)
    particles.append(q)


if __name__ == '__main__':

    fig, ax = plt.subplots()
    particles = []
    for _ in range(NUM_OBSTACLES):
        random_particles(OBSTACLE_RADIUS, particles)
    # pprint(particles)
    obstacles = [Obstacles(p) for p in particles]
    while True:
        ax.cla()
        for obj in obstacles:
            ax.add_patch(obj.update(obstacles))
        plt.axis(target_area)
        plt.pause(0.1)
