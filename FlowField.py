import numpy as np
from math import *
import matplotlib.pyplot as plt
import scipy.io
from mpl_toolkits.mplot3d import Axes3D


#np.random.seed(88192019)
#

area = [-10, 30]
CELL_RESOLUTION = 1 #m


def transform_to_cell_index(X, feature_range):
    X_min, X_max = area
    min_, max_ = feature_range
    X_std = (X - X_min) / (X_max - X_min)
    X_scaled = X_std * (max_ - min_) + min_
    return int(X_scaled/CELL_RESOLUTION)

class FlowField(object):
    def __init__(self):
        ##extracting ROMS flow field data
        mat = scipy.io.loadmat('data/UUVV711b.mat')
        lonmesh = mat['lonmesh']
        latmesh = mat['latmesh']
        Z = np.zeros((lonmesh.shape))
        self.m, self.n = lonmesh.shape  ## m and n represents the size of lonmesh and latmesh matrices
        L = mat['Depth']
        UU = mat['UU']
        VV = mat['VV']
        self.W = np.full(lonmesh.shape, 0)
        self.goal= [-4.96416653, -4.46200365]
        for k in range(0, 1):  # K represents the depth
            self.Z = np.full(lonmesh.shape, L[k])
            self.U = np.zeros((lonmesh.shape))
            self.V = np.zeros((latmesh.shape))
            for y in range(0, self.m):
                for x in range(0,self.n):
                    self.U[y][x] = UU[1][k][y][x]   ## index 1 represents the time, k represents the depth, i and j represent the longitude and latitude
                    self.V[y][x] = VV[1][k][y][x]
        # For debugging
        # self.plot()

    def plot(self, ax):


        x, y = np.arange(self.n), np.arange(self.m)
        xv, yv = np.meshgrid(x, y, sparse=True, indexing='xy')
        ax.quiver(xv, yv, self.U, self.V, units='width')

    def __call__(self, r):
        Fx= self.U[r[0]][r[1]]
        Fy= self.V[r[0]][r[1]]
        return Fx, Fy


    def control_input(self, r):
        '''
        :param r: robot coordinate in cartesian coordinate
        Transform the cartesian coordinate to index coordinate
        Compute angular velocity from force field
        :return: external disturbance in velocities
        '''
        assert min(r)> area[0] and max(r)<area[1]
        x, y = r
        r[0] = transform_to_cell_index(r[0], (0,self.n))
        r[1] = transform_to_cell_index(r[1], (0, self.m))
        print(r, (self.m, self.n))
        Fx, Fy = self(r)
        phi = atan2(Fy, Fx)
        u = tanh(x**2 + y**2)
        return [u, phi]

if __name__ == '__main__':
    FF = FlowField()
    r = [2, 29]
    print(FF.control_input(r))
    fig = plt.figure()
    ax = fig.gca()
    FF.plot(ax)

    plt.axis('equal')
    plt.show()

