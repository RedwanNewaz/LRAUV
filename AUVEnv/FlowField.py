import numpy as np
from math import *
import scipy.io
from sklearn.preprocessing import MinMaxScaler



class FlowField(object):
    def __init__(self, data_path, target_area):
        self.target_area = target_area
        ##extracting ROMS flow field data
        mat = scipy.io.loadmat(data_path)
        lonmesh = mat['lonmesh']
        latmesh = mat['latmesh']
        Z = np.zeros((lonmesh.shape))
        self.set_scaler(lonmesh)
        L = mat['Depth']
        UU = mat['UU']
        VV = mat['VV']
        self.W = np.full(lonmesh.shape, 0)
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
    def set_scaler(self, lonmensh):
        target_area = self.target_area
        self.m, self.n = lonmensh.shape
        bounding_box = [
            [target_area[0], target_area[2]],
            [self.n + target_area[0], self.m + target_area[2]]
        ]
        self.scaler = MinMaxScaler()
        self.scaler.fit(bounding_box)
        self.xyScale = np.array([self.n + target_area[0], self.m + target_area[2]])
    def transform(self, r):
        x = np.array([r])
        x = self.scaler.transform(x)
        x *= self.xyScale
        return list(map(int, x[0]))

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
        assert min(r)> self.target_area[0] and max(r)<self.target_area[1]
        x, y = r
        r = self.transform(r)
        Fx, Fy = self(r)
        phi = atan2(Fy, Fx)
        u = tanh(x**2 + y**2)
        return [u, phi]

