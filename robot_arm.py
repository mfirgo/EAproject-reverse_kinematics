import numpy as np
import matplotlib.pyplot as plt

class RobotArm():
    def __init__(self, lengths, anglesmin, anglesmax) -> None:
        self.lengths = np.array(lengths, dtype=float)
        self.anglesmin = np.array(anglesmin, dtype=float)
        self.anglesmax = np.array(anglesmax, dtype=float)
        self.startx = 0
        self.starty = 0
        self.segments_num = self.lengths.size
    
    def print_info(self):
        print(self.lengths)
        print(self.anglesmin)
        print(self.anglesmax)
    
    def get_default_animation_info(self):
        arm_length = self.lengths.sum()
        margin = arm_length*0.05
        animation_info = {'start_point': (self.startx, self.starty)}
        animation_info['xlim'] = (self.startx - arm_length-margin, self.startx + arm_length+margin)
        animation_info['ylim'] = (self.starty - arm_length-margin, self.starty + arm_length+margin)
        return animation_info

    def is_legal_mask(self, angles):
        return np.logical_and(self.anglesmin <= angles, angles <= self.anglesmax)

    def is_legal(self, angles):
        return np.all(self.anglesmin <= angles) and np.all(angles <= self.anglesmax)
    
    def get_random(self, N):
        return np.random.uniform(low=self.anglesmin, high=self.anglesmax, size=(N, self.anglesmin.size))

    def correct_to_edges(self, angles):
        np.copyto(angles, self.anglesmin, where = self.anglesmin > angles)
        np.copyto(angles, self.anglesmax, where = self.anglesmax < angles)
        return angles
    
    def correct_random(self, angles):
        mask = np.logical_not(self.is_legal_mask(angles))
        angles[mask] = np.random.uniform(low=self.anglesmin[mask], high=self.anglesmax[mask], size= self.anglesmin[mask].size)
        return angles

    def get_points_population(self, angles_pop):
        angles_pop = np.cumsum(angles_pop, axis = 1)
        cosines = np.c_[np.zeros(angles_pop.shape[0]),np.cos(angles_pop)*self.lengths]
        sines = np.c_[np.zeros(angles_pop.shape[0]),np.sin(angles_pop)*self.lengths]
        X = np.cumsum(cosines, axis=1)
        Y = np.cumsum(sines, axis=1)
        return X, Y

    def get_points(self, angles):
        X = [self.startx]
        Y = [self.starty]

        x = self.startx
        y = self.starty
        
        a = 0
        for cur_ang, l in zip(angles, self.lengths):
            a += cur_ang % (2*np.pi)
            x = x + np.cos(a)*l
            y = y + np.sin(a)*l
            X.append(x)
            Y.append(y)
        return X, Y

    def draw(self, angles):
        X, Y = self.get_points(angles)
        plt.scatter(X, Y)
        plt.plot(X, Y)
        plt.show()

    def draw_from_points(self, X, Y):
        plt.scatter(X, Y)
        plt.plot(X, Y)
        plt.show()
