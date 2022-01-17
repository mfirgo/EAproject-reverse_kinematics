import numpy as np
import matplotlib.pyplot as plt

class RobotArm():
    def __init__(self, lengths, anglesmin, anglesmax) -> None:
        self.lengths = np.array(lengths, dtype=float)
        self.anglesmin = np.array(anglesmin, dtype=float)
        self.anglesmax = np.array(anglesmax, dtype=float)
        self.startx = 0
        self.starty = 0
    
    def print_info(self):
        print(self.lengths)
        print(self.anglesmin)
        print(self.anglesmax)

    def is_legal_mask(self, angles):
        return np.logical_and(self.anglesmin <= angles, angles <= self.anglesmax)

    def is_legal(self, angles):
        return np.all(self.anglesmin <= angles) and np.all(angles <= self.anglesmax)
    
    def get_random(self):
        return np.random.uniform(low=self.anglesmin, high=self.anglesmax, size=self.anglesmin.size)

    def correct_to_edges(self, angles):
        np.copyto(angles, self.anglesmin, where = self.anglesmin > angles)
        np.copyto(angles, self.anglesmax, where = self.anglesmax < angles)
        return angles
    
    def correct_random(self, angles):
        mask = np.logical_not(self.is_legal_mask(angles))
        angles[mask] = np.random.uniform(low=self.anglesmin[mask], high=self.anglesmax[mask], size= self.anglesmin[mask].size)
        return angles

    def get_points(self, angles):
        X = [self.startx]
        Y = [self.starty]

        x = self.startx
        y = self.starty
        
        # I believe performance of this code can be better
        # (perhaps compute vector for each segment?)
        for a, l in zip(angles, self.lengths):
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