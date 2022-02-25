from robot_arm import RobotArm
import numpy as np
from shapely.geometry import LineString
from obstacle import Obstacle

class ReverseKinProblem:
    def __init__(self, robot_arm : RobotArm, targetX, targetY, obstacles, max_time = 100) -> None:
        self.robot_arm : RobotArm = robot_arm
        self.targetX = targetX
        self.targetY = targetY
        self.obstacles = obstacles ## single geometry object or obstacle class instance
        self.max_time = max_time
    
    def plot_obstacles(self, ax, color = "red"):
        if isinstance(self.obstacles, Obstacle):
            obs = self.obstacles.plot(ax, color=color)
        else:
            try:
                x, y = self.obstacles.xy
                obs = ax.plot(x, y, color = color)
            except AttributeError:
                print('no wiem')
        return obs

    def plot_individual(self, ax, x, y, color="blue"):
        arm_joints = ax.plot(x, y, 'o', color = color)
        arm_segments = ax.plot(x, y, color = color)
        return arm_joints, arm_segments
    
    def get_random_population(self, N):
        return self.robot_arm.get_random(N)

    # opcja1 : njit      - colliding_idx = np.zeros(N)
    # opcja2 : vectorise - zrobić jako funkcję bez for na x, y (pojedynczych wierszach)
    # @njit  : nie działa bo Linestring
    def get_colliding_idx(self, X, Y):
        #colliding_idx = []
        colliding_idx = np.zeros(X.shape[0], dtype=bool)
        for i, coords in enumerate(zip(X, Y)):
            x, y = coords
            individual = LineString(zip(x, y))
            if self.obstacles.intersects(individual):
                #colliding_idx.append(i)
                colliding_idx[i] = True
        return colliding_idx

    def correct_population(self, population):
        return self.robot_arm.correct_to_edges(population)
        #return self.robot_arm.correct_random(population)

    def evaluate_population(self, population):
        X, Y = self.robot_arm.get_points_population(population)
        score = np.square(X[:,-1] - self.targetX) + np.square(Y[:, -1]-self.targetY)
        return -score, X, Y
    
    def get_default_animation_info(self):
        animation_info = self.robot_arm.get_default_animation_info()
        animation_info['target_point'] = (self.targetX, self.targetY)
        return animation_info
    