from torch import angle
from algorytm import *
from obstacle import *
from evolution_animation import *
from reverse_kin_problem import *
from robot_arm import *

from shapely.geometry import LineString, LinearRing

class AlgoritmTest:
    def __init__(self, problem, N = 2000, T=20) -> None:
        self.algoritm = Algoritm(problem)
        self.N = N # pop size
        self.T = T # iterations
        self.number_of_offspring = 2*self.N 
        self.number_of_parents = 2
        self.start_sigma = 1
        self.tau = 1/np.sqrt(2*3)
        self.tau_0 = 1/np.sqrt(2*np.sqrt(3))

    def test(self):
        self.best_objective_value, self.best_chromosome, self.history_objective_values, self.history_best_chromosome, self.history_best_sigmas = self.algoritm.es(
            self.T, self.N, 
            self.number_of_offspring, self.number_of_parents,
            self.start_sigma, self.tau, self.tau_0)

    def print_plots(self):
        plt.figure(figsize=(18, 4))
        plt.plot(self.algoritm.log_objective_values[:, 0], 'r-')
        plt.plot(self.history_objective_values[:, 1], 'r-')
        plt.plot(self.history_objective_values[:, 2], 'r-')
        plt.xlabel('iteration')
        plt.ylabel('objective function value')
        plt.title('min/avg/max objective function values')
        plt.show()

        plt.figure(figsize=(18, 4))
        plt.plot(self.history_best_sigmas, 'r-')
        plt.xlabel('iteration')
        plt.ylabel('sigma value')
        plt.title('best sigmas')
        plt.show()

    def _get_time_frames(self):
        frames = []
        time = 0
        last_frame = None
        for i, frame in enumerate(self.algoritm.history):
            if frame['time']!= time:
                frames.append(self.algoritm.history[last_frame])
                time = frame['time']
            last_frame = i
        frames.append(self.algoritm.history[-1])
        return frames

    def save_animation(self, filename="animation", best_for_time = False):
        if best_for_time:
            frames = self._get_time_frames()
        else:
            frames = self.algoritm.history
        self.anim = EvolutionAnimation(frames, self.algoritm.problem.obstacles, self.algoritm.get_default_animation_info(), filename=filename)
        print("Saving animation...", end="")
        self.anim.animate()
        print("Done")

##################
# sample targets #
##################
keyhole_target = (4.5, 0)

def random_target(arm_length):
    target = np.random.uniform(low = -arm_length*0.9, high = arm_length*0.9, size = 2)
    return target

###############
# Sample arms #
###############
def random_lengths(size, min_seg_len, max_seg_len):
    return np.random.uniform(low = min_seg_len, high=max_seg_len, size=size)

def not_constrained_angles_min(size):
    return -np.ones(size)*np.pi

def not_constrained_angles_max(size):
    return np.ones(size)*np.pi

def no_back_angles_min(size):
    return -np.ones(size)*np.pi*0.5

def no_back_angles_max(size):
    return np.ones(size)*np.pi*0.5

def no_back_angles_min360(size):
    anglesmin = no_back_angles_min(size)
    anglesmin[0] = -np.pi
    return anglesmin

def no_back_angles_max360(size):
    anglesmax = no_back_angles_max(size)
    anglesmax[0] = np.pi
    return anglesmax

####################
# Sample obstacles #
####################
static_obstacle =  Obstacle([SingleObstacle(LinearRing([(0, 1), (1, 0), (1,1), (0,1)]), no_movement)])
keyhole_obstacle = SingleObstacle(LinearRing([(2,0), (3,0), (3,2), (6,2), (6, -5), (3, -5), (3, -3), (2, -3), (2, -2), (4, -2), (4,-4), (5, -4), (5,1), (4, 1), (4, -1), (2, -1)]), linear_movement(0, 3, 20))
obstacle1 =  SingleObstacle(LinearRing([(0.2, 1), (0.2,0) ,(1, 0), (1,1)]), linear_movement(0,-2, 20))
obstacle2 = SingleObstacle(LinearRing([(0,2), (1,2), (1,3), (0,3)]), linear_movement(3,-1.5,10))
obstacle3 = SingleObstacle(LinearRing([(-1,-2), (1, -2), (1, -3), (-1, -3)]), linear_movement(6, 0 ,26), start_time=13)
obstacle4 = SingleObstacle(LinearRing([(-1,-3), (1, -3), (1, -4), (-1, -4)]), linear_movement(-6, 0 ,26), start_time=13)
obstacle5 = SingleObstacle(LinearRing([(-2, 0), (-1, 0), (-1, -1), (-2, -1)]), linear_movement(0, 4 ,20), start_time=15)
obstacle6 = SingleObstacle(LinearRing([(-4, 0), (-3, 0), (-3, 1), (-4, 1)]), linear_movement(2, 2 ,26), start_time=0)
obstacle7 = SingleObstacle(LinearRing([(-2,-1), (-1, -1), (-1, -2), (-2, -2)]), linear_movement(-3, 4 ,30), start_time=0)
obstacle8 = SingleObstacle(LinearRing([(-1, 1), (-0.5, 1), (-0.5, -1), (-1, -1)]), linear_movement(0, -3 ,10), start_time=0)
obstacle9 = SingleObstacle(LinearRing([(-3, 1), (-3, 2), (-2, 2), (-2, 1)]), linear_movement(7, 0 ,15))
obstacle10 = SingleObstacle(LinearRing([(1, 1), (2, 1), (2, 2), (1, 2)]), linear_movement(2, 2 ,10))
obstacle11 = SingleObstacle(LinearRing([(1,-2), (1, -3.5), (3, -3.5), (3, -2)]), linear_movement(2.5, 1 ,17))


###################
# Sample problems #
###################

def single_static_obstacle():
    anglemin = [0 ,0, np.pi]
    anglemax = [1.5*np.pi, 1*np.pi, np.pi*2]
    lengths = [1,2,3]
    arm = RobotArm(lengths, anglemin, anglemax)

    problem = ReverseKinProblem(arm, 2.5, 3, static_obstacle)

    test = AlgoritmTest(problem, T = 15)
    return test

def double_moving_obstacles():
    anglemin = [-np.pi, -np.pi, -np.pi, -np.pi, -np.pi]
    anglemax = [np.pi, np.pi, np.pi, np.pi, np.pi]
    lengths = [1,1,1,1,1]
    arm = RobotArm(lengths, anglemin, anglemax)

    target = (2.5, 3)
    obstacle = Obstacle([obstacle1, obstacle2])
    problem = ReverseKinProblem(arm, target[0], target[1], obstacle)

    test = AlgoritmTest(problem, T = 50)
    test.algoritm.iteration_per_time = 2
    return test

def keyhole_obstacle_problem():
    anglemin = not_constrained_angles_min(8)
    anglemax = not_constrained_angles_max(8)
    lengths = np.ones(8)
    arm = RobotArm(lengths, anglemin, anglemax)
    
    target = keyhole_target
    obstacle = Obstacle([keyhole_obstacle])
    problem = ReverseKinProblem(arm, target[0], target[1], obstacle)

    test = AlgoritmTest(problem, T = 200)
    test.algoritm.iteration_per_time = 10
    return test

def multi_obstacle_random_target():
    anglemin = not_constrained_angles_min(5)
    anglemax = not_constrained_angles_max(5)
    lengths = random_lengths(5, 0.5, 3)
    arm = RobotArm(lengths, anglemin, anglemax)

    target = random_target(lengths.sum())
    obstacle = Obstacle([obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6, obstacle7, obstacle8, obstacle9, obstacle10, obstacle11])
    problem = ReverseKinProblem(arm, target[0], target[1], obstacle)

    test = AlgoritmTest(problem, T = 200)
    return test

def multi_obstacle_set_target(target):
    anglemin = not_constrained_angles_min(5)
    anglemax = not_constrained_angles_max(5)
    lengths = random_lengths(5, 0.5, 3)
    arm = RobotArm(lengths, anglemin, anglemax)

    obstacle = Obstacle([obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6, obstacle7, obstacle8, obstacle9, obstacle10, obstacle11])
    problem = ReverseKinProblem(arm, target[0], target[1], obstacle)

    test = AlgoritmTest(problem, T = 200)
    return test