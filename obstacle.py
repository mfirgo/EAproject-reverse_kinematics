import shapely.affinity
import numpy as np

class SingleObstacle:
    def __init__(self, starting_obj, movement, start_time = 0):
        self.starting_obj = starting_obj
        self.current_obj = starting_obj
        self.movement = movement
        self.start_time = start_time
    
    def move(self, time):
        self.current_obj = self.movement(self.starting_obj, time+self.start_time)    

    def get_obstacle(self):
        return self.current_obj 

    def get_plot_points(self, time):
        x, y = self.movement(self.starting_obj, time+self.start_time).xy
        return x, y
    
    def plot(self, ax, color='red'):
        x, y = self.current_obj.xy
        obs = ax.plot(x, y, color = color)
        return obs

    def intersects(self, individual):
        return self.current_obj.intersects(individual)

class Obstacle:
    def __init__(self, obstacle_array):
        self.obstacle_array : list[SingleObstacle] = obstacle_array

    def move(self, time):
        for obstacle in self.obstacle_array:
            obstacle.move(time)
    
    def get_obstacle(self):
        all_obstacles = self.obstacle_array[0].current_obj
        for obstacle in self.obstacle_array:
            all_obstacles = all_obstacles.union(obstacle.current_obj)
        return all_obstacles
    
    def plot(self, ax, color='red'):
        for obstacle in self.obstacle_array:
            ax = obstacle.plot(ax, color=color)
        return ax

    def intersects(self, individual):
        for obstacle in self.obstacle_array:
            if obstacle.intersects(individual):
                return True
        return False

# T - time for going one way
def linear_movement(x, y, T):
    def mov(obj, time):
        movement_amount = 0.5 - np.cos(time/(T)*np.pi)/2
        new_obj = shapely.affinity.translate(obj, x*movement_amount, y*movement_amount)
        return new_obj
    return mov
        
def no_movement(obj, time):
    return obj