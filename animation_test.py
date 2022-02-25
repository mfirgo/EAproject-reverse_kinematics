from shapely.geometry import LinearRing, LineString
import shapely.affinity
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation


class SingleObstacle:
    def __init__(self, starting_obj, movement):
        self.starting_obj = starting_obj
        self.current_obj = starting_obj
        self.movement = movement
    
    def move(self, time):
        self.current_obj = self.movement(self.starting_obj, time)    

    def get_obstacle(self):
        return self.current_obj  
    
    def plot(self, ax, color='red'):
        x, y = self.current_obj.xy
        obs = ax.plot(x, y, color = color)
        return obs
# T - time for going one way
def linear_movement(x, y, T):
    def mov(obj, time):
        movement_amount = 0.5 - np.cos(time/(T)*np.pi)/2
        new_obj = shapely.affinity.translate(obj, x*movement_amount, y*movement_amount)
        return new_obj
    return mov

test = SingleObstacle(LinearRing([(1,1), (1,0), (0,0), (0,1)]), linear_movement(1, 0, 20))
def next_frame(i, obstacle):
    test.move(i)
    #print(i)
    x, y = test.get_obstacle().xy
    obstacle.set_data(x, y) # "punkty z plota"
    return obstacle,


fig, ax = plt.subplots()
ax.set(xlim=(-1, 4), ylim=(-1, 4))
#obstacle, = test.plot(ax)
x, y = test.get_obstacle().xy
obstacle = ax.plot(x, y)



anim = animation.FuncAnimation(fig, next_frame, interval=100, fargs=(obstacle), save_count=200)
plt.show()


f = r"animation.mp4" 
writervideo = animation.FFMpegWriter(fps=60) 
anim.save(f, writer=writervideo)

