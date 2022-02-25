import numpy as np
from obstacle import Obstacle # *
import matplotlib.pyplot as plt
from matplotlib import animation

from robot_arm import RobotArm

class EvolutionAnimation:
    def __init__(self, frames, obstacle:Obstacle, plot_info, filename = "animation", by_time = False):
        self.filename = filename
        self.by_time = by_time
        self.frames = frames
        self.obstacle = obstacle
        self.target_point = plot_info['target_point']
        self.start_point = plot_info['start_point']
        self.plot_info = plot_info
        self.obstacle_artists = []

        self.fig, self.axes = plt.subplots(2, 1,gridspec_kw={'height_ratios': [3, 1]} )
        self.ax, self.bx = self.axes
        self.ax.set(xlim=plot_info['xlim'], ylim=plot_info['ylim'])
        self.ax.set_title(plot_info['title'])

        # draw start and target
        self.ax.plot(self.start_point[0], self.start_point[1], 'bo', markersize = 10)
        self.ax.plot(self.target_point[0], self.target_point[1], 'go', markersize = 10)
        
        self.objective_x = []
        self.objective_y = []
        self.objective_values_plot, =self.bx.plot([], [], color='black')

        self.bottom_text = self.ax.text(plot_info['xlim'][0],plot_info["ylim"][0],"")
        self.arm_segments, = self.ax.plot([], [], color = 'blue')
        self.arm_joints, = self.ax.plot([], [], 'o',color = 'blue')

        if obstacle:
            for o in obstacle.obstacle_array:
                obs_artist, =self.ax.plot([], [], color='red')
                self.obstacle_artists.append(obs_artist)

    def update(self, i):
        current_frame = self.frames[i]
        time = current_frame['time']
        self.bottom_text.set_text(self._get_text(current_frame))
        self.objective_x.append(current_frame['time' if self.by_time else 'iteration'])
        self.objective_y.append(current_frame['best_true_value'])
        #self.objective_values_plot.set_data(self.objective_x, self.objective_y)
        self.bx.plot(self.objective_x, self.objective_y, color='red' if current_frame['best_collides'] else 'blue', alpha = 0.5)
        self.bx.plot(current_frame['time' if self.by_time else 'iteration'], current_frame['best_true_value'], 'o', color='red' if current_frame['best_collides'] else 'blue', markersize = 4 )
        self.arm_segments.set_data(current_frame['bestX'], current_frame['bestY'])
        self.arm_joints.set_data(current_frame['bestX'], current_frame['bestY'])
        if current_frame['best_collides']:
            self.arm_segments.set_color('m')
            self.arm_joints.set_color('m')
        else:
            self.arm_segments.set_color('blue')
            self.arm_joints.set_color('blue')
        for obs_artist, obs in zip(self.obstacle_artists, self.obstacle.obstacle_array):
            x, y = obs.get_plot_points(time)
            obs_artist.set_data(x, y)
        return self.bottom_text, self.arm_segments, self.arm_joints, *self.obstacle_artists

    def animate(self):
        self.anim = animation.FuncAnimation(self.fig, self.update, frames=range(len(self.frames)), blit=True)
        full_filename = f"animations/{self.filename}.mp4"
        #f = full_filename # r"animation.mp4"
        writervideo = animation.FFMpegWriter(fps=6)
        self.anim.save(full_filename, writer=writervideo)


    def _get_text(self, frame):
        frame_text = ""
        for key in self.plot_info['text_keys']:
            frame_text += f"{key}: {frame[key]}\n"
        return frame_text #generation:, total individuals, best sigma, distance

# function for testing animation
def create_mock_frames(number_of_frames, robot_arm: RobotArm):
    frames = []
    random_pop = robot_arm.get_random(number_of_frames)
    X, Y = robot_arm.get_points_population(random_pop)
    for i in range(number_of_frames):
        x, y = X[i], Y[i]
        frames.append({'time': i, 'X':x, 'Y': y})
    return frames