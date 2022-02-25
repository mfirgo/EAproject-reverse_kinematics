import numpy as np

def not_constrained_angles_min(size):
    return -np.ones(size)*np.pi

def not_constrained_angles_max(size):
    return np.ones(size)*np.pi

def no_back_angles_min(size):
    return -np.ones(size)*np.pi*0.5

def no_back_angles_max(size):
    return np.ones(size)*np.pi*0.5

def random_lengths(size, min_seg_len, max_seg_len):
    return np.random.uniform(low = min_seg_len, high=max_seg_len, size=size)

def random_problem(size, min_seg_len, max_seg_len):
    pass