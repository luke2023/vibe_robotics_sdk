import numpy as np

def rad2step(x):
    return 4096 / (2 * np.pi) * x
def step2rad(x):
    return (2 * np.pi) / 4096 * x - np.pi