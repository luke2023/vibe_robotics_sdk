from classes import *
from typing import List

def generate_footsteps(distance, step_length, foot_spread, initial_y=0.) -> List[Footstep]:
    footsteps = []
    
    footsteps.append(Footstep(x=+foot_spread, y=initial_y))
    footsteps.append(Footstep(x=-foot_spread, y=initial_y))
    
    x = foot_spread
    y = initial_y
    
    while y < distance:
        if distance - y < step_length:
            y += min(distance - y, 0.5 * step_length)
        else:
            y += step_length
        x = -x
        footsteps.append(Footstep(x=x, y=y))
    footsteps.append(Footstep(x=-x, y=y))
    return footsteps

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    footsteps = generate_footsteps(5.0, 0.3, 0.1)
    xs = [fs.x for fs in footsteps]
    ys = [fs.y for fs in footsteps]
    plt.scatter(xs, ys)
    plt.plot(xs, ys)
    plt.axis('equal')
    plt.show()