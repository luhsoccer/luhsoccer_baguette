import random

MIN_X = -1.5
MAX_X = 1.5
MIN_Y = -1.0
MAX_X = 1.0

num_of_robots = 6
num_of_cycles = 20
num_of_poses = num_of_robots *num_of_cycles

poses = "{"
for i in range(num_of_poses):
    x = random.uniform(MIN_X,MAX_X)
    y = random.uniform(MIN_Y,MAX_X)
    poses += "{" + "{:.3f},{:.3f}".format(x,y) + "},"

poses += "}"
print(poses)