import json
import sys
import numpy as np
from matplotlib import pyplot as plt

path = "markers.json"
path2 = "poles.json"
poses = "/home/luzhengye/dataset/02/02.txt"
with open(path, "r") as f:
    res: dict = json.load(f)

with open(path2, "r") as f:
    pole: dict = json.load(f)

tx, ty, tz = [], [], []
i = 0
with open(poses, "r") as f:
    line = f.readline()
    while line:
        if i > 55:
            break
        if i % 5 == 0:
            t = line.split()
            tx.append(float(t[3]))
            ty.append(float(t[7]))
            tz.append(float(t[11]))

        line = f.readline()
        i += 1


fig = plt.figure()
ax = plt.axes(projection='3d')
ax.view_init(elev=20, azim=45)
ax.set_xlabel('$x$', fontsize=15)
ax.set_ylabel('$y$', fontsize=15)
ax.set_zlabel('$z$', fontsize=15)
# ax.set_aspect('equal')

for group_id, val in res.items():
    x, y, z = [], [], []
    for i in range(len(val)):
        x.append(val[str(i)][0])
        y.append(val[str(i)][1])
        z.append(val[str(i)][2])
    x.append(val["0"][0])
    y.append(val["0"][1])
    z.append(val["0"][2])
    ax.plot3D(x, y, z, c="red")

for group_id, val in pole.items():
    if group_id == "6":
        continue
    x, y, z = [], [], []
    for i in range(len(val)):
        x.append(val[str(i)][0])
        y.append(val[str(i)][1])
        z.append(val[str(i)][2])
    ax.plot3D(x, y, z, c="green")


ax.plot3D(tx, ty, tz, c="blue")

# draw bbox of this task
ax.plot3D([-25, 25], [0, 20], [0, 100], c="white")

plt.show()
