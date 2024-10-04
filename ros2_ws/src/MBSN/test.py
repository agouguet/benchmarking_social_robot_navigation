import math
import matplotlib.pyplot as plt
import numpy as np

def f_alpha(x, y, alpha=1.0):
    dnear = (np.linalg.norm(np.array((0, 0, 0)) - np.array((x, y, 0)))) + 10e-6
    return 1-np.exp(-alpha*dnear)

def f_limit(x, y, limit=1.0):
    dnear = ((np.linalg.norm(np.array((0, 0, 0)) - np.array((x, y, 0)))) + 10e-6) / limit
    return 1-np.exp(-dnear)

def f(x, y, alpha=1.0, limit=1.0):
    dist = np.linalg.norm(np.array((0, 0, 0)) - np.array((x, y, 0)))# + 10e-6
    return f_with_dist(dist, alpha=alpha, limit=limit)
    # return np.exp(-alpha*(dist/limit))
    # return np.where(dist >= limit, 1.0, np.exp(-alpha*(dist)))
    # return 1-(1/(1+np.exp(-alpha*(dist-limit))))
    # return 1-np.exp(-alpha*(dist-limit))
    # dnear = np.where(dist >= limit, 1.0, dist/limit)
    # return 1-np.exp(-alpha*dnear)
    # return np.where(dist >= limit, 1.0, 1-np.exp(-alpha*(dist/limit)))
    # return 1-np.exp(-alpha*dist)

def f_with_dist(dist, alpha=1.0, limit=1.0):
    return np.where(dist >= limit, 1.0, dist/limit * np.exp(-alpha*(limit-dist)**2))

DETAILS = 200
LIMIT_AXES = 1.5
LOW_FONT_SIZE = 24
FONT_SIZE = 32

# set up a figure twice as wide as it is tall
# fig, axs = plt.subplots(3, 3, subplot_kw={"projection": "3d"}, figsize=(10, 5))
fig = plt.figure(figsize=plt.figaspect(2.))

limit = 1.2

ax = fig.add_subplot(2, 3, 1, projection='3d')
r = np.linspace(-LIMIT_AXES, LIMIT_AXES, DETAILS)
X, Y = np.meshgrid(r, r)
Z = f(X, Y, alpha=0.1, limit=limit)
ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap='viridis', edgecolor='none')

ax.set_xlabel('Robot x', fontsize=LOW_FONT_SIZE)
ax.set_ylabel('Robot y', fontsize=LOW_FONT_SIZE)
ax.set_zlabel('$score_{near}$', fontsize=LOW_FONT_SIZE)
ax.set_zlim(0, 1)
ax.set_title("Human Personal Space=1.2; $\\alpha=0.1$ \n Human Position = (0.0, 0.0)", fontsize=FONT_SIZE)


ax = fig.add_subplot(2, 3, 2, projection='3d')
r = np.linspace(-LIMIT_AXES, LIMIT_AXES, DETAILS)
X, Y = np.meshgrid(r, r)
Z = f(X, Y, alpha=1.0, limit=limit)
ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap='viridis', edgecolor='none')

ax.set_xlabel('Robot x', fontsize=LOW_FONT_SIZE)
ax.set_ylabel('Robot y', fontsize=LOW_FONT_SIZE)
ax.set_zlabel('$score_{near}$', fontsize=LOW_FONT_SIZE)
ax.set_zlim(0, 1)
ax.set_title("Human Personal Space=1.2; $\\alpha=1.0$ \n Human Position = (0.0, 0.0)", fontsize=FONT_SIZE)


ax = fig.add_subplot(2, 3, 3, projection='3d')
r = np.linspace(-LIMIT_AXES, LIMIT_AXES, DETAILS)
X, Y = np.meshgrid(r, r)
Z = f(X, Y, alpha=10.0, limit=limit)
ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap='viridis', edgecolor='none')

ax.set_xlabel('Robot x', fontsize=LOW_FONT_SIZE)
ax.set_ylabel('Robot y', fontsize=LOW_FONT_SIZE)
ax.set_zlabel('$score_{near}$', fontsize=LOW_FONT_SIZE)
ax.set_zlim(0, 1)
ax.set_title("Human Personal Space=1.2; $\\alpha=10.0$ \n Human Position = (0.0, 0.0)", fontsize=FONT_SIZE)


ax = fig.add_subplot(2, 3, 4)
x = np.linspace(0, LIMIT_AXES, 60)
y = f_with_dist(x, alpha=0.1, limit=limit)
ax.plot(x, y, label="$score_{near}$")
ax.axvline(x = limit, color = 'r', label = "Human Personal Space")
ax.set_xlabel('Distance Human-Robot', fontsize=FONT_SIZE)
ax.legend(fontsize=LOW_FONT_SIZE)

ax = fig.add_subplot(2, 3, 5)
x = np.linspace(0, LIMIT_AXES, 60)
y = f_with_dist(x, alpha=1.0, limit=limit)
ax.plot(x, y, label="$score_{near}$")
ax.axvline(x = limit, color = 'r', label = "Human Personal Space")
ax.set_xlabel('Distance Human-Robot', fontsize=FONT_SIZE)
ax.legend(fontsize=LOW_FONT_SIZE)

ax = fig.add_subplot(2, 3, 6)
x = np.linspace(0, LIMIT_AXES, 60)
y = f_with_dist(x, alpha=10.0, limit=limit)
ax.plot(x, y, label="$score_{near}$")
ax.axvline(x = limit, color = 'r', label = "Human Personal Space")
ax.set_xlabel('Distance Human-Robot', fontsize=FONT_SIZE)
ax.legend(fontsize=LOW_FONT_SIZE)


plt.show()