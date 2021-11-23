#!/usr/bin/env python3
import math

import matplotlib.pyplot as plt
import numpy as np

xmin = -10
xmax = 10
ymin = -3
ymax = 10

# Grid of x, y points
nx, ny = 656, 656
xs = np.linspace(xmin, xmax, nx)
ys = np.linspace(ymin, ymax, ny)

dx = np.zeros((ny, nx))
dy = np.zeros((ny, nx))

# step_steep = 2
# well_width = 0.02
# step_height = 1

scaling_factor = 2
step_height = 1


# c = (
#     step_steep ** (2 / step_steep)
#     * step_height ** (1 / step_steep)
#     * (step_steep * step_height + step_steep ** 2 * step_height) ** (-1 / step_steep)
#     / well_width
# )

# Fills xs and ys with values from dx and dy function
for xi, x in enumerate(xs):
    for yi, y in enumerate(ys):
        dx[yi, xi] = -math.tanh(x * scaling_factor)
        dy[yi, xi] = (step_height * (math.tanh(x * scaling_factor) ** 2) - y) * scaling_factor
        # dx[yi, xi] = -x / abs(x) * np.exp(-abs(c * x ** -step_steep))
        # dy[yi, xi] = np.exp(-abs(c * x ** -step_steep)) - y / step_height

fig = plt.figure()
ax = fig.add_subplot(111)

# Plot the streamlines with an appropriate colormap and arrow style
color = np.hypot(dx, dy)
ax.streamplot(
    xs, ys, dx, dy, color=color, linewidth=1, cmap=plt.get_cmap("plasma"), density=2, arrowstyle="->", arrowsize=1.7
)

ax.axhline(y=step_height)
ax.axhline(y=0)

ax.set_xlabel("$x$")
ax.set_ylabel("$y$")
ax.set_xlim(xmin, xmax)
ax.set_ylim(ymin, ymax)
ax.set_aspect("equal")
plt.show()