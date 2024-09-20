import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.colors as colors

cmap = mpl.colormaps['coolwarm']

def truncate_colormap(cmap, minval=0.0, maxval=1.0, n=100):
    new_cmap = colors.LinearSegmentedColormap.from_list(
        'trunc({n},{a:.2f},{b:.2f})'.format(n=cmap.name, a=minval, b=maxval),
        cmap(np.linspace(minval, maxval, n)))
    return new_cmap

def draw_rectangle_gradient(ax, x1, y1, width, height, min_val=0.0, max_val=1.0, n=100, direction="horizontal"):
    new_cmap = truncate_colormap(cmap, min_val, max_val)
    gradient_colors = new_cmap(np.linspace(0, 1, n))

    if direction == "horizontal":
        for i, color in enumerate(gradient_colors):
            ax.add_patch(plt.Rectangle((x1 + width/n * i, y1), width/n, height, color=color, linewidth=0, zorder=0))
    else:
        for i, color in enumerate(gradient_colors):
            ax.add_patch(plt.Rectangle((x1, y1 + height/n * i), width, height/n, color=color, linewidth=0, zorder=0))
    return ax

# SAMPLE
fig, ax = plt.subplots(figsize=(4, 2))
ax.set_xlim(0, 100)
ax.set_ylim(0, 40)

draw_rectangle_gradient(ax, 0, 0, 10, 40, min_val=0.5, max_val=1.0)
draw_rectangle_gradient(ax, 10, 0, 10, 40, min_val=0.0, max_val=0.5)
draw_rectangle_gradient(ax, 30, 0, 10, 40, n=1000, direction="vertical")

plt.show()