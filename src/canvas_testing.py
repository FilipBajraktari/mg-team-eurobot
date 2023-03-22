import numpy as np
import matplotlib.pyplot as plt

def setup_canvas(ax):
    ax.set_xlim(-150, 150)
    ax.set_ylim(-100, 100)

    # Major ticks every 20, minor ticks every 5
    x_major_ticks = np.arange(-150, 151, 50)
    x_minor_ticks = np.arange(-150, 151, 5)
    y_major_ticks = np.arange(-100, 101, 50)
    y_minor_ticks = np.arange(-100, 101, 5)

    ax.set_xticks(x_major_ticks)
    ax.set_xticks(x_minor_ticks, minor=True)
    ax.set_yticks(y_major_ticks)
    ax.set_yticks(y_minor_ticks, minor=True)

    # And a corresponding grid
    ax.grid(which='both')

    # Or if you want different settings for the grids:
    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)

    return ax

def custom_canvas(interactive=True):
    fig, ax = plt.subplots()
    if interactive:
        plt.ion()

    ax = setup_canvas(ax)
    return fig, ax


if __name__ == "__main__":
    fig, ax = custom_canvas()
    for x, y in [(10,10), (20,20), (30,30), (40,40), (50,50)]:
        ax.scatter(x,y,s=3,c='blue')
        plt.draw()
        plt.pause(1)

    plt.show()