import numpy as np
import matplotlib.pyplot as plt

def custom_canvas():
    fig, ax = plt.subplots()
    plt.ion()

    ax.set_xlim(-150, 150)
    ax.set_ylim(-100, 100)

    # Major ticks every 20, minor ticks every 5
    major_ticks = np.arange(-150, 151, 50)
    minor_ticks = np.arange(-150, 151, 5)

    ax.set_xticks(major_ticks)
    ax.set_xticks(minor_ticks, minor=True)
    ax.set_yticks(major_ticks)
    ax.set_yticks(minor_ticks, minor=True)

    # And a corresponding grid
    ax.grid(which='both')

    # Or if you want different settings for the grids:
    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)

    return fig, ax


if __name__ == "__main__":
    fig, ax = custom_canvas()
    for x, y in [(10,10), (20,20), (30,30), (40,40), (50,50)]:
        ax.scatter(x,y,s=3,c='blue')
        plt.draw()
        plt.pause(1)

    plt.show()