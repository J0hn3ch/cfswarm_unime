import itertools
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


if __name__ == "__main__":
    fig = plt.figure()

    x = np.arange(-9, 10)
    y = np.arange(-9, 10).reshape(-1, 1)

    ims = []
    for t in np.arange(15):
        artist=plt.plot(x+t,y+t**2)
        artist.append(plt.scatter(x+t,y+t**2))
        ims.append(artist)

    im_ani = animation.ArtistAnimation(fig, ims, interval=1000, repeat_delay=3000,
                                   blit=True)
    plt.show()