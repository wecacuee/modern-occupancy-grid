import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np
def drawgrid(ax, fourptsclockwise, divisions):
    lefttop, righttop, rightbottom, leftbottom = fourptsclockwise
    horizontal = np.empty((divisions + 1, 2, 2)) # nlines x |start, end| x dim
    start = 0; end = 1; x =0; y = 1; 
    horizontal[:, start, x] = np.linspace(lefttop[0], leftbottom[0], divisions + 1)
    horizontal[:, start, y] = np.linspace(lefttop[1], leftbottom[1], divisions + 1)
    horizontal[:, end, x] = np.linspace(righttop[0], rightbottom[0], divisions + 1)
    horizontal[:, end, y] = np.linspace(righttop[1], rightbottom[1], divisions + 1)


    vertical = np.empty((divisions + 1, 2, 2))
    vertical[:, start, x] = np.linspace(lefttop[0], righttop[0], divisions + 1)
    vertical[:, start, y] = np.linspace(lefttop[1], righttop[1], divisions + 1)
    vertical[:, end, x] = np.linspace(leftbottom[0], rightbottom[0], divisions + 1)
    vertical[:, end, y] = np.linspace(leftbottom[1], rightbottom[1], divisions + 1)

    horiz_segs = LineCollection(list(horizontal))
    ax.add_collection(horiz_segs)
    vert_segs = LineCollection(list(vertical))
    ax.add_collection(vert_segs)


# main
fourptsclockwise = [[0, 0], [5. - 5./7., 0], [7.5 - 5./7., -2.5 + 2.5/7.],
                    [2.5 - 2.5/7, -2.5 + 2.5/7.]]
divisions = 6
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal')
ax.set_xlim(np.amin(horizontal[:, :, x]) - 1, 
            np.amax(horizontal[:, :, x]) + 1)
ax.set_ylim(np.amin(horizontal[:, :, y]) - 1, 
            np.amax(horizontal[:, :, y]) + 1)
plt.axis('off')
drawgrid(ax, fourptsclockwise, divisions)

show()
