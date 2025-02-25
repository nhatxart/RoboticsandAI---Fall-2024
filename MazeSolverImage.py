import matplotlib.pylab as plt
from skimage.morphology import skeletonize
import numpy as np 

maze_img = r'C:\Users\hoang\Downloads\geekdad_maze.png'
read_maze = plt.imread (maze_img)

plt.figure (figsize=(14,14))
plt.imshow (read_maze)

