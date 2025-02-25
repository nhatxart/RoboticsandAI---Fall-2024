import matplotlib.pyplot as plt
from skimage.morphology import skeletonize
import numpy as np

def find_maze_path(img_path, x0, y0, x1, y1, boxr=30):
    # Read the image
    rgb_img = plt.imread(img_path)
    
    # Threshold the image
    if len(rgb_img.shape) > 2:
        thr_img = rgb_img[:,:,0] > np.max(rgb_img[:,:,0])/2
    else:
        thr_img = rgb_img > np.max(rgb_img)/2
    
    # Skeletonize the image
    skeleton = skeletonize(thr_img)
    
    # Create map of routes
    mapT = ~skeleton
    
    # Safety check for points near edges
    if y1 < boxr: y1 = boxr
    if x1 < boxr: x1 = boxr
    
    # Find possible end points
    _mapt = np.copy(mapT)
    cpys, cpxs = np.where(_mapt[y1-boxr:y1+boxr, x1-boxr:x1+boxr]==0)
    
    # Calibrate points to main scale
    cpys += y1-boxr
    cpxs += x1-boxr
    
    # Find closest point of possible path end points
    idx = np.argmin(np.sqrt((cpys-y1)**2 + (cpxs-x1)**2))
    y, x = cpys[idx], cpxs[idx]
    
    # Mesh of displacements
    xmesh, ymesh = np.meshgrid(np.arange(-1,2), np.arange(-1,2))
    ymesh = ymesh.reshape(-1)
    xmesh = xmesh.reshape(-1)
    
    # Initialize path finding variables
    pts_x = [x]
    pts_y = [y]
    pts_c = [0]
    dst = np.zeros(thr_img.shape)
    
    # Breadth-first algorithm exploring a tree
    while True:
        # Update distance
        idc = np.argmin(pts_c)
        ct = pts_c.pop(idc)
        x = pts_x.pop(idc)
        y = pts_y.pop(idc)
        
        # Search 3x3 neighbourhood for possible paths
        ys, xs = np.where(_mapt[y-1:y+2, x-1:x+2] == 0)
        
        # Invalidate these points from future searches
        _mapt[ys+y-1, xs+x-1] = ct
        _mapt[y,x] = 9999999
        
        # Set the distance in the distance image
        dst[ys+y-1, xs+x-1] = ct+1
        
        # Extend our lists
        pts_x.extend(xs+x-1)
        pts_y.extend(ys+y-1)
        pts_c.extend([ct+1]*xs.shape[0])
        
        # Check termination conditions
        if not pts_x:
            break
        if np.sqrt((x-x0)**2 + (y-y0)**2) < boxr:
            edx, edy = x, y
            break
    
    # Trace best path
    path_x, path_y = [], []
    y, x = edy, edx
    while True:
        nbh = dst[y-1:y+2, x-1:x+2]
        nbh[1,1] = 9999999
        nbh[nbh==0] = 9999999
        
        # If we reach a dead end
        if np.min(nbh) == 9999999:
            break
        
        # Find direction
        idx = np.argmin(nbh)
        y += ymesh[idx]
        x += xmesh[idx]
        
        if np.sqrt((x-x1)**2 + (y-y1)**2) < boxr:
            print('Optimum route found.')
            break
        
        path_y.append(y)
        path_x.append(x)
    
    # Visualize results
    plt.figure(figsize=(14,14))
    plt.imshow(rgb_img)
    plt.plot(path_x, path_y, 'r-', linewidth=5)
    plt.plot(x0, y0, 'gx', markersize=14)
    plt.plot(x1, y1, 'rx', markersize=14)
    plt.show()
    
    return path_x, path_y

# Example usage
if __name__ == "__main__":
    img_name = r'C:\Users\hoang\Downloads\HardRectMaze.png'
    x0, y0 = 6, 7  # start point
    x1, y1 = 744, 1005  # end point
    
    path_x, path_y = find_maze_path(img_name, x0, y0, x1, y1)