import matplotlib.pylab as plt  # I use version 3.1.2
import numpy as np  # I use version 1.18.1

def solve_skeleton_maze(skeleton_img, start_point, end_point, box_radius=30):
    """
    Solve a maze given its skeleton image and start/end points.
    
    Parameters:
    - skeleton_img: Binary image of the maze skeleton
    - start_point: (x0, y0) coordinates of the start point
    - end_point: (x1, y1) coordinates of the end point
    - box_radius: Search radius around start/end points
    
    Returns:
    - path_x, path_y: Coordinates of the optimal path
    - dst: Distance map
    """
    x0, y0 = start_point
    x1, y1 = end_point
    
    # Create a copy of the skeleton to modify during pathfinding
    _mapt = np.copy(~skeleton_img)
    
    # Safety check for points near image edges
    if y1 < box_radius: y1 = box_radius
    if x1 < box_radius: x1 = box_radius
    
    # Find possible end points in the vicinity
    cpys, cpxs = np.where(_mapt[y1-box_radius:y1+box_radius, 
                                x1-box_radius:x1+box_radius] == 0)
    
    # Calibrate points to main scale
    cpys += y1 - box_radius
    cpxs += x1 - box_radius
    
    # Find closest point to end point
    idx = np.argmin(np.sqrt((cpys-y1)**2 + (cpxs-x1)**2))
    y, x = cpys[idx], cpxs[idx]
    
    # Initialize lists for breadth-first search
    pts_x = [x]
    pts_y = [y]
    pts_c = [0]
    
    # Mesh of displacements
    xmesh, ymesh = np.meshgrid(np.arange(-1,2), np.arange(-1,2))
    ymesh = ymesh.reshape(-1)
    xmesh = xmesh.reshape(-1)
    
    # Distance map
    dst = np.zeros(skeleton_img.shape)
    
    # Breadth-first search
    while pts_x:
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
        
        # Check if we've reached near the start point
        if np.sqrt((x-x0)**2 + (y-y0)**2) < box_radius:
            edx, edy = x, y
            break
    
    # Trace the best path
    path_x, path_y = [], []
    y, x = edy, edx
    
    while True:
        # Get neighbourhood
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
        
        # Check if we've reached the end point
        if np.sqrt((x-x1)**2 + (y-y1)**2) < box_radius:
            print('Optimum route found.')
            break
        
        path_y.append(y)
        path_x.append(x)
    
    return path_x, path_y, dst

# Example usage
def main():
    # Load the skeleton image
    img_name = r'C:\Users\hoang\Downloads\BlackPathMaze.png'
    skeleton_img = plt.imread(img_name)
    
    # Convert to binary if it's not already
    if len(skeleton_img.shape) > 2:
        skeleton_img = skeleton_img[:,:,0] > np.max(skeleton_img[:,:,0])/2
    
    # Define start and end points
    start_point = (198, 113)
    end_point = (964, 530)
    
    # Solve the maze
    path_x, path_y, dst = solve_skeleton_maze(skeleton_img, start_point, end_point)
    
    # Plotting
    plt.figure(figsize=(14,14))
    plt.subplot(2,2,1)
    plt.title('Original Skeleton')
    plt.imshow(skeleton_img, cmap='gray')
    plt.plot(start_point[0], start_point[1], 'gx', markersize=14)
    plt.plot(end_point[0], end_point[1], 'rx', markersize=14)
    
    plt.subplot(2,2,2)
    plt.title('Distance Map')
    plt.imshow(dst)
    
    plt.subplot(2,2,3)
    plt.title('Maze Solution')
    plt.imshow(skeleton_img, cmap='gray')
    plt.plot(path_x, path_y, 'r-', linewidth=5)
    plt.plot(start_point[0], start_point[1], 'gx', markersize=14)
    plt.plot(end_point[0], end_point[1], 'rx', markersize=14)
    
    plt.tight_layout()
    plt.show()

