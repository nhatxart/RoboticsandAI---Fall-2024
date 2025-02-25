import matplotlib.pyplot as plt
from skimage.morphology import skeletonize
import numpy as np

class MazeGridNavigator:
    def __init__(self, img_path, grid_size=10):
        """
        Initialize the maze grid navigator
        
        Args:
        img_path (str): Path to the maze image
        grid_size (int): Size of each grid cell in pixels
        """
        # Read the image
        self.rgb_img = plt.imread(img_path)
        
        # Threshold the image
        if len(self.rgb_img.shape) > 2:
            thr_img = self.rgb_img[:,:,0] > np.max(self.rgb_img[:,:,0])/2
        else:
            thr_img = self.rgb_img > np.max(self.rgb_img)/2
        
        # Skeletonize the image
        self.skeleton = skeletonize(thr_img)
        
        # Grid parameters
        self.grid_size = grid_size
        self.height, self.width = self.skeleton.shape
        
        # Create grid representation
        self.grid = self._create_grid()
        
    def _create_grid(self):
        """
        Convert skeleton to a grid representation
        
        Returns:
        numpy.ndarray: Grid representation of the maze
        """
        # Initialize grid
        grid_height = int(np.ceil(self.height / self.grid_size))
        grid_width = int(np.ceil(self.width / self.grid_size))
        grid = np.zeros((grid_height, grid_width), dtype=int)
        
        # Fill grid based on skeleton
        for y in range(grid_height):
            for x in range(grid_width):
                # Extract the grid cell region
                y_start = y * self.grid_size
                y_end = min((y + 1) * self.grid_size, self.height)
                x_start = x * self.grid_size
                x_end = min((x + 1) * self.grid_size, self.width)
                
                # Check if the grid cell contains a path
                cell_region = self.skeleton[y_start:y_end, x_start:x_end]
                grid[y, x] = 1 if np.any(cell_region) else 0
        
        return grid
    
    def find_path(self, start_grid_x, start_grid_y, end_grid_x, end_grid_y):
        """
        Find path between two grid points using A* algorithm
        
        Args:
        start_grid_x (int): Starting x grid coordinate
        start_grid_y (int): Starting y grid coordinate
        end_grid_x (int): Ending x grid coordinate
        end_grid_y (int): Ending y grid coordinate
        
        Returns:
        tuple: Path coordinates, instructions
        """
        # Ensure start and end points are valid
        assert 0 <= start_grid_x < self.grid.shape[1], "Invalid start x coordinate"
        assert 0 <= start_grid_y < self.grid.shape[0], "Invalid start y coordinate"
        assert 0 <= end_grid_x < self.grid.shape[1], "Invalid end x coordinate"
        assert 0 <= end_grid_y < self.grid.shape[0], "Invalid end y coordinate"
        
        # Possible movement directions (8-connectivity)
        directions = [
            (0, 1), (0, -1), (1, 0), (-1, 0),  # 4-way
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # Diagonal
        ]
        
        # Initialize data structures for A*
        open_set = [(start_grid_y, start_grid_x)]
        came_from = {}
        g_score = {(start_grid_y, start_grid_x): 0}
        
        while open_set:
            current = open_set.pop(0)
            
            # Check if reached the goal
            if current == (end_grid_y, end_grid_x):
                break
            
            # Explore neighbors
            for dy, dx in directions:
                neighbor = (current[0] + dy, current[1] + dx)
                
                # Check if neighbor is within grid and is a valid path
                if (0 <= neighbor[0] < self.grid.shape[0] and 
                    0 <= neighbor[1] < self.grid.shape[1] and 
                    self.grid[neighbor[0], neighbor[1]] == 1):
                    
                    # Calculate tentative g_score
                    tentative_g_score = g_score[current] + (1 if dy == 0 or dx == 0 else np.sqrt(2))
                    
                    # Update if this is a better path
                    if (neighbor not in g_score or 
                        tentative_g_score < g_score.get(neighbor, float('inf'))):
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        open_set.append(neighbor)
            
            # Sort open set by proximity to goal
            open_set.sort(key=lambda x: abs(x[0] - end_grid_y) + abs(x[1] - end_grid_x))
        
        # Reconstruct path
        path = []
        current = (end_grid_y, end_grid_x)
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append((start_grid_y, start_grid_x))
        path.reverse()
        
        # Generate instructions
        instructions = self._generate_instructions(path)
        
        return path, instructions
    
    def _generate_instructions(self, path):
        """
        Generate navigation instructions based on grid path
        
        Args:
        path (list): List of grid coordinates
        
        Returns:
        list: Navigation instructions
        """
        instructions = []
        
        for i in range(1, len(path) - 1):
            prev = path[i-1]
            curr = path[i]
            next = path[i+1]
            
            # Calculate movement vectors
            prev_move = (curr[0] - prev[0], curr[1] - prev[1])
            next_move = (next[0] - curr[0], next[1] - curr[1])
            
            # Determine turn direction using cross product
            cross_product = prev_move[0] * next_move[1] - prev_move[1] * next_move[0]
            
            # Determine instruction
            if cross_product > 0:
                turn = "turn right"
            elif cross_product < 0:
                turn = "turn left"
            else:
                turn = "go forward"
            
            instructions.append(f"Grid Point {curr}: {turn}")
        
        return instructions
    
    def visualize(self, path=None):
        """
        Visualize the grid and optionally the path
        
        Args:
        path (list, optional): Path coordinates to overlay
        """
        plt.figure(figsize=(15,15))
        
        # Plot grid
        plt.imshow(self.rgb_img, cmap='gray')
        
        # Plot grid lines
        for y in range(0, self.height, self.grid_size):
            plt.axhline(y, color='red', linewidth=0.5, alpha=0.5)
        for x in range(0, self.width, self.grid_size):
            plt.axvline(x, color='red', linewidth=0.5, alpha=0.5)
        
        # Plot path if provided
        if path:
            path_y = [p[0] * self.grid_size + self.grid_size // 2 for p in path]
            path_x = [p[1] * self.grid_size + self.grid_size // 2 for p in path]
            plt.plot(path_x, path_y, 'r-', linewidth=3)
        
        plt.title(f'Maze Grid (Grid Size: {self.grid_size})')
        plt.show()

# Example usage
if __name__ == "__main__":
    # Path to maze image
    img_name = r'C:\Users\hoang\Downloads\HardRectMaze.png'
    
    # Create navigator with custom grid size
    navigator = MazeGridNavigator(img_name, grid_size=20)
    
    # Find path between grid points
    start_grid_x, start_grid_y = 0, 0  # Starting grid coordinates
    end_grid_x, end_grid_y = -1, -1    # Ending grid coordinates
    
    # Find path
    path, instructions = navigator.find_path(start_grid_x, start_grid_y, end_grid_x, end_grid_y)
    
    # Visualize results
    navigator.visualize(path)
    
    # Print instructions
    print("\nNavigation Instructions:")
    for instruction in instructions:
        print(instruction)