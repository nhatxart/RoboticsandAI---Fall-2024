import cv2
import numpy as np
import matplotlib.pyplot as plt
import heapq

class MazeSolver:
    def __init__(self, skeleton, start_marker, end_marker):
        """
        Initialize the maze solver with a skeletonized maze
        
        :param skeleton: Skeletonized maze image
        :param start_marker: Coordinates of start marker
        :param end_marker: Coordinates of end marker
        """
        self.skeleton = skeleton
        self.start_marker = start_marker
        self.end_marker = end_marker
        self.height, self.width = skeleton.shape
        self.directions = [
            (0, 1),   # Right
            (1, 0),   # Down
            (0, -1),  # Left
            (-1, 0)   # Up
        ]
    
    def is_valid_move(self, x, y):
        """
        Check if the move is within maze bounds and on the path
        
        :param x: x-coordinate
        :param y: y-coordinate
        :return: Boolean indicating valid move
        """
        return (0 <= x < self.width and 
                0 <= y < self.height and 
                self.skeleton[y, x] > 0)
    
    def find_start_end(self):
        """
        Use predefined start and end markers
        
        :return: Tuple of (start_point, end_point)
        """
        return self.start_marker, self.end_marker

    def heuristic(self, a, b):
        """
        Manhattan distance heuristic
        
        :param a: Start point
        :param b: Goal point
        :return: Estimated distance
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def a_star_search(self):
        """
        Implement A* pathfinding algorithm
        
        :return: Optimal path from start to end
        """
        start, goal = self.find_start_end()
        
        # Priority queue for A* search
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        # Dictionaries to track path
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            # Goal reached
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return list(reversed(path))
            
            for dx, dy in self.directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Skip invalid moves
                if not self.is_valid_move(neighbor[0], neighbor[1]):
                    continue
                
                # Tentative g_score
                tentative_g_score = g_score[current] + 1
                
                # If neighbor not in g_score or we found a better path
                if (neighbor not in g_score or 
                    tentative_g_score < g_score.get(neighbor, float('inf'))):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return None  # No path found
    
    def generate_navigation_instructions(self, path):
        """
        Generate turn-by-turn navigation instructions
        
        :param path: List of (x, y) coordinates
        :return: List of navigation instructions
        """
        if not path or len(path) < 2:
            return []
        
        # Initial direction (right)
        current_direction = 0
        instructions = []
        
        for i in range(len(path) - 1):
            # Calculate move direction
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            
            # Find the index of this move in directions
            move_direction = self.directions.index((dx, dy))
            
            # Calculate turn needed
            turn = (move_direction - current_direction) % 4
            
            # Generate turn instructions
            if turn == 1:
                instructions.append("Turn Right")
            elif turn == 3:
                instructions.append("Turn Left")
            
            # Add move forward instruction
            instructions.append("Move Forward")
            
            # Update current direction
            current_direction = move_direction
        
        return instructions
    
    def simulate_serial_output(self, instructions):
        """
        Simulate serial output for navigation instructions
        Note: Actual serial communication would require pyserial library
        
        :param instructions: List of navigation instructions
        """
        # Commented out serial output implementation
        # import serial
        # ser = serial.Serial('COM3', 9600)  # Replace with appropriate port
        
        print("Navigation Instructions:")
        for instruction in instructions:
            print(instruction)
            # Uncomment for actual serial output
            # ser.write(f"{instruction}\n".encode())
        
        # ser.close()

def mark_start_end(maze_image):
    """
    Interactive method to mark start and end points
    
    :param maze_image: Input maze image
    :return: Processed image with marked start and end
    """
    def onclick(event):
        if event.button == 1 and len(points) < 2:  # Left click
            x, y = int(event.xdata), int(event.ydata)
            points.append((x, y))
            
            if len(points) == 1:
                plt.plot(x, y, 'go', markersize=10)  # Green for start
            else:
                plt.plot(x, y, 'ro', markersize=10)  # Red for end
            
            plt.draw()
        
        if len(points) == 2:
            plt.close()
    
    # Display image for marking
    plt.figure(figsize=(10, 10))
    plt.imshow(cv2.cvtColor(maze_image, cv2.COLOR_BGR2RGB))
    plt.title('Click to mark START (green) and END (red) points')
    
    points = []
    plt.connect('button_press_event', onclick)
    plt.show()
    
    return points

def skeletonize_maze(binary_image, start_marker=None, end_marker=None):
    """
    Convert wide path to single-pixel skeleton
    
    :param binary_image: Binary image with black path
    :param start_marker: Optional start point coordinates
    :param end_marker: Optional end point coordinates
    :return: Skeletonized single-pixel path
    """
    # Ensure binary image
    if len(binary_image.shape) > 2:
        binary_image = cv2.cvtColor(binary_image, cv2.COLOR_BGR2GRAY)
    
    # Create a copy to mark start and end
    marked_binary = binary_image.copy()
    
    # Mark start and end points if provided
    if start_marker:
        cv2.circle(marked_binary, start_marker, 5, 255, -1)
    if end_marker:
        cv2.circle(marked_binary, end_marker, 5, 255, -1)
    
    # Threshold to ensure binary
    _, binary = cv2.threshold(marked_binary, 127, 255, cv2.THRESH_BINARY_INV)
    
    # Optional: Add morphological operations to clean the image
    kernel = np.ones((3,3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    
    # Skeletonization using morphological operations
    skeleton = np.zeros(binary.shape, np.uint8)
    
    while cv2.countNonZero(binary) > 0:
        # Erode
        eroded = cv2.erode(binary, kernel)
        
        # Open
        temp = cv2.dilate(eroded, kernel)
        
        # Subtract to get the difference
        temp = cv2.subtract(binary, temp)
        
        # Add to skeleton
        skeleton = cv2.bitwise_or(skeleton, temp)
        
        # Update binary
        binary = eroded.copy()
    
    return skeleton

def visualize_maze_solution(original_image, skeleton, path, start_marker=None, end_marker=None):
    """
    Visualize maze solution with path and markers
    
    :param original_image: Original binary maze image
    :param skeleton: Skeletonized path
    :param path: Solved path coordinates
    :param start_marker: Start point coordinates
    :param end_marker: End point coordinates
    """
    plt.figure(figsize=(15,5))
    
    plt.subplot(131)
    plt.title('Original Binary Image')
    plt.imshow(original_image, cmap='binary')
    
    plt.subplot(132)
    plt.title('Skeletonized Path')
    plt.imshow(skeleton, cmap='binary')
    
    plt.subplot(133)
    plt.title('Solution Path')
    
    # Convert to 3-channel color image
    if len(original_image.shape) == 2:
        overlay = cv2.cvtColor(original_image, cv2.COLOR_GRAY2RGB)
    else:
        overlay = original_image.copy()
    
    # Mark skeleton in red
    overlay[skeleton > 0] = [255, 0, 0]
    
    # Mark path in green
    if path:
        for x, y in path:
            overlay[y, x] = [0, 255, 0]
    
    # Mark start and end points
    if start_marker:
        cv2.circle(overlay, start_marker, 5, (0, 255, 0), -1)  # Green start
    if end_marker:
        cv2.circle(overlay, end_marker, 5, (0, 0, 255), -1)  # Red end
    
    plt.imshow(overlay)
    
    plt.tight_layout()
    plt.show()

def main():
    # Load maze image
    maze_image = cv2.imread(r'C:\Users\hoang\Downloads\BlackPathMaze.png')
    
    # Interactively mark start and end points
    start_marker, end_marker = mark_start_end(maze_image)
    
    # Convert to grayscale
    gray = cv2.cvtColor(maze_image, cv2.COLOR_BGR2GRAY)
    
    # Threshold to binary
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
    
    # Skeletonize with marked start and end
    skeleton = skeletonize_maze(binary, start_marker, end_marker)
    
    # Create maze solver
    solver = MazeSolver(skeleton, start_marker, end_marker)
    
    try:
        # Find path
        path = solver.a_star_search()
        
        # Generate navigation instructions
        instructions = solver.generate_navigation_instructions(path)
        
        # Simulate serial output
        solver.simulate_serial_output(instructions)
        
        # Visualize solution
        visualize_maze_solution(binary, skeleton, path, start_marker, end_marker)
    
    except Exception as e:
        print(f"Error solving maze: {e}")
        print("Please check the maze image processing.")

if __name__ == '__main__':
    main()
