import cv2
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import serial
import time

class MazeNavigationSystem:
    def __init__(self, maze_image_path, serial_port='COM3', baud_rate=9600):
        """
        Initialize maze navigation system
        
        :param maze_image_path: Path to maze image
        :param serial_port: Serial port for communication
        :param baud_rate: Serial communication baud rate
        """
        # Image processing
        self.maze_image = cv2.imread(maze_image_path)
        self.gray = cv2.cvtColor(self.maze_image, cv2.COLOR_BGR2GRAY)
        _, self.binary = cv2.threshold(self.gray, 127, 255, cv2.THRESH_BINARY_INV)
        
        # Skeletonization
        self.skeleton = self.skeletonize_maze()
        
        # Navigation setup
        self.graph = self.create_navigation_graph()
        self.path = None
        
        # Serial communication
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Allow time for serial connection to initialize
        except serial.SerialException:
            print(f"Warning: Could not open serial port {serial_port}")
            self.ser = None

    def skeletonize_maze(self):
        """
        Convert wide path to single-pixel skeleton
        
        :return: Skeletonized single-pixel path
        """
        skeleton = np.zeros(self.binary.shape, np.uint8)
        kernel = np.ones((3,3), np.uint8)
        binary = self.binary.copy()
        
        while cv2.countNonZero(binary) > 0:
            eroded = cv2.erode(binary, kernel)
            temp = cv2.dilate(eroded, kernel)
            temp = cv2.subtract(binary, temp)
            skeleton = cv2.bitwise_or(skeleton, temp)
            binary = eroded.copy()
        
        return skeleton

    def create_navigation_graph(self):
        """
        Create a graph representation of the maze skeleton
        
        :return: NetworkX graph of maze paths
        """
        graph = nx.Graph()
        
        # Find skeleton coordinates
        skeleton_coords = np.column_stack(np.where(self.skeleton > 0))
        
        # Add nodes
        for coord in skeleton_coords:
            graph.add_node(tuple(coord))
        
        # Connect nearby points
        for i, point1 in enumerate(skeleton_coords):
            for point2 in skeleton_coords[i+1:]:
                if np.linalg.norm(point1 - point2) < 5:  # Connect points close to each other
                    graph.add_edge(tuple(point1), tuple(point2))
        
        return graph

    def a_star_path_finding(self, start, goal):
        """
        Find optimal path using A* algorithm
        
        :param start: Starting coordinates
        :param goal: Goal coordinates
        :return: Optimal path
        """
        try:
            path = nx.astar_path(self.graph, start, goal, 
                                  heuristic=lambda a, b: np.linalg.norm(np.array(a) - np.array(b)))
            return path
        except nx.NetworkXNoPath:
            print("No path found!")
            return None

    def mdp_navigation(self, path):
        """
        Convert path to movement instructions using MDP
        
        :param path: Coordinates path
        :return: List of navigation instructions
        """
        instructions = []
        current_direction = None

        for i in range(len(path) - 1):
            dx = path[i+1][1] - path[i][1]
            dy = path[i+1][0] - path[i][0]
            
            # Determine movement direction
            if dx > 0:
                new_direction = 'E'
            elif dx < 0:
                new_direction = 'W'
            elif dy > 0:
                new_direction = 'S'
            elif dy < 0:
                new_direction = 'N'
            else:
                continue

            # Determine turn instructions
            if current_direction is None:
                instructions.append('START')
            elif current_direction != new_direction:
                turn = self.get_turn_instruction(current_direction, new_direction)
                instructions.append(turn)
            
            instructions.append('FORWARD')
            current_direction = new_direction

        instructions.append('STOP')
        return instructions

    def get_turn_instruction(self, current, target):
        """
        Determine turn direction
        
        :param current: Current direction
        :param target: Target direction
        :return: Turn instruction
        """
        turns = {
            ('N', 'E'): 'TURN_RIGHT',
            ('N', 'W'): 'TURN_LEFT',
            ('S', 'E'): 'TURN_LEFT',
            ('S', 'W'): 'TURN_RIGHT',
            ('E', 'N'): 'TURN_LEFT',
            ('E', 'S'): 'TURN_RIGHT',
            ('W', 'N'): 'TURN_RIGHT',
            ('W', 'S'): 'TURN_LEFT'
        }
        return turns.get((current, target), 'FORWARD')

    def send_navigation_commands(self, instructions):
        """
        Send navigation instructions via serial
        
        :param instructions: List of navigation instructions
        """
        if not self.ser:
            print("Serial connection not available")
            return

        for instruction in instructions:
            self.ser.write(f"{instruction}\n".encode('utf-8'))
            time.sleep(0.5)  # Delay between commands

    def visualize_navigation(self, path):
        """
        Visualize navigation path on maze
        
        :param path: Path coordinates
        """
        plt.figure(figsize=(10, 5))
        
        plt.subplot(121)
        plt.title('Maze Skeleton')
        plt.imshow(self.skeleton, cmap='binary')
        
        plt.subplot(122)
        overlay = cv2.cvtColor(self.skeleton.copy(), cv2.COLOR_GRAY2RGB)
        
        # Plot path
        for point in path:
            overlay[point[0], point[1]] = [255, 0, 0]  # Red path
        
        plt.title('Navigation Path')
        plt.imshow(overlay)
        plt.show()

    def navigate_maze(self, start, goal):
        """
        Complete maze navigation process
        
        :param start: Starting coordinates
        :param goal: Goal coordinates
        """
        # Find path
        self.path = self.a_star_path_finding(start, goal)
        
        if not self.path:
            print("Navigation failed")
            return
        
        # Generate navigation instructions
        instructions = self.mdp_navigation(self.path)
        
        # Visualize navigation
        self.visualize_navigation(self.path)
        
        # Send commands via serial
        self.send_navigation_commands(instructions)

def main():
    # Example usage
    maze_nav = MazeNavigationSystem(r'C:\Users\hoang\Downloads\black-rectangular-labyrinth-game-for-kids-puzzle-for-children-maze-conundrum-flat-illustration-isolated-on-white-background-free-vector-modified.jpg')
    
    # Define start and goal (example coordinates from skeleton)
    start = (10, 10)  # Replace with actual coordinates
    goal = (200, 200)  # Replace with actual coordinates
    
    maze_nav.navigate_maze(start, goal)

if __name__ == '__main__':
    main()