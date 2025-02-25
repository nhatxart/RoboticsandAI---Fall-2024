import cv2
import numpy as np
import heapq
import serial
import matplotlib.pyplot as plt
from skimage.morphology import reconstruction, convex_hull_image

class AdvancedMazeSolver:
    def __init__(self, serial_port='/dev/ttyUSB0', baud_rate=9600):
        """
        Initialize MazeSolver with enhanced preprocessing and serial communication
        
        :param serial_port: Serial communication port
        :param baud_rate: Serial communication baud rate
        """
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
        except serial.SerialException as e:
            print(f"Serial connection error: {e}")
            self.ser = None

        # Cardinal directions: Right, Down, Left, Up
        self.directions = [
            (0, 1),   # Right
            (1, 0),   # Down
            (0, -1),  # Left
            (-1, 0)   # Up
        ]

    def simple_white_balancing(self, image):
        """
        Advanced white balancing technique
        
        :param image: Input image
        :return: White-balanced image
        """
        h, w = image.shape[:2]
        patch = image[int(h/2-20):int(h/2+20), int(w/2-20):int(w/2+20)]
        x, y = cv2.minMaxLoc(np.sum(patch.astype(int), axis=2))[3]
        white_b, white_g, white_r = patch[y, x, ...].astype(float)
        lum = (white_r + white_g + white_b) / 3
        
        image[..., 0] = np.clip(image[..., 0] * lum / white_b, 0, 255)
        image[..., 1] = np.clip(image[..., 1] * lum / white_g, 0, 255)
        image[..., 2] = np.clip(image[..., 2] * lum / white_r, 0, 255)
        
        return image.astype(np.uint8)

    def preprocess_maze(self, image_path):
        """
        Advanced maze preprocessing with reconstruction and convex hull
        
        :param image_path: Path to maze image
        :return: Preprocessed binary maze
        """
        # Read image
        img = cv2.imread(image_path)
        
        # Gaussian blur for noise reduction
        img = cv2.GaussianBlur(img, (11, 11), None)
        
        # White balancing
        maze = self.simple_white_balancing(img.copy())
        
        # Saturation masking
        sat = cv2.cvtColor(maze, cv2.COLOR_BGR2HSV)[..., 1]
        mask = (sat < 16).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_RECT, (31, 31)))
        mask = cv2.copyMakeBorder(mask, 1, 1, 1, 1, cv2.BORDER_CONSTANT, 0)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                cv2.getStructuringElement(cv2.MORPH_RECT, (201, 201)))
        
        # Find largest contour
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        cnt = max(cnts, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(cnt)
        
        # Crop to low saturation area
        cut = cv2.cvtColor(maze[y+1:y+1+h, x+1:x+1+w], cv2.COLOR_BGR2GRAY)
        
        # Reconstruction
        h_c, w_c = cut.shape
        seed = np.zeros_like(cut)
        size = 40
        hh, hw = h_c // 2, w_c // 2
        seed[hh-size:hh+size, hw-size:hw+size] = cut[hh-size:hh+size, hw-size:hw+size]
        
        rec = reconstruction(seed, cut)
        rec = cv2.erode(rec, np.ones((2, 2)), iterations=1)
        
        seed = np.ones_like(rec) * 255
        size = 240
        seed[hh-size:hh+size, hw-size:hw+size] = rec[hh-size:hh+size, hw-size:hw+size]
        rec = reconstruction(seed, rec, method='erosion').astype(np.uint8)
        
        # Threshold
        rec = cv2.threshold(rec, np.quantile(rec, 0.25), 255, cv2.THRESH_BINARY_INV)[1]
        
        # Create hull
        hull = np.zeros_like(img[:,:,0])
        hull[y+1:y+1+h, x+1:x+1+w] = convex_hull_image(rec) * 255
        
        return hull

    def get_path_direction_commands(self, path):
        """
        Generate detailed turn-by-turn navigation commands
        
        :param path: List of path coordinates
        :return: List of navigation commands
        """
        commands = []
        current_direction = 0  # Start facing right
        
        for i in range(1, len(path)):
            # Calculate movement vector
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            
            # Determine new direction
            new_direction = self.directions.index((dx, dy))
            
            # Calculate turn
            turn = (new_direction - current_direction) % 4
            
            if turn == 1:
                commands.append("turn right")
            elif turn == 3:
                commands.append("turn left")
            elif turn == 2:
                commands.append("turn around")
            
            commands.append("go forward")
            current_direction = new_direction
        
        return commands

    def send_navigation_commands(self, commands):
        """
        Send navigation commands via serial
        
        :param commands: List of navigation commands
        """
        if not self.ser:
            print("Serial connection unavailable")
            return
        
        for command in commands:
            try:
                self.ser.write(f"{command}\n".encode())
                print(f"Sent command: {command}")
            except Exception as e:
                print(f"Error sending command {command}: {e}")

    def solve_maze(self, image_path):
        """
        Comprehensive maze solving process
        
        :param image_path: Path to maze image
        """
        # Preprocess maze
        maze = self.preprocess_maze(image_path)
        
        # Find start and end points (you'll need to implement color detection)
        start, goal = self.find_start_and_end(image_path)
        
        # Solve maze using A* (previously implemented method)
        path = self.a_star_search(maze, start, goal)
        
        if not path:
            print("No path found!")
            return
        
        # Generate navigation commands
        commands = self.get_path_direction_commands(path)
        
        # Send commands via serial
        self.send_navigation_commands(commands)
