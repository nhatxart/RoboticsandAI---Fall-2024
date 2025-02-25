import cv2
import numpy as np
import matplotlib.pyplot as plt

def skeletonize_maze(binary_image):
    """
    Convert wide path to single-pixel skeleton

    :param binary_image: Binary image with black path
    :return: Skeletonized single-pixel path
    """

    # Ensure binary image
    if len(binary_image.shape) > 2:
        binary_image = cv2.cvtColor(binary_image, cv2.COLOR_BGR2GRAY)

    # Threshold to ensure binary
    _, binary = cv2.threshold(binary_image, 127, 255, cv2.THRESH_BINARY_INV)

    # Skeletonization using morphological operations
    kernel = np.ones((3,3), np.uint8)
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

def visualize_skeletonization(original_image, skeleton):
    """
    Visualize original path and its skeleton
    
    :param original_image: Original binary maze image
    :param skeleton: Skeletonized path
    """
    plt.figure(figsize=(12,4))
    
    plt.subplot(131)
    plt.title('Original Binary Image')
    plt.imshow(original_image, cmap='binary')
    
    plt.subplot(132)
    plt.title('Skeletonized Path')
    plt.imshow(skeleton, cmap='binary')
    
    plt.subplot(133)
    plt.title('Overlay')
    
    # Convert to 3-channel color image
    if len(original_image.shape) == 2:
        overlay = cv2.cvtColor(original_image, cv2.COLOR_GRAY2RGB)
    else:
        overlay = original_image.copy()
    
    # Mark skeleton in red
    overlay[skeleton > 0] = [255, 0, 0]
    
    plt.imshow(overlay)
    
    plt.tight_layout()
    plt.show()
# Example usage

def main():
    # Load maze image
    maze_image = cv2.imread(r'C:\Users\hoang\Downloads\black-rectangular-labyrinth-game-for-kids-puzzle-for-children-maze-conundrum-flat-illustration-isolated-on-white-background-free-vector-modified.jpg')

    # Convert to grayscale
    gray = cv2.cvtColor(maze_image, cv2.COLOR_BGR2GRAY)

    # Threshold to binary
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

    # Skeletonize
    skeleton = skeletonize_maze(binary)

    # Visualize
    visualize_skeletonization(binary, skeleton)

if __name__ == '__main__':
    main()