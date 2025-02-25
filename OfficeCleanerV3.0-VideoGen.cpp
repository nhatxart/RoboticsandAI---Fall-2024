#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include "opencv/cv.h"
#include "opencv/cv.hpp"
#include "opencv/cvaux.h"
#include "opencv/cvaux.hpp"
#include "opencv/highgui.h"
#include "opencv/ml.h"
#include "opencv/cvwimage.h"
#include "opencv/cxcore.h"
#include "opencv/cxcore.hpp"
#include "opencv/cxeigen.hpp"
#include "opencv/cxmisc.h"

#include <filesystem>
#include <sstream>
#include <iomanip>

class RLRobotNavigation {
private:
    // Grid representation
    std::vector<std::vector<char>> grid;

    // Memory tracking
    std::unordered_set<std::string> visitedCells;
    std::unordered_set<std::string> unvisitedCells;

    // Q-learning parameters
    double learningRate = 0.1;
    double discountFactor = 0.1;
    double explorationRate = 1;

    // Q-table to store action values
    std::unordered_map<std::string, std::vector<double>> qTable;

    // Possible movement directions: left, right, up, down
    std::vector<std::pair<int, int>> directions = { {0, -1}, {0, 1}, {-1, 0}, {1, 0} };

    // Random number generator
    std::random_device rd;
    std::mt19937 gen{ rd() };
    std::uniform_real_distribution<> dis{ 0.0, 1.0 };

    // Coordinates
    struct Coordinates {
        int x, y;
        Coordinates(int _x = 0, int _y = 0) : x(_x), y(_y) {}
    };
    Coordinates entryPoint, exitPoint;

    // Sensor proximity constraint (for Problem 3)
    bool checkSensorProximity(int x1, int y1, int x2, int y2) {
        return std::abs(x1 - x2) + std::abs(y1 - y2) <= 5;
    }

    // Generate unique state key
    std::string getStateKey(int x, int y) {
        return std::to_string(x) + "," + std::to_string(y);
    }

    // Initialize unvisited cells
    void initializeUnvisitedCells() {
        unvisitedCells.clear();
        for (int x = 0; x < grid.size(); ++x) {
            for (int y = 0; y < grid[x].size(); ++y) {
                // Mark all white cells as unvisited
                if (grid[x][y] == 'W' || grid[x][y] == 'G') {
                    unvisitedCells.insert(getStateKey(x, y));
                }
            }
        }
    }

    // Check if move is valid
    bool isValidMove(int x, int y, bool checkSensor = false) {
        // Basic grid boundary and obstacle check
        bool basicCheck = x >= 0 && x < grid.size() &&
            y >= 0 && y < grid[0].size() &&
            grid[x][y] != '#' &&
            grid[x][y] != 'C';

        // Additional sensor proximity check for Problem 3
        if (checkSensor && basicCheck) {
            // Find current position
            int currentX = -1, currentY = -1;
            for (int i = 0; i < grid.size(); ++i) {
                for (int j = 0; j < grid[i].size(); ++j) {
                    if (grid[i][j] == 'S') {
                        if (checkSensorProximity(x, y, i, j)) {
                            return false;
                        }
                    }
                }
            }
        }

        return basicCheck;
    }

    // Reward function with memory consideration
    double getReward(int x, int y, bool isGoal, bool isNewCell,
        std::unordered_map<std::string, int>& cellVisitCount) {
        std::string cellKey = getStateKey(x, y);

        // Invalid move penalty
        if (!isValidMove(x, y)) return -10.0;

        // Reaching goal
        if (isGoal) return 100.0;

        // Increment visit count
        cellVisitCount[cellKey]++;

        // Exponential decay of revisit rewards
        double revisitPenalty = std::pow(0.1, cellVisitCount[cellKey] - 1); //More revisits = less reward

        // Base rewards with decay
        if (isNewCell) {
            return 10.0;  // High reward for first visit
        }
        else {
            return 1.0 * revisitPenalty;  // Diminishing reward for repeated visits
        }
    }

public:
    // Constructor
    RLRobotNavigation(const std::vector<std::vector<char>>& inputGrid)
        : grid(inputGrid) {
        // Find entry and exit points
        findSpecialPoints();
        //// Initialize unvisited cells tracking
        //initializeUnvisitedCells();
    }

    // Find entry and exit points in the grid
    void findSpecialPoints() {
        for (int x = 0; x < grid.size(); ++x) {
            for (int y = 0; y < grid[x].size(); ++y) {
                if (grid[x][y] == 'G') {
                    entryPoint = { x, y };
                }
                if (grid[x][y] == 'R') {  // 'R' for Red exit point
                    exitPoint = { x, y };
                }
            }
        }
    }

    // Choose action using epsilon-greedy strategy
    std::size_t chooseAction(int x, int y) {
        std::string stateKey = getStateKey(x, y);

        // Initialize Q-values if state not seen before
        if (qTable.find(stateKey) == qTable.end()) {
            qTable[stateKey] = std::vector<double>(4, 0.0);
        }

        // Exploration vs exploitation
        if (dis(gen) < explorationRate) {
            return std::uniform_int_distribution<std::size_t>{0, 3}(gen);
        }

        // Exploit: choose action with highest Q-value
        return static_cast<std::size_t>(std::max_element(qTable[stateKey].begin(), qTable[stateKey].end())
            - qTable[stateKey].begin());
    }

    // Main navigation method with memory tracking
   // Modify the navigate method to track comprehensive exploration
    std::vector<std::pair<int, int>> navigate(bool useSensorCheck = false, int maxIterations = 1000) {
        int currentX = entryPoint.x, currentY = entryPoint.y;
        std::vector<std::pair<int, int>> path;
        path.push_back({ currentX, currentY });

        // Create a set of all visitable cells
        std::unordered_set<std::string> visitableCells;
        for (int x = 0; x < grid.size(); ++x) {
            for (int y = 0; y < grid[x].size(); ++y) {
                if (grid[x][y] == 'W' || grid[x][y] == 'G') {
                    visitableCells.insert(getStateKey(x, y));
                }
            }
        }

        // Track number of visits to each cell
        std::unordered_map<std::string, int> cellVisitCount;

        visitedCells.clear();
        visitedCells.insert(getStateKey(currentX, currentY));
        int steps = 0;
        bool exitReached = false;
        bool allCellsVisited = false;

        // Enhanced reward function
        auto getEnhancedReward = [&](int x, int y, bool isGoal, bool isNewCell) {
            std::string cellKey = getStateKey(x, y);

            // Invalid move penalty
            if (!isValidMove(x, y)) return -10.0;

            // Increment visit count
            cellVisitCount[cellKey]++;

            // Exponential decay of revisit rewards
            double revisitPenalty = std::pow(0.1, cellVisitCount[cellKey] - 1);

            // Special handling for exit point
            if (isGoal) {
                // Can only reach exit after exploring all cells
                if (allCellsVisited) {
                    return 100.0;  // High reward for completing mission
                }
                else {
                    return -50.0;  // Penalty for trying to exit prematurely
                }
            }

            // Base rewards with decay
            if (isNewCell) {
                return 10.0;  // High reward for first visit
            }
            else {
                return 1.0 * revisitPenalty;  // Diminishing reward for repeated visits
            }
            };

        for (int iter = 0; iter < maxIterations; ++iter) {
            // Check if all cells are visited
            allCellsVisited = (visitedCells == visitableCells);

            // Check if exit is reached after exploring all cells
            if (allCellsVisited &&
                currentX == exitPoint.x && currentY == exitPoint.y) {
                exitReached = true;
                break;
            }

            // Choose action
            int actionIndex = chooseAction(currentX, currentY);
            int dx, dy;
            std::tie(dx, dy) = directions[actionIndex];

            // Compute new position
            int newX = currentX + dx;
            int newY = currentY + dy;

            // Validate move (with optional sensor check)
            if (!isValidMove(newX, newY, useSensorCheck)) continue;

            // Check if cell is new
            std::string newStateKey = getStateKey(newX, newY);
            bool isNewCell = (visitedCells.find(newStateKey) == visitedCells.end());

            // Compute reward with new comprehensive exploration check
            bool isGoal = (newX == exitPoint.x && newY == exitPoint.y);
            double reward = getEnhancedReward(newX, newY, isGoal, isNewCell);

            // Update memory
            visitedCells.insert(newStateKey);

            // Update Q-value
            std::string currentState = getStateKey(currentX, currentY);
            std::string newState = getStateKey(newX, newY);

            // Initialize new state Q-values if not exists
            if (qTable.find(newState) == qTable.end()) {
                qTable[newState] = std::vector<double>(4, 0.0);
            }

            // Q-learning update
            double maxNextQ = *std::max_element(qTable[newState].begin(), qTable[newState].end());
            qTable[currentState][actionIndex] += learningRate *
                (reward + discountFactor * maxNextQ - qTable[currentState][actionIndex]);

            // Move to new position
            currentX = newX;
            currentY = newY;
            path.push_back({ currentX, currentY });
            steps++;
        }

        // Adjust print statements
        std::cout << "Navigation " << (exitReached ? "Successful" : "Failed")
            << " (Visited " << visitedCells.size() << "/" << visitableCells.size() << " cells)" << std::endl;
        std::cout << "Steps taken: " << steps << std::endl;

        return path;
    }
};

class NavigationVideoGenerator {
private:
    // Grid visualization parameters
    const int CELL_SIZE = 40;
    const int GRID_PADDING = 20;
    const cv::Scalar BACKGROUND_COLOR = cv::Scalar(240, 240, 240);
    const cv::Scalar WALL_COLOR = cv::Scalar(50, 50, 50);
    const cv::Scalar OBSTACLE_COLOR = cv::Scalar(100, 100, 100);
    const cv::Scalar PATH_COLOR = cv::Scalar(0, 0, 255);
    const cv::Scalar CURRENT_POSITION_COLOR = cv::Scalar(255, 0, 0);
    const cv::Scalar ENTRY_COLOR = cv::Scalar(0, 255, 0);
    const cv::Scalar EXIT_COLOR = cv::Scalar(255, 0, 0);

    // Video creation parameters
    const int FPS = 30;
    const int MOVES_PER_SECOND = 2;
    const cv::Size VIDEO_SIZE = cv::Size(1920, 1080);

    std::vector<std::vector<char>> grid;
    std::vector<std::pair<int, int>> path;

    // Create temporary directory for frames
    std::filesystem::path createTempDirectory() {
        std::filesystem::path tempDir = std::filesystem::temp_directory_path() /
            ("robot_navigation_" + std::to_string(std::time(nullptr)));
        std::filesystem::create_directory(tempDir);
        return tempDir;
    }

    // Draw grid on the frame
    void drawGrid(cv::Mat& frame) {
        // Scale and center the grid
        int gridWidth = grid[0].size() * CELL_SIZE;
        int gridHeight = grid.size() * CELL_SIZE;
        int startX = (VIDEO_SIZE.width - gridWidth) / 2;
        int startY = (VIDEO_SIZE.height - gridHeight) / 2;

        // Draw background
        frame.setTo(BACKGROUND_COLOR);

        // Draw cells
        for (int y = 0; y < grid.size(); ++y) {
            for (int x = 0; x < grid[y].size(); ++x) {
                cv::Rect cell(
                    startX + x * CELL_SIZE,
                    startY + y * CELL_SIZE,
                    CELL_SIZE, CELL_SIZE
                );

                switch (grid[y][x]) {
                case '#':  // Wall
                    cv::rectangle(frame, cell, WALL_COLOR, cv::FILLED());
                    break;
                case 'C':  // Obstacle
                    cv::rectangle(frame, cell, OBSTACLE_COLOR, cv::FILLED());
                    break;
                case 'G':  // Entry
                    cv::rectangle(frame, cell, ENTRY_COLOR, cv::FILLED());
                    break;
                case 'R':  // Exit
                    cv::rectangle(frame, cell, EXIT_COLOR, cv::FILLED());
                    break;
                }

                // Draw cell border
                cv::rectangle(frame, cell, cv::Scalar(200, 200, 200), 1);
            }
        }
    }

    void generateNavigationVideo(const std::filesystem::path& tempDir) {
        int framesBetweenMoves = FPS / MOVES_PER_SECOND;

        // Create video writer
        cv::VideoWriter videoWriter(
            (tempDir.parent_path() / "robot_navigation.mp4").string(),
            CV_FOURCC('m', 'p', '4', 'v'),
            FPS,
            VIDEO_SIZE
        );

        // Interpolate path with intermediate frames
        for (size_t i = 0; i < path.size() - 1; ++i) {
            int startX = path[i].second;
            int startY = path[i].first;
            int endX = path[i + 1].second;
            int endY = path[i + 1].first;

            for (int frame = 0; frame < framesBetweenMoves; ++frame) {
                cv::Mat videoFrame(VIDEO_SIZE, CV_8UC3);
                drawGrid(videoFrame);

                // Interpolate current position
                double t = static_cast<double>(frame) / framesBetweenMoves;
                int currentX = startX + t * (endX - startX);
                int currentY = startY + t * (endY - startY);

                // Scale and center grid coordinates
                int gridStartX = (VIDEO_SIZE.width - grid[0].size() * CELL_SIZE) / 2;
                int gridStartY = (VIDEO_SIZE.height - grid.size() * CELL_SIZE) / 2;

                // Draw path up to current point
                for (size_t j = 0; j <= i; ++j) {
                    cv::Point pathPoint(
                        gridStartX + path[j].second * CELL_SIZE + CELL_SIZE / 2,
                        gridStartY + path[j].first * CELL_SIZE + CELL_SIZE / 2
                    );
                    cv::circle(videoFrame, pathPoint, CELL_SIZE / 4, PATH_COLOR, -1);
                }

                // Draw current position
                cv::Point currentPoint(
                    gridStartX + currentX * CELL_SIZE + CELL_SIZE / 2,
                    gridStartY + currentY * CELL_SIZE + CELL_SIZE / 2
                );
                cv::circle(videoFrame, currentPoint, CELL_SIZE / 3, CURRENT_POSITION_COLOR, -1);

                videoWriter.write(videoFrame);
            }
        }

        videoWriter.release();
    }


public:
    NavigationVideoGenerator(
        const std::vector<std::vector<char>>& inputGrid,
        const std::vector<std::pair<int, int>>& robotPath
    ) : grid(inputGrid), path(robotPath) {}

    void createNavigationVideo() {
        // Create temporary directory for frames
        std::filesystem::path tempDir = createTempDirectory();

        // Generate video
        generateNavigationVideo(tempDir);

        std::cout << "Navigation video created: robot_navigation.mp4" << std::endl;
    }
};

int main() {
    // Problem 1 Grid
    std::vector<std::vector<char>> grid1 = {
        {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', 'G', 'R', '#'},
        {'#', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'C', 'C', 'C', 'W', 'W', 'C', 'C', 'C', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'C', 'C', 'C', 'W', 'W', 'C', 'C', 'C', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'C', 'C', 'C', 'W', 'W', 'C', 'C', 'C', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'W', 'W', 'W', 'W', '#'},
        {'#', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'W', 'W', 'W', 'W', '#'},
        {'#', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'W', 'W', 'W', 'W', '#'},
        {'#', 'C', 'C', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'C', 'C', 'W', 'W', 'W', 'C', 'C', 'C', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'C', 'C', 'W', 'W', 'W', 'C', 'C', 'C', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'C', 'C', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'}  // 'R' as exit point
    };

    // Problem 2 Grid (different layout)
    std::vector<std::vector<char>> grid2 = {
        {'#', 'G', 'R', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'},
        {'#', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'C', 'C', 'C', 'C', 'C', 'C', 'C', '#'},
        {'#', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'C', 'C', 'C', 'C', 'C', 'C', 'C', '#'},
        {'#', 'W', 'W', 'W', 'C', 'C', 'W', 'W', 'C', 'C', 'C', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'W', 'W', 'C', 'C', 'W', 'W', 'C', 'C', 'C', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'W', 'W', 'C', 'C', 'W', 'W', 'C', 'C', 'C', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'C', 'C', 'C', 'W', 'C', 'C', 'W', '#'},
        {'#', 'W', 'W', 'W', 'C', 'C', 'W', 'W', 'C', 'C', 'C', 'W', 'C', 'C', 'W', '#'},
        {'#', 'W', 'W', 'W', 'C', 'C', 'W', 'W', 'C', 'C', 'C', 'W', 'C', 'C', 'W', '#'},
        {'#', 'W', 'W', 'W', 'C', 'C', 'W', 'W', 'C', 'C', 'C', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'C', 'C', 'C', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'C', 'C', 'C', 'C', 'W', 'W', 'W', 'W', 'W', 'W', 'C', 'C', 'C', 'C', '#'},
        {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'}   // 'R' as exit point
    };

    // Problem 3 Grid with Location Sensors
    std::vector<std::vector<char>> grid3 = {
        {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', 'G', 'R', '#'},
        {'#', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'W', 'W', 'S', 'C', 'C', 'S', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'W', 'W', 'C', 'C', 'C', 'C', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'W', 'W', 'S', 'C', 'C', 'S', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'S', 'C', 'C', 'C', 'C', 'S', 'C', 'C', 'C', 'S', 'W', 'W', 'W', 'W', '#'},
        {'#', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'C', 'W', 'W', 'W', 'W', '#'},
        {'#', 'C', 'S', 'C', 'C', 'C', 'S', 'C', 'C', 'C', 'S', 'W', 'W', 'W', 'W', '#'},
        {'#', 'C', 'C', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'C', 'C', 'W', 'W', 'S', 'C', 'S', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'C', 'C', 'W', 'W', 'S', 'C', 'S', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', 'C', 'S', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', 'W', '#'},
        {'#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#', '#'}  // 'R' as exit point, 'S' as sensors
    };

    // Navigation for each problem
    RLRobotNavigation nav1(grid1);
    std::vector<std::pair<int, int>> robotPath1 = nav1.navigate();

    RLRobotNavigation nav2(grid2);
    std::vector<std::pair<int, int>> robotPath2 = nav2.navigate();

    RLRobotNavigation nav3(grid3);
    std::vector<std::pair<int, int>> robotPath3 = nav3.navigate(true);  // Enable sensor check

    // Print paths for verification
    auto printPath = [](const std::vector<std::pair<int, int>>& path) {
        std::cout << "Robot Path:" << std::endl;
        for (const auto& point : path) {
            std::cout << "(" << point.first << "," << point.second << ") ";
        }
        std::cout << std::endl;
        };

    std::cout << "\nProblem 1 Path:" << std::endl;
    printPath(robotPath1);

    std::cout << "\nProblem 2 Path:" << std::endl;
    printPath(robotPath2);

    std::cout << "\nProblem 3 Path:" << std::endl;
    printPath(robotPath3);

    // Add video generation for each problem
    NavigationVideoGenerator videoGen1(grid1, robotPath1);
    videoGen1.createNavigationVideo();

    NavigationVideoGenerator videoGen2(grid2, robotPath2);
    videoGen2.createNavigationVideo();

    NavigationVideoGenerator videoGen3(grid3, robotPath3);
    videoGen3.createNavigationVideo();

    return 0;
}