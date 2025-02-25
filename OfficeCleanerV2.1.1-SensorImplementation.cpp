#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>


class RLRobotNavigation {
private:
    // Grid representation
    std::vector<std::vector<char>> grid;

    // Memory tracking
    std::unordered_set<std::string> visitedCells;
    std::unordered_set<std::string> unvisitedCells;

    // Q-learning parameters
    double learningRate = 0.1;
    double discountFactor = 0.9;
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

    // Sensor proximity constraint (for Problem 3)
    bool crossesSensorRegion(int x1, int y1, int x2, int y2) {
        // Find all sensor locations
        std::vector<std::pair<int, int>> sensorLocations;
        for (int x = 0; x < grid.size(); ++x) {
            for (int y = 0; y < grid[x].size(); ++y) {
                if (grid[x][y] == 'S') {
                    sensorLocations.push_back({ x, y });
                }
            }
        }

        // Check if the line between (x1,y1) and (x2,y2) crosses any sensor regions
        for (size_t i = 0; i < sensorLocations.size(); ++i) {
            for (size_t j = i + 1; j < sensorLocations.size(); ++j) {
                int sx1 = sensorLocations[i].first;
                int sy1 = sensorLocations[i].second;
                int sx2 = sensorLocations[j].first;
                int sy2 = sensorLocations[j].second;

                // Check if the move crosses the line between these two sensors
                // if they are closer than 5 grid units
                if (std::abs(sx1 - sx2) + std::abs(sy1 - sy2) <= 5) {
                    // Use line segment intersection algorithm
                    auto cross = [](int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4) {
                        auto orientation = [](int x1, int y1, int x2, int y2, int x3, int y3) {
                            int val = (y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2);
                            if (val == 0) return 0;  // Collinear
                            return (val > 0) ? 1 : 2;
                            };

                        int o1 = orientation(x1, y1, x2, y2, x3, y3);
                        int o2 = orientation(x1, y1, x2, y2, x4, y4);
                        int o3 = orientation(x3, y3, x4, y4, x1, y1);
                        int o4 = orientation(x3, y3, x4, y4, x2, y2);

                        // General case
                        if (o1 != o2 && o3 != o4) return true;

                        return false;
                        };

                    // Check if the move crosses the line between these sensors
                    if (cross(x1, y1, x2, y2, sx1, sy1, sx2, sy2)) {
                        return true;
                    }
                }
            }
        }
        return false;
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
            // Find current position and check sensor crossings
            for (int i = 0; i < grid.size(); ++i) {
                for (int j = 0; j < grid[i].size(); ++j) {
                    if (grid[i][j] == 'S') {
                        // If current move crosses sensor regions, invalidate
                        if (crossesSensorRegion(i, j, x, y)) {
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
        initializeUnvisitedCells();
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

            // Compute reward
            bool isGoal = (newX == exitPoint.x && newY == exitPoint.y);
            double reward = getReward(newX, newY, isGoal, isNewCell, cellVisitCount);

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

    return 0;
}