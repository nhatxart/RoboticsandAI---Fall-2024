#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <random>
#include <algorithm>
#include <cmath>
#include <string>

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
    double explorationRate = 0.2;

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
    double getReward(int x, int y, bool isGoal, bool isNewCell) {
        // Invalid move penalty
        if (!isValidMove(x, y)) return -10.0;

        // Reaching goal
        if (isGoal) return 100.0;

        // Reward for visiting a new cell
        if (isNewCell) return 10.0;

        // Small penalty for revisiting or moving
        return -1.0;
    }

public:
    // Constructor
    RLRobotNavigation(const std::vector<std::vector<char>>& inputGrid)
        : grid(inputGrid) {
        // Find entry and exit points
        findSpecialPoints();
        // Initialize unvisited cells tracking
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
    int chooseAction(int x, int y) {
        std::string stateKey = getStateKey(x, y);

        // Initialize Q-values if state not seen before
        if (qTable.find(stateKey) == qTable.end()) {
            qTable[stateKey] = std::vector<double>(4, 0.0);
        }

        // Exploration vs exploitation
        if (dis(gen) < explorationRate) {
            return std::uniform_int_distribution<>{0, 3}(gen);
        }

        // Exploit: choose action with highest Q-value
        return std::max_element(qTable[stateKey].begin(), qTable[stateKey].end())
            - qTable[stateKey].begin();
    }

    // Main navigation method with memory tracking
    std::vector<std::pair<int, int>> navigate(bool useSensorCheck = false, int maxIterations = 1000) {
        int currentX = entryPoint.x, currentY = entryPoint.y;
        std::vector<std::pair<int, int>> path;
        path.push_back({ currentX, currentY });

        visitedCells.clear();
        visitedCells.insert(getStateKey(currentX, currentY));
        unvisitedCells.erase(getStateKey(currentX, currentY));

        int steps = 0;
        bool exitReached = false;

        for (int iter = 0; iter < maxIterations; ++iter) {
            // Check if all cells are visited and exit is reached
            if (unvisitedCells.empty() &&
                currentX == exitPoint.x &&
                currentY == exitPoint.y) {
                exitReached = true;
                break;
            }

            // Choose action
            int actionIndex = chooseAction(currentX, currentY);
            auto [dx, dy] = directions[actionIndex];

            // Compute new position
            int newX = currentX + dx;
            int newY = currentY + dy;

            // Validate move (with optional sensor check)
            if (!isValidMove(newX, newY, useSensorCheck)) continue;

            // Check if cell is new
            std::string newStateKey = getStateKey(newX, newY);
            bool isNewCell = (unvisitedCells.find(newStateKey) != unvisitedCells.end());

            // Compute reward
            bool isGoal = (newX == exitPoint.x && newY == exitPoint.y);
            double reward = getReward(newX, newY, isGoal, isNewCell);

            // Update memory
            if (isNewCell) {
                unvisitedCells.erase(newStateKey);
            }
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

        std::cout << "Navigation " << (exitReached ? "Successful" : "Failed") << std::endl;
        std::cout << "Steps taken: " << steps << std::endl;
        std::cout << "Unvisited cells: " << unvisitedCells.size() << std::endl;

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

    std::cout << "Problem 1 Navigation:" << std::endl;
    RLRobotNavigation nav1(grid1);
    std::vector<std::pair<int, int>> robotPath1 = nav1.navigate();

    std::cout << "\nProblem 2 Navigation:" << std::endl;
    RLRobotNavigation nav2(grid2);
    std::vector<std::pair<int, int>> robotPath2 = nav2.navigate();

    std::cout << "\nProblem 3 Navigation (with Sensor Constraints):" << std::endl;
    RLRobotNavigation nav3(grid3);
    std::vector<std::pair<int, int>> robotPath3 = nav3.navigate(true);  // Enable sensor check

    // Print paths for verification
    auto printPath = [](const std::vector<std::pair<int, int>>& path) {
        std::cout << "Robot Path:" << std::endl;
        for (const auto& [x, y] : path) {
            std::cout << "(" << x << "," << y << ") ";
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