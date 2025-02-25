#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <limits>

class RLRobotNavigation {
private:
    // Grid representation
    std::vector<std::vector<char>> grid;

    // Memory tracking
    std::unordered_set<std::string> visitedCells;

    // RL parameters with more careful tuning
    double learningRate = 0.3;  // Increased learning rate
    double discountFactor = 0.99;  // Slightly increased discount factor
    double explorationRate = 1.0;
    double explorationDecay = 0.59;  // Slower decay for more exploration
    double minExplorationRate = 0.01;

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
    std::string getStateKey(int x, int y) const {
        return std::to_string(x) + "," + std::to_string(y);
    }

    // Check if move is valid
    bool isValidMove(int x, int y) const {
        return x >= 0 && x < static_cast<int>(grid.size()) &&
            y >= 0 && y < static_cast<int>(grid[0].size()) &&
            grid[x][y] != '#' &&
            grid[x][y] != 'C';
    }

    // Comprehensive reward function
    double computeReward(int x, int y, const std::unordered_set<std::string>& visitableCells,
        const std::unordered_set<std::string>& visitedCells,
        bool isGoal) const {
        std::string cellKey = getStateKey(x, y);

        // Invalid move penalty
        if (!isValidMove(x, y)) return -100.0;

        // Reaching exit after exploring all cells
        if (isGoal && visitedCells.size() == visitableCells.size()) return 1000.0;

        // Premature exit attempt
        if (isGoal) return -50.0;

        // Reward for exploring new cells
        if (visitedCells.find(cellKey) == visitedCells.end()) return 10.0;

        // Small negative reward for revisiting cells to encourage exploration
        return -1.0;
    }

    // Action selection with improved exploration
    std::size_t selectAction(int x, int y) {
        std::string stateKey = getStateKey(x, y);

        // Initialize Q-values for new states
        if (qTable.find(stateKey) == qTable.end()) {
            qTable[stateKey] = std::vector<double>(4, 0.0);
        }

        // Epsilon-greedy action selection
        if (dis(gen) < explorationRate) {
            // Uniform random action
            return std::uniform_int_distribution<std::size_t>{0, 3}(gen);
        }

        // Exploit: choose action with highest Q-value
        auto& stateActions = qTable[stateKey];
        return std::max_element(stateActions.begin(), stateActions.end()) - stateActions.begin();
    }

public:
    // Constructor
    RLRobotNavigation(const std::vector<std::vector<char>>& inputGrid)
        : grid(inputGrid) {
        findSpecialPoints();
    }

    // Find special points in the grid
    void findSpecialPoints() {
        for (int x = 0; x < static_cast<int>(grid.size()); ++x) {
            for (int y = 0; y < static_cast<int>(grid[x].size()); ++y) {
                if (grid[x][y] == 'G') {
                    entryPoint = { x, y };
                }
                if (grid[x][y] == 'R') {
                    exitPoint = { x, y };
                }
            }
        }
    }

    // Main navigation method with improved learning
    std::vector<std::pair<int, int>> navigate(int maxEpisodes = 5000, int maxStepsPerEpisode = 1000) {
        // Collect all visitable cells
        std::unordered_set<std::string> visitableCells;
        for (int x = 0; x < static_cast<int>(grid.size()); ++x) {
            for (int y = 0; y < static_cast<int>(grid[x].size()); ++y) {
                if (grid[x][y] == 'W' || grid[x][y] == 'G') {
                    visitableCells.insert(getStateKey(x, y));
                }
            }
        }

        std::vector<std::pair<int, int>> bestPath;
        double bestPathCoverage = 0.0;

        // Multiple episodes for learning
        for (int episode = 0; episode < maxEpisodes; ++episode) {
            // Reset for each episode
            int currentX = entryPoint.x, currentY = entryPoint.y;
            visitedCells.clear();
            visitedCells.insert(getStateKey(currentX, currentY));

            std::vector<std::pair<int, int>> currentPath;
            currentPath.push_back({ currentX, currentY });

            // Decay exploration rate
            explorationRate = std::max(minExplorationRate,
                explorationRate * explorationDecay);

            for (int step = 0; step < maxStepsPerEpisode; ++step) {
                // Select action
                int actionIndex = selectAction(currentX, currentY);

                // Compute move
                int dx = directions[actionIndex].first;
                int dy = directions[actionIndex].second;
                int newX = currentX + dx;
                int newY = currentY + dy;

                // Skip invalid moves
                if (!isValidMove(newX, newY)) continue;

                // Compute reward
                bool isGoal = (newX == exitPoint.x && newY == exitPoint.y);
                std::string newStateKey = getStateKey(newX, newY);

                // Update Q-value using Q-learning update rule
                double reward = computeReward(newX, newY, visitableCells, visitedCells, isGoal);

                // Update Q-table
                std::string currentStateKey = getStateKey(currentX, currentY);

                // Ensure new state exists in Q-table
                if (qTable.find(newStateKey) == qTable.end()) {
                    qTable[newStateKey] = std::vector<double>(4, 0.0);
                }

                // Q-learning update
                double maxNextQ = *std::max_element(qTable[newStateKey].begin(), qTable[newStateKey].end());
                qTable[currentStateKey][actionIndex] += learningRate *
                    (reward + discountFactor * maxNextQ - qTable[currentStateKey][actionIndex]);

                // Update state and path
                currentX = newX;
                currentY = newY;
                currentPath.push_back({ currentX, currentY });
                visitedCells.insert(newStateKey);

                // Check if all cells explored and exit reached
                if (isGoal && visitedCells.size() == visitableCells.size()) {
                    double coverage = static_cast<double>(visitedCells.size()) / visitableCells.size();
                    if (coverage > bestPathCoverage) {
                        bestPath = currentPath;
                        bestPathCoverage = coverage;
                    }
                    break;
                }
            }
        }

        // Print results
        std::cout << "Best Path Coverage: " << bestPathCoverage * 100.0 << "%" << std::endl;
        std::cout << "Path Length: " << bestPath.size() << std::endl;

        return bestPath;
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