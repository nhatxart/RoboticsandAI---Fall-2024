#include <iostream>
#include <vector>
#include <unordered_map>
#include <random>
#include <algorithm>
#include <cmath>

class RLRobotNavigation {
private:
    // Q-learning parameters
    double learningRate = 0.1;
    double discountFactor = 0.9;
    double explorationRate = 0.2;

    // Grid representation
    std::vector<std::vector<char>> grid;

    // Q-table to store action values
    std::unordered_map<std::string, std::vector<double>> qTable;

    // Possible movement directions: left, right, up, down
    std::vector<std::pair<int, int>> directions = { {0, -1}, {0, 1}, {-1, 0}, {1, 0} };

    // Random number generator
    std::random_device rd;
    std::mt19937 gen{ rd() };
    std::uniform_real_distribution<> dis{ 0.0, 1.0 };

    // Generate unique state key
    std::string getStateKey(int x, int y) {
        return std::to_string(x) + "," + std::to_string(y);
    }

    // Check if move is valid
    bool isValidMove(int x, int y) {
        return x >= 0 && x < grid.size() &&
            y >= 0 && y < grid[0].size() &&
            grid[x][y] != '#' &&
            grid[x][y] != 'C';
    }

    // Reward function
    double getReward(int x, int y, bool isGoal) {
        if (!isValidMove(x, y)) return -10.0;  // Penalty for invalid moves
        if (isGoal) return 100.0;  // Large reward for reaching goal
        return -1.0;  // Small penalty for each move
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

public:
    RLRobotNavigation(const std::vector<std::vector<char>>& inputGrid)
        : grid(inputGrid) {}

    int navigate(int startX, int startY, int goalX, int goalY, int maxIterations = 1000) {
        int currentX = startX, currentY = startY;
        int steps = 0;

        for (int iter = 0; iter < maxIterations; ++iter) {
            // Check if goal is reached
            if (currentX == goalX && currentY == goalY) break;

            // Choose action
            int actionIndex = chooseAction(currentX, currentY);
            auto [dx, dy] = directions[actionIndex];

            // Compute new position
            int newX = currentX + dx;
            int newY = currentY + dy;

            // Validate move
            if (!isValidMove(newX, newY)) continue;

            // Compute reward
            bool isGoal = (newX == goalX && newY == goalY);
            double reward = getReward(newX, newY, isGoal);

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
            steps++;
        }

        return steps;
    }
};

int main() {
    // Problem 1 Grid
    std::vector<std::vector<char>> grid1 = {
        {'G', 'W', 'W', '#', 'C'},
        {'#', '#', 'W', 'W', 'C'},
        {'#', '#', 'W', 'W', 'C'},
        {'#', '#', 'W', 'W', 'C'}
    };
    RLRobotNavigation nav1(grid1);
    std::cout << "Problem 1 Steps: " << nav1.navigate(0, 0, 2, 2) << std::endl;

    // Problem 2 Grid
    std::vector<std::vector<char>> grid2 = {
        {'G', 'W', '#', 'W', 'C'},
        {'W', 'W', 'W', 'W', 'C'},
        {'#', '#', 'W', 'W', 'C'},
        {'#', '#', 'W', 'W', 'C'}
    };
    RLRobotNavigation nav2(grid2);
    std::cout << "Problem 2 Steps: " << nav2.navigate(0, 0, 2, 3) << std::endl;

    // Problem 3 Grid with Location Sensors
    std::vector<std::vector<char>> grid3 = {
        {'G', 'W', 'S', 'W', 'C'},
        {'W', 'W', 'S', 'W', 'C'},
        {'#', '#', 'W', 'S', 'C'},
        {'#', '#', 'W', 'S', 'C'}
    };
    RLRobotNavigation nav3(grid3);
    std::cout << "Problem 3 Steps: " << nav3.navigate(0, 0, 2, 3) << std::endl;

    return 0;
}