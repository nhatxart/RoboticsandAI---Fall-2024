#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Eigen library for matrix operations
#include <Eigen/Dense>

class KalmanFilter {
private:
    // State vector: [x, y, vx, vy]
    Eigen::VectorXd state;

    // Error covariance matrix
    Eigen::MatrixXd P;

    // State transition matrix
    Eigen::MatrixXd F;

    // Measurement matrix
    Eigen::MatrixXd H;

    // Measurement noise covariance
    Eigen::MatrixXd R;

    // Process noise covariance
    Eigen::MatrixXd Q;

public:
    KalmanFilter() {
        // Initialize state vector to zero
        state = Eigen::VectorXd::Zero(4);

        // Initialize error covariance matrix
        P = Eigen::MatrixXd::Identity(4, 4) * 1000.0;

        // State transition matrix (assuming constant velocity model)
        F = Eigen::MatrixXd::Identity(4, 4);
        F(0, 2) = 1.0;  // x = x + vx
        F(1, 3) = 1.0;  // y = y + vy

        // Measurement matrix (we can directly measure position)
        H = Eigen::MatrixXd::Zero(2, 4);
        H(0, 0) = 1.0;  // x measurement
        H(1, 1) = 1.0;  // y measurement

        // Measurement noise covariance (position uncertainty)
        R = Eigen::MatrixXd::Identity(2, 2) * 0.1;

        // Process noise covariance (model uncertainty)
        Q = Eigen::MatrixXd::Identity(4, 4) * 0.01;
    }

    // Predict step of Kalman filter
    void predict() {
        // Predict state
        state = F * state;

        // Predict error covariance
        P = F * P * F.transpose() + Q;
    }

    // Update step of Kalman filter
    void update(double measuredX, double measuredY) {
        // Measurement vector
        Eigen::VectorXd z(2);
        z << measuredX, measuredY;

        // Compute Kalman gain
        Eigen::MatrixXd S = H * P * H.transpose() + R;
        Eigen::MatrixXd K = P * H.transpose() * S.inverse();

        // Update state estimate
        Eigen::VectorXd y = z - H * state;
        state = state + K * y;

        // Update error covariance
        P = (Eigen::MatrixXd::Identity(4, 4) - K * H) * P;
    }

    // Getters for state
    double getEstimatedX() const { return state(0); }
    double getEstimatedY() const { return state(1); }
    double getEstimatedVX() const { return state(2); }
    double getEstimatedVY() const { return state(3); }
};

class RLRobotNavigation {
private:
    // Grid representation
    std::vector<std::vector<char>> grid;

    // Kalman Filter for state estimation
    KalmanFilter kalmanFilter;

    // Memory tracking
    std::unordered_set<std::string> visitedCells;
    std::unordered_set<std::string> unvisitedCells;

    // Q-learning parameters
    double learningRate = 0.1;
    double discountFactor = 0.9;

    // Thompson sampling parameters
    std::unordered_map<std::string, std::vector<int>> actionCounts;
    std::unordered_map<std::string, std::vector<double>> rewardSums;

    // Q-table to store action values
    std::unordered_map<std::string, std::vector<double>> qTable;

    // Possible movement directions: left, right, up, down
    std::vector<std::pair<int, int>> directions = { {0, -1}, {0, 1}, {-1, 0}, {1, 0} };

    // Random number generator
    std::random_device rd;
    std::mt19937 gen{ rd() };
    std::uniform_real_distribution<> dis{ 0.0, 1.0 };
    std::gamma_distribution<> gamma{ 1.0, 1.0 };

    // Coordinates
    struct Coordinates {
        int x, y;
        Coordinates(int _x = 0, int _y = 0) : x(_x), y(_y) {}
    };
    Coordinates entryPoint, exitPoint;

    // Sensor proximity and validation methods
    bool checkSensorProximity(int x1, int y1, int x2, int y2) {
        return std::abs(x1 - x2) + std::abs(y1 - y2) <= 5;
    }

    std::string getStateKey(int x, int y) {
        return std::to_string(x) + "," + std::to_string(y);
    }

    bool isValidMove(int x, int y, bool checkSensor = false) {
        // Basic grid boundary and obstacle check
        bool basicCheck = x >= 0 && x < grid.size() &&
            y >= 0 && y < grid[0].size() &&
            grid[x][y] != '#' &&
            grid[x][y] != 'C';

        // Additional sensor proximity check for Problem 3
        if (checkSensor && basicCheck) {
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

public:
    // Constructor
    RLRobotNavigation(const std::vector<std::vector<char>>& inputGrid)
        : grid(inputGrid) {
        findSpecialPoints();
    }

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

    // Thompson Sampling Action Selection with Kalman Filter integration
    std::size_t chooseAction(int x, int y) {
        std::string stateKey = getStateKey(x, y);

        // Predict next state using Kalman Filter
        kalmanFilter.predict();

        // Initialize action information if state not seen before
        if (actionCounts.find(stateKey) == actionCounts.end()) {
            actionCounts[stateKey] = std::vector<int>(4, 0);
            rewardSums[stateKey] = std::vector<double>(4, 0.0);
            qTable[stateKey] = std::vector<double>(4, 0.0);
        }

        // Thompson Sampling: Sample from Beta distribution for each action
        std::vector<double> sampledValues(4);
        for (int i = 0; i < 4; ++i) {
            // Use Gamma distribution as a simpler approximation of Beta
            double alpha = std::max(1.0, rewardSums[stateKey][i] + 1.0);
            double beta = std::max(1.0, actionCounts[stateKey][i] - rewardSums[stateKey][i] + 1.0);

            // Sample from Gamma distributions and normalize
            double gammaAlpha = gamma(gen);
            double gammaBeta = gamma(gen);

            sampledValues[i] = (gammaAlpha / (gammaAlpha + gammaBeta));
        }

        // Choose action with highest sampled value
        return static_cast<std::size_t>(std::max_element(sampledValues.begin(), sampledValues.end()) - sampledValues.begin());
    }

    // Navigate method with Kalman Filter integration
    std::vector<std::pair<int, int>> navigate(bool useSensorCheck = false, int maxIterations = 2000) {
        int currentX = entryPoint.x, currentY = entryPoint.y;
        std::vector<std::pair<int, int>> path;
        path.push_back({ currentX, currentY });

        // Initialize Kalman Filter with entry point
        kalmanFilter = KalmanFilter();
        kalmanFilter.update(currentX, currentY);

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

            // Choose action using Thompson Sampling
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

            // Update Kalman Filter with new position
            kalmanFilter.update(newX, newY);

            // Update memory
            visitedCells.insert(newStateKey);

            // Update action statistics for Thompson Sampling
            std::string currentState = getStateKey(currentX, currentY);
            actionCounts[currentState][actionIndex]++;
            rewardSums[currentState][actionIndex] += (reward > 0 ? 1.0 : 0.0);

            // Update Q-value (maintaining Q-learning update)
            if (qTable.find(currentState) == qTable.end()) {
                qTable[currentState] = std::vector<double>(4, 0.0);
            }
            if (qTable.find(newStateKey) == qTable.end()) {
                qTable[newStateKey] = std::vector<double>(4, 0.0);
            }

            // Q-learning update
            double maxNextQ = *std::max_element(qTable[newStateKey].begin(), qTable[newStateKey].end());
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

        // Print Kalman Filter estimated final state
        std::cout << "Final Estimated Position: ("
            << kalmanFilter.getEstimatedX() << ", "
            << kalmanFilter.getEstimatedY() << ")" << std::endl;
        std::cout << "Final Estimated Velocity: ("
            << kalmanFilter.getEstimatedVX() << ", "
            << kalmanFilter.getEstimatedVY() << ")" << std::endl;

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