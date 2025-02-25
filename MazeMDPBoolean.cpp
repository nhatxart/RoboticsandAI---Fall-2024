#include <iostream>
#include <vector>
#include <random>
#include <queue>
#include <unordered_map>
#include <ctime>

class MazeMDP {
private:
    static const int ROWS = 16;
    static const int COLS = 16;
    std::vector<std::vector<int>> maze;
    double error_prob;

    const std::vector<std::pair<int, int>> actions = { {0, 1}, {0, -1}, {1, 0}, {-1, 0} };

    std::random_device rd;
    std::mt19937 gen;

    struct PathResult {
        std::vector<std::pair<int, int>> path;
        int steps;
        bool found;

        PathResult() : steps(-1), found(false) {}
        PathResult(std::vector<std::pair<int, int>> p, int s)
            : path(p), steps(s), found(true) {}
    };

public:
    MazeMDP(double e = 0.0) : error_prob(e), gen(rd()) {
        initializeMaze();
    }

    void initializeMaze() {
        // Initialize maze with all walls first
        maze = std::vector<std::vector<int>>(ROWS, std::vector<int>(COLS, 1));

        // Define the paths (0 for path, 1 for wall)
        const std::vector<std::vector<int>> pathLayout = {
            { 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 },
            { 0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 1 },
            { 1 , 1 , 1 , 0 , 1 , 0 , 1 , 0 , 1 , 0 , 1 , 1 , 1 , 1 , 0 , 1 },
            { 1 , 0 , 0 , 0 , 1 , 0 , 1 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 1 },
            { 1 , 0 , 1 , 1 , 1 , 0 , 1 , 0 , 1 , 1 , 1 , 0 , 1 , 1 , 1 , 1 },
            { 1 , 1 , 1 , 0 , 0 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 0 , 1 },
            { 1 , 0 , 0 , 0 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 1 , 0 , 1 },
            { 1 , 0 , 1 , 1 , 1 , 0 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0 , 1 , 1 },
            { 1 , 0 , 0 , 1 , 0 , 0 , 1 , 0 , 1 , 0 , 1 , 0 , 1 , 0 , 0 , 1 },
            { 1 , 1 , 0 , 0 , 0 , 1 , 1 , 0 , 0 , 0 , 1 , 1 , 1 , 1 , 0 , 1 },
            { 1 , 0 , 0 , 1 , 1 , 0 , 1 , 0 , 1 , 0 , 1 , 0 , 0 , 0 , 0 , 1 },
            { 1 , 1 , 0 , 0 , 0 , 0 , 1 , 0 , 1 , 0 , 1 , 0 , 1 , 1 , 0 , 1 },
            { 1 , 0 , 0 , 1 , 1 , 0 , 1 , 1 , 1 , 0 , 1 , 0 , 1 , 0 , 0 , 1 },
            { 1 , 1 , 0 , 0 , 1 , 0 , 1 , 0 , 0 , 0 , 1 , 0 , 0 , 1 , 1 , 1 },
            { 1 , 0 , 0 , 1 , 0 , 0 , 1 , 0 , 1 , 0 , 0 , 1 , 0 , 0 , 0 , 0 }
        };

        // Copy the layout to the maze
        for (int i = 0; i < ROWS && i < pathLayout.size(); i++) {
            for (int j = 0; j < COLS && j < pathLayout[i].size(); j++) {
                maze[i][j] = pathLayout[i][j];
            }
        }
    }

    bool isValid(int x, int y) const {
        return x >= 0 && x < ROWS && y >= 0 && y < COLS;
    }

    bool isPath(int x, int y) const {
        if (!isValid(x, y)) return false;
        return maze[x][y] == 0;
    }

    std::vector<std::pair<int, int>> getValidMoves(int x, int y) {
        std::vector<std::pair<int, int>> valid_moves;

        for (const auto& action : actions) {
            int new_x = x + action.first;
            int new_y = y + action.second;

            if (isValid(new_x, new_y)) {
                bool is_actually_path = isPath(new_x, new_y);
                bool sensed_as_path = (std::uniform_real_distribution<>(0, 1)(gen) >= error_prob)
                    ? is_actually_path
                    : !is_actually_path;

                if (sensed_as_path) {
                    valid_moves.push_back({ new_x, new_y });
                }
            }
        }
        return valid_moves;
    }

    PathResult findPath(std::pair<int, int> start, std::pair<int, int> end) {
        if (!isValid(start.first, start.second) || !isValid(end.first, end.second)) {
            return PathResult();
        }

        std::queue<std::pair<int, int>> q;
        std::unordered_map<int, std::pair<int, int>> came_from;
        std::unordered_map<int, int> steps;

        q.push(start);
        steps[start.first * COLS + start.second] = 0;

        while (!q.empty()) {
            auto current = q.front();
            q.pop();

            if (current == end) {
                std::vector<std::pair<int, int>> path;
                auto pos = current;
                while (pos != start) {
                    path.push_back(pos);
                    pos = came_from[pos.first * COLS + pos.second];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return PathResult(path, steps[current.first * COLS + current.second]);
            }

            for (const auto& next : getValidMoves(current.first, current.second)) {
                int next_key = next.first * COLS + next.second;
                if (steps.find(next_key) == steps.end()) {
                    steps[next_key] = steps[current.first * COLS + current.second] + 1;
                    came_from[next_key] = current;
                    q.push(next);
                }
            }
        }
        return PathResult();
    }

    double simulateWalks(int num_trials = 1000) {
        std::pair<int, int> blue_exit = { 1, 0 };
        std::pair<int, int> red_exit = { 14, 15 };

        int total_steps = 0;
        int successful_trials = 0;

        for (int i = 0; i < num_trials; i++) {
            PathResult forward_result = findPath(blue_exit, red_exit);
            if (forward_result.found) {
                PathResult reverse_result = findPath(red_exit, blue_exit);
                if (reverse_result.found) {
                    total_steps += forward_result.steps + reverse_result.steps;
                    successful_trials++;
                }
            }
        }

        return successful_trials > 0 ?
            static_cast<double>(total_steps) / successful_trials : -1;
    }

    void printMaze() {
        std::cout << "  ";
        for (int j = 0; j < COLS; j++) {
            std::cout << j << " ";
        }
        std::cout << "\n";

        for (int i = 0; i < ROWS; i++) {
            std::cout << i << " ";
            for (int j = 0; j < COLS; j++) {
                // Using simple ASCII characters for better console compatibility
                if (i == 0 && j == 0) {
                    std::cout << "B "; // Blue exit
                }
                else if (i == ROWS - 1 && j == COLS - 1) {
                    std::cout << "R "; // Red exit
                }
                else {
                    std::cout << (maze[i][j] ? "# " : ". "); // # for wall, . for path
                }
            }
            std::cout << "\n";
        }
    }
};

int main() {
    try {
        // Part a & b: Solve maze with no error
        MazeMDP mdp(0.0);
        std::cout << "Initial maze configuration:\n";
        std::cout << "Legend: B = Blue exit, R = Red exit, # = Wall, . = Path\n\n";
        mdp.printMaze();

        double avg_steps = mdp.simulateWalks(100); // Reduced number of trials for quicker testing
        std::cout << "\nAverage steps with no error: " << avg_steps << std::endl;

        // Part c: Solve maze with error probability 0.1
        MazeMDP mdp_with_error(0.1);
        avg_steps = mdp_with_error.simulateWalks(100); // Reduced number of trials for quicker testing
        std::cout << "Average steps with error probability 0.1: " << avg_steps << std::endl;

    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}