#include <iostream>
#include <vector>
#include <random>
#include <queue>
#include <unordered_map>
#include <ctime>
#include <cstdlib>

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
        double cost;
        bool found;
        int steps;

        PathResult() : cost(-1), found(false), steps(0) {}
        PathResult(const std::vector<std::pair<int, int>>& p, double c)
            : path(p), cost(c), found(true), steps(static_cast<int>(p.size()) - 1) {}
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
                    valid_moves.push_back(std::make_pair(new_x, new_y));
                }
            }
        }
        return valid_moves;
    }

    double getManhattanDistance(const std::pair<int, int>& a, const std::pair<int, int>& b) {
        return static_cast<double>(std::abs(a.first - b.first) + std::abs(a.second - b.second));
    }

    PathResult findPathMDP(const std::pair<int, int>& start, const std::pair<int, int>& end) {
        if (!isValid(start.first, start.second) || !isValid(end.first, end.second)) {
            return PathResult();
        }

        using PQEntry = std::pair<double, std::pair<int, int>>;
        std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;
        std::unordered_map<int, double> cost_so_far;
        std::unordered_map<int, std::pair<int, int>> came_from;

        pq.push(std::make_pair(0.0, start));
        cost_so_far[start.first * COLS + start.second] = 0.0;

        while (!pq.empty()) {
            auto current_pair = pq.top();
            double current_cost = current_pair.first;
            std::pair<int, int> current = current_pair.second;
            pq.pop();

            if (current == end) {
                std::vector<std::pair<int, int>> path;
                auto pos = current;
                while (pos != start) {
                    path.push_back(pos);
                    pos = came_from[pos.first * COLS + pos.second];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());

                std::cout << "\nFound path from (" << start.first << "," << start.second
                    << ") to (" << end.first << "," << end.second << "):\n";
                printMaze(path);

                return PathResult(path, cost_so_far[current.first * COLS + current.second]);
            }

            for (const auto& next : getValidMoves(current.first, current.second)) {
                double new_cost = cost_so_far[current.first * COLS + current.second] +
                    getManhattanDistance(current, next);
                int next_key = next.first * COLS + next.second;

                if (cost_so_far.find(next_key) == cost_so_far.end() || new_cost < cost_so_far[next_key]) {
                    cost_so_far[next_key] = new_cost;
                    came_from[next_key] = current;
                    pq.push(std::make_pair(new_cost, next));
                }
            }
        }

        return PathResult();
    }

    void printMaze(const std::vector<std::pair<int, int>>& path = std::vector<std::pair<int, int>>()) const {
        std::vector<std::vector<char>> maze_chars(ROWS, std::vector<char>(COLS));

        // Convert the numeric maze to characters
        for (int i = 0; i < ROWS; i++) {
            for (int j = 0; j < COLS; j++) {
                maze_chars[i][j] = maze[i][j] == 1 ? '#' : ' ';
            }
        }

        // Mark the path with asterisks
        for (const auto& pos : path) {
            maze_chars[pos.first][pos.second] = '*';
        }

        // Mark the special positions (blue and red exits)
        maze_chars[1][0] = 'B';    // Blue exit
        maze_chars[14][15] = 'R';  // Red exit

        // Print column numbers
        std::cout << "  ";
        for (int j = 0; j < COLS; j++) {
            std::cout << (j / 10);  // Print tens digit
        }
        std::cout << "\n  ";
        for (int j = 0; j < COLS; j++) {
            std::cout << (j % 10);  // Print ones digit
        }
        std::cout << "\n";

        // Print the maze with row numbers
        for (int i = 0; i < ROWS; i++) {
            std::cout << (i < 10 ? " " : "") << i << " ";  // Row number
            for (int j = 0; j < COLS; j++) {
                std::cout << maze_chars[i][j];
            }
            std::cout << "\n";
        }
    }

    double simulateWalks(int num_trials = 1000) {
        std::pair<int, int> blue_exit = std::make_pair(1, 0);
        std::pair<int, int> red_exit = std::make_pair(14, 15);

        int total_steps = 0;
        int successful_trials = 0;

        for (int i = 0; i < num_trials; i++) {
            PathResult forward_result = findPathMDP(blue_exit, red_exit);
            if (forward_result.found) {
                PathResult reverse_result = findPathMDP(red_exit, blue_exit);
                if (reverse_result.found) {
                    total_steps += forward_result.steps + reverse_result.steps;
                    successful_trials++;
                }
            }
        }

        return successful_trials > 0 ?
            static_cast<double>(total_steps) / successful_trials : -1.0;
    }
};

int main() {
    try {
        MazeMDP mdp(0.0);
        std::cout << "Initial maze configuration:\n";
        std::cout << "Legend: B = Blue exit, R = Red exit, # = Wall, Space = Path, * = Solution path\n\n";
        mdp.printMaze();

        std::pair<int, int> blue_exit = std::make_pair(1, 0);
        std::pair<int, int> red_exit = std::make_pair(14, 15);

        std::cout << "\nFinding path from blue exit to red exit...\n";
        auto result = mdp.findPathMDP(blue_exit, red_exit);

        if (!result.found) {
            std::cout << "No path found!\n";
        }

        double avg_mdp_steps = mdp.simulateWalks(100);
        std::cout << "\nAverage steps with MDP and no error: " << avg_mdp_steps << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}