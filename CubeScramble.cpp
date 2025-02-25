#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <string>
#include <memory>

class PuzzleState {
private:
    std::vector<std::vector<int>> board;
    std::shared_ptr<PuzzleState> parent;
    std::string action;
    int cost;
    std::pair<int, int> blank_pos;

    std::pair<int, int> findBlank() const {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (board[i][j] == 0) {
                    return { i, j };
                }
            }
        }
        return { -1, -1 };
    }

public:
    PuzzleState(const std::vector<std::vector<int>>& b,
        std::shared_ptr<PuzzleState> p = nullptr,
        const std::string& a = "",
        int c = 0) : board(b), parent(p), action(a), cost(c) {
        blank_pos = findBlank();
    }

    std::vector<std::pair<std::string, std::pair<int, int>>> getPossibleMoves() const {
        std::vector<std::pair<std::string, std::pair<int, int>>> moves;
        int x = blank_pos.first;
        int y = blank_pos.second;

        std::vector<std::pair<std::string, std::pair<int, int>>> possible_moves = {
            {"Up", {x - 1, y}},
            {"Down", {x + 1, y}},
            {"Left", {x, y - 1}},
            {"Right", {x, y + 1}}
        };

        for (const auto& move : possible_moves) {
            int new_x = move.second.first;
            int new_y = move.second.second;
            if (new_x >= 0 && new_x < 3 && new_y >= 0 && new_y < 3) {
                moves.push_back(move);
            }
        }
        return moves;
    }

    std::shared_ptr<PuzzleState> makeMove(const std::pair<std::string, std::pair<int, int>>& move) const {
        std::vector<std::vector<int>> new_board = board;
        int x = blank_pos.first;
        int y = blank_pos.second;
        int new_x = move.second.first;
        int new_y = move.second.second;

        std::swap(new_board[x][y], new_board[new_x][new_y]);
        return std::make_shared<PuzzleState>(new_board,
            std::make_shared<PuzzleState>(*this),
            move.first,
            cost + 1);
    }

    bool isGoal(const std::vector<std::vector<int>>& goal_state) const {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (board[i][j] != goal_state[i][j]) {
                    return false;
                }
            }
        }
        return true;
    }

    const std::vector<std::vector<int>>& getBoard() const { return board; }
    const std::string& getAction() const { return action; }
};

// Custom hash function for the board state
struct BoardHash {
    std::size_t operator()(const std::vector<std::vector<int>>& board) const {
        std::size_t hash = 0;
        for (const auto& row : board) {
            for (int val : row) {
                hash = hash * 31 + val;
            }
        }
        return hash;
    }
};

std::vector<std::string> solvePuzzle(const std::vector<std::vector<int>>& initial_state,
    const std::vector<std::vector<int>>& goal_state) {
    auto initial = std::make_shared<PuzzleState>(initial_state);
    if (initial->isGoal(goal_state)) {
        return std::vector<std::string>();
    }

    std::queue<std::pair<std::shared_ptr<PuzzleState>, std::vector<std::string>>> frontier;
    std::set<std::size_t> explored;
    frontier.push({ initial, std::vector<std::string>() });

    while (!frontier.empty()) {
        auto [current_state, path] = frontier.front();
        frontier.pop();

        // Hash the current board state
        std::size_t board_hash = BoardHash()(current_state->getBoard());
        if (explored.find(board_hash) != explored.end()) {
            continue;
        }

        explored.insert(board_hash);

        for (const auto& move : current_state->getPossibleMoves()) {
            auto new_state = current_state->makeMove(move);
            std::size_t new_board_hash = BoardHash()(new_state->getBoard());

            if (explored.find(new_board_hash) == explored.end()) {
                if (new_state->isGoal(goal_state)) {
                    path.push_back(move.first);
                    return path;
                }
                std::vector<std::string> new_path = path;
                new_path.push_back(move.first);
                frontier.push({ new_state, new_path });
            }
        }
    }

    return std::vector<std::string>(); // No solution found
}

int main() {
    // Example usage
    std::vector<std::vector<int>> initial_state = {
        {1, 2, 0},
        {3, 4, 5},
        {6, 7, 8}
    };

    std::vector<std::vector<int>> goal_state = {
        {1, 4, 2},
        {3, 5, 8},
        {6, 7, 0}
    };

    std::vector<std::string> solution = solvePuzzle(initial_state, goal_state);

    std::cout << "Solution path: ";
    for (const auto& move : solution) {
        std::cout << move << " ";
    }
    std::cout << std::endl;

    return 0;
}