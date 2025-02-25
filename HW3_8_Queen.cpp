#include <iostream>
#include <vector>
using namespace std;

// Queen moveset logics
bool queenMove(vector<int>& board, int row, int col) {
    for (int i = 0; i < row; ++i) {
        if (board[i] == col || // If in the same column
            (board[i] - i == col - row) || // Or in the same diagonal (/)
            (board[i] + i == col + row)) { // Or in the same diagonal (\)
            return false;
        } /* The diagonal works because the absolute difference between the index
          of the rows and columns are always the same for squares on the same diagonal 
          i.e. (2,4 and 4,6; 1,7 and 2, 8) */
    }
    return true;
}

// Using backtracking to solve 8 queen
bool solve(vector<int>& board, int row) {
    if (row == 8) { // Base case; will return true for the entire function when done
        return true;
    }
    // Find the first safe column in the selected row
    for (int col = 0; col < 8; ++col) {
        if (queenMove(board, row, col)) {
            board[row] = col;  // Place the queen in the board[row][col] if queenMove is true
            if (solve(board, row + 1)) { // Nested function within function to recur, but move to next row
                return true;
            }
            // Backtrack if the placed queen does not work and try another column with the for loop
            board[row] = -1;
        }
    }
    return false;  // Safe position cannot be found
}

// Function to print the solution
void printSolution(const vector<int>& board) {
    for (int row = 0; row < 8; ++row) {
        for (int col = 0; col < 8; ++col) {
            if (board[row] == col) {
                cout << "Q ";  // Print a queen
            }
            else {
                cout << "x ";  // Print an empty space
            }
        }
        cout << endl;
    }
}

// Main code to run functions
int main() {
    vector<int> board(8, -1);  // Initialize the board (no queen placed)

    if (solve(board, 0)) {
        printSolution(board);  // Print the solution
    }
    else {
        cout << "No solution exists" << endl;
    }

    return 0;
}


/* The time complexity of this problem should be O=(8!), as there could be no more than one queeen per row
or column, leading to 8*7*6*5*4*3*2*1 possibilites. In other words, the problem is O=(N!) */