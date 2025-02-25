#include <iostream>
#include <vector>

using namespace std;

// Mean of dataset
double mean(const vector<double>& values) {
    double sum = 0.0;
    for (double value : values) {
        sum += value;
    }
    return sum / values.size();
}

// Slope and Y-intercept of best-fit
pair<double, double> linearRegression(const vector<double>& t, const vector<double>& y) {
    double t_mean = mean(t);
    double y_mean = mean(y);

    double numerator = 0.0;
    double denominator = 0.0;

    for (int i = 0; i < t.size(); ++i) {
        numerator += (t[i] - t_mean) * (y[i] - y_mean);
        denominator += (t[i] - t_mean) * (t[i] - t_mean);
    }

    double b1 = numerator / denominator;  // slope
    double b0 = y_mean - b1 * t_mean;     // intercept

    return {b0, b1};
}

// Best-fit regression
double predict(double t, double b0, double b1) {
    return b0 + b1 * t;
}

int main() {
    // Time vector (0, 1, 2, 3, 4) corresponding to the 5 seconds of observations
    vector<double> t = {0, 1, 2, 3, 4};

    // Coordinates of the robot arm as given
    vector<double> x_coords = {-1.3, -0.9, 0.1, 0.8, 1.2};  // x-coordinates
    vector<double> y_coords = {-2.4, -0.8, -0.1, 1.3, 1.7}; // y-coordinates

    // Perform x-axis linear regression
    pair<double, double> x_params = linearRegression(t, x_coords);
    double b0_x = x_params.first;  // intercept for x
    double b1_x = x_params.second; // slope for x

    // Perform y-axis linear regression
    pair<double, double> y_params = linearRegression(t, y_coords);
    double b0_y = y_params.first;  // intercept for y
    double b1_y = y_params.second; // slope for y

    // Regress x and y coordinates at t = 5 based on best-fit model
    double predicted_x = predict(5, b0_x, b1_x);
    double predicted_y = predict(5, b0_y, b1_y);

    // Return the coordinates at t = 5
    cout << "Predicted location at t = 5: (" << predicted_x << ", " << predicted_y << ")" << endl;

    return 0;
}
