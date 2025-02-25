#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <memory>
#include <limits>
#include <map>

// Structure to represent a 3D point
struct Point3D {
    double x, y, z;
    Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
};

// Structure to represent a product
struct Product {
    double length, width, height;
    double radius;  // for spherical products (Product C)
    double profit;
    char type;
    bool isSphere;

    Product(double l, double w, double h, double p, char t)
        : length(l), width(w), height(h), radius(0), profit(p), type(t), isSphere(false) {}

    Product(double r, double p, char t)
        : length(0), width(0), height(0), radius(r), profit(p), type(t), isSphere(true) {}

    double getVolume() const {
        if (isSphere) {
            return (4.0 / 3.0) * 3.1415927* pow(radius, 3);
        }
        return length * width * height;
    }
};

// Structure to represent the box state
class BoxState {
private:
    const double BOX_LENGTH = 12.5;
    const double BOX_WIDTH = 7.5;
    const double BOX_HEIGHT = 5.5;
    const double MEDIUM_COST = 0.25;  // cost per cubic inch of soft medium

    std::vector<std::pair<Product, Point3D>> placed_products;
    double total_profit;
    bool has_product_a;
    bool has_product_b;
    double remaining_volume;

public:
    BoxState() : total_profit(0), has_product_a(false), has_product_b(false) {
        remaining_volume = BOX_LENGTH * BOX_WIDTH * BOX_HEIGHT;
    }

    bool canPlace(const Product& product, const Point3D& position) const {
        if (position.x + product.length > BOX_LENGTH ||
            position.y + product.width > BOX_WIDTH ||
            position.z + product.height > BOX_HEIGHT) {
            return false;
        }

        // Check for overlap with existing products
        for (const auto& placed : placed_products) {
            if (checkOverlap(product, position, placed.first, placed.second)) {
                return false;
            }
        }

        return true;
    }

    bool checkOverlap(const Product& p1, const Point3D& pos1,
        const Product& p2, const Point3D& pos2) const {
        if (p1.isSphere && p2.isSphere) {
            // Sphere-sphere collision
            double distance = sqrt(pow(pos1.x - pos2.x, 2) +
                pow(pos1.y - pos2.y, 2) +
                pow(pos1.z - pos2.z, 2));
            return distance < (p1.radius + p2.radius);
        }
        else if (p1.isSphere) {
            // Sphere-box collision
            return checkSphereBoxCollision(p1, pos1, p2, pos2);
        }
        else if (p2.isSphere) {
            // Box-sphere collision
            return checkSphereBoxCollision(p2, pos2, p1, pos1);
        }

        // Box-box collision
        return !(pos1.x + p1.length <= pos2.x || pos1.x >= pos2.x + p2.length ||
            pos1.y + p1.width <= pos2.y || pos1.y >= pos2.y + p2.width ||
            pos1.z + p1.height <= pos2.z || pos1.z >= pos2.z + p2.height);
    }

    bool checkSphereBoxCollision(const Product& sphere, const Point3D& sphere_pos,
        const Product& box, const Point3D& box_pos) const {
        // Find closest point on box to sphere center
        Point3D closest;
        closest.x = std::max(box_pos.x, std::min(sphere_pos.x, box_pos.x + box.length));
        closest.y = std::max(box_pos.y, std::min(sphere_pos.y, box_pos.y + box.width));
        closest.z = std::max(box_pos.z, std::min(sphere_pos.z, box_pos.z + box.height));

        double distance = sqrt(pow(closest.x - sphere_pos.x, 2) +
            pow(closest.y - sphere_pos.y, 2) +
            pow(closest.z - sphere_pos.z, 2));
        return distance < sphere.radius;
    }

    bool placeProduct(const Product& product, const Point3D& position) {
        if (!canPlace(product, position)) {
            return false;
        }

        placed_products.push_back({ product, position });
        total_profit += product.profit;
        remaining_volume -= product.getVolume();

        if (product.type == 'A') has_product_a = true;
        if (product.type == 'B') has_product_b = true;

        return true;
    }

    double calculateFinalProfit(bool include_medium_cost) const {
        if (!has_product_a || !has_product_b) {
            return -std::numeric_limits<double>::infinity();
        }

        if (!include_medium_cost) {
            return total_profit;
        }

        double medium_volume = remaining_volume;
        double medium_cost = medium_volume * MEDIUM_COST;
        return total_profit - medium_cost;
    }

    std::vector<Point3D> getPossiblePositions(const Product& product) const {
        std::vector<Point3D> positions;
        double step = 0.5; // Grid step size for position testing

        for (double x = 0; x <= BOX_LENGTH - product.length; x += step) {
            for (double y = 0; y <= BOX_WIDTH - product.width; y += step) {
                for (double z = 0; z <= BOX_HEIGHT - product.height; z += step) {
                    Point3D pos(x, y, z);
                    if (canPlace(product, pos)) {
                        positions.push_back(pos);
                    }
                }
            }
        }
        return positions;
    }
};

// Main optimization function using tree search
double optimizePacking(bool include_medium_cost) {
    Product product_a(4, 3, 2, 3.0, 'A');
    Product product_b(3, 2, 2, 1.5, 'B');
    Product product_c(1, 1.0, 'C');  // Sphere with radius 1

    std::queue<std::shared_ptr<BoxState>> states;
    states.push(std::make_shared<BoxState>());
    double best_profit = -std::numeric_limits<double>::infinity();

    while (!states.empty()) {
        auto current_state = states.front();
        states.pop();

        // Try placing each type of product
        std::vector<Product> products = { product_a, product_b, product_c };

        for (const Product& product : products) {
            auto positions = current_state->getPossiblePositions(product);

            for (const Point3D& position : positions) {
                auto new_state = std::make_shared<BoxState>(*current_state);
                if (new_state->placeProduct(product, position)) {
                    double current_profit = new_state->calculateFinalProfit(include_medium_cost);
                    best_profit = std::max(best_profit, current_profit);
                    states.push(new_state);
                }
            }
        }
    }

    return best_profit;
}

int main() {
    // Solve part (a)
    double best_profit_without_medium = optimizePacking(false);
    std::cout << "Part (a) - Maximum profit without medium cost: $"
        << best_profit_without_medium << std::endl;

    // Solve part (b)
    double best_profit_with_medium = optimizePacking(true);
    std::cout << "Part (b) - Maximum profit with medium cost: $"
        << best_profit_with_medium << std::endl;

    return 0;
}