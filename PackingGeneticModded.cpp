#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>

// Initialize product dimensions and 3D positioning
struct Point3D {
    double x, y, z;
    Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
};

struct Product {
    double width, height, depth;
    double radius;
    double profit;
    char type;
    bool isSphere;

    Product(double w, double h, double d, double p, char t)
        : width(w), height(h), depth(d), radius(0), profit(p), type(t), isSphere(false) {}

    Product(double r, double p, char t)
        : width(0), height(0), depth(0), radius(r), profit(p), type(t), isSphere(true) {}

    double getVolume() const {
        if (isSphere) {
            return (4.0 / 3.0) * M_PI * pow(radius, 3);
        }
        return width * height * depth;
    }
};

struct ProductPlacement {
    Point3D position;
    const Product* product;
    
    ProductPlacement(const Point3D& pos, const Product* prod) 
        : position(pos), product(prod) {}
};

// Box properties - Constraints
const double BOX_WIDTH = 12.5;
const double BOX_HEIGHT = 7.5;
const double BOX_DEPTH = 5.5;
const double SOFT_MEDIUM_COST = 0.25;  // per cubic inch

// Define products
const Product PRODUCT_A(4, 3, 2, 3.0, 'A');
const Product PRODUCT_B(3, 2, 2, 1.5, 'B');
const Product PRODUCT_C(1, 1.0, 'C');  // Sphere with radius 1

// Collision Detection System
class CollisionDetector {
public:
    static bool checkCollision(const ProductPlacement& p1, const ProductPlacement& p2) {
        if (p1.product->isSphere && p2.product->isSphere) {
            return checkSphereSphereCollision(p1, p2);
        }
        else if (p1.product->isSphere) {
            return checkSphereBoxCollision(p1, p2);
        }
        else if (p2.product->isSphere) {
            return checkSphereBoxCollision(p2, p1);
        }
        return checkBoxBoxCollision(p1, p2);
    }

private:
    static bool checkSphereSphereCollision(const ProductPlacement& s1, const ProductPlacement& s2) {
        double dx = s1.position.x - s2.position.x;
        double dy = s1.position.y - s2.position.y;
        double dz = s1.position.z - s2.position.z;
        double distance = sqrt(dx*dx + dy*dy + dz*dz);
        return distance < (s1.product->radius + s2.product->radius);
    }

    static bool checkBoxBoxCollision(const ProductPlacement& b1, const ProductPlacement& b2) {
        bool xOverlap = !(b1.position.x + b1.product->width <= b2.position.x || 
                         b2.position.x + b2.product->width <= b1.position.x);
        
        bool yOverlap = !(b1.position.y + b1.product->height <= b2.position.y || 
                         b2.position.y + b2.product->height <= b1.position.y);
        
        bool zOverlap = !(b1.position.z + b1.product->depth <= b2.position.z || 
                         b2.position.z + b2.product->depth <= b1.position.z);
        
        return xOverlap && yOverlap && zOverlap;
    }

    static bool checkSphereBoxCollision(const ProductPlacement& sphere, const ProductPlacement& box) {
        Point3D closest;
        
        closest.x = std::max(box.position.x, 
                    std::min(sphere.position.x, box.position.x + box.product->width));
        closest.y = std::max(box.position.y, 
                    std::min(sphere.position.y, box.position.y + box.product->height));
        closest.z = std::max(box.position.z, 
                    std::min(sphere.position.z, box.position.z + box.product->depth));
        
        double dx = closest.x - sphere.position.x;
        double dy = closest.y - sphere.position.y;
        double dz = closest.z - sphere.position.z;
        
        double distance = sqrt(dx*dx + dy*dy + dz*dz);
        return distance < sphere.product->radius;
    }
};

class PackingValidator {
private:
    std::vector<ProductPlacement> placements;
    double boxWidth, boxHeight, boxDepth;

public:
    PackingValidator(double width, double height, double depth) 
        : boxWidth(width), boxHeight(height), boxDepth(depth) {}
    
    bool addProduct(const Product* product, const Point3D& position) {
        ProductPlacement newPlacement(position, product);
        
        if (!isWithinBounds(newPlacement)) {
            return false;
        }
        
        for (const auto& existing : placements) {
            if (CollisionDetector::checkCollision(newPlacement, existing)) {
                return false;
            }
        }
        
        placements.push_back(newPlacement);
        return true;
    }
    
    void clear() {
        placements.clear();
    }
    
private:
    bool isWithinBounds(const ProductPlacement& placement) {
        if (placement.product->isSphere) {
            return (placement.position.x - placement.product->radius >= 0 &&
                   placement.position.x + placement.product->radius <= boxWidth &&
                   placement.position.y - placement.product->radius >= 0 &&
                   placement.position.y + placement.product->radius <= boxHeight &&
                   placement.position.z - placement.product->radius >= 0 &&
                   placement.position.z + placement.product->radius <= boxDepth);
        } else {
            return (placement.position.x >= 0 &&
                   placement.position.x + placement.product->width <= boxWidth &&
                   placement.position.y >= 0 &&
                   placement.position.y + placement.product->height <= boxHeight &&
                   placement.position.z >= 0 &&
                   placement.position.z + placement.product->depth <= boxDepth);
        }
    }
};

class PackingSolution {
public:
    std::vector<Product> products;
    std::vector<Point3D> positions;
    double fitness;
    bool isValid;

    PackingSolution() : fitness(0), isValid(false) {}

    void calculateFitness(bool includeMediumCost) {
        PackingValidator validator(BOX_WIDTH, BOX_HEIGHT, BOX_DEPTH);
        
        double totalVolume = 0;
        double totalProfit = 0;
        bool hasA = false;
        bool hasB = false;

        // Check product placement and collisions
        for (size_t i = 0; i < products.size(); i++) {
            if (!validator.addProduct(&products[i], positions[i])) {
                isValid = false;
                fitness = 0;
                return;
            }
            
            totalVolume += products[i].getVolume();
            totalProfit += products[i].profit;
            
            if (products[i].type == 'A') hasA = true;
            if (products[i].type == 'B') hasB = true;
        }

        double boxVolume = BOX_WIDTH * BOX_HEIGHT * BOX_DEPTH;
        isValid = (totalVolume <= boxVolume) && hasA && hasB;

        if (!isValid) {
            fitness = 0;
            return;
        }

        if (includeMediumCost) {
            double emptyVolume = boxVolume - totalVolume;
            double mediumCost = emptyVolume * SOFT_MEDIUM_COST;
            fitness = totalProfit - mediumCost;
        }
        else {
            fitness = totalProfit;
        }
    }
};

class GeneticAlgorithm {
private:
    std::mt19937 rng;
    int populationSize;
    int maxGenerations;
    bool includeMediumCost;
    std::vector<PackingSolution> population;

    PackingSolution createRandomSolution() {
        PackingSolution solution;

        // Always add at least one A and one B
        solution.products.push_back(PRODUCT_A);
        solution.products.push_back(PRODUCT_B);

        // Generate positions for initial products
        for (const auto& product : solution.products) {
            Point3D position = generateValidPosition(product);
            solution.positions.push_back(position);
        }

        // Randomly add more products
        int maxAttempts = 55;
        for (int i = 0; i < maxAttempts; i++) {
            double rand = std::uniform_real_distribution<>(0, 1)(rng);
            Product newProduct = (rand < 0.3) ? PRODUCT_A : 
                               (rand < 0.6) ? PRODUCT_B : PRODUCT_C;
            
            Point3D position = generateValidPosition(newProduct);
            solution.products.push_back(newProduct);
            solution.positions.push_back(position);
        }

        solution.calculateFitness(includeMediumCost);
        return solution;
    }

    Point3D generateValidPosition(const Product& product) {
        double maxX = BOX_WIDTH - (product.isSphere ? 2 * product.radius : product.width);
        double maxY = BOX_HEIGHT - (product.isSphere ? 2 * product.radius : product.height);
        double maxZ = BOX_DEPTH - (product.isSphere ? 2 * product.radius : product.depth);
        
        return Point3D(
            std::uniform_real_distribution<>(0, maxX)(rng),
            std::uniform_real_distribution<>(0, maxY)(rng),
            std::uniform_real_distribution<>(0, maxZ)(rng)
        );
    }

    PackingSolution crossover(const PackingSolution& parent1, const PackingSolution& parent2) {
        PackingSolution child;

        // Ensure at least one A and one B
        child.products.push_back(PRODUCT_A);
        child.products.push_back(PRODUCT_B);
        child.positions.push_back(parent1.positions[0]);
        child.positions.push_back(parent1.positions[1]);

        // Randomly select products and positions from parents
        size_t maxSize = std::max(parent1.products.size(), parent2.products.size());
        for (size_t i = 2; i < maxSize; i++) {
            if (std::uniform_real_distribution<>(0, 1)(rng) < 0.5) {
                if (i < parent1.products.size()) {
                    child.products.push_back(parent1.products[i]);
                    child.positions.push_back(parent1.positions[i]);
                }
            }
            else {
                if (i < parent2.products.size()) {
                    child.products.push_back(parent2.products[i]);
                    child.positions.push_back(parent2.positions[i]);
                }
            }
        }

        child.calculateFitness(includeMediumCost);
        return child;
    }

    void mutate(PackingSolution& solution) {
        if (std::uniform_real_distribution<>(0, 1)(rng) < 0.1) {
            int operation = std::uniform_int_distribution<>(0, 3)(rng);

            switch (operation) {
                case 0: // Add a random product
                {
                    double rand = std::uniform_real_distribution<>(0, 1)(rng);
                    Product newProduct = (rand < 0.3) ? PRODUCT_A : 
                                       (rand < 0.6) ? PRODUCT_B : PRODUCT_C;
                    Point3D position = generateValidPosition(newProduct);
                    solution.products.push_back(newProduct);
                    solution.positions.push_back(position);
                }
                break;
                
                case 1: // Remove a random product (but keep at least one A and one B)
                    if (solution.products.size() > 2) {
                        int indexToRemove = std::uniform_int_distribution<>(2, solution.products.size() - 1)(rng);
                        solution.products.erase(solution.products.begin() + indexToRemove);
                        solution.positions.erase(solution.positions.begin() + indexToRemove);
                    }
                    break;
                
                case 2: // Swap two products
                    if (solution.products.size() > 2) {
                        int idx1 = std::uniform_int_distribution<>(2, solution.products.size() - 1)(rng);
                        int idx2 = std::uniform_int_distribution<>(2, solution.products.size() - 1)(rng);
                        std::swap(solution.products[idx1], solution.products[idx2]);
                        std::swap(solution.positions[idx1], solution.positions[idx2]);
                    }
                    break;
                
                case 3: // Modify a random position
                    if (!solution.positions.empty()) {
                        int idx = std::uniform_int_distribution<>(0, solution.positions.size() - 1)(rng);
                        solution.positions[idx] = generateValidPosition(solution.products[idx]);
                    }
                    break;
            }

            solution.calculateFitness(includeMediumCost);
        }
    }

public:
    GeneticAlgorithm(int popSize, int maxGen, bool includeMedium)
        : populationSize(popSize), maxGenerations(maxGen), includeMediumCost(includeMedium) {
        std::random_device rd;
        rng = std::mt19937(rd());
    }

    PackingSolution solve() {
        // Initialize population
        for (int i = 0; i < populationSize; i++) {
            population.push_back(createRandomSolution());
        }

        PackingSolution bestSolution;
        bestSolution.fitness = -std::numeric_limits<double>::infinity();

        // Main genetic algorithm loop
        for (int generation = 0; generation < maxGenerations; generation++) {
            // Sort population by fitness
            std::sort(population.begin(), population.end(),
                [](const PackingSolution& a, const PackingSolution& b) {
                    return a.fitness > b.fitness;
                });

            // Update best solution
            if (population[0].fitness > bestSolution.fitness && population[0].isValid) {
                bestSolution = population[0];
            }

            // Create new generation
            std::vector<PackingSolution> newPopulation;

            // Elitism: keep best solutions
            int eliteSize = populationSize / 10;
            for (int i = 0; i < eliteSize; i++) {
                newPopulation.push_back(population[i]);

            // Elitism: keep best solutions
            int eliteSize = populationSize / 10;
            for (int i = 0; i < eliteSize; i++) {
                newPopulation.push_back(population[i]);
            }

            // Create rest of new population
            while (newPopulation.size() < populationSize) 
                // Tournament selection
                int idx1 = std::uniform_int_distribution<>(0, populationSize - 1)(rng);
                int idx2 = std::uniform_int_distribution<>(0, populationSize - 1)(rng);
                PackingSolution parent1 = population[idx1].fitness > population[idx2].fitness ?
                    population[idx1] : population[idx2];

                idx1 = std::uniform_int_distribution<>(0, populationSize - 1)(rng);
                idx2 = std::uniform_int_distribution<>(0, populationSize - 1)(rng);
                PackingSolution parent2 = population[idx1].fitness > population[idx2].fitness ?
                    population[idx1] : population[idx2];

                // Crossover and mutation
                PackingSolution child = crossover(parent1, parent2);
                mutate(child);

                newPopulation.push_back(child);
            }

            population = newPopulation;
        }

        return bestSolution;
    }
};

int main() {
    // Part A: Maximize profit without considering soft medium
    GeneticAlgorithm gaA(100, 1000, false);
    PackingSolution solutionA = gaA.solve();

    std::cout << "Part A Solution:\n";
    std::cout << "Total profit: $" << solutionA.fitness << std::endl;
    std::cout << "Products used:\n";
    int countA = 0, countB = 0, countC = 0;
    for (const auto& product : solutionA.products) {
        if (product.type == 'A') countA++;
        else if (product.type == 'B') countB++;
        else countC++;
    }
    std::cout << "Product A: " << countA << std::endl;
    std::cout << "Product B: " << countB << std::endl;
    std::cout << "Product C: " << countC << std::endl;

    // Part B: Maximize profit considering soft medium cost
    GeneticAlgorithm gaB(100, 10000, true);
    PackingSolution solutionB = gaB.solve();

    std::cout << "\nPart B Solution:\n";
    std::cout << "Total profit (after medium cost): $" << solutionB.fitness << std::endl;
    std::cout << "Products used:\n";
    countA = 0, countB = 0, countC = 0;
    for (const auto& product : solutionB.products) {
        if (product.type == 'A') countA++;
        else if (product.type == 'B') countB++;
        else countC++;
    }
    std::cout << "Product A: " << countA << std::endl;
    std::cout << "Product B: " << countB << std::endl;
    std::cout << "Product C: " << countC << std::endl;

    return 0;
}