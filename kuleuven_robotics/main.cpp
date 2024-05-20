#include "astar.h"
#include <iostream>


int main() {
    // Initialize the Astar object
    const double resolution = 0.6;
    astar::Astar astar(resolution);

    // Prepare a set for occupied cells
    std::set<astar::Cell> obstacles;
    obstacles.insert(astar.toGrid(0.2, 10.3, -5.5));
    obstacles.insert(astar.toGrid(2, 3, 4));

    // Prepare start and goal (in real-world coords.)
    astar::Position start(-3.2, -4.4, 3.4);
    astar::Position goal(10.2, 0.0, -5.5);

    // Call the planner
    std::optional<std::list<astar::Position>> path = astar.plan(start, goal, obstacles);

    // Process the result
    if (path) {
        std::cout << "path found:\n";
        for (astar::Position pos : path.value()) {
            std::cout << "[" << pos.x() << ", " << pos.y() << ", " << pos.z() << "]\n";
        }
    } else {
        std::cout << "path not found\n";
    }

    return 0;
}