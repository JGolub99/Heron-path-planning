#include "astar.h"
#include <iostream>
#include <cmath>


int main() {

    enum State { Planning, Moving, Idle }; // Define the possible states
    
    /* Inputs: */

    State state = Idle; 



    // Prepare start and goal (in real-world coords.)
    astar::Position start(3.2, -4.4, 3.4);
    astar::Position goal(10.2, 0.0, -5.5);
    double bearing = 3; //Radians

    if(std::abs(start.x()-goal.x())==0.1 && std::abs(start.x()-goal.x())==0.1){
        state = Idle;
    }

    switch(state){
        case Idle:
        std::cout << "In an idle state" << std::endl;
        break;

        case Planning:
        // Initialize the Astar object
        const double resolution = 0.6;
        const int max_iters = 1000000;
        astar::Astar astar(resolution,max_iters);

        // Prepare a set for occupied cells
        std::set<astar::Cell> obstacles;

        // Call the planner
        std::optional<std::list<astar::Position>> path = astar.plan(start, goal, obstacles);

        /*
        Now we will calculate the velocity and yaw to reach the 2nd pose on the Astar path
        */

        if (path) {
            auto it = path.value().begin();
            const astar::Position& firstPos = *it;
            std::advance(it, 1);
            const astar::Position& secondPos = *it;
            
            std::cout << "The first is: " << firstPos.x() << ", " << firstPos.y() << std::endl;
            std::cout << "The second is: " << secondPos.x() << ", " << secondPos.y() << std::endl;

            double theta1 = std::atan2(secondPos.x()-firstPos.x(),secondPos.y()-firstPos.y());
            double yaw = theta1 - bearing;

            double speed = std::sqrt(std::pow(secondPos.x()-firstPos.x(),2) + std::pow(secondPos.y()-firstPos.y(),2));

            std::cout << "The required yaw is: " << yaw << std::endl;
            std::cout << "The required speed is: " << speed << std::endl;

            /* Here we will test to make sure that the yaw and speed are correct */

            double y2 = firstPos.y() + speed*std::cos(bearing+yaw);
            double x2 = firstPos.x() + speed*std::sin(bearing+yaw);

            std::cout << "The new coordinates are: (" << x2 << "," << y2 << ")" << std::endl; 

        }
        break;

    }


    return 0;
}