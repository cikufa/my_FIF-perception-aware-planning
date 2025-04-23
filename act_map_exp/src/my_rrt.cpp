#ifndef VOXBLOX_ROS_ESDF_SERVER_H_
#define VOXBLOX_ROS_ESDF_SERVER_H_


#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <ctime>
#include <vector>
#include <typeinfo>
#include <chrono>
#include <fstream>
#include <iostream>

// OMPL includes
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/config.h>

// Voxblox includes
#include <voxblox/core/esdf_map.h>
#include <voxblox/io/layer_io.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

std::shared_ptr<voxblox::EsdfMap> loadEsdfMapFromFile(const std::string& filepath) {
    // Create a new ESDF map with default config
    voxblox::EsdfMap::Config esdf_config;
    auto esdf_map = std::make_shared<voxblox::EsdfMap>(esdf_config);
    
    // Load the ESDF layer from file
    bool success = voxblox::io::LoadLayer<voxblox::EsdfVoxel>(filepath, 
                                                             esdf_map->getEsdfLayerPtr());
    if (!success) {
        std::cerr << "Failed to load ESDF map from " << filepath << std::endl;
        return nullptr;
    }
    
    return esdf_map;
}

int main(int argc, char **argv) {
    std::string esdf_file_path = "../exp_data/warehouse_voxblox/tsdf_esdf_max10.vxblx";
    std::shared_ptr<voxblox::EsdfMap> esdf_map;
    esdf_map = loadEsdfMapFromFile(esdf_file_path);
    
    if (!esdf_map) {
        std::cerr << "Failed to load ESDF map, exiting." << std::endl;
        return -1;
    }

    double clearance_thresh = 2;  // Robot radius
    auto isStateValid = [esdf_map, clearance_thresh](const ob::State *state) -> bool {
        const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
        double x = s->values[0];
        double y = s->values[1];
        double z = s->values[2];
        Eigen::Vector3d pos(x, y, z);
        double dist = 0.0;
        
        bool res = esdf_map->getDistanceAtPosition(pos, &dist);
        if (!res) {
            std::cerr << "Position " << pos.transpose() << " query failed." << std::endl;
            return false;
        }
        return (dist > clearance_thresh);
    };
    
    // Create state space
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));
    
    // Get bounds from the ESDF map
    
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, -22.5);
    bounds.setHigh(0, 2.5);
    bounds.setLow(1, 14.5);
    bounds.setHigh(1, 64.5);
    bounds.setLow(2, 1);
    bounds.setHigh(2, 3);
    space->setBounds(bounds);
    
    // Create SimpleSetup
    og::SimpleSetup ss(space);
    ss.setStateValidityChecker(isStateValid);
    
    // Set start and goal states
    ob::ScopedState<> start(space);
    start[0] = -9.0;  // Adjust based on your environment
    start[1] = -9.0;
    start[2] = 1.0;
    
    ob::ScopedState<> goal(space);
    goal[0] = 9.0;  // Adjust based on your environment
    goal[1] = 9.0;
    goal[2] = 1.0;
    
    ss.setStartAndGoalStates(start, goal);
    
    // Setup the problem
    auto si = ss.getSpaceInformation();
    ss.setup();
    
    // Plan using RRT*
    auto planner = std::make_shared<og::RRTstar>(si);
    
    // Configure RRT* parameters
    planner->setRange(5.0);  // Maximum distance to extend tree
    planner->setGoalBias(0.1);  // 10% probability of sampling the goal
    
    ss.setPlanner(planner);
    
    // Solve
    ob::PlannerStatus solved = ss.solve(10.0);
    
    if (solved) {
        std::cout << "Found a solution!" << std::endl;
        ss.simplifySolution();
        og::PathGeometric& path = ss.getSolutionPath();
        path.printAsMatrix(std::cout);
        
        // Save path to file
        std::ofstream outFile("my_solution_path.txt");
        if (outFile.is_open()) {
            for (size_t i = 0; i < path.getStateCount(); ++i) {
                const ob::State* state = path.getState(i);
                const auto* state3D = state->as<ob::RealVectorStateSpace::StateType>();
                outFile << state3D->values[0] << " " << state3D->values[1] << " " 
                       << state3D->values[2] << "\n";
            }
            outFile.close();
            std::cout << "Solution path saved to 'solution_path.txt'" << std::endl;
        }
    } else {
        std::cout << "No solution found." << std::endl;
    }
    
    return 0;
}
#endif  // VOXBLOX_ROS_ESDF_SERVER_H_
