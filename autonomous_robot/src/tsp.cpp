#include "tsp.hpp"
#include <cmath>
#include <limits>

tsp::tsp() {}

double tsp::calculateDistance(const geometry_msgs::msg::Pose &a, const geometry_msgs::msg::Pose &b) {
    double dx = a.position.x - b.position.x;
    double dy = a.position.y - b.position.y;
    return std::sqrt(dx * dx + dy * dy); // Euclidean distance in 2D plane
}

// Function to return the optimized path as a vector of indices
std::vector<unsigned int> tsp::optimizePath(const geometry_msgs::msg::PoseArray &poses) {
    std::vector<unsigned int> optimized_indices;
    std::vector<bool> visited(poses.poses.size(), false);

    // Start at the first pose
    optimized_indices.push_back(0);
    visited[0] = true;
    size_t currentIndex = 0;

    while (optimized_indices.size() < poses.poses.size()) {
        double nearestDistance = std::numeric_limits<double>::max();
        size_t nearestIndex = 0;

        // Find the nearest unvisited pose
        for (size_t i = 0; i < poses.poses.size(); ++i) {
            if (!visited[i]) {
                double dist = calculateDistance(poses.poses[currentIndex], poses.poses[i]);
                if (dist < nearestDistance) {
                    nearestDistance = dist;
                    nearestIndex = i;
                }
            }
        }

        // Add the nearest pose index to the optimized path
        optimized_indices.push_back(nearestIndex);
        visited[nearestIndex] = true;
        currentIndex = nearestIndex;
    }

    return optimized_indices;
}

// Function to map the poses to their optimized order based on the indices
std::vector<geometry_msgs::msg::Pose> tsp::mapGoalsToOptimizedOrder(const geometry_msgs::msg::PoseArray &poses, const std::vector<unsigned int> &optimized_indices) {
    std::vector<geometry_msgs::msg::Pose> optimized_path;
    
    for (const auto &index : optimized_indices) {
        optimized_path.push_back(poses.poses[index]);
    }

    return optimized_path;
}
