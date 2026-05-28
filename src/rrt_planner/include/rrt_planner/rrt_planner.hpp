#pragma once

#include <string>
#include <vector>
#include <memory>
#include <random>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"

namespace rrt_planner
{

struct Node
{
  double x, y;
  int parent_idx;  // -1 for root
  double cost;     // accumulated path cost (for RRT*)
};

class RRTPlanner : public nav2_core::GlobalPlanner
{
public:
  RRTPlanner() = default;
  ~RRTPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // Nearest node index in tree
  int nearestNeighbor(const std::vector<Node> & tree, double x, double y) const;

  // Steer from nearest toward sample by step_size
  Node steer(const Node & nearest, double tx, double ty) const;

  // Bresenham collision check in costmap coordinates
  bool collisionFree(double x1, double y1, double x2, double y2) const;

  // Collect nodes within rewire_radius (for RRT*)
  std::vector<int> near(const std::vector<Node> & tree, double x, double y) const;

  // Convert world coords to costmap cell
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;

  // Publish RRT tree for RViz visualization
  void publishTree(const std::vector<Node> & tree);

  // Extract path by backtracking parent pointers
  nav_msgs::msg::Path extractPath(
    const std::vector<Node> & tree,
    int goal_idx,
    const std::string & frame_id);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  std::string global_frame_;

  // Parameters
  int max_iterations_{5000};
  double step_size_{0.1};
  double goal_tolerance_{0.2};
  double goal_bias_{0.05};
  bool enable_rrt_star_{true};
  double rewire_radius_{0.5};

  // RViz publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tree_pub_;

  std::mt19937 rng_{std::random_device{}()};
};

}  // namespace rrt_planner
