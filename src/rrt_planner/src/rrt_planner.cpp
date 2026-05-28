#include "rrt_planner/rrt_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_util/node_utils.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace rrt_planner
{

void RRTPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto node = node_.lock();
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".max_iterations",  rclcpp::ParameterValue(5000));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".step_size",       rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".goal_tolerance",  rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".goal_bias",       rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".enable_rrt_star", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, name_ + ".rewire_radius",   rclcpp::ParameterValue(0.5));

  node->get_parameter(name_ + ".max_iterations",  max_iterations_);
  node->get_parameter(name_ + ".step_size",        step_size_);
  node->get_parameter(name_ + ".goal_tolerance",   goal_tolerance_);
  node->get_parameter(name_ + ".goal_bias",        goal_bias_);
  node->get_parameter(name_ + ".enable_rrt_star",  enable_rrt_star_);
  node->get_parameter(name_ + ".rewire_radius",    rewire_radius_);

  tree_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/rrt_tree", rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(node->get_logger(), "RRTPlanner configured (RRT*=%s, max_iter=%d)",
    enable_rrt_star_ ? "ON" : "OFF", max_iterations_);
}

void RRTPlanner::cleanup()  {}
void RRTPlanner::activate() {}
void RRTPlanner::deactivate() {}

nav_msgs::msg::Path RRTPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  auto node = node_.lock();

  // Refresh costmap pointer (may have changed)
  costmap_ = costmap_ros_->getCostmap();

  double sx = start.pose.position.x, sy = start.pose.position.y;
  double gx = goal.pose.position.x,  gy = goal.pose.position.y;

  // Costmap bounds for random sampling
  double origin_x = costmap_->getOriginX();
  double origin_y = costmap_->getOriginY();
  double map_w    = costmap_->getSizeInMetersX();
  double map_h    = costmap_->getSizeInMetersY();

  std::uniform_real_distribution<double> rand_x(origin_x, origin_x + map_w);
  std::uniform_real_distribution<double> rand_y(origin_y, origin_y + map_h);
  std::uniform_real_distribution<double> rand_bias(0.0, 1.0);

  std::vector<Node> tree;
  tree.reserve(max_iterations_);
  tree.push_back({sx, sy, -1, 0.0});

  int goal_node_idx = -1;

  for (int iter = 0; iter < max_iterations_; ++iter) {
    // Sample: goal bias or random
    double rx, ry;
    if (rand_bias(rng_) < goal_bias_) {
      rx = gx; ry = gy;
    } else {
      rx = rand_x(rng_); ry = rand_y(rng_);
    }

    int nearest_idx = nearestNeighbor(tree, rx, ry);
    Node new_node = steer(tree[nearest_idx], rx, ry);

    if (!collisionFree(tree[nearest_idx].x, tree[nearest_idx].y, new_node.x, new_node.y)) {
      continue;
    }

    new_node.parent_idx = nearest_idx;
    new_node.cost = tree[nearest_idx].cost +
      std::hypot(new_node.x - tree[nearest_idx].x, new_node.y - tree[nearest_idx].y);

    if (enable_rrt_star_) {
      // RRT*: choose parent with minimum cost
      auto near_indices = near(tree, new_node.x, new_node.y);
      for (int ni : near_indices) {
        double candidate_cost = tree[ni].cost +
          std::hypot(new_node.x - tree[ni].x, new_node.y - tree[ni].y);
        if (candidate_cost < new_node.cost &&
          collisionFree(tree[ni].x, tree[ni].y, new_node.x, new_node.y))
        {
          new_node.parent_idx = ni;
          new_node.cost = candidate_cost;
        }
      }

      int new_idx = static_cast<int>(tree.size());
      tree.push_back(new_node);

      // Rewire neighbors through new node
      for (int ni : near_indices) {
        double rewired_cost = new_node.cost +
          std::hypot(tree[ni].x - new_node.x, tree[ni].y - new_node.y);
        if (rewired_cost < tree[ni].cost &&
          collisionFree(new_node.x, new_node.y, tree[ni].x, tree[ni].y))
        {
          tree[ni].parent_idx = new_idx;
          tree[ni].cost = rewired_cost;
        }
      }
    } else {
      tree.push_back(new_node);
    }

    // Check if goal reached
    int last_idx = static_cast<int>(tree.size()) - 1;
    double dist_to_goal = std::hypot(tree[last_idx].x - gx, tree[last_idx].y - gy);
    if (dist_to_goal <= goal_tolerance_) {
      // Add final goal node
      if (collisionFree(tree[last_idx].x, tree[last_idx].y, gx, gy)) {
        tree.push_back({gx, gy, last_idx,
          tree[last_idx].cost + dist_to_goal});
        goal_node_idx = static_cast<int>(tree.size()) - 1;
        RCLCPP_INFO(node->get_logger(), "RRT path found in %d iterations (%zu nodes)",
          iter + 1, tree.size());
        break;
      }
    }
  }

  publishTree(tree);

  if (goal_node_idx < 0) {
    RCLCPP_WARN(node->get_logger(), "RRT failed to find path after %d iterations", max_iterations_);
    return nav_msgs::msg::Path{};
  }

  return extractPath(tree, goal_node_idx, global_frame_);
}

int RRTPlanner::nearestNeighbor(const std::vector<Node> & tree, double x, double y) const
{
  int nearest = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (int i = 0; i < static_cast<int>(tree.size()); ++i) {
    double d = std::hypot(tree[i].x - x, tree[i].y - y);
    if (d < min_dist) { min_dist = d; nearest = i; }
  }
  return nearest;
}

Node RRTPlanner::steer(const Node & from, double tx, double ty) const
{
  double dx = tx - from.x, dy = ty - from.y;
  double dist = std::hypot(dx, dy);
  if (dist <= step_size_) {
    return {tx, ty, -1, 0.0};
  }
  double scale = step_size_ / dist;
  return {from.x + dx * scale, from.y + dy * scale, -1, 0.0};
}

bool RRTPlanner::collisionFree(double x1, double y1, double x2, double y2) const
{
  unsigned int mx1, my1, mx2, my2;
  // If start is out of bounds treat as free (robot near map edge)
  // If end is out of bounds treat as free (planning into unexplored territory)
  bool start_in_map = worldToMap(x1, y1, mx1, my1);
  bool end_in_map   = worldToMap(x2, y2, mx2, my2);
  if (!start_in_map || !end_in_map) return true;

  // Bresenham line traversal — only check cells inside map bounds
  int ix = static_cast<int>(mx1), iy = static_cast<int>(my1);
  int ex = static_cast<int>(mx2), ey = static_cast<int>(my2);
  int dx = std::abs(ex - ix), dy = -std::abs(ey - iy);
  int sx = ix < ex ? 1 : -1, sy = iy < ey ? 1 : -1;
  int err = dx + dy;

  int w = static_cast<int>(costmap_->getSizeInCellsX());
  int h = static_cast<int>(costmap_->getSizeInCellsY());

  while (true) {
    if (ix >= 0 && iy >= 0 && ix < w && iy < h) {
      unsigned char cost = costmap_->getCost(
        static_cast<unsigned int>(ix),
        static_cast<unsigned int>(iy));
      // Only block known lethal/inscribed obstacles; unknown (255) is treated as free
      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
          cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) return false;
    }
    if (ix == ex && iy == ey) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; ix += sx; }
    if (e2 <= dx) { err += dx; iy += sy; }
  }
  return true;
}

std::vector<int> RRTPlanner::near(const std::vector<Node> & tree, double x, double y) const
{
  std::vector<int> result;
  for (int i = 0; i < static_cast<int>(tree.size()); ++i) {
    if (std::hypot(tree[i].x - x, tree[i].y - y) <= rewire_radius_) {
      result.push_back(i);
    }
  }
  return result;
}

bool RRTPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  return costmap_->worldToMap(wx, wy, mx, my);
}

void RRTPlanner::publishTree(const std::vector<Node> & tree)
{
  if (!tree_pub_) return;

  visualization_msgs::msg::MarkerArray ma;
  visualization_msgs::msg::Marker lines;
  lines.header.frame_id = global_frame_;
  lines.header.stamp = rclcpp::Clock().now();
  lines.ns = "rrt_tree";
  lines.id = 0;
  lines.type = visualization_msgs::msg::Marker::LINE_LIST;
  lines.action = visualization_msgs::msg::Marker::ADD;
  lines.scale.x = 0.01;
  lines.color.r = 0.0f; lines.color.g = 0.8f; lines.color.b = 0.2f; lines.color.a = 0.6f;
  lines.pose.orientation.w = 1.0;

  for (const auto & node : tree) {
    if (node.parent_idx < 0) continue;
    geometry_msgs::msg::Point p1, p2;
    p1.x = tree[node.parent_idx].x; p1.y = tree[node.parent_idx].y; p1.z = 0.05;
    p2.x = node.x; p2.y = node.y; p2.z = 0.05;
    lines.points.push_back(p1);
    lines.points.push_back(p2);
  }

  ma.markers.push_back(lines);
  tree_pub_->publish(ma);
}

nav_msgs::msg::Path RRTPlanner::extractPath(
  const std::vector<Node> & tree,
  int goal_idx,
  const std::string & frame_id)
{
  std::vector<int> indices;
  int idx = goal_idx;
  while (idx >= 0) {
    indices.push_back(idx);
    idx = tree[idx].parent_idx;
  }
  std::reverse(indices.begin(), indices.end());

  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = rclcpp::Clock().now();

  for (int i : indices) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = tree[i].x;
    ps.pose.position.y = tree[i].y;
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(ps);
  }
  return path;
}

}  // namespace rrt_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav2_core::GlobalPlanner)
