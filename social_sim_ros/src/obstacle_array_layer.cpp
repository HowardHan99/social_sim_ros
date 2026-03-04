#include <obstacle_array_layer.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>
#include <unordered_set>

PLUGINLIB_EXPORT_CLASS(social_sim_ros::ObstacleArrayLayer, costmap_2d::Layer)

namespace social_sim_ros
{

ObstacleArrayLayer::ObstacleArrayLayer()
  : has_data_(false), has_prev_bounds_(false)
{
}

ObstacleArrayLayer::~ObstacleArrayLayer() {}

void ObstacleArrayLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  std::string topic;
  nh.param("topic", topic, std::string("/social_sim/obstacles"));
  nh.param("enabled", enabled_, true);
  current_ = true;

  ros::NodeHandle g_nh;
  obstacle_sub_ = g_nh.subscribe(topic, 10, &ObstacleArrayLayer::obstacleCallback, this);

  ROS_INFO("ObstacleArrayLayer: subscribed to %s (enabled: %s)", topic.c_str(), enabled_ ? "true" : "false");
}

void ObstacleArrayLayer::obstacleCallback(const social_sim_ros::ObstacleArray::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(data_mutex_);

  current_footprint_.clear();
  bounds_min_x_ =  std::numeric_limits<double>::max();
  bounds_min_y_ =  std::numeric_limits<double>::max();
  bounds_max_x_ = -std::numeric_limits<double>::max();
  bounds_max_y_ = -std::numeric_limits<double>::max();

  std::unordered_set<int> seen_ids;
  const double resolution = 0.05;

  for (const auto& obstacle : msg->obstacles) {
    if (seen_ids.count(obstacle.id)) continue;
    seen_ids.insert(obstacle.id);

    double cx = obstacle.pose.position.x;
    double cy = obstacle.pose.position.y;

    tf2::Quaternion q(obstacle.pose.orientation.x, obstacle.pose.orientation.y,
                      obstacle.pose.orientation.z, obstacle.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    std::string type = obstacle.type;
    std::transform(type.begin(), type.end(), type.begin(), ::tolower);

    if (type.find("cylinder") != std::string::npos ||
        type.find("circle")   != std::string::npos) {
      addCylinderFootprint(cx, cy, obstacle.scale.x / 2.0, resolution);
    } else {
      addBoxFootprint(cx, cy, yaw,
                      obstacle.scale.x / 2.0, obstacle.scale.y / 2.0, resolution);
    }
  }

  has_data_ = true;
}

void ObstacleArrayLayer::addBoxFootprint(double cx, double cy, double yaw,
                                          double hx, double hy, double resolution)
{
  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);

  for (double dx = -hx; dx <= hx; dx += resolution) {
    for (double dy = -hy; dy <= hy; dy += resolution) {
      double rx = cx + dx * cos_yaw - dy * sin_yaw;
      double ry = cy + dx * sin_yaw + dy * cos_yaw;
      current_footprint_.push_back({rx, ry});
      bounds_min_x_ = std::min(bounds_min_x_, rx);
      bounds_min_y_ = std::min(bounds_min_y_, ry);
      bounds_max_x_ = std::max(bounds_max_x_, rx);
      bounds_max_y_ = std::max(bounds_max_y_, ry);
    }
  }
}

void ObstacleArrayLayer::addCylinderFootprint(double cx, double cy,
                                               double radius, double resolution)
{
  double r2 = radius * radius;
  for (double dx = -radius; dx <= radius; dx += resolution) {
    for (double dy = -radius; dy <= radius; dy += resolution) {
      if (dx * dx + dy * dy <= r2) {
        double rx = cx + dx;
        double ry = cy + dy;
        current_footprint_.push_back({rx, ry});
        bounds_min_x_ = std::min(bounds_min_x_, rx);
        bounds_min_y_ = std::min(bounds_min_y_, ry);
        bounds_max_x_ = std::max(bounds_max_x_, rx);
        bounds_max_y_ = std::max(bounds_max_y_, ry);
      }
    }
  }
}

void ObstacleArrayLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
                                       double* min_x, double* min_y,
                                       double* max_x, double* max_y)
{
  boost::mutex::scoped_lock lock(data_mutex_);
  if (!has_data_) return;

  // Include previous bounds so the master grid reset clears old positions,
  // then include current bounds so new positions get written.
  if (has_prev_bounds_) {
    *min_x = std::min({*min_x, bounds_min_x_, prev_min_x_});
    *min_y = std::min({*min_y, bounds_min_y_, prev_min_y_});
    *max_x = std::max({*max_x, bounds_max_x_, prev_max_x_});
    *max_y = std::max({*max_y, bounds_max_y_, prev_max_y_});
  } else {
    *min_x = std::min(*min_x, bounds_min_x_);
    *min_y = std::min(*min_y, bounds_min_y_);
    *max_x = std::max(*max_x, bounds_max_x_);
    *max_y = std::max(*max_y, bounds_max_y_);
  }
}

void ObstacleArrayLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                      int /*min_i*/, int /*min_j*/,
                                      int /*max_i*/, int /*max_j*/)
{
  boost::mutex::scoped_lock lock(data_mutex_);
  if (!has_data_) return;

  // The master grid was already reset in the update bounds region by
  // LayeredCostmap::updateMap before layers are called.  The static layer
  // has already restored wall/free-space data.  We simply stamp current
  // obstacle positions as LETHAL -- no manual clearing needed.
  unsigned int mx, my;
  for (const auto& pt : current_footprint_) {
    if (master_grid.worldToMap(pt.x, pt.y, mx, my)) {
      master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
    }
  }

  prev_min_x_ = bounds_min_x_;
  prev_min_y_ = bounds_min_y_;
  prev_max_x_ = bounds_max_x_;
  prev_max_y_ = bounds_max_y_;
  has_prev_bounds_ = true;
}

}  // namespace social_sim_ros
