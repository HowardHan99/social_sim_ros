#ifndef OBSTACLE_ARRAY_LAYER_H_
#define OBSTACLE_ARRAY_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <social_sim_ros/ObstacleArray.h>
#include <boost/thread/mutex.hpp>

namespace social_sim_ros
{

class ObstacleArrayLayer : public costmap_2d::Layer
{
public:
  ObstacleArrayLayer();
  virtual ~ObstacleArrayLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                           int max_i, int max_j);

private:
  void obstacleCallback(const social_sim_ros::ObstacleArray::ConstPtr& msg);

  void addBoxFootprint(double cx, double cy, double yaw,
                       double hx, double hy, double resolution);
  void addCylinderFootprint(double cx, double cy,
                            double radius, double resolution);

  ros::Subscriber obstacle_sub_;
  boost::mutex data_mutex_;

  struct FootprintPoint { double x, y; };
  std::vector<FootprintPoint> current_footprint_;

  double bounds_min_x_, bounds_min_y_, bounds_max_x_, bounds_max_y_;
  double prev_min_x_, prev_min_y_, prev_max_x_, prev_max_y_;
  bool has_data_;
  bool has_prev_bounds_;
};

}  // namespace social_sim_ros

#endif
