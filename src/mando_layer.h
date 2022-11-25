#ifndef MANDO_LAYER_H_
#define MANDO_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <vector>
#include <tf/transform_listener.h>

namespace mando_namespace
{

struct m_pose
{
  double x;
  double y;
};


class MandoLayer : public costmap_2d::Layer
{
public:
  MandoLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  void callback(const geometry_msgs::PoseArray::ConstPtr& msg);
  double calculateYaw(geometry_msgs::Quaternion geo_quat);
  
private:
  ros::NodeHandle g_nh;
  ros::Subscriber sub;
  geometry_msgs::PoseArray filtered_pose;
  std::vector<m_pose> m_list;

  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif