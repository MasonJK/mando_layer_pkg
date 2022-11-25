#include "mando_layer.h"
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>


PLUGINLIB_EXPORT_CLASS(mando_namespace::MandoLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace mando_namespace{
MandoLayer::MandoLayer() {}

void MandoLayer::onInitialize(){

  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &MandoLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  sub = g_nh.subscribe("/filtered_points", 1, &MandoLayer::callback, this);
}

void MandoLayer::callback(const geometry_msgs::PoseArray::ConstPtr& msg){
  // std::cout<<"Callback - mando"<<std::endl;
  filtered_pose = *msg;
}

void MandoLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
  enabled_ = config.enabled;
}

void MandoLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y){
  if (!enabled_)
    return;
  
  for(auto i: filtered_pose.poses){
    m_pose marked;

    marked.x = i.position.x;
    marked.y = i.position.y;
    m_list.push_back(marked);

    *min_x = std::min(*min_x, marked.x);
    *min_y = std::min(*min_y, marked.y);
    *max_x = std::max(*max_x, marked.x);
    *max_y = std::max(*max_y, marked.y);
  }
}

void MandoLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j){
  if (!enabled_)
    return;

  unsigned int mx;
  unsigned int my;
  while(!m_list.empty()){
    m_pose marked = m_list.back();
    if(master_grid.worldToMap(marked.x, marked.y, mx, my))
      master_grid.setCost(mx,my,LETHAL_OBSTACLE);
    m_list.pop_back();

  }
}

double MandoLayer::calculateYaw(geometry_msgs::Quaternion geo_quat){
  tf::Quaternion q(
        geo_quat.x,
        geo_quat.y,
        geo_quat.z,
        geo_quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

} // end namespace