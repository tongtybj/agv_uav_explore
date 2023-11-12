#pragma once

#include <ros/ros.h>

class ExistingPathPlanner
{
 public:
  ExistingPathPlanner(ros::NodeHandle nh, ros::NodeHandle nh_private);
  ~ExistingPathPlanner();

  void mainFunc(const ros::TimerEvent & e);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Timer main_timer_;

  boost::shared_ptr<aerial_robot_model::RobotModelRos> robot_model_ros_;
  boost::shared_ptr<aerial_robot_estimation::StateEstimator>  estimator_;

  pluginlib::ClassLoader<aerial_robot_navigation::BaseNavigator> navigator_loader_;
  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator_;

  pluginlib::ClassLoader<aerial_robot_control::ControlBase> controller_loader_;
  boost::shared_ptr<aerial_robot_control::ControlBase> controller_;

  ros::AsyncSpinner callback_spinner_; // Use 4 threads
  ros::AsyncSpinner main_loop_spinner_; // Use 1 threads
  ros::CallbackQueue main_loop_queue_;
};
