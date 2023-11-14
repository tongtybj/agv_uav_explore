#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/OccupancyGrid.h>

namespace navfn
{
  class ExistingPathGlobalPlanner : public nav_core::BaseGlobalPlanner
  {
  public:
    ExistingPathGlobalPlanner();
    ExistingPathGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    ExistingPathGlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

    bool makePlan(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    bool makePlan(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);

    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a);

    void clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my);

    ~ExistingPathGlobalPlanner(){}

    bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

  protected:

    ros::Publisher plan_pub_;
    ros::Subscriber existing_path_map_sub_;
    bool initialized_;
    nav_msgs::OccupancyGrid existing_path_map_;
    std::vector<std::vector<int>> existing_path_grid_;

    costmap_2d::Costmap2D* costmap_; // TODO: no need?

  private:

    // void mapToWorld(double mx, double my, double& wx, double& wy);
    std::vector<geometry_msgs::PoseStamped> linearInterpolation(geometry_msgs::Point start_p, geometry_msgs::Point goal_p);
    bool findClosestPoint(const nav_msgs::OccupancyGrid& map, const double& p_x, const double& p_y, const double& max_radius, int& m_x, int& m_y);

    void worldTomap(const nav_msgs::OccupancyGrid& map, const double &wx, const double &wy, int &mx, int &my);
    void mapToWorld(const nav_msgs::OccupancyGrid& map, const int &mx, const int &my, double& wx, double& wy);

    void existPathMapCallback(const nav_msgs::OccupancyGridConstPtr &msg);

    double default_tolerance_;
    double path_resolution_;
    double max_radius_;

    boost::mutex mutex_;
    ros::ServiceServer make_plan_srv_;
    std::string global_frame_;
  };
};

