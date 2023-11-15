#include <agv_explore/existing_path_global_planner.h>
#include <agv_explore/a_star.h>

namespace navfn {

  ExistingPathGlobalPlanner::ExistingPathGlobalPlanner()
    : costmap_(NULL), initialized_(false) {}

  ExistingPathGlobalPlanner::ExistingPathGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_(NULL), initialized_(false) {

      initialize(name, costmap_ros);
  }

  ExistingPathGlobalPlanner::ExistingPathGlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
    : costmap_(NULL), initialized_(false) {

      initialize(name, costmap, global_frame);
  }

  void ExistingPathGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame){
    if(!initialized_){
      costmap_ = costmap;
      global_frame_ = global_frame;

      existing_path_map_.data.resize(0);
      existing_path_grid_.resize(0);

      ros::NodeHandle private_nh("~/" + name);

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
      std::string topic_name;
      private_nh.param("existing_path_topic_name", topic_name, std::string("/path_map"));
      existing_path_map_sub_ = private_nh.subscribe(topic_name, 1, &ExistingPathGlobalPlanner::existPathMapCallback, this);

      private_nh.param("default_tolerance", default_tolerance_, 0.0);
      private_nh.param("path_resolution", path_resolution_, 0.025);
      private_nh.param("max_radius", max_radius_, 2.0);


      make_plan_srv_ =  private_nh.advertiseService("make_plan", &ExistingPathGlobalPlanner::makePlanService, this);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }

  void ExistingPathGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
  }

  std::vector<geometry_msgs::PoseStamped> ExistingPathGlobalPlanner::linearInterpolation(geometry_msgs::Point start_p, geometry_msgs::Point goal_p) {
    std::vector<geometry_msgs::PoseStamped> linear_plan;

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.pose.orientation.w = 1.0;

    double path_len = std::sqrt(std::pow(goal_p.x - start_p.x, 2) + std::pow(goal_p.y - start_p.y, 2));
    int path_num = path_len / path_resolution_;
    for(int i = 0; i <= path_num; i++)
      {
        pose.pose.position.x = (goal_p.x - start_p.x) * i / path_num + start_p.x;
        pose.pose.position.y = (goal_p.y - start_p.y) * i / path_num + start_p.y;
        linear_plan.push_back(pose);
      }

    return linear_plan;
  }


  bool ExistingPathGlobalPlanner::findClosestPoint(const nav_msgs::OccupancyGrid& map, const double& p_x, const double& p_y, const double& max_radius, int& m_x, int& m_y) {

    double resolution = map.info.resolution;
    int width = map.info.width;
    int height = map.info.height;

    // TODO: use the default navfn to find the best path for this short route

#if 0
    int prev_x = 0;
    int prev_y = 0;

    int c_x, c_y;
    worldTomap(map, p_x, p_y, c_x, c_y);

    int m_radius = 1;
    while (m_radius < max_radius / resolution)
      {
        for (double theta = 0; theta < 2 * M_PI; theta += 0.01)
          {
            m_x = cos(theta) * m_radius + c_x;
            m_y = sin(theta) * m_radius + c_y;

            if (m_x < 0 || m_x >= width || m_y < 0 || m_y >= height) continue;

            if (prev_x == m_x && prev_y == m_y) continue;

            if (map.data.at(m_x + m_y * width) > 0) return true;

            prev_x = m_x;
            prev_y = m_y;
          }
        m_radius ++;
      }
#else

    double radius = resolution;
    int prev_x = 0;
    int prev_y = 0;

    while (radius < max_radius)
      {
        for (double theta = 0; theta < 2 * M_PI; theta += 0.01)
          {
            double w_x = cos(theta) * radius + p_x;
            double w_y = sin(theta) * radius + p_y;

            worldTomap(map, w_x, w_y, m_x, m_y);

            if (m_x < 0 || m_x >= width || m_y < 0 || m_y >= height) continue;

            if (prev_x == m_x && prev_y == m_y) continue;

            if (map.data.at(m_x + m_y * width) > 0) return true;

            prev_x = m_x;
            prev_y = m_y;
          }

        radius += resolution;
      }
#endif
    m_x = 0;
    m_y = 0;

    return false;
  }

  bool ExistingPathGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                           const geometry_msgs::PoseStamped& goal,
                                           double tolerance,
                                           std::vector<geometry_msgs::PoseStamped>& plan) {

    boost::mutex::scoped_lock lock(mutex_);

    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    if(existing_path_map_.data.size() == 0) {
      ROS_ERROR("This planner has not receive the existing path map");
      return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if(goal.header.frame_id != global_frame_){
      ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                global_frame_.c_str(), goal.header.frame_id.c_str());
      return false;
    }

    if(start.header.frame_id != global_frame_){
      ROS_ERROR("The start pose passed to this planner must be in the %s frame. It is instead in the %s frame.",
                global_frame_.c_str(), start.header.frame_id.c_str());
      return false;
    }


    ros::Time plan_time = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = global_frame_;
    pose.pose.orientation.w = 1.0;

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;


    /* 1. generate linear interpolaration path between start point and closest point in map */
    int start_closest_m_x, start_closest_m_y;
    bool find
      = findClosestPoint(existing_path_map_, start.pose.position.x, start.pose.position.y,
                         max_radius_, start_closest_m_x, start_closest_m_y);

    if (!find) {
      ROS_ERROR("can not find a close point for the start point");
      return false;
    }

    geometry_msgs::Point start_closest_point;
    mapToWorld(existing_path_map_, start_closest_m_x, start_closest_m_y,
               start_closest_point.x, start_closest_point.y);
    /* linear interpolarion: [start_point, start_closest_point] */
    plan = linearInterpolation(start.pose.position, start_closest_point);


    /* 2. generate linear interpolaration path between goal point and closest point in map */
    int goal_closest_m_x, goal_closest_m_y;
    find = findClosestPoint(existing_path_map_, goal.pose.position.x, goal.pose.position.y,
                             max_radius_, goal_closest_m_x, goal_closest_m_y);

    if (!find) {
      ROS_ERROR("can not find a close point for the goal point");
      return false;
    }

    /* 3. generate intermediate trajectory based on the the existing path map */
    std::vector<a_star::Point> a_star_path;
    a_star::Point start_p(start_closest_m_x, start_closest_m_y);
    a_star::Point goal_p(goal_closest_m_x, goal_closest_m_y);

    find = a_star::astarSearch(existing_path_grid_, start_p, goal_p, a_star_path);
    if (!find) {
      ROS_ERROR("can not find path by A*");

      /* test */
      geometry_msgs::Point goal_closest_point;
      mapToWorld(existing_path_map_, goal_closest_m_x, goal_closest_m_y,
                 goal_closest_point.x, goal_closest_point.y);
      /* linear interpolarion: [goal_point, goal_closest_point] */
      auto last_path = linearInterpolation(goal_closest_point, goal.pose.position);
      plan.insert(plan.end(), last_path.begin() + 1, last_path.end());

      plan.front() = start;
      plan.back() = goal;

      publishPlan(plan, 0.0, 1.0, 0.0, 0.0);

      return false;
    }

    for (auto m_p: a_star_path) {
      geometry_msgs::PoseStamped w_pose;
      w_pose.header.frame_id = global_frame_;
      w_pose.pose.orientation.w = 1.0;
      mapToWorld(existing_path_map_, m_p.x, m_p.y, w_pose.pose.position.x, w_pose.pose.position.y);
      plan.push_back(w_pose);
    }

    geometry_msgs::Point goal_closest_point;
    mapToWorld(existing_path_map_, goal_closest_m_x, goal_closest_m_y,
               goal_closest_point.x, goal_closest_point.y);
    /* linear interpolarion: [goal_point, goal_closest_point] */
    auto last_path = linearInterpolation(goal_closest_point, goal.pose.position);
    plan.insert(plan.end(), last_path.begin() + 1, last_path.end());


    /* to set the orientation */
    plan.front() = start;
    plan.back() = goal;

    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);

    return !plan.empty();
  }

  bool ExistingPathGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                           const geometry_msgs::PoseStamped& goal,
                                           std::vector<geometry_msgs::PoseStamped>& plan){
    return makePlan(start, goal, default_tolerance_, plan);
  }

  void ExistingPathGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if(!path.empty())
    {
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
  }

  bool ExistingPathGlobalPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {

    bool result = makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = global_frame_;

    return result;
  }

  void ExistingPathGlobalPlanner::existPathMapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
  {
    existing_path_map_ = *msg;


    int width = existing_path_map_.info.width;
    int height = existing_path_map_.info.height;

    existing_path_grid_ = std::vector<std::vector<int>>(width, std::vector<int>(height, 1));

    for (int i = 0; i < existing_path_map_.data.size(); i++) {
      int w = i % width;
      int h = i / width;

      if (existing_path_map_.data.at(i)) {
        existing_path_grid_[w][h] = 0;
      }
    }

    existing_path_map_sub_.shutdown();
  }

  void ExistingPathGlobalPlanner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my){
    if(!initialized_) {
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
  }

  void ExistingPathGlobalPlanner::worldTomap(const nav_msgs::OccupancyGrid& map, const double &wx, const double &wy, int &mx, int &my) {
    double resolution = map.info.resolution;
    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;

    mx = (wx - origin_x) / resolution;
    my = (wy - origin_y) / resolution;
  }

  void ExistingPathGlobalPlanner::mapToWorld(const nav_msgs::OccupancyGrid& map, const int &mx, const int &my, double& wx, double& wy) {
    double resolution = map.info.resolution;
    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;
    wx = origin_x + mx * resolution;
    wy = origin_y + my * resolution;
  }


};

#include <pluginlib/class_list_macros.h>
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(navfn::ExistingPathGlobalPlanner, nav_core::BaseGlobalPlanner)
