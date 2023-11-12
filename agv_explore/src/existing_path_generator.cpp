#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

class ExistingPathGenerator
{
 public:
  ExistingPathGenerator(ros::NodeHandle nh, ros::NodeHandle nhp):
    nh_(nh), nhp_(nhp), tf_listener_(tf_buffer_)
  {

    nhp_.param("map_frame", map_frame_, std::string("map"));
    nhp_.param("body_frame", body_frame_, std::string("base_link"));

    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1);
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("path_map", 1);
    map_sub_ = nh_.subscribe("map", 1, &ExistingPathGenerator::mapCallback, this);

    double rate;
    nhp_.param("rate", rate, 20.0);
    main_timer_ = nhp_.createTimer(ros::Duration(1.0 / rate), &ExistingPathGenerator::mainFunc, this);
  }

  ~ExistingPathGenerator() {}



 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  nav_msgs::Path path_;
  ros::Publisher path_pub_;
  ros::Publisher map_pub_;
  ros::Subscriber map_sub_;

  nav_msgs::OccupancyGrid path_map_;

  ros::Timer main_timer_;

  std::string map_frame_, body_frame_;

  void mapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
  {
    path_map_.header = msg->header;
    path_map_.info = msg->info;
    path_map_.data.resize(msg->data.size(), 0);

    ROS_INFO("Receive the raw map info");

    map_sub_.shutdown();
  }

  void mainFunc(const ros::TimerEvent & e)
  {
    ros::Time now = ros::Time::now();
    double timeout = 1.0;
    if(!tf_buffer_.canTransform(map_frame_, body_frame_, now, ros::Duration(timeout)))
      {
        ROS_WARN("Could not lookup transform from %s to %s, in duration %f [sec]", map_frame_.c_str(), body_frame_.c_str(), timeout);
        return;
    }

    geometry_msgs::TransformStamped tf;
    try{
      tf = tf_buffer_.lookupTransform(map_frame_, body_frame_, now, ros::Duration(0.1));
    }
    catch(tf2::TransformException &ex){
      ROS_WARN("%s", ex.what());
      return;
    }

    // visualize the path
    path_.header.stamp = now;
    path_.header.frame_id = map_frame_;
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = map_frame_;
    pose.pose.orientation = tf.transform.rotation;
    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    path_.poses.push_back(pose);

    path_pub_.publish(path_);


    // fill the map
    double resolution = path_map_.info.resolution;
    // right-lower corner is (0,0)
    int w = (tf.transform.translation.x - path_map_.info.origin.position.x) / resolution;
    int h = (tf.transform.translation.y - path_map_.info.origin.position.y) / resolution;
    int index = w + path_map_.info.width * h;
    path_map_.data.at(index) = 100;
    map_pub_.publish(path_map_);
  }

};

int main (int argc, char **argv)
{
  ros::init (argc, argv, "existing_path_generator");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  ExistingPathGenerator generator(nh, nhp);

  ros::spin();

  return 0;
}




