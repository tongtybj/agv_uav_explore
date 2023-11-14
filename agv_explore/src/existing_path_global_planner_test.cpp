#include <agv_explore/existing_path_global_planner.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "existing_path_global_planner test");

  costmap_2d::Costmap2D costmap;

  navfn::ExistingPathGlobalPlanner planner("global_planner", &costmap, "map");

  ros::spin();

  return 0;
}

