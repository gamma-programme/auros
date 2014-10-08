#include <ros/ros.h>
#include <mavros/Waypoint.h>
#include <mavros/WaypointList.h>

void wp_cb(const mavros::WaypointList& wp_list)
{
  std::vector<mavros::Waypoint> waypoints = wp_list.waypoints;
  
  for(int i = 0; i < waypoints.size(); i++) 
  {
    ROS_INFO("Waypoint loaded @ %f, %f", waypoints[i].x_lat, waypoints[i].y_long);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_lister");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/auros/fcu/mission/waypoints", 1, wp_cb);
  ROS_INFO("Setup complete");
  ros::spin();
  return 0;
}
