#include <ros/ros.h>
#include "filtered_cloud_publisher/FilteredCloudPublisher.hpp"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "filtered_cloud_publisher");
  ros::NodeHandle nodeHandle("~");

  filtered_cloud_publisher::FilteredCloudPublisher filteredCloudPublisher(nodeHandle);

  ros::spin();
  return 0;
}
