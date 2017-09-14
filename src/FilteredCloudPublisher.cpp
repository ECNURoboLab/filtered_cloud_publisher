#include "filtered_cloud_publisher/FilteredCloudPublisher.hpp"

// STD
#include <string>

namespace filtered_cloud_publisher {
FilteredCloudPublisher::FilteredCloudPublisher(ros::NodeHandle &nodeHandle)
        :nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,
                                      &FilteredCloudPublisher::topicCallback, this);
  serviceServer_ = nodeHandle_.advertiseService("get_average",
                                                &FilteredCloudPublisher::serviceCallback, this);
  ROS_INFO("Successfully launched node.");
}



    FilteredCloudPublisher::~RosPackageTemplate()
{
}

bool FilteredCloudPublisher::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  return true;
}

void FilteredCloudPublisher::topicCallback(const sensor_msgs::Temperature& message)
{
  algorithm_.addData(message.temperature);
}

bool FilteredCloudPublisher::serviceCallback(std_srvs::Trigger::Request& request,
                                         std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "The average is " + std::to_string(algorithm_.getAverage());
  return true;
}

} /* namespace */
