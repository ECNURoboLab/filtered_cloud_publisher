//
// Created by sun on 17-9-14.
//

#include <ros/ros.h>
#include <filtered_cloud_publisher/GpdAdapterActionServer.hpp>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "filtered_cloud_publisher");
    gpd_adapter_action_server::GpdAdapterActionServer gpd_adapter_action("gpd_adapter_action");
    ros::spin();
    return 0;
}