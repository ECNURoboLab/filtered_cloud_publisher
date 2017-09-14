//
// Created by sun on 17-9-14.
//

#include <ros/ros.h>
#include <filtered_cloud_publisher/GpdAdapterActionServer.hpp>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "gpd_adapter_action_server");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    gpd_adapter_action_server::GpdAdapterActionServer gpd_adapter_action("gpd_adapter_action");
    ros::waitForShutdown();
    return 0;
}