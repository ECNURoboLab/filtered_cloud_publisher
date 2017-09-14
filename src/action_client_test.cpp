//
// Created by sun on 17-9-14.
//

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <filtered_cloud_publisher/GpdAdapterAction.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_action_client");

//    ros::NodeHandle pnh("~");

    std::string action_server_name;
//    pnh.param<std::string>("action_server_name", action_server_name, "gpd_adapter_action");
    action_server_name = "gpd_adapter_action";
    actionlib::SimpleActionClient<filtered_cloud_publisher::GpdAdapterAction> ac("gpd_adapter_action", true);

    ROS_INFO("Waiting for the action server start ...");
    ac.waitForServer(ros::Duration()); // wait for infinite time

    ROS_INFO("Action server got ..., sending goal ");
    // prepare the goal msgs

    filtered_cloud_publisher::GpdAdapterGoal goal;
    filtered_cloud_publisher::GpdAdapterResultConstPtr res;
    ac.sendGoal(goal);

    ac.waitForResult(ros::Duration(0.1));

//    bool finish_before_timeout = ac.waitForResult(ros::Duration(30.0));
//
//    sleep(10);
//    goal.command.position = 0.8;
//    goal.command.max_effort = 100;
//    ac.sendGoal(goal);

//    if(finish_before_timeout)
//    {
//        actionlib::SimpleClientGoalState state = ac.getState();
//        ROS_INFO("Action finished: %s", state.toString().c_str());
//    } else
//    {
//        ROS_INFO("fucking we failed ....");
//
//    }
    while (!ros::isShuttingDown())
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO_STREAM("State is " << state.toString() );
        if(state == state.SUCCEEDED)
        {
            res = ac.getResult();
            ROS_INFO_STREAM("Result is " << res->pose );
        }



        sleep(1);
    }
    return 0;

}