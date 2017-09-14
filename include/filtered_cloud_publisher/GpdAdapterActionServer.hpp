//
// Created by sun on 17-9-14.
//

#ifndef FILTERED_CLOUD_PUBLISHER_GDP_ADAPTER_ACTION_SERVER_H
#define FILTERED_CLOUD_PUBLISHER_GDP_ADAPTER_ACTION_SERVER_H
//STL
#include <string>
// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>
//GPD
#include <gpd/GraspConfigList.h>
//Action type
#include <filtered_cloud_publisher/GpdAdapterAction.h>

namespace gpd_adapter_action_server{

    class GpdAdapterActionServer{
    public:

        GpdAdapterActionServer(const std::string& name);
        virtual ~GpdAdapterActionServer();
        void goalCB();
        void preemptCB();
        void analysisCB(const gpd::GraspConfigList msg);

    private:
        //! public node handle
        ros::NodeHandle nh_;
        //! server to trigger one cloud service
        ros::ServiceClient service_client_;
        //! action
        actionlib::SimpleActionServer<filtered_cloud_publisher::GpdAdapterAction> as_;
        //! subscriber for grasp config list
        ros::Subscriber sub_;
        //! publish marker
        ros::Publisher marker_pub_;

        boost::shared_ptr<tf::TransformListener> tf_listener_;

        std::string action_name_;
    };

}// end of namespace
#endif //FILTERED_CLOUD_PUBLISHER_GDP_ADAPTER_ACTION_H
