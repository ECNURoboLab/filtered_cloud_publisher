#pragma once

#include "filtered_cloud_publisher/Algorithm.hpp"

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
// PCL ROS
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
// PCL
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
// GPD pointcloud types
#include <gpd/CloudIndexed.h>
#include <gpd/GraspConfigList.h>

namespace filtered_cloud_publisher {


    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointXYZ Point;

/*!
 * Main class for the node to handle the ROS interfacing.
 */
    class FilteredCloudPublisher {
    public:
        /*!
         * Constructor.
         * @param nodeHandle the ROS node handle.
         */
        FilteredCloudPublisher(ros::NodeHandle &nodeHandle);

        /*!
         * Destructor.
         */
        virtual ~FilteredCloudPublisher();

    private:
        /*!
         * Reads and verifies the ROS parameters.
         * @return true if successful.
         */
        bool readParameters();

        /*!
         * ROS topic callback method.
         * @param message the received message.RosPackageTemplate
         */
        void pointCloudCb(const PointCloud::ConstPtr& cloud_in);

        /*!
         * ROS service server callback.
         * @param request the request of the service.
         * @param response the provided response.
         * @return true if successful, false otherwise.
         */
        bool serviceCallback(std_srvs::Trigger::Request &request,
                             std_srvs::Trigger::Response &response);

        bool createGPDPointCloud(const PointCloud::Ptr cloud_in, gpd::CloudIndexed& gpd_cloud_out, pcl::PointIndices& indices);

        //! ROS node handle.

        ros::NodeHandle &nh_;

        ros::NodeHandle p_nh_;

        //! ROS topic subscriber.
        ros::Subscriber subscriber_;

        //! ROS transform listener

        boost::shared_ptr<tf::TransformListener> tf_listener_;

        //! GPD point cloud publisher
        ros::Publisher gpd_indexed_cloud_pub_;

        //! filtered cloud publisher
        ros::Publisher filtered_cloud_pub_;

        //! object cloud publisher
        ros::Publisher object_cloud_pub_;

        //! ROS topic name to subscribe to.
        std::string subscriber_topic_;

        //! ROS service server.
        ros::ServiceServer serviceServer_;

        //! Algorithm computation object.
        Algorithm algorithm_;

        bool sended;
    };

} /* namespace */
