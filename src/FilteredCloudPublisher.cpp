#include "filtered_cloud_publisher/FilteredCloudPublisher.hpp"

// STD
#include <string>

namespace filtered_cloud_publisher {
    FilteredCloudPublisher::FilteredCloudPublisher(ros::NodeHandle &nodeHandle)
            : nh_(nodeHandle)
            , p_nh_()
    {
        if (!readParameters()) {
            ROS_ERROR("Could not read parameters.");
            ROS_ERROR("Use default value !!!!");
            subscriber_topic_ = "/camera/depth/points";
//            ros::requestShutdown();
        }
        subscriber_ = p_nh_.subscribe(subscriber_topic_, 1,
                                    &FilteredCloudPublisher::pointCloudCb, this);
        gpd_indexed_cloud_pub_ = p_nh_.advertise<gpd::CloudIndexed>("cloud_indexed",1);
        filtered_cloud_pub_    = p_nh_.advertise<PointCloud>("over_table_cloud",1);
        object_cloud_pub_      = p_nh_.advertise<PointCloud>("object_cloud",1);
        serviceServer_ = p_nh_.advertiseService("send_one_cloud",
                                              &FilteredCloudPublisher::serviceCallback, this);
        tf_listener_.reset(new tf::TransformListener);
        sended = true;
        ROS_INFO("Successfully launched node.");
    }


    FilteredCloudPublisher::~FilteredCloudPublisher() {
    }

    bool FilteredCloudPublisher::readParameters() {
        return nh_.getParam("subscriber_topic", subscriber_topic_);
    }

    void FilteredCloudPublisher::pointCloudCb(const PointCloud::ConstPtr &cloud_in) {
        PointCloud::Ptr cloud_tf (new PointCloud);
        PointCloud::Ptr cloud_filtered (new PointCloud);
        PointCloud::Ptr object_cloud(new PointCloud);

        sensor_msgs::PointCloud2 gpd_cloud_;
        std_msgs::Header header = pcl_conversions::fromPCL(cloud_in->header);

        tf::StampedTransform transform;
        try{
            tf_listener_->waitForTransform("/table_top", header.frame_id, header.stamp, ros::Duration(5.0));
            tf_listener_->lookupTransform ("/table_top", header.frame_id, header.stamp, transform);
        }
        catch(std::runtime_error &e){
            ROS_ERROR("Could not find /table_top to %s transform", header.frame_id.c_str());
            return;
        }

        //! 将点云转到table_top坐标系下面
        pcl_ros::transformPointCloud (*cloud_in, *cloud_tf, transform);
        cloud_tf->header.frame_id = "/table_top";
        //! 点云切割 设置桌子尺寸 z 数值设低一点，将桌面和桌面上点云全部获取
        pcl::CropBox<Point> box(true);
        box.setInputCloud(cloud_tf);
        box.setMin(Eigen::Vector4f(-0.10f,-0.06f,0.00f,1.0f));
        box.setMax(Eigen::Vector4f(0.35,0.45,0.5,1.0));
        box.filter (*cloud_filtered);
        //! 将z轴数值设到4cm，过滤掉桌面。
        box.setMin(Eigen::Vector4f(-0.10f,-0.06f,0.04f,1.0f));
        box.setInputCloud(cloud_filtered);
        box.setNegative(true);
        box.filter(*object_cloud);
        pcl::PointIndices indices;
        //! 获取object的indices
        box.getRemovedIndices(indices);
        box.setNegative(false);
        box.filter(*object_cloud);

        gpd::CloudIndexed gpd_cloud;
        createGPDPointCloud(cloud_filtered,gpd_cloud,indices);
        if(!sended)
        {
            gpd_indexed_cloud_pub_.publish(gpd_cloud);
            sended = true;
        }
        filtered_cloud_pub_.publish(cloud_filtered);
        object_cloud_pub_.publish(object_cloud);
    }

    bool FilteredCloudPublisher::createGPDPointCloud(const PointCloud::Ptr cloud_in,
                                                     gpd::CloudIndexed &gpd_cloud_out,pcl::PointIndices& indices)
    {
        geometry_msgs::Point camera_viewpoint;
        camera_viewpoint.x = 0;
        camera_viewpoint.y = 0;
        camera_viewpoint.z = 0;
        pcl::toROSMsg(*cloud_in,gpd_cloud_out.cloud_sources.cloud);
        gpd_cloud_out.cloud_sources.view_points.push_back(camera_viewpoint);
        std_msgs::Int64 camera_source, index;
        camera_source.data = 0;
        for(int i=0; i < gpd_cloud_out.cloud_sources.cloud.width ; i++)
        {
            gpd_cloud_out.cloud_sources.camera_source.push_back(camera_source);
        }

        for(int i=0; i < indices.indices.size(); i++)
        {
            index.data = indices.indices[i];
            gpd_cloud_out.indices.push_back(index);
        }


    }

    bool FilteredCloudPublisher::serviceCallback(std_srvs::Trigger::Request &request,
                                                 std_srvs::Trigger::Response &response) {
        response.success = true;
        sended = false;
        response.message = "Successfully sending one image !!! ";

        return true;
    }

} /* namespace */
