//
// Created by sun on 17-9-14.
//
#include <filtered_cloud_publisher/GpdAdapterActionServer.hpp>

namespace {
geometry_msgs::Pose creatPickingEEFPose(std::vector<gpd::GraspConfig> grasp_config_list_, tf::StampedTransform tf_base_table)
{

    tf::Transform tf_base_object;
    geometry_msgs::Pose grasp_pose;
    gpd::GraspConfig grasp_config;
    if(grasp_config_list_.size()==0)
    {
        ROS_ERROR("No grasp valid !!!!");
        exit(1);
    }
    grasp_config = grasp_config_list_[0]; // the first one is the best



    tf::Matrix3x3 rot_table_grasp(-grasp_config.axis.x,grasp_config.binormal.x,grasp_config.approach.x,
                                 -grasp_config.axis.y,grasp_config.binormal.y,grasp_config.approach.y,
                                 -grasp_config.axis.z,grasp_config.binormal.z,grasp_config.approach.z);
    tf::Transform tf_z_90;
    tf_z_90.setIdentity();
    tf_z_90.setRotation(tf::createQuaternionFromRPY(0,0,3.141592/2));
    ROS_INFO_STREAM("Z 90:" << tf_z_90.getRotation().x() << "|"<< tf_z_90.getRotation().y()<< "|"<<tf_z_90.getRotation().z()<< "|" <<tf_z_90.getRotation().w() ) ;

    // bottom指的是爪子的手掌点
    tf::Vector3 tr_table_grasp(grasp_config.bottom.x,grasp_config.bottom.y,grasp_config.bottom.z);
    tf::Transform tf_table_grasp(rot_table_grasp,tr_table_grasp);


    tf_base_object = tf_base_table*tf_table_grasp*tf_z_90;
    tf::poseTFToMsg(tf_base_object,grasp_pose);
    return grasp_pose;

}

    geometry_msgs::Pose convertGraspConfig2PoseMsgs(gpd::GraspConfig grasp_config)
    {
        geometry_msgs::Pose grasp_pose;
        tf::Matrix3x3 rot_base_grasp(-grasp_config.axis.x,grasp_config.binormal.x,grasp_config.approach.x,
                                     -grasp_config.axis.y,grasp_config.binormal.y,grasp_config.approach.y,
                                     -grasp_config.axis.z,grasp_config.binormal.z,grasp_config.approach.z);
        // bottom指的是爪子的手掌点
        tf::Transform tf_z_90;
        tf_z_90.setIdentity();
        tf_z_90.setRotation(tf::createQuaternionFromRPY(0,0,3.141592/2));
        tf::Vector3 tr_base_grasp(grasp_config.bottom.x,grasp_config.bottom.y,grasp_config.bottom.z);
        tf::Transform tf_base_grasp(rot_base_grasp,tr_base_grasp);

        tf::poseTFToMsg(tf_base_grasp*tf_z_90,grasp_pose);
        return grasp_pose;
    }

    visualization_msgs::MarkerArray creatMarker(geometry_msgs::Pose pose)
    {
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker marker;

        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "/table_top";
        marker.lifetime = ros::Duration(10);
        marker.ns = "grasp";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        marker.color.r = 1.0;
        marker.color.a = 1.0;

        marker.scale.x = 0.08;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;

        marker.pose = pose;

        markers.markers.push_back(marker);
        tf::Quaternion q,q_res,q_res_2;
        geometry_msgs::Quaternion q_msgs,q_msgs2;
        tf::quaternionMsgToTF(marker.pose.orientation, q);
        tf::Matrix3x3 rot(q);
        tf::Matrix3x3 res_rot,res_rot_2;
        tf::Matrix3x3 change_axis(0,-1,0,
                                  1,0,0,
                                  0,0,1);
        res_rot = rot*change_axis;
        res_rot.getRotation(q_res);
        tf::quaternionTFToMsg(q_res,q_msgs);

        marker.id = 1;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.pose.orientation = q_msgs;
        markers.markers.push_back(marker);

        tf::Matrix3x3 change_axis_z(0,0,-1,
                                    0,1,0,
                                    1,0,0);
        res_rot_2 = rot*change_axis_z;

        res_rot_2.getRotation(q_res_2);
        tf::quaternionTFToMsg(q_res_2,q_msgs2);

        marker.id = 2;
        marker.color.b = 1.0;
        marker.color.g = 0.0;
        marker.pose.orientation = q_msgs2;
        markers.markers.push_back(marker);


        return markers;
    }


};

namespace gpd_adapter_action_server{

    GpdAdapterActionServer::GpdAdapterActionServer(const std::string &name)
            :nh_()
            , as_(nh_,name, false)
            , action_name_(name)
    {
        as_.registerGoalCallback(boost::bind(&GpdAdapterActionServer::goalCB,this));
        as_.registerPreemptCallback(boost::bind(&GpdAdapterActionServer::preemptCB,this));
        // 发送gpd点云 触发服务
        service_client_ = nh_.serviceClient<std_srvs::Trigger>("send_one_cloud");
        if(!service_client_.waitForExistence(ros::Duration(5)))
        {
            ROS_INFO("send one cloud does not exist !!!!");
            exit(1);

        }
        // gpd 抓取结果订阅
        sub_ = nh_.subscribe<gpd::GraspConfigList>("/detect_grasps/clustered_grasps",1,&GpdAdapterActionServer::analysisCB,this);
        // 标记选中的姿态
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/selected_pose",1);
        tf_listener_.reset(new tf::TransformListener);
        ROS_INFO("successfully launched gpd adapter action server!!!!!!");

        //! 所有的action，如果在开始的时候不选择启动，就要手动启动
        as_.start();
    }

    GpdAdapterActionServer::~GpdAdapterActionServer()
    {

    }
    void GpdAdapterActionServer::goalCB()
    {
        std_srvs::Trigger req;
        //! action 在使用的时候要注意，必须接受新的goal，才可以开始一个任务。
        as_.acceptNewGoal();
        service_client_.call(req);
        if(req.response.success)
        {
            ROS_INFO("Sended a cloud to gpd server!!!!");
        }

    }
    void GpdAdapterActionServer::preemptCB()
    {
        ROS_INFO("%s: Preempted",action_name_.c_str());
        as_.setPreempted();
    }
    void GpdAdapterActionServer::analysisCB(const gpd::GraspConfigList msg)
    {
        ROS_INFO_STREAM("--------Received : " << msg.grasps.size() << "grasps---------");
        ROS_INFO_STREAM("is active "<< as_.isActive() );
        ROS_INFO_STREAM("is preemt" << as_.isPreemptRequested());
        if (!as_.isActive() || as_.isPreemptRequested()) return;
        filtered_cloud_publisher::GpdAdapterResult result;

        geometry_msgs::Pose res_pose;
        tf::StampedTransform transform;
        try{
            tf_listener_->waitForTransform("/camera_link", "/table_top", ros::Time::now(), ros::Duration(5.0));
            tf_listener_->lookupTransform ("/camera_link", "/table_top", ros::Time(0), transform);
        }
        catch(std::runtime_error &e){
            ROS_ERROR("Could not find /base_link to /table_top transform");
            as_.setAborted();
            return;
        }
        res_pose = creatPickingEEFPose(msg.grasps,transform);
        result.pose = res_pose;
        visualization_msgs::MarkerArray markers;
        markers = creatMarker(convertGraspConfig2PoseMsgs(msg.grasps[0]));
        marker_pub_.publish(markers);
        as_.setSucceeded(result,"get grasp pose success");
    }

}