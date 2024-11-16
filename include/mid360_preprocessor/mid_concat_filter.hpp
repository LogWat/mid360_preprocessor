#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>
#include <string>
#include <vector>

rmw_qos_profile_t qos_profile_lidar{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

auto qos_lidar = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile_lidar.history,
        qos_profile_lidar.depth
    ),
    qos_profile_lidar);

namespace mid360_preprocessor {

class MidConcatFilter : public rclcpp::Node {
public:
    MidConcatFilter(const rclcpp::NodeOptions & options);
    MidConcatFilter(
        const std::string &name_space,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
    );

private:
    void concatenate_ros_pointclouds(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl1,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl2,
        const Eigen::Affine3f &transform1,
        const Eigen::Affine3f &transform2,
        sensor_msgs::msg::PointCloud2::SharedPtr &pointcloud_out
    );

    bool check_compatibility(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl1,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl2
    );

    void sync_callback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr ros_pcd1,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr ros_pcd2
    );

    // variables --------------------------------------------------------------
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_pcd1_, sub_pcd2_;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;
    std::shared_ptr<Sync> sync_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcd_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string pcd1_topic_, pcd2_topic_, pcd1_frame_id_, pcd2_frame_id_;
    std::string output_topic_, output_frame_id_;
    Eigen::Affine3f transform1_, transform2_;
    bool get_tf1_ = false, get_tf2_ = false;
};

} // namespace mid360_preprocessor