#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace mid360_preprocessor {

class RingGranter : public rclcpp::Node {
public:
    RingGranter(const rclcpp::NodeOptions & options);
    RingGranter(
        const std::string &name_space,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
    );

private:
    void grant_ring_field(
        const sensor_msgs::msg::PointCloud2::UniquePtr &ros_pcd,
        sensor_msgs::msg::PointCloud2::SharedPtr &pointcloud_out
    );

    void timefield_overwrite(
        sensor_msgs::msg::PointCloud2::SharedPtr &pointcloud_out
    );

    void callback(sensor_msgs::msg::PointCloud2::UniquePtr ros_pcd);

    // variables---------------------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcd_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcd_;
    std::string sub_pcd_topic_, pub_pcd_topic_;
    
    bool timefield_overwrite_ = false;

    // ring関係のパラメータ
    int ring_num_;
    const float deg2rad__ = M_PI / 180.0;
};

} // namespace mid360_preprocessor