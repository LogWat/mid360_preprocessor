#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace mid360_preprocessor {

class MidFeatureExtractor : public rclcpp::Node {
public:
    MidFeatureExtractor(const rclcpp::NodeOptions & options);
    MidFeatureExtractor(
        const std::string &name_space,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
    );

private:
};

} // namespace mid360_preprocessor