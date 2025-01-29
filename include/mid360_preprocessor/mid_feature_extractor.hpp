#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>           // pcl::getMinMax3D
#include <pcl_ros/transforms.hpp>
#include <pcl/filters/extract_indices.h> // pcl::ExtractIndices
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <map>
#include <tuple>
#include <algorithm>
#include <cmath>
// #include "matplotlibcpp.h" // https://github.com/lava/matplotlib-cpp

namespace mid360_preprocessor {

class MidFeatureExtractor : public rclcpp::Node {
public:
    MidFeatureExtractor(const rclcpp::NodeOptions & options);
    MidFeatureExtractor(
        const std::string &name_space,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions()
    );

private:

    void callback(sensor_msgs::msg::PointCloud2::UniquePtr ros_pcd);

    // utils -------------------------------------------------------------------
    void remove_indices(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        const pcl::PointIndices::Ptr inliers
    ) {
        extract_->setInputCloud(cloud);
        extract_->setIndices(inliers);
        extract_->setNegative(true);
        extract_->filter(*cloud);
    }

    void filter_by_distance(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        const float distance_threshold
    ) {
        auto inliers = std::make_shared<pcl::PointIndices>();
        for (size_t i = 0; i < cloud->points.size(); i++) {
            float distance = sqrt(
                cloud->points[i].x * cloud->points[i].x +
                cloud->points[i].y * cloud->points[i].y +
                cloud->points[i].z * cloud->points[i].z
            );
            if (distance > distance_threshold) inliers->indices.push_back(i);
        }
        remove_indices(cloud, inliers);
    }

    void filter_by_z_range(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        const float z_min,
        const float z_max
    ) {
        auto inliers = std::make_shared<pcl::PointIndices>();
        for (size_t i = 0; i < cloud->points.size(); i++) {
            if (cloud->points[i].z > z_min && cloud->points[i].z < z_max) inliers->indices.push_back(i);
        }
        remove_indices(cloud, inliers);
    }

    // clusterごとにintensityで色付け
    void visualize_segmentation(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        const std::vector<pcl::PointIndices>& cluster_indices
    ) {
        for (size_t i = 0; i < cloud->points.size(); i++) {
            int cluster_id = -1;
            for (size_t j = 0; j < cluster_indices.size(); j++) {
                if (std::find(cluster_indices[j].indices.begin(), cluster_indices[j].indices.end(), i) != cluster_indices[j].indices.end()) {
                    cluster_id = j;
                    break;
                }
            }
            if (cluster_id == -1) continue;
            cloud->points[i].intensity = cluster_id * 10;
        }
    }
    // -------------------------------------------------------------------------

    // Intensity Voxel Standard Deviation Filter -------------------------------
    void filter_by_intensity_voxel_stddev(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered,
        const float voxel_size,
        const float stddev_threshold
    );

    void retrieve_voxel_stddev(
        pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
        const float voxel_size,
        std::map<std::tuple<float, float, float>, float> &voxel_stddev_map,
        std::map<std::tuple<float, float, float>, std::vector<int>> &voxel_indices_map
    );
    //--------------------------------------------------------------------------

    // Intensity Filter --------------------------------------------------------
    void filter_by_intensity(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        const float intensity_threshold
    );

    void plot_intensity(
        pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud
    );
    // --------------------------------------------------------------------------

    // Hough Transform ---------------------------------------------------------
    struct LineParameter {
        float theta;
        float phi;
        float rho;
        int votes;
    };

    bool is_local_maximum(
        const std::vector<int>& accumulator,
        const int accumulator_index,
        const int theta_index,
        const int phi_index,
        const int rho_index,
        const int neighborhood_size
    );

    std::vector<pcl::PointIndices> extract_maxima(
        const std::vector<int>& accumulator,
        pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
        const float theta_bins,
        const float phi_bins,
        const float rho_bins,
        const float distance_threshold,
        const float max_distance,
        const float threshold_ratio 
    );

    pcl::PointIndices::Ptr extract_line_indices(
        pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
        const int theta_idx,
        const int phi_idx,
        const int rho_idx,
        const float max_distance,
        const float distance_threshold
    );

    void vote_hough_space(
        std::vector<int>& accumulator,
        const Eigen::Vector3f& point,
        const float theta_bins,
        const float phi_bins,
        const float rho_bins,
        const float max_dist
    );

    pcl::PointCloud<pcl::PointXYZI>::Ptr generate_line_points(
        const LineParameter& line,
        const float max_distance,
        const float resolution
    );

    pcl::PointCloud<pcl::PointXYZI>::Ptr hought_transform(
        pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
        const float theta_bins,
        const float phi_bins,
        const float rho_bins,
        const float distance_threshold,
        const float threshold_ratio
    );
    // --------------------------------------------------------------------------

    // variables ================================================================
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcd_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pcd_pub_vec_;
    std::string sub_pcd_topic_;
    std::string pub_pcd_topic_head_;

    // pcl
    std::shared_ptr<pcl::ExtractIndices<pcl::PointXYZI>> extract_;

    // 各filter処理関係params ---------------------------------------------------

    // Intensity Voxel Standard Deviation Filter
    bool use_intensity_voxel_stddev_filter_;
    float intensity_voxel_size_;
    float intensity_stddev_threshold_;
    float intensity_cofficient_;

    // Intensity Filter
    bool use_intensity_filter_, plot_intensity_;
    float intensity_threshold_;

    // Hough Transform
    bool use_hough_transform_;
    int theta_bins_, phi_bins_, rho_bins_;
    float distance_threshold_, hough_threshold_ratio_;

    // height-threshold filter
    bool use_height_threshold_filter_;
    float height_max_, height_min_;


    // DEBUG
    bool debug_verbose_ = false;

    // ==========================================================================
};

} // namespace mid360_preprocessor