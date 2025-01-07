#include "mid360_preprocessor/mid_feature_extractor.hpp"

namespace mid360_preprocessor {

MidFeatureExtractor::MidFeatureExtractor(const rclcpp::NodeOptions &options)
    : Node("mid_feature_extractor", options),
    extract_(std::make_shared<pcl::ExtractIndices<pcl::PointXYZI>>())
{
    this->declare_parameter<std::string>("input_topic", "livox/concatenated/ring_granted/pointcloud");
    this->declare_parameter<std::string>("output_topic_head", "livox/concatenated/feature_extracted");
    this->get_parameter("input_topic", sub_pcd_topic_);
    this->get_parameter("output_topic_head", pub_pcd_topic_head_);
    // Intensity Voxel Standard Deviation Filter Params
    this->declare_parameter<float>("intensity_voxel_size", 0.1);
    this->declare_parameter<float>("intensity_stddev_threshold", 0.0);
    this->declare_parameter<bool>("use_intensity_voxel_stddev_filter", false);
    this->get_parameter("intensity_voxel_size", intensity_voxel_size_);
    this->get_parameter("intensity_stddev_threshold", intensity_stddev_threshold_);
    this->get_parameter("use_intensity_filter", use_intensity_filter_);
    // Intensity Filter Params
    this->declare_parameter<float>("intensity_threshold", 0.0);
    this->declare_parameter<bool>("use_intensity_filter", false);
    this->declare_parameter<bool>("plot_intensity", false);
    this->get_parameter("intensity_threshold", intensity_threshold_);
    this->get_parameter("use_intensity_voxel_stddev_filter", use_intensity_voxel_stddev_filter_);
    this->get_parameter("plot_intensity", plot_intensity_);
    // Hough Transform Params
    this->declare_parameter<bool>("use_hough_transform", false);
    this->declare_parameter<int>("theta_bins", 180);
    this->declare_parameter<int>("phi_bins", 90);
    this->declare_parameter<int>("rho_bins", 100);
    this->declare_parameter<float>("distance_threshold", 0.01);
    this->declare_parameter<float>("hough_threshold_ratio", 0.75);
    this->get_parameter("use_hough_transform", use_hough_transform_);
    this->get_parameter("distance_threshold", distance_threshold_);
    this->get_parameter("hough_threshold_ratio", hough_threshold_ratio_);
    this->get_parameter("theta_bins", theta_bins_);
    this->get_parameter("phi_bins", phi_bins_);
    this->get_parameter("rho_bins", rho_bins_);
    // Height Threshold Filter Params
    this->declare_parameter<bool>("use_height_threshold_filter", false);
    this->declare_parameter<float>("height_max", 0.0);
    this->declare_parameter<float>("height_min", 0.0);
    this->get_parameter("use_height_threshold_filter", use_height_threshold_filter_);
    this->get_parameter("height_max", height_max_);
    this->get_parameter("height_min", height_min_);

    sub_pcd_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        sub_pcd_topic_,
        rclcpp::QoS(5).best_effort(),
        std::bind(&MidFeatureExtractor::callback, this, std::placeholders::_1)
    );

    std::string ivsddf_topic = pub_pcd_topic_head_ + "/intensity_voxel_stddev_filtered/pointcloud";
    std::string intensity_filtered_topic = pub_pcd_topic_head_ + "/intensity_filtered/pointcloud";
    std::string hough_transformed_topic = pub_pcd_topic_head_ + "/hough_transform/pointcloud";
    std::string height_filtered_topic = pub_pcd_topic_head_ + "/height_filtered/pointcloud";
    pcd_pub_vec_.push_back(this->create_publisher<sensor_msgs::msg::PointCloud2>(
        ivsddf_topic,
        rclcpp::QoS(5).best_effort()
    ));
    pcd_pub_vec_.push_back(this->create_publisher<sensor_msgs::msg::PointCloud2>(
        intensity_filtered_topic,
        rclcpp::QoS(5).best_effort()
    ));
    pcd_pub_vec_.push_back(this->create_publisher<sensor_msgs::msg::PointCloud2>(
        hough_transformed_topic,
        rclcpp::QoS(5).best_effort()
    ));
    pcd_pub_vec_.push_back(this->create_publisher<sensor_msgs::msg::PointCloud2>(
        height_filtered_topic,
        rclcpp::QoS(5).best_effort()
    ));
    
}

void MidFeatureExtractor::callback(sensor_msgs::msg::PointCloud2::UniquePtr ros_pcd) {
    RCLCPP_INFO(this->get_logger(), "\033[1;32mFeature Extractor Received PointCloud Address: %p\033[0m", (void *)ros_pcd.get());
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*ros_pcd, *cloud);
    if (use_hough_transform_) {
        auto line_points = hought_transform(cloud, theta_bins_, phi_bins_, rho_bins_, distance_threshold_, hough_threshold_ratio_);
        sensor_msgs::msg::PointCloud2 ros_line_points;
        pcl::toROSMsg(*line_points, ros_line_points);
        ros_line_points.header = ros_pcd->header;
        ros_line_points.header.frame_id = ros_pcd->header.frame_id;
        RCLCPP_INFO(this->get_logger(), "\033[1;32mHough Transformed Published PointCloud Address: %p\033[0m", (void *)line_points.get());
        pcd_pub_vec_[2]->publish(ros_line_points);
    }
    if (use_height_threshold_filter_) {
        filter_by_z_range(cloud, height_min_, height_max_);
        sensor_msgs::msg::PointCloud2 ros_filtered;
        pcl::toROSMsg(*cloud, ros_filtered);
        ros_filtered.header = ros_pcd->header;
        ros_filtered.header.frame_id = ros_pcd->header.frame_id;
        RCLCPP_INFO(this->get_logger(), "\033[1;32mHeight Threshold Filtered Published PointCloud Address: %p\033[0m", (void *)cloud.get());
        pcd_pub_vec_[3]->publish(ros_filtered);
    }
}

// .*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*.*

// Intensity Filter --------------------------------------------------------
void MidFeatureExtractor::filter_by_intensity(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    const float intensity_threshold
) {
    float mean = 0.0;
    for (const auto &p : cloud->points) mean += p.intensity;
    mean /= cloud->points.size();
    float var = 0.0;
    for (const auto &p : cloud->points) var += (p.intensity - mean) * (p.intensity - mean);
    var /= cloud->points.size();
    float stddev = sqrt(var);
}

// -------------------------------------------------------------------------

// Hough Transform ---------------------------------------------------------
bool MidFeatureExtractor::is_local_maximum(
    const std::vector<int>& accumulator,
    const int accumulator_index,
    const int theta_index,
    const int phi_index,
    const int rho_index,
    const int neighborhood_size
) {
    int center_votes = accumulator[accumulator_index];
    for (int dt = -neighborhood_size; dt <= neighborhood_size; dt++) {
        for (int dp = -neighborhood_size; dp <= neighborhood_size; dp++) {
            for (int dr = -neighborhood_size; dr <= neighborhood_size; dr++) {
                if (dt == 0 && dp == 0 && dr == 0) continue;
                int t = theta_index + dt, p = phi_index + dp, r = rho_index + dr;
                if (t < 0 || t >= theta_bins_) continue;
                if (p < 0 || p >= phi_bins_) continue;
                if (r < 0 || r >= rho_bins_) continue;
                if (accumulator[r * theta_bins_ * phi_bins_ + t * phi_bins_ + p] > center_votes) return false;
            }
        }
    }
    return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr MidFeatureExtractor::generate_line_points(
    const LineParameter& line,
    const float max_distance,
    const float resolution
) {
    auto line_points = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    Eigen::Vector3d direction;
    direction << sin(line.phi) * cos(line.theta),
                 sin(line.phi) * sin(line.theta),
                 cos(line.phi);
    Eigen::Vector3d origin = line.rho * direction;
    for (float d = 0; d < max_distance; d += resolution) {
        pcl::PointXYZI p;
        p.x = origin[0] + d * direction[0];
        p.y = origin[1] + d * direction[1];
        p.z = origin[2] + d * direction[2];
        p.intensity = 255;
        line_points->push_back(p);
    }
    return line_points;
}

void MidFeatureExtractor::vote_hough_space(
    std::vector<int>& accumulator,
    const Eigen::Vector3f& point,
    const float theta_bins,
    const float phi_bins,
    const float rho_bins,
    const float max_dist
) {
    double point_norm = point.norm();
    if (point_norm < 1e-6) return;
    for (int theta_idx = 0; theta_idx < theta_bins; theta_idx++) {
        double theta = 2.0 * M_PI * theta_idx / theta_bins;
        for (int phi_idx = 0; phi_idx < phi_bins; phi_idx++) {
            double phi = M_PI * phi_idx / phi_bins;
            Eigen::Vector3f n; // 法線ベクトル
            n << sin(phi) * cos(theta),
                 sin(phi) * sin(theta),
                 cos(phi);
            double rho = abs(point.dot(n));
            int rho_idx = static_cast<int>((rho / max_dist) * (rho_bins - 1));
            if (rho_idx < 0 || rho_idx >= rho_bins) continue;
            accumulator[rho_idx * theta_bins * phi_bins + theta_idx * phi_bins + phi_idx]++;
        }
    }
}

pcl::PointIndices::Ptr MidFeatureExtractor::extract_line_indices(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
    const int theta_idx,
    const int phi_idx,
    const int rho_idx,
    const float max_distance,
    const float distance_threshold
) {
    auto indices = std::make_shared<pcl::PointIndices>();
    double theta = 2.0 * M_PI * theta_idx / theta_bins_;
    double phi = M_PI * phi_idx / phi_bins_;
    double rho = (rho_idx * max_distance) / (rho_bins_ - 1);
    Eigen::Vector3f line_direction;
    line_direction << sin(phi) * cos(theta),
                      sin(phi) * sin(theta),
                      cos(phi);
    for (int i = 0; i < cloud->points.size(); i++) {
        double dist = abs(cloud->points[i].getVector3fMap().dot(line_direction) - rho);
        // 対象の直線と点の距離が閾値以下の場合、その点を抽出
        if (dist < distance_threshold) indices->indices.push_back(i);
    }
}

std::vector<pcl::PointIndices> MidFeatureExtractor::extract_maxima(
    const std::vector<int>& accumulator,
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
    const float theta_bins,
    const float phi_bins,
    const float rho_bins,
    const float distance_threshold,
    const float max_distance,
    const float threshold_ratio 
) {
    int max_votes = *std::max_element(accumulator.begin(), accumulator.end());
    int threshold = static_cast<int>(threshold_ratio * max_votes);
    std::vector<pcl::PointIndices> detected_lines;
    for (int theta = 0; theta < theta_bins; theta++) {
        for (int phi = 0; phi < phi_bins; phi++) {
            for (int rho = 0; rho < rho_bins; rho++) {
                int idx = theta * phi_bins * rho_bins + phi * rho_bins + rho;
                if (accumulator[idx] < threshold) continue;
                auto line_indices = extract_line_indices(
                    cloud,
                    theta,
                    phi,
                    rho,
                    max_distance,
                    distance_threshold
                );
                if (line_indices->indices.size() > 0) detected_lines.push_back(*line_indices);
            }
        }
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr MidFeatureExtractor::hought_transform(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
    const float theta_bins,
    const float phi_bins,
    const float rho_bins,
    const float distance_threshold,
    const float threshold_ratio     // 最大値に対しての割合
) {
    std::vector<int> accumulator(rho_bins * theta_bins * phi_bins, 0);
    Eigen::Vector4f min_p, max_p;
    pcl::getMinMax3D(*cloud, min_p, max_p);
    float max_dist = (max_p - min_p).norm();
    // Hough空間への変換
    for (const auto &point : cloud->points) {
        vote_hough_space(accumulator, point.getVector3fMap(), theta_bins, phi_bins, rho_bins, max_dist);
    }
    RCLCPP_INFO(this->get_logger(), "Hough Space Voting Completed.");
    // 最大値の抽出
    auto line_indices = extract_maxima(
        accumulator,
        cloud,
        theta_bins,
        phi_bins,
        rho_bins,
        distance_threshold,
        max_dist,
        threshold_ratio
    );
    RCLCPP_INFO(this->get_logger(), "Detected %d lines.", line_indices.size());
    // 抽出した直線を点群に変換
    auto line_points = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    for (const auto &indices : line_indices) {
        for (const auto &i : indices.indices) {
            line_points->push_back(cloud->points[i]);
        }
    }

    return line_points;

}
// -------------------------------------------------------------------------


} // namespace mid360_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mid360_preprocessor::MidFeatureExtractor)
