#include "mid360_preprocessor/ring_granter.hpp"

namespace mid360_preprocessor {

RingGranter::RingGranter(const rclcpp::NodeOptions & options)
    : Node("ring_granter", options)
{
    this->declare_parameter<std::string>("input_topic", "livox/concatenated/pointcloud");
    this->declare_parameter<std::string>("output_topic", "livox/concatenated/ring_granted/pointcloud");
    this->declare_parameter<int>("ring_num", 10);
    this->declare_parameter<bool>("timefield_overwrite", false);
    this->get_parameter("input_topic", sub_pcd_topic_);
    this->get_parameter("output_topic", pub_pcd_topic_);
    this->get_parameter("ring_num", ring_num_);
    this->get_parameter("timefield_overwrite", timefield_overwrite_);

    sub_pcd_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        sub_pcd_topic_,
        rclcpp::QoS(5).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
        std::bind(&RingGranter::callback, this, std::placeholders::_1)
    );
    pub_pcd_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        pub_pcd_topic_,
        rclcpp::QoS(5).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
    );
}

// 主にliosam用 (timestampだとwarnになるため)
void RingGranter::timefield_overwrite(
    sensor_msgs::msg::PointCloud2::SharedPtr &pointcloud_out
) {
    for (auto &field : pointcloud_out->fields) {
        if (field.name == "timestamp") {
            field.name = "time";
            break;
        }
    }
}

void RingGranter::grant_ring_field(
    const sensor_msgs::msg::PointCloud2::UniquePtr &ros_pcd,
    sensor_msgs::msg::PointCloud2::SharedPtr &pointcloud_out
) {
    pointcloud_out->header = ros_pcd->header;
    pointcloud_out->height = 1;
    pointcloud_out->is_dense = ros_pcd->is_dense;
    pointcloud_out->is_bigendian = ros_pcd->is_bigendian;
    pointcloud_out->width = ros_pcd->width;
    pointcloud_out->fields = ros_pcd->fields;
    // add ring field -------------------------------------
    pointcloud_out->fields.emplace_back();
    auto &ring_field = pointcloud_out->fields.back();
    ring_field.name = "ring";
    ring_field.offset = pointcloud_out->point_step;
    ring_field.datatype = sensor_msgs::msg::PointField::UINT16;
    ring_field.count = 1;
    pointcloud_out->point_step = ros_pcd->point_step + sizeof(uint16_t);
    // -----------------------------------------------------
    pointcloud_out->row_step = pointcloud_out->point_step * pointcloud_out->width;
    pointcloud_out->data.resize(pointcloud_out->row_step);
    size_t x_idx = 0, y_idx = 0, z_idx = 0, i_idx = 0;
    for (size_t i = 0; i < pointcloud_out->fields.size(); ++i) {
        auto &field = pointcloud_out->fields[i];
        if (field.name == "x") x_idx = field.offset;
        else if (field.name == "y") y_idx = field.offset;
        else if (field.name == "z") z_idx = field.offset;
        else if (field.name == "intensity") i_idx = field.offset;
    }

    float fov_top_ = 52.0 * deg2rad__;
    float fov_bottom_ = -52.0 * deg2rad__;
    float vertical_resolution = (fov_top_ - fov_bottom_) / (ring_num_ - 1);
    float z_center = 0.0;
    for (size_t i = 0; i < pointcloud_out->width; i++) {
        float x = *(float *)(&ros_pcd->data[i * ros_pcd->point_step + x_idx]);
        float y = *(float *)(&ros_pcd->data[i * ros_pcd->point_step + y_idx]);
        float z = *(float *)(&ros_pcd->data[i * ros_pcd->point_step + z_idx]);
        float intensity = *(float *)(&ros_pcd->data[i * ros_pcd->point_step + i_idx]);
        float angle = atan2(z - z_center, sqrt(x * x + y * y));
        int ring = (angle - fov_bottom_) / vertical_resolution;
        if (ring < 0) ring = 0;
        if (ring >= ring_num_) ring = ring_num_ - 1;
        *(float *)(&pointcloud_out->data[i * pointcloud_out->point_step + x_idx]) = x;
        *(float *)(&pointcloud_out->data[i * pointcloud_out->point_step + y_idx]) = y;
        *(float *)(&pointcloud_out->data[i * pointcloud_out->point_step + z_idx]) = z;
        *(float *)(&pointcloud_out->data[i * pointcloud_out->point_step + i_idx]) = intensity;
        *(uint16_t *)(&pointcloud_out->data[i * pointcloud_out->point_step + ring_field.offset]) = ring;
    }
}

void RingGranter::callback(sensor_msgs::msg::PointCloud2::UniquePtr ros_pcd) {
    // RCLCPP_INFO(this->get_logger(), "\033[1;36mRingGranter Received PointCloud Address: %p\033[0m", (void *)ros_pcd.get());
    auto pointcloud_out = std::make_shared<sensor_msgs::msg::PointCloud2>();
    grant_ring_field(ros_pcd, pointcloud_out);
    pub_pcd_->publish(*pointcloud_out);
}

} // namespace mid360_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mid360_preprocessor::RingGranter)