#include "mid360_preprocessor/mid_concat_filter.hpp"

namespace mid360_preprocessor {

MidConcatFilter::MidConcatFilter(const rclcpp::NodeOptions & options)
    : Node("mid_concat_filter", options),
        tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
        tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) {
    this->declare_parameter<std::string>("livox1_topic", "livox1");
    this->declare_parameter<std::string>("livox1_frame", "livox1");
    this->declare_parameter<std::string>("livox2_topic", "livox2");
    this->declare_parameter<std::string>("livox2_frame", "livox2");
    this->declare_parameter<std::string>("output_topic", "livox/concatenated_pcd");
    this->declare_parameter<std::string>("output_frame", "livox_link");
    this->get_parameter("livox1_topic", pcd1_topic_);
    this->get_parameter("livox1_frame", pcd1_frame_id_);
    this->get_parameter("livox2_topic", pcd2_topic_);
    this->get_parameter("livox2_frame", pcd2_frame_id_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("output_frame", output_frame_id_);

    // 同期subscriber
    sub_pcd1_.subscribe(this, pcd1_topic_, qos_profile_lidar);
    sub_pcd2_.subscribe(this, pcd2_topic_, qos_profile_lidar);
    sync_.reset(new Sync(SyncPolicy(10), sub_pcd1_, sub_pcd2_));
    sync_->registerCallback(&MidConcatFilter::sync_callback, this);

    // publisher
    pub_pcd_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        output_topic_,
        qos_lidar
    );
}

void MidConcatFilter::concatenate_ros_pointclouds(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl2,
    const Eigen::Affine3f &transform1,
    const Eigen::Affine3f &transform2,
    sensor_msgs::msg::PointCloud2::SharedPtr &pointcloud_out
) {
    auto pointcloud1 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto pointcloud2 = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*pcl1, *pointcloud1);
    pcl::fromROSMsg(*pcl2, *pointcloud2);
    pcl::transformPointCloud(*pointcloud1, *pointcloud1, transform1.matrix());
    pcl::transformPointCloud(*pointcloud2, *pointcloud2, transform2.matrix());
    auto concatenated = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    *concatenated = *pointcloud1 + *pointcloud2;
    pcl::toROSMsg(*concatenated, *pointcloud_out);
}

bool MidConcatFilter::check_compatibility(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl2
) {
    return pcl1->is_bigendian == pcl2->is_bigendian &&
           pcl1->point_step == pcl2->point_step &&
           pcl1->fields.size() == pcl2->fields.size() &&
           pcl1->is_dense == pcl2->is_dense &&
           std::equal(
               pcl1->fields.begin(),
               pcl1->fields.end(),
               pcl2->fields.begin(),
               [](const sensor_msgs::msg::PointField &f1, const sensor_msgs::msg::PointField &f2) {
                   return f1.name == f2.name &&
                          f1.offset == f2.offset &&
                          f1.datatype == f2.datatype &&
                          f1.count == f2.count;
               }
           );
}

void MidConcatFilter::sync_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr ros_pcd1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr ros_pcd2
) {
    // memory address check
    RCLCPP_INFO(this->get_logger(), "PointCloud1 Address: %p", (void *)ros_pcd1.get());
    RCLCPP_INFO(this->get_logger(), "PointCloud2 Address: %p", (void *)ros_pcd2.get());

    if (!check_compatibility(ros_pcd1, ros_pcd2)) {
        RCLCPP_WARN(this->get_logger(), "PointCloud2 messages are not compatible.");
        return;
    }
    // livox1 -> output_livox への変換行列取得
    if (!get_tf1_) {
        try {
            auto tf_pcd1 = tf_buffer_->lookupTransform(output_frame_id_, pcd1_frame_id_, rclcpp::Time(0));
            transform1_.translation() << 
                tf_pcd1.transform.translation.x, 
                tf_pcd1.transform.translation.y,
                tf_pcd1.transform.translation.z;
            transform1_.linear() = Eigen::Quaternionf(
                tf_pcd1.transform.rotation.w,
                tf_pcd1.transform.rotation.x,
                tf_pcd1.transform.rotation.y,
                tf_pcd1.transform.rotation.z
            ).toRotationMatrix();
            get_tf1_ = true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }
    }
    // livox2 -> output_livox への変換行列取得
    if (!get_tf2_) {
        try {
            auto tf_pcd2 = tf_buffer_->lookupTransform(output_frame_id_, pcd2_frame_id_, rclcpp::Time(0));
            transform2_.translation() << 
                tf_pcd2.transform.translation.x, 
                tf_pcd2.transform.translation.y,
                tf_pcd2.transform.translation.z;
            transform2_.linear() = Eigen::Quaternionf(
                tf_pcd2.transform.rotation.w,
                tf_pcd2.transform.rotation.x,
                tf_pcd2.transform.rotation.y,
                tf_pcd2.transform.rotation.z
            ).toRotationMatrix();
            get_tf2_ = true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }
    }

    auto pointcloud_out = std::make_shared<sensor_msgs::msg::PointCloud2>();
    concatenate_ros_pointclouds(
        ros_pcd1,
        ros_pcd2,
        transform1_,
        transform2_,
        pointcloud_out
    );
    pointcloud_out->header = ros_pcd1->header;
    pointcloud_out->header.frame_id = output_frame_id_;
    RCLCPP_INFO(this->get_logger(), "Concatenated PointCloud Address: %p", (void *)pointcloud_out.get());
    pub_pcd_->publish(*pointcloud_out);
}


} // namespace mid360_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mid360_preprocessor::MidConcatFilter)