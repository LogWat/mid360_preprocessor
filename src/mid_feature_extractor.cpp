#include "mid360_preprocessor/mid_feature_extractor.hpp"

namespace mid360_preprocessor {

MidFeatureExtractor::MidFeatureExtractor(const rclcpp::NodeOptions &options)
    : Node("mid_feature_extractor", options)
{
}
} // namespace mid360_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mid360_preprocessor::MidFeatureExtractor)
