cmake_minimum_required(VERSION 3.8)
project(mid360_preprocessor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(mid_concat_filter SHARED src/mid_concat_filter.cpp)
add_library(ring_granter SHARED src/ring_granter.cpp)
add_library(mid_feature_extractor SHARED src/mid_feature_extractor.cpp)

rclcpp_components_register_nodes(
  mid_concat_filter "mid360_preprocessor::MidConcatFilter"
)
rclcpp_components_register_nodes(
  ring_granter "mid360_preprocessor::RingGranter"
)
rclcpp_components_register_nodes(
  mid_feature_extractor "mid360_preprocessor::MidFeatureExtractor"
)

target_compile_definitions(mid_concat_filter
  PRIVATE "MID_CONCAT_FILTER_BUILDING_LIBRARY"
)
target_compile_definitions(ring_granter
  PRIVATE "RING_GRANTER_BUILDING_LIBRARY"
)
target_compile_definitions(mid_feature_extractor
  PRIVATE "MID_FEATURE_EXTRACTOR_BUILDING_LIBRARY"
)

target_include_directories(mid_concat_filter
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(ring_granter
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(mid_feature_extractor
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(mid_concat_filter
  PUBLIC
  c_std_99
  cxx_std_17
)
target_compile_features(ring_granter
  PUBLIC
  c_std_99
  cxx_std_17
)
target_compile_features(mid_feature_extractor
  PUBLIC
  c_std_99
  cxx_std_17
)

ament_target_dependencies(mid_concat_filter
  "rclcpp"
  "sensor_msgs"
  "pcl_ros"
  "pcl_conversions"
  "rclcpp_components"
  "tf2"
  "tf2_ros"
)
ament_target_dependencies(ring_granter
  "rclcpp"
  "sensor_msgs"
  "pcl_ros"
  "pcl_conversions"
  "rclcpp_components"
  "tf2"
  "tf2_ros"
)
ament_target_dependencies(mid_feature_extractor
  "rclcpp"
  "sensor_msgs"
  "pcl_ros"
  "pcl_conversions"
  "rclcpp_components"
  "tf2"
  "tf2_ros"
)

install(
  TARGETS mid_concat_filter
  DESTINATION lib
)
install(
  TARGETS ring_granter
  DESTINATION lib
)
install(
  TARGETS mid_feature_extractor
  DESTINATION lib
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
