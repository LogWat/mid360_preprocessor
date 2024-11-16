# Mid360 Preprocessor

## What
- Point cloud merging for Livox Mid360 (currently supports merging of 2 units only)
- Ring Field reconstruction (for use with LIO-SAM and similar applications)

## Components
- MidConcatFilter: Component for point cloud merging
- RingGranter: Component for Ring reconstruction

## Test Launch Files
- launch/mid360_concatenate.launch.xml
    - Launch file for testing point cloud merging only
    - Relative positions of each Livox unit are provided via TF
- launch/mid360_with_ring.launch.xml
    - Launch file for testing point cloud merging + ring reconstruction

## Dependencies
- ROS2 Humble
- PCL
- Eigen
- tf2_ros

## Other
- Zero-Copy implementation is possible with some modifications
- Please adjust `fov_top` and `fov_bottom` during Ring reconstruction according to your usage conditions and TF setup