/**
 * @file    voxel_filter_node.cpp
 * @brief   ROS 2 node: Voxel Grid downsampling of raw depth-camera point clouds.
 *
 * The OAK-D Lite depth camera produces dense point clouds (~307 200 points at
 * VGA resolution).  Passing that volume raw to the stair detector would be
 * both slow and noisy.  This node acts as the first stage of the two-stage
 * perception pipeline:
 *
 *   [OAK-D Lite]  →  /depth_camera/points
 *        ↓
 *   [VoxelFilterNode]  →  /depth_camera/points/filtered
 *        ↓
 *   [StairDetectorNode]  →  /stair_info  /step_state
 *
 * ALGORITHM
 * ---------
 * A VoxelGrid filter divides 3-D space into uniform cubic cells ("voxels") of
 * side length `leaf_size`.  All points that fall inside the same voxel are
 * replaced by their centroid, drastically reducing point count while
 * preserving geometric structure.
 *
 * PARAMETERS  (runtime-tunable via `ros2 param set`)
 * ----------
 *   leaf_size  [double, default 0.03 m]
 *       Side length of each voxel cube.  Larger → fewer points, faster but
 *       coarser.  0.03 m is a good balance for 60 mm stair risers.
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class VoxelFilterNode : public rclcpp::Node
{
public:
    VoxelFilterNode() : Node("voxel_filter_node")
    {
        this->declare_parameter("leaf_size", 0.03);
        leaf_size_ = this->get_parameter("leaf_size").as_double();

        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params)
            {
                for (const auto &p : params) {
                    if (p.get_name() == "leaf_size") {
                        leaf_size_ = p.as_double();
                    }
                }
                rcl_interfaces::msg::SetParametersResult res;
                res.successful = true;
                return res;
            });

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/depth_camera/points", rclcpp::SensorDataQoS(),
            std::bind(&VoxelFilterNode::cloudCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/depth_camera/points/filtered", 10);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS 2 message → PCL cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud_in);

        // Apply VoxelGrid downsampling 
        // Each voxel (leaf_size³ cube) is collapsed to its centroid point.
        // This preserves stair-edge geometry while cutting ~10–20× point count.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud_in);
        voxel.setLeafSize(
            static_cast<float>(leaf_size_),   // X voxel size (depth axis)
            static_cast<float>(leaf_size_),   // Y voxel size (lateral axis)
            static_cast<float>(leaf_size_));  // Z voxel size (vertical axis)
        voxel.filter(*cloud_out);

        // Convert PCL cloud → ROS 2 message and publish 
        // Header (frame_id + timestamp) is preserved from the incoming message
        // so downstream nodes have correct TF context.
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_out, output);
        output.header = msg->header;
        pub_->publish(output);
    }

    double leaf_size_;          
    int    count_{0};           

    OnSetParametersCallbackHandle::SharedPtr              param_cb_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelFilterNode>());
    rclcpp::shutdown();
    return 0;
}