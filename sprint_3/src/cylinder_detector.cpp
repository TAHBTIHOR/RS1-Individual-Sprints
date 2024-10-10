#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <cmath>
#include <vector>

// Define the CylinderDetectorNode class, which inherits from rclcpp::Node
class CylinderDetectorNode : public rclcpp::Node
{
public:
    // Constructor for the CylinderDetectorNode
    CylinderDetectorNode()
    : Node("cylinder_detector"),
      tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
      tf_listener_(tf_buffer_)
    {
        // Call the initialization function to set up subscriptions and publishers
        init();
    }

private:
    // ROS 2 components for handling laser scan, marker visualization, and transformations
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_; // Subscription to laser scan data
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_; // Publisher for RViz marker visualization
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_; // Publisher for detected cylinder pose
    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_; // TF2 broadcaster for cylinder transform

    tf2_ros::Buffer tf_buffer_; // TF2 buffer to store transformations
    tf2_ros::TransformListener tf_listener_; // Listener to retrieve transformations from the buffer

    double cylinder_diameter_; // Diameter of the cylinder (in meters)
    double cylinder_radius_; // Radius of the cylinder (in meters)

    // Initialization function to set up subscriptions, publishers, and parameters
    void init()
    {
        // Subscribe to the laser scan topic to receive scan data
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetectorNode::process_scan, this, std::placeholders::_1));

        // Create a publisher for visualizing the detected cylinder in RViz
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/detected_cylinder_marker", 10);

        // Create a publisher for the pose of the detected cylinder (optional)
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/detected_cylinder_pose", 10);

        // Create a TF2 broadcaster for broadcasting the detected cylinder's position
        transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Set the expected cylinder diameter and calculate its radius
        cylinder_diameter_ = 0.30; // Set cylinder diameter to 30 cm
        cylinder_radius_ = cylinder_diameter_ / 2.0; // Radius is half of the diameter (0.15 m)
    }

    // Callback function to process incoming laser scan data
    void process_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan_data)
    {
        // Vectors to store Cartesian coordinates of scan points
        std::vector<double> x_coords, y_coords;

        // Convert the laser scan data from polar to Cartesian coordinates
        convert_scan_to_cartesian(scan_data, x_coords, y_coords);

        // Attempt to detect a cylindrical object based on the converted coordinates
        int detected_idx = detect_cylinder_cluster(x_coords, y_coords);
        if (detected_idx != -1) // If a cylinder is detected
        {
            // Retrieve the position of the detected cylinder
            double cylinder_x = x_coords[detected_idx];
            double cylinder_y = y_coords[detected_idx];

            // Visualize the detected cylinder in RViz
            visualize_cylinder(cylinder_x, cylinder_y);

            // Broadcast the cylinder's position as a transform
            send_cylinder_transform(cylinder_x, cylinder_y);
        }
    }

    // Function to convert laser scan data from polar to Cartesian coordinates
    void convert_scan_to_cartesian(
        const sensor_msgs::msg::LaserScan::SharedPtr& scan_data,
        std::vector<double>& x_coords,
        std::vector<double>& y_coords)
    {
        double angle_min = scan_data->angle_min; // Starting angle of the scan
        double angle_increment = scan_data->angle_increment; // Angle between each scan point

        // Iterate over all scan ranges to convert them to Cartesian coordinates
        for (size_t i = 0; i < scan_data->ranges.size(); ++i)
        {
            double distance = scan_data->ranges[i]; // Distance measurement for this scan point
            double angle = angle_min + i * angle_increment; // Calculate the angle for this point

            // Convert from polar (distance, angle) to Cartesian (x, y)
            double x = distance * std::cos(angle);
            double y = distance * std::sin(angle);

            // Store the converted coordinates
            x_coords.push_back(x);
            y_coords.push_back(y);
        }
    }

    // Function to detect a potential cylindrical cluster from the Cartesian coordinates
    int detect_cylinder_cluster(const std::vector<double>& x_coords, const std::vector<double>& y_coords)
    {
        size_t num_points = x_coords.size();
        std::vector<double> distances(num_points - 1);

        // Compute distances between successive points
        for (size_t i = 0; i < num_points - 1; ++i)
        {
            distances[i] = std::hypot(x_coords[i + 1] - x_coords[i], y_coords[i + 1] - y_coords[i]);
        }

        // Identify clusters of points that might correspond to a cylindrical object
        return find_cluster_in_distances(distances, x_coords, y_coords);
    }

    // Function to find clusters within the calculated distances that might represent a cylinder
    int find_cluster_in_distances(
        const std::vector<double>& distances,
        const std::vector<double>& x_coords,
        const std::vector<double>& y_coords)
    {
        size_t num_points = distances.size();
        double clustering_threshold = 0.15; // Threshold for clustering (15 cm)

        // Iterate over all points to identify clusters
        for (size_t i = 0; i < num_points; ++i)
        {
            std::vector<size_t> cluster_indices = {i}; // Start a new cluster with the current point

            // Continue adding points to the cluster as long as they are close enough
            while (i < num_points && distances[i] < clustering_threshold)
            {
                cluster_indices.push_back(i + 1); // Add the next point to the cluster
                ++i;
            }

            // Check if the detected cluster matches the expected size of the cylinder
            if (is_cylinder_cluster(x_coords, y_coords, cluster_indices))
            {
                return cluster_indices.front(); // Return the index of the first point in the cluster
            }
        }

        return -1; // Return -1 if no cylinder-like cluster is found
    }

    // Function to verify if a detected cluster matches the expected dimensions of a cylinder
    bool is_cylinder_cluster(
        const std::vector<double>& x_coords,
        const std::vector<double>& y_coords,
        const std::vector<size_t>& cluster_indices)
    {
        // Calculate the bounds of the cluster
        double min_x = x_coords[cluster_indices.front()];
        double max_x = x_coords[cluster_indices.back()];
        double min_y = y_coords[cluster_indices.front()];
        double max_y = y_coords[cluster_indices.back()];

        // Calculate the cluster's width
        double cluster_width = std::hypot(max_x - min_x, max_y - min_y);

        // Check if the width is within the range of the expected cylinder size (25 cm to 30 cm)
        return (cluster_width >= 0.25 && cluster_width <= 0.30);
    }

    // Function to visualize the detected cylinder in RViz using a marker
    void visualize_cylinder(double x, double y)
    {
        visualization_msgs::msg::Marker marker;
        marker.ns = "cylinder_visualization"; // Namespace for marker
        marker.id = 1; // Unique ID for the marker
        marker.type = visualization_msgs::msg::Marker::CYLINDER; // Set the marker type to a cylinder
        marker.action = visualization_msgs::msg::Marker::ADD; // Action to add the marker
        marker.scale.x = cylinder_diameter_; // Set the marker's width (x-axis) to the cylinder's diameter
        marker.scale.y = cylinder_diameter_; // Set the marker's depth (y-axis) to the cylinder's diameter
        marker.scale.z = 1.0; // Set the height of the marker
        marker.color.r = 0.0; // Set the marker color to green (RGB)
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0; // Set the transparency of the marker (1.0 = fully opaque)

        // Try to transform the detected position to the map frame for visualization
        try
        {
            auto transformed_position = transform_position_to_map(x, y);
            marker.pose.position.x = transformed_position.point.x;
            marker.pose.position.y = transformed_position.point.y;
            marker.pose.position.z = 0.0; // Set the height of the cylinder on the ground
            marker.pose.orientation.w = 1.0;

            marker.header.frame_id = "map"; // Frame in which the marker is published
            marker.header.stamp = this->get_clock()->now();

            // Publish the marker to RViz
            marker_publisher_->publish(marker);
        }
        catch (const tf2::TransformException& ex)
        {
            // Warn if the transformation failed
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
        }
    }

    // Function to transform the position of a detected cylinder from base_link to the map frame
    geometry_msgs::msg::PointStamped transform_position_to_map(double x, double y)
    {
        geometry_msgs::msg::PointStamped base_point, map_point;
        base_point.header.frame_id = "base_link"; // Set the original frame
        base_point.point.x = x; // X position of the detected cylinder
        base_point.point.y = y; // Y position of the detected cylinder

        // Look up the transformation between base_link and map frames
        auto transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
        tf2::doTransform(base_point, map_point, transform); // Apply the transformation
        return map_point; // Return the transformed point
    }

    // Function to broadcast a TF transform for the detected cylinder's position
    void send_cylinder_transform(double x, double y)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now(); // Set the time for the transform
        transform.header.frame_id = "base_link"; // The parent frame of the transform
        transform.child_frame_id = "cylinder_frame"; // The child frame (name for the detected object)
        transform.transform.translation.x = x; // Set x translation
        transform.transform.translation.y = y; // Set y translation
        transform.transform.translation.z = 0.0; // No z translation (on the ground)
        transform.transform.rotation.w = 1.0; // Identity quaternion (no rotation)

        // Broadcast the transform so other nodes can use the position
        transform_broadcaster_->sendTransform(transform);
    }
};

// Main function to start the ROS 2 node
int main(int argc, char** argv)
{
    // Initialize ROS 2 with command-line arguments
    rclcpp::init(argc, argv);

    // Create and spin the node
    auto node = std::make_shared<CylinderDetectorNode>();
    rclcpp::spin(node);

    // Shutdown ROS 2 when done
    rclcpp::shutdown();
    return 0;
}
