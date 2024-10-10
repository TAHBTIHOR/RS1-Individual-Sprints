#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <random>
#include <chrono>
#include <cmath>
#include <mutex>

class ScanMatchingLocalizer : public rclcpp::Node
{
public:
    ScanMatchingLocalizer() : Node("scan_matching_localizer"), gen_(rd_()), noise_dist_(0.0, 0.01), match_found_(false)
    {
        // Parameters for movement and localization
        linear_speed_ = this->declare_parameter("linear_speed", 0.15);
        angular_speed_ = this->declare_parameter("angular_speed", 0.0);
        distance_ = this->declare_parameter("distance", 2.0);
        direction_ = this->declare_parameter("direction", "forward");

        // Publishers and subscribers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        // Odom topic provides ground truth
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&ScanMatchingLocalizer::odom_callback, this, std::placeholders::_1));
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ScanMatchingLocalizer::laser_scan_callback, this, std::placeholders::_1));
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&ScanMatchingLocalizer::map_callback, this, std::placeholders::_1));
        amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&ScanMatchingLocalizer::amcl_callback, this, std::placeholders::_1));
        scan_matching_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("scan_matching_pose", 10);
        // Publisher to publish RMSE values
        rmse_pub_gt_amcl_ = this->create_publisher<geometry_msgs::msg::PointStamped>("RMSE_gt_amcl_values", 10);
        // Publisher to publish RMSE values
        rmse_pub_gt_scanmatching_ = this->create_publisher<geometry_msgs::msg::PointStamped>("RMSE_gt_scanmatching_values", 10);
        // OpenCV window for displaying the map
        cv::namedWindow(WINDOW1, cv::WINDOW_AUTOSIZE);

        // Timer to control image display and robot movement
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ScanMatchingLocalizer::timer_callback, this));

        amcl_flag - false;

        initialized_ = false;
        last_time_ = this->now();
        previous_scan_available_ = false;
    }

private:
    // Mutex for synchronizing access to images between callbacks and display
    std::mutex image_mutex_;

    // Timer callback to control image display and robot movement
    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(image_mutex_);

        // Display the scaled map image
        if (!m_MapColImage.empty())
        {
            cv::Mat tmp_col_img;
            int scale_factor = 2; // Scale the map image by 2x
            cv::resize(m_MapColImage, tmp_col_img, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);
            cv::rotate(tmp_col_img, tmp_col_img, cv::ROTATE_90_COUNTERCLOCKWISE);
            cv::imshow(WINDOW1, tmp_col_img);
        }

        // Display Image A (map section) with scaling
        if (!map_section_.empty())
        {
            cv::Mat scaled_map_section;
            int scale_factor = 2; // Scale Image A by 2x
            cv::resize(map_section_, scaled_map_section, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);
            cv::imshow("Map Section (Image A)", scaled_map_section); // Show scaled Image A
        }

        // Display Image B (edge detection of Image A) with scaling
        if (!edge_map_.empty())
        {
            cv::Mat scaled_edge_map;
            int scale_factor = 2; // Scale Image B by 2x
            cv::resize(edge_map_, scaled_edge_map, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);
            cv::imshow("Edge Detection (Image B)", scaled_edge_map); // Show scaled Image B
        }

        if (!scan_image_.empty())
        {
            cv::imshow("Laser Scan Image (Image C)", scan_image_);
        }

        // Allow OpenCV to refresh the GUI
        cv::waitKey(1);
    }

    // Odometry callback for updating the robot pose
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(image_mutex_);

        if (!initialized_)
        {
            initial_pose_x_ = msg->pose.pose.position.x;
            initial_pose_y_ = msg->pose.pose.position.y;

            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, initial_pose_theta_);

            current_pose_x_ = initial_pose_x_;
            current_pose_y_ = initial_pose_y_;
            current_pose_theta_ = initial_pose_theta_;

            initialized_ = true;

            RCLCPP_INFO_ONCE(this->get_logger(), "Robot pose updated: (x: %f, y: %f, theta: %f)", current_pose_x_, current_pose_y_, current_pose_theta_);
        }
        // Force processing and displaying images after pose update
        extract_and_process_images();

        // Store ground truth pose
    ground_truth_pose_ = geometry_msgs::msg::Point();
    ground_truth_pose_->x = msg->pose.pose.position.x;
    ground_truth_pose_->y = msg->pose.pose.position.y;
    ground_truth_pose_->z = 0;  // No z-position in 2D navigation

    // After updating poses, calculate RMSE
    calculate_and_publish_rmse_gt_amcl();
    }

    void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        amcl_pose_ = msg->pose.pose.position;
        amcl_flag = true;
        calculate_and_publish_rmse_gt_amcl();
    }

    // Laser scan callback for processing laser data (Image C) and obstacle avoidance
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        std::lock_guard<std::mutex> lock(image_mutex_);

        if (!initialized_)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Robot pose not initialized yet.");
            return;
        }

        if (map_.data.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Map data is not yet available.");
            return; // Ensure the map is available before processing
        }

        // Convert laser scan to an image (Image C)
        scan_image_ = laser_scan_to_image(scan_msg);

        // Check for obstacles in front of the robot
        bool obstacle_detected = check_for_obstacles(scan_msg);

        // If obstacle is detected, rotate the robot
        if (obstacle_detected)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Obstacle detected! Rotating to avoid.");
            rotate_robot(scan_msg); // Rotate in place to avoid the obstacle
        }
        else
        {
            // Extract map section (Image A) and apply edge detection to get Image B
            map_section_ = extract_map_section();                 // Store the extracted map section (Image A)
            edge_map_ = apply_canny_edge_detection(map_section_); // Image B

            // Estimate the rotation using Image B and Image C
            estimate_and_correct_rotation();
            RCLCPP_INFO_ONCE(this->get_logger(), "Laser scan and map section processed.");
        }
    }

    void publish_scan_matching_pose()
    {   
        geometry_msgs::msg::PoseStamped pose_msg;  // Declare pose_msg
        pose_msg.header.stamp = this->now();

        // Set the estimated pose from scan matching
        pose_msg.pose.position.x = current_pose_x_; // Estimated x position
        pose_msg.pose.position.y = current_pose_y_; // Estimated y position

        // Set the orientation using the current theta (yaw angle)
        tf2::Quaternion q;
        q.setRPY(0, 0, current_pose_theta_); // Roll and pitch are 0, theta is the yaw
        pose_msg.pose.orientation = tf2::toMsg(q);

        scan_matching_pose_->x = pose_msg.pose.position.x;
        scan_matching_pose_->y = pose_msg.pose.position.y;
        scan_matching_pose_->z = pose_msg.pose.position.z;
        // Publish the pose
        scan_matching_pose_pub_->publish(pose_msg);
    }

    // Function to check for obstacles in front of the robot using laser scan data
    bool check_for_obstacles(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        float min_range = 1;                         // Minimum distance threshold for obstacles (in meters)
        float front_angle_range = 60 * M_PI / 180.0; // 45-degree range in front of the robot

        int num_ranges = scan_msg->ranges.size();
        float angle_min = scan_msg->angle_min;
        float angle_max = scan_msg->angle_max;
        float angle_increment = scan_msg->angle_increment;

        // Iterate through the laser scan ranges in the front of the robot
        for (int i = 0; i < num_ranges; ++i)
        {
            float range = scan_msg->ranges[i];
            float angle = angle_min + i * angle_increment;

            // Check if the laser scan point is within the front angle range
            if (std::abs(angle) < front_angle_range && std::isfinite(range) && range < min_range)
            {
                // Obstacle detected within the threshold distance
                return true;
            }
        }
        return false;
    }

    // Function to extract Image A (map section), Image B (edges), and ensure Image C is updated
    void extract_and_process_images()
    {
        if (!map_.data.empty())
        {
            // Extract map section (Image A)
            map_section_ = extract_map_section();

            // Apply Canny edge detection to get Image B
            edge_map_ = apply_canny_edge_detection(map_section_);

            // Laser scan (Image C) is updated in the laser_scan_callback
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No map data available to extract Image A and B.");
        }
    }

    // Map callback to store the current map and generate the Map Image
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg)
    {
        std::lock_guard<std::mutex> lock(image_mutex_);

        // Store map data in the member variable `map_`
        map_ = *mapMsg;

        RCLCPP_INFO_ONCE(this->get_logger(), "Received map, converting to image...");
        occupancyGridToImage(mapMsg);
        RCLCPP_INFO_ONCE(this->get_logger(), "Map callback processing map image.");
    }

    // Function to estimate the rotation between Image B and Image C and correct the robot's orientation
    void estimate_and_correct_rotation()
    {
        if (edge_map_.empty() || scan_image_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No valid images to estimate rotation.");
            return;
        }

        // Detect and match features between Image B and Image C
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(edge_map_, scan_image_, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for transformation estimation.");
            return;
        }

        // Estimate rotation using affine transformation
        cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
        if (transform_matrix.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
            return;
        }

        // Extract the rotation angle
        double angle_difference = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
        double rotation_degrees = angle_difference * 180.0 / CV_PI;
        RCLCPP_INFO_ONCE(this->get_logger(), "Estimated rotation difference: %f degrees", rotation_degrees);

        // Update the robot's orientation based on the estimated rotation
        current_pose_theta_ += angle_difference;

        // Normalize the angle to the range [-pi, pi]
        current_pose_theta_ = normalize_angle(current_pose_theta_);

        // Propagate the robot using the corrected orientation
        publish_cmd_vel();

        // **Publish the estimated pose**
        publish_scan_matching_pose();
    }

    // Function to detect and match features between two images (Image B and Image C)
    void detectAndMatchFeatures(const cv::Mat &img1, const cv::Mat &img2,
                                std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints)
    {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        // Detect and compute keypoints and descriptors
        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        // Match features using BFMatcher
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches based on distance
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch &a, const cv::DMatch &b)
                  { return a.distance < b.distance; });

        // Keep only the best matches (e.g., top 30%)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.30);
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        // Extract the matched points
        for (const auto &match : goodMatches)
        {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }

        // Draw keypoints on both images
        cv::Mat img_keypoints1, img_keypoints2;
        cv::drawKeypoints(img1, keypoints1, img_keypoints1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
        cv::drawKeypoints(img2, keypoints2, img_keypoints2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

        // Display the images with detected keypoints
        cv::imshow("Keypoints Image B (Edges)", img_keypoints1);      // Image B with keypoints
        cv::imshow("Keypoints Image C (Laser Scan)", img_keypoints2); // Image C with keypoints

        // Draw matches between the two images
        cv::Mat img_matches;
        cv::drawMatches(img1, keypoints1, img2, keypoints2, goodMatches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT);

        // Display the image showing matched features
        cv::imshow("Matched Features between Image B and Image C", img_matches);

        // Allow OpenCV to refresh the GUI
        cv::waitKey(1);
    }

    // Function to propagate the robot using corrected orientation
    void publish_cmd_vel()
    {
        if (initialized_ && amcl_flag)
        {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = (direction_ == "forward") ? linear_speed_ : -linear_speed_;
            cmd_vel.angular.z = angular_speed_;
            cmd_vel_pub_->publish(cmd_vel);
        }

        else
        {
            // Stop the robot if not initialized
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(stop_cmd);
        }
    }

    // Function to rotate the robot in place to avoid obstacles
    void rotate_robot(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;  // Stop forward movement
        cmd_vel.angular.z = 0.5; // Rotate the robot (you can adjust the angular velocity)
        cmd_vel_pub_->publish(cmd_vel);

        // Sleep for a while to allow the robot to rotate (adjust the duration as needed)
        rclcpp::sleep_for(std::chrono::milliseconds(500)); // Rotate for 0.5 seconds

        // Stop the rotation after a brief time
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
    }

    // Function to stop the robot
    void stop_robot()
    {
        if (initialized_)
        {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;  // Stop forward movement
            cmd_vel.angular.z = 0.0; // Rotate the robot (you can adjust the angular velocity)
            cmd_vel_pub_->publish(cmd_vel);

            // Sleep for a while to allow the robot to rotate (adjust the duration as needed)
            rclcpp::sleep_for(std::chrono::milliseconds(500)); // Rotate for 0.5 seconds
        }
    }

    // Function to normalize angle to the range [-pi, pi]
    double normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    // Function to convert occupancy grid to an image (Map Image)
    void occupancyGridToImage(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
    {
        int grid_data;
        unsigned int row, col, val;

        m_temp_img = cv::Mat::zeros(grid->info.height, grid->info.width, CV_8UC1);

        for (row = 0; row < grid->info.height; row++)
        {
            for (col = 0; col < grid->info.width; col++)
            {
                grid_data = grid->data[row * grid->info.width + col];
                if (grid_data != -1)
                {
                    val = 255 - (255 * grid_data) / 100;
                    val = (val == 0) ? 255 : 0;
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = val;
                }
                else
                {
                    m_temp_img.at<uchar>(grid->info.height - row - 1, col) = 0;
                }
            }
        }

        map_scale_ = grid->info.resolution;
        origin_x = grid->info.origin.position.x;
        origin_y = grid->info.origin.position.y;
        size_x = grid->info.width;
        size_y = grid->info.height;

        cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0,
                          0, 1, 0,
                          0, 0, 0);
        cv::erode(m_temp_img, m_MapBinImage, kernel);

        m_MapColImage.create(m_MapBinImage.size(), CV_8UC3);
        cv::cvtColor(m_MapBinImage, m_MapColImage, cv::COLOR_GRAY2BGR);

        RCLCPP_INFO(this->get_logger(), "Occupancy grid map converted to image.");
    }

    // Extract a section of the map around the robot (Image A)
    cv::Mat extract_map_section()
    {
        if (map_.data.empty())
            return cv::Mat(); // Return if no map is available

        // Map parameters
        double resolution = map_.info.resolution;
        int map_width = map_.info.width;
        int map_height = map_.info.height;

        // Robot's current position in the map frame
        int robot_x = static_cast<int>((current_pose_x_ - map_.info.origin.position.x) / resolution);
        int robot_y = static_cast<int>((current_pose_y_ - map_.info.origin.position.y) / resolution);

        RCLCPP_INFO_ONCE(this->get_logger(), "Extracting map section around robot: robot_x=%d, robot_y=%d", robot_x, robot_y);

        // **Increased the size of the extracted map section (200x200)**
        int window_size = 200;
        int start_x = std::max(0, robot_x - window_size / 2);
        int start_y = std::max(0, robot_y - window_size / 2);
        int end_x = std::min(map_width, robot_x + window_size / 2);
        int end_y = std::min(map_height, robot_y + window_size / 2);

        RCLCPP_INFO_ONCE(this->get_logger(), "Map section bounds: start_x=%d, start_y=%d, end_x=%d, end_y=%d", start_x, start_y, end_x, end_y);

        cv::Mat map_section(end_y - start_y, end_x - start_x, CV_8UC1); // Create a larger map section

        for (int i = start_x; i < end_x; ++i)
        {
            for (int j = start_y; j < end_y; ++j)
            {
                int map_value = map_.data[j * map_width + i];
                map_section.at<uchar>(j - start_y, i - start_x) = (map_value == -1) ? 128 : map_value * 255 / 100;
            }
        }

        return map_section;
    }

    // Function to apply Canny edge detection to create Image B from Image A
    cv::Mat apply_canny_edge_detection(const cv::Mat &image)
    {
        cv::Mat edges;
        cv::Canny(image, edges, 100, 200); // Apply Canny edge detection with thresholds
        return edges;
    }

    // Convert laser scan to OpenCV image (Image C)
    cv::Mat laser_scan_to_image(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        int num_ranges = scan_msg->ranges.size();
        cv::Mat scan_image = cv::Mat::zeros(500, 500, CV_8UC1);

        for (int i = 0; i < num_ranges; ++i)
        {
            float range = scan_msg->ranges[i];
            if (std::isfinite(range))
            {
                int x = static_cast<int>(range * cos(scan_msg->angle_min + i * scan_msg->angle_increment) * 50 + 250);
                int y = static_cast<int>(range * sin(scan_msg->angle_min + i * scan_msg->angle_increment) * 50 + 250);
                cv::circle(scan_image, cv::Point(x, y), 1, cv::Scalar(255), -1);
            }
        }

        return scan_image;
    }

    double calculate_rmse(double gt_val, double estimated_val)
    {
        double error = gt_val - estimated_val;
        return error * error;
    }

    void calculate_and_publish_rmse_gt_amcl()
    {
        if (!ground_truth_pose_ || !amcl_pose_)
        {
            RCLCPP_WARN(this->get_logger(), "Pose information not available yet to compute RMSE.");
            return;
        }

        // Calculate RMSE for x, y, and theta between ground truth and AMCL pose
        double rmse_x = calculate_rmse(ground_truth_pose_->x, amcl_pose_->x);
        double rmse_y = calculate_rmse(ground_truth_pose_->y, amcl_pose_->y);

        // Assuming theta is handled separately (you can use atan2 of orientation if needed)
        // Publish RMSE result
        geometry_msgs::msg::PointStamped rmse_msg;
        rmse_msg.header.stamp = this->now();
        rmse_msg.point.x = std::sqrt(rmse_x); // RMSE in x
        rmse_msg.point.y = std::sqrt(rmse_y); // RMSE in y
        rmse_msg.point.z = 0.0;               // Assuming no z-axis evaluation

        rmse_pub_gt_amcl_->publish(rmse_msg);
    }

    void calculate_and_publish_rmse_gt_scanmatching_()
    {
        if (!ground_truth_pose_ || !scan_matching_pose_)
        {
            RCLCPP_WARN(this->get_logger(), "Pose information not available yet to compute RMSE.");
            return;
        }

        // Calculate RMSE for x, y, and theta between ground truth and AMCL pose
        double rmse_x = calculate_rmse(ground_truth_pose_->x, scan_matching_pose_->x);
        double rmse_y = calculate_rmse(ground_truth_pose_->y, scan_matching_pose_->y);

        // Assuming theta is handled separately (you can use atan2 of orientation if needed)
        // Publish RMSE result
        geometry_msgs::msg::PointStamped rmse_msg;
        rmse_msg.header.stamp = this->now();
        rmse_msg.point.x = std::sqrt(rmse_x); // RMSE in x
        rmse_msg.point.y = std::sqrt(rmse_y); // RMSE in y
        rmse_msg.point.z = 0.0;               // Assuming no z-axis evaluation

        rmse_pub_gt_scanmatching_->publish(rmse_msg);
    }

    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr scan_matching_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rmse_pub_gt_amcl_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rmse_pub_gt_scanmatching_;
    rclcpp::TimerBase::SharedPtr timer_;

    double linear_speed_;
    double angular_speed_;
    double distance_;
    std::string direction_;
    bool initialized_;
    bool previous_scan_available_;
    bool match_found_;

    double initial_pose_x_, initial_pose_y_, initial_pose_theta_;
    double current_pose_x_, current_pose_y_, current_pose_theta_;
    std::optional<geometry_msgs::msg::Point> amcl_pose_;
    std::optional<geometry_msgs::msg::Point> ground_truth_pose_;
    std::optional<geometry_msgs::msg::Point> scan_matching_pose_;

    rclcpp::Time last_time_;

    cv::Mat m_temp_img;
    cv::Mat m_MapBinImage;
    cv::Mat m_MapColImage;
    cv::Mat scan_image_, edge_map_;
    cv::Mat map_section_; // Declare map_section_ for storing Image A
    double map_scale_;
    double origin_x;
    double origin_y;
    unsigned int size_x;
    unsigned int size_y;
    bool amcl_flag;

    const std::string WINDOW1 = "Map Image";

    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<> noise_dist_;
    nav_msgs::msg::OccupancyGrid map_; // Store the map data
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanMatchingLocalizer>());
    rclcpp::shutdown();
    return 0;
}

