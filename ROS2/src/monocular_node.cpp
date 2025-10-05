/**
 * ROS2 Monocular Node for ORB-SLAM3
 * 
 * Subscribes to camera images and publishes:
 * - Camera pose (geometry_msgs/PoseStamped)
 * - Camera trajectory path (nav_msgs/Path)
 * - TF transforms (camera pose in world frame)
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.h>

#include "System.h"

using namespace std;

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(const std::string& vocabulary_path, 
                      const std::string& settings_path,
                      bool use_viewer = true)
    : Node("orb_slam3_mono")
    {
        // Declare and get ROS parameters
        this->declare_parameter<std::string>("image_topic", "camera/image_raw");
        this->declare_parameter<std::string>("pose_topic", "orb_slam3/camera_pose");
        this->declare_parameter<std::string>("path_topic", "orb_slam3/camera_path");
        this->declare_parameter<std::string>("world_frame_id", "world");
        this->declare_parameter<std::string>("camera_frame_id", "camera");
        this->declare_parameter<int>("queue_size", 10);
        this->declare_parameter<bool>("publish_tf", true);
        
        std::string image_topic = this->get_parameter("image_topic").as_string();
        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        std::string path_topic = this->get_parameter("path_topic").as_string();
        world_frame_id_ = this->get_parameter("world_frame_id").as_string();
        camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
        int queue_size = this->get_parameter("queue_size").as_int();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();

        RCLCPP_INFO(this->get_logger(), "=== ROS2 Parameters ===");
        RCLCPP_INFO(this->get_logger(), "Image topic: %s", image_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Pose topic: %s", pose_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Path topic: %s", path_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "World frame: %s", world_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "Camera frame: %s", camera_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publish TF: %s", publish_tf_ ? "true" : "false");

        // Initialize ORB-SLAM3
        RCLCPP_INFO(this->get_logger(), "Initializing ORB-SLAM3...");
        slam_system_ = std::make_shared<ORB_SLAM3::System>(
            vocabulary_path,
            settings_path,
            ORB_SLAM3::System::MONOCULAR,
            use_viewer
        );
        RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 initialized!");

        // Create subscriber for image
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic,
            queue_size,
            std::bind(&MonocularSlamNode::imageCallback, this, std::placeholders::_1)
        );

        // Create publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, queue_size);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, queue_size);

        // TF broadcaster
        if (publish_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }

        // Initialize path message
        path_msg_.header.frame_id = world_frame_id_;

        RCLCPP_INFO(this->get_logger(), "Monocular SLAM node ready!");
    }

    ~MonocularSlamNode()
    {
        if (slam_system_) {
            RCLCPP_INFO(this->get_logger(), "Shutting down ORB-SLAM3...");
            slam_system_->Shutdown();
            
            // Save trajectory
            slam_system_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
            slam_system_->SaveTrajectoryTUM("CameraTrajectory.txt");
            RCLCPP_INFO(this->get_logger(), "Trajectories saved!");
        }
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Get timestamp
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        // Track frame
        cv::Mat Tcw = slam_system_->TrackMonocular(cv_ptr->image, timestamp);

        // Publish pose if tracking successful
        if (!Tcw.empty()) {
            publishPose(Tcw, msg->header.stamp);
        }
    }

    void publishPose(const cv::Mat& Tcw, const builtin_interfaces::msg::Time& timestamp)
    {
        // Convert camera pose to world frame (invert transformation)
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc * Tcw.rowRange(0,3).col(3);

        // Create pose message
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = timestamp;
        pose_msg.header.frame_id = world_frame_id_;

        // Position
        pose_msg.pose.position.x = twc.at<float>(0);
        pose_msg.pose.position.y = twc.at<float>(1);
        pose_msg.pose.position.z = twc.at<float>(2);

        // Orientation (convert rotation matrix to quaternion)
        Eigen::Matrix3f rotation_matrix;
        rotation_matrix << Rwc.at<float>(0,0), Rwc.at<float>(0,1), Rwc.at<float>(0,2),
                          Rwc.at<float>(1,0), Rwc.at<float>(1,1), Rwc.at<float>(1,2),
                          Rwc.at<float>(2,0), Rwc.at<float>(2,1), Rwc.at<float>(2,2);
        
        Eigen::Quaternionf q(rotation_matrix);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        // Publish pose
        pose_pub_->publish(pose_msg);

        // Add to path
        path_msg_.header.stamp = timestamp;
        path_msg_.poses.push_back(pose_msg);
        path_pub_->publish(path_msg_);

        // Publish TF
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = timestamp;
            transform.header.frame_id = world_frame_id_;
            transform.child_frame_id = camera_frame_id_;
            transform.transform.translation.x = twc.at<float>(0);
            transform.transform.translation.y = twc.at<float>(1);
            transform.transform.translation.z = twc.at<float>(2);
            transform.transform.rotation = pose_msg.pose.orientation;
            
            tf_broadcaster_->sendTransform(transform);
        }
    }

    std::shared_ptr<ORB_SLAM3::System> slam_system_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Path path_msg_;
    
    // Parameters
    std::string world_frame_id_;
    std::string camera_frame_id_;
    bool publish_tf_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                     "Usage: ros2 run orb_slam3_ros2 mono <vocabulary> <settings> [use_viewer]");
        return 1;
    }

    std::string vocabulary_path = argv[1];
    std::string settings_path = argv[2];
    bool use_viewer = true;
    
    if (argc >= 4) {
        use_viewer = (std::string(argv[3]) == "true");
    }

    auto node = std::make_shared<MonocularSlamNode>(vocabulary_path, settings_path, use_viewer);
    
    RCLCPP_INFO(node->get_logger(), "Starting monocular SLAM node...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
