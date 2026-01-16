// mobile_robotics_update/src/odometryRB.cpp
#include <chrono>
#include <memory>
#include <string>
#include <mutex>
#include <array>  // <<< ADD THIS: เพิ่ม header สำหรับ std::array

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
// #include "tf2_ros/transform_broadcaster.h"  // <<< CORRECT: ไม่จำเป็นต้องใช้แล้ว

using std::placeholders::_1;
using namespace std::chrono_literals;

class Odom_RB : public rclcpp::Node {
public:
    Odom_RB(): Node("encoder_to_odom_node"), 
      x_(0.0), y_(0.0), theta_(0.0), d_center_(0.0), d_theta_(0.0)
    {
        this->set_parameter(rclcpp::Parameter("use_sim_time", false));
        
        declare_parameter("wheel_radius", 0.0635);
        declare_parameter("wheel_base", 0.4166);

        get_parameter("wheel_radius", wheel_radius_);
        get_parameter("wheel_base", wheel_base_);
        this->declare_parameter("right_wheel_joint", "Right_Wheel_Joint");
        this->declare_parameter("left_wheel_joint", "Left_Wheel_Joint");
        
        get_parameter("right_wheel_joint", right_joint_name);
        get_parameter("left_wheel_joint", left_joint_name);
        
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states",
            20,
            std::bind(&Odom_RB::Odometry_sending, this, std::placeholders::_1)
        );

        // <<< CHANGE 1: เปลี่ยนชื่อ Topic เป็น "odom/raw"
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/raw", 10);
        
        // <<< CHANGE 2: ปิดการทำงานของ TF Broadcaster เพราะ EKF จะทำหน้าที่นี้แทน
        // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        update_timer_ = this->create_wall_timer(
            10ms, std::bind(&Odom_RB::update_odometry, this));
    }

private:
    void Odometry_sending(const sensor_msgs::msg::JointState &msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        auto current_time = this->now();
        
        int left_joint_idx = -1;
        int right_joint_idx = -1;
        
        for(size_t i = 0; i < msg.name.size(); ++i)
        {
            if(msg.name[i] == left_joint_name) left_joint_idx = i;
            if(msg.name[i] == right_joint_name) right_joint_idx = i;
        }
        
        if (left_joint_idx == -1 || right_joint_idx == -1) return;
        
        joint_pos_[0] = msg.position[left_joint_idx] * wheel_radius_;
        joint_pos_[1] = msg.position[right_joint_idx] * wheel_radius_;
        
        // Don't update prev_time_ here, update it in the main loop
    }

    void update_odometry() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        rclcpp::Time current_time = this->now();

        if (!prev_left_initialized_ || !prev_right_initialized_) {
            prev_left_ = joint_pos_[0];
            prev_right_ = joint_pos_[1];
            prev_time_ = current_time; // Initialize prev_time_
            prev_left_initialized_ = true;
            prev_right_initialized_ = true;
            return;
        }
        
        double dt = (current_time - prev_time_).seconds();
        
        if (dt <= 0.0) return;

        double d_left = joint_pos_[0] - prev_left_;
        double d_right = joint_pos_[1] - prev_right_;
        
        d_center_ = (d_left + d_right) / 2.0;
        d_theta_ = (d_right - d_left) / wheel_base_;
        
        x_ += d_center_ * cos(theta_ + d_theta_ / 2.0);
        y_ += d_center_ * sin(theta_ + d_theta_ / 2.0);
        theta_ += d_theta_;

        publish_odometry(current_time, dt);
        
        prev_left_ = joint_pos_[0];
        prev_right_ = joint_pos_[1];
        prev_time_ = current_time;
    }

    void publish_odometry(const rclcpp::Time& current_time, double dt) {
        if (std::isnan(x_) || std::isnan(y_) || std::isnan(theta_)) {
            RCLCPP_ERROR(this->get_logger(), "NaN detected in odometry! Resetting...");
            x_ = y_ = theta_ = 0.0;
            return;
        }
    
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";
        
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.z = sin(theta_ / 2.0);
        odom_msg.pose.pose.orientation.w = cos(theta_ / 2.0);
        
        if (dt > 0.0) {
            odom_msg.twist.twist.linear.x = d_center_ / dt;
            odom_msg.twist.twist.angular.z = d_theta_ / dt;
        } else {
            odom_msg.twist.twist.linear.x = 0.0;
            odom_msg.twist.twist.angular.z = 0.0;
        }
        
        // Setting covariance values
        odom_msg.pose.covariance = {
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        };
        odom_msg.twist.covariance = {
            0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.02
        };
        odom_publisher_->publish(odom_msg);
    }

    // ==============================================================================
    // +++ THIS IS THE MISSING PART - DECLARATION OF MEMBER VARIABLES +++
    // ==============================================================================
    // ROS
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // Correctly commented out
    rclcpp::TimerBase::SharedPtr update_timer_;

    // Parameters
    double wheel_radius_;
    double wheel_base_;
    std::string left_joint_name;
    std::string right_joint_name;
    std::array<double, 2> joint_pos_ = {0.0, 0.0};

    // Odometry state
    double x_;
    double y_;
    double theta_;
    double d_center_;
    double d_theta_;
    double prev_left_;
    double prev_right_;
    rclcpp::Time prev_time_;
    
    bool prev_left_initialized_ = false;
    bool prev_right_initialized_ = false;
    
    std::mutex data_mutex_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Odom_RB>());
    rclcpp::shutdown();
    return 0;
}