#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// ใช้ C++ Class เพื่อสร้าง Node ของเรา
class ImuToTfNode : public rclcpp::Node
{
public:
    // Constructor ของ Class
    ImuToTfNode() : Node("imu_to_tf_broadcaster_cpp")
    {
        // 1. สร้าง Static Transform Broadcaster
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // สร้างข้อความ Static Transform เพื่อประกาศความสัมพันธ์ระหว่าง 'map' -> 'odom'
        geometry_msgs::msg::TransformStamped static_transform;
        static_transform.header.stamp = this->get_clock()->now();
        static_transform.header.frame_id = "map";
        static_transform.child_frame_id = "odom";
        static_transform.transform.translation.x = 0.0;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.0;
        static_transform.transform.rotation.w = 1.0;
        
        // ส่ง Static Transform ออกไป
        static_broadcaster_->sendTransform(static_transform);
        RCLCPP_INFO(this->get_logger(), "Published static transform from 'map' to 'odom'.");

        // 2. สร้าง Dynamic Transform Broadcaster
        dynamic_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 3. ตั้งค่า QoS Profile ให้เป็น Best Effort เพื่อให้ตรงกับฝั่ง micro-ROS
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        // 4. สร้าง Subscriber เพื่อรับข้อมูลจาก /imu/data_raw
        // ใช้ std::bind เพื่อบอกให้เรียกใช้ imu_callback function ของ object นี้
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw", 
            qos_profile, 
            std::bind(&ImuToTfNode::imu_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "IMU to TF broadcaster node (C++) is running...");
    }

private:
    // ฟังก์ชัน Callback ที่จะถูกเรียกเมื่อมีข้อมูลใหม่เข้ามา
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped t;

        // ตั้งค่า Header
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "imu_link";

        // คัดลอกข้อมูลทิศทาง (Orientation) จาก IMU มาใส่โดยตรง
        t.transform.rotation = msg->orientation;
        // การเคลื่อนที่ (Translation) เป็น 0
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;

        // ส่ง Dynamic Transform ออกไป
        dynamic_broadcaster_->sendTransform(t);
    }

    // ตัวแปร Member ของ Class
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

// ฟังก์ชัน main มาตรฐานของ C++ ROS2 Node
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuToTfNode>());
    rclcpp::shutdown();
    return 0;
}