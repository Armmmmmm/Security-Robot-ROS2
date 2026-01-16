// mobile_robotics_update/src/joint_nodeRB.cpp
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/qos.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <thread>
#include<memory>
#include<chrono>
#include<string>
#include<vector>
#include<functional>


using namespace std::chrono_literals;

#define PI_M 3.14285
class Joint_states_RB : public rclcpp::Node
{
  public:
      Joint_states_RB() : Node("joint_state_publisher"),last_time_(this->now())
      {
        this->declare_parameter("pulish_rate",50.0); // HZ unit
        this->declare_parameter("joint_names", std::vector<std::string>{"joint1", "joint2"});
        this->declare_parameter("position_encode", std::vector<double>{0,0});
        
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);  // เปลี่ยนเป็น BestEffort
        qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
        
        Joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
        status_publish = this->create_publisher<std_msgs::msg::Int32>("status_node",10);
        encoderleft = this->create_subscription<std_msgs::msg::Int32>(
            "PULSEL",
            qos_profile,
            [this](const std_msgs::msg::Int32::SharedPtr msg){
              // positions_[0] = ((0.127*PI_M/810)*msg->data);
              positions_[0] = (static_cast<double>(msg->data) / 810.0) * (2.0 * M_PI);
              // RCLCPP_INFO(this->get_logger(), "pos left : %f",positions_[0]);
            });
        encoderright = this->create_subscription<std_msgs::msg::Int32>(
            "PULSER",qos_profile,[this](const std_msgs::msg::Int32::SharedPtr msg){
              // positions_[1] = ((0.127*PI_M/810)*msg->data);
              positions_[1] = (static_cast<double>(msg->data) / 810.0) * (2.0 * M_PI);
              // RCLCPP_INFO(this->get_logger(), "pos right : %f",positions_[1]);

            });
        
        timer_ = this->create_wall_timer(50ms,std::bind(&Joint_states_RB::timer_call_back,this)); 
        positions_.resize(2);
        // 0.05 s = 20Hz
      }


  private:
      /*
        write the process code on timer_ball_bck because in that side they can 
        control frequency(HZ, pulse cycle), Not stop blocking event
        
      */
      void timer_call_back()
      {
          auto current_time = this->now();
          double delta_time = (current_time - last_time_).seconds();
              
          if(delta_time > 0.0)
          {
              // RCLCPP_INFO(
              //   this->get_logger(), "omega left : %.3f omega right : %.3f", 
              //   (positions_[0] - last_pos[0])/delta_time,(positions_[1] - last_pos[1])/delta_time
              // );
          }


          auto messager = sensor_msgs::msg::JointState();
          messager.header.stamp = current_time;
          messager.name = joints_name_;
          messager.position = positions_;
          // messager.velocity = {(positions_[0] - last_pos[0])/delta_time, (positions_[1] - last_pos[1])/delta_time};

          Joint_publisher_-> publish(messager);
          last_pos = positions_;
          last_time_ = current_time;
       }

    
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr Joint_publisher_;
      rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_publish;
      rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr encoderleft;
      rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr encoderright;
      rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr encoder_sub_;

      rclcpp::TimerBase::SharedPtr timer_;      
      rclcpp::Time last_time_;
      std::vector<std::string> joints_name_{"Left_Wheel_Joint", "Right_Wheel_Joint"};
      std::vector<double> positions_;
      std::vector<double> last_pos{0.0 , 0.0};



};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Joint_states_RB>());
  rclcpp::shutdown();
  return 0;
}
