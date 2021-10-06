#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "lgsvl_msgs/msg/vehicle_control_data.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
double frequency = 20; //hertz
double prev_error, integrated_error, P_gain_velocity=0.3, D_gain_velocity=0.1, I_gain_velocity=0.045;
double integration_limit = 1.4;
double target_velocity = 25;
double target_thrust = 0;
nav_msgs::msg::Odometry vehicle_state;


double thrust_generator (nav_msgs::msg::Odometry vehicle_pose, double target_velocity){
    double error = target_velocity - vehicle_pose.twist.twist.linear.x;
    double d_error = frequency * (error-prev_error);
    integrated_error += error / frequency;

    if (integrated_error > integration_limit){
        integrated_error = integration_limit;
    } else if(integrated_error < -integration_limit){
        integrated_error = - integration_limit;
    }

    target_thrust =  (P_gain_velocity * error + D_gain_velocity * d_error + I_gain_velocity * integrated_error);

    if(target_thrust > 1){
        target_thrust = 1;
    }
    else if (target_thrust < -1){
        target_thrust = -1;
    }

    return target_thrust;
}


class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
            : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<lgsvl_msgs::msg::VehicleControlData>("/lgsvl/vehicle_control_cmd", 10);
        timer_ = this->create_wall_timer(
                50ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = lgsvl_msgs::msg::VehicleControlData();

        double  thrust = thrust_generator(vehicle_state, target_velocity);
        message.target_gear = 1;
        if(thrust > 0){
            message.acceleration_pct = thrust;
            message.braking_pct = 0;
        }
        else{
            message.acceleration_pct = 0;
            message.braking_pct = -thrust;
        }
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr publisher_;
    size_t count_;
};


class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
            : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/lgsvl/gnss_odom", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) const
    {
        vehicle_state = * msg;
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto node1 = std::make_shared<MinimalSubscriber>();
    auto node2 = std::make_shared<MinimalPublisher>();
    exec.add_node(node1);
    exec.add_node(node2);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
