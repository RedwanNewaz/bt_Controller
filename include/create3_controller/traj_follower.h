//
// Created by redwan on 11/10/22.
//

#ifndef CREATE3_CONTROLLER_TRAJ_FOLLOWER_H
#define CREATE3_CONTROLLER_TRAJ_FOLLOWER_H
#include "create3_controller/state_estimator.h"
#define DEBUG(x) std::cout << x << std::endl
#define STATE_DIM 5



class TrajFollower:public rclcpp::Node
{
public:

    TrajFollower(StatePtr stateEstimator):stateEstimator_(stateEstimator),Node("create3_controller")
    {


        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(
                &TrajFollower::odom_callback, this, std::placeholders::_1)
                );
        cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",1);
        timer_ = this->create_wall_timer(
                3ms, std::bind(&TrajFollower::timer_callback, this));

    }
private:
    void timer_callback()
    {
        if(stateEstimator_->is_cmd_arrived())
        {
            auto cmd = stateEstimator_->get_cmds();
            publish_cmd(cmd.first, cmd.second);
        }
    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
//        RCLCPP_INFO(this->get_logger(), "odom message received");
        stateEstimator_->odom_callback(msg);
    }

    void publish_cmd(double v, double w)
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = v;
        cmd_vel.angular.z = w;
        cmd_vel_->publish(cmd_vel);
//        RCLCPP_INFO(this->get_logger(), "%lf, %lf", v, w);
    }

    StatePtr stateEstimator_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
private:

};


#endif //CREATE3_CONTROLLER_TRAJ_FOLLOWER_H
