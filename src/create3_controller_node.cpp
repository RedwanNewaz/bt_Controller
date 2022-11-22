#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "create3_controller/state_estimator.h"
#include "utilities/viz_objects.h"
#include "create3_controller/dwa_planner.h"
#include <mutex>

class DWA: public rclcpp::Node{
public:
    DWA(StatePtr stateEstimator):Node("DWA"), stateEstimator_(stateEstimator)
    {
        initialized_ = false;
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(
                &DWA::odom_callback, this, std::placeholders::_1)
        );

        rviz_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(
                &DWA::rviz_callback, this, std::placeholders::_1)
        );
        cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",1);
        timer_ = this->create_wall_timer(
                30ms, std::bind(&DWA::timer_callback, this));
        control_[0] = control_[1] = 0;
        std::function<double(const string&)> f = [&](const string& param)
        {return stateEstimator_->parameters->get_param<double>(param);};
        config_.update_param(f);
        traj_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/short_traj", 10);
        obs_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/obstacles", 10);

        obs_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/obstacles", 10, [&](const geometry_msgs::msg::PoseArray::SharedPtr msg)
        {return obscacle_callback(msg);});

        stateEstimator_->parameters->get_obstacles(obstacles_);

    }
private:
    bool initialized_;
    StatePtr stateEstimator_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr traj_pub_, obs_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obs_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_;
    DynamicWindow::planner planner_;
    DynamicWindow::Config config_;
    Control control_;
    Obstacle obstacles_;
    tf2::Transform goal_pose_;
    std::mutex mu_;
    std::once_flag obs_flag_;

    void timer_callback()
    {
        if(!initialized_)
            return;


        auto goal_position = goal_pose_.getOrigin();
        tf2::Transform current_pose = stateEstimator_->getCurrentPose();
        auto curr_position = current_pose.getOrigin();
        tf2::Vector3 position_diff = goal_position - curr_position;
        double current_angle = tf2::getYaw(current_pose.getRotation());

        double remainDist = sqrt(pow(position_diff.x(), 2) + pow(position_diff.y(), 2) );
        Point goal;
        goal[0] = goal_position.x();
        goal[1] = goal_position.y();


        if (remainDist < config_.goal_radius)
        {
            initialized_ = false;
            control_[0] = control_[1] = 0;
        }
        else
        {
            State x({{curr_position.x(), curr_position.y(), current_angle, control_[0], control_[1]}});
            Traj ltraj = planner_.compute_control(x, control_, config_, goal, obstacles_);
            publish_short_horizon_traj(ltraj);
        }

//        RCLCPP_INFO(this->get_logger(),"[remaining = %lf] v = %lf | w = %lf", remainDist, u[0], u[1]);
        publish_cmd(control_[0], control_[1]);
        stateEstimator_->add_cmd_vel(control_[0], control_[1]);

    }

    template<class T>
    void publish_short_horizon_traj(T& traj)
    {
        geometry_msgs::msg::PoseArray msg;
        msg.header.stamp = this->get_clock()->now();
        for(auto & p: traj)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = p[0];
            pose.position.y = p[1];
            pose.position.z = 0;
            msg.poses.push_back(pose);
        }
        traj_pub_->publish(msg);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        const std::lock_guard<mutex> lk(mu_);
        stateEstimator_->odom_callback(msg);
        std::call_once(obs_flag_, [&](){ publish_obstacles(); });

    }

    void publish_obstacles()
    {
        geometry_msgs::msg::PoseArray msg;
        msg.header.stamp = this->get_clock()->now();

        for(auto &item:obstacles_)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = item[0];
            pose.position.y = item[1];
            pose.position.z = 0;
            msg.poses.push_back(pose);
        }
        obs_pub_->publish(msg);
    }

    void rviz_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        const std::lock_guard<mutex> lk(mu_);
        goal_pose_ = state_estimator::poseToTransform(msg);
        initialized_ = true;
        publish_obstacles();
    }

    void obscacle_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        const std::lock_guard<mutex> lk(mu_);
        obstacles_.clear();
        for(auto& pose: msg->poses)
        {
            obstacles_.push_back({pose.position.x, pose.position.y});
        }

    }

    void publish_cmd(double v, double w)
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = v;
        cmd_vel.angular.z = w;
        cmd_vel_->publish(cmd_vel);
//        RCLCPP_INFO(this->get_logger(), "%lf, %lf", v, w);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::string param =  (argc > 1) ? argv[1] : "";
    StatePtr stateEstimator = make_shared<state_estimator>(param);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node1 = std::make_shared<DWA>(stateEstimator->get_ptr());
    auto node2 = std::make_shared<viz_objects>();
    executor.add_node(node1);
    executor.add_node(node2);
    executor.spin();
    rclcpp::shutdown();

  return 0;
}
