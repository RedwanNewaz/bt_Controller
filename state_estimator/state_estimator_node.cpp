//
// Created by airlab on 3/8/23.
//
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/convert.h"
#include <irobot_create_msgs/msg/ir_intensity_vector.hpp>
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>

//https://docs.ros.org/en/foxy/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html

using namespace  std::chrono_literals;

enum DATA_TYPE{
    ODOM = 0, 
    APRILTAG,
    CMD_VEL
};

struct FusedData{
    FusedData()
    {
        updateStatus[ODOM] = updateStatus[APRILTAG] = updateStatus[CMD_VEL] = false; 
    }
    tf2::Transform odom;
    tf2::Transform apriltag;
    geometry_msgs::msg::Twist cmd;
    std::array<bool, 3> updateStatus;
};


/// @brief The ComplementaryFilter.h class implements the complmentary filter where it initializes the state and updates it using the filter's alpha paramater in update() method.
class ComplementaryFilter{
public:
    ComplementaryFilter(double alpha): m_alpha(alpha)
    {
        init_ = false;
    }

    void init(const std::vector<double>& X0)
    {
        X_.clear();
        std::copy(X0.begin(), X0.end(), std::back_inserter(X_));
    }
    void update(const std::vector<double>& obs, std::vector<double>& result)
    {
        if(!init_)
        {
            init(obs);
            init_ = true;
        }
        for (int i = 0; i < obs.size(); ++i) {
            X_[i] = m_alpha * X_[i] + (1 - m_alpha) * obs[i];
        }
        if(result.empty())
            std::copy(X_.begin(), X_.end(),std::back_inserter(result));
        else
            std::copy(X_.begin(), X_.end(),result.begin());
    }
private:
    double m_alpha;
    std::vector<double>X_;
    bool init_;
};

class RobotSafety:public rclcpp::Node
{
public:
    RobotSafety(const std::string &nodeName) : Node(nodeName)
    {
        intensity_sub_ = this->create_subscription<irobot_create_msgs::msg::IrIntensityVector>("ir_intensity", 10, std::bind(
                &RobotSafety::intensity_callback, this, std::placeholders::_1)
        );
        ir_values_.resize(7);
        filter_ = std::make_unique<ComplementaryFilter>(0.99);
    }

    double maxIrValue()
    {
        if(ir_values_.empty())
            return 0;
        return *std::max_element(ir_values_.begin(), ir_values_.end());
    }
protected:
    void intensity_callback(irobot_create_msgs::msg::IrIntensityVector::SharedPtr msg)
    {
        std::vector<double> mes(7);
        for (int i = 0; i < 7; ++i) {
            mes[i] = msg->readings[i].value;
        }
        filter_->update(mes, ir_values_);
    }
private:
    rclcpp::Subscription<irobot_create_msgs::msg::IrIntensityVector>::SharedPtr intensity_sub_;
    std::vector<double> ir_values_;
    std::unique_ptr<ComplementaryFilter> filter_;


};


class JointStateEstimator: public RobotSafety
{
public:
    JointStateEstimator(const std::string &nodeName) : RobotSafety(nodeName) {

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        // collect all the sensor and control information: (i) odom (ii) apriltag tf, and (iii) cmd_vel
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", qos, std::bind(
                &JointStateEstimator::odom_callback, this, std::placeholders::_1)
        );
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(
                &JointStateEstimator::cmd_callback, this, std::placeholders::_1)
        );
        // prepare for transformation
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // Call on_timer function every second
        timer_ = this->create_wall_timer(15ms, [this] { timer_callback(); });
        // save them as tf::transform format
        fusedData_ = std::make_unique<FusedData>();
        // ekf odom
        ekf_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("ekf/odom", 10);
        ekf_apriltag_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("ekf/apriltag", 10);
        RCLCPP_INFO(this->get_logger(), "Joint State Estimator Initialized!!");

    }
    ~JointStateEstimator()
    {
        delete initOdom_;
    }
protected:
    void tf_to_odom(const tf2::Transform& t, nav_msgs::msg::Odometry& odom)
    {
        odom.header.frame_id = "map";
        auto pos = t.getOrigin();
        auto ori = t.getRotation();

        odom.pose.pose.position.x = pos.x();
        odom.pose.pose.position.y = pos.y();
        odom.pose.pose.position.z = pos.z();

        odom.pose.pose.orientation.x = ori.x();
        odom.pose.pose.orientation.y = ori.y();
        odom.pose.pose.orientation.z = ori.z();
        odom.pose.pose.orientation.w = ori.w();
    }

    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        fusedData_->odom.setOrigin(tf2::Vector3(msg->pose.pose.position.x,
                                               msg->pose.pose.position.y,
                                               msg->pose.pose.position.z
                                               ));
        fusedData_->odom.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x,
                                                    msg->pose.pose.orientation.y,
                                                    msg->pose.pose.orientation.z,
                                                    msg->pose.pose.orientation.w
                                                    ));
        fusedData_->updateStatus[ODOM] = true;
    }

    void cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        fusedData_->cmd = *msg.get();
        fusedData_->updateStatus[CMD_VEL] = true;
    }

    void timer_callback()
    {
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform(
                    fromFrameRel, toFrameRel,
                    tf2::TimePointZero);
            fusedData_->apriltag.setOrigin(tf2::Vector3(t.transform.translation.x,
                                                        t.transform.translation.y,
                                                        t.transform.translation.z
            ));
            fusedData_->apriltag.setRotation(
                    tf2::Quaternion(
                            t.transform.rotation.x,
                            t.transform.rotation.y,
                            t.transform.rotation.z,
                            t.transform.rotation.w)
            );

            fusedData_->updateStatus[APRILTAG] = true;
//            RCLCPP_INFO(this->get_logger(), "Found transform %s to %s",  toFrameRel.c_str(), fromFrameRel.c_str());
        } catch (const tf2::TransformException & ex) {
//            RCLCPP_INFO(
//                    this->get_logger(), "Could not transform %s to %s: %s",
//                    toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
//            return;
        }

        // generate consensus using sensor fusion algorithm
        // check if all sensors are updated or not: there is no false in the array
        if(!std::count(fusedData_->updateStatus.begin(), fusedData_->updateStatus.end(), false))
        {
            // TODO should this update need to be done frequently or once?
            std::call_once(flagOdom_, [&](){
                initOdom_ = new tf2::Transform(fusedData_->odom);
                // use apriltag position as reference
                initOdom_->setOrigin(fusedData_->apriltag.getOrigin());
            });
            // reset the fused data
             fusedData_ = std::make_unique<FusedData>();
        }
        sensorFusion();
    }

    void print(const tf2::Transform& t, const char *name)
    {
        auto pos = t.getOrigin();
        auto yaw = t.getRotation().getAngle();
        RCLCPP_INFO(this->get_logger(), "[%s] = state (%lf, %lf, %lf)", name, pos.x(), pos.y(), yaw);
    }

    void sensorFusion()
    {
        if(initOdom_ == nullptr)
            return;
        // do some computation here
        if(fusedData_->updateStatus[ODOM])
        {
            auto Odom = fusedData_->odom.inverseTimes(*initOdom_);
            print(Odom, "Odom");
            nav_msgs::msg::Odometry msg;
            tf_to_odom(Odom, msg);
            ekf_odom_pub_->publish(msg);
        }

        if(fusedData_->updateStatus[APRILTAG])
        {
            auto Apriltag = fusedData_->apriltag;
            // check odom initialized or not
            // use odom rotation for apriltag
            Apriltag.setRotation(fusedData_->odom.getRotation());
            print(Apriltag, "Apriltag");
            nav_msgs::msg::Odometry msg;
            tf_to_odom(Apriltag, msg);
            ekf_apriltag_pub_->publish(msg);
        }

    }



private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    ///@brief variables related to tf listener
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<FusedData> fusedData_;
    const std::string fromFrameRel = "camera";
    const std::string toFrameRel = "tag36h11:7";
    tf2::Transform *initOdom_;
    std::once_flag flagOdom_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_pub_, ekf_apriltag_pub_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto stateEstimator = std::make_shared<JointStateEstimator> ("stateEstimator");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(stateEstimator);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}