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
#include <visualization_msgs/msg/marker.hpp>
#include "EKF.h"
//https://docs.ros.org/en/foxy/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html
//ros2 run tf2_ros static_transform_publisher 0 0 0  0 0 0 map odom

using namespace  std::chrono_literals;
const auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

enum DATA_TYPE{
    ODOM = 0, 
    APRILTAG,
    CMD_VEL
};

class StateViz:public rclcpp::Node{
    enum COLOR{
        RED,
        GREEN,
        DEFAULT
    };
    std::unordered_map<int, std::vector<geometry_msgs::msg::Point>> pubData_;
public:
    StateViz(const std::string& nodeName):Node(nodeName)
    {
        create3_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("ekf/apriltag", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
            state_callback(msg, RED);});

        create3_state_sub2_ = this->create_subscription<nav_msgs::msg::Odometry>("ekf/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
            state_callback(msg, DEFAULT);});

        create3_state_sub3_ = this->create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
            state_callback(msg, GREEN);});

        create3_state_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("ekf/apriltag/viz", 10);
        timer_ = this->create_wall_timer(1s, [this] { publish_traj(); });
    }
protected:
    void state_callback(nav_msgs::msg::Odometry::SharedPtr msg, const COLOR& color)
    {
        // convert odom to viz marker
        visualization_msgs::msg::Marker marker;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.header.stamp  = get_clock()->now();
        marker.header.frame_id  = "map";

        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.mesh_resource = "package://irobot_create_description/meshes/body_visual.dae";
        marker.id = static_cast<int>(color);
        marker.ns = "robot";
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
        switch (color) {
            case RED: marker.color.r = 1; break;
            case GREEN: marker.color.g = 1; break;
            default:
                marker.color.r = marker.color.g  = marker.color.b = 0.66;

        }
        marker.color.a = 0.85;
        marker.pose = msg->pose.pose;
        create3_state_pub_->publish(marker);
        // show traj
        pubData_[color].push_back(marker.pose.position);

    }

    void publish_traj()
    {
        auto color = DEFAULT;
        if(pubData_[color].empty())
            return;

        visualization_msgs::msg::Marker trajMarker;
        std::copy(pubData_[color].begin(), pubData_[color].end(), std::back_inserter(trajMarker.points));
        trajMarker.id = 202;
        trajMarker.action = visualization_msgs::msg::Marker::ADD;
        trajMarker.header.stamp = get_clock()->now();
        trajMarker.header.frame_id = "map";

        trajMarker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        trajMarker.ns = "traj";
        trajMarker.color.r = 1;
        trajMarker.color.a = 0.7;
        trajMarker.scale.x = trajMarker.scale.y = trajMarker.scale.z = 0.1;
        create3_state_pub_->publish(trajMarker);

    }
private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr create3_state_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr create3_state_sub_, create3_state_sub2_, create3_state_sub3_;
    rclcpp::TimerBase::SharedPtr timer_;
};

struct FusedData{
    FusedData()
    {
        updateStatus[ODOM] = updateStatus[APRILTAG] = updateStatus[CMD_VEL] = false; 
    }
    tf2::Transform odom;
    tf2::Transform apriltag;
    geometry_msgs::msg::Twist cmd, odomTwsit;
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

    void update(const tf2::Transform& obs, tf2::Transform& res)
    {
        // convert tf to vector
        auto origin = obs.getOrigin();
        auto theta = obs.getRotation().getAngle();
        std::vector<double> input{origin.x(), origin.y(), origin.z(), theta};
        std::vector<double> result;
        update(input, result);
        res.setOrigin(tf2::Vector3(result[0], result[1], result[2]));
        tf2::Quaternion q;
        q.setRPY(0, 0, result[3]);
        res.setRotation(q);


    }
private:
    double m_alpha;
    std::vector<double>X_;
    bool init_;
};

class StateEstimatorBase: public rclcpp::Node
{
public:
    StateEstimatorBase(const std::string &nodeName) : Node(nodeName)
    {
        intensity_sub_ = this->create_subscription<irobot_create_msgs::msg::IrIntensityVector>("ir_intensity", qos, std::bind(
                &StateEstimatorBase::intensity_callback, this, std::placeholders::_1)
        );
        ir_values_.resize(7);
        // Call on_timer function every second
        filter_ = std::make_unique<ComplementaryFilter>(0.99);
        timer_ = this->create_wall_timer(15ms, [this] { timer_callback(); });

    }

    double safetyProb()
    {
        const double mu = 10.0;
        const double std = 2.0;
        double x = std::min(maxIrValue(), mu ) - 1e-6;
        const double normFactor = 1.0 / std * sqrt(2 * M_PI);
        double collisionProb = normFactor * exp(-0.5 * (mu - x) / std);
        return 1.0 - collisionProb;
    }

protected:
    virtual void lookupTransform() = 0;
    virtual void sensorFusion() = 0;

    void timer_callback()
    {
        lookupTransform();
        sensorFusion();
        double safe = safetyProb();
        if(safe <= 0.5)
            RCLCPP_INFO(get_logger(), "[safety ]: prob = %lf", safe);
    }
    double maxIrValue()
    {
        if(ir_values_.empty())
            return 0;
        return *std::max_element(ir_values_.begin(), ir_values_.end());
    }
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
    rclcpp::TimerBase::SharedPtr timer_;


};


class JointStateEstimator: public StateEstimatorBase
{
public:
    JointStateEstimator(const std::string &nodeName) : StateEstimatorBase(nodeName) {

        // collect all the sensor and control information: (i) odom (ii) apriltag tf, and (iii) cmd_vel
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", qos, std::bind(
                &JointStateEstimator::odom_callback, this, std::placeholders::_1)
        );
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", qos   , std::bind(
                &JointStateEstimator::cmd_callback, this, std::placeholders::_1)
        );
        // prepare for transformation
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // save them as tf::transform format
        fusedData_ = std::make_unique<FusedData>();
        // ekf odom
        ekf_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("ekf/odom", 10);
        ekf_apriltag_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("ekf/apriltag", 10);
        RCLCPP_INFO(this->get_logger(), "Joint State Estimator Initialized!!");
        ekf_ = std::make_unique<EKF>(1.0 / 100); // 20 Hz

    }
    ~JointStateEstimator()
    {
        delete odomInit_;
        delete apriltagInit_;
    }
protected:
    void tf_to_odom(const tf2::Transform& t, nav_msgs::msg::Odometry& odom)
    {
        odom.header.frame_id = "odom";
        auto pos = t.getOrigin();
        auto ori = t.getRotation();
        odom.header.stamp = get_clock()->now();

        odom.pose.pose.position.x = pos.x();
        odom.pose.pose.position.y = pos.y();
        odom.pose.pose.position.z = pos.z();

        odom.pose.pose.orientation.x = ori.x();
        odom.pose.pose.orientation.y = ori.y();
        odom.pose.pose.orientation.z = ori.z();
        odom.pose.pose.orientation.w = ori.w();

        odom.twist.twist = fusedData_->cmd;
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
        fusedData_->odomTwsit = msg->twist.twist;
        fusedData_->updateStatus[ODOM] = true;
    }

    void cmd_callback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        fusedData_->cmd = *msg.get();
        fusedData_->updateStatus[CMD_VEL] = true;
    }

    void lookupTransform() override
    {


        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform(
                    fromFrameRel, toFrameRel,
                    tf2::TimePointZero);
            fusedData_->apriltag.setOrigin(tf2::Vector3(-t.transform.translation.y,
                                                        -t.transform.translation.x,
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
                odomInit_ = new tf2::Transform(fusedData_->odom);
                apriltagInit_ = new tf2::Transform(fusedData_->apriltag);
            });

        }

    }


    void sensorFusion() override
    {
        if(odomInit_ == nullptr)
            return;


        auto q = fusedData_->odom.getRotation();
        fusedData_->apriltag.setRotation(q);
        tf2::Transform OdomFilter;
        ekf_->update(fusedData_->apriltag, fusedData_->odomTwsit, OdomFilter);
        // consider control input
        ekf_->update(fusedData_->apriltag, fusedData_->cmd, OdomFilter);
        fusedData_->apriltag = OdomFilter;

        auto newAngle = OdomFilter.getRotation();
        double finalAngle = fmod(newAngle.getAngle() + 3 * M_PI_2, 2 * M_PI);
//        finalAngle = std::min(2 * M_PI - finalAngle, finalAngle);
        newAngle.setRPY(0, 0, finalAngle);
        OdomFilter.setRotation(newAngle);



        // convert to odom message
        nav_msgs::msg::Odometry msg1;
        tf_to_odom(OdomFilter, msg1);
        ekf_odom_pub_->publish(msg1);

        // reset
        if(!std::count(fusedData_->updateStatus.begin(), fusedData_->updateStatus.end(), false))
        {
            fusedData_ = std::make_unique<FusedData>();
        }

    }



private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    ///@brief variables related to tf listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<FusedData> fusedData_;
    const std::string fromFrameRel = "camera";
    const std::string toFrameRel = "tag36h11:7";
    tf2::Transform *odomInit_, *apriltagInit_;
    std::once_flag flagOdom_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_pub_, ekf_apriltag_pub_;


    std::unique_ptr<EKF> ekf_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto stateEstimator = std::make_shared<JointStateEstimator> ("stateEstimator");
    auto stateViz = std::make_shared<StateViz>("stateViz");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(stateEstimator);
    executor.add_node(stateViz);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}