//
// Created by redwan on 11/10/22.
//

#ifndef CREATE3_CONTROLLER_STATE_ESTIMATOR_H
#define CREATE3_CONTROLLER_STATE_ESTIMATOR_H
#include "COM.h"
#include "utilities/param_manager.h"
#define STATE_DIM 5

using namespace std;
class state_estimator;
typedef shared_ptr<state_estimator> StatePtr;

class state_estimator: public enable_shared_from_this<state_estimator>
{
public:
    state_estimator(const string& param_file)
    {
        state_.resize(STATE_DIM);
        for (int i = 0; i < STATE_DIM; ++i) {
            state_[i] = 0;
        }
        parameters = make_shared<param_manager>(param_file);
    }
    void add_cmd_vel(double v, double w)
    {
        control_queues_.push_back(make_pair(v, w));
    }

    std::pair<double, double> get_cmds()
    {
        auto val = control_queues_.back();
        control_queues_.pop_back();
        return val;
    }

    bool is_cmd_arrived()
    {
        return !control_queues_.empty();
    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        state_[0] = msg->pose.pose.position.x;
        state_[1] = msg->pose.pose.position.y;

        tf2::Quaternion q;
        q.setX(msg->pose.pose.orientation.x);
        q.setY(msg->pose.pose.orientation.y);
        q.setZ(msg->pose.pose.orientation.z);
        q.setW(msg->pose.pose.orientation.w);

        state_[2] = q.getAngle();
        state_[3] = msg->twist.twist.linear.x;
        state_[4] = msg->twist.twist.angular.z;
    }
    StatePtr get_ptr()
    {
        return shared_from_this();
    }

    double at(int index)
    {
        return state_.at(index);
    }

    ParamPtr parameters;
private:
    std::vector<double>state_;
    std::deque<std::pair<double, double>> control_queues_;
};

#endif //CREATE3_CONTROLLER_STATE_ESTIMATOR_H
