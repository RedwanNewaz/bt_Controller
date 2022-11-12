//
// Created by Abdullah Al Redwan Newaz on 11/10/22.
//

#ifndef BT_CONTROLLER_POSITION_CONTROLLER_H
#define BT_CONTROLLER_POSITION_CONTROLLER_H
#include "gen_waypoints.h"
#include "pid.h"
#include "create3_controller/state_estimator.h"
/**
 * @brief control robot 2D position. This node will execute after orientation controller.
 * The robot will make progress toward its destination
 */
class PositionController : public ThreadedAction, public PID
{
public:
    PositionController(const std::string& name, const NodeConfig& config, StatePtr stateEstimator) :
            ThreadedAction(name, config), stateEstimator_(stateEstimator)
    {
        x_ = stateEstimator_->at(0);
        y_ = stateEstimator_->at(1);


        auto ori_stability = stateEstimator->parameters->get_orientation_stability();
        ori_thres_ = ori_stability.deadzone;

        auto stability = stateEstimator->parameters->get_position_stability();
        dt_ = stability.dt; // ms
        goal_thres_ = stability.deadzone; // m
        timeout_ = stability.timeout; // (dt * timeout) ms

        auto gains = stateEstimator->parameters->get_orientation_gains();
        init(dt_ / 1000.0, 1, -1, gains.kp, gains.kd, gains.ki);
        count_ = 0;
        DEBUG(gains);
    }

    NodeStatus tick() override
    {
        Pose2D wp;
        if (getInput("waypoint", wp)) {

            double theta = stateEstimator_->at(2);
            double heading_error = fabs(fmod((wp.theta - theta + M_PI), 2 * M_PI) - M_PI);

//            double x, y;
            x_ = stateEstimator_->at(0);
            y_ = stateEstimator_->at(1);
            double alpha = atan2(y_, x_);

            auto desire = sqrt(wp.x * wp.x + wp.y * wp.y);
            auto curr = sqrt(x_ * x_ + y_ * y_);

            double error = fabs(desire-curr);

            if(heading_error > ori_thres_)
                std::cout << "[Position Controller]: heading error cross limits " << wp.theta << " > " <<theta;

            if (heading_error <= ori_thres_ &&  error > goal_thres_) {
                std::cout << "[Position Controller]: chasing " << wp.x << ", " << wp.y;
                double v = calculate(desire, curr);
                stateEstimator_->add_cmd_vel(v, 0);
                std::cout << "| position = " << x_ << ", " << y_ << " | error = " << error << std::endl;
            }

            return (heading_error <= ori_thres_ && error <= goal_thres_) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        }
        return NodeStatus::SUCCESS;
    }


    double get_distance(double dx, double dy)
    {
        return sqrt(dx * dx + dy * dy);
    }

    static PortsList providedPorts()
    {
        return {InputPort<Pose2D>("waypoint")};
    }
private:
    double x_, y_;
    int dt_;
    int timeout_;
    double goal_thres_, ori_thres_;
    StatePtr stateEstimator_;
    int count_;
};
#endif //BT_CONTROLLER_POSITION_CONTROLLER_H
