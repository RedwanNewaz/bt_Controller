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

        auto stability = stateEstimator->parameters->get_position_stability();
        dt_ = stability.dt; // ms
        goal_thres_ = stability.deadzone; // m
        timeout_ = stability.timeout; // (dt * timeout) ms

        auto gains = stateEstimator->parameters->get_orientation_gains();
        init(dt_ / 1000.0, 1, -1, gains.kp, gains.kd, gains.ki);
        DEBUG(gains);
    }

    NodeStatus tick() override
    {
        Pose2D wp;
        if (getInput("waypoint", wp))
        {
            double theta = wp.theta;
            double error = get_distance(wp.x, wp.y);
            std::cout << "[Position Controller]: chasing " << wp.x << ", " << wp.y << std::endl;

            int step = 0;
            while (error > goal_thres_)
            {
                error = get_distance(wp.x, wp.y);
                auto desire = sqrt(wp.x * wp.x + wp.y * wp.y);
                auto curr = sqrt(x_ * x_ + y_ * y_);
                double v = calculate(desire, curr);
                stateEstimator_->add_cmd_vel(v, 0);
//                x_ += v * cos(theta);
//                y_ += v * sin(theta);
                x_ = stateEstimator_->at(0);
                y_ = stateEstimator_->at(1);
                std::this_thread::sleep_for(std::chrono::milliseconds(dt_));

//                if(++step > timeout_)
//                {
//                    std::cout << "[Position Controller]: skipped deadlock \n";
//                    break;
//                }
            }

            std::cout << "[Position Controller]: " << x_ << ", " << y_ << " | error = " << error  << " step = "<< step <<std::endl;
            return NodeStatus::SUCCESS;
        }
        else
        {
            return NodeStatus::FAILURE;
        }
    }

    double get_distance(double x, double y)
    {
        double dx = x - x_;
        double dy = y - y_;
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
    double goal_thres_;
    StatePtr stateEstimator_;
};
#endif //BT_CONTROLLER_POSITION_CONTROLLER_H
