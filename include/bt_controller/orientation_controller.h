//
// Created by Abdullah Al Redwan Newaz on 11/10/22.
//

#ifndef BT_CONTROLLER_ORIENTATION_CONTROLLER_H
#define BT_CONTROLLER_ORIENTATION_CONTROLLER_H
#include "gen_waypoints.h"
#include "pid.h"
#include "create3_controller/state_estimator.h"
/**
 * @brief Orientation controller is responsible to change orientation of the robot.
 * This node will execute first before the position controller.
 * The robot will align its orientation toward goal direction first, then start to make progress toward goal
 */
class OrientationController : public ThreadedAction, public PID
{
public:
    OrientationController(const std::string& name, const NodeConfig& config, StatePtr stateEstimator) :
            ThreadedAction(name, config), stateEstimator_(stateEstimator)
    {
        heading_angle_ = stateEstimator_->at(2);

        auto stability = stateEstimator->parameters->get_orientation_stability();
        dt_ = stability.dt; // ms
        goal_thres_ = stability.deadzone; // radian
        timeout_ = stability.timeout; // (dt * timeout) ms

        //      dt  double max, double min, double Kp, double Kd, double Ki
        auto gains = stateEstimator->parameters->get_position_gains();
        init(dt_ / 1000.0, 1, -1, gains.kp, gains.kd, gains.ki);

        DEBUG(gains);
    }

    NodeStatus tick() override
    {
        Pose2D wp;

        if (getInput("waypoint", wp)) {



            heading_angle_ = stateEstimator_->at(2);
            double dx, dy;
            dx = wp.x - stateEstimator_->at(0);
            dy = wp.y - stateEstimator_->at(1);
            double alpha = atan2(dy, dx);
//            heading_angle_ = fmod(( atan2(y_, x_) - heading_angle_ + M_PI), 2 * M_PI) - M_PI;
            double error = fabs(fmod((alpha - heading_angle_ + M_PI), 2 * M_PI) - M_PI);

            if (error > goal_thres_) {
                std::cout << "[Orientation Controller]: chasing " << alpha << " | current heading " << heading_angle_;
                double w = calculate( alpha, heading_angle_);
                stateEstimator_->add_cmd_vel(0, w);
                std::cout << "| heading = " << heading_angle_ << " | error = " << error << std::endl;
            }

            return (error <= goal_thres_) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        }
        return NodeStatus::SUCCESS;
    }

    static PortsList providedPorts()
    {
        return {InputPort<Pose2D>("waypoint")};
    }
private:
    double heading_angle_;
    int dt_;
    int timeout_;
    double goal_thres_;
    StatePtr stateEstimator_;
};

#endif //BT_CONTROLLER_ORIENTATION_CONTROLLER_H
