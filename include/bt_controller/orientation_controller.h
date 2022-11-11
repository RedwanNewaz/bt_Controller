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
    }

    NodeStatus tick() override
    {
        Pose2D wp;
        if (getInput("waypoint", wp))
        {
            heading_angle_ = stateEstimator_->at(2);
            double error = fabs(wp.theta - heading_angle_);
            int step = 0;
            std::cout << "[Orientation Controller]: chasing " << wp.theta << " | current heading " << heading_angle_ << std::endl;
            while (error > goal_thres_)
            {
                heading_angle_ = stateEstimator_->at(2);
//                double w = calculate(wp.theta, heading_angle_);

                double w = (wp.theta > heading_angle_)?calculate(wp.theta, heading_angle_):calculate( heading_angle_, wp.theta);
                // keep the angle between [-pi, pi]
//                heading_angle_ = fmod((heading_angle_ + w * dt_) + M_PI, 2 * M_PI) - M_PI;
                stateEstimator_->add_cmd_vel(0, w);

                std::this_thread::sleep_for(std::chrono::milliseconds(dt_));
                double error1 = fabs(wp.theta - heading_angle_);
                double error2 = fabs(2 * M_PI - error1);
                error = std::min(error1, error2);
//                if(++step > timeout_)
//                {
//                    std::cout << "[Orientation Controller]: skipped deadlock \n";
//                    break;
//                }
            }
            std::cout << "[Orientation Controller]: " << heading_angle_ << " | error = " << error << std::endl;

            return NodeStatus::SUCCESS;
        }
        else
        {
            return NodeStatus::FAILURE;
        }
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
