//
// Created by Abdullah Al Redwan Newaz on 11/10/22.
//

#ifndef BT_CONTROLLER_ORIENTATION_CONTROLLER_H
#define BT_CONTROLLER_ORIENTATION_CONTROLLER_H
#include "gen_waypoints.h"
#include "pid.h"
/**
 * @brief Orientation controller is responsible to change orientation of the robot.
 * This node will execute first before the position controller.
 * The robot will align its orientation toward goal direction first, then start to make progress toward goal
 */
class OrientationController : public ThreadedAction, public PID
{
public:
    OrientationController(const std::string& name, const NodeConfig& config) :
            ThreadedAction(name, config)
    {
        heading_angle_ = 0;
//      dt  double max, double min, double Kp, double Kd, double Ki
        dt_ = 3; // ms
        goat_thres_ = 0.01; // radian
        timeout_ = 250; // (dt * timeout) ms
        init(dt_ / 100.0, 1, -1, 0.1, 0, 0);
    }

    NodeStatus tick() override
    {
        Pose2D wp;
        if (getInput("waypoint", wp))
        {

            double error = fabs(wp.theta - heading_angle_);
            int step = 0;
            while (error > goat_thres_)
            {
                double w = (wp.theta > heading_angle_)?calculate(wp.theta, heading_angle_):calculate( heading_angle_, wp.theta);
                // keep the angle between [-pi, pi]
                heading_angle_ = fmod((heading_angle_ + w * dt_) + M_PI, 2 * M_PI) - M_PI;
                std::this_thread::sleep_for(std::chrono::milliseconds(dt_));
                error = fabs(wp.theta - heading_angle_);
                if(++step > timeout_)
                {
                    std::cout << "[Position Controller]: skipped deadlock \n";
                    break;
                }
            }
            std::cout << "[Orientation Controller]: " << wp.theta << " | error = " << error << std::endl;

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
    double goat_thres_;
};

#endif //BT_CONTROLLER_ORIENTATION_CONTROLLER_H
