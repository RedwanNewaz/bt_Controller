//
// Created by Abdullah Al Redwan Newaz on 11/10/22.
//

#ifndef BT_CONTROLLER_POSITION_CONTROLLER_H
#define BT_CONTROLLER_POSITION_CONTROLLER_H
#include "gen_waypoints.h"
#include "pid.h"
/**
 * @brief control robot 2D position. This node will execute after orientation controller.
 * The robot will make progress toward its destination
 */
class PositionController : public ThreadedAction, public PID
{
public:
    PositionController(const std::string& name, const NodeConfig& config) :
            ThreadedAction(name, config)
    {
        x_, y_ = 0;
        dt_ = 3; // ms
        goal_thres_ = 0.1; // m
        timeout_ = 250; // (dt * timeout) ms
        init(dt_/ 100.0, 1, -1, 0.05, 0, 0);
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
                double v = calculate(error, 1e-10);
                x_ += v * cos(theta);
                y_ += v * sin(theta);
                std::this_thread::sleep_for(std::chrono::milliseconds(dt_));

                if(++step > timeout_)
                {
                    std::cout << "[Position Controller]: skipped deadlock \n";
                    break;
                }
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
};
#endif //BT_CONTROLLER_POSITION_CONTROLLER_H
