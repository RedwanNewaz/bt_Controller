//
// Created by redwan on 11/12/22.
//

#ifndef CREATE3_CONTROLLER_BT_EXECUTOR_H
#define CREATE3_CONTROLLER_BT_EXECUTOR_H
#include "COM.h"
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "create3_controller/traj_follower.h"
#include "bt_controller/gen_waypoints.h"
#include "bt_controller/position_controller.h"
#include "bt_controller/orientation_controller.h"

class bt_executor:public rclcpp::Node
{
public:
    bt_executor(StatePtr stateEstimator):Node("create3_controller")
    {
        BehaviorTreeFactory factory;

        factory.registerNodeType<PopFromQueue<Pose2D>>("PopFromQueue");
        factory.registerNodeType<QueueSize<Pose2D>>("QueueSize");
        factory.registerNodeType<ConsumeQueue<Pose2D>>("ConsumeQueue");

        factory.registerNodeType<GenerateWaypoints>("GenerateWaypoints", stateEstimator->parameters->get_path_file());
        factory.registerNodeType<PositionController>("PositionController", stateEstimator->get_ptr());
        factory.registerNodeType<OrientationController>("OrientationController", stateEstimator->get_ptr());

//        tree_ = factory.createTreeFromText(xml_A);
        tree_ = factory.createTreeFromFile(stateEstimator->parameters->get_xml_file());
        timer_ = this->create_wall_timer(
                3ms, std::bind(&bt_executor::timer_callback, this));
    }
private:
    void timer_callback()
    {
        auto status = tree_.tickOnce();
        if (status != BT::NodeStatus::RUNNING)
        {
            DEBUG("BT terminated!");
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree tree_;

};


#endif //CREATE3_CONTROLLER_BT_EXECUTOR_H
