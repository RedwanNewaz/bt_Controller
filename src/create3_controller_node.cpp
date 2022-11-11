#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "create3_controller/traj_follower.h"
#include "bt_controller/gen_waypoints.h"
#include "bt_controller/position_controller.h"
#include "bt_controller/orientation_controller.h"

// clang-format off

static const char* xml_A = R"(
 <root main_tree_to_execute = "TreeA" >
     <BehaviorTree ID="TreeA">
        <Sequence>
            <GenerateWaypoints waypoints="{waypoints}" />
            <QueueSize queue="{waypoints}" size="{wp_size}" />
            <Repeat num_cycles="{wp_size}" >
                <Sequence>
                    <PopFromQueue  queue="{waypoints}" popped_item="{wp}" />
                    <Sequence>
                        <OrientationController waypoint="{wp}" />
                        <PositionController waypoint="{wp}" />
                    </Sequence>
                </Sequence>
            </Repeat>
        </Sequence>
     </BehaviorTree>
 </root>
 )";




int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);



    BehaviorTreeFactory factory;

    factory.registerNodeType<PopFromQueue<Pose2D>>("PopFromQueue");
    factory.registerNodeType<QueueSize<Pose2D>>("QueueSize");
    factory.registerNodeType<ConsumeQueue<Pose2D>>("ConsumeQueue");



    StatePtr stateEstimator = make_shared<state_estimator>(argv[1]);

    factory.registerNodeType<GenerateWaypoints>("GenerateWaypoints", stateEstimator->parameters->get_path_file());
    factory.registerNodeType<PositionController>("PositionController", stateEstimator->get_ptr());
    factory.registerNodeType<OrientationController>("OrientationController", stateEstimator->get_ptr());



    rclcpp::executors::MultiThreadedExecutor executor;
    auto node1 = std::make_shared<TrajFollower>(stateEstimator->get_ptr());
    executor.add_node(node1);


    auto tree = factory.createTreeFromText(xml_A);







    while (rclcpp::ok())
    {
        executor.spin_once();
        auto status = tree.tickOnce();
        if (status != BT::NodeStatus::RUNNING)
            break;
    }
//
//    executor.spin();
    DEBUG("[+] Node terminated, shutting down ...");
    rclcpp::shutdown();

  return 0;
}
