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
    std::string roomba20("subsampled20_odom_roomba20.csv"), roomba21("subsampled20_roomba21.csv");

    BehaviorTreeFactory factory;

    factory.registerNodeType<PopFromQueue<Pose2D>>("PopFromQueue");
    factory.registerNodeType<QueueSize<Pose2D>>("QueueSize");
    factory.registerNodeType<ConsumeQueue<Pose2D>>("ConsumeQueue");

    factory.registerNodeType<GenerateWaypoints>("GenerateWaypoints");

    StatePtr stateEstimator = make_shared<state_estimator>();

    factory.registerNodeType<PositionController>("PositionController", stateEstimator->get_ptr());
    factory.registerNodeType<OrientationController>("OrientationController", stateEstimator->get_ptr());


    auto tree = factory.createTreeFromText(xml_A);
//    tree.tickWhileRunning();
//    std::cout << "--------------" << std::endl;
//



    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node1 = std::make_shared<TrajFollower>(stateEstimator->get_ptr());
    executor.add_node(node1);

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
