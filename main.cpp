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

// clang-format on

int main()
{
    BehaviorTreeFactory factory;

    factory.registerNodeType<PopFromQueue<Pose2D>>("PopFromQueue");
    factory.registerNodeType<QueueSize<Pose2D>>("QueueSize");
    factory.registerNodeType<ConsumeQueue<Pose2D>>("ConsumeQueue");

    factory.registerNodeType<OrientationController>("OrientationController");
    factory.registerNodeType<PositionController>("PositionController");
    factory.registerNodeType<GenerateWaypoints>("GenerateWaypoints");

    auto tree = factory.createTreeFromText(xml_A);
    tree.tickWhileRunning();
    std::cout << "--------------" << std::endl;


    return 0;
}
