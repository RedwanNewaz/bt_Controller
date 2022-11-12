#include "../include/create3_controller/bt_executor.h"




int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    StatePtr stateEstimator = make_shared<state_estimator>(argv[1]);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node1 = std::make_shared<TrajFollower>(stateEstimator->get_ptr());
    auto node2 = std::make_shared<bt_executor>(stateEstimator->get_ptr());
    executor.add_node(node1);
    executor.add_node(node2);

    executor.spin();
    DEBUG("[+] Node terminated, shutting down ...");
    rclcpp::shutdown();

  return 0;
}
