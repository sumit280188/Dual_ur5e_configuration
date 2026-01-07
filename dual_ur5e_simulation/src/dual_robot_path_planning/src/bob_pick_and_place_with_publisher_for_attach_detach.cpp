#include <rclcpp/rclcpp.hpp>
#include "dual_robot_path_planning/pick_place_node.hpp"
#include "dual_robot_path_planning/guid_monitor.hpp"
#include <thread>
#include <chrono>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_publisher_demo");

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("pick_place_publisher_demo", node_options);

    try
    {
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);

        std::thread executor_thread([&executor]()
                                    { executor.spin(); });

        std::this_thread::sleep_for(std::chrono::seconds(1));

        RCLCPP_INFO(LOGGER, "Initializing pick and place node...");
        PickPlaceNode pick_place_node(node);

        // Execute the pick and place task
        pick_place_node.executePickAndPlaceTask();

        executor.cancel();
        executor_thread.join();
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(LOGGER, "Exception during pick and place execution: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}