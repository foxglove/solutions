#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
// C++ namespace for representing time durations
using namespace std::chrono_literals;

int main(int argc, char **argv) {
    // init rclcpp lib
    rclcpp::init(argc, argv);

    // create shared pointer to node and call it diagnostics_node
    std::shared_ptr<rclcpp::Node> node=rclcpp::Node::make_shared("diagnostics_node");

    // create client in node and call checks server node
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client = 
        node->create_client<std_srvs::srv::Trigger>("checks");

    // create empty request 
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // wait for service to be active
    while (!client->wait_for_service(1s)) {
        // if ros is shut down before service is active, show this error 
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "interrupted while waiting for the service. Exiting.");
            return 0;
        }
        // print in the screen some info so user knows what is happening
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // client sends async request 
    auto result = client->async_send_request(request);
    // wait for result 
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        // get response success field to see if all checks passed 
        if (result.get()->success) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The checks were successful");
        } else {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "The checks were not successful: %s", result.get()->message.c_str());
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service 'checks'");
    }

    // shut down rclcpp and client node
    rclcpp::shutdown();
    return 0;
}