// include rclcpp library and trigger service type 
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

bool moveMotorToMinAndMax(int motor_id) {
    bool reached_min = false;
    bool reached_max = false;

    // add code here that moves the motor to the minimum and maximum positions

    if (reached_min && reached_max) {
        return true;
    } else {
        return false;
    }
}

// DDS helps pass the request and response between the client and server
void doChecks(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    // prepare response 
    response->success = true;
    response->message = "";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request to check motors...");

    // iterate over all motors to perform the check
    for (int i = 0; i < 5; i++) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Checking motor %i", i);
        auto res = moveMotorToMinAndMax(i);

        // if fails, change response `success` to false and add info to the `message`
        if (!res) {
            response->success = false;
            response->message += "Motor"+std::to_string(i)+" Failed";
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response...");
}

int main(int argc, char **argv_) {
    // init rclcpp library 
    rclcpp::init(argc, argv);

    // create shared pointer to a Node type and name it "motor_node"
    std::shared_ptr<rclcpp::Node> node=rclcpp::Node::make_shared("motor_node");

    // create the "checks" service with a doChecks callback 
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service =
        node->create_service<std_srvs::srv::Trigger>("checks", &doChecks);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to check motors");

    // spin node until it's terminated
    rclcpp::spin(node);
    rclcpp:shutdown();
}