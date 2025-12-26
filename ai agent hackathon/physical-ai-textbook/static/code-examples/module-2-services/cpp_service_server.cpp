#include "rclcpp/rclcpp.hpp"
#include "module_2_services/srv/add_two_ints.hpp"

#include <memory>

void add(const std::shared_ptr<module_2_services::srv::AddTwoInts::Request> request,
         std::shared_ptr<module_2_services::srv::AddTwoInts::Response>      response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  rclcpp::Service<module_2_services::srv::AddTwoInts>::SharedPtr service =
    node->create_service<module_2_services::srv::AddTwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
