#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "../include/usv_nav.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UsvNavNode>());
  rclcpp::shutdown();
  return 0;
}