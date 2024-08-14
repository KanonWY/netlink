#include <iostream>
#include <nlinktrack_module.h>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<linktrack::NetlinkNode>());
    rclcpp::shutdown();


    return 0;
}