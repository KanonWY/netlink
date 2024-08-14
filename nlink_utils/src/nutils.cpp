#include "nutils.h"

// #include <ros/ros.h>
#include <cstdio>


void TopicAdvertisedTip(const char *topic) {
  // RCLCPP_INFO("%s has been advertised,use 'rostopic "
  //          "echo /%s' to view the data",
  //          topic, topic);
  printf("%s has been advertised, use 'ros2 topic echo /%s to view the data",topic, topic);
}
