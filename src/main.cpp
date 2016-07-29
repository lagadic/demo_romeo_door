#include <ros/ros.h>

#include <visp_naoqi/vpNaoqiRobot.h>
#include <sensor_msgs/JointState.h>
#include "demo_romeo_robot_door.h"

int main( int argc, char** argv )
{
  ros::init( argc, argv, "demo_romeo_door" );

  ros::NodeHandle n(std::string("~"));

  DemoRomeoDoor *node = new DemoRomeoDoor(n);

  node->spin();

  delete node;

  return 0;
}
