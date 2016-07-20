#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Transform.h>

#include <visp_bridge/3dpose.h>

#include <visp_naoqi/vpNaoqiRobot.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#include <vpCartesianDisplacement.h>

#include <visp/vpDisplayX.h>
#include <visp3/core/vpDisplay.h>

typedef enum {
  HeadToZero,
  WaitHeadToZero,
  MoveRightArm,
  VisualServoRHand,
  WaitRHandServoed,
  PutHandOnDoorHandle1,
  PutHandOnDoorHandle2,
  GraspingDoorHandle,
  PutTheHandCloser,
  GraspingDoorHandle2,
  PutTheHandCloser2,
  GraspingDoorHandle3,
  RotatingHandle,
  OpenDoor,
  CloseDoor,
  ReleaseDoorHandle,
  GetAwayFromHandle,
  GoBacktoResPosition,
  DoorOpenedAndQuit,
} State_t;


class DemoRomeoDoor
{
public:

  DemoRomeoDoor(ros::NodeHandle &nh);
  ~DemoRomeoDoor();
  void spin();
  void getStatusDoorHandleCB(const std_msgs::Int8ConstPtr &status_door_handle);
  void getStatusHandCB(const std_msgs::Int8ConstPtr &status);
  void getStatusPBVSCB(const std_msgs::BoolConstPtr &status);
  void moveRArmFromRestPosition ();
  void moveRArmToRestPosition ();
  void initDisplay();

protected:

  // Robot
  vpNaoqiRobot romeo;
  int port;
  std::string statusDoorHandleTopicName;
  std::string statusHandTopicName;
  std::string pbvsFinishedTopicName;
  std::string ip;
  std::vector<std::string> jointNamesHead;
  std::vector<std::string> jointNames_tot;
  // ROS
  ros::NodeHandle n;
  ros::Subscriber state_door_handle_sub;
  ros::Subscriber state_hand_sub;
  ros::Subscriber pbvs_finished_sub;
  ros::Publisher pbvs_active_pub;
  int freq;

  //State
  State_t  state;

  unsigned int status_door_handle;
  unsigned int status_hand;
  unsigned int pbvs_finished;
  std_msgs::Int8 pbvs_active;

  bool next_step;

  vpDisplay* disp;
  vpImage<unsigned char> img_;
  vpMouseButton::vpMouseButtonType button;


};
