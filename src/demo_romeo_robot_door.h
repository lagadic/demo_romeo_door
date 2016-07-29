#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#include <visp_bridge/3dpose.h>
#include <visp_bridge/camera.h>

#include <visp_naoqi/vpNaoqiRobot.h>

#include <vpCartesianDisplacement.h>

#include <visp/vpDisplayX.h>
#include <visp3/core/vpDisplay.h>

#include <vpServoArm.h>

typedef enum {
  HeadToZero,
  WaitHeadToZero,
  MoveRightArm,
  VisualServoRHand,
  WaitRHandServoed,
  SecondPBVS,
  WaitRHandServoedSecond,
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
  SaveOffset,
} State_t;


class DemoRomeoDoor
{
public:

  DemoRomeoDoor(ros::NodeHandle &nh);
  ~DemoRomeoDoor();
  void computeControlLaw(const vpHomogeneousMatrix &doorhandleMoffset);
  void getActualPoseCb(const geometry_msgs::PoseStamped::ConstPtr &actualPose);
  void getDesiredPoseCb(const geometry_msgs::PoseStamped::ConstPtr &desiredPose);
  void getStatusPoseHandCb(const std_msgs::Int8::ConstPtr  &status);
  void getStatusPoseDesiredCb(const std_msgs::Int8::ConstPtr  &status);
  void getStatusDoorHandleCB(const std_msgs::Int8ConstPtr &status_door_handle);
  void getStatusHandCB(const std_msgs::Int8ConstPtr &status);
  void getStatusPBVSCB(const std_msgs::BoolConstPtr &status);
  void initDisplay();
  void moveRArmFromRestPosition ();
  void moveRArmToRestPosition ();
  void publishCmdVel(const vpColVector &q);
  void saveOffset();
  void setupCameraParameters(const sensor_msgs::CameraInfoConstPtr &cam_rgb);
  void spin();

protected:

  // Robot
  vpNaoqiRobot romeo;
  int port;
  std::string ip;
  std::vector<std::string> jointNamesHead;
  std::vector<std::string> jointNames_tot;
  std::vector<std::string> jointNames_arm;
  int numJoints;
  std::string chain_name;
  vpColVector jointMin;
  vpColVector jointMax;

  // ROS
  ros::NodeHandle n;
  std::string actualPoseTopicName;
  std::string desiredPoseTopicName;
  std::string cmdVelTopicName;
  std::string opt_arm;
  std::string offsetFileName;
  std::string offsetInitialName;
  std::string offsetFinalName;
  std::string statusHandTopicName;
  std::string statusDoorHandleTopicName;
  std::string cameraRGBTopicName;
  ros::Subscriber actualPoseSub;
  ros::Subscriber cam_rgb_info_sub;
  ros::Subscriber desiredPoseSub;
  ros::Subscriber state_door_handle_sub;
  ros::Subscriber state_hand_sub;
  ros::Publisher cmdVelPub;
  int freq;

  //State
  State_t  state;
  bool door_closed;

  //Image and Display
  vpDisplay* disp;
  vpImage<unsigned char> img_;

  vpMouseButton::vpMouseButtonType button;
  //Servo Arm
  vpServoArm servo_arm;
  vpColVector q;
  vpColVector q_dot;
  vpColVector q2_dot;
  sensor_msgs::JointState q_dot_msg;
  double servo_time_init;

  //Pose
  vpHomogeneousMatrix cMh;
  vpHomogeneousMatrix cMdh;
  vpHomogeneousMatrix oMe_Arm;
  vpHomogeneousMatrix dhMoffsetInitial;
  vpHomogeneousMatrix dhMoffsetFinal;
  vpHomogeneousMatrix cMdh_fixed;

  //Camera Parameter
  vpCameraParameters cam_;

  //Parameters
  bool init;
  bool start_pbvs;
  bool pose_door_handle_is_fixed;
  bool savePose;
  unsigned int status_door_handle;
  unsigned int status_hand;
  unsigned int pbvs_finished;


};
