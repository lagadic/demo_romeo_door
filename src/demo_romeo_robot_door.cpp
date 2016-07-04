#include <iostream>
#include <vector>
#include <algorithm>

#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Transform.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>

#include "demo_romeo_robot_door.h"

#include <std_msgs/Int8.h>


DemoRomeoDoor::DemoRomeoDoor(ros::NodeHandle &nh)
{
  // read in config options
  n = nh;

  n.param( "frequency", freq, 100);
  n.param<std::string>("Ip", ip, "198.18.0.1");
  n.param<std::string>("StatusDoorHandleTopicName", statusDoorHandleTopicName, "/door_handle_detection/status");
  n.param<std::string>("StatusHandTopicName", statusHandTopicName, "/visp_blobs_tracker/status");
  n.param<std::string>("StatusFinishedTopicName", pbvsFinishedTopicName, "/pbvs_arm_servo/pbvs_computed");
  n.param("Port", port, 9559);

  pbvs_active.data = 0;

  pbvs_finished = 0;

  ROS_INFO("Launch DemoRomeoDoor node");
//  romeo = new vpNaoqiRobot;
  romeo.setRobotIp(ip);
  romeo.open();
  state_door_handle_sub = n.subscribe( statusDoorHandleTopicName, 1, (boost::function < void(const std_msgs::Int8ConstPtr&)>) boost::bind( &DemoRomeoDoor::getStatusDoorHandleCB, this, _1 ));
  state_hand_sub = n.subscribe( statusHandTopicName, 1, (boost::function < void(const std_msgs::Int8ConstPtr&)>) boost::bind( &DemoRomeoDoor::getStatusHandCB, this, _1 ));
  pbvs_finished_sub = n.subscribe( pbvsFinishedTopicName, 1, (boost::function < void(const std_msgs::BoolConstPtr&)>) boost::bind( &DemoRomeoDoor::getStatusPBVSCB, this, _1 ));
  pbvs_active_pub = n.advertise< std_msgs::Int8 >("pbvs_active", 1);

  state = HeadToZero;
}

DemoRomeoDoor::~DemoRomeoDoor(){

//  if (romeo){
//    romeo.stop(jointNames_tot_hroll);
//    delete romeo;
//    romeo = NULL;
//  }
  romeo.stop(jointNames_tot);
}


void DemoRomeoDoor::spin()
{
  ros::Rate loop_rate(100);
  std::vector<std::string> jointNames = romeo.getBodyNames("Head");
  jointNames.pop_back(); // We don't consider  the last joint of the head = HeadRoll

  // Create vector with joint names of the head +eyes with Head Roll
  std::vector<std::string> jointNames_hroll = romeo.getBodyNames("Head");

  std::vector<std::string> jointNamesLEye = romeo.getBodyNames("LEye");
  std::vector<std::string> jointNamesREye = romeo.getBodyNames("REye");

  jointNames.insert(jointNames.end(), jointNamesLEye.begin(), jointNamesLEye.end());
  std::vector<std::string> jointNames_tot_eyes = jointNames;
  jointNames_tot_eyes.push_back(jointNamesREye.at(0));
  jointNames_tot_eyes.push_back(jointNamesREye.at(1));

  jointNames_hroll.insert(jointNames_hroll.end(), jointNamesLEye.begin(), jointNamesLEye.end());
  jointNamesHead = jointNames_hroll;
  jointNamesHead.push_back(jointNamesREye.at(0));
  jointNamesHead.push_back(jointNamesREye.at(1));

  std::vector<std::string> jointNamesRArm = romeo.getBodyNames("RArm");
  jointNames_tot = jointNamesHead;
  jointNames_tot.insert(jointNames_tot.end(), jointNamesRArm.begin(), jointNamesRArm.end());


  vpColVector head_pose(jointNamesHead.size(), 0);

  romeo.getProxy()->setStiffnesses("RHand", 1.0f);
  AL::ALValue angle = 1.00;
  romeo.getProxy()->setAngles("RHand", angle, 0.15);


  while(ros::ok()){
    if (state == HeadToZero)
    {
      //OpenLoop Control to see the door handle
      head_pose = 0;
      head_pose[0] = vpMath::rad(-19.7); // NeckYaw
      head_pose[1] = vpMath::rad(27.); // NeckPitch
      head_pose[2] = vpMath::rad(-7.6); // HeadPitch
      head_pose[3] = vpMath::rad(0.0); // HeadRoll
      romeo.setPosition(jointNamesHead, head_pose, 0.06);
      state = WaitHeadToZero;
      ROS_INFO("Head should go in zero position");

    }
    if (state == WaitHeadToZero)
    {
      vpColVector head_pose_mes = romeo.getPosition(jointNamesHead);
      double error = sqrt((head_pose_mes-head_pose).sumSquare());
      if (error < vpMath::rad(4) && status_door_handle)
      {
        state = MoveRightArm;
        ROS_INFO("Head is in zero position");
      }
      else
        std::cout << "error = " << error << "  status door handle : " << status_door_handle << std::endl;
    }
    if (state == MoveRightArm)
    {
      moveRArmFromRestPosition(); // move up
      ROS_INFO("Right Arm should go in zero position");
      state = VisualServoRHand;
//      ros::shutdown();
    }
    if (state == VisualServoRHand)
    {
      if (status_hand && status_door_handle)
      {
        //Start the pbvs node
        pbvs_active.data = 1;
        pbvs_active_pub.publish(pbvs_active);
        state = WaitRHandServoed;
        ROS_INFO("Right Arm should be servoed to the door handle with an offset");

      }

    }
    if (state == WaitRHandServoed)
    {
      pbvs_active.data = 1;
      pbvs_active_pub.publish(pbvs_active);
      if (pbvs_finished)
      {
        //Stop the pbvs node
        pbvs_active.data = 2;
        pbvs_active_pub.publish(pbvs_active);
        state = OpenDoor1;
        ROS_INFO("Right Arm should have finished to be servoed");
        vpTime::sleepMs(1000);
      }

    }
    if (state == OpenDoor1)
    {
      // Open loop upward motion of the hand
      vpColVector cart_delta_pos(6, 0);
      cart_delta_pos[1] = -0.03;
      cart_delta_pos[3] = vpMath::rad(-90);
      double delta_t = 3;


      static vpCartesianDisplacement moveCartesian;
      vpVelocityTwistMatrix V;
      if (moveCartesian.computeVelocity(romeo, cart_delta_pos, delta_t, "RArm", V)) {
        romeo.setVelocity(moveCartesian.getJointNames(), moveCartesian.getJointVelocity());
        ROS_INFO("Right Arm should go in openLoop to open the door");
      }
      else
      {
        romeo.stop(moveCartesian.getJointNames());
        state = OpenDoor2;
        ROS_INFO("Right Arm should have finished to open the door");
      }
    }
    if (state == OpenDoor2)
    {
      // Open loop upward motion of the hand
      vpColVector cart_delta_pos(6, 0);
      cart_delta_pos[5] = vpMath::rad(-20);
      cart_delta_pos[0] = 0.04;
      cart_delta_pos[1] = 0.072;
      double delta_t = 3;


      static vpCartesianDisplacement moveCartesian;
      vpVelocityTwistMatrix V;
      if (moveCartesian.computeVelocity(romeo, cart_delta_pos, delta_t, "RArm", V)) {
        romeo.setVelocity(moveCartesian.getJointNames(), moveCartesian.getJointVelocity());
        ROS_INFO("Right Arm should go in openLoop to open the door");
      }
      else
      {
        romeo.stop(moveCartesian.getJointNames());
        state = GraspingDoorHandle;
        ROS_INFO("Right Arm should have finished to open the door");
      }
    }
    if (state == GraspingDoorHandle)
    {
      romeo.getProxy()->setStiffnesses("RHand", 1.0f);
      AL::ALValue angle = 0.50;
      romeo.getProxy()->setAngles("RHand", angle, 0.15);
      state = RotatingHandle;
      ROS_INFO("Right Arm should have grasped the handle");
    }
    if (state == RotatingHandle)
    {
      vpHomogeneousMatrix hMp;
      for (int i = 0; i < 3; i++)
        hMp[i][i] = 0;

      hMp[1][0] = -1;
      hMp[2][1] = 1;
      hMp[0][2] = -1;
      hMp[0][3] = 0.15;
      hMp[1][3] = 0.05;
      hMp[2][3] = -0.02;

      vpVelocityTwistMatrix wVh(hMp);
      vpColVector v_wrist, v_handle(6,0);
      v_handle[5] = -vpMath::rad(10);
      v_wrist = wVh * v_handle;
      static double t_1 = vpTime::measureTimeSecond();
      if ( vpTime::measureTimeSecond() - t_1 < 7.0)
      {
         vpColVector q = romeo.get_eJe("RArm").pseudoInverse() * v_wrist;
//         ROS_INFO_STREAM("q = " << q << " v_wrist = " << v_wrist);
         std::vector<std::string> jointNames = romeo.getBodyNames("RArm");
         jointNames.pop_back(); // We don't consider  the last joint of the arm = Hand
         romeo.setVelocity(jointNames, q);
      }
      else
      {
        romeo.stop(jointNames_tot);
        state = WaitDoorOpening;
        ROS_INFO("Right Arm should have opened the door");
      }
    }
    if (state == WaitDoorOpening)
    {
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("Shutdown fait n'importe quoi ?");
//  ros::shutdown();
}

void DemoRomeoDoor::getStatusDoorHandleCB(const std_msgs::Int8ConstPtr &status_dh)
{
  status_door_handle = status_dh->data;
}

void DemoRomeoDoor::getStatusHandCB(const std_msgs::Int8ConstPtr &status)
{
  status_hand = status->data;
}

void DemoRomeoDoor::getStatusPBVSCB(const std_msgs::BoolConstPtr &status)
{
  pbvs_finished = status->data;
}

void DemoRomeoDoor::moveRArmFromRestPosition ()
{

  try
  {

    AL::ALValue pos1 = AL::ALValue::array(0.3741794228553772, -0.33311545848846436, -0.036883752793073654, 1.260278582572937, 0.4322237968444824, 0.009434251114726067);
    AL::ALValue pos2 = AL::ALValue::array(0.39718443155288696, -0.282814621925354, 0.17343932390213013, 1.194741129875183, -0.5093367099761963, 0.18652880191802979);
    AL::ALValue pos3 = AL::ALValue::array(0.35522401332855225, -0.1643453985452652, 0.13889043033123016, 1.400763988494873, -0.566399872303009, 0.49108603596687317);

    AL::ALValue time1 = 1.5f;
    AL::ALValue time2 = 3.0f;
    AL::ALValue time3 = 5.0f;



    AL::ALValue path;
    path.arrayPush(pos1);
    path.arrayPush(pos2);
    path.arrayPush(pos3);


    AL::ALValue times;
    times.arrayPush(time1);
    times.arrayPush(time2);
    times.arrayPush(time3);

    AL::ALValue chainName  = AL::ALValue::array ("RArm");
    AL::ALValue space      = AL::ALValue::array (0); // Torso
    AL::ALValue axisMask   = AL::ALValue::array (63);

    romeo.getProxy()->positionInterpolations(chainName, space, path, axisMask, times);

  }
  catch(const std::exception&)
  {
    throw vpRobotException (vpRobotException::badValue,
                            "servo apply the motion");
  }

  return;

}
