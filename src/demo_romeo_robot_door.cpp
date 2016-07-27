#include <iostream>
#include <vector>
#include <algorithm>

#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Transform.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>

#include "demo_romeo_robot_door.h"

#include <vpRomeoTkConfig.h>


#include <std_msgs/Int8.h>


DemoRomeoDoor::DemoRomeoDoor(ros::NodeHandle &nh)
{
  // read in config options
  n = nh;

  n.param( "frequency", freq, 100);
  n.param<std::string>("Ip", ip, "198.18.0.1");
  n.param<std::string>("StatusDoorHandleTopicName", statusDoorHandleTopicName, "/door_handle_detection/status");
  n.param<std::string>("StatusHandTopicName", statusHandTopicName, "/visp_blobs_tracker/status");
  n.param<std::string>("actualPoseTopicName", actualPoseTopicName, "/visp_blobs_tracker/object_position");
  n.param<std::string>("desiredPoseTopicName", desiredPoseTopicName, "/visp_blobs_tracker/object_des_position");
  n.param<std::string>("cameraRGBTopicName", cameraRGBTopicName, "/SR300/rgb/camera_info");
  n.param<std::string>("cmdVelTopicName", cmdVelTopicName, "joint_state");
  n.param<std::string>("armToControl", opt_arm, "right");
  n.param<std::string>("offsetFileName", offsetFileName, "/udd/bheintz/data_romeo/pose.xml");
  n.param<std::string>("offsetInitialName", offsetInitialName, "HandleOffset");
  n.param<std::string>("offsetFinalName", offsetFinalName, "FinalOffset");
  n.param("savePose", savePose, false);
  n.param("Port", port, 9559);

  pbvs_finished = 0;
  pose_handle_fixed = false;
  servo_time_init = 0;
  init = false;

  img_.init(480,640);
  img_ = 0;
  initDisplay();

  ROS_INFO("Launch DemoRomeoDoor node");
//  romeo = new vpNaoqiRobot;
  romeo.setRobotIp(ip);
  romeo.open();

  // Initialize subscriber and publisher
  desiredPoseSub = n.subscribe( desiredPoseTopicName, 1, (boost::function < void(const geometry_msgs::PoseStampedConstPtr&)>) boost::bind( &DemoRomeoDoor::getDesiredPoseCb, this, _1 ));
  actualPoseSub = n.subscribe( actualPoseTopicName, 1, (boost::function < void(const geometry_msgs::PoseStampedConstPtr &)>) boost::bind( &DemoRomeoDoor::getActualPoseCb, this, _1 ));
  state_door_handle_sub = n.subscribe( statusDoorHandleTopicName, 1, (boost::function < void(const std_msgs::Int8ConstPtr&)>) boost::bind( &DemoRomeoDoor::getStatusDoorHandleCB, this, _1 ));
  state_hand_sub = n.subscribe( statusHandTopicName, 1, (boost::function < void(const std_msgs::Int8ConstPtr&)>) boost::bind( &DemoRomeoDoor::getStatusHandCB, this, _1 ));
  cam_rgb_info_sub = n.subscribe( cameraRGBTopicName, 1, (boost::function < void(const sensor_msgs::CameraInfo::ConstPtr&)>) boost::bind( &DemoRomeoDoor::setupCameraParameters, this, _1 ));

  cmdVelPub = n.advertise<sensor_msgs::JointState >(cmdVelTopicName, 10);

  state = HeadToZero;

  if (opt_arm == "right")
    chain_name = "RArm";
  else
    chain_name = "LArm";

  std::string filename_transform = std::string(ROMEOTK_DATA_FOLDER) + "/transformation.xml";
  std::string name_transform = "qrcode_M_e_" + chain_name;
  vpXmlParserHomogeneousMatrix pm; // Create a XML parser

  if (pm.parse(oMe_Arm, filename_transform, name_transform) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
    std::cout << "Cannot found the homogeneous matrix named " << name_transform << "." << std::endl;
    ros::shutdown();
  }
  else
    std::cout << "Homogeneous matrix " << name_transform <<": " << std::endl << oMe_Arm << std::endl;

  ROS_INFO("Launch NaoqiRobotros node");
  jointNames_arm =  romeo.getBodyNames(chain_name);
  jointNames_arm.pop_back(); // Delete last joints LHand, that we don't consider in the servo

  numJoints = jointNames_arm.size();
  q.resize(numJoints);
  q_dot.resize(numJoints);
  q2_dot.resize(numJoints);
  q_dot_msg.velocity.resize(numJoints);
  q_dot_msg.name = jointNames_arm;
  jointMin.resize(numJoints);
  jointMax.resize(numJoints);

  if( pm.parse(dhMoffsetInitial, offsetFileName, offsetInitialName) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
    std::cout << "Cannot found the homogeneous matrix named " << offsetInitialName << "." << std::endl;
    ros::shutdown();
  }
  else
    std::cout << "Homogeneous matrix " << offsetInitialName <<": " << std::endl << dhMoffsetInitial << std::endl;

  if( pm.parse(dhMoffsetFinal, offsetFileName, offsetFinalName) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
    std::cout << "Cannot found the homogeneous matrix named " << offsetFinalName << "." << std::endl;
    ros::shutdown();
  }
  else
    std::cout << "Homogeneous matrix " << offsetFinalName <<": " << std::endl << dhMoffsetFinal << std::endl;

  //Get joint limits
  romeo.getJointMinAndMax(jointNames_arm, jointMin, jointMax);

  //Set the stiffness
  romeo.setStiffness(jointNames_arm, 1.f);
}

DemoRomeoDoor::~DemoRomeoDoor(){

//  if (romeo){
//    romeo.stop(jointNames_tot);
//    delete romeo;
//    romeo = NULL;
//  }
  delete disp;
  romeo.stop(jointNames_tot);
  ROS_INFO("DemoRomeoDoor normally destroyed !");
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

  bool click_done;
  bool once = 0;

  while(ros::ok()){

    vpDisplay::display(img_);
//    vpDisplay::displayText(img_, 10, 10, "Head should go in zero position", vpColor::red);
    click_done = vpDisplay::getClick(img_, button, false);

    if(button == vpMouseButton::button3)
    {
        ros::shutdown();
    }

    if ( pose_handle_fixed && status_door_handle)
      vpDisplay::displayFrame(img_, pose_door_handle_fixed, cam_, 0.1, vpColor::none, 2);
    else if ( (state == VisualServoRHand || state == WaitRHandServoed) && status_door_handle)
    {
      vpDisplay::displayFrame(img_, cMdh * dhMoffsetInitial, cam_, 0.1, vpColor::none, 2);
      vpDisplay::displayFrame(img_, cMdh, cam_, 0.1, vpColor::green);
    }
    else if ( (state == SecondPBVS || state == WaitRHandServoedSecond) && status_door_handle)
    {
      vpDisplay::displayFrame(img_, pose_door_handle_fixed * dhMoffsetFinal, cam_, 0.1, vpColor::none, 2);
      vpDisplay::displayFrame(img_, pose_door_handle_fixed, cam_, 0.1, vpColor::green);
    }
    else if (status_door_handle)
      vpDisplay::displayFrame(img_, cMdh, cam_, 0.1, vpColor::none, 2);
    if (status_hand)
      vpDisplay::displayFrame(img_, cMh, cam_, 0.1, vpColor::none, 2);

    if (state == HeadToZero)
    {
      //OpenLoop Control to see the door handle
      head_pose = 0;
      head_pose[0] = vpMath::rad(-24.3); // NeckYaw
      head_pose[1] = vpMath::rad(16.2); // NeckPitch
      head_pose[2] = vpMath::rad(-7.6); // HeadPitch
      head_pose[3] = vpMath::rad(0.0); // HeadRoll
      romeo.setPosition(jointNamesHead, head_pose, 0.06);
      state = WaitHeadToZero;
      vpDisplay::displayText(img_, vpImagePoint(15,10), "Head should go in zero position", vpColor::red);
      ROS_INFO("Head should go in zero position");

    }
    if (state == WaitHeadToZero)
    {
      vpColVector head_pose_mes = romeo.getPosition(jointNamesHead);
      double error = sqrt((head_pose_mes-head_pose).sumSquare());
      vpDisplay::displayText(img_, vpImagePoint(15,10), "Head should go in zero position", vpColor::red);
      if (error < vpMath::rad(4) && status_door_handle)
      {
        if (savePose)
        {
          vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to save the offset between the hand and the handle", vpColor::red);
          vpDisplay::displayText(img_, vpImagePoint(30,10), "Middle click to keep the actual pose of the handle", vpColor::red);
          if (click_done && button == vpMouseButton::button1 && once == 0)
          {
            state = SaveOffset;
            click_done = false;
          }
          if(button == vpMouseButton::button2)
          {
            //Keep the door handle pose fixed
            pose_door_handle_fixed = cMdh;
            pose_handle_fixed = true;
          }
        }
        else
          state = MoveRightArm;
        ROS_INFO("Head is in zero position");
      }
//      else
//        std::cout << "error = " << error << "  status door handle : " << status_door_handle << std::endl;
    }
    if (state == MoveRightArm)
    {
//      ROS_INFO("Right Arm should go in zero position");
      if (!once)
        vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to move the Right Arm in zero position", vpColor::red);
      else
        vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to servo the RArm near the door handle", vpColor::red);

      if (click_done && button == vpMouseButton::button1 && once == 0) {
//        state = GoBacktoResPosition;
        moveRArmFromRestPosition(); // move up
        once = 1;
        click_done = false;
      }
      if (click_done && button == vpMouseButton::button1 && once == 1) {
        state = VisualServoRHand;
        once = 0;
        click_done = false;
      }

    }
    if (state == VisualServoRHand )
    {
      if (status_hand && status_door_handle)
      {
        //Start the pbvs node
        start_pbvs = true;
        computeControlLaw(dhMoffsetInitial);
        pbvs_finished = false;
        state = WaitRHandServoed;
//        ROS_INFO("Right Arm should be servoed to the door handle with an offset");

      }
      else {
        vpDisplay::displayText(img_, vpImagePoint(15,10), "The servoing cannot be done as Romeo cannot", vpColor::red);
        vpDisplay::displayText(img_, vpImagePoint(30,10), "see either the door handle or his hand", vpColor::red);
      }


    }
    if (state == WaitRHandServoed)
    {
      if (pbvs_finished)
      {
        //Stop the pbvs node
        start_pbvs = false;
        vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to put the hand on the door handle", vpColor::red);
        if (click_done && button == vpMouseButton::button1 ) {
          state = PutHandOnDoorHandle1;
          click_done = false;
          once = 0;
        }
        if (!once)
        {
          ROS_INFO("Right Arm should have finished to be servoed");
          once = 1;
        }

      }
      else
      {
        start_pbvs = true;
        computeControlLaw(dhMoffsetInitial);
        vpDisplay::displayText(img_, vpImagePoint(15,10), "The servoing is not finished", vpColor::red);

      }


    }
    if (state == SecondPBVS )
    {
      if (status_hand && pose_handle_fixed)
      {
        //Start the pbvs node
        start_pbvs = true;
        computeControlLaw(dhMoffsetFinal);
        pbvs_finished = false;
        state = WaitRHandServoedSecond;

      }
      else {
        vpDisplay::displayText(img_, vpImagePoint(15,10), "The servoing cannot be done as Romeo cannot", vpColor::red);
        vpDisplay::displayText(img_, vpImagePoint(30,10), "see his hand or the door handle is not fixed", vpColor::red);
      }


    }
    if (state == WaitRHandServoedSecond)
    {
      if (pbvs_finished)
      {
        //Stop the pbvs node
        start_pbvs = false;
        vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to graps the door handle", vpColor::red);
        if (click_done && button == vpMouseButton::button1 ) {
          state = GraspingDoorHandle;
          click_done = false;
          once = 0;
        }
        if (!once)
        {
          ROS_INFO("Right Arm should have finished to be servoed");
          once = 1;
        }

      }
      else
      {
        start_pbvs = true;
        computeControlLaw(dhMoffsetFinal);
        vpDisplay::displayText(img_, vpImagePoint(15,10), "The servoing is not finished", vpColor::red);

      }


    }
    if (state == PutHandOnDoorHandle1)
    {
      // Open loop upward motion of the hand
      vpColVector cart_delta_pos(6, 0);
//      cart_delta_pos[1] = 0.02;
      cart_delta_pos[3] = vpMath::rad(-7);
      double delta_t = 1;


      static vpCartesianDisplacement moveCartesian;
      vpVelocityTwistMatrix V;
      if (moveCartesian.computeVelocity(romeo, cart_delta_pos, delta_t, "RArm", V)) {
        vpDisplay::displayText(img_, vpImagePoint(15,10), "The Right Hand should get close to the door handle", vpColor::red);
        romeo.setVelocity(moveCartesian.getJointNames(), moveCartesian.getJointVelocity());
//        ROS_INFO("Right Arm should go in openLoop to open the door");
      }
      else
      {
        romeo.stop(moveCartesian.getJointNames());
//        vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to do next step", vpColor::red);
//        if (click_done && button == vpMouseButton::button1 ) {
          state = PutHandOnDoorHandle2;
//          click_done = false;
          once = 0;
//        }
        if (!once)
        {
          ROS_INFO("Right Arm should have finished to open the door");
          once = 1;
        }
      }

    }
    if (state == PutHandOnDoorHandle2)
    {
        romeo.getProxy()->setStiffnesses("RHand", 1.0f);
        AL::ALValue angle = 0.70;
        romeo.getProxy()->setAngles("RHand", angle, 0.50);
        vpTime::sleepMs(100);

        //Open loop upward motion of the hand
        vpColVector cart_delta_pos(6, 0);
        cart_delta_pos[5] = vpMath::rad(-5);
        cart_delta_pos[1] = 0.10;
        cart_delta_pos[2] = -0.025;
        double delta_t = 3;


        static vpCartesianDisplacement moveCartesian;
        vpVelocityTwistMatrix V;
        if (moveCartesian.computeVelocity(romeo, cart_delta_pos, delta_t, "RArm", V)) {
          romeo.setVelocity(moveCartesian.getJointNames(), moveCartesian.getJointVelocity());
          //        ROS_INFO("Right Arm should go in openLoop to open the door");
        }
        else
        {
          romeo.stop(moveCartesian.getJointNames());
          vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to grasp the door handle", vpColor::red);
          if (click_done && button == vpMouseButton::button1 ) {
            state = GraspingDoorHandle;
            click_done = false;
            once = 0;
          }
          ROS_INFO("Right Arm should have put the hand on the door handle");
        }

    }
    if (state == GraspingDoorHandle)
    {
      romeo.getProxy()->setStiffnesses("RHand", 1.0f);
      AL::ALValue angle = 0.30;
      romeo.getProxy()->setAngles("RHand", angle, 0.30);
      vpTime::sleepMs(100);
      state = PutTheHandCloser;
      ROS_INFO("Right Arm should have grasped the handle");
    }
    if (state == PutTheHandCloser)
    {
      // Open loop motion of the hand to put the hand of Romeo nearer to the door
      vpColVector cart_delta_pos(6, 0);
      cart_delta_pos[0] = 0.02;
      double delta_t = 2;


      static vpCartesianDisplacement moveCartesian;
      vpVelocityTwistMatrix V;
      if (moveCartesian.computeVelocity(romeo, cart_delta_pos, delta_t, "RArm", V)) {
        romeo.setVelocity(moveCartesian.getJointNames(), moveCartesian.getJointVelocity());
//        ROS_INFO("Right Arm should go in openLoop to open the door");
      }
      else
      {
        romeo.stop(moveCartesian.getJointNames());
        state = GraspingDoorHandle2;
        ROS_INFO("Right Arm should have finished to open the door");
      }
    }
    if (state == GraspingDoorHandle2)
    {
      romeo.getProxy()->setStiffnesses("RHand", 1.0f);
      AL::ALValue angle = 0.15;
      romeo.getProxy()->setAngles("RHand", angle, 0.30);
      vpTime::sleepMs(100);
      state = PutTheHandCloser2;
      ROS_INFO("Right Arm should have grasped the handle");
    }
    if (state == PutTheHandCloser2)
    {
      // Open loop upward motion of the hand
      vpColVector cart_delta_pos(6, 0);
      cart_delta_pos[0] = 0.01;
      cart_delta_pos[1] = 0.01;
      double delta_t = 2;


      static vpCartesianDisplacement moveCartesian;
      vpVelocityTwistMatrix V;
      if (moveCartesian.computeVelocity(romeo, cart_delta_pos, delta_t, "RArm", V)) {
        romeo.setVelocity(moveCartesian.getJointNames(), moveCartesian.getJointVelocity());
//        ROS_INFO("Right Arm should go in openLoop to open the door");
      }
      else
      {
        romeo.stop(moveCartesian.getJointNames());
        state = GraspingDoorHandle3;
        ROS_INFO("Right Arm should have finished to open the door");
      }
    }
    if (state == GraspingDoorHandle3)
    {
      if (!once) {
        romeo.getProxy()->setStiffnesses("RHand", 1.0f);
        AL::ALValue angle = 0.00;
        romeo.getProxy()->setAngles("RHand", angle, 0.30);
        vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to rotate the door handle", vpColor::red);
        vpTime::sleepMs(1000);
        once = 1;
      }
      vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to rotate the door handle", vpColor::red);
      if (click_done && button == vpMouseButton::button1 ) {
        state = RotatingHandle;
        once = 0;
        click_done = false;
      }
//      ROS_INFO("Right Arm should have grasped the handle");
    }
    if (state == RotatingHandle)
    {/*
      vpHomogeneousMatrix hMp;
      for (int i = 0; i < 3; i++)
        hMp[i][i] = 0;

      hMp[1][0] = -1;
      hMp[2][1] = 1;
      hMp[0][2] = -1;
      hMp[0][3] = 0.20;  //15
      hMp[1][3] = 0.05;
      hMp[2][3] = -0.02;


      vpVelocityTwistMatrix wVh(hMp);
      vpColVector v_wrist, v_handle(6,0);
      v_handle[5] = -vpMath::rad(10);
      v_wrist = wVh * v_handle;
      ROS_INFO_STREAM("hMp" << hMp << "wVh" << wVh << "v_wrist" << v_wrist);
      static double t_1 = vpTime::measureTimeSecond();
      if ( vpTime::measureTimeSecond() - t_1 < 6.0)
      {
         vpColVector q = romeo.get_eJe("RArm").pseudoInverse() * v_wrist;
//         ROS_INFO_STREAM("q = " << q << " v_wrist = " << v_wrist);
         std::vector<std::string> jointNames = romeo.getBodyNames("RArm");
         jointNames.pop_back(); // We don't consider  the last joint of the arm = Hand
         romeo.setVelocity(jointNames, q);
      }*/

      // Open loop motion of the hand to rotate the handle
      vpColVector cart_delta_pos(6, 0);
      cart_delta_pos[3] = vpMath::rad(+40);
      cart_delta_pos[2] = -0.07;
      double delta_t = 3;


      static vpCartesianDisplacement moveCartesian;
      vpVelocityTwistMatrix V;
      if (moveCartesian.computeVelocity(romeo, cart_delta_pos, delta_t, "RArm", V)) {
        romeo.setVelocity(moveCartesian.getJointNames(), moveCartesian.getJointVelocity());
//        ROS_INFO("Right Arm should go in openLoop to open the door");
      }

      else
      {
        romeo.stop(jointNames_tot);
        vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to open the door", vpColor::red);
        if (click_done && button == vpMouseButton::button1 ) {
          state = OpenDoor;
          click_done = false;
        }
        ROS_INFO("Right Arm should have opened the door");
      }
    }
    if (state == OpenDoor)
    {
      // Open loop motion of the hand to open the door
      vpColVector cart_delta_pos(6, 0);
      cart_delta_pos[0] = -0.10;
      cart_delta_pos[4] = vpMath::rad(+15);
//      cart_delta_pos[2] = 0.01;
      double delta_t = 3;


      static vpCartesianDisplacement moveCartesian;
      vpVelocityTwistMatrix V;
      if (moveCartesian.computeVelocity(romeo, cart_delta_pos, delta_t, "RArm", V)) {
        romeo.setVelocity(moveCartesian.getJointNames(), moveCartesian.getJointVelocity());
//        ROS_INFO("Right Arm should go in openLoop to open the door");
      }
      else
      {
        romeo.stop(moveCartesian.getJointNames());
        vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to close the door", vpColor::red);
        vpDisplay::displayText(img_, vpImagePoint(30,10), "Middle kick to release the door", vpColor::red);
        if (click_done && button == vpMouseButton::button1 ) {
          state = CloseDoor;
          click_done = false;
        }
        if (click_done && button == vpMouseButton::button2 ) {
          state = ReleaseDoorHandle;
          click_done = false;
        }
        ROS_INFO("Right Arm should have finished to open the door");
      }
    }
    if (state == CloseDoor)
    {
      // Open loop upward motion of the hand
      vpColVector cart_delta_pos(6, 0);
      cart_delta_pos[0] = 0.10;
      cart_delta_pos[1] = 0.03;
      double delta_t = 3;


      static vpCartesianDisplacement moveCartesian;
      vpVelocityTwistMatrix V;
      if (moveCartesian.computeVelocity(romeo, cart_delta_pos, delta_t, "RArm", V)) {
        romeo.setVelocity(moveCartesian.getJointNames(), moveCartesian.getJointVelocity());
//        ROS_INFO("Right Arm should go in openLoop to open the door");
      }
      else
      {
        romeo.stop(moveCartesian.getJointNames());
        vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to release the handle", vpColor::red);
        if (click_done && button == vpMouseButton::button1 ) {
          state = ReleaseDoorHandle;
          click_done = false;
        }
        ROS_INFO("Right Arm should have finished to open the door");
      }
    }

    if (state == ReleaseDoorHandle)
    {
      romeo.getProxy()->setStiffnesses("RHand", 1.0f);
      AL::ALValue angle = 0.5;
      romeo.getProxy()->setAngles("RHand", angle, 0.1);
      moveRArmToRestPosition();

//      vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to go away from the handle", vpColor::red);
//      if (click_done && button == vpMouseButton::button1 ) {
        state = GoBacktoResPosition;
//        click_done = false;
//      }
      ROS_INFO("Right Arm should have released the handle");
    }
    if (state == GetAwayFromHandle)
    {
      std::vector<float> RArmElbowWrist_pos;
      std::vector<std::string> RArmElbowWrist_name;
      RArmElbowWrist_name.push_back("RElbowYaw");
      RArmElbowWrist_name.push_back("RWristPitch");

      RArmElbowWrist_pos = romeo.getProxy()->getAngles(RArmElbowWrist_name, true);

      RArmElbowWrist_pos[0] -= vpMath::rad(+11.);
      RArmElbowWrist_pos[1] -= vpMath::rad(+5.);

      romeo.getProxy()->setAngles(RArmElbowWrist_name, RArmElbowWrist_pos, 0.02);

      once = 0;

//      vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to quit the demo", vpColor::red);
//      if (click_done && button == vpMouseButton::button1 ) {
        state = GoBacktoResPosition;
//        click_done = false;
//      }
      ROS_INFO("Right Arm should have finished to open the door");
    }
    if (state == GoBacktoResPosition)
    {
//      ROS_INFO("Right Arm should go in zero position");
      if (once == 0)
        vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to open the fingers", vpColor::red);
      else
        vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to put back the RArm in rest position", vpColor::red);

//      if (click_done && button == vpMouseButton::button1 && once == 0) {
//        once = 1;
//        click_done = false;
//      }
      if (click_done && button == vpMouseButton::button1 && once == 0) {
        romeo.getProxy()->setStiffnesses("RHand", 1.0f);
        AL::ALValue angle = 1.0;
        romeo.getProxy()->setAngles("RHand", angle, 1.);
        state = DoorOpenedAndQuit;
//        once = 0;
        click_done = false;
      }

    }

    if (state == DoorOpenedAndQuit)
    {
      state_door_handle_sub.shutdown();
      state_hand_sub.shutdown();
      vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to quit the demo", vpColor::red);
      if (click_done && button == vpMouseButton::button1 ) {
        break;
        click_done = false;
      }

    }
    if (state == SaveOffset)
    {
      saveOffset();
    }

    ros::spinOnce();
    loop_rate.sleep();
    vpDisplay::displayText(img_, vpImagePoint(460,10), "Right click to exit the demo", vpColor::green);
    vpDisplay::flush(img_);
  }
  ROS_INFO("End of spin");
//  ros::shutdown();
}

void DemoRomeoDoor::moveRArmFromRestPosition ()
{

  try
  {

//    AL::ALValue pos1 = AL::ALValue::array(0.20587599277496338, -0.4126491844654083, -0.18764731287956238, 1.0564998388290405, 0.6922455430030823, -0.44560280442237854);
//    AL::ALValue pos2 = AL::ALValue::array(0.29286253452301025, -0.38179177045822144, -0.09054779261350632, 1.2106484174728394, 0.13958579301834106, -0.011433127336204052);
//    AL::ALValue pos3 = AL::ALValue::array(0.3158690333366394, -0.3941819667816162, 0.0980803444981575, 1.2527700662612915, -0.028855890035629272, 0.19667623937129974);

    //New ones
//    AL::ALValue pos1 = AL::ALValue::array(0.2581080198287964, -0.371820867061615, -0.17735163867473602, 1.582729697227478, 0.5320466756820679, 0.17236517369747162);
//    AL::ALValue pos2 = AL::ALValue::array(0.3097272515296936, -0.35210996866226196, -0.003855433315038681, 1.2893942594528198, 0.10979358851909637, 0.1836995631456375);
//    AL::ALValue pos3 = AL::ALValue::array(0.31288379430770874, -0.32327330112457275, 0.038474030792713165, 1.4297363758087158, -0.06524830311536789, 0.14893878996372223);

//    AL::ALValue pos3 = AL::ALValue::array(0.3689846992492676, -0.20184889435768127, 0.08642885833978653, 1.496846318244934, -0.3719552457332611, 0.3171626925468445);
    AL::ALValue pos1 = AL::ALValue::array(0.3741794228553772, -0.33311545848846436, -0.036883752793073654, 1.260278582572937, 0.4322237968444824, 0.009434251114726067);
    AL::ALValue pos2 = AL::ALValue::array(0.39718443155288696, -0.282814621925354, 0.17343932390213013, 1.194741129875183, -0.5093367099761963, 0.18652880191802979);
    AL::ALValue pos3 = AL::ALValue::array(0.3460591435432434, -0.2849748432636261, 0.08855552971363068, 0.9453462958335876, -0.11968476325273514, 0.25619229674339294);

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

void DemoRomeoDoor::moveRArmToRestPosition ()
{

  try
  {

    AL::ALValue pos1 = AL::ALValue::array(0.3373715579509735, -0.23838065564632416, 0.0157206729054451, 1.4386650323867798, -0.1224169209599495, 0.8254975080490112);
    AL::ALValue pos2 = AL::ALValue::array(0.2764400839805603, -0.3419274687767029, -0.1975732147693634, 1.102345585823059, 1.1167892217636108, 0.14908771216869354);
//    AL::ALValue pos3 = AL::ALValue::array(0.2911868691444397, -0.3036026656627655, -0.19023677706718445, 0.7319300770759583, 0.8177691698074341, -0.13333836197853088);
    AL::ALValue pos3 = AL::ALValue::array(0.117273710668087, -0.22325754165649414, -0.3122972846031189, 1.6027156114578247, 1.20582115650177, 0.5312549471855164);

// Pose to move back from a closed door
//    AL::ALValue pos1 = AL::ALValue::array(0.3924088776111603, -0.18414883315563202, 0.053752146661281586, 1.065794825553894, -0.030273856595158577, 0.6450750827789307);
//    AL::ALValue pos2 = AL::ALValue::array(0.3896014392375946, -0.2007695585489273, 0.08273855596780777, 1.154071569442749, -0.3156268894672394, 0.21828164160251617);
//    AL::ALValue pos3 = AL::ALValue::array(0.2911868691444397, -0.3036026656627655, -0.19023677706718445, 0.7319300770759583, 0.8177691698074341, -0.13333836197853088);
//    AL::ALValue pos4 = AL::ALValue::array(0.117273710668087, -0.22325754165649414, -0.3122972846031189, 1.6027156114578247, 1.20582115650177, 0.5312549471855164);

    AL::ALValue time1 = 2.0f;
    AL::ALValue time2 = 4.0f;
    AL::ALValue time3 = 6.0f;
//    AL::ALValue time4 = 8.0f;



    AL::ALValue path;
    path.arrayPush(pos1);
    path.arrayPush(pos2);
    path.arrayPush(pos3);
//    path.arrayPush(pos4);


    AL::ALValue times;
    times.arrayPush(time1);
    times.arrayPush(time2);
    times.arrayPush(time3);
//    times.arrayPush(time4);

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

void DemoRomeoDoor::initDisplay()
{
    //init graphical interface
    disp = new vpDisplayX();
    disp->init(img_);
    disp->setTitle("Image viewer");
    vpDisplay::flush(img_);
    vpDisplay::display(img_);
    ROS_INFO("Initialisation done");
    vpDisplay::flush(img_);

    return;
}

void DemoRomeoDoor::computeControlLaw(const vpHomogeneousMatrix &doorhandleMoffset)
{
    vpHomogeneousMatrix currentFeature;
    vpHomogeneousMatrix cMhandle_des;
    vpRotationMatrix cRh;
    vpTranslationVector cTh;
    geometry_msgs::Pose cMh_msg;
    tf::Transform transformdh;
    static tf::TransformBroadcaster br;

//    std::cout << cMh_isInitialized << "  " << cMdh_isInitialized  << "  " <<  statusPoseHand << "  " <<  statusPoseDesired << "  " << start_pbvs << std::endl;
    if ( status_hand && status_door_handle && start_pbvs == 1)
    {
        static bool first_time = true;
        if (first_time) {
            std::cout << "-- Start visual servoing of the arm" << std::endl;
            servo_time_init = vpTime::measureTimeSecond();
            first_time = false;
        }
        vpAdaptiveGain lambda(1.5, 0.12, 8);
        servo_arm.setLambda(lambda);
        servo_arm.set_eJe(romeo.get_eJe(chain_name));
        currentFeature = doorhandleMoffset.inverse() * cMdh.inverse() * cMh;
        cMhandle_des = cMdh * doorhandleMoffset;

        ////Publish the TF BEGIN////
        transformdh.setOrigin( tf::Vector3(cMhandle_des[0][3], cMhandle_des[1][3], cMhandle_des[2][3] ));
        cTh = cMhandle_des.getTranslationVector();
        cRh = cMhandle_des.getRotationMatrix();
//        intern_cMh = vpHomogeneousMatrix(cTh, cRh);
        cMh_msg = visp_bridge::toGeometryMsgsPose(vpHomogeneousMatrix(cTh, cRh));

        tf::Quaternion qdh;
        qdh.setX(cMh_msg.orientation.x);
        qdh.setY(cMh_msg.orientation.y);
        qdh.setZ(cMh_msg.orientation.z);
        qdh.setW(cMh_msg.orientation.w);

        transformdh.setRotation(qdh);
        br.sendTransform(tf::StampedTransform(transformdh, ros::Time::now(), "SR300_rgb_optical_frame", "desired_pose_tf"));
        ////Publish the TF END////
        servo_arm.setCurrentFeature(currentFeature) ;
        // Create twist matrix from target Frame to Arm end-effector (WristPitch)
        vpVelocityTwistMatrix oVe_LArm(oMe_Arm);
        servo_arm.m_task.set_cVe(oVe_LArm);

        //Compute velocities PBVS task
        q_dot = - servo_arm.computeControlLaw(vpTime::measureTimeSecond() - servo_time_init);

        q = romeo.getPosition(jointNames_arm);
        q2_dot  = servo_arm.m_task.secondaryTaskJointLimitAvoidance(q, q_dot, jointMin, jointMax);

//        vpDisplay::displayFrame(img_, cMh, )
        publishCmdVel(q_dot + q2_dot);

        vpTranslationVector t_error_grasp = currentFeature.getTranslationVector();
        vpRotationMatrix R_error_grasp;
        currentFeature.extract(R_error_grasp);
        vpThetaUVector tu_error_grasp;
        tu_error_grasp.buildFrom(R_error_grasp);
        double theta_error_grasp;
        vpColVector u_error_grasp;
        tu_error_grasp.extract(theta_error_grasp, u_error_grasp);
        double error_t_treshold = 0.001;

        init = false;

        if ( (sqrt(t_error_grasp.sumSquare()) < error_t_treshold) && (theta_error_grasp < vpMath::rad(3)) )
        {
          pbvs_finished = true;
          vpColVector q_dot_zero(numJoints,0);
          publishCmdVel(q_dot_zero);
        }
        std::cout << "We are servoing the arm " << start_pbvs << std::endl;
    }
    else if (!init && start_pbvs == 0)
    {
      init = true;
      vpColVector q_dot_zero(numJoints,0);
      publishCmdVel(q_dot_zero);
      std::cout << "publishing just once" << std::endl;
    }
    else if ( start_pbvs == 1 &&( status_hand == 0 || status_door_handle == 0) )
    {
      vpColVector q_dot_zero(numJoints,0);
      publishCmdVel(q_dot_zero);
    }

}

void DemoRomeoDoor::publishCmdVel(const vpColVector &q)
{
    for (int i = 0; i < q.size(); i++)
    {
        q_dot_msg.velocity[i] = q[i];
    }

    cmdVelPub.publish(q_dot_msg);

}


void DemoRomeoDoor::getDesiredPoseCb(const geometry_msgs::PoseStamped::ConstPtr &desiredPose)
{
  cMdh = visp_bridge::toVispHomogeneousMatrix(desiredPose->pose);
}


void DemoRomeoDoor::getActualPoseCb(const geometry_msgs::PoseStamped::ConstPtr &actualPose)
{
  cMh = visp_bridge::toVispHomogeneousMatrix(actualPose->pose);
}

void DemoRomeoDoor::getStatusDoorHandleCB(const std_msgs::Int8ConstPtr &status_dh)
{
  status_door_handle = status_dh->data;
}

void DemoRomeoDoor::getStatusHandCB(const std_msgs::Int8ConstPtr &status)
{
  status_hand = status->data;
}

void DemoRomeoDoor::saveOffset()
{
    vpHomogeneousMatrix dhMoffset;
    if (pose_handle_fixed)
      dhMoffset = pose_door_handle_fixed.inverse() * cMh;
    else
      dhMoffset = cMdh.inverse() * cMh;
    vpXmlParserHomogeneousMatrix xml;

    if ( pose_handle_fixed )
    {
      if ( status_hand && status_door_handle )
      {
        ROS_INFO_STREAM("cMdh = " << cMdh << "\n cMh = " << cMh << "\n dhMh = " << dhMoffset);
        if( xml.save(dhMoffset, offsetFileName.c_str(), offsetFinalName) == vpXmlParserHomogeneousMatrix::SEQUENCE_OK )
          std::cout << "Pose between the hand and the object successfully saved in \"" << offsetFileName << "\"" << std::endl;
        else {
          std::cout << "Failed to save the pose in \"" << offsetFileName << "\"" << std::endl;
          std::cout << "A file with the same name exists. Remove it to be able to save the parameters..." << std::endl;
        }
        ros::shutdown();
      }
    }
    else
    {
      if ( status_hand && status_door_handle )
      {
        ROS_INFO_STREAM("cMdh = " << cMdh << "\n cMh = " << cMh << "\n dhMh = " << dhMoffset);
        if( xml.save(dhMoffset, offsetFileName.c_str(), offsetInitialName) == vpXmlParserHomogeneousMatrix::SEQUENCE_OK )
          std::cout << "Pose between the hand and the object successfully saved in \"" << offsetFileName << "\"" << std::endl;
        else {
          std::cout << "Failed to save the pose in \"" << offsetFileName << "\"" << std::endl;
          std::cout << "A file with the same name exists. Remove it to be able to save the parameters..." << std::endl;
        }
        ros::shutdown();
      }
    }
}

void DemoRomeoDoor::setupCameraParameters(const sensor_msgs::CameraInfoConstPtr &cam_rgb)
{
  //init m_camera parameters
  cam_ = visp_bridge::toVispCameraParameters(*cam_rgb);
  ROS_INFO_STREAM("Camera param  = \n" << cam_rgb);

  cam_rgb_info_sub.shutdown();
}

