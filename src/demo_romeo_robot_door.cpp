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

  img_.init(240,320);
  img_ = 0;
  initDisplay();

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
        //Stop the pbvs node
        pbvs_active.data = 2;
        pbvs_active_pub.publish(pbvs_active);
        ros::shutdown();

    }

    if (state == HeadToZero)
    {
      //OpenLoop Control to see the door handle
      head_pose = 0;
      head_pose[0] = vpMath::rad(-24.3); // NeckYaw
      head_pose[1] = vpMath::rad(20.2); // NeckPitch
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
        pbvs_active.data = 1;
        pbvs_active_pub.publish(pbvs_active);
        state = WaitRHandServoed;
//        next_step = false;
//        ROS_INFO("Right Arm should be servoed to the door handle with an offset");

      }
      else {
        vpDisplay::displayText(img_, vpImagePoint(15,10), "The servoing cannot be done as Romeo cannot", vpColor::red);
        vpDisplay::displayText(img_, vpImagePoint(30,10), "see either the door handle or his hand", vpColor::red);
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
        vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to put the hand on the door handle", vpColor::red);
        if (click_done && button == vpMouseButton::button1 ) {
          state = PutHandOnDoorHandle1;
          click_done = false;
        }
        ROS_INFO("Right Arm should have finished to be servoed");
        vpTime::sleepMs(100);
      }
      else
        vpDisplay::displayText(img_, vpImagePoint(15,10), "The servoing is not finished", vpColor::red);


    }
    if (state == PutHandOnDoorHandle1)
    {
      // Open loop upward motion of the hand
      vpColVector cart_delta_pos(6, 0);
//      cart_delta_pos[1] = 0.02;
      cart_delta_pos[3] = vpMath::rad(-5);
      double delta_t = 3;


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
        vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to do next step", vpColor::red);
        if (click_done && button == vpMouseButton::button1 ) {
          state = PutHandOnDoorHandle2;
          click_done = false;
        }
        ROS_INFO("Right Arm should have finished to open the door");
      }

    }
    if (state == PutHandOnDoorHandle2)
    {
//      if (!once)
//      {
//        // Open loop upward motion of the hand
//        vpColVector cart_delta_pos(6, 0);
//        //      cart_delta_pos[0] = 0.04;
//        //      cart_delta_pos[1] = 0.065;
//        //      cart_delta_pos[2] = +0.015;
//        double delta_t = 3;


//        static vpCartesianDisplacement moveCartesian;
//        vpVelocityTwistMatrix V;
//        if (moveCartesian.computeVelocity(romeo, cart_delta_pos, delta_t, "RArm", V)) {
//          romeo.setVelocity(moveCartesian.getJointNames(), moveCartesian.getJointVelocity());
//          //        ROS_INFO("Right Arm should go in openLoop to open the door");
//        }
//        else
//        {
//          romeo.stop(moveCartesian.getJointNames());
//          vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to grasp the door handle", vpColor::red);
//          once = 1;
//          ROS_INFO("Right Arm should have put the hand on the door handle");
//        }
//      }
//      else if (once == 1) {

        // Second open loop upward motion of the hand
        vpColVector cart_delta_pos2(6, 0);
//        cart_delta_pos2[0] = 0.04;
//        cart_delta_pos2[0] = 0.02;
        cart_delta_pos2[5] = vpMath::rad(-5);
        cart_delta_pos2[1] = 0.11;
        cart_delta_pos2[2] = -0.025;
        double delta_t2 = 3;


        static vpCartesianDisplacement moveCartesian2;
        vpVelocityTwistMatrix V2;
        if (moveCartesian2.computeVelocity(romeo, cart_delta_pos2, delta_t2, "RArm", V2)) {
          romeo.setVelocity(moveCartesian2.getJointNames(), moveCartesian2.getJointVelocity());
          //        ROS_INFO("Right Arm should go in openLoop to open the door");
        }
        else
        {
          romeo.stop(moveCartesian2.getJointNames());
          vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to grasp the door handle", vpColor::red);
          if (click_done && button == vpMouseButton::button1 ) {
            state = GraspingDoorHandle;
            click_done = false;
            once = 0;
          }
          ROS_INFO("Right Arm should have put the hand on the door handle");
        }
//      }

    }
    if (state == GraspingDoorHandle)
    {
      romeo.getProxy()->setStiffnesses("RHand", 1.0f);
      AL::ALValue angle = 0.50;
      romeo.getProxy()->setAngles("RHand", angle, 0.30);
      vpTime::sleepMs(2000);
      state = PutTheHandCloser;
      ROS_INFO("Right Arm should have grasped the handle");
    }
    if (state == PutTheHandCloser)
    {
      // Open loop upward motion of the hand
      vpColVector cart_delta_pos(6, 0);
      cart_delta_pos[0] = 0.02;
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
        state = GraspingDoorHandle2;
        ROS_INFO("Right Arm should have finished to open the door");
      }
    }
    if (state == GraspingDoorHandle2)
    {
      romeo.getProxy()->setStiffnesses("RHand", 1.0f);
      AL::ALValue angle = 0.30;
      romeo.getProxy()->setAngles("RHand", angle, 0.30);
      vpTime::sleepMs(2000);
      state = PutTheHandCloser2;
      ROS_INFO("Right Arm should have grasped the handle");
    }
    if (state == PutTheHandCloser2)
    {
      // Open loop upward motion of the hand
      vpColVector cart_delta_pos(6, 0);
      cart_delta_pos[0] = 0.02;
      cart_delta_pos[1] = 0.01;
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
        state = GraspingDoorHandle3;
        ROS_INFO("Right Arm should have finished to open the door");
      }
    }
    if (state == GraspingDoorHandle3)
    {
      if (!once) {
        romeo.getProxy()->setStiffnesses("RHand", 1.0f);
        AL::ALValue angle = 0.15;
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

      // Open loop upward motion of the hand
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
      // Open loop upward motion of the hand
      vpColVector cart_delta_pos(6, 0);
      cart_delta_pos[0] = -0.10;
      cart_delta_pos[1] = -0.03;
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
        if (click_done && button == vpMouseButton::button1 ) {
          state = CloseDoor;
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
      AL::ALValue angle = 0.25;
      romeo.getProxy()->setAngles("RHand", angle, 0.15);
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
        vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to put back the RArm in rest position", vpColor::red);
      else
        vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to open the fingers", vpColor::red);

      if (click_done && button == vpMouseButton::button1 && once == 0) {
        moveRArmToRestPosition();
        once = 1;
        click_done = false;
      }
      if (click_done && button == vpMouseButton::button1 && once == 1) {
        romeo.getProxy()->setStiffnesses("RHand", 1.0f);
        AL::ALValue angle = 1.0;
        romeo.getProxy()->setAngles("RHand", angle, 1.);
        state = DoorOpenedAndQuit;
        once = 0;
        click_done = false;
      }

    }

    if (state == DoorOpenedAndQuit)
    {
      state_door_handle_sub.shutdown();
      state_hand_sub.shutdown();
      pbvs_finished_sub.shutdown();
      vpDisplay::displayText(img_, vpImagePoint(15,10), "Left click to quit the demo", vpColor::red);
      if (click_done && button == vpMouseButton::button1 ) {
        break;
        click_done = false;
      }

    }

    ros::spinOnce();
    loop_rate.sleep();
    vpDisplay::displayText(img_, vpImagePoint(220,10), "Right click to exit the demo", vpColor::green);
    vpDisplay::flush(img_);
  }
  ROS_INFO("End of spin");
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

    AL::ALValue pos1 = AL::ALValue::array(0.36160808801651, -0.14481818675994873, -0.012668244540691376, 0.9340378642082214, -0.3044275641441345, 0.25746822357177734);
    AL::ALValue pos2 = AL::ALValue::array(0.3901706337928772, -0.26107993721961975, -0.012815611436963081, 0.9594348669052124, 0.11563336104154587, 0.38348156213760376);
    AL::ALValue pos3 = AL::ALValue::array(0.2876889705657959, -0.27668479084968567, -0.22870546579360962, 0.8602628111839294, 0.8492440581321716, 0.10930496454238892);
    AL::ALValue pos4 = AL::ALValue::array(0.117273710668087, -0.22325754165649414, -0.3122972846031189, 1.6027156114578247, 1.20582115650177, 0.5312549471855164);

    AL::ALValue time1 = 2.0f;
    AL::ALValue time2 = 4.0f;
    AL::ALValue time3 = 6.0f;
    AL::ALValue time4 = 8.0f;



    AL::ALValue path;
    path.arrayPush(pos1);
    path.arrayPush(pos2);
    path.arrayPush(pos3);
    path.arrayPush(pos4);


    AL::ALValue times;
    times.arrayPush(time1);
    times.arrayPush(time2);
    times.arrayPush(time3);
    times.arrayPush(time4);

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
