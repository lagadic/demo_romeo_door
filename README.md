# Demo Romeo open and/or close a door

## Description of the demo step by step:

* Romeo detects a door handle using depth information and localize its pose.
* Open-loop motion of the arm to get close to the door handle
* Blobs tracker automatically detected and computation pose
* Grasping phase:
   * Pose-Based visual servoing to approach the arm to the door handle
   * Open-loop motion of the arm to get on top of the door handle
   * Grasping the door handle
* Open-loop motion of the arm to rotate the door handle
* Romeo open the door
  * Romeo can release the door handle or
  * Romeo close the door and release the door handle
* Open-loop motion to go back in initial position

## Instruction
`$ roslaunch realsense_camera realsense_sr300.launch`
`$ roslaunch door_handle_detection door_handle_detection_realsense.launch`
`$ roslaunch visp_blobs_tracker visp_blobs_tracker_SR300.launch `
`$ roslaunch visp_naoqi_ros  romeo_cmd_vel.launch`
`$ roslaunch demo_romeo_door demo_romeo_door.launch` 
