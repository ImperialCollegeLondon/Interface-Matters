# GuMi Scripts
This folder contains dockerfied environment for GuMi. `gumi_ros` package communicates with GuMi and publishes the commands to the robot.  

## EGM
GuMi ROS relies on [Externally Guided Motion (EGM)](https://search.abb.com/library/Download.aspx?DocumentID=3HAC073319-001&DocumentPartId=) from ABB to control YuMi. Check with your local ABB representative to obtain the EGM license for YuMi.
To install the EGM package on the robot controller 
1. Connect YuMi to a Windows device running RobotStudio. Backup the current controller before proceeding.
1. Navigate to Installation->RobotWare->YuMi Controller->Next->Add-> select EGM license file. Be aware that this will remove all existing programs on the robot controller.
1. The robot controller will restart after the installation.

## Configure the drivers for YuMi
GuMi ROS relies on `abb_robot_driver` for communication with the robot. In case this is not completed already, following the steps below to configure the drivers:
1. Install the `state_machine` add-on on the robot controller through RobotStudio. 
   - The `state_machine` add-on was once removed from RoboApps. In case this happens again, download the add-on from [here](https://github.com/ros-industrial/abb_libegm/files/10754679/ABB.StateMachine-1.1.zip).
1. Configure YuMi in RobotStudio 
    - The Communication Protocols: make sure the remote addresses for both arms and UCdevice are the address of the remote machine. Use the same remote port for ROB_L and UCdevice and leave ROB_R alone.
    - Add YuMi gripper tasks to the robot controller. 
        - Find the RAPID code for the gripper tasks in `yumi_gripper_driver/RAPID/`.
        - Follow the [instructions](https://github.com/kth-ros-pkg/yumi/wiki/Firmware#grippers-and-cameras) from KTH to add the gripper tasks to the robot controller (the RAPID codes are adapted from [their repo](https://github.com/kth-ros-pkg/yumi/wiki), please visit for more information about YuMi).

## Calibrate GuMi
The motor positions of the robot need to be calibrated before running the package. The calibration is done by moving the robot to a known position and recording the motor positions. Once you finish, pop the values to the `gumi_ros/scripts/gello.yaml` file. 

## Running the package
1. Pull YuMi drivers:
    `make pull-drivers`
1. Build the Docker image:
    `make build`
1. Build the ROS packages:
    `make compile`
1. Start YuMi positional controller:
    `make pos-controller`
1. Start GELLO interface:
    `make gumi`
    - YuMi will start moving to and stop at the grasp configuration. Now you can check and make sure that GELLO joints are aligned with YuMi. You can hit enter once you are satisfied with the alignment.
    - The robot will move to align with the GELLO model. Once the robot stops, you can hit enter to start the teleoperation.
1. Move the robot back to the grasp position:
    `make yumi-go-grasp`
1. Move the robot to the home position:
    `make yumi-go-home`

<!-- 
# Teleoperation

## Constructing Gello models
TODO: upload the `read_urdf.py` script
1. Use the `read_urdf.py` script to find the shortest link and compare it with the length of the motors
1. Decide on the scale of the model and set the scale in the `read_urdf.py` script
1. Use the `read_urdf.py` script to print the joint Cartesian positions relative to the upper stream joint frame.
    * Set all joints to zero so that it is easier to move from one joint to the next
1. Construct / download models of the motors
1. Set motor positions to the joint positions obtained from the `read_urdf.py` script
    * Double check that the motor axis is the same as the joint axis in RViz
1. Build model holders and align them with the motors
1. Connect the motor holders
1. Add virtual joints to the motor holders

## Gravity Compensation
1. Cannot be done the same as in Gello. YuMi links do not have square angles and the gravity compensation is not as simple as in Gello
1. [Aloha 2](https://aloha-2.github.io/) uses spring balancers to compensate for the gravity as in this [photo](https://aloha-2.github.io/assets/workcell_render.png).
1. The spring balancers they used should be this [AER-RB2](https://www.aero-motivedirect.com/Products/Retractors/?cart_id=UQRuD5LLhNl6haXxaNbuuhn6wPYBaSZ2&user_id=) from Aero-Motive. The company only ships to US addresses. Purchase link [here](https://www.cromwell.co.uk/shop/cutting-tools/tool-balancers/rb2-tool-balancer-0-5-0-9-kg/p/CTL2807800A?msclkid=9d49634bdbd81ab314c205211f29ee12&utm_source=bing&utm_medium=cpc&utm_campaign=(GB%3ASEA)%20DSA%20%3E%20Cutting%20Tools&utm_term=%2Fcutting-tools%2F&utm_content=(GB%3ASEA)%20DSA%20%3E%20Cutting%20Tools) from Cromwell. Datasheet [here](https://www.aero-motivedirect.com/Products/Positioners/PDF/Balancers%20and%20Tool%20Support%20Catalog.pdf)
1. The RB2 balancers are a bit tricky to set up. It needs to be pulled very hard when increasing the tension.

 -->
