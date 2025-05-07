export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://localhost:11311

pull-drivers:
	git -C ${PWD}/gumi_ws/src clone git@github.com:ros-industrial/abb_robot_driver.git
	git -C ${PWD}/gumi_ws/src clone git@github.com:ros-industrial/abb_robot_driver_interfaces.git
	git -C ${PWD}/gumi_ws/src clone git@github.com:ros-industrial/abb_librws.git
	git -C ${PWD}/gumi_ws/src clone git@github.com:ros-industrial/abb_libegm.git
	git -C ${PWD}/gumi_ws/src clone git@github.com:ros-industrial/abb_egm_rws_managers.git
	git -C ${PWD}/gumi_ws/src clone git@github.com:ImperialCollegeLondon/yumi-moveit.git && rm -rf ${PWD}/gumi_ws/src/yumi-moveit/yumi_moveit_config

clean-drivers:
	rm -rf ${PWD}/gumi_ws/src/abb_robot_driver
	rm -rf ${PWD}/gumi_ws/src/abb_robot_driver_interfaces
	rm -rf ${PWD}/gumi_ws/src/abb_librws
	rm -rf ${PWD}/gumi_ws/src/abb_libegm
	rm -rf ${PWD}/gumi_ws/src/abb_egm_rws_managers
	rm -rf ${PWD}/gumi_ws/src/yumi-moveit

build:
	docker build -t prl/gumi .

compile:
	docker container stop gumi | true && docker container rm gumi | true
	docker run \
		-it \
		-e ROS_IP="${ROS_IP}" \
		-e ROS_MASTER_URI="${ROS_MASTER_URI}" \
		-e DISPLAY \
    	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v /dev:/dev \
		-v ${PWD}/gumi_ws:/gumi_ws:rw \
		-v ${PWD}:/gumi_docker:rw \
		--detach \
		--privileged \
		--network host \
		--name gumi \
		prl/gumi
	docker exec gumi bash -c "source /opt/ros/noetic/setup.bash && cd /gumi_ws && catkin build"
	docker container stop gumi

clean-compiled:
	docker start gumi
	docker exec gumi bash -c "source /opt/ros/noetic/setup.bash && cd /gumi_ws && catkin clean -y"
	docker container stop gumi

debug:
	docker start gumi
	docker exec -it gumi bash -c "bash"

stop:
	docker container stop gumi 

pos-controller:
	docker start gumi
	docker exec -it gumi bash -c "source gumi_ws/devel/setup.bash && roslaunch gumi_ros yumi_pos_controller.launch"
	docker container stop gumi

gumi:
	docker start gumi
	docker exec -it gumi bash -c "source gumi_ws/devel/setup.bash && roslaunch gumi_ros gumi_ros.launch"
	docker container stop gumi

yumi-go-grasp:
	docker start gumi
	docker exec -it gumi bash -c "source gumi_ws/devel/setup.bash && roslaunch gumi_ros yumi_go_grasp.launch"
	docker container stop gumi

yumi-go-home:
	docker start gumi
	docker exec -it gumi bash -c "source gumi_ws/devel/setup.bash && roslaunch gumi_ros yumi_go_home.launch"
	docker container stop gumi

motor-on:
	docker start gumi
	docker exec -it gumi bash -c "source devel/setup.bash && rosservice call /yumi/rws/set_motors_on"

motor-off:
	docker start gumi
	docker exec -it gumi bash -c "source devel/setup.bash && rosservice call /yumi/rws/set_motors_off"

get-ee-left:
	rosrun tf tf_echo yumi_base_link yumi_link_7_l
get-ee-right:
	rosrun tf tf_echo yumi_base_link yumi_link_7_r

open-ee-right:
	rostopic pub /yumi/gripper_r_position_cmd std_msgs/Float64 "data: 10.0"
open-ee-left:
	rostopic pub /yumi/gripper_l_position_cmd std_msgs/Float64 "data: 10.0"
close-ee-right:
	rostopic pub /yumi/gripper_r_position_cmd std_msgs/Float64 "data: 0.0"
close-ee-left:
	rostopic pub /yumi/gripper_l_position_cmd std_msgs/Float64 "data: 0.0"