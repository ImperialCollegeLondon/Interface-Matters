import rospy
from yumi import YuMi

def main():
    rospy.init_node('yumi_wakeup', anonymous=True)
    yumi = YuMi()
        
    yumi.stop_egm()
    # slow down the speed of the robot
    yumi.change_max_speed(yumi.left_rapid_task, 30) # 5 degrees per second
    yumi.change_max_speed(yumi.right_rapid_task, 30) # 5 degrees per second

    yumi.start_egm()
    yumi.start_controllers()
    print("YuMi ready for teleoperation")


if __name__ == "__main__":
    main()
