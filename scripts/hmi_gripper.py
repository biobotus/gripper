#!/usr/bin/python

# Imports
import rospy
from std_msgs.msg import String

class HMIGripper():
    def __init__(self):
        # ROS init
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.done = False

        # ROS subscriptions
        self.subscriber = rospy.Subscriber('Done_Move', String, self.callback_done)

        # ROS publishments
        self.gripper_pos = rospy.Publisher('Gripper_Pos', String, queue_size=10)

    def callback_done(self, data):
        if data.data == 'Gripper':
            self.done = True

    def hmi_gripper_pos(self):
        self.run = 1

        while self.run:
            self.done = False
            pos = {}
            angle_dict = {}
            pos[0] = raw_input(" Angle of motor 0: ")
            pos[1] = raw_input(" Angle of motor 1: ")
            pos[2] = raw_input(" Angle of motor 2: ")

            for ID in pos:
                if not pos[ID] == "":
                    angle_dict[ID] = float(pos[ID])
                    if abs(angle_dict[ID])> 150:
                        raise ValueError("Input angle out of limits")
                    
                    
            self.gripper_pos.publish(str(angle_dict))

            while not self.done:
                self.rate.sleep()

            self.run = int(raw_input(" Movement done! Try again? (yes = 1/no = 0) "))


if __name__ == '__main__':
    try:
        h = HMIGripper()
        h.hmi_gripper_pos()

    except (rospy.ROSInterruptException, EOFError, ValueError) as e:
        print(e)

