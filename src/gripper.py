#!/usr/bin/python
import rospy
import ast
from std_msgs.msg import String, Float64
from dynamixel_controllers.srv import SetTorqueLimit, SetSpeed

#from my_dynamixel_tutorial.msg import GripCommand

PI = 3.14159265359

class Gripper:
    def __init__(self,node_name):

        # ROS init
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True, log_level=rospy.INFO)
        self.rate = rospy.Rate(10)  # 10 Hz

        # ROS subscriptions
        self.sub_move_wrist = rospy.Subscriber(self.node_name +'/Wrist_Pos', Float64, self.move_wrist)
        self.sub_grip = rospy.Subscriber(self.node_name +'/Grip', Float64, self.grip)
        self.sub_grip_speed = rospy.Subscriber(self.node_name +'/Grip_Speed', Float64, self.set_grip_speed)

        #self.sub_cmd = rospy.Subscriber('/gripper/command', GripCommand, self.process_grip)

        # ROS publishments
        self.pub_wrist = rospy.Publisher('/wrist_controller/command', Float64, queue_size=1)
        self.pub_grip = rospy.Publisher('/dual_grip_controller/command', Float64, queue_size=1)

        # Motors init (useless, all information are in the controllers...)
        self.baud_rate = 500000
        self.nb_motors = 3
        self.motor_IDs = range(self.nb_motors)

        # Get active controllers...
        if self.init_motors():
            rospy.loginfo("Gripper ready !")
            # Get min and maxmimum raw value for grip motors
            self.grip_min_angle_raw = rospy.get_param('/dual_grip_controller/motor_master/min')
            self.grip_max_angle_raw = rospy.get_param('/dual_grip_controller/motor_master/max')
            self.grip_range_angle_raw = self.grip_max_angle_raw - self.grip_min_angle_raw
            self.ENCODER_RESOLUTION = 1024
            #print self.grip_range_angle_raw
            #print self.grip_max_angle_raw
            #print self.grip_min_angle_raw
        else:
            rospy.logwarn("Error initializing motors...")


    def init_motors(self):

        """This function sets default values to the connected motors."""
        try:
            rospy.loginfo("Validating motor ID [0,1,2] are connected... ")
            available_ids = rospy.get_param('dynamixel/%s/connected_ids' % "ttlUSB0_port", [])
            if available_ids == self.motor_IDs:
                return True
            else:
                return False

        except AssertionError:
            return False

    # Subscriber callbacks
    # ---------------------------------------------------------------------- #
    def set_grip_torque(self, msg):
        # Value is a pecentage between 0 to 1
        torque_grip = msg.data
        # Set the maximum torque for the grip
        rospy.wait_for_service('/dual_grip_controller/set_torque_limit',timeout=2)
        torque_limit_service = rospy.ServiceProxy('/dual_grip_controller/set_torque_limit', SetTorqueLimit)
        try:
            resp1 = torque_limit_service(torque_grip)
        except rospy.ServiceException as exc:
            rospy.logwarn("Service did not process request: " + str(exc))

    def move_wrist(self, msg):
        # Move the wrist motor to the desired pos
        wrist_pos = msg.data
        angle_rad = float(wrist_pos*(PI/180.0))
        self.pub_wrist.publish(angle_rad)

    def grip(self, msg):
        grip_pc = 1.0 - msg.data
        angle_rad = float(grip_pc * self.grip_range_angle_raw + self.grip_min_angle_raw)*((300.0*PI/180.0)/self.ENCODER_RESOLUTION)
        self.pub_grip.publish(angle_rad)

    def set_grip_speed(self,msg):
        speed_val = msg.data
        # Set the speed of the grip
        rospy.wait_for_service('/dual_grip_controller/set_speed',timeout=2)
        speed_service = rospy.ServiceProxy('/dual_grip_controller/set_speed', SetSpeed)
        try:
            resp1 = speed_service(speed_val)
        except rospy.ServiceException as exc:
            rospy.logwarn("Service did not process request: " + str(exc))

    # ---------------------------------------------------------------------- #

    def listener(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        g = Gripper('Gripper')
        g.listener()

    except (AssertionError, rospy.ROSInterruptException) as e:
        rospy.logerr(e)
