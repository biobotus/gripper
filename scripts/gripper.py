#!/usr/bin/python
import ast
import dynamixel
import numbers
import rospy
from std_msgs.msg import String

class Gripper:
    def __init__(self):
        # ROS init
        self.node_name = self.__class__.__name__
        rospy.init_node(self.node_name, anonymous=True, log_level=rospy.INFO)
        self.rate = rospy.Rate(10)  # 10 Hz

        # ROS subscriptions
        self.subscriber = rospy.Subscriber('Gripper_Pos', String, self.callback_pos)

        # ROS publishments
        self.done_move = rospy.Publisher('Done_Move', String, queue_size=10)
        self.error = rospy.Publisher('Error', String, queue_size=10)

        # Motors init
        self.baud_rate = 500000
        self.dir_pin = 18  # pin 12
        # Uses UART pins 8 (TXD) and 10 (RXD)
        self.nb_motors = 3
        self.motor_IDs = range(self.nb_motors)
        self.motor_limit_l = [-91, -1, -1]
        self.motor_limit_h = [91, 63, 63]
        self.default_speed = {0: 10, 1: 10, 2: 10}
        self.motor = dynamixel.ComAX12a(self.baud_rate, self.dir_pin)
        assert self.try_x_times(5, self.init_motors)

    def init_motors(self):
        """This function sets default values to the connected motors."""

        try:
            rospy.loginfo("Initializing motors...")
            for ID in self.motor_IDs:
                rospy.logdebug("Initializing motor {0}".format(ID))
                rospy.logdebug("Ping test")
                assert self.motor.ping(ID)
                self.rate.sleep()

                rospy.logdebug("Set return delay time")
                assert self.motor.set_return_delay_time(ID, 0xfe)
                self.rate.sleep()

                rospy.logdebug("Set return respond level")
                assert self.motor.set_status_return_level(ID, self.motor.RETURN_ALL)
                self.rate.sleep()

                rospy.logdebug("Set punch limit")
                assert self.motor.set_punch_limit(ID, 32)
                self.rate.sleep()

                rospy.logdebug("Set torque values")
                assert self.motor.set_torque_limit(ID, 0.80)
                self.rate.sleep()
                assert self.motor.set_max_torque(ID, 0.80)
                self.rate.sleep()

                rospy.logdebug("Set temperature limit")
                assert self.motor.set_temp_limit(ID, 75)
                self.rate.sleep()

                rospy.logdebug("Set angle limit for all motors")
                assert self.motor.set_angle_limit(ID, self.motor_limit_l[ID], \
                                                      self.motor_limit_h[ID])
                self.rate.sleep()

            rospy.loginfo("Motors sucessfully initialized.")
            return True

        except AssertionError:
            rospy.logwarn("Error initializing motors...")
            return False

    def set_angle(self, angle_dict, speed_dict={0: False, 1: False, 2: False}):
        """
        Sets motors ID to angle at specified speed (RPM) and waits until
        the motor stops moving to exit the function. If no speed is given,
        default speed is used.
        """

        speed = self.default_speed.copy()

        for ID in angle_dict:
            if not ID in self.motor_IDs:
                raise ValueError("Error: Invalid ID received: {0}".format(ID))

            if ID in speed_dict and speed_dict[ID] > 0 and speed_dict[ID] < 100:
                speed[ID] = speed_dict[ID]

        for ID in angle_dict:
            if not self.try_x_times(10, self.motor.set_goal_pos_speed, \
                                         ID, angle_dict[ID], speed[ID]):
                return False
        self.rate.sleep()

        cond = True
        while cond:
            cond = False
            for ID in angle_dict:
                cond = cond or self.motor.read_moving_status(ID)
                self.rate.sleep()

        self.done_move.publish(self.node_name)
        rospy.loginfo("Gripper movement done.")
        return True

    def callback_pos(self, data):
        try:
            angle_dict = ast.literal_eval(data.data)

            assert type(angle_dict) == dict
            if angle_dict and type(angle_dict.keys()[0]) == str:
                angle_dict = dict([(int(k), v) for k, v in angle_dict.items()])

            for ID in angle_dict:
                assert type(ID) == int
                assert isinstance(angle_dict[ID], numbers.Real)
                if angle_dict[ID] < self.motor_limit_l[ID]:
                    angle_dict[ID] = self.motor_limit_l[ID] + 1
                    rospy.logwarn("Warning, requested angle out of limits. " + \
                     "Angle of motor {0} set to {1}".format(ID, angle_dict[ID]))
                elif angle_dict[ID] > self.motor_limit_h[ID]:
                    angle_dict[ID] = self.motor_limit_h[ID] - 1
                    rospy.logwarn("Warning, requested angle out of limits. " + \
                     "Angle of motor {0} set to {1}".format(ID, angle_dict[ID]))

        except (AssertionError, SyntaxError, ValueError):
            rospy.logerr("Invalid value recieved in Gripper_Pos.")
            # self.error.publish(blablabla)
            return

        try:
            self.set_angle(angle_dict)

        except ValueError as e:
            rospy.logerr(e)
            # self.error.publish(blablabla)
            return

    def try_x_times(self, max_tries, func, *args):
        """
        This method tries func(*args) until success, for up to max_tries times.
        The function has to return False for an error or True for a success.
        """
        tries = 0
        while tries < max_tries:
            if func(*args):
                return True
            self.rate.sleep()
            tries += 1
        return False

    def listener(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        g = Gripper()
        g.listener()

    except (AssertionError, rospy.ROSInterruptException) as e:
        rospy.logerr(e)

