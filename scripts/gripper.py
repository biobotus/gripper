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
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)  # 10 Hz

        # ROS subscriptions
        self.subscriber = rospy.Subscriber('Gripper_Pos', String, self.callback_pos)

        # ROS publishments
        self.done_move = rospy.Publisher('Done_Move', String, queue_size=10)
        self.error = rospy.Publisher('Error', String, queue_size=10)

        # Motors init
        self.baud_rate = 1000000
        self.dir_pin = 18  # pin 12
        # Uses UART pins 8 (TXD) and 10 (RXD)
        self.nb_motors = 3
        self.motor_IDs = range(self.nb_motors)
        self.default_speed = {0: 10, 1: 10, 2: 10}
        self.motor = dynamixel.ComAX12a(self.baud_rate, self.dir_pin)
        self.init_motors()

    def init_motors(self):
        """This function sets default values to the connected motors."""

        print("Verify 3 motors are connected : IDs 0, 1 and 2")
        assert self.motor.servo_census() == self.motor_IDs
        self.rate.sleep()

        for ID in self.motor_IDs:
            print("Initializing motor {0}".format(ID))
            print("Set return delay time")
            assert self.motor.set_return_delay_time(ID, 0xfe)
            self.rate.sleep()

            print("Set return respond level")
            assert self.motor.set_status_return_level(ID, self.motor.RETURN_ALL)
            self.rate.sleep()

            print("Set punch limit")
            assert self.motor.set_punch_limit(ID, 50)
            self.rate.sleep()

            print("Set torque values")
            assert self.motor.set_torque_limit(ID, 0.80)
            self.rate.sleep()
            assert self.motor.set_max_torque(ID, 0.80)
            self.rate.sleep()

            print("Set temperature limit")
            assert self.motor.set_temp_limit(ID, 75)
            self.rate.sleep()

        print("Set angle limit for all motors")
        assert self.motor.set_angle_limit(0, -91,91)
        self.rate.sleep()
        assert self.motor.set_angle_limit(1, -1, 63)
        self.rate.sleep()
        assert self.motor.set_angle_limit(2, -1, 63)
        self.rate.sleep()

        print("Motors sucessfully initialized.")

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
            self.motor.set_goal_pos_speed(ID, angle_dict[ID], speed[ID])
        self.rate.sleep()

        cond = True
        while cond:
            self.rate.sleep()
            cond = False
            for ID in angle_dict:
                cond = cond or self.motor.read_moving_status(ID)

        self.done_move.publish(self.node_name)


    def callback_pos(self, data):
        angle_dict = ast.literal_eval(data.data)

        try:
            assert type(angle_dict) == dict
            for ID in angle_dict:
                assert type(ID) == int
                assert isinstance(angle_dict[ID], numbers.Real)
                assert abs(angle_dict[ID]) < 150

        except AssertionError:
            print("Invalid value recieved.")
            # self.error.publish(blablabla)
            return

        try:
            self.set_angle(angle_dict)

        except ValueError as e:
            print(e)
            # self.error.publish(blablabla)
            return


    def listener(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        g = Gripper()
        g.listener()

    except (AssertionError, rospy.ROSInterruptException) as e:
        print(e)

