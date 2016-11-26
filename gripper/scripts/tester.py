#!/usr/bin/python
import rospy
import ast
from std_msgs.msg import String, Float64

class Tester:
	def __init__(self,node_name):

		# ROS init
		self.node_name = node_name
		rospy.init_node(self.node_name, anonymous=True, log_level=rospy.INFO)
		self.rate = rospy.Rate(10)  # 10 Hz

		# ROS subsribers
		self.sub = rospy.Subscriber('Send_Test',Float64,self.send_pub)

		# ROS publishments
		self.pub = rospy.Publisher('Start_Protocol', String, queue_size=1)

	    # Subscriber callbacks
	    # ---------------------------------------------------------------------- #

	    # ---------------------------------------------------------------------- #
	def send_pub(self, msg):
		number = msg.data
		self.pub.publish('{"name":"test","author":"a","description":"test","refs":[{"name":"gripper","type":"gripper"}],"instructions":[{"op":"gripper","groups":[{"clap":10}]}]}')
		self.rate.sleep()

	def listener(self):
		rospy.spin()

if __name__ == "__main__":

	try:
		test = Tester('test1')
		#test.listener()
        	#protocol_path = raw_input('Enter protocol path (0 for default): ')
		#if protocol_path == '0':
        	#protocol_path = '{"name":"test","author":"a","description":"test","refs":[{"name":"gripper","type":"gripper"}],"instructions":[{"op":"gripper","groups":[{"clap"}]}]}'
		#test.pub.publish('ALLO !')
		test.rate.sleep()
		test.listener()

	except (AssertionError, rospy.ROSInterruptException) as e:
		rospy.logerr(e)
		
		
		
