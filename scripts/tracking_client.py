#!/usr/bin/env python
import rospy
import actionlib

from car_tracking.msg import car_trackingAction, car_trackingGoal, car_trackingFeedback



class Tracking_Client:
	
	def __init__(self):
		rospy.init_node('tracking_client')		
		#lane_start_goal is goal to server, car_trackingAction is type
		self.client = actionlib.SimpleActionClient('lane_start_goal', car_trackingAction)
		self.steering = 0
		
		self.goal = car_trackingGoal()
		
	def execute(self):	
		self.client.wait_for_server()
		
		self.client.send_goal(goal, feedback_cb = self.feedback_steering)
		self.client.wait_for_result()

	
	def feedback_steering(self, feedback):
		self.steering = feedback
		



if __name__ == '__main__':
	try:
		
		Tracking_client = Tracking_Client()
		Tracking_client.execute()
		rospy.spin()
		

	except rospy.ROSInterruptException:
		print(error)
		pass	
		

