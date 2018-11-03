#!/usr/bin/env python
import rospy
import math

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped

class tracking:
	def __init__(self):
		self.pub = rospy.Publisher('ackermann', AckermannDriveStamped, queue_size=10)		
		self.sub = rospy.Subscriber('raw_obstacles', Obstacles, self.obstacles_cb)
		rospy.init_node('narrow_path', anonymous=True)	
		self.constant_distance = rospy.get_param("/pg_tracking/constant_distance")
		self.KI = rospy.get_param("/pg_tracking/KI")
		self.KP = rospy.get_param("/pg_tracking/KP")
		self.count = 0
		
	def PID(self):
		self.speed = self.last_speed + (self.target_distance - self.constant_distance)*(self.KP + (self.KI))/700 
	def get_last_speed(self, speed ):
		self.last_speed = speed	

	def obstacles_cb(self, data):
		self.target_segment_center = Point(100,100,0)
		self.target_distance = math.sqrt(self.target_segment_center.x**2 + self.target_segment_center.y**2)
		segment_center_point = data.segments
		#search nearest segments		
		for segment in segment_center_point:
			distance = math.sqrt(((segment.last_point.x+segment.first_point.x)/2)**2 + ((segment.last_point.y+segment.first_point.y)/2)**2)
			if (self.target_distance > distance):
				self.target_segment_center = Point(((segment.last_point.x+segment.first_point.x)/2),((segment.last_point.y+segment.first_point.y)/2),0)
		
		self.target_distance = math.sqrt(self.target_segment_center.x**2 + self.target_segment_center.y**2)
		print("Target Obstacle : " + str(self.target_segment_center))
		print("Target distance : " + str(self.target_distance))
		acker_data = AckermannDriveStamped()
		#speed, steering
		if (self.count < 1):
			self.get_last_speed(0)
		else:
			self.get_last_speed(self.speed)
		self.PID()
		if (self.speed > 5):
			self.speed = 5
		elif (self.speed < 0):
			self.speed = 1
		if(self.target_distance <= 1):
			self.speed = 1
		acker_data.drive.speed = int(self.speed)		
		acker_data.drive.steering_angle = 0
		self.count = self.count + 1
		print("speed : " + str(acker_data.drive.speed))
		print("steering : " + str(acker_data.drive.steering_angle))	
		self.pub.publish(acker_data)
if __name__ == '__main__':
	try: 
				
	
		trakcing_mission = tracking()
		rospy.spin()	
		
	except rospy.ROSInterruptException:
		print(error)
		pass			
				
	

