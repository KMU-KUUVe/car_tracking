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
		rospy.init_node('car_tracking', anonymous=True)	
		self.S_goal = rospy.get_param("/car_tracking/S_goal") # desired Relative distance
		self.HZ = rospy.get_param("/car_tracking/HZ")
		self.speed_error = rospy.get_param("/car_tracking/speed_error")
		


	def obstacles_cb(self, data):
		self.target_segment_center = Point(100,100,0)
		self.S_now = math.sqrt(self.target_segment_center.x**2 + self.target_segment_center.y**2) # Relative distance
		segment_center_point = data.segments
		#search nearest segments		
		for segment in segment_center_point:
			distance = math.sqrt(((segment.last_point.x+segment.first_point.x)/2)**2 + ((segment.last_point.y+segment.first_point.y)/2)**2)
			if (self.S_now > distance):
				self.target_segment_center = Point(((segment.last_point.x+segment.first_point.x)/2),((segment.last_point.y+segment.first_point.y)/2),0)
		
		self.S_now = math.sqrt(self.target_segment_center.x**2 + self.target_segment_center.y**2)
		print("Target Obstacle : " + str(self.target_segment_center))
		print("Target distance : " + str(self.S_now))
		acker_data = AckermannDriveStamped()
		#speed, steering
		self.speed_t = speed + S_now/HZ  #target speed		
		if (self.S_goal > self.S_now + 0.7):
			self.speed = self.speed_t - self.speed_error
			self.brake = 50
		elif(self.S_goal + 0.7 < self.S_now ):
			self.speed = self.speed_t + self.speed_error
			self.brake = 0
		elif(abs(self.S_now - self.S_goal) <= 0.7):
			self.speed = self.speed_t
			self.brake = 0

		acker_data.drive.speed = int(self.speed)
		acker_data.drive.speed = int(self.brake)		
		acker_data.drive.steering_angle = 0
		self.count = self.count + 1
		print("speed : " + str(acker_data.drive.speed))
		print("speed : " + str(acker_data.drive.brake))
		print("steering : " + str(acker_data.drive.steering_angle))	
		self.pub.publish(acker_data)
if __name__ == '__main__':
	try: 
				
	
		trakcing_mission = tracking()
		rospy.spin()	
		
	except rospy.ROSInterruptException:
		print(error)
		pass			
				
	

