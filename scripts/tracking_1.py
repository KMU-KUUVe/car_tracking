#!/usr/bin/env python
import rospy
import math

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from mission_planner.msg import MissionPlannerAction, MissionPlannerGoal, MissionPlannerResult, MissionPlannerFeedback

from car_tracking.msg import car_trackingGoal, car_trackingAction, car_trackingFeedback

class tracking:
	def __init__(self):		
		rospy.init_node('car_tracking', anonymous=True)

		self.pub = rospy.Publisher('ackermann', AckermannDriveStamped, queue_size=10)

		# Client
		self.client = actionlib.SimpleActionClient('target_lane_detect_goal', car_trackingAction)
		self.goal = car_trackingGoal()

		# Server
		self.server = actionlib.SimpleActionServer('car_tracking', MissionPlannerAction, execute_cb=execute_cb, auto_start=False)
		self.result = MissionPlannerResult()


		self.S_goal = rospy.get_param("/car_tracking/S_goal") # desired Relative distance
		self.HZ = rospy.get_param("/car_tracking/HZ")
		self.speed_error = rospy.get_param("/car_tracking/speed_error")
                self.brake_unit = rospy.get_param("/car_tracking/brake_unit")
                self.max_speed = rospy.get_param("/car_tracking/max_speed")
                self.speed = 0
		
	def target_lane_feedback_cb(self, feedback):
		acker_data = ackermannDriveStamped()
		acker_data.drive.steering_angle = self.feedback.tracking_feedback

	def target_lane_done_cb(self, result):
		#self.is_detect_crosswalk = True
		acker_data = AckermannDriveStamped()
		acker_data.drive.speed = 0
		acker_data.drive.steering_angle = 0
		self.pub.publish(acker_data)
################
		self.server.set_succeeded(self.result)
################

	# LiDAR Algorithm Start
	def execute_cb(self, goal):
		# find server!
		self.client.wait_for_server()
		# send goal to target lane node
		self.client.send_goal(self.goal, done_cb=target_lane_done_cb)
		# run algotihm
		self.sub = rospy.Subscriber('raw_obstacles', Obstacles, self.obstacles_cb)

	def obstacles_cb(self, data):
		self.client.wait_for_server()
		self.client.send_goal(goal)

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
		self.speed_t = self.speed + (self.S_now-self.S_goal)/self.HZ  #target speed		
		if (self.S_goal > self.S_now + 0.5):
			self.speed = self.speed_t - self.speed_error*100
			self.brake = self.brake_unit
		elif(self.S_goal + 0.5 < self.S_now ):
			self.speed = self.speed_t + self.speed_error
			self.brake = 0
		elif(abs(self.S_now - self.S_goal) <= 0.5):
			self.speed = self.speed_t
			self.brake = 0

                if(self.speed > self.max_speed):
                    self.speed = self.max_speed
                elif(self.speed < 0):
                    print("speed is less than 0!!")
                    self.speed = 0

                if(self.S_now < 1.5):
                    self.speed = 0

		acker_data.drive.speed = int(self.speed)
		acker_data.drive.brake = int(self.brake)		
		acker_data.drive.steering_angle = self.lane_detecting.steering #feedback steering data
		print("speed : " + str(acker_data.drive.speed))
		print("brake : " + str(acker_data.drive.brake))
		print("steering : " + str(acker_data.drive.steering_angle))	
		self.pub.publish(acker_data)
if __name__ == '__main__':
	try: 
				
	
		trakcing_mission = tracking()
		rospy.spin()	
		
	except rospy.ROSInterruptException:
		print(error)
		pass			
				
	

