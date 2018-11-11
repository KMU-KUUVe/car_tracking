#!/usr/bin/env python
import rospy
import math
import actionlib

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
		self.client = actionlib.SimpleActionClient('target_lane', car_trackingAction)
		self.goal = car_trackingGoal()

		# Server
		self.server = actionlib.SimpleActionServer('car_tracking', MissionPlannerAction, execute_cb=self.execute_cb, auto_start=False)
		self.server.start()

		self.HZ = rospy.get_param("/car_tracking/HZ")  # frequency of Ros node
		self.speed_error = rospy.get_param("/car_tracking/speed_error")
                self.brake_unit = rospy.get_param("/car_tracking/brake_unit")
                self.max_speed = rospy.get_param("/car_tracking/max_speed")
                self.speed = 0
		self.steer = 0
		self.temp_x = 0
		self.wall_dist = rospy.get_param("/car_tracking/wall_dist") # distance of platform & wall desired
		self.car_dist = rospy.get_param("/car_tracking/car_dist")   # distance of platform & car desired
		
		self.start_flag = False
		self.finish_flag = False

		self.wall_flag = False
		
	def target_lane_feedback_cb(self, feedback):
		self.steer = feedback.tracking_feedback


	# LiDAR Algorithm Start
	def execute_cb(self, goal):
		# find server!
		self.client.wait_for_server()
		# send goal to taet lane node
		self.client.send_goal(self.goal, feedback_cb = self.target_lane_feedback_cb)
		# run algotihm
		self.sub = rospy.Subscriber('raw_obstacles', Obstacles, self.obstacles_cb)

		result = MissionPlannerResult()

		self.start_flag = True	

		r = rospy.Rate(100)
		while not rospy.is_shutdown():
			if(self.finish_flag == True):
				self.start_flag = False
				self.server.set_succeeded(result)	
				break
			r.sleep()


	def obstacles_cb(self, data):
		
		
		if(self.start_flag == True):
			
			for segment in data.segments:
				nearest_x = segment.first_point.x
				if nearest_x >= segment.first_point.x:
					nearest_x = segment.first_point.x
				if nearest_x >= segment.last_point.x:
					nearest_x = segment.last_point.x
				if(nearest_x - self.temp_x > self.wall_dist):
					rospy.loginfo('wall detected')
					self.wall_flag = True

			self.temp_x = nearest_x
			acker_data = AckermannDriveStamped() 
			# wall is not detected	
			if(self.wall_flag == False):
				# searching for nearest x point for detecting wall		
 				#speed, steering
	
				# detect nothing & speed is constant
				if(nearest_x == 0):
					rospy.loginfo("detect nothing!")
					acker_date.drive.speed = 3
				# detect the object (including car) & control speed
				else:
					
					rospy.loginfo("car is detected & chases!")
					self.speed_t = self.speed + (nearest_x-self.car_dist)/self.HZ  #target speed by 'pi' controller
					if (self.car_dist > nearest_x + 0.5):
						rospy.loginfo("deceleration")
						self.speed = self.speed_t - self.speed_error*100
						self.before_speed = self.speed # define speed before a 1 func
						self.brake = self.brake_unit
					elif(self.car_dist + 0.5 < nearest_x):
						rospy.loginfo("acceleration")
						self.speed = self.speed_t + self.speed_error
						self.before_speed = self.speed
						self.brake = 0
					elif(abs(nearest_x - self.car_dist) <= 0.5):
						rospy.loginfo("const speed")
						self.speed = self.before_speed
						self.brake = 0

					if(self.speed > self.max_speed):
					    rospy.loginfo("speed reached max_...")
					    self.speed = self.max_speed
					elif(self.speed < 0):
					    rospy.loginfo("speed is less than 0!!")
					    self.speed = 0
			else:	                                                                      
				rospy.loginfo("wall is detected & chases!")
				self.speed_t = self.speed + (nearest_x-self.car_dist)/self.HZ  #target speed by 'pi' controller
				if (self.car_dist > nearest_x ):
					rospy.loginfo("deceleration")
					self.speed = self.speed_t - self.speed_error*100
					self.before_speed = self.speed # define speed before a 1 func
					self.brake = self.brake_unit
				elif(self.car_dist < nearest_x):
					rospy.loginfo("acceleration")
					self.speed = self.speed_t + self.speed_error
					self.before_speed = self.speed
					self.brake = 0
				elif(abs(nearest_x - self.car_dist) <= 0):
					rospy.loginfo("finish!")
					self.speed = 0
					self.brake = 0
					self.client.cancel_goal()
					self.finish_flag = True

				if(self.speed > self.max_speed):
				    rospy.loginfo("speed reached max_...")
				    self.speed = self.max_speed
				elif(self.speed < 0):
				    rospy.loginfo("speed is less than 0!!")
				    self.speed = 0
						
			#safety action
			if(nearest_x <0.5):
				acker_data.drive.speed=0
			
			acker_data.drive.speed = int(self.speed)
			acker_data.drive.brake = int(self.brake)		
			acker_data.drive.steering_angle = self.steer #feedback steering data
			rospy.loginfo("speed : " + str(acker_data.drive.speed))
			rospy.loginfo("brake : " + str(acker_data.drive.brake))
			rospy.loginfo("steering : " + str(acker_data.drive.steering_angle))	

			if self.finish_flag == False:
				self.pub.publish(acker_data)


if __name__ == '__main__':
	try: 	
		trakcing_mission = tracking()
		rospy.spin()	
		
	except rospy.ROSInterruptException:
		rospy.loginfo(error)
		pass			
				
	

