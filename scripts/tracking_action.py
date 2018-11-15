#!/usr/bin/env python
import rospy
import math
import actionlib

from std_msgs.msg import String
from obstacle_detector.msg import Obstacles
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from mission_planner.msg import MissionPlannerAction, MissionPlannerGoal, MissionPlannerResult, MissionPlannerFeedback

from keyboard.msg import Key

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
		self.temp_x = 777
		self.wall_dist = rospy.get_param("/car_tracking/wall_dist") # distance of platform & wall desired
		self.car_dist = rospy.get_param("/car_tracking/car_dist")   # distance of platform & car desired

		self.wall_detect_start_count = 0
		self.wall_detect_start_thres = 5

		self.before_speed = 0
		
		self.start_flag = False
		self.finish_flag = False

		self.wall_flag = False

		self.wall_start_check = False
		self.detect_wall_distance = 0

		self.mission_finish_check = False
		self.finish_wall_distance = 0

		self.mission_finish_count = 0
		self.mission_finish_thres = 5

		
	def target_lane_feedback_cb(self, feedback):
		pass
		#self.steer = feedback.tracking_feedback
			
	def keyboard_cb(self, data):
		key_value = data.code
		if key_value == 273 or key_value == 274:
			self.steer = 0
		elif key_value == 275:
			if self.steer > 23:
				self.steer = 27
			else:
				self.steer = self.steer + 4
		elif key_value == 276:
			if self.steer < -23:
				self.steer = -27
			else:
				self.steer = self.steer - 4
		



	# LiDAR Algorithm Start
	def execute_cb(self, goal):
		# find server!
		self.client.wait_for_server()
		# send goal to taet lane node
		self.client.send_goal(self.goal, feedback_cb = self.target_lane_feedback_cb)
		# run algotihm
		self.sub = rospy.Subscriber('raw_obstacles', Obstacles, self.obstacles_cb)
		self.key_sub = rospy.Subscriber('/keyboard/keyup', Key, self.keyboard_cb)

		result = MissionPlannerResult()

		self.start_flag = True	

		r = rospy.Rate(100)
		while not rospy.is_shutdown():
			if(self.finish_flag == True):
				self.start_flag = False
				acker_data = AckermannDriveStamped() 
				acker_data.drive.speed = 0
				acker_data.drive.brake = 0
				acker_data.drive.steering_angle = 0
				self.pub.publish(acker_data)

				self.server.set_succeeded(result)	
				break
			r.sleep()


	def obstacles_cb(self, data):
		if(self.start_flag == True):
			nearest_initialized = False
			nearest_x = 0
			for segment in data.segments:
				if nearest_initialized == False:
					nearest_x = segment.first_point.x
					nearest_initialized = True

				if nearest_x >= segment.first_point.x:
					nearest_x = segment.first_point.x
				if nearest_x >= segment.last_point.x:
					nearest_x = segment.last_point.x

                        # instant difference of x
                        # TODO: change 1.0 value to launch parameter!!!!
                        if(nearest_x - self.temp_x > 4.0):
                                self.wall_start_check = True
                                self.detect_wall_distance = nearest_x

                        rospy.loginfo("nearest_x: %f"%nearest_x)
                        rospy.loginfo("temp_x: %f"%self.temp_x)
				
			if self.wall_start_check == True:
				if abs(self.detect_wall_distance - nearest_x) < 1.0:
					rospy.loginfo('wall_Detect_start_count: %d'% self.wall_detect_start_count)
					self.wall_detect_start_count = self.wall_detect_start_count + 1
                                        self.detect_wall_distance = nearest_x
				else:
					self.wall_detect_start_count = 0
					self.wall_start_check = False

				if self.wall_detect_start_count >= self.wall_detect_start_thres:
					rospy.loginfo('wall detected, from now approach to wall')
					self.wall_flag = True

			acker_data = AckermannDriveStamped() 

			if nearest_x == 0:
				rospy.loginfo("detect nothing!, speed is 3. exit obstacles_cb")
				acker_data.drive.speed = 3
				acker_data.drive.brake = 0
				return

			self.temp_x = nearest_x
			# wall is not detected	
			if(self.wall_flag == False):
				# searching for nearest x point for detecting wall		
 				#speed, steering
	
				# detect the object (including car) & control speed
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
				rospy.loginfo("wall is detected & approaching to the wall")
				self.speed_t = self.speed + (nearest_x-self.wall_dist)/self.HZ  #target speed by 'pi' controller
				
				# optimizing 1.5 value, change to launch parameter!!!
				if (self.wall_dist + 1.0> nearest_x ):
					rospy.loginfo("deceleration")
					self.speed = self.speed_t - self.speed_error*100
					if self.speed < 1:
						self.speed = 3
					self.before_speed = self.speed # define speed before a 1 func
				elif(self.wall_dist + 1.0 < nearest_x):
					rospy.loginfo("acceleration")
					self.speed = self.speed_t + self.speed_error
					self.before_speed = self.speed
					self.brake = 0
					

				if(nearest_x <= self.wall_dist):
					self.mission_finish_check = True
					self.finish_wall_distance = nearest_x
				
				if self.mission_finish_check == True:
					if abs(self.finish_wall_distance - nearest_x) < 1.0:
						rospy.loginfo('mission_finish_count: %d'% self.mission_finish_count)
						self.mission_finish_count = self.mission_finish_count + 1
                                                self.finish_wall_distance = nearest_x
					else:
						self.mission_finish_count = 0
						self.mission_finish_check = False

					if self.mission_finish_count >= self.mission_finish_thres:
						rospy.loginfo("finish!")
						self.speed = 0
						self.brake = 0
						self.client.cancel_goal()
						self.finish_flag = True

				if(self.speed > self.max_speed):
				    rospy.loginfo("speed reached max_...")
				    self.speed = self.max_speed
						
			#safety action
			if(nearest_x < 1.0):
				acker_data.drive.speed=0
			
			acker_data.drive.speed = int(self.speed)
			acker_data.drive.brake = int(self.brake)		
			acker_data.drive.steering_angle = self.steer #feedback steering data
			rospy.loginfo("speed : " + str(acker_data.drive.speed))
			rospy.loginfo("brake : " + str(acker_data.drive.brake))
			rospy.loginfo("steering : " + str(acker_data.drive.steering_angle))	

			if self.finish_flag == False:
				self.pub.publish(acker_data)
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
				
	

