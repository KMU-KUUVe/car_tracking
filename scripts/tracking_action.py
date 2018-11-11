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
		self.wall_width = rospy.get_param("/car_tracking/wall_width")
		#self.car_width = rospy.get_param("/car_tracking/car_width")
		self.wall_dist = rospy.get_param("/car_tracking/wall_dist") # distance of platform & wall desired
		self.car_dist = rospy.get_param("/car_tracking/car_dist")   # distance of platform & car desired

		self.start_flag = False
		self.finish_flag = False


	def target_lane_feedback_cb(self, feedback):
		self.steer = self.feedback.tracking_feedback


	# LiDAR Algorithm Start
	def execute_cb(self, goal):
		# find server!
		self.client.wait_for_server()
		# send goal to taet lane node
		self.client.send_goal(self.goal, feedbackcb = target_lane_feedback_cb)
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
			self.client.wait_for_server()
			self.client.send_goal(goal)

			self.target_segment_center = Point(100,100,0)
			# width of object 
			self.detected_width = math.sqrt((self.segment.last_point.y - self.segment.first_point.y)**2 + (self.segment.last_point.x - self.segment.first_point.x)**2)
			# distance of object (foucus distance)
			self.detected_dist = math.sqrt(self.target_segment_center.x**2 + self.target_segment_center.y**2)
			segment_center_point = data.segments
			#search nearest segments		
			for segment in segment_center_point:
				distance = math.sqrt(((segment.last_point.x+segment.first_point.x)/2)**2 + ((segment.last_point.y+segment.first_point.y)/2)**2)
				if (self.detected_dist> distance):
					self.target_segment_center = Point(((segment.last_point.x+segment.first_point.x)/2),((segment.last_point.y+segment.first_point.y)/2),0)
			
			print("Target Obstacle : " + str(self.target_segment_center))
			print("Target distance : " + str(self.S_now))
			acker_data = AckermannDriveStamped()
			#speed, steering
			# detect the wall &  stop in a desired distance
			if (abs(self.detected_width - self.wall_width)<0.5 && detected_dist< self.wall_dist):
				self.finish_flag = True
				
				print("wall is detected!")
				self.client.cancel_goal()
				acker_data.drive.speed = 0
				acker_data.drive.steering_angle = 0
				self.pub.publish(acker_data)
			# detect nothing & speed is constant
			elif(self.target_segment_center.x == 0):
				print("detect nothing!")
				acker_date.drive.speed = 3
			# detect the object (including car) & control speed
			else:
				print("object is detected & chases!")
				self.speed_t = self.speed + (self.detected_dist-self.car_dist)/self.HZ  #target speed by 'pi' controller
				if (self.car_dist > self.detected_dist + 0.5):
					print("deceleration")
					self.speed = self.speed_t - self.speed_error*100
					self.before_speed = self.speed # define speed before a 1 func
					self.brake = self.brake_unit
				elif(self.car_dist + 0.5 < self.detected_dist):
					print("acceleration")
					self.speed = self.speed_t + self.speed_error
					self.before_speed = self.speed
					self.brake = 0
				elif(abs(self.detected_dist - self.car_dist) <= 0.5):
					print("const speed")
					self.speed = before_speed
					self.brake = 0

				if(self.speed > self.max_speed):
				    print("speed reached max_...")
				    self.speed = self.max_speed
				elif(self.speed < 0):
				    print("speed is less than 0!!")
				    self.speed = 0


			#safety action
			if(self.detected_dist <0.5)
				acker_data.drive.speed=0
			
			acker_data.drive.speed = int(self.speed)
			acker_data.drive.brake = int(self.brake)		
			acker_data.drive.steering_angle = self.steer #feedback steering data
			print("speed : " + str(acker_data.drive.speed))
			print("brake : " + str(acker_data.drive.brake))
			print("steering : " + str(acker_data.drive.steering_angle))	

			if self.finish_flag == False:
				self.pub.publish(acker_data)


if __name__ == '__main__':
	try: 	
		trakcing_mission = tracking()
		rospy.spin()	
		
	except rospy.ROSInterruptException:
		print(error)
		pass			
				
	

