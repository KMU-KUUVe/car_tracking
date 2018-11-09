#!/usr/bin/env python
import rospy
import actionlib

from mission_planner.msg import MissionPlannerAction, MissionPlannerGoal, MissionPlannerResult, MissionPlannerFeedback
from tracking_1 import tracking


def execute_cb(goal):
	rospy.loginfo("Goal Received")
	result = MissionPlannerResult()
	tracking_mission = tracking()
	action_server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('car_tracking', anonymous=True)
    try:
        action_name = 'car_tracking'
        action_server = actionlib.SimpleActionServer(action_name, 		MissionPlannerAction, execute_cb=execute_cb, auto_start=False)
        action_server.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(error)
	pass
