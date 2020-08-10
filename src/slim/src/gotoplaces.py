#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from math import sqrt
from std_msgs.msg import String
import tf
from move_base_msgs.msg import MoveBaseActionGoal

class GoToPlace:
	def __init__(self):
		rospy.init_node('gotoplace')
		self.goalpub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)

		 #rooms
		self.bedroom_coords = Pose()
		self.bedroom_coords.position.x = -5.90896390397
		self.bedroom_coords.position.y= -0.182613641614
		self.bedroom_coords.position.z= 0.0
		self.bedroom_coords.orientation.x= 0.0
		self.bedroom_coords.orientation.y= 0.0
		self.bedroom_coords.orientation.z= 0.766980055119
		self.bedroom_coords.orientation.w=  0.641670939852

		self.gym_coords = Pose()
		self.gym_coords.position.x= 2.08761533804
		self.gym_coords.position.y= 4.18099830614
		self.gym_coords.position.z= 0.0
		self.gym_coords.orientation.x= 0.0
		self.gym_coords.orientation.y= 0.0
		self.gym_coords.orientation.z= 0.104205358661
		self.gym_coords.orientation.w= 0.994555801967

		self.bathroom_coords = Pose()
		self.bathroom_coords.position.x= -7.37957953818
		self.bathroom_coords.position.y= -3.42667018381
		self.bathroom_coords.position.z= 0.0
		self.bathroom_coords.orientation.x= 0.0
		self.bathroom_coords.orientation.y= 0.0
		self.bathroom_coords.orientation.z= -0.957570048422
		self.bathroom_coords.orientation.w= 0.28820062867

		self.livingroom_coords = Pose()
		self.livingroom_coords.position.x= 1.72770424947
		self.livingroom_coords.position.y= -2.84897038012
		self.livingroom_coords.position.z= 0.0
		self.livingroom_coords.orientation.x= 0.0
		self.livingroom_coords.orientation.y= 0.0
		self.livingroom_coords.orientation.z= 0.273485657151
		self.livingroom_coords.orientation.w= 0.961876081069

		self.diningroom_coords = Pose()
		self.diningroom_coords.position.x= 4.67544469965
		self.diningroom_coords.position.y= 0.985203490689
		self.diningroom_coords.position.z= 0.0
		self.diningroom_coords.orientation.x= 0.0
		self.diningroom_coords.orientation.y= 0.0
		self.diningroom_coords.orientation.z= 0.0960743884122
		self.diningroom_coords.orientation.w= 0.995374156733

		self.kitchen_coords = Pose()
		self.kitchen_coords.position.x= 7.1871667022
		self.kitchen_coords.position.y= -3.15355636372
		self.kitchen_coords.position.z= 0.0
		self.kitchen_coords.orientation.x= 0.0
		self.kitchen_coords.orientation.y= 0.0
		self.kitchen_coords.orientation.z= 0.00265381718347
		self.kitchen_coords.orientation.w= 0.999996478621

		self.lab_table_coords = Pose()
		self.lab_table_coords.position.x=-6.8587046
		self.lab_table_coords.position.y=-2.72935
		self.lab_table_coords.position.z=0.0
		self.lab_table_coords.orientation.x=0.0
		self.lab_table_coords.orientation.y=0.0
		self.lab_table_coords.orientation.z=-0.9917877
		self.lab_table_coords.orientation.w=0.12789477

		rospy.Subscriber('/go_to', String, self.command_handler)
		rospy.Subscriber('/go_to_pose', Pose, self.pose_command_handler)
		rospy.spin()


	def command_handler(self, place):
		pose = None
		if place.data == 'bedroom':
			pose = self.bedroom_coords
		elif place.data == 'bathroom':
			pose = self.bathroom_coords
		elif place.data == 'livingroom':
			pose = self.livingroom_coords
		elif place.data == 'diningroom':
			pose = self.diningroom_coords
		elif place.data == 'kitchen':
			pose = self.kitchen_coords
		elif place.data == 'gym':
			pose = self.gym_coords
		elif place.data == 'labtable':
			pose = self.lab_table_coords

		if pose is not None:
			self.gotoplace(pose)

	def pose_command_handler(self, pose):
		self.gotoplace(pose)


	def gotoplace(self,pose):
		goal = MoveBaseActionGoal()
		goal.goal.target_pose.header.frame_id="map"
		goal.goal.target_pose.pose = pose
		self.goalpub.publish(goal)


if __name__=='__main__':
	z = GoToPlace()