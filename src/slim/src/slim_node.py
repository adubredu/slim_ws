#! /usr/bin/env python

import rospy
import numpy as np 
from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult
from math import pi, sin, cos
from sklearn.mixture import GaussianMixture as GMM 
import time
import tf

class Slim:
	def __init__(self):
		rospy.init_node('slim_node')
		self.particle_pub = rospy.Publisher('/particles', PoseArray, queue_size=1)
		self.goto_pub = rospy.Publisher('/go_to', String, queue_size=1)
		self.gotoposepub = rospy.Publisher('/go_to_pose', Pose, queue_size=1)
		self.top_left_x = -8.6
		self.top_left_y = 5.5
		self.bottom_right_x = 8.7
		self.bottom_right_y = -4.7
		self.num_particles = 100
		self.reached_destination = True
		self.robot_pose = Pose()
		self.seeing = 'nothing'
		self.view_radius = 1.0

		self.commonsense = {
		'coke': {'kitchen':0.3, 'diningroom':0.3, 'gym':0.05, 'bathroom':0.0, 
		'bedroom':0.15, 'livingroom':0.20},

		'drill': {'kitchen':0.0, 'diningroom':0.0, 'gym':0.0, 'bathroom':0.3, 
		'bedroom':0.6, 'livingroom':0.1},

		'saucepan': {'kitchen':0.6, 'diningroom':0.3, 'gym':0.0, 'bathroom':0.0, 
		'bedroom':0.0, 'livingroom':0.1},

		'beer': {'kitchen':0.3, 'diningroom':0.3, 'gym':0.05, 'bathroom':0.0, 
		'bedroom':0.05, 'livingroom':0.30},
		}

		self.places = {
		'kitchen':{'x':7.18, 'y':-3.15}, 'diningroom':{'x':4.67, 'y':0.98},
		'gym':{'x':2.08, 'y':4.18},'bedroom':{'x':-5.90, 'y':-0.18}, 
		'livingroom':{'x':1.72, 'y':-2.84}, 'bathroom':{'x':-7.37 , 'y':-3.42 }
		}

		rospy.Subscriber('/search_for', String, self.target_handler)
		rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.movebase_result_handler)
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.odometry_handler)
		rospy.Subscriber('/seeing_what', String, self.perception_handler)



		# while not rospy.is_shutdown():
		for i in range(5):
			self.uniform_sample()
			time.sleep(1)

		rospy.spin()


	def movebase_result_handler(self, res):
		if res.status.status == 3:
			self.reached_destination = True

		elif res.status.status == 4:
			self.reached_destination = False


	def odometry_handler(self, odom):
		self.robot_pose = odom.pose.pose


	def perception_handler(self, data):
		self.seeing = data.data


	def found(self, target):
		if target == self.seeing:
			print('found the target: {}'.format(target))
			return True
		else:
			return False


	def uniform_sample(self):
		xs = np.random.uniform(self.top_left_x, self.bottom_right_x, self.num_particles)
		ys = np.random.uniform(self.bottom_right_y, self.top_left_y,  self.num_particles)

		particles = PoseArray()
		particles.header.frame_id = "map"
		particles.header.stamp  = rospy.Time.now()

		for i in range(self.num_particles):
			pose = Pose()
			pose.position.x = xs[i]
			pose.position.y = ys[i]
			particles.poses.append(pose)

		self.particle_pub.publish(particles)


	def target_handler(self, data):
		target = data.data
		probabilities = self.commonsense[target]
		choice = self.importance_sample(probabilities)
		self.resample(choice)
		self.send_robot_to(choice)
		# self.look_around_for(target)
		self.search_around(target)


	def importance_sample(self, prob_dict):
		places = list(prob_dict.keys())
		probs = list(prob_dict.values())
		choice = np.random.choice(places, size=1, p=probs)

		return choice[0]


	def resample(self, choice):
		coords = self.places[choice]
		self.xs = np.random.normal(coords['x'], 1.0, size=self.num_particles)
		self.ys = np.random.normal(coords['y'], 1.0, size = self.num_particles)

		particles = PoseArray()
		particles.header.frame_id = "map"
		particles.header.stamp  = rospy.Time.now()

		for i in range(self.num_particles):
			pose = Pose()
			pose.position.x = self.xs[i]
			pose.position.y = self.ys[i]
			particles.poses.append(pose)

		self.particle_pub.publish(particles)


	def send_robot_to(self, place):
		pl = String()
		pl.data = place
		self.reached_destination = False
		self.goto_pub.publish(pl)
		time.sleep(1)


	def search_around(self, target):
		while not self.reached_destination:
			pass
		self.reached_destination = False

		while not self.found(target):
			print('turning around')
			quad = self.robot_pose.orientation
			quaternion = (quad.x,quad.y,quad.z,quad.w)
			euler = tf.transformations.euler_from_quaternion(quaternion)
			roll = euler[0]; pitch = euler[1]; yaw = euler[2]
			yaw+=1.57
			quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
			pose = self.robot_pose
			pose.orientation.x = quaternion[0]
			pose.orientation.y = quaternion[1]
			pose.orientation.z = quaternion[2]
			pose.orientation.w = quaternion[3]
			self.gotoposepub.publish(pose)
			time.sleep(15)

			while not self.reached_destination:
				pass
			self.reached_destination = False

			if not self.found(target):
				print('going to new place')
				probabilities = self.commonsense[target]
				choice = self.importance_sample(probabilities)
				self.resample(choice)
				self.send_robot_to(choice)

				while not self.reached_destination:
					pass




			

	def look_around_for(self, target):
		while not self.reached_destination:
			pass
		print('robot has reached destination. Now going to look around')
		self.reached_destination=False
		r = self.view_radius
		views = []

		#fit gmm on particles
		gx = GMM(n_components=3)
		gx.fit(self.xs.reshape(-1,1))
		gy = GMM(n_components=3)
		gy.fit(self.ys.reshape(-1,1))

		#choose particle with greatest weight
		mean_x = gx.means_[0]
		mean_y = gy.means_[0]

		#generate viewing angles and choose optimal 
		for ang in range(0,360,36):
			th = (pi*ang)/180
			x = mean_x + r*cos(th)
			y = mean_y+ r*sin(th)
			views.append([x,y])

		ranked_views = []
		for idx, view in enumerate(views):
			v = (idx*36*pi)/180
			score = 1 - v *np.linalg.norm(np.subtract([mean_x,mean_y] , view) / np.linalg.norm(np.subtract([mean_x,mean_y],view)))
			ranked_views.append((idx, score))

		ranked_views.sort(key=lambda x: x[1])
		selected_view = views[ranked_views[0][0]]
		pose = Pose()
		pose.position.x = selected_view[0]
		pose.position.y = selected_view[1]
		pose.orientation.z = 1
		self.gotoposepub.publish(pose)
		print('going to view position {}, {}'.format(selected_view[0], selected_view[1]))
		#send robot to selected_view[0,1]
















if __name__ == '__main__':
	s = Slim()