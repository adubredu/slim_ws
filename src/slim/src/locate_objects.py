#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from math import sqrt
from std_msgs.msg import String
import tf

class Perception:
    def __init__(self):
        rospy.init_node('Perception')
        self.robot_pose = Pose()
        self.pub_obs = rospy.Publisher('/seeing_what', String, queue_size=1)

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
        self.diningroom_coords.position.x= 0.871188572304
        self.diningroom_coords.position.y= -18.95440936
        self.diningroom_coords.position.z= 0.0
        self.diningroom_coords.orientation.x= 0.0
        self.diningroom_coords.orientation.y= 0.0
        self.diningroom_coords.orientation.z= 0.717687653858
        self.diningroom_coords.orientation.w= -0.696365156724

        self.kitchen_coords = Pose()
        self.kitchen_coords.position.x= 7.1871667022
        self.kitchen_coords.position.y= -3.15355636372
        self.kitchen_coords.position.z= 0.0
        self.kitchen_coords.orientation.x= 0.0
        self.kitchen_coords.orientation.y= 0.0
        self.kitchen_coords.orientation.z= 0.00265381718347
        self.kitchen_coords.orientation.w= 0.999996478621

        #seen objects
        self.seen_saucepan = Pose()
        self.seen_saucepan.position.x= 7.28019401229
        self.seen_saucepan.position.y= -3.83174282842
        self.seen_saucepan.position.z= 0.0
        self.seen_saucepan.orientation.x= 0.0
        self.seen_saucepan.orientation.y= 0.0
        self.seen_saucepan.orientation.z= -0.608809141514
        self.seen_saucepan.orientation.w=  0.793316726919

        self.seen_coke = Pose()
        self.seen_coke.position.x= 5.21946681309
        self.seen_coke.position.y= 1.08387607235
        self.seen_coke.position.z= 0.0
        self.seen_coke.orientation.x= 0.0
        self.seen_coke.orientation.y= 0.0
        self.seen_coke.orientation.z= -0.108269188337
        self.seen_coke.orientation.w= 0.994121613716

        self.seen_beer = Pose()
        self.seen_beer.position.x= 1.33984783435
        self.seen_beer.position.y= -3.33457800579
        self.seen_beer.position.z= 0.0
        self.seen_beer.orientation.x= 0.0
        self.seen_beer.orientation.y= 0.0
        self.seen_beer.orientation.z= 0.71200683357
        self.seen_beer.orientation.w= 0.702172535029

        self.seen_drill = Pose()
        self.seen_drill.position.x=-7.76401341942
        self.seen_drill.position.y= 1.13558661716
        self.seen_drill.position.z= 0.0
        self.seen_drill.orientation.x= 0.0
        self.seen_drill.orientation.y= 0.0
        self.seen_drill.orientation.z= 0.696671739171
        self.seen_drill.orientation.w= 0.717390052789



        self.seen_tolerance = 2.5
        self.orient_tolerance = 0.8
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.odometry_handler)
        self.publish_seen_objects()
        rospy.spin()


    def odometry_handler(self, odom):
        self.robot_pose = odom.pose.pose


    def what_are_you_seeing(self):
        seen_objects=[]
        if self.seeing(self.seen_beer):
            seen_objects.append('beer')

        if self.seeing(self.seen_coke):
            seen_objects.append('coke')

        if self.seeing(self.seen_saucepan):
            seen_objects.append('saucepan')

        if self.seeing(self.seen_drill):
            seen_objects.append('drill')

        return seen_objects


    def seeing(self,pose):
        rx = self.robot_pose.position.x; ry = self.robot_pose.position.y 
        gx = pose.position.x; gy = pose.position.y 

        orz = self.robot_pose.orientation.z; orw = self.robot_pose.orientation.w 
        ogz = pose.orientation.z; ogw = pose.orientation.w

        Rquaternion = (self.robot_pose.orientation.x,self.robot_pose.orientation.y,
            self.robot_pose.orientation.z,self.robot_pose.orientation.w)
        Reuler = tf.transformations.euler_from_quaternion(Rquaternion)
        ryaw = Reuler[2]

        Pquaternion = (pose.orientation.x,pose.orientation.y,pose.orientation.z,
            pose.orientation.w)
        Peuler = tf.transformations.euler_from_quaternion(Pquaternion)
        pyaw = Peuler[2]

        dist = sqrt((rx-gx)**2 + (ry-gy)**2)
        ordelta = sqrt((ryaw-pyaw)**2)

        if ((dist < self.seen_tolerance) and (ordelta < self.orient_tolerance)):
            return True
        else:
            return False


    def publish_seen_objects(self):
        while not rospy.is_shutdown():
            obs = self.what_are_you_seeing()
            if len(obs) > 0:
                for ob in obs:
                    name = String()
                    name.data = ob 
                    self.pub_obs.publish(name)
            rospy.sleep(1)
            # else:
            #     name = String()
            #     name.data = 'nothing' 
            #     self.pub_obs.publish(name)


if __name__=='__main__':
    p = Perception()





