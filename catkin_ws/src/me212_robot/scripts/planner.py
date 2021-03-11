#!/usr/bin/python

# 2.12 Lab 4 me212_robot: ROS driver controlling the dynamics of the robot
# Jerry Ng March 2021

import rospy
import tf
import numpy as np
import threading
import serial
import pdb

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose
from me212_robot.msg import RobotPose, EncVals, WheelVel


class Planner(object):
	# Planner
	def __init__(self):

		self.PLAN = True


		self.RobotPose = RobotPose()
		self.velocities = WheelVel()
		self.rev2enc = 1000
		self.gearing = 53
		self.b = 0.225
		self.r = 0.037
		self.a = 0.3
		self.PERIOD = 0.01
		self.enc2rev = 1.0 / self.rev2enc
		self.enc2rad = self.enc2rev * 2 * np.pi
		self.enc2wheel = self.enc2rad * self.r 
		self.encoder1CountPrev = 0
		self.encoder2CountPrev = 0
		rospy.init_node('robot_controller', anonymous=True)
		rospy.Subscriber("/robot_encoder", EncVals, self.callback)
		self.vel_pub = rospy.Publisher('/wheel_velocities', WheelVel, queue_size=10)
		self.pose_pub = rospy.Publisher('/robot_pose', RobotPose, queue_size=10)
		rospy.spin()


	def callback(self, msg):
		# Takes in encoder values from simulator and goes through the different functions to update the pose
		[dPhiL, dPhiR] = self.calc_dPhis(msg.leftEnc, msg.rightEnc)
		self.updatePose(dPhiL, dPhiR, self.RobotPose)
		self.pose_pub.publish(self.RobotPose)
		desiredV = [0,0]
		if self.PLAN:
			# Task 4:
			# Below is code that has the robot drive in a straight line.
			# You can call the current distance travelled by the robot using
			# self.RobotPose.pathDistance. For this task, modify the code
			# below so that the robot follows the U-shaped trajectory
			# depicted in the handout.

			#Drive in a straight direction
			 desiredV = self.PathPlanner(robotVel = 0.5, K = 0)

			#Commented out the answer for a u shaped trajectory
		    # if self.RobotPose.pathDistance < ?:
		    #     robotVel = ?
		    #     K = ?
		    #     desiredV =self.PathPlanner(robotVel, K)
		   	# # Hemicircle
		    # elif self.RobotPose.pathDistance < ?:
		    #     robotVel = ?
		    #     K = ?
		    #     desiredV =self.PathPlanner(robotVel, K)
			#
		    # # Straight line back
		    # elif self.RobotPose.pathDistance < ?:
		    #     robotVel = ?
		    #     K = ?
		    #     desiredV =self.PathPlanner(robotVel, K)
			#
		    # # Stop at the end
		    # elif self.RobotPose.pathDistance > ?:
			# 	robotVel = ?
			# 	K = ?
			# 	desiredV =self.PathPlanner(robotVel, K)
		self.updateVels(desiredV)
	

	def calc_dPhis(self, leftEnc, rightEnc):

		# Task 1:
		# Write code that determines the change in angle phi for each wheel.
		# To do this, you need to find the change in value for each encoder,
		# and then convert this to the appropriate angle. The angle changes
		# will be returned as dPhiL and dPhiR. Make sure that your final
		# answers are in radians.
		# You'll find it useful to use the following variables/attributes:
		# self.encoder1CountPrev, self.encoder2CountPrev, self.enc2rad
		
		self.encoder1CountPrev = rightEnc
		self.encoder2CountPrev = leftEnc
		return dPhiR, dPhiL

	def updatePose(self, dPhiL, dPhiR, Pose):

		# Task 2:
		# Fill in the equations needed to find the change in theta (dTh),
		# X (dX), and Y (dY). You may find the variables dPhiR (angle change
		# for the right wheel), dPhiL (angle change for the left wheel),
		# and Pose.Th (current value of theta) useful. Note that to take the
		# power of something, you must use a pair of asterisks, **. e.g. X
		# squared is X**2.
		#Takes in encoder values from the simulator to update the pose.
		
		#MODIFY CODE BELOW TO SET THE CORRECT VALUES
		#   Relevant constants: r, b
		#   Relevant function: np.cos(), np.sin(), np.sqrt()
		#   Use the equations referenced in the handout to set these values.
		r = self.r
		b = self.b

		#dTh = ?
		Pose.Th = Pose.Th + dTh
		
		#dX = ?
		#dY = ?
		
		Pose.X = Pose.X + dX
		Pose.Y = Pose.Y + dY

		Pose.pathDistance = Pose.pathDistance + np.sqrt(dX*dX + dY*dY)

	def PathPlanner(self, robotVel, K):
		# Task 3:
		# Write the equations for the required velocity for each wheel,
		# given a desired robot velocity (robotVel) and path curvature (K).
		# robotVel, K, and self.b may be useful variables.
		#Takes in robot velocity and curvature, outputs wheel velocities for control

		#desiredWV_L = ?
		#desiredWV_R = ?
		return desiredWV_L, desiredWV_R

	def updateVels(self, velList):
		# publishes wheel velocities to the simulator
		self.velocities.leftVel = velList[0]
		self.velocities.rightVel = velList[1]
		self.vel_pub.publish(self.velocities)

		

if __name__ == '__main__':
	planner = Planner()