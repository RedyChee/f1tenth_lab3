#!/usr/bin/env python3
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

#PID CONTROL PARAMS
kp = 10
kd = 70
ki = 0.00001
prev_error = 0 
integral = 0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan

class WallFollow:
	""" Implement Wall Following on the car
	"""
	def __init__(self):
		self.rate = rospy.Rate(10)
		self.lidar_sub = rospy.Subscriber('/scan' , LaserScan, self.lidar_callback, queue_size = 1)
		self.drive_pub = rospy.Publisher('/nav', AckermannDriveStamped, queue_size = 1)
		
	def getRange(self, data, angle, distance):
		self.Dt_max = False
		angle_btwnScan = 70
		future_dist = 0.55
		theta = angle[int((180-angle_btwnScan+45)/ANGLE_RANGE * len(angle))]
		a = distance[int((180-angle_btwnScan+45)/ANGLE_RANGE * len(angle))]
		b = distance[int((180+45)/ANGLE_RANGE * len(angle))]
		alpha = math.atan((a*np.cos(np.pi-theta) - b)/a*np.sin(np.pi-theta))
		Dt = b*np.cos(alpha)
		Dt1 = Dt + future_dist*np.sin(alpha)
		# check condition for bottom part of map
		a2 = a = distance[int((180-15+45)/ANGLE_RANGE * len(angle))]
		theta2 = angle[int((180-15+45)/ANGLE_RANGE * len(angle))]
		alpha2 = math.atan((a2*np.cos(np.pi-theta2) - b)/a2*np.sin(np.pi-theta2))
		bot_error = a2*np.cos(np.pi-theta2+alpha2)
		Dt2 = b*np.cos(alpha2)
		self.bot_state = math.isclose(bot_error, Dt2, rel_tol = 0.1)
		if Dt > 1.1:
			self.Dt_max = True
		return Dt1

	def filter_scan(self, scan_msg):
		angle_range = enumerate(scan_msg.ranges)
		filter_data = [[count*scan_msg.angle_increment, val] for count, val in angle_range if not np.isinf(val) and not np.isnan(val)]
		filter_data = np.array(filter_data)
		angles = filter_data[:,0]
		distance = filter_data[:,1]
		filter_angle = np.array([])
		idx = 0
		start_idx = 0
		end_idx = 0
		for angle in angles:
			if (not 0 <= angle < np.pi/4) and (not 7*np.pi/4 < angle <= 2*np.pi):
				if start_idx == 0:
					start_idx = idx
				angle -= np.pi/2
				filter_angle = np.append(filter_angle, angle)
			if end_idx == 0 and angle > 7*np.pi/4:
				end_idx = idx - 1
			idx += 1
		distance = distance[start_idx: end_idx+1]
		return filter_angle, distance
			
	def pid_control(self, error):
		global integral
		global prev_error
		global kp
		global ki
		global kd
		integral += error
		angle = kp*error + kd*(error - prev_error) + ki*integral
		prev_error = error
		if self.bot_state == True and self.Dt_max == True:
			angle = -0.01*np.pi/180
		if -np.pi/18 < angle < np.pi/18:
			velocity = 1.5
		elif -np.pi/9 < angle <= -np.pi/18 or np.pi/18 <= angle < np.pi/9:
			velocity = 1
		else:
			velocity = 0.5
		drive_msg = AckermannDriveStamped()
		drive_msg.header.stamp = rospy.Time.now()
		drive_msg.header.frame_id = "laser"
		drive_msg.drive.steering_angle = angle
		drive_msg.drive.speed = velocity
		self.drive_pub.publish(drive_msg)

	def followLeft(self, leftDist):
		distToWall = 0.55
		error = -(distToWall - leftDist)
		return error
    
	def lidar_callback(self, data):
		try:
			angle, distance = self.filter_scan(data)
			Dt1 = self.getRange(data, angle, distance)
			error = self.followLeft(Dt1)
			self.pid_control(error)
		except rospy.ROSException as e:
		#			rospy.logerr(e)
			pass
				
def main(args):
	rospy.init_node("WallFollow_node", anonymous=True)
	wf = WallFollow()
	rospy.spin()

if __name__=='__main__':
	main(sys.argv)
