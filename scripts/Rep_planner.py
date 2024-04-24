#!/usr/bin/env python3
import math
import rospy
import tf2_ros

from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist 
import tf2_geometry_msgs
from robot_vision_lectures.msg import XYZarray, SphereParams
from std_msgs.msg import UInt8, Bool

pt_in_tool = tf2_geometry_msgs.PointStamped()
params_received = False
move_robot = False

plan_printed = False

#if __name__ == '__main__':
	# initialize the node
#	rospy.init_node('Rep_planner', anonymous = True)
	
	# add a publisher for sending joint position commands
#	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
#	tfBuffer = tf2_ros.Buffer()
#	listener = tf2_ros.TransformListener(tfBuffer)
	# set a 10Hz frequency for this loop
#	loop_rate = rospy.Rate(10)


def sphere_params_callback(data):
	global pt_in_tool
	global params_received
	# Callback function to update the point coordinates
	pt_in_tool.point.x = data.xc
	pt_in_tool.point.y = data.yc
	pt_in_tool.point.z = data.zc
	params_received = True
	#print(pt_in_tool.point.x, pt_in_tool.point.y, pt_in_tool.point.z)	
	#return pt_in_tool.point.x, pt_in_tool.point.y, pt_in_tool.point.z

def moveCallback(data):
	global move_robot
	move_robot = data.data


if __name__ == '__main__':
	# initialize the node
	rospy.init_node('Rep_planner', anonymous=True)

	# add a publisher for sending joint position commands
	plan_pub = rospy.Publisher('/plan', Plan, queue_size=10)
	rospy.Subscriber("/sphere_params", SphereParams, sphere_params_callback)
	rospy.Subscriber("/move_robot", Bool, moveCallback)
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
		if params_received: 
			try:
				trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
			except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				print('Frames not available!!!')
				loop_rate.sleep()

			#pt_in_tool = tf2_geometry_msgs.PointStamped()
			pt_in_tool.header.frame_id = 'camera_color_optical_frame'
			pt_in_tool.header.stamp = rospy.get_rostime()
			#pt_in_tool.point.x = -0.013910026289522648
			#pt_in_tool.point.y = -0.017049305140972137
			#pt_in_tool.point.z = 0.477964848279953

			pt_in_base = tfBuffer.transform(pt_in_tool,'base', rospy.Duration(1.0))
			#print('Test point in the TOOL frame:  x= ', format(pt_in_tool.point.x, '.3f'), '(m), y= ', format(pt_in_tool.point.y, '.3f'), '(m), z= ', format(pt_in_tool.point.z, '.3f'),'(m)')
			#print('Transformed point in the BASE frame:  x= ', format(pt_in_base.point.x, '.3f'), '(m), y= ', format(pt_in_base.point.y, '.3f'), '(m), z= ', format(pt_in_base.point.z, '.3f'),'(m)')
			#print('-------------------------------------------------')

			plan = Plan()
			#plan = Plan()
			plan_point1 = Twist()
			point1_mode = UInt8()
			point1_mode.data = 0
			plan_point1.linear.x = -0.014360220292283805
			plan_point1.linear.y = -0.4087666473460073
			plan_point1.linear.z = 0.27438665254509353
			plan_point1.angular.x = 3.1261380387567526
			plan_point1.angular.y = 0.016664270768824693
			plan_point1.angular.z = 1.5307625983036905
			#ADD points
			plan.points.append(plan_point1)
			plan.modes.append(point1_mode)

			plan_point2 = Twist()
			point2_mode = UInt8()
			point2_mode.data = 0
			plan_point2.linear.x = pt_in_base.point.x
			plan_point2.linear.y = pt_in_base.point.y - 0.01
			plan_point2.linear.z = pt_in_base.point.z + 0.01
			plan_point2.angular.x = 3.1261380387567526
			plan_point2.angular.y = 0.016664270768824693
			plan_point2.angular.z = 1.5307625983036905
			# add this point to the plan
			plan.modes.append(point2_mode)
			plan.points.append(plan_point2)

			plan_pointPU = Twist()
			pointPU_mode = UInt8()
			pointPU_mode.data = 2
			plan_pointPU.linear.x = pt_in_base.point.x
			plan_pointPU.linear.y = pt_in_base.point.y - 0.01
			plan_pointPU.linear.z = pt_in_base.point.z + 0.01
			plan_pointPU.angular.x = 3.1261380387567526
			plan_pointPU.angular.y = 0.016664270768824693
			plan_pointPU.angular.z = 1.5307625983036905
			# add this point to the plan
			plan.modes.append(pointPU_mode)
			plan.points.append(plan_pointPU)

			plan_point3 = Twist()
			point3_mode = UInt8()
			point3_mode.data = 0
			plan_point3.linear.x = pt_in_base.point.x + .2
			plan_point3.linear.y = pt_in_base.point.y - 0.01
			plan_point3.linear.z = pt_in_base.point.z + .15
			plan_point3.angular.x = 3.1261380387567526
			plan_point3.angular.y = 0.016664270768824693
			plan_point3.angular.z = 1.5307625983036905
			# add this point to the plan
			plan.modes.append(point3_mode)
			plan.points.append(plan_point3)


			plan_point4 = Twist()
			point4_mode = UInt8()
			point4_mode.data = 0 
			plan_point4.linear.x = pt_in_base.point.x + .2
			plan_point4.linear.y = pt_in_base.point.y - 0.01
			plan_point4.linear.z = pt_in_base.point.z + 0.01
			plan_point4.angular.x = 3.1261380387567526
			plan_point4.angular.y = 0.016664270768824693
			plan_point4.angular.z = 1.5307625983036905
			# add this point to the plan
			plan.modes.append(point4_mode)
			plan.points.append(plan_point4)

			plan_pointDrop = Twist()
			pointDrop_mode = UInt8()
			pointDrop_mode.data = 1
			plan_pointDrop.linear.x = pt_in_base.point.x + .2
			plan_pointDrop.linear.y = pt_in_base.point.y - 0.01
			plan_pointDrop.linear.z = pt_in_base.point.z + 0.01
			plan_pointDrop.angular.x = 3.1261380387567526
			plan_pointDrop.angular.y = 0.016664270768824693
			plan_pointDrop.angular.z = 1.5307625983036905
			# add this point to the plan
			plan.modes.append(pointDrop_mode)
			plan.points.append(plan_pointDrop)

			plan_point5 = Twist()
			point5_mode = UInt8()
			point5_mode.data = 0
			plan_point5.linear.x = -0.014360220292283805
			plan_point5.linear.y = -0.4087666473460073
			plan_point5.linear.z = 0.27438665254509353
			plan_point5.angular.x = 3.1261380387567526
			plan_point5.angular.y = 0.016664270768824693
			plan_point5.angular.z = 1.5307625983036905
			#ADD points
			plan.modes.append(point5_mode)
			plan.points.append(plan_point5)

			if move_robot:
				plan_pub.publish(plan)
			else:
				if not plan_printed:
					print(plan)
					plan_printed = True
			# wait for 0.1 seconds until the next loop and repeat
			loop_rate.sleep()
