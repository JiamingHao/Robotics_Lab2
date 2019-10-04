#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from tf import transformations
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi, isnan
from sensor_msgs.msg import LaserScan

import math

# Some global variables
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degrees
dist_precision_ = 0.3
# robot current position
position_ = Point()


# publisher, make it global is convenient to share accross all methods
pub_ = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)


# start point
initial_position_ = Point()
initial_position_.x = 0
initial_position_.y = 0
initial_position_.z = 0

# goal point
desired_position_ = Point()
desired_position_.x = 10
desired_position_.y = 0
desired_position_.z = 0

# where the robot encountered the obstacle
hit_position_ = Point()


regions_ = None
state_desc_ = ['Go to point', 'wall following']
state_ = 0
count_state_time_ = 0 # seconds the robot is in a state
count_loop_ = 0

# 0 - go to point
# 1 - wall following
# 2 - reach the goal
# 3 - no paths
#---------------------------------
motion_to_goal_state = 0
# motion_to_goal_state:
#                           0 - fix yaw
#                           1 - go straight
#                           2 - done
#------------------------------------
boundary_following_state = 0
# boundary_following_state:
#			    0:need to find the boundary
#                           1:turn left
#                           2:need to follow the boundary
#
#--------------------------------------

# callback functions
def clbk_odom(msg):
    global position_, yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]
	
	
def clbk_laser(msg):
	global regions_
	'''the len of msg.range is 640, the range of the angle is from -30 degree to 
	   30 degree 640/5 = 128. The whole range of the laser scanner is 
	   divided into 5 sub-regions, each is approimately 12 degree'''
	# msg.ranges[0] = rightmost scan msg.ranges[len(msg.ranges)-1] = leftmost
	regions_ = {
        'right':  min(min(msg.ranges[0:128]), 10),
        'fright': min(min(msg.ranges[128:256]), 10),
        'front':  min(min(msg.ranges[256:384]), 10),
        'fleft':  min(min(msg.ranges[384:512]), 10),
        'left':   min(min(msg.ranges[512:640]), 10),
	}

'''fix_yaw won't modify the state_, namely, itself cannot control the switch 
between motion_to_goal and  boundary_following mode. It just adjusts the yaw,
when the yaw is no longer needed to be adjusted, it set  motion_to_goal_state
properly, then the motion_to_goal mode will switch to go_straight_ahead submode'''
def fix_yaw(des_pos):
	global yaw_, pub_, yaw_precision_
	
	# use arctan to calculate the correct angle towards the goal
	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
	err_yaw = desired_yaw - yaw_
    
	twist_msg = Twist()
    
	if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.3 if err_yaw > 0 else -0.3
    
	# this action will only rotate the robot(won't move the roboy)
	pub_.publish(twist_msg)
    
	# state change conditions
	if math.fabs(err_yaw) <= yaw_precision_:
		print 'Yaw error: [%s]' % err_yaw
		motion_to_goal_state = 1
	return 
	
'''go_straight_ahead moves the robot ahead, it uses the laser to detect
   whether there is obstacle appeared in the front, if there is, it set 
   state_ properly to switch to the following boundary mode, otherwise it calculates
   the distance between the current position and the goal, depending on different situations,
   there are three cases:	1. return to the previous hitpoint(no path)
				2. sucessfully drive the robot forward
				3. already reach the goal
'''
def go_straight_ahead(des_pos):
	
	# this method being called indicates that motion_to_goal_state = 1, state_ = 0
	global yaw_, pub_, yaw_precision_, state_, dist_precision_
	global regions_
	global hit_position_, position_
	
	desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
	
	# keep updating the error yaw every time being called
	err_yaw = desired_yaw - yaw_
	
	err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
	'''When the robot is moving towards the goal, if there is a obstacle appears
	   in front of it, switch to following the boundary mode'''
	if regions_['front'] > 0.15 and regions_['front'] < 1:
		# do not forget to record the hitpoint 
		hit_position_ = position_
		
		state_ = 1 # this controls the outer loop, switch to follow boundary mode 
    
	'''If there is no obstacle appeared yet, calculate the distance from the current position to
	   the goal'''
	# if the robot still hasn't reached the goal
	elif err_pos > dist_precision_:
		twist_msg = Twist()
		twist_msg.linear.x = 0.3
		pub_.publish(twist_msg)
    
	else:
        print 'Position error: [%s]' % err_pos
		# reached the goal
        state_ = 2
		
    
	# state change conditions
	if math.fabs(err_yaw) > yaw_precision_:
        	print 'Yaw error: [%s]' % err_yaw
		motion_to_goal = 0 # need to fix yaw, while state_ is still 0
		
def execute_motion_to_goal():
	global desired_position_
	
	if motion_to_goal_state == 0:
		fix_yaw(desired_position_)
	
	elif motion_to_goal_state == 1:
		go_straight_ahead(desired_position_)
	
	else:
		print "ERROR: Unknown motion_to_goal state."
	
def distance_to_line(p0):
	# p0 is the current position
	# p1 and p2 points define the line
	global initial_position_, desired_position_
	p1 = initial_position_
	p2 = desired_position_
	# here goes the equation
	up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
	lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
	distance = up_eq / lo_eq

	return distance

'''Execute the following boundary mode, once enter this mode, there are two ways to switch 
   to other modes, either the robot hits the m_line, then it switches back to motion-to-goal mode, or
   it coms back to the hitpoint, then we can draw the conclusion that there are no paths to the goal. Apart
   from the above two cases, the robot will continue to follow the boundary and state_ keeps being 1
'''
def execute_follow_boundary():
	global regions_, position_, hit_position_, dist_precision_
	global pub_
	regions = regions_
	msg = Twist()
	linear_x = 0
	angular_z = 0
	state_description = ''
	
	distance_position_to_line = distance_to_line(position_)
	# if the robot hits the m line, switch back to motion_to_goal mode
	dist_from_hit = math.sqrt(pow(hit_position_.y - position_.y, 2) + pow(hit_position_.x - position_.x, 2))
	
	if distance_position_to_line < 0.1:
		state_ = 0
		return 
	elif dist_from_hit <= dist_precision_:
		state_ = 3 # no paths found
		return 
	
	d = 1
	if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        	state_description = 'case 1 - No obstacle sensed at all'
        	pub_.publish(find_wall())
	elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        	state_description = 'case 2 - Sensed obstacle in front, but nothing on left and right'
		# let robot turn left
		pub_.publish(turn_left())
    	elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        	state_description = 'case 3 - Sensed obstacle on right side, nothing in front and left'
		# let robot follow the boundary
		pub_.publish(follow_the_wall())
    	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        	state_description = 'case 4 - Sensed obstacle on the left side, nothing in front and right'
		# let robot find the wall, namely go ahead and at the same time, slightly turning right
		pub_.publish(find_wall())
    	elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        	state_description = 'case 5 - Sensed obstacle in front and right, nothing in left'
		# let robot turn left
		pub_.publish(turn_left())
    	elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        	state_description = 'case 6 - Sensed obstacle in front and left, nothing in right'
		# let robot turn left
		pub_.publish(turn_left())
    	elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        	state_description = 'case 7 - Sensed obstalce in front, left an right'
		# let robot turn left
		pub_.publish(turn_left())
    	elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        	state_description = 'case 8 - Sensed obtacle in right and left, nothing in front'
		# let robot find the wall
		pub_.publish(find_wall())
    	else:
        	state_description = 'ERROR: unknown case'
        	rospy.loginfo(regions) # for debug purpose
	return 
	
'''Find the wall action actually contains the right side roatation while moving forward'''
def find_wall():
	msg = Twist()
	msg.linear.x = 0.2
	msg.angular.z = -0.3
    	return msg

def turn_left():
    	msg = Twist()
    	msg.angular.z = 0.3
    	return msg

def follow_the_wall():
    	msg = Twist()
    	msg.linear.x = 0.5
    	return msg

def main():
    	global regions_, position_, state_
    
    	global count_state_time_, count_loop_
	
	global pub_, rate

    	rospy.init_node('bug2')

    	sub_laser = rospy.Subscriber('scan', LaserScan, clbk_laser)
    	sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
	
	rate = rospy.Rate(20) 
	

    	# initialize to going to the point
    	state_ = 0

	
	'''This is the outer loop controling the main flow
	   in this loop, depending on different situations, 
	   the algorithm will switch between motion_to_goal 
	   mode and follow boundary mode, and in each mode, 
	   they at their own actions and will change state
	   in their mode respectively
	'''
	   
	while not rospy.is_shutdown():
		# if the scan data has not arrived
		if regions_ == None:
			continue

		distance_position_to_line = distance_to_line(position_)
		
        	if state_ == 0:
			execute_motion_to_goal()

        	elif state_ == 1:
			execute_follow_boundary()
			
		elif state_ == 2:
			print "Reached the goal!"
			break
			
		elif state_ == 3:
			print "No paths found!"
			break
			
		else:
			print "Unknown state",state_

		rospy.loginfo("distance to line: [%.2f], position: [%.2f, %.2f]", distance_to_line(position_), position_.x, position_.y)
        	rate.sleep()
	
	
