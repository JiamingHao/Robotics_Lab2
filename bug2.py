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
position_ = Point()


#publishers
pub_ = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
#rate 
rate = rospy.Rate(20) 

#start point
initial_position_ = Point()
initial_position_.x = 0
initial_position_.y = 0
initial_position_.z = 0

#goal point
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
#                       0 - fix yaw
#                       1 - go straight
#                       2 - done
#------------------------------------
boundary_following_state = 0
# boundary_following_state:
#							0:need to find the boundary
#                           1:turn left
#                           2:need to follow the boundary
#
#--------------------------------------
# callbacks
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
	   30 degree 640/5 = 128'''
    regions_ = {
        'right':  min(min(msg.ranges[0:128]), 10),
        'fright': min(min(msg.ranges[128:256]), 10),
        'front':  min(min(msg.ranges[256:384]), 10),
        'fleft':  min(min(msg.ranges[384:512]), 10),
        'left':   min(min(msg.ranges[512:640]), 10),
    }

'''fix_yaw won't modify the state_, namely itself cannot control the switch 
   between motion_to_goal and  boundary_following mode. It just adjusts the yaw,
   when the yaw is no longer needed to be adjusted, it set  motion_to_goal_state
   properly, then the motion_to_goal mode will switch to go_straight_ahead submode'''
def fix_yaw(des_pos):
    global yaw_, pub_, yaw_precision_
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

'''go_straight_ahead moves the robot ahead, it uses the laser to detect
   whether there is obstacle appeared in the front, if there is, it set 
   state_ properly to switch to the following boundary mode, otherwise it calculates
   the distance between the current position and the goal, depending on different situations,
   there are three cases:
							1. return to the previous hitpoint(no path)
							2. sucessfully drive the robot forward
							3. already reach the goal'''
def go_straight_ahead(des_pos):
	# this method being called indicates that motion_to_goal_state = 1, state_ = 0
    global yaw_, pub_, yaw_precision_, state_
	global regions_
	global hit_position_
	
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
	
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
	'''When the robot is moving towards the goal, if there is a obstacle appears
	   in front of it, switch to following the boundary mode'''
	if regions_['front'] > 0.15 and regions_['front'] < 1:
		# do not forget to record the hitpoint 
		hit_position_ = position_
		
		state_ = 1 # this controls the outer loop 
    
	'''If there is no obstacle appeared yet, calculate the distance from the current position to
	   the goal'''
	elif err_pos > dist_precision_:
		
		# this is the case we cannot find a path
		if position_ == hit_position_:
			state_ = 3
			
		else:	
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
	
def execute_follow_boundary():
	global regions_, position_
	global pub_
	regions = regions_
	msg = Twist()
    linear_x = 0
    angular_z = 0
	state_description = ''
	
	distance_position_to_line = distance_to_line(position_)
	# if the robot hits the m line, switch back to motion_to_goal mode
	
	if distance_position_to_line < 0.1:
		state_ = 0
		return 
		
	d = 1
	if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        pub_.publish(find_wall)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
		pub_.publish(turn_left)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
		pub_.publish(follow_the_wall)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
		pub_.publish(find_wall)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
		pub_.publish(turn_left)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
		pub_.publish(turn_left)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
		pub_.publish(turn_left)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
		pub_.publish(find_wall)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)
	
	

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
    global regions_
    
    msg = Twist()
    msg.linear.x = 0.5
    return msg

def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    global count_state_time_
    count_state_time_ = 0
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)

def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    
    global count_state_time_, count_loop_
	
	global pub_, rate

    rospy.init_node('bug2')

    sub_laser = rospy.Subscriber('scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

	

    # initialize going to the point
    state_ = 0

	
	'''This is the outer loop controling the main flow
	   in this loop, depending on different situations, 
	   the algorithm will switch between motion_to_goal 
	   mode and follow boundary mode, and in each mode, 
	   they at their own actions and will change state
	   in their mode respectively'''
	   
	while not rospy.is_shutdown():
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

        count_loop_ = count_loop_ + 1
        if count_loop_ == 20:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0

        rospy.loginfo("distance to line: [%.2f], position: [%.2f, %.2f]", distance_to_line(position_), position_.x, position_.y)
        rate.sleep()
	
	
