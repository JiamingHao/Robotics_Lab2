#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi, isnan
from sensor_msgs.msg import LaserScan

# Constants
GOAL = 10
LINEAR_SPEED = 0.15
ANGULAR_SPEED = 0.5

range_center = 0
range_left = 0
range_right = 0
motion_to_goal = True
boundary_following = False

def scan_callback(msg):
	range_center = msg.ranges[len(msg.ranges)/2]
	range_left = msg.ranges[len(msg.ranges)-1]
	range_right = msg.ranges[0]
	print "range_center: ", range_center
	print "range_left ", range_left
	print "range_right", range_right
	
	
class Bug2():
	def __init__(self):
		scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
		cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=5)
		rospy.init_node('bug2', anonymous=False)
		rospy.on_shutdown(self.shutdown)
		
		self.tf_listener = tf.TransformListener()
		rospy.sleep(2)
		self.odom_frame = '/odom'
		
		
		try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  
				
		self.position = Point()
		(self.position, self.rotation) = self.get_odom()
		self.x_start = self.position.x
		self.y_start = self.position.y
		
		'''while not rospy.is_shutdown():
			if self.whether_on_mline and range_center > 0.8:
				# move the robot along the m_line
			else:
				motion_to_goal = False
				boundary_following = True
				if range_center <= 0.8:
					self.hit_point = Point()
					# store the hit point
					self.hit_point = self.get_odom()[0]
					# turn left until the obstcale is no longer detected to the right of the robot
					while True:
						# each time turn left in a small degree, and do the checking
						
						if isnan(range_right):
							
							break'''
						
			
		
	def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
	
	def whether_on_mline(self):
		if self.position.x <= 10 and self.position.x >= 0 and self.position.y == 0:
			return True
		else:
			return False
			
	
	
	def shutdown(self):
		# Always stop the robot when shutting down the node.
		rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
		
if __name__ == "__main__":
	robot = Bug2()