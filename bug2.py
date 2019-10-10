#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
import math

class OutAndBack():
    def __init__(self):
        rospy.init_node('out_and_back', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        # Publisher to control the robot's speed
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.tf_listener = tf.TransformListener() # Initialize the tf listener
        rospy.sleep(2) # Give tf some time to fill its buffer
        self.odom_frame = '/odom' # Set the odom frame
        # Find out if the robot uses /base_link or /base_footprint
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

        # Set speeds
        self.rate = rospy.Rate(20)
        self.linear_speed = 0.15
        self.angular_speed = 0.5
        self.linear_unit = 0.3
        self.angular_unit = math.pi / 6.

        # Initialize points
        self.cur_pos = Point()
        (self.cur_pos, self.rotation) = self.get_odom()
        self.start_position = self.cur_pos
        self.q_goal = Point()
        self.q_goal.x = 10.0
        self.q_goal.y = 0.0
        self.q_goal.z = 0.0
        self.q_H = None

        # Flags
        self.has_printed_hit = False
        self.has_corrected_yaw = False
        self.has_hit_object = False
        self.cur_action = 'gf' # 'tl': turn left; 'tr': turn right; 'fw': go foward
        self.cur_status = 'm' # 'm': moving along m-line; 'o': moving along obstacle

        # Set parameters for actions
        self.next_action_time = rospy.Time.now()
        self.safe_distance = 1.25
        self.hit_mline_threshold = 0.15
        self.goal_tolarence = 0.5
        self.hit_point_tolarence = 0.8

        while self.get_distance_between(self.cur_pos, self.q_goal) > self.goal_tolarence and not rospy.is_shutdown():
            if rospy.Time.now() > self.next_action_time:
                if self.get_mline_distance() <= self.hit_mline_threshold:
                    if self.cur_status == 'm':
                        if not math.isnan(g_range_ahead) and g_range_ahead < self.safe_distance:
                            if not self.has_printed_hit:
                                print("Obstacle hit!")
                                self.has_printed_hit = True
                            self.q_H = self.cur_pos
                            self.q_H.x += self.safe_distance / 2.
                            self.has_hit_object = True
                            self.cur_action = 'tl'
                            self.next_action_time = rospy.Time.now() + rospy.Duration(self.angular_unit / 4. / self.angular_speed)
                        else:
                            if self.get_mline_distance() <= self.hit_mline_threshold / 2. or self.has_corrected_yaw or self.has_hit_object:
                                self.cur_action = 'gf'
                                self.next_action_time = rospy.Time.now() + rospy.Duration(self.linear_unit / self.linear_speed)
                                self.has_corrected_yaw = False
                            else:
                                self.get_goal_direction()
                                self.has_corrected_yaw = True
                    elif self.cur_status == 'o':
                        self.has_hit_object = False
                        if self.get_distance_between(self.cur_pos, self.q_H) <= self.hit_point_tolarence:
                            print("Goal is unreachable!!!")
                            break
                        elif self.get_distance_between(self.cur_pos, self.q_goal) < self.get_distance_between(self.q_H, self.q_goal):
                            if self.cur_pos.x < self.q_goal.x:
                                print("Hit m-line, turn towards goal!")
                                self.get_goal_direction()
                                self.cur_status = 'm'
                        else:
                            #print("Hit m-line, not closer to the goal than hit point, keep following the boundary...")
                            self.follow_boundary()
                    else:
                        print("[Wrong status]")
                else: # Moving along obstacle, not m_line
                    self.cur_status = 'o'
                    self.has_printed_hit = False
                    self.follow_boundary()

            # Execute the action chosen above
            twist = Twist()
            if self.cur_action == 'tl':
                twist.angular.z = self.angular_speed
            elif self.cur_action == 'tr':
                twist.angular.z = -self.angular_speed
            elif self.cur_action == 'gf':
                twist.linear.x = self.linear_speed
            else:
                print("[Wrong state]")
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

            # Update the current position
            (self.cur_pos, self.rotation) = self.get_odom()

            # Check if goal is reached
            if self.get_distance_between(self.cur_pos, self.q_goal) <= self.goal_tolarence:
                print("Goal reached!!!")
                break

        # Stop the robot for good
        self.cmd_vel_pub.publish(Twist())
        #========================== END of __init__ ==========================

    def follow_boundary(self):
        if self.cur_action == 'tl':
            self.cur_action = 'gf'
            self.next_action_time = rospy.Time.now() + rospy.Duration(self.linear_unit / self.linear_speed)
        elif self.cur_action == 'tr':
            if math.isnan(g_range_ahead) or g_range_ahead > self.safe_distance:
                self.cur_action = 'gf'
                self.next_action_time = rospy.Time.now() + rospy.Duration(self.linear_unit / self.linear_speed)
            else:
                self.cur_action = 'tl'
                self.next_action_time = rospy.Time.now() + rospy.Duration(self.angular_unit / self.angular_speed)
        elif self.cur_action == 'gf':
            if math.isnan(g_range_ahead) or g_range_ahead > self.safe_distance:
                self.cur_action = 'tr'
                self.next_action_time = rospy.Time.now() + rospy.Duration(self.angular_unit / self.angular_speed)
            else:
                self.cur_action = 'tl'
                self.next_action_time = rospy.Time.now() + rospy.Duration(self.angular_unit / self.angular_speed)
        else:
            print("[Wrong action]")

    def get_goal_direction(self):
        rho_robot = math.atan2(self.q_goal.y - self.cur_pos.y, self.q_goal.x - self.cur_pos.x)
        yaw_err = rho_robot - self.rotation
        if yaw_err < 0:
            self.cur_action = 'tr'
        else:
            self.cur_action = 'tl'
        self.next_action_time = rospy.Time.now() + rospy.Duration(abs(yaw_err) / self.angular_speed)

    def get_mline_distance(self):
        # p0 is the current position
        p0 = self.cur_pos
        # p1 and p2 points define the line
        p1 = self.start_position
        p2 = self.q_goal
        up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
        lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
        distance = up_eq / lo_eq
        return distance

    def get_distance_between(self, p1, p2):
        return math.sqrt(math.pow((p1.x - p2.x), 2) + math.pow((p1.y - p2.y), 2))

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

    def scan_callback(self, msg):
        global g_range_ahead
        cleaned_ranges = [range for range in msg.ranges if str(range) != 'nan']
        if len(cleaned_ranges) > 0:
            g_range_ahead = min(cleaned_ranges)
        else:
            g_range_ahead = float('nan')

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")
