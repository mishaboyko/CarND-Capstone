#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import TwistStamped

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
IGNORE_POSES = 5
DEACCELERATION_WPS = 50

class WaypointUpdater(object):
    current_linear_velocity = 0
    pos_waypoints_passed = 0
    next_waypoints= []
    poses_ignored = 5

    next_stop_index = None
    next_stop_prev_velocity = None

    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.INFO)

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.wait_for_message('/base_waypoints', Lane)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.set_current_velocity)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        self.cte_pub = rospy.Publisher('/cte', Float64, queue_size=1)
        self.ccp_wp_index_pub = rospy.Publisher('/ccp_wp_index', Int32, queue_size=1)

        # keeps the node from exiting until the node has been shutdown.
        rospy.spin()

    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2))

    def calculate_next_waypoints(self):
        # strip track_waypoints to the path begginning at current_vehicle_pose
        # first iteration since startup
        prev_wp_distance = 99999
        if len(self.next_waypoints) < 1:
            waypoints = self.track_waypoints_msg.waypoints
        # re-use list of waypoints cut from CCP
        else:
            waypoints = self.track_waypoints_msg.waypoints
        for i in range(self.pos_waypoints_passed, len(waypoints)):
            i_distance = self.euclidean_distance(self.current_vehicle_pose.pose.position.x, self.current_vehicle_pose.pose.position.y, \
                                                 waypoints[i].pose.pose.position.x, waypoints[i].pose.pose.position.y)
            if i_distance < prev_wp_distance:
                prev_wp_distance = i_distance
            # Local minima reached
            elif i_distance > prev_wp_distance:
                # Extract first LOOKAHEAD_WPS amount of points starting from CCP
                self.pos_waypoints_passed = i-1
                self.next_waypoints = waypoints[self.pos_waypoints_passed:LOOKAHEAD_WPS+self.pos_waypoints_passed]
                # rospy.loginfo("CCP: %s/%s (%s, %s).", i, len(waypoints), waypoints[i].pose.pose.position.x, waypoints[i].pose.pose.position.y)
                break

        self.next_waypoints_msg.waypoints = self.next_waypoints

    def pose_cb(self, msg):
        self.current_vehicle_pose = msg
        if len(self.track_waypoints_msg.waypoints) <= 1:
            rospy.logerr("Unable to publish vehicle path: No track waypoints available")
        else:
            # calculate further path using every 5th waypoint.
            # Rationale: save compitational efforts
            if self.poses_ignored != IGNORE_POSES:
                self.poses_ignored +=1
            else:
                self.poses_ignored = 0
                self.next_waypoints_msg = Lane()
                self.next_waypoints_msg.header.frame_id = self.track_waypoints_msg.header.frame_id
                self.next_waypoints_msg.header.stamp = rospy.Time.now()
                self.calculate_next_waypoints()

                self.final_waypoints_pub.publish(self.next_waypoints_msg)
                self.cte_pub.publish(self.calculate_cte())
                self.ccp_wp_index_pub.publish(self.pos_waypoints_passed+1)

    def waypoints_cb(self, lane_waypoints):
        self.track_waypoints_msg = lane_waypoints

    def traffic_cb(self, msg):
        safe_distance_to_stop_line = 5
        stop_index = msg.data - safe_distance_to_stop_line if msg.data != -1 else len(self.track_waypoints_msg.waypoints)-1
        if self.next_stop_index != stop_index:
            # rospy.loginfo("Upcoming stop: # %s (%s, %s)", stop_index, \
            #               self.track_waypoints_msg.waypoints[stop_index].pose.pose.position.x, \
            #               self.track_waypoints_msg.waypoints[stop_index].pose.pose.position.y)
            if self.next_stop_index is not None:
                self.restore_velocity_to_prev_stop_line()
            self.next_stop_index = stop_index
            self.next_stop_prev_velocity = self.track_waypoints_msg.waypoints[stop_index].twist.twist.linear.x
            self.set_velocity_to_next_stop_line(self.next_stop_index)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def set_current_velocity(self, msg):
        # in m/s
        self.current_linear_velocity = msg.twist.linear.x

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def calculate_cte(self):
        return (self.next_waypoints[0].pose.pose.position.y - self.current_vehicle_pose.pose.position.y)

    def restore_velocity_to_prev_stop_line(self):
        # Idea take velocities within -1<deacceleration range> and intrapolate values back
        start_index = self.next_stop_index - ( DEACCELERATION_WPS +1)
        start_velocity = self.track_waypoints_msg.waypoints[start_index].twist.twist.linear.x
        end_index = self.next_stop_index
        end_velocity = self.next_stop_prev_velocity
        delta_v = (end_velocity - start_velocity)/(end_index-start_index)
        # rospy.loginfo("Restoring acceleration from %s (index %s ) to %s (index %s ) in %s steps", \
        #               start_velocity, start_index, end_velocity, end_index, delta_v)

        cnt = 0
        for i in (range(start_index, end_index+1)):
            velocity = start_velocity+(cnt*delta_v)
            self.set_waypoint_velocity(self.track_waypoints_msg.waypoints, i, velocity)
            cnt +=1

    def set_velocity_to_next_stop_line(self, stop_waypoint_index):
        # By default use DEACCELERATION_WPS prior to the stop line waypoint for deacceleration.
        # If vehicle is already closer than DEACCELERATION_WPS to the red TL: use vehicle position for deacceleration start
        deacceleration_start_index = stop_waypoint_index - DEACCELERATION_WPS
        deacceleration_start_velocity = self.track_waypoints_msg.waypoints[deacceleration_start_index].twist.twist.linear.x
        if deacceleration_start_index < self.pos_waypoints_passed:
            deacceleration_start_index = self.pos_waypoints_passed
            deacceleration_start_velocity = self.current_linear_velocity
        self.set_deacceleration_velocities(stop_waypoint_index, deacceleration_start_index, deacceleration_start_velocity)

    def set_deacceleration_velocities(self, stop_waypoint_index, deacceleration_start_index, deacceleration_start_velocity):
        # simple linear deacceleration
        # rospy.loginfo("Deaccelerating in range %s, %s from start_velocity %s to 0", \
        #               deacceleration_start_index, stop_waypoint_index, deacceleration_start_velocity)
        deacceleration_koefficient = deacceleration_start_velocity/DEACCELERATION_WPS
        deacc_wp = DEACCELERATION_WPS

        for i in reversed(range(deacceleration_start_index, stop_waypoint_index)):
            velocity = deacceleration_start_velocity-(deacc_wp*deacceleration_koefficient)
            if velocity < 0.4:
                # set -1 to the last waypoint in order to generate 700 Nm torque and prevent Carla from moving
                velocity = -1
            self.set_waypoint_velocity(self.track_waypoints_msg.waypoints, i, velocity)
            deacc_wp -=1

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
