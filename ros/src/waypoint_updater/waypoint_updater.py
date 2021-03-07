#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

class WaypointUpdater(object):
    pos_waypoints_passed = 0
    next_waypoints= []
    poses_ignored = 5

    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.wait_for_message('/base_waypoints', Lane)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        # subscriber for /traffic_waypoint and /obstacle_waypoint below
        # rospy.Subscriber('/traffic_waypoint', Waypoint, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Waypoint, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # keeps the node from exiting until the node has been shutdown.
        rospy.spin()

    def calculate_next_waypoints(self):

        # strip track_waypoints to the path begginning at current_vehicle_pose
        # first iteration since startup
        if len(self.next_waypoints) < 1:
            waypoints = self.track_waypoints_msg.waypoints
        # re-use list of waypoints cut from CCP
        else:
            waypoints = self.track_waypoints_msg.waypoints
        for i in range(self.pos_waypoints_passed, len(waypoints)):
            if self.current_vehicle_pose.pose.position.x < waypoints[i].pose.pose.position.x:
                # Extract first LOOKAHEAD_WPS amount of points starting from CCP
                self.pos_waypoints_passed = i
                self.next_waypoints = waypoints[self.pos_waypoints_passed:LOOKAHEAD_WPS+self.pos_waypoints_passed]
                break

        self.next_waypoints_msg.waypoints = self.next_waypoints[:LOOKAHEAD_WPS]

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
                rospy.loginfo("Publishing vehicle path")
                self.next_waypoints_msg = Lane()
                self.next_waypoints_msg.header.frame_id = self.track_waypoints_msg.header.frame_id
                self.next_waypoints_msg.header.stamp = rospy.Time.now()
                self.calculate_next_waypoints()

                self.final_waypoints_pub.publish(self.next_waypoints_msg)

    def waypoints_cb(self, lane_waypoints):
        rospy.loginfo("Track Waypoints received")
        self.track_waypoints_msg = lane_waypoints

    def traffic_cb(self, msg):
        rospy.loginfo("traffic_cb triggered")
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        rospy.loginfo("obstacle_cb triggered")
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
