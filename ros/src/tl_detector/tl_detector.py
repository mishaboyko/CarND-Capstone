#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.INFO)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.traffic_lights = []
        self.passed_traffic_light_indexes =[]

        # An index of the waypoint within self.waypoints which is closest to the CCP.
        # Rationale: do not loop through waypoints the vehicle has passed.
        self.last_stopline_index = 0

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg
    def waypoints_cb(self, lane_msg):
        self.waypoints = lane_msg.waypoints

    def traffic_cb(self, msg):
        self.traffic_lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            # Be proactive, avoid emergency breaking on the stop line
            light_wp = light_wp if (state == TrafficLight.RED or state == TrafficLight.YELLOW) else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2))

    def update_waypoint_index(self, next_stop_line_coord):
        min_dist_to_wp = self.euclidean_distance(self.waypoints[self.last_stopline_index].pose.pose.position.x, \
                                       self.waypoints[self.last_stopline_index].pose.pose.position.y, \
                                       next_stop_line_coord[0], next_stop_line_coord[1])
        start = self.last_stopline_index
        for i in range(start+1, len(self.waypoints)):
            next_distance = self.euclidean_distance(next_stop_line_coord[0], next_stop_line_coord[1], \
                            self.waypoints[i].pose.pose.position.x, self.waypoints[i].pose.pose.position.y)
            if next_distance < min_dist_to_wp:
                self.last_stopline_index = i
                min_dist_to_wp = next_distance
            else:
                # the loop passed the waypoint, no need to iterate further
                break


    def get_next_stop_line(self, stop_line_positions, next_traffic_light):
        related_stopline = None
        # Assumpition: stopline coordinates come before traffic light coordinates
        dist_to_stopline = 9999999
        for i in range(len(stop_line_positions)):
            dist = self.euclidean_distance(stop_line_positions[i][0], stop_line_positions[i][1], \
                                           next_traffic_light.pose.pose.position.x, next_traffic_light.pose.pose.position.y)
            if dist < dist_to_stopline:
                related_stopline = stop_line_positions[i]
                dist_to_stopline = dist
        return related_stopline

    def get_upcomming_traffic_light(self):
        """Identifies the closest traffic light ahead of the the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            none
        Returns:
            TrafficLight: a an upcomming TL from the current car position

        """
        closest_traffic_light = None
        dist_to_tl = 9999999

        # List of TLs behind the vehicle. First one is ignored by default, cause it is considered before entering the loop.
        for i in range(len(self.traffic_lights)):
            if i in self.passed_traffic_light_indexes:
                continue
            dist = self.euclidean_distance(self.traffic_lights[i].pose.pose.position.x, self.traffic_lights[i].pose.pose.position.y, \
                                 self.pose.pose.position.x, self.pose.pose.position.y)

            if dist < dist_to_tl:
                dist_to_tl = dist
                closest_traffic_light = self.traffic_lights[i]
                # do not consider the TL anymore if we've passed it
                if dist < 4.0:
                    self.passed_traffic_light_indexes.append(i)
                    # rospy.loginfo("Chosen: (%s, %s), dist: %s", \
                    #   closest_traffic_light.pose.pose.position.x, closest_traffic_light.pose.pose.position.y, \
                    #   dist_to_tl)
        return closest_traffic_light

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        #return self.light_classifier.get_classification(cv_image)
        return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        next_traffic_light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            next_traffic_light = self.get_upcomming_traffic_light()

            # use x coordinate only for simplicity
            next_stop_line_coord = self.get_next_stop_line(stop_line_positions, next_traffic_light)
            self.update_waypoint_index(next_stop_line_coord)

            rospy.loginfo("from ccp (%s, %s) closest TL (%s, %s) and stopline (%s, %s) indicates light %s", \
                          self.pose.pose.position.x, self.pose.pose.position.y, \
                          next_traffic_light.pose.pose.position.x, next_traffic_light.pose.pose.position.y, \
                          next_stop_line_coord[0], next_stop_line_coord[1], \
                          next_traffic_light.state)
            rospy.loginfo("Waypoint index of this stopline is %s/%s", self.last_stopline_index, len(self.waypoints))

        if next_traffic_light:
            state = self.get_light_state(next_traffic_light)
            return self.last_stopline_index, state
        # Why would you invalidate all the waypoints forever just because of the missing TL?
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
