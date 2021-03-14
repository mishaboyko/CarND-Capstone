#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight, Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3
ACTIVATE_TL_CLASSIFIER_RANGE = 100
EXPORT_DIR='/home/backups/'
SKIP_IMAGES_THRESHOLD = 2

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.INFO)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.ccp_wp_index = None
        self.traffic_lights = []
        self.passed_traffic_light_indexes =[]
        self.classified_tl_state = 0

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

        ccp_wp_index_subscriber = rospy.Subscriber('/ccp_wp_index', Int32, self.ccp_wp_index_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        # List of positions that correspond to the line to stop in front of for a given intersection
        self.stop_line_positions = self.config['stop_line_positions']

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.skip_images_cnt = SKIP_IMAGES_THRESHOLD
        self.images_counter = 0


        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, lane_msg):
        self.waypoints = lane_msg.waypoints
        self.stop_line_wps = self.get_stopline_waypoints()


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

    def ccp_wp_index_cb(self, msg):
        self.ccp_wp_index = msg.data

    def get_stopline_waypoints(self):
        """Helper function which returns set the waypoint indexes
        for all stoplines in the track.
        This is then used to start TL color classification only within those regions,
        where the TLs are visible from CCP.

        Returns:
            List of all waypoint indexes of the traffic lights on the whole track
        """
        all_stopline_wps = []
        for stop_line in self.stop_line_positions:
            min_dist_to_stopline_wp = 999999
            stopline_index = 999999
            for waypoint_index in range(len(self.waypoints)):
                next_distance = self.euclidean_distance(stop_line[0], stop_line[1], \
                                                        self.waypoints[waypoint_index].pose.pose.position.x, \
                                                        self.waypoints[waypoint_index].pose.pose.position.y)
                if next_distance < min_dist_to_stopline_wp:
                    stopline_index = waypoint_index
                    min_dist_to_stopline_wp = next_distance
            rospy.loginfo("Stop line (%s, %s,) index %s", stop_line[0], stop_line[1], stopline_index)
            all_stopline_wps.append(stopline_index)
        return all_stopline_wps

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

    def get_next_stop_line(self, next_traffic_light):
        related_stopline = None
        # Assumpition: stopline coordinates come before traffic light coordinates
        dist_to_stopline = 9999999
        for i in range(len(self.stop_line_positions)):
            dist = self.euclidean_distance(self.stop_line_positions[i][0], self.stop_line_positions[i][1], \
                                           next_traffic_light.pose.pose.position.x, next_traffic_light.pose.pose.position.y)
            if dist < dist_to_stopline:
                related_stopline = self.stop_line_positions[i]
                dist_to_stopline = dist
        return related_stopline

    def get_upcoming_traffic_light(self):
        """Identifies the closest traffic light ahead of the the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            none
        Returns:
            TrafficLight: a an upcomming TL from the current car position

        """
        closest_traffic_light = None
        dist_to_tl = 9999999

        # vehicle has passed last traffic light on the track. Reset values
        # Assumption: every TL has exectly 1 stop line and all SL in the config are valid
        if len(self.passed_traffic_light_indexes) >= len(self.stop_line_positions):
            rospy.logwarn("have passed last traffic light. Re-setting passed_traffic_light_indexes")
            self.passed_traffic_light_indexes = []

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

    def process_image(self, cv_image, tl_light_state):
        """Helper function to:
        - export images that are close to the upcoming TL
            in order to use them for training the TL classifier
        - classify images that are close to the upcoming TL
        Range:
        -ACTIVATE_TL_CLASSIFIER_RANGE____CCP____+10

        Args:
            tl_light_state (State of the TrafficLight): used to:
                - label the image for classifier
                - double-check the correctness of the classifier

        """
        tl_state = 0
        rospy.loginfo("CCP index: %s/%s", self.ccp_wp_index, len(self.waypoints))
        for stop_line_wp in self.stop_line_wps:
            if stop_line_wp-ACTIVATE_TL_CLASSIFIER_RANGE <= self.ccp_wp_index < stop_line_wp+10:

                # Uncomment this to store images on the Filesystem for later training and testing
                filename = "tl_{}_state{}_frame_{}.png".format(stop_line_wp, tl_light_state, self.images_counter)
                self.images_counter +=1
                cv2.imwrite(EXPORT_DIR+filename, cv_image)

                # Get classification
                tl_state = self.light_classifier.get_classification(cv_image)
        return tl_state

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

        tl_color = self.process_image(cv_image, light.state)
        return tl_color

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        next_traffic_light = None

        if(self.pose):
            next_traffic_light = self.get_upcoming_traffic_light()

            next_stop_line_coord = self.get_next_stop_line(next_traffic_light)
            self.update_waypoint_index(next_stop_line_coord)

            # rospy.loginfo("from ccp (%s, %s) closest TL (%s, %s) and stopline (%s, %s) indicates light %s", \
            #               self.pose.pose.position.x, self.pose.pose.position.y, \
            #               next_traffic_light.pose.pose.position.x, next_traffic_light.pose.pose.position.y, \
            #               next_stop_line_coord[0], next_stop_line_coord[1], \
            #               next_traffic_light.state)
            rospy.loginfo("Next stopline index: %s/%s", self.last_stopline_index, len(self.waypoints))

            # process every SKIP_IMAGES_CNT image
            if self.skip_images_cnt == SKIP_IMAGES_THRESHOLD:
                self.classified_tl_state = self.get_light_state(next_traffic_light)
                self.skip_images_cnt = 0
            else:
                self.skip_images_cnt +=1

        if next_traffic_light:
            # self.classified_tl_state = next_traffic_light.state
            return self.last_stopline_index, self.classified_tl_state
        # Why would you invalidate all the waypoints forever just because of the missing TL?
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
