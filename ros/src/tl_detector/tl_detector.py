#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, Point
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import time

STATE_COUNT_THRESHOLD = 1

class TLDetector(object):

    # Fine state machine states
    FSM_STATE_IDLE = 0
    FSM_STATE_STOP_LINE = 1
    FSM_STATE_STOP_CLASSIFY = 2
    FSM_STATE_GO = 3
    
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.last_pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = None
        self.lights_wp = None

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
        # KB 10Oct 2017
        # adding the path to the light classifier initialisation call in the line below
        self.light_classifier = TLClassifier("light_classification/model/GAN")
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.fsm_state = TLDetector.FSM_STATE_IDLE
        self.npose = 0
        self.move_threshold = .001

        rospy.spin()

    def pose_cb(self, msg):
        self.last_pose = self.pose
        self.pose = msg

    def waypoints_cb(self, lane):
        if hasattr(self, 'waypoints') and self.waypoints != lane.waypoints:
            self.waypoints = lane.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights
        if self.waypoints and not self.lights_wp:
            lights_wp = []
            for tl in self.lights:
                tl_position = tl.pose.pose.position
                tl_wp = self.get_closest_waypoint(tl_position)
                lights_wp.append(tl_wp)
            self.lights_wp = lights_wp

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.camera_image = msg

        print "FSM state : ", self.fsm_state
        
        if self.fsm_state == TLDetector.FSM_STATE_IDLE:            
            if self.pose and self.last_pose and self.waypoints and self.lights:
##                self.upcoming_red_light_pub.publish(Int32(-1))

                # Init classifier
                self.get_light_state(self.lights[0])
                  
                self.state_count = 0
                self.fsm_state = TLDetector.FSM_STATE_STOP_LINE
            
        elif self.fsm_state == TLDetector.FSM_STATE_STOP_LINE:
            # Publish stop line
            light, light_wp = self.process_traffic_lights()
            self.upcoming_red_light_pub.publish(Int32(light_wp))
            
            # Check if car is stopped
            if light_wp >= 0:
                current_car_position = self.pose.pose.position
                last_car_position = self.last_pose.pose.position
                current_car_wp = self.get_closest_waypoint(current_car_position)
                if (self.distance_between_two_points(current_car_position, last_car_position) < self.move_threshold) and ((light_wp - current_car_wp) <= 2):
                    self.state_count += 1
                    if self.state_count >= STATE_COUNT_THRESHOLD:
                        self.state_count = 0
                        self.fsm_state = TLDetector.FSM_STATE_STOP_CLASSIFY
                else:
                    self.state_count = 0
            else:
                self.state_count = 0
                       
        elif self.fsm_state == TLDetector.FSM_STATE_STOP_CLASSIFY:
            # Publish stop stop and begin classification
            light, light_wp = self.process_traffic_lights()
            self.upcoming_red_light_pub.publish(Int32(light_wp))

            if light_wp < 0:
                lightstate = TrafficLight.UNKNOWN
            else:
                lightstate = self.get_light_state(light)
            
            if lightstate != TrafficLight.RED:
                self.state_count += 1
                if self.state_count >= STATE_COUNT_THRESHOLD:
                    self.state_count = 0
                    self.fsm_state = TLDetector.FSM_STATE_GO
            else:
                self.state_count = 0
                
        elif self.fsm_state == TLDetector.FSM_STATE_GO:
            self.upcoming_red_light_pub.publish(Int32(-1))
            self.state_count += 1
            if self.state_count >= STATE_COUNT_THRESHOLD:
                self.state_count = 0
                
                # Check if car is moving
                current_car_position = self.pose.pose.position
                last_car_position = self.last_pose.pose.position                    
                if self.distance_between_two_points(current_car_position, last_car_position) >= self.move_threshold:                
                    self.fsm_state = TLDetector.FSM_STATE_STOP_LINE
            

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement ( should be optimized)
        minimum_distance_value = 999999
        closest_waypoint_index = 0
        current_waypoint_index = 0
        for waypoint in self.waypoints:
            waypoint_pos = waypoint.pose.pose.position
            distance_to_waypoint = self.distance_between_two_points(pose, waypoint_pos)
            if distance_to_waypoint < minimum_distance_value:
                minimum_distance_value = distance_to_waypoint
                closest_waypoint_index = current_waypoint_index
            current_waypoint_index += 1
        return closest_waypoint_index



    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image
        # KB 10Oct2017
        # commenting line below, and using dummy values, so that tf_classifier.py can be tested 
        # x, y = self.project_to_image_plane(light.pose.pose.position)
        x = 0
        y = 0

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.camera_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image

        #TODO: DEBUG (Salim) this is only for debugging, this reads the
        # current light status from the simulator instead of getting it
        # from the classifier please remove the following line as soon as
        # the classifer works.
        # KB 10Oct2017
        # commenting out line below
        # return light.state

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def distance_between_two_points(self, p1, p2):
        """Calculates the distance between two given points

        Returns: the distance between the two given points
        """
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_wp = None

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose.position)
        
        if (self.lights):
            distance_to_closest_light = 9999999
            # loop throw all traffic lights
            for (idx, tl) in enumerate(self.lights):
                # if the traffic light is in front of the car and its distance is smaller then the reference distance:
                # TODO: change the value 200 to a more rational value
                tl_wp = self.lights_wp[idx]
                if tl_wp <= car_position or tl_wp > car_position + 300:
                    continue

                # calculate the distance between the current car position and the traffic light
                current_car_position = self.pose.pose.position
                tl_position = tl.pose.pose.position                
                distance = self.distance_between_two_points(current_car_position, tl_position)
                if distance < distance_to_closest_light:
                    light = tl
                    light_wp = tl_wp
                    distance_to_closest_light = distance
        
        if light:
            closest_ss_wp_index = self.get_stopline_wp_before_tl(light_wp)
##            print " -- process_traffic_lights: light_wp = ", light_wp, " vs. closest_ss_wp_index = " , closest_ss_wp_index
            return light, closest_ss_wp_index
        else:
            return None, -1
    
    def get_stopline_wp_before_tl(self, light_wp_index):
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        current_car_position = self.pose.pose.position
        light_position = self.waypoints[light_wp_index].pose.pose.position
        stop_line_position = Point()
        
        distance_to_closest_stopline = 9999999
        closest_ss_wp_index = -1
        
        for idx, sp in enumerate(stop_line_positions):
            stop_line_position.x = sp[0]
            stop_line_position.y = sp[1]
            distance = self.distance_between_two_points(light_position, stop_line_position)
            # print " -- stop_line_position : x = ", sp[0], " / y = ", sp[1], " / distance to light_wp = ", distance
            
            # stop-sign waypoint index
            ss_wp_index = self.get_closest_waypoint(stop_line_position)
            
            if ss_wp_index < light_wp_index and distance < distance_to_closest_stopline:
                closest_ss_wp_index = ss_wp_index
                distance_to_closest_stopline = distance
            
        if closest_ss_wp_index == -1:
            # no waypoint found for the stopline which is before the waypoint for the TL
            #  -> then stop at the the TL
            return light_wp_index
        
        return closest_ss_wp_index

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
