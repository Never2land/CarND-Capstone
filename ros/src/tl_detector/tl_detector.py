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
IMG_SIZE = 32

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.base_waypoints = None
        self.camera_image = None
        self.lights = []

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

        #self.speed_limit = float(self.kmph2mps(rospy.get_param('waypoint_loader/velocity')))

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, msg):
        self.base_waypoints = msg.waypoints
        #rospy.loginfo('base_waypoints[100] = %d', self.base_waypoints[100].pose.pose.position.x)

    def traffic_cb(self, msg):
        self.lights = msg.lights

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
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, car_pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_dist = 999999
        closest_index = -1

        # Find waypoint closest to vehicle position
        for i in range(len(self.base_waypoints)):
            dist = self.get_dist(self.base_waypoints[i].pose.pose.position, car_pose.position)
            if dist < closest_dist:
                closest_index = i
                closest_dist = dist
        return closest_index

    def buffer_dist(self, current_velocity, decel):
        v = current_velocity.twist.linear.x
        return 0.5 * v * v / decel

    def get_closest_waypoint_light(self, light_pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        min_dist = 999999
        min_index = -1

        # Find waypoint closest to vehicle position
        for i in range(len(self.base_waypoints)):
            dist = self.get_dist_light(self.base_waypoints[i].pose.pose.position, light_pose)
            if dist < min_dist:
                min_index = i
                min_dist = dist
        return min_index

    def get_dist(self, pos1, pos2):
        return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z)**2)

    def get_dist_light(self, waypoint_pose, light_pose):
        return math.sqrt((waypoint_pose.x - light_pose[0])**2 + (waypoint_pose.y - light_pose[1])**2)

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
        light_image = cv2.resize(cv_image, (IMG_SIZE, IMG_SIZE), interpolation = cv2.INTER_CUBIC)
        '''
        bbox = self.project_to_image_plane(light)
        if bbox is None:
            return TrafficLight.UNKNOW

        x1, y1, x2, y2 = bbox
        if x1 is not None and abs(y2-y1) > 70 and abs(x2-x1) > 70:
            light_roi = cv_image[y1:y2, x1:x2]
            light_image = cv2.resize(light_roi, (IMG_SIZE, IMG_SIZE), interpolation = cv2.INTER_CUBIC)
        '''

        #Get classification
        return self.light_classifier.get_classification(light_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        #rospy.loginfo("stop_line_positions: x= %d, y= %d", stop_line_positions[0][0],stop_line_positions[0][1])

        light_waypoint_pos = []

        if self.base_waypoints is None:
            return -1, TrafficLight.UNKNOWN
        for i in range(len(stop_line_positions)):
            light_pos = self.get_closest_waypoint_light(stop_line_positions[i])
            light_waypoint_pos.append(light_pos)

        #rospy.loginfo("light pos = %i", light_waypoint_pos[1])
        #rospy.loginfo("light_waypoint_pos = %d", self.base_waypoints[light_waypoint_pos[0]].pose.pose.position.x)

        self.last_tl_pos_wp = light_waypoint_pos

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            if car_position is not None:
                self.last_car_position = car_position
        else:
            return -1, TrafficLight.UNKNOWN

        if self.last_car_position > max(self.last_tl_pos_wp):
            light_num_wp = min(self.last_tl_pos_wp)
        else:
            light_delta = self.last_tl_pos_wp[:]
            light_delta[:] = [x - self.last_car_position for x in light_delta]
            light_num_wp = min(i for i in light_delta if i >= 0) + self.last_car_position

        light_index = self.last_tl_pos_wp.index(light_num_wp)
        light = stop_line_positions[light_index]

        #light_distance = self.distance_light(light, point.pose.pose.position)
        light_distance = self.get_dist_light(self.base_waypoints[self.last_car_position].pose.pose.position, light)
        search_for_light_distance = 5

        if light:
            if light_distance >= search_for_light_distance:
                return -1, TrafficLight.UNKNOWN
            else:
                state = self.get_light_state(light)
                rospy.loginfo('Traffic light index = %i, state = %d', light_num_wp, state)
                return light_num_wp, state

        self.base_waypoints = None
        return -1, TrafficLight.UNKNOWN

    def process_traffic_lights_2(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        tl_waypoint_pos = []


    def distance_light(self, pos1, pos2):
        x = pos1[0] - pos2.x
        y = pos1[1] - pos2.y
        #z = pos1.z - pos2.z

        return math.sqrt(x*x + y*y)

    def project_with_fov(self, d, x, y):
        # Camera characteristics
        fov_x = self.config['camera_info']['focal_length_x']
        fov_y = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        img_x = 0.5*image_width - 2574*x/d
        img_y = image_height - 2740*y/d

        img_x = self.clamp(img_x, 0, image_width)
        img_y = self.clamp(img_y, 0, image_height)

        return int(img_x), int(img_y)

    def project_to_image_plane(self, light):
        """Project point from 3D world coordinates to 2D camera image location
        Args:
            point_in_world (Point): 3D location of a point in the world
        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image
        """
        fx = self.config['camera_info']['focal_length_x']

        # get transform between pose of camera and world frame
        # trans = None
        # rot = None
        base_light = None

        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  light.header.frame_id, now, rospy.Duration(1.0))
            base_light = self.listener.transformPose("base_link", light.pose)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")
            return None

        # Find bounding box of traffic light in image
        if base_light is not None:
            # Simulator uses FOV
            if fx < 100:
                # x, y = self.project_with_fov(base_light)
                d = base_light.pose.position.x
                x = base_light.pose.position.y + 0.5
                y = base_light.pose.position.z - 1.75

                ux, uy = self.project_with_fov(d, x + 0.5*LIGHT_WIDTH, y + 0.5*LIGHT_HEIGHT)
                lx, ly = self.project_with_fov(d, x - 0.5*LIGHT_WIDTH, y - 0.5*LIGHT_HEIGHT)

                return ux, uy, lx, ly

            # Real car uses focal length
            else:
                rospy.loginfo('Real car detected...  Process image using focal length!')

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
