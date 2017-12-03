#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight
from std_msgs.msg import Int32

import math
import tf
import copy

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level = rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        #rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.speed_limit = float(self.kmph2mps(rospy.get_param('waypoint_loader/velocity')))

        self.base_waypoints = None
        self.current_pose = None
        self.closest_wp = -1
        self.final_waypoints = []

        # Main loop
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            if(self.base_waypoints is None) or (self.current_pose is None):
                continue

            current_pose = self.current_pose
            base_waypoints = self.base_waypoints
            self.final_waypoints = []

            # Find closest waypoint in front
            id_waypoint_closest = self.get_closest_waypoint(base_waypoints, current_pose)

            # Construct waypoints for vehicle to follow
            for i in range(LOOKAHEAD_WPS):
                idx = (id_waypoint_closest + i) % len(base_waypoints)
                wp = copy.deepcopy(base_waypoints[idx])
                #wp.twist.twist.linear.x = self.speed_limit
                self.final_waypoints.append(wp)


            '''
            rospy.loginfo('1   Waypoint pose: x %d, y %d : Waypoint speed: %d',
                                    final_waypoints[1].pose.pose.position.x,
                                    final_waypoints[1].pose.pose.position.y,
                                    final_waypoints[1].twist.twist.linear.x)
            rospy.loginfo('100 Waypoint pose: x %d, y %d : Waypoint speed: %d',
                                    final_waypoints[100].pose.pose.position.x,
                                    final_waypoints[100].pose.pose.position.y,
                                    final_waypoints[100].twist.twist.linear.x)
            rospy.loginfo('199 Waypoint pose: x %d, y %d : Waypoint speed: %d',
                                    final_waypoints[199].pose.pose.position.x,
                                    final_waypoints[199].pose.pose.position.y,
                                    final_waypoints[199].twist.twist.linear.x)
                                    '''
            self.publish_final_waypoints()

        #rospy.spin()

    def get_closest_waypoint(self, waypoints, cur_pos):
        min_dist = 999999
        min_index = -1

        # Find waypoint closest to vehicle position
        for i in range(len(waypoints)):
            dist = self.get_dist(waypoints[i].pose.pose.position, cur_pos.position)
            if dist < min_dist:
                min_index = i
                min_dist = dist

        dist_x_waypoint = waypoints[min_index].pose.pose.position.x - cur_pos.position.x
        dist_y_waypoint = waypoints[min_index].pose.pose.position.y - cur_pos.position.y
        explicit_quaternion = [cur_pos.orientation.x,cur_pos.orientation.y,cur_pos.orientation.z,cur_pos.orientation.w]
        car_euler = tf.transformations.euler_from_quaternion(explicit_quaternion)
        car_yaw = car_euler[2]
        if car_yaw > math.pi:
            car_yaw -= 2 * math.pi

        angle_car_waypoint = math.atan2(dist_y_waypoint, dist_x_waypoint)

        if car_yaw * angle_car_waypoint < 0:
            min_index += 1
            if min_index >= len(waypoints):
                min_index = 0

        return min_index


    def publish_final_waypoints(self):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = self.final_waypoints
        rospy.loginfo('1   Waypoint pose: x %d, y %d : Waypoint speed: %d',
                                self.final_waypoints[1].pose.pose.position.x,
                                self.final_waypoints[1].pose.pose.position.y,
                                self.final_waypoints[1].twist.twist.linear.x)
        rospy.loginfo('100 Waypoint pose: x %d, y %d : Waypoint speed: %d',
                                self.final_waypoints[100].pose.pose.position.x,
                                self.final_waypoints[100].pose.pose.position.y,
                                self.final_waypoints[100].twist.twist.linear.x)
        rospy.loginfo('199 Waypoint pose: x %d, y %d : Waypoint speed: %d',
                                self.final_waypoints[199].pose.pose.position.x,
                                self.final_waypoints[199].pose.pose.position.y,
                                self.final_waypoints[199].twist.twist.linear.x)
        self.final_waypoints_pub.publish(lane)

    '''
    def velocity_cb(self, msg):
		self.current_velocity = msg.twist.linear.x
		pass
    '''

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg.pose
        rospy.loginfo('Current position : %d ', self.current_pose.position.x)
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints.waypoints
        rospy.loginfo('%s Base Waypoints Loaded', len(self.base_waypoints))
        rospy.loginfo('%d ', self.base_waypoints[0].pose.pose.position.x)
        pass


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
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

    def get_dist(self, pos1, pos2):
        return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2 + (pos1.z - pos2.z) ** 2)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
