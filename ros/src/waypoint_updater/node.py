#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

from waypoint_updater.srv import *
import math

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
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)
        rospy.Service('~next_waypoint', NextWaypoint, self.next_waypoint_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.traffic_waypoint = -1
        self.obstacle_waypoint = -1
        self.current_pose = None
        self.base_waypoints = None

        self.loop()

    def loop(self):
        # TODO: adjust rate
        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            if self.current_pose is None or self.base_waypoints is None:
                continue

            self.publish()
            rate.sleep()

    def publish(self):
        """publish Lane message to /final_waypoints topic"""

        next_waypoint = self.next_waypoint(self.base_waypoints.waypoints, self.current_pose)
        waypoints = self.base_waypoints.waypoints
        # shift waypoint indexes to start on next_waypoint so it's easy to grab LOOKAHEAD_WPS
        waypoints = waypoints[next_waypoint:] + waypoints[:next_waypoint]
        waypoints = waypoints[:LOOKAHEAD_WPS]

        lane = Lane()
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.current_pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints

    def traffic_cb(self, msg):
        self.traffic_waypoint = msg

    def obstacle_cb(self, msg):
        self.obstacle_waypoint = msg

    def next_waypoint_cb(self, msg):
        """callback for ~next_waypoint service
        Identifies the closest path waypoint to the given position

        Args:
            NextWaypointRequest
                waypoints (Lane): position to match a waypoint to
                pose (Pose): position to match a waypoint to

        Returns:
            NextWaypointResponse
                Int32: index of the closest waypoint in waypoints
        """

        closest_idx = self.next_waypoint(msg.waypoints, msg.pose)
        return NextWaypointResponse(Int32(closest_idx))

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2+1):
            dist += self.distance_p1_p2(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance_p1_p2(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

    def closest_waypoint(self, waypoints, pose):
        """ get index of closest waypoint to car

        see https://github.com/udacity/CarND-Path-Planning-Project/blob/59a4ffc9b56f896479a7e498087ab23f6db3f100/src/main.cpp#L41-L62

        Returns:
            int: the index within base_waypoints
        """

        closest_len = 100000; # large number
        closest_waypoint = 0;

        for idx, waypoint in enumerate(waypoints):
            dist = self.distance_p1_p2(pose.pose.position, waypoint.pose.pose.position)
            if dist < closest_len:
                closest_len = dist
                closest_waypoint = idx

        return closest_waypoint

    def next_waypoint(self, waypoints, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
            https://github.com/udacity/CarND-Path-Planning-Project/blob/59a4ffc9b56f896479a7e498087ab23f6db3f100/src/main.cpp#L64-L87

        Args:
            waypoints (Waypoint[]): position to match a waypoint to
            pose (Pose): position to match a waypoint to
            

        Returns:
            Int: index of the closest waypoint in waypoints
        """

        closest_idx = self.closest_waypoint(waypoints, pose)
        closest = waypoints[closest_idx]
        num_waypoints = len(waypoints)

        x = pose.pose.position.x
        y = pose.pose.position.y

        # TODO: is this the correct value for theta?
        rotation_to_radian = 6.28 # 1 rotation = 6.28 radians
        theta = pose.pose.orientation.z * rotation_to_radian

        map_x = closest.pose.pose.position.x
        map_y = closest.pose.pose.position.y

        heading = math.atan2((map_y - y), (map_x - x))
        theta_pos = math.fmod(theta + (2 * math.pi), 2 * math.pi)
        heading_pos = math.fmod(heading + (2 * math.pi), 2 * math.pi)
        angle = math.fabs(theta_pos - heading_pos)

        if angle > math.pi:
            angle = (2 * math.pi) - angle;

        if angle > math.pi / 2:
            closest_idx = (closest_idx + 1) % num_waypoints

        return closest_idx


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
