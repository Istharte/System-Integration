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

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	#rospy.Subscriber('/traffic_waypoint', Lane, self.traffic_cb)
	#rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
	self.current_x = 0
	self.current_y = 0
	self.current_z = 0
	
	self.near_detected = False
	self.near_id = 0

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
	self.current_x = msg.pose.position.x
	self.current_y = msg.pose.position.y
	self.current_z = msg.pose.position.z

    def waypoints_cb(self, waypoints):
        # TODO: Implement
	# Search near the current position.
	dist_min = 1000
	final_waypoints = Lane()
	final_wps = []
	near_id = self.near_id

	if self.near_detected: 
	    for i in range(self.near_id, len(waypoints.waypoints)):
		dist = math.sqrt((self.current_x - waypoints.waypoints[i].pose.pose.position.x)**2 * \
		       (self.current_y - waypoints.waypoints[i].pose.pose.position.y)**2 + \
		       (self.current_z - waypoints.waypoints[i].pose.pose.position.z)**2)
		if dist < dist_min:
		    dist_min = dist
		    near_id = i
		    break
	    self.near_id = near_id
	    self.near_detected = True
	else:
	    for i in range(len(waypoints.waypoints)):
		dist = math.sqrt((self.current_x - waypoints.waypoints[i].pose.pose.position.x)**2 * \
                       (self.current_y - waypoints.waypoints[i].pose.pose.position.y)**2 + \
                       (self.current_z - waypoints.waypoints[i].pose.pose.position.z)**2)
                if dist < dist_min:
                    dist_min = dist
                    near_id = i
	    self.near_id = near_id

	for i in range(self.near_id, self.near_id+LOOKAHEAD_WPS):
	    final_wps.append(waypoints.waypoints[i])

	# Publish final_waypoints.
	final_waypoints.waypoints = final_wps
	final_waypoints.header = waypoints.header
	self.final_waypoints_pub.publish(final_waypoints)

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
