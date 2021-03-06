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
	rospy.Subscriber('/traffic_waypoint', Lane, self.traffic_cb)
	rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
	self.current_x = 0
	self.current_y = 0
	self.current_z = 0
	
	self.near_detected = False
	self.near_id = 0
	self.tw_id = 0
	self.ow_id = 0
	# target velocity is 10mph. in our unit, m/sec
	self.target_velocity = 10 * 0.44704
	self.accel_limit = 4.0 #m/sec^2
	
	self.base_waypoints = Lane()
	self.final_waypoints = Lane()
	
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
	self.current_x = msg.pose.position.x
	self.current_y = msg.pose.position.y
	self.current_z = msg.pose.position.z

    def waypoints_cb(self, waypoints):
        # TODO: Implement
	# set base_waypoints
	self.base_waypoints = waypoints
	# Search near the current position.
	dist_min = 1000
	
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

	# set initial final_waypoints.
	self.final_waypoints.waypoints = final_wps
	self.final_waypoints.header = waypoints.header

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.tw_id = msg
	v0 = get_waypoint_velocity(self.base_waypoints.waypoints[self.near_id])

	if self.tw_id != -1:
	    # compute distance from current position to red tl.
	    d = self.distance(self.base_waypoints, self.near_id, self.tw_id)
	    
	    # solve the quadratic equation.
	    qe = lambda a,b c: (math.srqt(b*b + 4*a*c) - b) /2/a

	    if chk_stp(v0, d):	#check whether or not can stop.
	        for i, wp in enumerate(self.final_waypoints.waypoints):
		    # distance from near_id to the point where set the vehicle speed this time.
		    d_int = d / (self.tw_id - self.near_id - i)
		    decel = v0 * v0 /2/d
		    # Solve the quadratic equation to find the time required to travel
		    # a certain distance.
		    delta_t = qe(decel/2, v0, d_int)
		    set_v = max(v0 - decel * delta_t, 0)
		    set_waypoint_velocity(wp, i, set_v)
		
	else:
            # Accelerate to set speed.
	    for i, wp in enumerate(self.final_waypoints.waypoints):
	        
		d = self.distance(self.base_waypoints, self.near_id, self.near_id + i)
	        delta_t = qe(self.accel_limit, v0, d_int)
	        set_v = min(self.target_velocity, v0 + self.accel_limit * delta_t)
		set_waypoint_velocity(wp, i, set_v)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        self.ow_id = msg

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

    # utility.
    # Determine whether or not we can stop under the limit deceleration.
    def chk_stp(self, v_init, dist):
	stp_t = v_init / self.accel_limit
	d_stp = v_init * stp_t - self_accel_limit * stp_t * stp_t / 2
	
	if d_stp>dist:
	    return False
	else:
	    return True

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
