#!/usr/bin/env python3

import numpy as np
import rospy
from threading import Lock

from geometry_msgs.msg import PoseStamped
from autoware_mini.msg import Path, Waypoint

from shapely.geometry import LineString, Point

import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest, distance


class GlobalPlanner:
    def __init__(self):

        # Parameters
        lanelet2_map_path = rospy.get_param("~lanelet2_map_path")
        self.speed_limit = float(rospy.get_param("~speed_limit"))

        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        self.output_frame = rospy.get_param("lanelet2_global_planner/output_frame")
        self.distance_to_goal_limit = rospy.get_param("lanelet2_global_planner/distance_to_goal_limit")

        # Load Lanelet2 map
        if coordinate_transformer == "utm":
            projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            raise RuntimeError('Only "utm" is supported for lanelet2 map loading')
        self.lanelet2_map = load(lanelet2_map_path, projector)

        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                       lanelet2.traffic_rules.Participants.VehicleTaxi)
        self.graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules)

        # Internal variables
        self.lock = Lock()
        self.current_location = None
        self.goal_point = None

        # Publishers
        self.global_path_pub = rospy.Publisher('global_path', Path, latch=True, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

    def goal_callback(self, msg):
        with self.lock:
            self.goal_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        if self.current_location is None:
            return

        rospy.loginfo("%s - goal position (%f, %f, %f) in %s frame", rospy.get_name(),
                      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                      msg.header.frame_id)

        start_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.current_location, 1)[0][1]
        goal_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.goal_point, 1)[0][1]

        route = self.graph.getRoute(start_lanelet, goal_lanelet, 0, False)
        if route is None:
            rospy.logwarn("%s - No route found to goal position", rospy.get_name())
            return

        path = route.shortestPath()
        if path is None:
            rospy.logwarn("%s - No path found to goal position", rospy.get_name())
            return

        path_no_lane_change = path.getRemainingLane(start_lanelet)

        waypoints = self.convert_laneletseq_to_waypoints_list(path_no_lane_change)
        self.publish_lane_from_waypoints_list(waypoints)

    def current_pose_callback(self, msg):
        with self.lock:
            self.current_location = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        if self.goal_point is None:
            return

        d_to_goal = distance(self.current_location, self.goal_point)
        if d_to_goal < self.distance_to_goal_limit:
            self.publish_lane_from_waypoints_list([])
            rospy.loginfo("%s - goal position reached", rospy.get_name())
            self.goal_point = None

    def convert_laneletseq_to_waypoints_list(self, laneletseq):
        waypoints = []

        for j, lanelet in enumerate(laneletseq):
            # Get speed from lanelet attribute or use global speed limit (km/h), capped at speed limit, converted to m/s
            if 'speed_ref' in lanelet.attributes:
                speed = min(float(lanelet.attributes['speed_ref']), self.speed_limit) / 3.6
            else:
                speed = self.speed_limit / 3.6

            # Iterate through the centerline points and create waypoints.
            for i, point in enumerate(lanelet.centerline):
                # Skip first point of every lanelet except the very first (endpoints overlap)
                if i == 0 and j != 0:
                    continue
                waypoint = Waypoint()
                waypoint.position.x = point.x
                waypoint.position.y = point.y
                waypoint.position.z = point.z
                waypoint.speed = speed
                waypoints.append(waypoint)

        # Sync path end with goal point: project the goal onto the path, truncate
        # the waypoints past that point, and add an exact waypoint there.
        waypoints_xy = np.array([(w.position.x, w.position.y) for w in waypoints])
        path_linestring = LineString(waypoints_xy)
        d_to_goal = path_linestring.project(Point(self.goal_point.x, self.goal_point.y))
        goal_on_path = path_linestring.interpolate(d_to_goal)

        distances = np.cumsum(np.sqrt(np.sum(np.diff(waypoints_xy, axis=0)**2, axis=1)))
        distances = np.insert(distances, 0, 0)
        waypoints = [w for w, d in zip(waypoints, distances) if d < d_to_goal]

        end_waypoint = Waypoint()
        end_waypoint.position.x = goal_on_path.x
        end_waypoint.position.y = goal_on_path.y
        end_waypoint.position.z = waypoints[-1].position.z if waypoints else 0.0
        end_waypoint.speed = waypoints[-1].speed if waypoints else self.speed_limit / 3.6
        waypoints.append(end_waypoint)

        with self.lock:
            self.goal_point = BasicPoint2d(goal_on_path.x, goal_on_path.y)

        return waypoints

    def publish_lane_from_waypoints_list(self, waypoints):
        lane = Path()
        lane.header.frame_id = self.output_frame
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        self.global_path_pub.publish(lane)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('global_planner')
    node = GlobalPlanner()
    node.run()
