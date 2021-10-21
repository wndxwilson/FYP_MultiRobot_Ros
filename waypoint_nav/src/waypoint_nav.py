#!/usr/bin/env python3
# license removed for brevity

from numpy.lib.function_base import append
import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler

from visualization_msgs.msg import Marker

class WaypointNavigation():

    def __init__(self,waypoint_list):
        '''
        waypoint_list = [[x1,y1,yaw1],...,[xn,yn,yawn]]
        x : float
        y : float
        yaw : float 
        '''
        # Wait for action server
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()

        # On shutdown
        rospy.on_shutdown(self.shutdown)

        # Generate waypoints
        self.waypoints = self.__generate_waypoints(waypoint_list)

        # Publisher 
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

 
        # Init markers
        self.init_markers()

    def navigate(self):

        for i in range(len(self.waypoints)):
            if (rospy.is_shutdown()):
                break

            # Generate goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.waypoints[i]

            # Send goal
            self.client.send_goal(goal)

            # wait for result
            result = self.client.wait_for_result(rospy.Duration(60))

            if not result:
                self.client.cancel_goal()
                rospy.logwarn("Timed out ...")

            else:
                state = self.client.get_state()
                print(state)

    def __generate_waypoints(self,waypoint_list):
        waypoints = list()

        for waypoint in waypoint_list:
            orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, waypoint[2]))
            waypoints.append(Pose(Point(waypoint[0], waypoint[1], 0.0),orientation))
        
        return waypoints

    def init_markers(self):
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.points = list()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = Marker.SPHERE_LIST
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for waypoint in self.waypoints:           
            p = Point()
            p = waypoint.position
            marker.points.append(p)

        rospy.sleep(1)
        print(marker)
        self.marker_pub.publish(marker)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.client.cancel_goal()
        rospy.sleep(1)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        nav = WaypointNavigation([[-7.0,-6.0,0],[7.82,-6,0],[6.0,8.0,0],[-6.0,8.5,0]])
        nav.navigate()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
