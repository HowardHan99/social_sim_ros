#!/usr/bin/env python3

import rospy
from social_sim_ros.msg import ObstacleArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
import random

# A dictionary to cache random colors for each obstacle type
TYPE_COLORS = {}

def get_color_for_type(obstacle_type):
    """
    Returns a consistent, randomly generated color for a given obstacle type.
    """
    if obstacle_type not in TYPE_COLORS:
        TYPE_COLORS[obstacle_type] = ColorRGBA(
            r=random.uniform(0.2, 1.0),
            g=random.uniform(0.2, 1.0),
            b=random.uniform(0.2, 1.0),
            a=0.8  # Semi-transparent
        )
    return TYPE_COLORS[obstacle_type]

def obstacle_callback(msg, marker_publisher):
    """
    Callback function to process ObstacleArray messages and publish MarkerArray.
    """
    marker_array = MarkerArray()
    
    # Create a DELETEALL marker to clear old markers. This handles cases where
    # obstacles are removed from the simulation. A more optimized approach would
    # be to track IDs, but this is simpler and effective.
    delete_all_marker = Marker()
    delete_all_marker.action = Marker.DELETEALL
    marker_array.markers.append(delete_all_marker)

    obstacles_by_id = {}
    for obstacle in msg.obstacles:
        if obstacle.id in obstacles_by_id:
            rospy.logwarn_throttle(5.0, "Duplicate obstacle id %d received; visualizer keeps latest.", obstacle.id)
        obstacles_by_id[obstacle.id] = obstacle

    for obstacle in obstacles_by_id.values():
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = rospy.Time.now()
        
        # Namespace and ID help RViz differentiate markers
        marker.ns = "obstacles"
        marker.id = obstacle.id
        
        # Set the marker type based on the obstacle's type string
        obstacle_type_lower = obstacle.type.lower()
        if "box" in obstacle_type_lower or "wall" in obstacle_type_lower or "furniture" in obstacle_type_lower:
            marker.type = Marker.CUBE
        elif "cylinder" in obstacle_type_lower:
            marker.type = Marker.CYLINDER
        else: # Default to a sphere if type is unknown
            marker.type = Marker.SPHERE
            
        # Add marker to be published
        marker.action = Marker.ADD
        
        # Copy pose and scale from the obstacle message.
        # Enforce a minimum scale so obstacles with a zero dimension (e.g. flat
        # objects with no height) are still visible in RViz instead of invisible.
        marker.pose = obstacle.pose
        MIN_SCALE = 0.05
        marker.scale = Vector3(
            x=max(obstacle.scale.x, MIN_SCALE),
            y=max(obstacle.scale.y, MIN_SCALE),
            z=max(obstacle.scale.z, MIN_SCALE),
        )
        
        # Assign a color based on the obstacle type for better visualization
        marker.color = get_color_for_type(obstacle.type)
        
        # Keep marker until explicitly replaced/deleted by next snapshot.
        # This avoids flicker when Unity publishes at a low rate.
        marker.lifetime = rospy.Duration(0.0)
        
        marker_array.markers.append(marker)
        
    # Publish the complete array of markers
    marker_publisher.publish(marker_array)

def visualizer():
    """
    Initializes the ROS node, subscriber, and publisher.
    """
    rospy.init_node('obstacle_visualizer', anonymous=True)
    
    # Create a publisher for the MarkerArray topic
    marker_pub = rospy.Publisher('/obstacle_markers', MarkerArray, queue_size=10)
    
    # Create a subscriber to the ObstacleArray topic
    rospy.Subscriber('/social_sim/obstacles', ObstacleArray, obstacle_callback, marker_pub)
    
    rospy.loginfo("Obstacle visualizer node started.")
    rospy.loginfo("Subscribing to /social_sim/obstacles")
    rospy.loginfo("Publishing to /obstacle_markers")
    
    rospy.spin()

if __name__ == '__main__':
    try:
        visualizer()
    except rospy.ROSInterruptException:
        pass
