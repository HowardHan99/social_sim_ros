#!/usr/bin/env python3

import rospy
from social_sim_ros.msg import ObstacleArray
from visualization_msgs.msg import Marker, MarkerArray
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

    for obstacle in msg.obstacles:
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
        
        # Copy pose and scale directly from the obstacle message
        marker.pose = obstacle.pose
        marker.scale = obstacle.scale
        
        # Assign a color based on the obstacle type for better visualization
        marker.color = get_color_for_type(obstacle.type)
        
        # Markers will auto-delete if a new one isn't published after this duration
        marker.lifetime = rospy.Duration(2.0) # Slightly longer than the publish interval
        
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
