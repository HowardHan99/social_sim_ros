#!/usr/bin/env python3

import rospy
from social_sim_ros.msg import ObstacleArray

def obstacle_callback(msg):
    """
    Callback function to process received ObstacleArray messages.
    """
    rospy.loginfo("--- Obstacle Array Received ---")
    rospy.loginfo("Timestamp: {}".format(msg.header.stamp))
    rospy.loginfo("Frame ID: {}".format(msg.header.frame_id))
    rospy.loginfo("Number of Obstacles: {}".format(len(msg.obstacles)))

    if not msg.obstacles:
        rospy.loginfo("Obstacle array is empty.")
        return

    for i, obstacle in enumerate(msg.obstacles):
        rospy.loginfo("\n--- Obstacle #{} ---".format(i + 1))
        rospy.loginfo("  ID: {}".format(obstacle.id))
        rospy.loginfo("  Type: {}".format(obstacle.type))
        rospy.loginfo("  Position (x, y, z): ({:.2f}, {:.2f}, {:.2f})".format(
            obstacle.pose.position.x,
            obstacle.pose.position.y,
            obstacle.pose.position.z
        ))
        rospy.loginfo("  Orientation (x, y, z, w): ({:.2f}, {:.2f}, {:.2f}, {:.2f})".format(
            obstacle.pose.orientation.x,
            obstacle.pose.orientation.y,
            obstacle.pose.orientation.z,
            obstacle.pose.orientation.w
        ))
        rospy.loginfo("  Scale (x, y, z): ({:.2f}, {:.2f}, {:.2f})".format(
            obstacle.scale.x,
            obstacle.scale.y,
            obstacle.scale.z
        ))
    rospy.loginfo("--- End of Obstacle Array ---\n")

def listener():
    """
    Initializes the ROS node and subscriber.
    """
    rospy.init_node('obstacle_listener', anonymous=True)
    rospy.loginfo("Obstacle listener node started, subscribing to /social_sim/obstacles")
    rospy.Subscriber('/social_sim/obstacles', ObstacleArray, obstacle_callback)
    
    # rospy.spin() keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
