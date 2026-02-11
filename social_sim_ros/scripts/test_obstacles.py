#!/usr/bin/env python3
"""
Simple test script to verify obstacles are being received from Unity
"""
import rospy
from social_sim_ros.msg import ObstacleArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

obstacle_count = 0
point_count = 0

def obstacle_callback(msg):
    global obstacle_count
    obstacle_count = len(msg.obstacles)
    if obstacle_count > 0:
        rospy.loginfo("=" * 60)
        rospy.loginfo("Received %d obstacles from Unity:", obstacle_count)
        for i, obs in enumerate(msg.obstacles):
            rospy.loginfo("  Obstacle %d:", i)
            rospy.loginfo("    ID: %d", obs.id)
            rospy.loginfo("    Type: %s", obs.type)
            rospy.loginfo("    Position: (%.2f, %.2f, %.2f)", 
                         obs.pose.position.x, obs.pose.position.y, obs.pose.position.z)
            rospy.loginfo("    Scale: (%.2f, %.2f, %.2f)", 
                         obs.scale.x, obs.scale.y, obs.scale.z)

def pointcloud_callback(msg):
    global point_count
    # Count points in the cloud
    point_count = msg.width * msg.height
    if point_count > 0:
        rospy.loginfo("PointCloud received: %d points", point_count)
        # Sample first few points
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if len(points) > 0:
            rospy.loginfo("  First point: (%.2f, %.2f, %.2f)", points[0][0], points[0][1], points[0][2])
            if len(points) > 1:
                rospy.loginfo("  Last point: (%.2f, %.2f, %.2f)", points[-1][0], points[-1][1], points[-1][2])

if __name__ == '__main__':
    rospy.init_node('test_obstacles')
    
    rospy.loginfo("Starting obstacle test script...")
    rospy.loginfo("This will monitor obstacles from Unity and the converted point cloud")
    rospy.loginfo("")
    
    # Subscribe to both topics
    rospy.Subscriber('/social_sim/obstacles', ObstacleArray, obstacle_callback)
    rospy.Subscriber('/obstacle_cloud', PointCloud2, pointcloud_callback)
    
    rate = rospy.Rate(1.0)  # 1 Hz status updates
    
    while not rospy.is_shutdown():
        rospy.loginfo_throttle(5.0, "Status: %d obstacles, %d points in cloud", 
                              obstacle_count, point_count)
        rate.sleep()

