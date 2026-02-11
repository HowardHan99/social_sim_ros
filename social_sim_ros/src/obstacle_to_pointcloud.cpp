#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <social_sim_ros/ObstacleArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

ros::Publisher cloud_pub;

// Convert a box obstacle to a point cloud
void addBoxToCloud(const geometry_msgs::Pose& pose,
                   const geometry_msgs::Vector3& scale,
                   std::vector<float>& x_data,
                   std::vector<float>& y_data,
                   std::vector<float>& z_data)
{
    // Extract position
    double cx = pose.position.x;
    double cy = pose.position.y;
    double cz = pose.position.z;
    
    // Extract yaw from quaternion (we only care about 2D rotation for now)
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, 
                      pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    // Half dimensions
    double hx = scale.x / 2.0;
    double hy = scale.y / 2.0;
    
    // Sample points along the perimeter and fill
    // Using a dense grid to ensure obstacle is detected
    double resolution = 0.05; // 5cm resolution - matches costmap
    
    // Sample the footprint densely
    for (double dx = -hx; dx <= hx; dx += resolution) {
        for (double dy = -hy; dy <= hy; dy += resolution) {
            // Rotate point by yaw
            double rx = cx + dx * cos(yaw) - dy * sin(yaw);
            double ry = cy + dx * sin(yaw) + dy * cos(yaw);
            
            x_data.push_back(rx);
            y_data.push_back(ry);
            z_data.push_back(cz); // Keep z for 3D obstacles
        }
    }
}

// Convert a cylinder obstacle to a point cloud
void addCylinderToCloud(const geometry_msgs::Pose& pose,
                        const geometry_msgs::Vector3& scale,
                        std::vector<float>& x_data,
                        std::vector<float>& y_data,
                        std::vector<float>& z_data)
{
    double cx = pose.position.x;
    double cy = pose.position.y;
    double cz = pose.position.z;
    
    // For cylinder, scale.x is typically the radius
    double radius = scale.x / 2.0;
    double resolution = 0.05; // 5cm resolution
    
    // Sample a filled circle
    for (double dx = -radius; dx <= radius; dx += resolution) {
        for (double dy = -radius; dy <= radius; dy += resolution) {
            // Check if point is inside circle
            if (dx*dx + dy*dy <= radius*radius) {
                x_data.push_back(cx + dx);
                y_data.push_back(cy + dy);
                z_data.push_back(cz);
            }
        }
    }
}

void obstacleCallback(const social_sim_ros::ObstacleArray::ConstPtr& msg)
{
    ROS_INFO("Received %lu obstacles", msg->obstacles.size());
    
    if (msg->obstacles.empty()) {
        // Publish empty cloud if no obstacles
        sensor_msgs::PointCloud2 cloud;
        cloud.header = msg->header;
        cloud.height = 1;
        cloud.width = 0;
        cloud_pub.publish(cloud);
        ROS_INFO("Published empty cloud (no obstacles)");
        return;
    }
    
    // Temporary storage for points
    std::vector<float> x_data, y_data, z_data;
    
    // Convert each obstacle to points
    for (const auto& obstacle : msg->obstacles) {
        std::string type = obstacle.type;
        
        // Convert to lowercase for comparison
        std::transform(type.begin(), type.end(), type.begin(), ::tolower);
        
        if (type.find("box") != std::string::npos || 
            type.find("wall") != std::string::npos ||
            type.find("furniture") != std::string::npos ||
            type.empty()) {
            // Treat as box
            addBoxToCloud(obstacle.pose, obstacle.scale,
                         x_data, y_data, z_data);
        }
        else if (type.find("cylinder") != std::string::npos ||
                 type.find("circle") != std::string::npos) {
            // Treat as cylinder
            addCylinderToCloud(obstacle.pose, obstacle.scale,
                              x_data, y_data, z_data);
        }
        else {
            // Default: treat as box
            addBoxToCloud(obstacle.pose, obstacle.scale,
                         x_data, y_data, z_data);
        }
    }
    
    // Create PointCloud2 message
    sensor_msgs::PointCloud2 cloud;
    cloud.header = msg->header;
    cloud.height = 1;
    cloud.width = x_data.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    
    // Define fields (x, y, z)
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(x_data.size());
    
    // Fill in the data
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    
    for (size_t i = 0; i < x_data.size(); ++i) {
        *iter_x = x_data[i];
        *iter_y = y_data[i];
        *iter_z = z_data[i];
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
    
    ROS_INFO("Publishing obstacle cloud with %lu points from %lu obstacles", 
             x_data.size(), msg->obstacles.size());
    
    cloud_pub.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_to_pointcloud");
    ros::NodeHandle nh;
    
    // Subscribe to obstacles from Unity
    ros::Subscriber obstacle_sub = nh.subscribe("/social_sim/obstacles", 10, obstacleCallback);
    
    // Publish point cloud for costmap
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_cloud", 10);
    
    ROS_INFO("Obstacle to PointCloud converter started");
    ROS_INFO("  Subscribing to: /social_sim/obstacles");
    ROS_INFO("  Publishing to: /obstacle_cloud");
    
    ros::spin();
    return 0;
}

