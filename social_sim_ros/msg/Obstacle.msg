# Message defining a single obstacle in the environment

int32                  id          # Unique identifier for this obstacle
string                 type        # Obstacle category (box, cylinder, wall, furniture, etc.)
geometry_msgs/Pose     pose        # Position and orientation in map frame
geometry_msgs/Vector3  scale       # Dimensions of the obstacle (x, y, z)

