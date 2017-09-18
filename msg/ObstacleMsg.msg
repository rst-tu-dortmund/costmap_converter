# Special types:
# Polygon with 1 vertex: Point obstacle
# Polygon with 2 vertices: Line obstacle
# Polygon with more than 2 vertices: First and last points are assumed to be connected

std_msgs/Header header

# Obstacle footprint (polygon descriptions)
geometry_msgs/Polygon polygon

# Obstacle ID
# Specify IDs in order to provide (temporal) relationships
# between obstacles among multiple messages.
int64 id

# Individual orientation (centroid)
geometry_msgs/Quaternion orientation

# Individual velocities (centroid)
geometry_msgs/TwistWithCovariance velocities

