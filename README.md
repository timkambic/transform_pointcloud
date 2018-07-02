# transform_pointcloud #
ROS nodelet that transforms pointcloud (`sensor_msgs/Pointcloud` or `sensor_msgs/Pointcloud`) to some other 
TF frame (e.g. from "camera_frame" to "base_link"). It uses `sensor_msgs/point_cloud_conversion.h` for conversion.
It will publish both `Pointcloud` and `Pointcloud2` messages for any input type (if any node is subscribed to it)
# Subscribed topics
* `~input_pcl (sensor_msgs/Pointcloud)` input topic
* `~inptu_pcl2 (sensor_msgs/Pointcloud2)` input topic

# Published topics
* `~output_pcl (sensor_msgs/Pointcloud)` output topic
* `~output_pcl2 (sensor_msgs/Pointcloud2)` output topic

# Parameters
* `to_frame (string, default:base_link)` to what TF frame to transform the output

# TF
* required is the transform from frame that input pointcloud is to frame set by `to_frame` parameter

# Usage
See `transform_pointcloud/launch/transform.launch` for example
