# ground_segmentation

ROS2 Composable Node for Ground Segmentation. 
Subscribes to a PointCloud2 topic and publishes a PointCloud2 Topic.

How to launch:
```
ros2 launch ros2_ground_segmentation ground_segmentation.launch.xml \
    input/topic:=/lidar/pointcloud \
    output/topic:=/processed/pointcloud
```

LidarTag:
```
ros2 launch ros2_ground_segmentation ray_ground_segmentation.launch.xml \
     output_frame:=map \
     input/topic:=/cloud_pcd  \
     max_point_distance:=15.0  \
     clipping_height:=1.0  \
     sensor_height:=1.3  \
     pointcloud_min_z:=-1.5  \
     max_point_distance:=15.0  \
     radial_divider_angle:=0.01  \
     general_max_slope:=5.0  \
     local_max_slope:=5.0  \
     min_outlier_filter_neighbors:=50  \
     min_outlier_filter_radius:=0.2  \
     outlier_filter:=True
```

## Parameters
| Parameter       | Default         | Description              |
|-----------------|-----------------|--------------------------|
| `input/topic`   | `/input_topic`  | ROS2 Topic to subscribe. |
| `output/topic`  | `/output_topic` | ROS2 Topic to publish.   |

