# ground_segmentation

ROS2 Composable Node for Ground Segmentation. 
Subscribes to a PointCloud2 topic and publishes a PointCloud2 Topic.

How to launch:
```
ros2 launch ros2_ground_segmentation ground_segmentation.launch.xml \
    input/topic:=/lidar/pointcloud \
    output/topic:=/processed/pointcloud
```

## Parameters
| Parameter       | Default         | Description              |
|-----------------|-----------------|--------------------------|
| `input/topic`   | `/input_topic`  | ROS2 Topic to subscribe. |
| `output/topic`  | `/output_topic` | ROS2 Topic to publish.   |

