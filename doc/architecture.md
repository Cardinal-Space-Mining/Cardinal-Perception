## Cardinal Perception System Architecture
### Localization
The localization solution consists of two components: LiDAR odometry and fiducial detection for global alignment. The LiDAR odometry is based off of [direct_lidar_odometry](https://github.com/vectr-ucla/direct_lidar_odometry) (DLO) and utilizes a scan-to-scan and scan-to-map based odometry solution, with optional IMU input to seed registration. Fiducial detection is not required but can come in one of two forms - AprilTag detections or LiDAR-reflector detection (this is custom). Fusion between local and global measurements is currently only done on a simple transform-offset basis, utilizing the `map` and `odom` transform-tree frames (meaning that without fiducial detection, odometry is implicitly treated as the full localization solution).

![localization overview](localization-v050.svg)

**Localization subsribes to the following ROS2 topics:**
1. A `sensor_msgs/msg/PointCloud2` topic as configured by the parameter `scan_topic` (default is `scan`). This can be any pointcloud and is not restricted to being organized or needing have timestamp data, etc..
2. A `sensor_msgs/msg/Imu` topic as configured by the parameter `imu_topic` (default is `imu`). *This is optional and IMU usage can be disabled by setting the `dlo.imu.use` parameter to `false`.* IMU data must be in the same time-base as LiDAR scans, since these are timestamp matched internally.
3. A `cardinal_perception/msg/TagsTransform` topic which defaults to `"/tags_detections"` in both nodes. This is only relevant when using the AprilTag detector node to provide global measurements, which can be parameter-disabled by setting `use_tag_detections` to `false` or compile-time disabled by using setting the `USE_TAG_DETECTION_PIPELINE` macro to `0`.

### Terrain Mapping
Terrain mapping attempts to model the real world utilizing a set of cartesian points. The map itself is stored as a pointcloud, and indexed using a specialized octree which automatically voxelizes all points in the same leaf and allows for direct deletion of points/leaves. Point-deletion is calculating using the custom-developed "KDTree Frustum Collision" (KFC) mapping model, which provides (more or less) realtime mapping using just LiDAR scans. This stage does not have any additonal ROS subscriptions but works directly off of buffers which are reused from the localization stage, as well as the most recent localization results.

![mapping overview](mapping-v050.svg)

#### KFC Map Iteration Steps
1. Assemble submap from global map using distance from current LiDAR scanner position.
2. Calculate the direction to each submap point from the LiDAR position, and normalize to be of unit length.
3. Build a KDTree from the global submap direction vectors.
4. Calculate the direction to each scan point from the LiDAR position, and normalize to be of unit length. For each point, complete a range search in the submap KDTree to find points within the "frustum cone" for the current LiDAR ray.
5. Apply the collision model to determine which points should be deleted (points that are closer than the scan point get deleted).
6. Perform delete operation on global map.
7. Add scan points to global map.
8. Optimize the underlying octree structure.

### Traversability Generation
(Not implemented yet!)

### Trajectory Generation
(Not implemented yet!)

__*Last updated: 2/19/25*__
