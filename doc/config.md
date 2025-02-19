## Node Parameters
### Perception Node
_Defaults: [config/perception.yaml](../config/perception.yaml)_

* `map_frame_id` (String) : The global frame id -- _default: "map"_
* `odom_frame_id` (String) : The odometry frame id -- _default: "odom"_
* `base_frame_id` (String) : The robot frame id -- _default: "base_link"_

* `scan_topic` (String) : Topic for the lidar scan data -- _default: "/lance/lidar_scan"_
* `imu_topic` (String) : Topic for the IMU data -- _default: "/lance/imu"_

* `use_tag_detections` (Integer) : Tag detections enable/disable:
  | Option | Description              |
  |--------|--------------------------|
  | `1`    | Enable tag detections    |
  | `0`    | Disable tag detections   |
  | `-1`   | Filtered tag detections  |

  -- _default: 1_

* `require_rebias_before_tf_pub` (Boolean) : Wait until successful trajectory filter rebias before publishing map-to-odom transform -- _default: false_
* `require_rebias_before_scan_pub` (Boolean) : Wait until successful trajectory filter rebias before republishing scans -- _default: false_

* `metrics_pub_freq` (Float) : Frequency for printing statistics -- _default: 10.0_
* `publish_odometry_debug` (Boolean) : Enable publishing of odometry debug information -- _default: true_

* `cropbox_filter` - Points within this bounding box are excluded from processing (in base_frame coordinates):
  * `min` (Float array) : Minimum corner coordinates -- _default: `[-0.405, -0.6, 0.0]`_
  * `max` (Float array) : Maximum corner coordinates -- _default: `[0.735, 0.6, 0.959]`_

* `trajectory_filter` (Object) : Configuration for trajectory filtering:
  * `sampling_window_s` (Float) : Time window in seconds for filtering -- _default: 0.6_
  * `min_filter_window_s` (Float) : Minimum time window in seconds for filtering -- _default: 0.4_
  * `thresh` (Object) : Thresholds for filtering:
    * `avg_linear_error` (Float) : Average linear error threshold -- _default: 0.13_
    * `avg_angular_error` (Float) : Average angular error threshold -- _default: 0.09_
    * `max_linear_deviation` (Float) : Maximum linear deviation threshold -- _default: 0.04_
    * `max_angular_deviation` (Float) : Maximum angular deviation threshold -- _default: 0.04_

* `dlo` (Object) : DLO-related settings:
  * `use_timestamps_as_init` (Boolean) : Use timestamps for initial alignment -- _default: true_
  * `gravity_align` (Boolean) : Enable/disable gravity alignment on startup -- _default: false_
  * `adaptive_params` (Object) : Adaptive parameters:
    * `use` (Boolean) : Whether or not keyframe parameters scale with spaciousness -- _default: true_
    * `lpf_coeff` (Float) : Low-pass filter coefficient -- _default: 0.8_
  * `keyframe` (Object) : Keyframe parameters:
    * `thresh_D` (Float) : Distance threshold (in meters) for creating a new keyframe -- _default: 1.0_
    * `thresh_R` (Float) : Rotation threshold (in degrees) for creating a new keyframe -- _default: 30.0_
    * `submap` (Object) : Submap construction parameters:
      * `knn` (Integer) : Number of nearest-neighbor poses to extract when building a submap -- _default: 10_
      * `kcv` (Integer) : Convex hull parameter -- _default: 10_
      * `kcc` (Integer) : Concave hull parameter -- _default: 10_
  * `initial_pose` (Object) : Initial pose configuration:
    * `use` (Boolean) : Start with the provided pose? -- _default: true_
    * `position` (Array) : Initial position: `[0.0, 0.0, 0.0]`
    * `orientation` (Array) : Initial orientation (quaternion): `[1.0, 0.0, 0.0, 0.0]`
  * `voxel_filter` (Object) : Voxel filter parameters:
    * `scan` (Object) : Voxelize each input scan:
      * `use` (Boolean) : Enable voxelization for scans? -- _default: true_
      * `res` (Float) : Voxel leaf size -- _default: 0.04_
    * `submap` (Object) : Voxelize each submap:
      * `use` (Boolean) : Enable voxelization for submaps? -- _default: true_
      * `res` (Float) : Voxel leaf size -- _default: 0.1_
    * `adaptive_leaf_size` (Object) : Adaptive voxel size:
      * `range_coeff` (Float) : Range coefficient for voxel adaptation -- _default: 0.063_
      * `stddev_coeff` (Float) : Standard deviation coefficient for voxel adaptation -- _default: 0.064_
      * `offset` (Float) : Offset for voxel adaptation -- _default: -0.29_
      * `floor` (Float) : Floor value for voxel adaptation -- _default: 0.04_
      * `ceil` (Float) : Ceiling value for voxel adaptation -- _default: 0.5_
      * `precision` (Float) : Precision for voxel adaptation -- _default: 0.01_
  * `immediate_filter` (Object) : Immediate filter configuration:
    * `use` (Boolean) : Enable/disable immediate filter -- _default: false_
    * `range` (Float) : Range for immediate filter -- _default: 1.0_
    * `thresh_proportion` (Float) : Threshold proportion for immediate filter -- _default: 0.3_
  * `imu` (Object) : IMU-related settings:
    * `use` (Boolean) : Integrate IMU data to hint GICP? -- _default: true_
    * `use_orientation` (Boolean) : Use orientation data from IMU? -- _default: true_
    * `calib_time` (Float) : IMU calibration time in seconds -- _default: 3.0_
  * `gicp` (Object) : GICP-related settings:
    * `num_threads` (Integer) : Number of threads for GICP computation -- _default: 4_
    * `min_num_points` (Integer) : Minimum number of points required for GICP -- _default: 500_
    * `s2s` (Object) : Point-to-point matching parameters:
      * `k_correspondences` (Integer) : Number of correspondences -- _default: 15_
      * `max_correspondence_distance` (Float) : Maximum correspondence distance -- _default: 0.25_
      * `max_iterations` (Integer) : Maximum iterations -- _default: 32_
      * `transformation_epsilon` (Float) : Transformation epsilon -- _default: 0.01_
      * `euclidean_fitness_epsilon` (Float) : Euclidean fitness epsilon -- _default: 0.01_
      * `ransac` (Object) : RANSAC parameters:
        * `iterations` (Integer) : RANSAC iterations -- _default: 5_
        * `outlier_rejection_thresh` (Float) : RANSAC outlier rejection threshold -- _default: 0.1_
    * `s2m` (Object) : Point-to-map matching parameters:
      * `k_correspondences` (Integer) : Number of correspondences -- _default: 30_
      * `max_correspondence_distance` (Float) : Maximum correspondence distance -- _default: 0.1_
      * `max_iterations` (Integer) : Maximum iterations -- _default: 32_
      * `transformation_epsilon` (Float) : Transformation epsilon -- _default: 0.0009_
      * `euclidean_fitness_epsilon` (Float) : Euclidean fitness epsilon -- _default: 0.005_
      * `ransac` (Object) : RANSAC parameters:
        * `iterations` (Integer) : RANSAC iterations -- _default: 6_
        * `outlier_rejection_thresh` (Float) : RANSAC outlier rejection threshold -- _default: 0.05_

* `mapping` (Object) : Mapping-related settings:
  * `frustum_search_radius` (Float) : Frustum search radius -- _default: 0.009_
  * `radial_distance_thresh` (Float) : Radial distance threshold -- _default: 0.01_
  * `delete_delta_coeff` (Float) : Coefficient for deleting map points -- _default: 0.03_
  * `delete_max_range` (Float) : Maximum range for deleting map points -- _default: 4.0_
  * `add_max_range` (Float) : Maximum range for adding map points -- _default: 4.0_
  * `voxel_size` (Float) : Voxel size for map processing -- _default: 0.08_

* `fiducial_detection` (Object) : Fiducial detection settings:
  * `max_range` (Float) : Maximum range for fiducial detection -- _default: 2.0_
  * `plane_distance_threshold` (Float) : Distance threshold for plane detection -- _default: 0.005_
  * `plane_eps_thresh` (Float) : Epsilon threshold for plane detection -- _default: 0.1_
  * `vox_resolution` (Float) : Voxel resolution for fiducial detection -- _default: 0.03_
  * `remaining_points_thresh` (Float) : Remaining points threshold for fiducial detection -- _default: 0.05_
  * `minimum_input_points` (Integer) : Minimum number of input points for fiducial detection -- _default: 100_
  * `minimum_segmented_points` (Integer) : Minimum number of segmented points for fiducial detection -- _default: 15_

* `traversibility` (Object) : Traversibility analysis parameters:
  * `chunk_horizontal_range` (Float) : Horizontal range for traversibility analysis -- _default: 3.5_
  * `chunk_vertical_range` (Float) : Vertical range for traversibility analysis -- _default: 1.0_
