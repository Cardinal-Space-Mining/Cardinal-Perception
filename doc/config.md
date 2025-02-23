## Node Parameters
---
### Perception Node
_See file: [config/perception.yaml](../config/perception.yaml)_

* `map_frame_id` (String) : **The global frame id**
   - Default: `"map"`

* `odom_frame_id` (String) : **The odometry frame id**
   - Default: `"odom"`

* `base_frame_id` (String) : **The robot frame id**
   - Default: `"base_link"`

* `scan_topic` (String) : **Topic for the lidar scan data**
   - Default: `"/lance/lidar_scan"` (Specific to LANCE-1)

* `imu_topic` (String) : **Topic for the IMU data**
   - Default: `"/lance/imu"` (Specific to LANCE-1)

* `use_tag_detections` (Integer) : **Tag detection usage behavior**
   | Option | Description |
   |-|-|
   | `1`    | Use all available detections |
   | `0`    | Don't use detections |
   | `-1`   | Use fully filtered detections only |
   - Default: `1`

* `require_rebias_before_tf_pub` (Boolean) : **Wait until successful trajectory filter rebias before publishing map-to-odom transform**
   - Default: `false`

* `require_rebias_before_scan_pub` (Boolean) : **Wait until successful trajectory filter rebias before republishing scans**
   - Default: `false`

* `metrics_pub_freq` (Float) : **Frequency for printing statistics (hz)**
   - Default: `10.0`

* `publish_odometry_debug` (Boolean) : **Enable publishing of odometry debug information**
   - Default: `true`

* `cropbox_filter` : **Points within this bounding box are excluded from processing (relative to `base_frame_id`)**
   * `min` (Float Array) : **Minimum corner coordinates**
     - Default: `[-0.405, -0.6, 0.0]` (Specific to LANCE-1)
   * `max` (Float Array) : **Maximum corner coordinates**
     - Default: `[0.735, 0.6, 0.959]` (Specific to LANCE-1)

* `trajectory_filter` : **Trajectory filter configutation**
   * `sampling_window_s` (Float) : **Time window in seconds for filtering**
     - Default: `0.6`
   * `min_filter_window_s` (Float) : **Minimum time window in seconds for filtering**
     - Default: `0.4`
   * `thresh` : **Thresholds for filtering**
     * `avg_linear_error` (Float) : **Average linear error threshold**
       - Default: `0.13`
     * `avg_angular_error` (Float) : **Average angular error threshold**
       - Default: `0.09`
     * `max_linear_deviation` (Float) : **Maximum linear deviation threshold**
       - Default: `0.04`
     * `max_angular_deviation` (Float) : **Maximum angular deviation threshold**
       - Default: `0.04`

* `dlo` : **Odometry configuration - for advanced settings reference DLO**
   * `use_timestamps_as_init` (Boolean) : **Use timestamps for initial alignment**
     - Default: `true`
   * `gravity_align` (Boolean) : **Enable/disable gravity alignment on startup**
     - Default: `false`
   * `adaptive_params`
     * `use` (Boolean) : **Whether or not keyframe parameters scale with spaciousness**
       - Default: `true`
     * `lpf_coeff` (Float) : **Low-pass filter coefficient**
       - Default: `0.8`
   * `keyframe`
     * `thresh_D` (Float) : **Distance threshold (in meters) for creating a new keyframe**
       - Default: `1.0`
     * `thresh_R` (Float) : **Rotation threshold (in degrees) for creating a new keyframe**
       - Default: `30.0`
     * `submap` : **Submap construction parameters**
       * `knn` (Integer) : **Number of nearest-neighbor poses to extract when building a submap**
         - Default: `10`
       * `kcv` (Integer) : **Convex hull parameter**
         - Default: `10`
       * `kcc` (Integer) : **Concave hull parameter**
         - Default: `10`
   * `initial_pose`
     * `use` (Boolean) : **Start with the provided pose?**
       - Default: `true`
     * `position` (Float Array) : **Initial position**
       - Default: `[0.0, 0.0, 0.0]`
     * `orientation` (Float Array) : **Initial orientation (quaternion)**
       - Default: `[1.0, 0.0, 0.0, 0.0]`
   * `voxel_filter`
     * `scan` : **Voxelize each input scan**
       * `use` (Boolean) : **Enable voxelization for scans?**
         - Default: `true`
       * `res` (Float) : **Voxel leaf size**
         - Default: `0.04`
     * `submap` : **Voxelize each submap**
       * `use` (Boolean) : **Enable voxelization for submaps?**
         - Default: `true`
       * `res` (Float) : **Voxel leaf size**
         - Default: `0.1`
     * `adaptive_leaf_size`
       * `range_coeff` (Float) : **Range coefficient for voxel adaptation**
         - Default: `0.063`
       * `stddev_coeff` (Float) : **Standard deviation coefficient for voxel adaptation**
         - Default: `0.064`
       * `offset` (Float) : **Offset for voxel adaptation**
         - Default: `-0.29`
       * `floor` (Float) : **Floor value for voxel adaptation**
         - Default: `0.04`
       * `ceil` (Float) : **Ceiling value for voxel adaptation**
         - Default: `0.5`
       * `precision` (Float) : **Precision for voxel adaptation**
         - Default: `0.01`
   * `immediate_filter`
     * `use` (Boolean) : **Enable/disable immediate filter**
       - Default: `false`
     * `range` (Float) : **Range for immediate filter**
       - Default: `1.0`
     * `thresh_proportion` (Float) : **Threshold proportion for immediate filter**
       - Default: `0.3`
   * `imu`
     * `use` (Boolean) : **Integrate IMU data to hint GICP?**
       - Default: `true`
     * `use_orientation` (Boolean) : **Use orientation data from IMU?**
       - Default: `true`
     * `calib_time` (Float) : **IMU calibration time in seconds**
       - Default: `3.0`
   * `gicp`
     * `num_threads` (Integer) : **Number of threads for GICP computation**
       - Default: `4`
     * `min_num_points` (Integer) : **Minimum number of points required for GICP**
       - Default: `500`
     * `s2s` : **Point-to-point matching parameters**
       * `k_correspondences` (Integer) : **Number of correspondences**
         - Default: `15`
       * `max_correspondence_distance` (Float) : **Maximum correspondence distance**
         - Default: `0.25`
       * `max_iterations` (Integer) : **Maximum iterations**
         - Default: `32`
       * `transformation_epsilon` (Float) : **Transformation epsilon**
         - Default: `0.01`
       * `euclidean_fitness_epsilon` (Float) : **Euclidean fitness epsilon**
         - Default: `0.01`
       * `ransac`
         * `iterations` (Integer) : **RANSAC iterations**
           - Default: `5`
         * `outlier_rejection_thresh` (Float) : **RANSAC outlier rejection threshold**
           - Default: `0.1`
     * `s2m` : **Point-to-map matching parameters**
       * `k_correspondences` (Integer) : **Number of correspondences**
         - Default: `30`
       * `max_correspondence_distance` (Float) : **Maximum correspondence distance**
         - Default: `0.1`
       * `max_iterations` (Integer) : **Maximum iterations**
         - Default: `32`
       * `transformation_epsilon` (Float) : **Transformation epsilon**
         - Default: `0.0009`
       * `euclidean_fitness_epsilon` (Float) : **Euclidean fitness epsilon**
         - Default: `0.005`
       * `ransac`
         * `iterations` (Integer) : **RANSAC iterations**
           - Default: `6`
         * `outlier_rejection_thresh` (Float) : **RANSAC outlier rejection threshold**
           - Default: `0.05`

* `mapping` : **KFC-Map configuration**
   * `frustum_search_radius` (Float) : **Frustum search radius**
     - Default: `0.009`
   * `radial_distance_thresh` (Float) : **Radial distance threshold**
     - Default: `0.01`
   * `delete_delta_coeff` (Float) : **Coefficient for deleting map points**
     - Default: `0.03`
   * `delete_max_range` (Float) : **Maximum range for deleting map points**
     - Default: `4.0`
   * `add_max_range` (Float) : **Maximum range for adding map points**
     - Default: `4.0`
   * `voxel_size` (Float) : **Voxel size for map processing**
     - Default: `0.08`

* `fiducial_detection` : **Fiducial detection configuration**
   * `max_range` (Float) : **Maximum range for fiducial detection**
     - Default: `2.0`
   * `plane_distance_threshold` (Float) : **Distance threshold for plane detection**
     - Default: `0.005`
   * `plane_eps_thresh` (Float) : **Epsilon threshold for plane detection**
     - Default: `0.1`
   * `vox_resolution` (Float) : **Voxel resolution for fiducial detection**
     - Default: `0.03`
   * `remaining_points_thresh` (Float) : **Remaining points threshold for fiducial detection**
     - Default: `0.05`
   * `minimum_input_points` (Integer) : **Minimum number of input points for fiducial detection**
     - Default: `100`
   * `minimum_segmented_points` (Integer) : **Minimum number of segmented points for fiducial detection**
     - Default: `15`

* `traversibility` : **Traversibility analysis configuration**
   * `chunk_horizontal_range` (Float) : **Horizontal range for traversibility analysis**
     - Default: `3.5`
   * `chunk_vertical_range` (Float) : **Vertical range for traversibility analysis**
     - Default: `1.0`

---
### Tag Detection Node
_See file: [config/tag_detection.yaml](../config/tag_detection.yaml)_

* `image_transport` (String) : **Transport type for image stream**
   - Default: `"raw"`

* `num_streams` (Integer) : **Number of camera streams that will be used - determines how many stream configurations are subsequently checked (below)**
   - Default: `1`

* `stream0` (String Array) : **String configurations for the first stream. First element is the image topic, second is the camera info topic, last is the frame id of the camera (Subsequent streams configured by incrementing the index)**
   - Default: `["/arena/cam1/image", "/arena/cam1/camera_info", "map"]`

* `stream0_offset` (Float Array) : **The camera's pose within the previously provided frame id - if not provided an identity pose will be used (Subsequent streams configured by incrementing the index)**
   - Default: `[0., 0., 0., 1., 0., 0., 0.]`

* `feature` : **General feature configuration**
   * `publish_best_detection_tf` (Integer) : **Transform publishing behavior**
      | Option | Description |
      |-|-|
      | `1`    | Publish best detection transforms |
      | `0`    | Don't publish |
      | `-1`   | Publish best detection transform only when filtering succeeded |
     - Default: `0`
   * `export_best_detection` (Integer) : **Detection publishing behavior**
      | Option | Description |
      |-|-|
      | `1`    | Publish best detection |
      | `0`    | Don't publish |
      | `-1`   | Publish best detection only when filtering succeeded |
     - Default: `-1`
   * `debug`
     * `export_all_detections` (Boolean) : **Export all detections for debugging**
       - Default: `true`
     * `publish_stream` (Boolean) : **Enable publishing output stream with annotations**
       - Default: `true`
     * `publish_individual_tag_solution_tfs` (Boolean) : **Enable publishing of individual tag solution transforms**
       - Default: `false`
     * `publish_group_solution_tfs` (Boolean) : **Enable publishing of group solution transforms**
       - Default: `true`

* `filtering` : **Filtering configuration**
   * `bounds_min` (Float Array) : **Minimum valid bound for tag detection (in world frame)**
     - Default: `[0.2, 0.2, -0.3]`
   * `bounds_max` (Float Array) : **Maximum valid bound for tag detection (in world frame)**
     - Default: `[6.68, 5., 0.3]`
   * `use_bounds` (Boolean) : **Enable/disable the use of bounds for tag detection**
     - Default: `true`
   * `thresh`
     * `min_tags_per_range` (Float) : **Minimum number of tags required per [avg] meter of distance to detected tags**
       - Default: `0.0` (disabled)
     * `max_rms_per_tag` (Float) : **Maximum valid RMS error per tag**
       - Default: `0.40`
     * `min_sum_pix_area` (Float) : **Minimum total pixel area required for detected tags**
       - Default: `5000.0`
     * `require_nonplanar_after` (Float) : **Distance beyond which nonplanar tags (2 or more required) are required**
       - Default: `1.5`

* `aruco` : **Definition of which tags to detect**
   * `predefined_family_idx` (Integer) : **ID code for tag family to detect (20 = AprilTag 36h11)**
     - Default: `20`
   * `tag_ids` (Float Array) : **IDs of the tags to detect**
     - Default: `[0., 1., 2., 3., 4., 5.]`

   * `tag0_static` (Boolean) : **Determines if the tag is treated as part of a mobile body or statically attached to the environment (Subsequent tag parameters defined by incrementing the index)**
     - Default: `false`
   * `tag0_frames` (String Array) : **The primary and root frame ids for which the tag is definted within. The first element is the frame id that the tag is immediately within, and the second is an optional frame id which can be used to group tags that are statically linked (TF2 transforms must be available between these frames for this to work) -- (Subsequent tag parameters defined by incrementing the index)**
     - Default: `["base_link"]` (no root frame provided)
   * `tag0_corners` (Float Array) : **Tag corner coordinates (x, y, z) whose order must begin with the bottom right corner and traverse in the clockwise direction! Non-coplanar points will cause errors. (Subsequent tag parameters defined by incrementing the index)**
     - Default: `[0.230, -0.354, 0.401, 0.038, -0.354, 0.397, 0.034, -0.354, 0.589, 0.226, -0.354, 0.593]`

   * `tag1_static` (Boolean)
     - Default: `false`
   * `tag1_frames` (String Array)
     - Default: `["base_link"]`
   * `tag1_corners` (Float Array)
     - Default: `[0.038, 0.354, 0.397, 0.230, 0.354, 0.401, 0.226, 0.354, 0.593, 0.034, 0.354, 0.589]`

   * `tag2_static` (Boolean)
     - Default: `false`
   * `tag2_frames` (String Array)
     - Default: `["base_link"]`
   * `tag2_corners` (Float Array)
     - Default: `[-0.003, -0.353, 0.338, -0.100, -0.281, 0.336, -0.102, -0.281, 0.456, -0.005, -0.353, 0.458]`

   * `tag3_static` (Boolean)
     - Default: `false`
   * `tag3_frames` (String Array)
     - Default: `["base_link"]`
   * `tag3_corners` (Float Array)
     - Default: `[-0.100, 0.281, 0.336, -0.003, 0.353, 0.338, -0.005, 0.353, 0.458, -0.102, 0.281, 0.456]`

   * `tag4_static` (Boolean)
     - Default: `false`
   * `tag4_frames` (String Array)
     - Default: `["base_link"]`
   * `tag4_corners` (Float Array)
     - Default: `[0.364, -0.280, 0.477, 0.303, -0.360, 0.476, 0.301, -0.360, 0.576, 0.362, -0.280, 0.577]`

   * `tag5_static` (Boolean)
     - Default: `false`
   * `tag5_frames` (String Array)
     - Default: `["base_link"]`
   * `tag5_corners` (Float Array)
     - Default: `[0.303, 0.360, 0.476, 0.364, 0.280, 0.477, 0.362, 0.280, 0.577, 0.301, 0.360, 0.576]`

__*Last updated: 2/19/25*__
