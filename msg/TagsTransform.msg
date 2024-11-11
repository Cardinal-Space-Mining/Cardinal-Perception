geometry_msgs/TransformStamped estimated_tf
float32 pix_area
float32 avg_range
float32 rms
uint32 num_tags
# Mask Bits:
# 0. in_bounds
# 1. tags_per_range_ok
# 2. rms_per_tag_ok
# 3. pix_area_ok
# 4. nonplanar_ok
uint32 filter_mask