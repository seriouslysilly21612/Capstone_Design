# Original COLOR camera capture time, propagated from DetectionArray.header.stamp.
# PickTarget has no std_msgs/Header, so this dedicated field carries the capture
# instant end-to-end and enables true E2E latency measurement in downstream nodes.
builtin_interfaces/Time capture_stamp

bool target_valid
int32 class_id
string class_name
float32 confidence
float32 center_x
float32 center_y
float32 width
float32 height
