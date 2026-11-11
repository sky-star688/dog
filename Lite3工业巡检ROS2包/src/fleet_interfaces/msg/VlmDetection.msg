# One large-model (VLM) detection result, typed variant of the JSON contract.

std_msgs/Header header
string class_name
string label
float32 confidence
float32[4] bbox
float32[2] center
float32[2] grasp_point
bool has_grasp_point
string backend
string model
float32 latency_sec
string raw_text
