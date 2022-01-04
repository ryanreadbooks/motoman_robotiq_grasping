# 检测节点不断发布的话题类型
# 字段说明：
#   success: 标志是否检测成功 
#   transformation: 检测成功时相机坐标系和抓取坐标系的转换关系
#   grasp_width: 抓取宽度，单位m
#   message: 检测结果信息
bool success 
geometry_msgs/TransformStamped transformation
float32 grasp_width
string message