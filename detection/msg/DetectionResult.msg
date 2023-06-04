# 定义检测方法常量
#   METHOD_PLANAR   二维抓取检测
#   METHOD_SPATIAL  三维抓取检测
uint8 METHOD_PLANAR=1
uint8 METHOD_SPATIAL=2
# 检测节点不断发布的话题类型
# 字段说明：
#   method: 检测方法,又上面常量定义
#   success: 标志是否检测成功 
#   transformation: 检测成功时相机坐标系和抓取坐标系的转换关系,相当于相机坐标系下的抓取姿态
#   grasp_width: 抓取宽度,单位m
#   angle: 法兰盘旋转角度,单位rad
#   message: 检测结果信息
#   pre_grasp_pose: 预抓取点在相机坐标系下的坐标,仅在method为METHOD_SPATIAL时有效
uint8 method
bool success 
geometry_msgs/TransformStamped transformation
float32 grasp_width
float32 angle
string message
geometry_msgs/Vector3 pre_grasp_pose