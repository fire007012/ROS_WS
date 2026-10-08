# 单个路径点定义
float64 x          # 目标 x 坐标 (m)
float64 y          # 目标 y 坐标 (m)
float64 tolerance  # 到达判定误差阈值 (m)
bool has_next       # 当前点之后是否还有路径点
string frame_id     # 坐标系，默认由 path_manager 配置
bool has_yaw        # 是否要求终点朝向
float64 yaw         # 终点朝向 (rad)，仅 has_yaw=true 时有效
