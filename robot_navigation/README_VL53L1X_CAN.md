# STM32 六路 VL53L1X、后方防撞与整车横移

`vl53l1x_can_bridge_node` 沿用 SocketCAN 接收架构。CAN1 必须在系统配置为
500 kbit/s（通常对应 Linux `can0`）；ROS 不改接口位速率。接收标准数据帧
`0x110/0x111`、DLC=8、byte0=1；每路独立解析毫米小端距离、状态和序号。
六路均为红外 VL53L1X，有效量程 50～3000 mm。sigma=0xFFFF 表示不支持，
不会拒绝正常测距。STM32 持续上报，比较与阶段控制由 ROS 执行。

| ID | 位置 | Range（米） | Float32（毫米，兼容接口） | frame_id |
|---|---|---|---|---|
| 0 | 前方 | `/front/range` | `/vl53l1x_distance` | `front_range_link` |
| 1 | 原左侧 | `/left/range` | `/vl53l1x_distance_left` | `left_range_link` |
| 2 | 原右侧 | `/right/range` | `/vl53l1x_distance_right` | `right_range_link` |
| 3 | 机械臂上方左 | `/arm_upper_left/range` | `/arm_upper_left/distance_mm` | `arm_upper_left_range_link` |
| 4 | 机械臂上方右 | `/arm_upper_right/range` | `/arm_upper_right/distance_mm` | `arm_upper_right_range_link` |
| 5 | 后方 | `/rear/range` | `/vl53l1x_distance_rear` | `rear_range_link` |

每路诊断位于 `/diagnostics` 的 `STM32 Range/<配置的 name>`，包含数据/接收年龄、
有效状态、距离、sigma、sequence、重复/丢帧数量和 `0x111` 错误码。
STM32 sequence 保留在每路状态与诊断中（Range.header.seq 由 roscpp 分配）。
255→0 正常更新；相同序号不更新接收时间，
不重复发布有效值。无效测距、诊断故障和超时发布 NaN，使下游立即丢弃历史有效值。
LOW_QUALITY 可保留 Range 用于观察，但禁止用于横移；EMERGENCY 触发已有急停链路。

安装位置未知，未添加上方两路的虚构 TF。`frame_id`、量程与视场角在
`config/vl53l1x_can.yaml` 配置，需按实测安装确认。底盘四路静态 TF 默认不发布；
如已有 URDF/TF，直接沿用。如手工设置 `publish_tf:=true`，须填写底盘四路实际
位置与角度参数。后方提供 `rear_x/y/z/yaw/pitch/roll`，默认 yaw=π（朝后），
平移为安装占位值。机械臂上方两路应在实际机器人 URDF/静态 TF 中补齐实测变换。

## 后方传感器与防撞

无激光雷达的航点模式现可用底盘四路测距做横移绕行；任务启动默认开启。
实现位于路径跟踪器，防撞节点仍负责最高优先级停车。具体动作、参数和调试步骤见
[四路测距航点绕行](../docs/四路测距航点绕行.md)。

STM32 必须用 sensor_id=5 上报后方数据，保留原 ID 0～4。距离帧使用
`0x110: 01 05 distance_lo distance_hi sigma_lo sigma_hi status sequence`；
例如后方 800 mm、有效、序号 1：`01 05 20 03 FF FF 01 01`。
诊断沿用 `0x111` 的原字段布局，byte1=5；每路序号独立递增。
本仓库不含 STM32 固件，ROS 更新不会自动为 STM32 增加采样或发送逻辑。

`distance_safety_node` 订阅前/左/右/后四路。行驶、倒车和转弯只对新鲜有效的近障碍读数制动；
NaN、无回波、超量程、诊断无效或超时不单独停车。有效近障碍仍触发 `STOP` / `WARNING`，急停状态仍锁存。
P4/P6 前向 VL53 校准目标为 250 mm ±10 mm，需要有效前向读数，其他三路无需同时有效；
圆形车体半径 250 mm；前传感器在外缘，其余三路内缩 50 mm。制动和绕行内部分别将侧后读数减去 50 mm（最小为零），原始话题不变。
平移按车体外缘 100 mm 余量叠加制动距离、最低 150 mm；圆形原地转向净空为 100 mm。安装参数见 `config/chassis_geometry.yaml`。
健康监控默认关键话题为 `/odom` 和 `/motor_state`。若自定义关键列表仍包含测距，需同步调整。

主机端停车通过既有 `/cmd_vel_safety` → mux → 电机链路生效。后方固件本地
停车策略需在 STM32 中单独实现并实测，不能假定现有 ID 0/1/2 本地停车规则
自动覆盖 ID 5。首次上车应低速验证倒车、左右转弯、近障碍停车和无回波继续行驶，
并按实测制动距离调整参数。

## 阶段入口与控制权

默认关闭，普通启动仅采样与发布。完整任务中，显式启用：

```bash
roslaunch robot_navigation mission_complete.launch enable_arm_upper_alignment:=true
```

这一个开关同时启用任务入口和 CAN 桥中的控制模块。CAN 桥不能禁用，否则任务
会等待到阶段超时并停车。入口是 `mission_controller.cpp::checkGotoComplete()`：

- 到床 A 的导航完成 → `ARM_UPPER_ALIGN_A` → 原 `POSITION_IN_CIRCLE_A`。
- 到床 B 的导航完成 → `ARM_UPPER_ALIGN_B` → 原 `POSITION_IN_CIRCLE_B`。
- 两路新数据满足条件，差值持续处于停止阈值内 0.3 s，发布 done，进入原定位流程。
- 无有效输入时保持停车；原阶段超时机制终止任务，不跳过本横移阶段。

独立使用 CAN 桥时，`vl53l1x_can.launch` 同样默认关闭，需显式传入上述开关。
阶段名称列表在 `upper_alignment/stages`；外部任务若使用其他阶段，需在实际阶段
进入/退出处发布 `/mission/state`（String），并通过已有 `/mission/actions_enabled`
（Bool）授权任务动作。取消发布 `/mission/cancel`（Empty）。不要将阶段绑定到
正在执行导航或机械臂动作的状态；现有任务已加入单独阶段避免这种重叠。

阶段进入前排空缓存 CAN 帧，并清除两路有效值；保留 sequence 防止重复旧帧重新
授权。必须收到两路阶段进入后的新有效帧，才开始横移。各路接收时间使用单调时钟，
ROS 时间暂停或跳变不会延长有效期。接收年龄≤200 ms且两路接收差≤100 ms。

| 参数（CAN 桥私有 `upper_alignment/`） | 默认值 | 行为 |
|---|---|---|
| enabled | false | 总开关；launch 开关覆盖此值 |
| stages | ARM_UPPER_ALIGN_A/B | 允许启用的明确阶段 |
| start_difference_mm | 50 | 达到此差值开始移动 |
| stop_difference_mm | 20 | 差值≤此值立即停止 |
| speed_mps | 0.05 | 横移目标速度上限 |
| data_timeout_sec | 0.2 | 两路各自最大接收年龄；只能收紧 |
| max_skew_sec | 0.1 | 两路最大接收时间差；只能收紧 |
| settled_sec | 0.3 | 阶段完成要求的持续接近相等时间 |

中间区间只维持已有同向运动；符号反转先输出零，微小反向波动不能启动反向运动。
左短则 `linear.y>0`，右短则 `linear.y<0`，目标 `linear.x=angular.z=0`。

控制模块直接复用 CAN 桥收到的帧，向 `/cmd_vel_arm_alignment` 发布 Twist，
以 `/arm_upper_alignment/active`（Bool）切换现有 mux 的独占任务模式。
`/arm_upper_alignment/done`（Bool）供任务判断完成。控制模块不发送电机 CAN 帧。
输出沿用 `/cmd_vel_muxed` → 原 planner → `/motor_velocity_cmd` → 原 CAN 电机接口，
轮序、地址、方向符号及 `0x100/0x01`（或原有 `0x201`）协议均保持原实现。

mux 保留原加速度限制，停车直接输出零。急停、底盘锁和原障碍安全通道优先于横移。
退出模式清除缓存命令并实际发零；取消锁住速度输出，直到新任务授权动作。
横移模式下发布端失联也持续输出零，不回退到导航/遥控。mux 私有
`alignment_cmd_timeout_sec` 默认 0.2 s、只允许收紧。原 heartbeat 仍每 100 ms
发送 `0x100: 09 FF 00 00 00 00 00 00`；急停仍发送
`0x100: 03 FF 00 00 00 00 00 00`。

文档原约定 STM32 的 ID 0/1/2 ≤300 mm 本地停车；P4/P6 改为 250 mm 校准后，固件必须同步调整车体净空和停车规则，ROS 侧修改不会自动更新固件。横移模块也对这三路有效近距离
输出零，避免横移请求绕过原停车作用域。ID 3/4 的距离不会送入 ROS 障碍
停车节点的原三路话题，也不应在 STM32 中参与该本地阈值停车。ROS 不绕过本地
停车、故障或急停；原左/右仍用于方向障碍保护，永不作为上方比较输入。

## 验证

`test_vl53l1x_can_bridge.cpp` 检查六路协议、样例、有效量程、状态、sigma、标准帧
与回绕；`test_upper_alignment.cpp` 检查阶段门控、新数据、方向、滞回、缺失/无效、
超时、重复、时间差与重新进入。
`upper_alignment.test` 用测试专用原始 CAN 帧注入器运行生产解析/桥、mux、障碍
安全节点和 planner，并冻结 ROS 时间。它不启动真实 CAN 电机节点。
`rear_can_safety.test` 使用原始 ID 5 帧验证后方话题、倒车停车/减速/释放、
故障/无效/重复/超时/畸形帧，以及原地转弯和前进转弯的后方保护。

```bash
catkin_make
rostest robot_navigation rear_can_safety.test
```

后方接入验证（2026-10-14）：ROS Noetic 下 CAN 桥、安全节点、任务控制器和
健康监控编译成功；六路协议 10 项测试、后方原始 CAN 防撞集成测试、
原安全/微调测试和两种送药顺序的任务闭环测试通过。验证使用临时构建与模拟
传感器，不驱动实车；现有工作区可执行文件需重新 `catkin_make` 后再部署。

这些是自动化模拟验证。实车仍需确认传感器安装/TF、左右安装对应关系、CAN1
500 kbit/s、采样频率/延迟、车轮实际横移方向、低速及加减速、障碍与急停停车、
断线/心跳停车，以及本地停车仅覆盖 ID 0/1/2。

上方横移接入时的检查结果：目标节点在 ROS Noetic 下编译成功；测距协议 9 项、横移策略 7 项、
原电机协议/链路 20 项、键盘路由/接口默认值 8 项测试通过。新增原始帧横移 ROS
集成测试、新增任务阶段入口测试、原安全/微调 ROS 测试通过。完整任务 launch
已用现有音视频配置所需环境变量完成解析。

旧任务测试的放药服务夹具现已补齐 `/medicine/placed` 事件与床旁完成服务。
`mission_routes.test`、`mission_cancellation.test`、`mission_closed_loop.test` 均已通过。
状态机也修复了服务响应先于释放事件到达时的拒收问题，并保留缺失事件时的停车保护。
详见 [放药事件接入与验证](README_MEDICINE_EVENT.md)。

上方横移接入的修改文件按功能分组（未注明其他包名的路径相对于 robot_navigation）：

- 协议与测距：`include/robot_navigation/range_protocol.h`、
  `include/robot_navigation/vl53l1x_can_bridge.h`、`src/vl53l1x_can_bridge.cpp`。
- 横移策略：`include/robot_navigation/upper_alignment.h`。
- 任务入口：`include/robot_navigation/mission_controller.h`、`src/mission_controller.cpp`。
- 控制仲裁：`motion_planner/include/motion_planner/cmd_vel_mux_node.h`、
  `motion_planner/src/cmd_vel_mux_node.cpp`。
- 安全计时：`include/robot_navigation/distance_safety_node.h`、`src/distance_safety_node.cpp`。
- 配置与启动：`config/vl53l1x_can.yaml`、`launch/vl53l1x_can.launch`、
  `launch/mission_complete.launch`、`CMakeLists.txt`。
- 验证：`test/test_vl53l1x_can_bridge.cpp`、`test/test_upper_alignment.cpp`、
  `test/upper_alignment_bridge_fixture.cpp`、`test/test_upper_alignment_ros.py`、
  `test/upper_alignment.test`、`test/test_mission_upper_alignment.py`、
  `test/mission_upper_alignment.test`、`motion_planner/test/test_keyboard_velocity_routing.py`。
- 交付说明：本文档。
