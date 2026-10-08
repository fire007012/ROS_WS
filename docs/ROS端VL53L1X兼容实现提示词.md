# ROS 端 VL53L1X 兼容实现提示词

将下面完整内容发送给 ROS 端 AI。方括号中的值应由 ROS 工程实际情况替换；没有明确值时，AI 必须配置化而非猜测。

---

```text
你正在修改一个 ROS 2 机器人项目。请先阅读整个 ROS 工作区的 README、package.xml、launch 文件、现有 CAN 节点、TF 配置、Nav2 配置和测试方式，再实施。不要修改 STM32 固件；STM32 侧 CAN 协议已经固定如下。

## 任务目标

实现一个 ROS 2 节点，将 STM32 上报的三个 VL53L1X 距离传感器 CAN 帧转换为标准 ROS 话题和诊断信息，并安全地接入现有的局部避障/Nav2 配置。

传感器安装方向：
- ID 0：前方，ROS frame_id 为 `front_range_link`
- ID 1：左侧，ROS frame_id 为 `left_range_link`
- ID 2：右侧，ROS frame_id 为 `right_range_link`

注意：这三个单点 ToF 传感器只能辅助近距离避障，不能代替 2D 激光雷达的 `/scan`。不要把它们伪装成完整 360 度雷达。

## 已固定的 CAN 协议

总线：SocketCAN，默认接口 `can0`，500 kbit/s。
CAN 类型：标准数据帧，DLC=8，little-endian。

### 距离样本：CAN ID `0x110`

```text
byte0     type，固定 `0x01`
byte1     sensor_id：0=前、1=左、2=右
byte2..3  distance_mm，uint16 little-endian
byte4..5  sigma_mm，uint16 little-endian；`0xFFFF` 表示固件未提供
byte6     status 位
byte7     sequence，每个传感器独立递增，uint8 回绕
```

`status` 位：

```text
bit0 VALID         距离有效
bit1 OUT_OF_RANGE  超出配置量程
bit2 TIMEOUT       本次测量超时
bit3 I2C_ERROR     STM32 与该传感器的 I2C 通信错误
bit4 LOW_QUALITY   质量不足
bit5 EMERGENCY     当前样本触发 STM32 本地急停
bit6..7 保留，必须忽略
```

无效样本的约定：

```text
distance_mm = 0xFFFF
sigma_mm = 0xFFFF
VALID = 0
```

### 传感器诊断：CAN ID `0x111`

```text
byte0     固定 `0x01`
byte1     sensor_id，或 `0xFF` 表示总线级诊断
byte2     当前错误码 / 最近 HAL 状态
byte3     连续错误次数，饱和到 255
byte4..5  最近一次有效距离，uint16 little-endian
byte6     最近样本 status
byte7     sequence
```

## 实现约束

1. 优先复用项目已有的 CAN 库/封装。若没有，选择与项目依赖、ROS 发行版一致的方案，例如现有 `python-can`、`ros2_socketcan` 或 C++ SocketCAN；不要无理由再引入第二套 CAN 框架。
2. CAN 接口名称、CAN ID、话题名、frame_id、最小/最大量程、超时、ToF 视场角、安装位姿必须全部参数化，并提供 YAML 默认值。默认 CAN 接口可为 `can0`。
3. 对 `0x110` 严格校验：标准帧、ID、DLC=8、`byte0==0x01`、sensor_id 仅允许 0/1/2。非法帧丢弃并增加诊断计数，不能导致节点退出。
4. 使用接收时刻作为 ROS Header 时间戳；记录并检查每个传感器的 sequence，检测丢帧、重复和回绕。
5. 为每个传感器发布 `sensor_msgs/msg/Range`：
   - `/front/range`
   - `/left/range`
   - `/right/range`
   - `header.frame_id` 分别为 `front_range_link`、`left_range_link`、`right_range_link`
   - `radiation_type = INFRARED`
   - `range = distance_mm / 1000.0`，仅对 `VALID=1` 的样本发布正常数值
   - `min_range`、`max_range`、`field_of_view` 从参数读取，单位为 m/rad
   - 不要把 `0xFFFF`、超时、I2C 错误、低质量值转换为“远处无障碍物”。无效数据应体现为诊断错误；若仍发布 Range，需采用 ROS 消费者能正确处理的明确无效语义，并在 README 说明。
6. 发布 `diagnostic_msgs/msg/DiagnosticArray` 到 `/diagnostics` 或项目已有的诊断聚合接口：每个传感器至少报告最后接收时间、数据年龄、有效性、状态位、最近距离、sigma、sequence、丢帧数、非法帧数和连续故障数。
7. 增加 watchdog：在参数 `sensor_timeout_ms`（建议初值 200 ms）内没有某一路的有效新帧时，将该路诊断置为 ERROR。不要把过期数据继续作为当前障碍物数据。
8. 收到 `EMERGENCY` 状态位或 `0x111` 诊断帧时，发布清晰的诊断事件/日志。不要绕过 STM32 的停车决定；STM32 是本地停车的最终执行者。
9. 提供 `base_link -> front_range_link`、`left_range_link`、`right_range_link` 的静态 TF 接入方式。安装的 x/y/z 和 roll/pitch/yaw 必须作为 launch/YAML 参数，不能编造具体坐标。
10. 检查当前 Nav2 配置：默认 Nav2 `obstacle_layer` 通常消费 `LaserScan` 或 `PointCloud2`，不应假设它自动消费 `sensor_msgs/Range`。
    - 若工作区已有 range sensor layer，正确配置三个 Range 话题、观察源和数据超时。
    - 若没有，应根据当前 Nav2 版本和已有依赖，选择兼容的 Range-to-PointCloud2/成本地图接入方案，并说明选择原因。
    - 不能将无效或过期 ToF 样本转换为自由空间清除数据。
    - ToF 仅作为近距离局部避障补充；现有 `/scan` 激光雷达、`/odom`、`/imu/data`、TF 和 Nav2 主流程不可破坏。
11. 节点退出时关闭 CAN socket/线程/定时器；不要 busy-loop；使用适合传感器数据的 QoS。
12. 不修改任何电机 RPM、底盘运动学或 STM32 命令协议，除非当前 ROS 工程已有相同的接口且确有必要。这个任务只负责测距 CAN 接收、ROS 发布、诊断、TF 接入和 Nav2 兼容配置。

## 交付物

请直接完成并展示：

1. 新增或修改的 ROS 2 package、节点源文件、`package.xml` 和构建文件。
2. 默认 YAML 参数文件，至少包含：
   - `can_interface`
   - `distance_can_id`、`diagnostic_can_id`
   - 三个传感器的话题、frame_id、min/max range、FOV、timeout
   - 静态 TF 位姿参数或与现有 robot_description/TF 的集成方法
3. 可运行的 launch 文件，能够启动 CAN bridge、TF 和对应 Nav2 配置接入。
4. Nav2 兼容配置变更，并说明其使用的是哪一种 Range 传感器接入机制。
5. 单元测试：至少覆盖 little-endian 距离解码、三个 sensor_id 路由、无效状态、未知 ID、错误 DLC、sequence 回绕/丢帧和 watchdog 超时。
6. 一份简短 README：接口协议、启动命令、SocketCAN 配置前提、话题列表、TF 参数、无效数据语义和 Nav2 行为。
7. 运行项目已有的相关测试/构建命令，报告实际结果；如果失败，保留失败日志和原因，不要声称通过。

## 验收示例

给定 CAN 帧：

```text
ID=0x110, DLC=8, data=01 00 52 03 0A 00 01 2C
```

节点必须识别为前向传感器、距离 `0.850 m`、sigma `0.010 m`、有效样本、sequence=44，并发布到 `/front/range`。

给定：

```text
ID=0x110, DLC=8, data=01 01 FF FF FF FF 04 2D
```

节点必须识别为左侧传感器超时/无效，不能把它发布为正常的远距离障碍物数据，且诊断必须反映错误。

请先给出你发现的现有 ROS 工程接口和实施计划，然后以最小、可测试的改动完成实现。不要使用占位符、虚构的 package 名称或假设的 TF 安装坐标；未知信息请使用参数并在 README 明确说明。
```
