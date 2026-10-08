# ROS Noetic 全向轮 + STM32(CAN) 联调测试说明

本文档用于验证以下链路：

1. `/cmd_vel`（直接遥控输入）-> `cmd_vel_mux` -> `/cmd_vel_muxed` -> `motion_planner` 全向轮解算
2. `/motor_velocity_cmd` -> `can_motor_interface` 自定义 CAN 速度帧
3. STM32 收到后转发给驱动器，并回传状态帧

> 2026-10-08 校正：以下协议已按 `STM32_Motor_Controller` 源码更新。
> 原文的 DLC=7、0x181/0x81/sum8 与该固件不匹配，导致速度命令被丢弃、状态无法解析。
> `y42_stm32_keyboard_test.launch` 是原生 Y42 扩展帧透传测试，不能用于替代
> 本文的自定义命令解析链路。详细排查见 [STM32 CAN 链路排查](STM32_CAN链路排查.md)。

---

## 1. 当前协议与节点约定

### 1.1 ROS -> STM32 速度控制帧

- CAN ID: `0x100`（标准帧）
- DLC: `8`
- Payload:
  - Byte0: 命令码 `0x01`（速度控制）
  - Byte1: 电机索引 `0~3` 或 `0xFF`（广播）
  - Byte2~5: `int32_t` 目标转速（RPM，默认小端）
  - Byte6: 加速度（RPM/s，默认 50）
  - Byte7: 同步标志（0，立即执行）
  - 无额外校验字段

### 1.2 STM32 -> ROS 状态帧

- CAN ID: `0x101`（标准帧，DLC=8）
- Byte0: `0x01`（基础状态）或 `0x03`（实际速度）
- Byte1: 电机索引
- type=0x01：Byte2~3 为状态字，Byte4~5 为故障码，Byte6 为故障标志
- type=0x03：Byte2~5 为小端 int32 实际速度，单位 0.1 RPM，ROS 除以 10
- 无 sum8 校验；type=0x04 是目标速度，不能当实际速度反馈
- ROS 以命令 0x07 请求 report_mask=7，修复固件默认未启用速度上报的问题

ROS 侧发布：
- `/motor_state` (`std_msgs/Float32MultiArray`)
- `/motor_status_flags` (`std_msgs/UInt8MultiArray`)
- `/motor_link_ready` (`std_msgs/Bool`)

注意：0x101 中的速度是 STM32 缓存重发。当前 `/motor_state` 改用固件转发的 CAN2 原生
0x35 回包，并结合 0x3A 状态检查每轮在线、使能和故障。四轮反馈、心跳 ACK、统计均就绪后才允许运动。
逻辑前/右/后/左默认对应 STM32 索引 `[1,0,3,2]`、电机地址 `[2,1,4,3]`；
可用 `motor_indices` 和 `direction_signs` 对称配置发送与反馈映射。

---

## 2. 启动步骤

当前固件 8 MHz HSE 下 CAN1 为 500 kbps；电机停止后配置接口：

```bash
sudo ip link set can0 down || true
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

编译并加载环境：

```bash
cd ~/ros_ws
catkin_make -j4 -l4
source devel/setup.bash
```

启动本文对应的自定义协议底盘测试（不启动任务或机械臂）：

```bash
roslaunch robot_bringup stm32_custom_keyboard_test.launch
```

另开一个终端抓包：

```bash
candump can0
```

---

## 3. 用 rostopic 手动发送 Twist

### 3.1 前进 0.5 m/s（`vy=0, wz=0`）

```bash
rostopic pub /cmd_vel geometry_msgs/Twist \
"linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10
```

本项目默认参数：`wheel_radius=0.05m`，`wheel_base=0.18m`。

全向轮解算（前/右/后/左）：

- `v1 = vx + ωL = 0.5`
- `v2 = vy + ωL = 0.0`
- `v3 = -vx + ωL = -0.5`
- `v4 = -vy + ωL = 0.0`

RPM 计算：`RPM = (v * 60) / (2πr)`

- 前轮约 `+95.5 RPM`（发送时取整为 `95`）
- 右轮 `0 RPM`
- 后轮约 `-95.5 RPM`（取整 `-95`）
- 左轮 `0 RPM`

### 3.2 预期 candump 现象（示例）

应持续看到 4 条发送帧（索引 0/1/2/3）：

- idx0, +95RPM: `01 00 5F 00 00 00 32 00`
- idx1, 0RPM: `01 01 00 00 00 00 32 00`
- idx2, -95RPM: `01 02 A1 FF FF FF 32 00`
- idx3, 0RPM: `01 03 00 00 00 00 32 00`

> Byte6=0x32 表示 50 RPM/s，Byte7=0 表示立即执行；不是校验字节。

新测试入口默认限幅 30 RPM，上述约 95 RPM 示例仅在显式调整限幅后成立。
扩展帧与标准帧即使数值 ID 相同也不能混用。`candump` 的本地 TX 回显只证明
Linux 产生了帧，不证明 STM32 已接收；请结合 STM32 接收计数/中断和真实回包判断。

---

## 4. 观察 STM32 回传与 ROS 状态

查看 ROS 状态：

```bash
rostopic echo /motor_state
rostopic echo /motor_status_flags
```

正常情况下：

1. `candump` 中有标准帧 `0x100`、DLC=8 下发帧；
2. 有标准帧 `0x101` 状态、`0x102` ACK、`0x103` 统计；
3. `/motor_state` 数据跟随命令变化；
4. 负转速应能正确显示为负值。

---

## 5. 常见问题排查

1. 看不到 `0x100`：
   - 检查 `can_interface_node` 是否启动。
   - 检查是否有 `/motor_velocity_cmd` 输入。

2. 有 `0x100` 但没有 `0x101/0x102/0x103`：
   - 检查 CANable 是否接入、CAN1 是否 500 kbps、STM32 FIFO0 中断是否进入。
   - 检查 `rx_can_id` 是否与 STM32 实际发送 ID 一致。

3. 有上报但 ROS 不更新：
   - 确认启动的是重新编译后的节点，按 type=0x01/0x03 分流，无 sum8 校验。
   - 确认 0x07 上报配置已被 ACK，速度字段为小端 0.1 RPM。
   - 底盘只使用索引 0~3，四个轮子均有新鲜速度子帧后才发布 /motor_state。

4. 驱动器转速异常：
   - 确认 ROS 下发单位是 RPM（整数）。
   - X 固件 `RPM*10` 换算由 STM32 执行，ROS 不再乘10。
