# ROS → STM32 CAN1 → 电机：链路与协议核对

2026-10-08。当前架构是 STM32 接收并解析 ROS 自定义命令，再控制电机。

```text
键盘 /cmd_vel_teleop → cmd_vel_mux → /cmd_vel_muxed
  → planner → /motor_velocity_cmd → can_interface_node
  → Linux can0 / CANable → STM32 CAN1 命令解析 → 电机侧总线/驱动器
```

## 本次确认的问题与修复

- **根因：ROS 速度帧 DLC=7，STM32 CAN1 解析器要求 DLC=8，直接丢弃旧命令。**
  已改为 `01 idx int32_RPM_LE accel sync`，最后两个字节不再错误地使用校验字节。
- **反馈不匹配：ROS 等待 0x81+校验，固件发送 0x101/type=0x01、0x03，无校验。**
  现正确解析状态和实际转速，速度字段除以 10 恢复 RPM，不将目标值当实际反馈。
- **固件默认 report_mask=3，未启用速度上报。** ROS 现发送 `07 FF 07 00 00 00 00 00`，
  请求基础状态、位置、速度，等待 0x102 ACK，未确认时重试，重连后重新配置。
- 原来只处理广播故障，忽略单电机 type=0x06 事件；已补齐。急停复位会清除旧运动目标。
- 心跳每 100 ms 发送，低于固件 500 ms 超时。ACK/统计回包可确认 STM32 在线。
- 原阻塞 socket 和跨线程 close/reopen 可能挂住读取或发送；现使用非阻塞 I/O 并同步 fd 生命周期。
- 原日志把 bind 成功称为已连接；现区分绑定、内核接受 TX 与收到 STM32 回包，明确输出接口、ID、帧类型。
- 整机入口传递接口与轮序参数；本固件仅允许标准帧 TX=0x100/RX=0x101，错误 ID/IDE 启动即报错。
- Y42 原生发送脚本和键盘入口没有执行权限；已修复。它们仍属于不同用途，不能混用。

最初离线检查时主机只有 `lo/ens37`，没有 `can0`。后来接入 CANable 后的实机结果见文末。
已读取后来加入的 `STM32_Motor_Controller` 源码；CAN1 滤波器接收所有帧、FIFO0 通知已启用，
8 MHz HSE 下 PCLK1=36 MHz，Prescaler=6、总时间片=12，所以 CAN1/CAN2 均为 500 kbps。
已将实际 ROS 编码器输出送入实际 STM32 解析器/电机控制源文件做 HAL mock 联测，
复现旧 DLC=7 拒收，并验证修复后四轮正负转速到 CAN2 F6 报文的 20 组字节。
上述离线结果本身不能证明物理总线正常；后续实机已验证地址 2 的转动与停车，见文末记录。

## 先区分三种入口

| 场景 | ROS 入口 | 发给 CAN1 的内容 |
|---|---|---|
| 整机自定义协议 | `can_interface.launch`、`robot_bringup.launch`、导航整机入口 | 标准帧 TX=0x100，RX=0x101，DLC=8 |
| 同协议的独立底盘测试 | `stm32_custom_keyboard_test.launch` | 标准帧 TX=0x100，RX=0x101，DLC=8，30 RPM 限幅 |
| 原生 Y42 透传 | `y42_stm32_keyboard_test.launch` | 扩展帧 ID=地址<<8，F6/FE/35/3A 等原生负载 |

新自定义协议测试默认限幅 30 RPM，不启动任务或机械臂。
它没有 `/y42_direct/arm`；原生 Y42 节点的 arm 服务不适用于自定义协议节点。
启动前架空车轮、保持零速；同一时间只运行一套底盘 CAN 发送节点。

底盘 idx=0..3，默认映射电机地址 1..4；ROS 不向第 5 个机械臂电机发送速度指令。
逻辑轮序为前、右、后、左；上位端 `motor_indices=[1,0,3,2]` 对应实际地址 `[2,1,4,3]`，
与原生 Y42 键盘测试一致。命令和反馈都使用同一映射，并对称应用 `direction_signs`。
参数要求索引为 0..3 的排列、方向为 +/-1，禁止把机械臂索引 4 混入底盘。
若实际接线为另一种排列，可通过任一整机/测试入口传递这两个参数：

```bash
roslaunch robot_bringup stm32_custom_keyboard_test.launch \
  motor_indices:='[0,1,2,3]' direction_signs:='[1,1,1,1]'
```

此地址对应关系使用固件默认映射；运行期间不要另外发送 cmd=8 修改固件地址表。
心跳 DLC=8：`09 FF 00 00 00 00 00 00`。
固件兼容 0x201 标准/扩展速度命令，但仍要求 DLC=8；状态仍从标准帧 0x101 返回，
不存在旧 ROS 文档所写的 0x181/0x81/sum8 协议。本次使用标准帧 0x100 主协议。

## 构建并加载

```bash
cd /home/fire/ros_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg can_motor_interface arm_interface motion_planner robot_bringup
source devel/setup.bash
```

每个运行终端都加载同一工作空间。只更新源文件但继续使用旧二进制，不会应用 C++ 修复。

## 只读检查物理接口

```bash
ip -details -statistics link show can0
rosrun can_motor_interface check_stm32_can.py --device can0 --bitrate 500000 \
  --rx-id 0x101 --frame-format standard --listen-seconds 5
```

诊断脚本不发 CAN 帧、不更改接口；检查 UP、CAN 控制器状态、速率、listen-only/loopback，
并区分本地 TX 回显与真正从总线收到的 RX。未运行节点时没有回包不能单独判定故障。
`ENOBUFS`、TX error 增长或 BUS-OFF 时先处理 ACK/波特率/收发器/H-L-GND/终端电阻。
STM32 CAN1 外设名不等于 Linux `can1`；单 CANable 通常只有 Linux `can0`。

## 联调当前固件

若接口未配置，电机停止后在运行主机配置 500 kbps：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

启动自定义协议底盘测试：

```bash
roslaunch robot_bringup stm32_custom_keyboard_test.launch
```

另一个终端使用键盘入口（先保持零输入，确认回包正常再进行低速测试）：

```bash
rosrun robot_bringup y42_keyboard.py
```

`rostopic echo /motor_velocity_cmd` 应有四个 RPM 值；观察节点启动日志的 TX/RX/帧类型，
结合 `candump can0`、STM32 CAN1 RX 回调计数和 `/motor_state` 定位断点。
Linux 上有 TX 回显但 STM32 RX 中断不进：检查 CAN1 速率、滤波器 IDE/ID、FIFO/通知、
CAN 收发器及接线。STM32 中断进了但命令不执行：检查 DLC、负载、索引、ACK result。
不要通过关闭心跳/急停来掩盖无反馈。

确认底盘链路正常后再启动整机，协议默认已经匹配，无需切换扩展帧：

```bash
roslaunch robot_navigation mission_complete.launch
```

这只配置底盘节点；机械臂、药箱、测距等仍有各自协议，不能认为底盘可用就已完成整机联调。

## 电机健康、故障和急停

`0x101/type=3` 是 STM32 周期重发的缓存，不能证明电机刚刚回复。
ROS 每 50 ms 交错查询一个电机的 0x35 实际速度或 0x3A 状态，每轮查询周期 400 ms。
这些是只读原生扩展帧，利用现有固件的双向透传；运动命令仍由标准 0x100 解析执行。
`/motor_state` 仅取真实 CAN2 原生回包，不再从缓存刷新速度或接收时间。

运动前必须满足：四轮速度/状态均未过期、均使能且没有堵转保护、收到心跳 cmd=9 的 ACK、
report_mask 配置已确认、三类统计均在线。默认过期时间 1 s；启动最多等待 2 s。
`/motor_link_ready` 发布当前可运动状态。缓存/统计回包不会替代真实电机在线证据。
里程计反馈过期后停止发布 odom/TF，恢复时不跨断链时段积分。

标准 0x102 负 ACK、CAN2 原生错误回复，以及 0x103 中队列丢弃、执行失败、
CAN1/CAN2 发送失败或停车失败计数变化，都会锁定急停。首次统计作为历史基线；后续比较支持 16 位回绕。
实机发现：固件汇总响应超时计数包含未接入的第 5 路后台查询，不能直接等同于底盘运动失败。
该计数变化会告警；底盘另行检查四轮真实速度/状态和逐轮 F6 原生运动 ACK，
运动发出后任一轮超过 500 ms 没有运动 ACK 就急停。连续重发目标不会延长应答期限。
急停与每个运动写入共用互锁，急停锁定期间不发送 F6/同步启动，并每 100 ms 重试 cmd=3。
心跳、查询和命令过期使用实际经过时间，不受 `/clock` 暂停影响。

故障修复后，要求真实反馈与 ACK/统计均恢复，且至少 1 s 没有新故障，再通过现有
`/emergency_stop/reset` 发送 true。复位会清掉旧目标，必须重新给出运动命令。

## 单轴机械臂接口

`arm_interface` 现在使用标准 0x100 的 cmd=4 设置速度、cmd=2 设置绝对位置，固定索引 4 / 地址 5。
`/arm_joint_cmd` 只接受一个关节 `joint_1`，单位 rad；按 3200 脉冲/圈编码，param1=1。
原生 0x36 位置回包单位为 0.1°，转回 rad 后发布 `/arm_state`，不会发布缓存位置冒充新采样。
接口依赖底盘节点的 `/can_rx` 和 `/motor_link_ready`；未就绪、急停或机械臂反馈过期时拒绝运动。
六关节旧占位配置与 0x200+关节号/0x10 报文已移除。

同一电机只启动一个上位控制者。`mission_complete.launch` 默认保留 `arm_and_gripper` 放药控制；
`arm_joint_control:=true` 切换为 JointState 单轴接口。其他整机入口同时启用两者时选择放药控制。

## 离线验证

```bash
python3 can_motor_interface/test/run_stm32_contract_test.py --firmware STM32_Motor_Controller
```

编译实际 ROS 编码器和固件解析/驱动/控制源文件，使用固件已有 HAL mocks；
产物放临时目录，不修改固件、不烧录、不向物理 CAN 发报文。
联测还覆盖逻辑轮序到 CAN2 地址，以及机械臂 +/-pi rad 到地址 5 的绝对 +/-180° 报文。

## 2026-10-08 实机验证

主机识别 gs_usb / can0，用户启用 500 kbps 后收到标准 0x101/0x103 和真实 CAN2 原生回包。
实际只接四个底盘电机，第 5 路未接；此前直接把汇总响应超时变化作为底盘急停的策略会误拦截。
已保留该统计告警，改由逐轮真实反馈和 F6 运动 ACK 期限保护底盘；相关回归测试通过。

通过实际 can_interface_node（max_rpm=10）向逻辑前轮 / STM32 index=1 / CAN2 地址 2
发送 10 RPM 目标，持续最多 1 s，其他三个目标保持 0。真实驱动速度反馈经过约 5、9.7 RPM，
峰值约 10.6 RPM。随后发送零目标和急停，真实四轮反馈均回到 0；没有运动 ACK 超时。
测试进程随后关闭。上述首轮测试只验证地址 2 的实际转动。

随后完成四轮单独转动、两组正反向及四轮同时正反转共 10 项测试，均通过并已停车。
实测速度、CAN 错误计数变化及验证范围见
[四轮底盘实机测试记录](STM32_底盘实机测试_2026-10-08.md)。
