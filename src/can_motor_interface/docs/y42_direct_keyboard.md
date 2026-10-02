# Y42 X 固件 USB-CAN 直连底盘测试

2026-10-01。适用：Linux can0 -> USB-CAN -> Y42 地址1/2/3/4。机械臂5/6可同总线，但本节点永不向它们发送，也不使用地址0广播。
旧 can_interface_node / robot_bringup.launch 未改变；测试时必须关闭它们以及所有其他CAN控制进程。

## 编译与运行
```bash
cd ~/ROS_WS-2/ROS_WS
catkin_make
source devel/setup.bash
roslaunch robot_bringup y42_keyboard_test.launch can_device:=can0 max_rpm:=30
```
can0应已UP且和所有电机一致为500000。电机必须设置X固件、CAN通信、固定6B校验以及正常默认速度单位。代码不会修改电机参数、写Flash、自动使能。
架空四轮，确保机械急停可用；确认电机已使能、无故障。启动时只查询/停止，不会自行运动。

终端2：
```bash
source ~/ROS_WS-2/ROS_WS/devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel_teleop _speed:=0.10 _turn:=0.30 _repeat_rate:=10.0 _key_timeout:=0.6
```
默认是i前进、逗号后退、j/l转向、k停止，不是WASD。此时先保持停止，不要按运动键。

终端3：
```bash
source ~/ROS_WS-2/ROS_WS/devel/setup.bash
rostopic echo -n 1 /y42_direct/status
rostopic echo -n 1 /y42_direct/ready
rosservice call /y42_direct/arm '{}'
```
ready表示反馈就绪，不表示已解锁。arm必须返回success:true，再回终端2按i等。
```bash
rosservice call /y42_direct/disarm '{}'
```
手动停止。Ctrl+C也会尽力逐轮发送FE停止；USB拔出/进程kill -9/电源故障无法保证停止命令送达，必须有硬件急停并检查电机通信丢失保护。

## 协议
发送速度：扩展ID=(地址<<8)，DLC8，F6 direction accel_hi accel_lo speed_hi speed_lo 00 6B。
默认accel=50 RPM/s，speed=abs(RPM)*10；负号direction=1。
示例仅用于抓包核对：地址1、+10RPM -> ID00000100 data F60000320064006B。
停止：FE 98 00 6B，逐地址发送。
查询：35 6B（转速）、3A 6B（状态）。
预期回复：35 sign speed_hi speed_lo 6B；3A flags 6B，扩展ID地址<<8。
只接受地址1..4包序号0的合法长度/校验回复；F6 ACK不算速度/状态心跳。
反馈必须每个电机两类都新鲜；使能bit0必须为1；0x08堵转保护立即阻止运动；0x04堵转标志按文末确认窗口处理。bit7/0x80 是默认置1的掉电记录，不单独阻止启动；0x83表示已使能、已到位及掉电记录，并非当前堵转故障。
若实际回复与上述不同，先保留candump定位，不允许通过关反馈检查绕过。

## 保护与限幅
- 手动arm前：全部反馈有效、已使能、无故障、四轮速度<=1RPM、最近输入为零。
- armed时：反馈超时1秒或轮速命令超时0.5秒立即发停止，必须重新arm。
- /emergency_stop=true永久锁存到节点重启，arm不能清除外部急停。
- CAN发送失败/非法输入锁故障，纠正原因并重启。
- 正常停止指令直接发送FE，不必等待原先F6加速度斜坡。
- 不自动清除驱动故障，不自动重连后恢复旧目标。
- 仅/cmd_vel_teleop的键盘超时仍由teleop和mux负责；驱动另行检查/motor_velocity_cmd超时。

## 地址顺序和方向
motor_ids按/motor_velocity_cmd数组顺序映射（planner日志F,R,B,L），不假定轮子物理安装。
```bash
roslaunch robot_bringup y42_keyboard_test.launch motor_ids:='[1,2,3,4]' direction_signs:='[1,1,1,1]' max_rpm:=30
```
direction_signs允许+1/-1，输入和反馈同时反向。不要未检查方向就落地跑。

## 诊断
```bash
candump can0
rostopic echo /y42_direct/status
rostopic echo /motor_state
rostopic echo /cmd_vel_mux/selected_source
```
本节点不再使用STM32自定义0x201/0x181心跳；也不发布旧/can_rx话题。
如还出现旧CAN heartbeat timeout，说明旧can_interface_node仍在运行，先停掉。
某个地址missing/stale：检查地址唯一性、500k、供电、使能、终端电阻和实际应答帧。

## 验证范围
已做Python语法/XML检查、7项纯Python协议和arm保护测试：
```bash
python3 src/can_motor_interface/test/test_y42_direct.py
```
未做Linux catkin编译、vcan集成或实机验证。不声明已保证四轮实机能转。


## 2026-10-02：短暂堵转标志确认窗口
默认 `stall_warning_timeout:=0.5` 秒，范围0..1秒（0恢复立即停止）。
- 已armed：0x04连续存在达到窗口才停止；每个电机独立计时，重复状态包不重置计时，标志清除才结束本次计时。
- 0x08堵转保护立即停止；0x89/0x8D不会等待窗口。
- 未使能、反馈超时、命令超时、CAN失败、外部急停逻辑保留。
- /arm仍拒绝正在报告0x04的电机；不会靠重复arm绕过检测。
- 短暂0x04期间ready=false（不满足新解锁条件），但已armed可继续至确认窗口。status会明确输出armed + WARNING。
- 日志包含地址、目标/实际RPM及标志持续时间。帧轮询和调度引入额外检测时间，窗口不是硬实时保证。
- 不修改电机电流、最大转速、加速度、硬件堵转参数，不声称驱动保护在所有情况下必然有效。
```bash
roslaunch robot_bringup y42_keyboard_test.launch can_device:=can0 max_rpm:=30 stall_warning_timeout:=0.5
```
按实际验证过的轮位另加motor_ids；本次未改映射。
仍然需要硬件急停，软件/USB-CAN断开时不能保证停止命令送达。


## 2026-10-02：ENOBUFS / Errno 105发送队列修正
旧实现每轮可能集中发送8条查询+4条控制，且故障后仍不断追加查询和停止。
新TxSchedule一次只选一帧，最小发送间隔4ms，不补发因调度延迟错过的周期，不缓存旧运动目标。
运动指令和查询轮询错开；正常运行每轮速度目标约20Hz、每个查询约10Hz（实际取决于调度）。
未解锁时降低停止帧频率。发送失败立即取消armed并锁存故障；停止普通查询和运动，仅做两轮逐轮FE停止尝试，帧间至少50ms。后续不自动清故障、不重放运动。
FE成功写入socket仍不证明电机已执行停止，断线/堵总线必须使用硬件急停。
/arm不能清CAN发送故障，必须修复链路并重启节点。

排查步骤：
1. 确认硬件停止、支撑车体；不要仅凭绿灯判断已停止。
2. 在重启/复位CAN接口前保存：`ip -details -statistics link show can0` 和 `tc -s qdisc show dev can0`。
3. 检查BUS-OFF/ERROR-PASSIVE、错误计数是否增长、USB断连和电机侧供电/接线/终端电阻。
4. 关闭所有底盘发送程序。同步新版代码、catkin_make并重启测试。
5. 如需复位接口/调整短队列，必须在硬件已停止且没有发送节点时操作；可使用txqueuelen=100作为诊断配置，但不能靠无限加大队列掩盖无ACK或总线故障。不要启用自动恢复运动。
6. 再确认四轮新鲜反馈与零输入后手动arm，先架空验证。

纯Python测试现有23项，新增错峰、公平轮询、停止优先、重复停止请求合并、ENOBUFS故障锁存/有限尝试/不重放运动测试。未验证真实Linux SocketCAN/gs_usb的实际队列行为。
