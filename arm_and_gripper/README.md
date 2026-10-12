# STM32 两路 DS3230MG + SG90 放药动作

硬件：PC6/TIM8_CH1、PC7/TIM8_CH2 两只机械臂舵机；PB0/TIM3_CH3、
PD14/TIM4_CH3 两只药箱 SG90。PC8/CH3、PC9/CH4仍为预留，不输出PWM。
无需运行旧Y42机械臂接口或pigpiod。

## 启动与触发

先烧录当前STM32固件，两端源码同步后在树莓派工作空间编译、source：

```bash
cd ~/ROS_WS-2/ROS_WS
catkin_make
source devel/setup.bash
# 先检查接口；以下配置适用于can0尚未UP时
ip -details link show can0
sudo ip link set can0 up type can bitrate 500000
roslaunch arm_and_gripper arm_and_gripper.launch
```

启动只等待服务，不自动运动。另一个已source的终端触发：

```bash
# 1床、1号药箱（PB0）
rosservice call /arm_place_medicine "bed_id: 1
box_id: 1"
# 1床、3号药箱（PD14），药箱与床号独立
rosservice call /arm_place_medicine "bed_id: 1
box_id: 3"
rostopic echo /medicine_release_done
```

每次服务按以下顺序执行：

1. 两只DS3230MG到配置的伸出角度。
2. 根据box_id只打开对应SG90。
3. 对应SG90回到关闭角度。
4. 两只DS3230MG回到配置的收回角度，释放远程控制权。

完整比赛任务入口仍为 `roslaunch robot_navigation mission_complete.launch`；
它已包含本节点，不要重复启动。完整任务中的 `/open_medicine_box` 只确认药箱选择，
避免SG90早于机械臂动作。路径、扫码、音视频等完整比赛设置仍需按实际硬件配置。
`robot_bringup.launch` / `complete_navigation.launch` 可用 `enable_arm_and_gripper:=true`
包含本节点；旧 `enable_arm` 不再启动 Y42 JointState 接口。

## 配置

默认文件：`config/arm_and_gripper.yaml`。

- `arm_place_angle_1/2`：两只舵机相对STM32校准零位的绝对伸出角；默认15/15仅供联调。
- `arm_home_angle_1/2`：绝对收回角，默认0/0；不是反向旋转相同角度。
- `arm_move_timeout_s`：ACK之后的动作等待时间，默认3秒，**不是物理到位反馈**。
- `old_servo_*`：PB0药箱1；`new_servo_*`：PD14药箱3。
- 默认SG90关闭180度、打开0度，与此前上电180度目标对齐；按真实机构校准开关方向。
- `arm_ack_timeout_s`：等待STM32命令接受确认，默认0.5秒。

允许两路角度分别设置，当前ROS校验范围为-60～60度，STM32也有独立限位。
STM32的 `Core/Inc/arm_servo_config.h` 保存每路零位脉宽、180/270度行程及方向。
使用其它接口可传 `can_device:=...`；其它配置传 `arm_config:=/absolute/path/config.yaml`。

## CAN 与取消行为

- 机械臂：扩展ID0x701，0x21为两路绝对角度；ACK扩展0x702。
- SG90：扩展ID0x700，0x22为位置命令；ACK扩展0x703。
- 帧有序号和sum8校验；缺少ACK或固件拒绝时中止，不继续开箱。
- ACK表示写入PWM目标，不证明实际位置。`/medicine_release_done=true`只表示
  命令已确认并完成配置等待；真实放药成功需实测或后续位置/药品反馈。
- 急停/取消：保留机械臂当前PWM目标，关闭两个SG90，中止后续动作，不自动缩回机械臂。
- `/emergency_stop/reset` 只清除急停标志，不恢复旧动作；需要任务重新授权并重新调用服务。
- 正常完成后恢复本地按键；取消后保留远程锁，防止按键改动未知负载姿态。

`ARM_SERVO_KEY_TEST_ENABLE=1`本地控制DS3230MG，0控制SG90；两种取值均初始化
DS3230MG并支持远程CAN。默认仍保留用户已验证的1U。复位会将机械臂目标重新设为校准零位。

## 验证

```bash
catkin_make tests
rostest arm_and_gripper arm_cancellation.test
```

测试注入内存CAN发送和确认，仅检查真实ROS控制器的顺序/取消等逻辑，不驱动机构。
旧 `/place_medicine` 仅作兼容别名：三个角度设-1且reset_arm=true，使用YAML配置
执行药箱1的四步流程；推荐明确指定床号与药箱的 `/arm_place_medicine`。
