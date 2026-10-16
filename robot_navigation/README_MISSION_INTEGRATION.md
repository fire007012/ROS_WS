# 全流程启动与业务模块整合（2026-10-17）

入口是 `roslaunch robot_navigation mission_complete.launch`，文件后缀是 `.launch`。
本次基于 ROS `035a669` 整合；沿用已联通的底盘路径规划、里程计、测距和避障配置。

## 实际流程

1. 提前在机器人启动全流程、医生电脑启动 `audio_video_slave.launch`，然后等待机器人本体启动按钮。
2. STM32 物理启动事件通过 CAN 0x112 转到任务节点；任务发布 Ready，启动音视频采集和传输。
3. 前往护士台，扫码得到 11/13/31/33，确定首床、首箱和另一床、另一箱。
4. 前往首床、微调入圈，等待床头条码识别，在 GUI 对应行保留条码。
5. 选择正确药箱，由 STM32 PWM 机械臂执行伸出、指定 SG90 开箱、关闭、收回序列。
6. 定时释放事件 `/medicine/placed` 到来时立即播放对应床号语音；播报与机械臂回程可以并行，均完成后进入床旁服务，底盘保持锁定。
7. 床旁服务等待本床新音视频接收连续满足 5 秒、ESP32 有效体征连续满足 2 秒；默认 28 秒内完成。满足后才前往下一床，重复扫码、放药、播报、床旁交流和测量。
8. 返回起始区，验证配置的整机投影与底盘静止，保持 5 秒后结束；GUI 继续保留两床条码。

`/medicine/placed` 当前由开箱等待时间推断，不是药品实际落点传感器反馈。有效体征位表示数据有效，不代表已经达到比赛精度。

## 本次修复与接通

- 修复任务节点内 YAML 最后加载导致 `skip_bedside_scan=false` 被旧 true 默认覆盖的问题；YAML 和 C++ 默认均改为必须扫码，launch 参数最后覆盖配置。
- 最新音视频约定是机器人 master、医生 slave。修复床旁服务还固定订阅 `/audio_chat/slave/peer_ready` 的问题；现在按角色选择真实接收状态与对端视频。
- 任务入口复用 `status_display.launch`，GUI 和 ESP32 桥接各启动一份；共享配置只加载一次，避免后续加载覆盖串口、全屏等显式参数。
- GUI 显示当前任务阶段和床旁等待状态；两床条码独立保留。调用摄像头状态及视频路由跟随音视频角色。
- 接通 ESP32 串口、体征整帧及数值话题参数；床旁完成依赖整帧，不能仅发布两个 Float32 数值冒充完整测量接口。
- 接通通话摄像头、麦克风、扬声器参数；放药播报默认使用与通话相同的扬声器，也可单独覆盖。
- 自定义 Ready 话题现在同时重映射任务节点的发布/订阅与音视频组件。
- GUI、扫码识别、播报和床旁节点为关键进程；启动失败或退出时全流程退出，避免显示/播报进程已经消失仍继续任务。

## 启动

机器人与医生端保持当前已联调的 ROS 网络配置；ROS Master 在机器人上。
默认音视频 IP 沿用最新源码：机器人 `10.161.170.93`，医生 `10.161.170.41`，本次未修改网络。

机器人 Ubuntu 工作空间内先构建并加载环境：

```bash
catkin_make
source devel/setup.bash
roslaunch robot_navigation mission_complete.launch
```

医生电脑加载相同 ROS 工作空间后：

```bash
roslaunch robot_navigation audio_video_slave.launch
```

随后使用机器人本体按钮开始任务。医生端需要在同一个 ROS Master 下，Ready 信号才会到达两端。不要再另启 `audio_video_master.launch` 或默认的 `status_display.launch`：全流程已包含机器人音视频、GUI 和 ESP32 桥接，重复启动会抢占节点名、摄像头或串口。

设备参数示例（路径和 PulseAudio 设备名按实际机器填写）：

```bash
roslaunch robot_navigation mission_complete.launch \
  scan_camera_device:=/dev/video0 \
  call_camera_device:=/dev/video2 \
  vitals_serial_device:=/dev/ttyUSB0 \
  gui_fullscreen:=false
```

可配置项：

| 参数 | 默认/作用 |
| --- | --- |
| `enable_camera` | true；护士台二维码和两床条码 |
| `skip_bedside_scan` | false；比赛保持 false，true 仅供显式联调 |
| `enable_arm` | true；包含 PWM 机械臂和两路 SG90；旧 enable_servo 参数已不控制本流程 |
| `enable_audio_video` | true；机器人 master，医生另启 slave |
| `audio_role` | master；床旁检测和 GUI 路由跟随此角色 |
| `audio_local_ip` / `audio_remote_ip` | 10.161.170.93 / 10.161.170.41；改变网络时还需保证启动 shell 的 ROS 网络变量正确 |
| `audio_mic_device` / `audio_speaker_device` | default；PulseAudio 设备名 |
| `speech_speaker_device` | 默认跟随 audio_speaker_device，可单独指定 |
| `speech_bed1_audio_file` / `speech_bed3_audio_file` | 随包提供的 bed1_kai.wav / bed3_kai.wav |
| `enable_patient_measurement` | true；ESP32 USB 桥接及每床测量门槛 |
| `vitals_config` | config/status_display.yaml；串口等持久配置 |
| `vitals_transport` / `vitals_serial_device` / `vitals_baudrate` | 空值沿用 YAML，显式值覆盖；默认串口115200 |
| `vitals_frame_topic` | /vitals/frame；bridge、GUI、床旁服务一致 |
| `heart_rate_topic` / `temperature_topic` | /vitals/heart_rate / /vitals/temperature；实际重映射到 bridge 和 GUI |
| `enable_gui` / `gui_fullscreen` | true / true；比赛需要实体屏显示 |
| `bedside_timeout_s` / `call_duration_s` | 28 / 5 秒 |
| `measurement_valid_hold_s` | 2 秒 |

默认稳定串口路径见 status_display.yaml；若本机 CP2102 的 by-id 不同，通过 vitals_serial_device 覆盖。ESP32 当前源码上电连续测量，ROS 不下发采集启停指令。

自定义 `patient_measurement_launch` 需要声明 config_file、load_config、transport、serial_device、baudrate、vitals_frame_topic、heart_rate_topic、temperature_topic 这些传入参数，并向指定整帧话题发布符合默认 bridge 的 JSON；如不使用某参数，也应声明以通过 roslaunch 的 include 参数检查。

## 对照比赛规则仍需验收的事项

对照用户提供规则第 10—11、13 页，流程所需的扫码、两箱送药、播报、两床音视频与体征显示、回归已接入。以下条件需要实物/场地验收，不能据离线测试宣称全部得分：

- 药品是否全部落在床头柜中心直径100mm圆内；当前机械臂角度、SG90开关方向与时长须实测。
- 心率 ±3 次/分、额头体温 ±0.5°C 的满分精度；需按规则指定部位与参考仪表比对。
- 开箱至播报结束底盘持续在半径300mm圈内且不移动。目前已有入圈检查和底盘锁定，仍未增加全程持续复核位置与速度的保护。
- GUI 在实体屏幕上保持两床条码、有效体征与医生视频；离线没有启动真实 Tk 窗口或摄像头。
- home_zone_vertices 仍为注明占位的0.6m方框，full_projection_radius_m 需要按包括收回机械臂在内的实际投影标定。
- 完整随机障碍布局、两端真人双向通话和急停效果需要实机跑全程。用户已确认底盘路径规划联通，本次不改该实现。

任务当前仍有180秒软件总超时、单阶段30秒默认超时。这些是项目配置，不据此宣称规则规定了180秒全程限时；现场测量/交流耗时较长时，通过 mission_config 调整。

实时场地位置图尚未加入；可以在现有 GUI 读取 /odom、场地边界与路径，显示车辆估计位置、朝向和轨迹。该图反映控制器的定位估计，不会自行校正里程计漂移。

## 验证

本次 Windows 离线验证共69项通过：音视频与完整任务 launch 回归25项、床旁行为8项、体征协议/GUI18项、单CAN默认5项、Ready生命周期1项、扫码策略7项、按钮启动脚本5项；34个launch均通过XML解析。

其中 launch 回归使用本地缓存的 ROS roslaunch/xmlloader、参数加载与替换模块进行离线解析，适配了本地包路径，没有启动 ROS Master、任何 ROS 节点或硬件。另做9组配置的独立静态展开，确认节点无重复及串口/话题/全屏覆盖。

没有在此Windows机器执行catkin构建或ROS运行时集成测试。已增加的回归在Ubuntu/CATKIN_ENABLE_TESTING环境会被运行。现有真实状态机rostest包含11/13/31/33四组合、放药事件与取消流程，仍应在目标Ubuntu运行。

```bash
catkin_make run_tests_robot_navigation
catkin_test_results
```
