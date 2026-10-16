# 音视频低积压版本：部署与验收

## 兼容性
两台设备必须同时更新、重新编译。音频 AV01 包不兼容旧裸 Opus 包。
默认 mono/48kHz/20ms；不改 default 虚拟音频设备。
音频仍使用 pacat/PulseAudio，不是 ALSA/GStreamer 新后端。
请求的 capture/playback latency 不是实际声卡延迟。
队列最多保留 max_queue_ms 毫秒，迟到/重复/播放过的包丢弃。
采样时间戳取读取 PCM 的时刻，不包含虚拟声卡此前延迟。
相对时间偏差估计不要求两台系统时钟同步，但不能检测恒定的端到端基线延迟。
这是有限抖动吸收队列，不是完整自适应 RTP jitter buffer，也没有 AEC。
视频保留 MJPEG：usb_cam 本地解码，发送节点 JPEG 编码后跨网络，接收节点解码显示。
不是摄像头原生 MJPEG 零拷贝，也没有 H.264。

## 树莓派双摄像头配置

共享配置文件：`config/camera_devices.launch`。

当前树莓派 `v4l2-ctl --list-devices` 对应关系：

```text
扫码摄像头：/dev/video0，备用 /dev/video1，发现设备 /dev/media2
音视频摄像头：/dev/video2，备用 /dev/video3，发现设备 /dev/media3
```

`usb_cam` 应使用 `/dev/video*`，不使用 `/dev/media*`。默认启动时：

```bash
# 树莓派扫码
roslaunch robot_navigation scan_and_display.launch

# 树莓派机器人端音视频，使用 /dev/video2
roslaunch robot_navigation audio_video_master.launch
```

如果摄像头编号变化，只修改 `camera_devices.launch` 中的：

```xml
<arg name="scan_device" default="/dev/video0"/>
<arg name="robot_audio_device" default="/dev/video2"/>
```

也可以临时覆盖，不改文件：

```bash
roslaunch robot_navigation scan_and_display.launch video_device:=/dev/video1
roslaunch robot_navigation audio_video_master.launch lower_cam_device:=/dev/video3
```

电脑 VM 是医生端，默认选择 VM 自己的 `/dev/video0`，不会读取树莓派的 `/dev/video2`。
如果 VM 没有摄像头，使用 `start_upper_cam:=false`，仍可接收树莓派视频。

## 单命令启动与集中配置（树莓派 + 医生端 VM）

角色已按当前部署统一：`audio_video_master.launch` 是树莓派机器人端，
`audio_video_slave.launch` 是医生端 VM。这里的音视频角色与 ROS Master 是两件事；
ROS Master 仍运行在树莓派。

默认参数集中在包内的 `config/audio_video_defaults.launch`。它是 ROS 原生 XML
参数配置文件，不是节点 YAML；这样相机/音频开关在创建节点前就能生效，且保留
原来的 `enable_video:=false` 等命令行覆盖功能。两端共用
`launch/audio_video_peer.launch` 实现，原有节点名、相机命名空间及跨网话题保持不变。
默认视频为 640x480@25/MJPEG、音频为 48kHz/mono/20ms、UDP 5004；HTTP 默认关闭。

树莓派摄像头路径统一由 `config/camera_devices.launch` 管理：扫码默认 `/dev/video0`（备用 `/dev/video1`），机器人端音视频默认 `/dev/video2`（备用 `/dev/video3`）。`/dev/media2` 和 `/dev/media3` 只用于 v4l2 设备发现，不传给 `usb_cam`。电脑 VM 医生端默认使用自己的 `/dev/video0`；没有本地摄像头时启动 `start_upper_cam:=false`。换树莓派摄像头编号时只改该文件的 `scan_device` 或 `robot_audio_device`；启动命令仍可用 `scan_camera_device:=`、`lower_cam_device:=` 或 `upper_cam_device:=` 临时覆盖。

两边 `.bashrc` 的 ROS 部分先 source Noetic 和实际工作空间，再设置网络变量。
当前部署默认地址为树莓派 `10.161.170.93`、VM `10.161.170.41`；地址变更时更新
`config/audio_video_defaults.launch` 中的 `local_ip`/`remote_ip` 默认表达式，或在命令行覆盖：

```bash
# 树莓派
export ROBOT_PI_IP=10.161.170.93
export ROBOT_VM_IP=10.161.170.41
unset ROS_HOSTNAME
export ROS_MASTER_URI="http://${ROBOT_PI_IP}:11311"
export ROS_IP="${ROBOT_PI_IP}"
```

```bash
# 医生端 VM
export ROBOT_PI_IP=10.161.170.93
export ROBOT_VM_IP=10.161.170.41
unset ROS_HOSTNAME
export ROS_MASTER_URI="http://${ROBOT_PI_IP}:11311"
export ROS_IP="${ROBOT_VM_IP}"
```

角色默认直接设置本机/对端 IP，不必在 roslaunch 命令后追加 `local_ip`、`remote_ip`。
每个终端仍须设置正确的 `ROS_MASTER_URI` 和本机 `ROS_IP`，并 `source ~/.bashrc`；
改网络后停止旧节点、更新地址再重启。`master_uri/local_ip/remote_ip` 仍可临时覆盖；
launch 内的 `<env>` 不能搬迁 roslaunch 自身使用的 Master。

```bash
# 树莓派终端 1：仅此一处启动 ROS Master
roscore
# 树莓派终端 2
roslaunch robot_navigation audio_video_master.launch
# 医生端 VM
roslaunch robot_navigation audio_video_slave.launch
# 任一端的新终端：等两边节点启动完成再开启通话
rostopic pub -1 /Ready std_msgs/Int32 'data: 1'
# 停止通话
rostopic pub -1 /Ready std_msgs/Int32 'data: 0'
```

树莓派端的最终状态 UI 由 `audio_video_master.launch` 默认启动，显示医生端视频、任务状态、条码、体征和相机状态。无桌面或只做后台测试时使用 `enable_status_gui:=false`。不要同时启动它和 `mission_complete.launch` 的整车 UI；完整比赛流程应只启动 `mission_complete.launch`。

视频回传分段检查：在任意已连接 ROS Master 的终端执行：

```bash
# VM 医生端本地采集和发送
rostopic hz /upper_cam/upper_usb_cam/image_raw
rostopic hz /upper_cam/network/compressed

# Pi 机器人端接收和最终显示
rostopic info /upper_cam/network/compressed
rostopic hz /camera/upper_display
rosnode info /video_stream_master
```

如果 VM 的两个 `rostopic hz` 都没有数据，检查 VM 的 `/dev/video0` 和 `rosparam get /upper_cam/upper_usb_cam/video_device`；如果 VM 发送话题正常但 Pi 的 `/camera/upper_display` 没数据，检查 `video_stream_master` 的订阅连接及两端是否已经同步更新并重启。

这只简化音视频参数，不自动启动完整机器人任务，也不自动发布 `/Ready=1`。
不要因此同时启动 `mission_complete.launch` 的音频和另一套音视频节点。
没有桌面时，将配置中的 `enable_display` 默认值改成 `false`，如需网页接收再开启
`enable_http`。只有一侧有摄像头时，关闭无摄像头侧的 `start_upper_cam` 或
`start_lower_cam`；无真实摄像头测试可将 `use_test_camera` 改为 `true`。

改设置时编辑对应 `<arg name="..." default="..." />`，无需改节点实现。
例如仅测试音频仍可临时使用：

```bash
roslaunch robot_navigation audio_video_slave.launch enable_video:=false
roslaunch robot_navigation audio_video_master.launch enable_video:=false
```

要保存另一套配置，复制默认配置文件并传
`config_file:=/absolute/path/to/audio_video_defaults.launch`。更新两端的包文件后，
在 devel 工作空间下仅改 XML 不需要重新编译 C++；如果使用 install 工作空间，
需重新执行 `catkin_make install` 将 launch/config 安装更新。

## 两台 VM 编译
```bash
cd ~/ROS_WS-2/ROS_WS
sudo apt install libopus-dev pulseaudio-utils pkg-config
catkin_make
source devel/setup.bash
catkin_make tests
ctest --test-dir build -R audio_packet_queue --output-on-failure
```
先关闭旧 launch。只启动一个 roscore。终端设置正确 ROS_MASTER_URI 和 ROS_IP。

## 纯音频（两端）
```bash
# 树莓派机器人端
roslaunch robot_navigation audio_video_master.launch enable_video:=false
# 医生端 VM
roslaunch robot_navigation audio_video_slave.launch enable_video:=false
# 已 source 且连接同一个 master 的第三个终端
rostopic pub -1 /Ready std_msgs/Int32 'data: 1'
# 停止
rostopic pub -1 /Ready std_msgs/Int32 'data: 0'
```
如声音断续先两端增大 playback_latency_ms:=60 jitter_buffer_ms:=60 max_queue_ms:=140。
没有欠载时再尝试 playback_latency_ms:=20 jitter_buffer_ms:=20 max_queue_ms:=80。
每次只改一组参数，重启 launch，不要把缓存越小当成必然越好。
frame_duration_ms 如果修改，两端必须一致（仅支持10或20）。

## 仅有一个摄像头，接 VM1
```bash
# 医生端 VM（相机），角色 slave
roslaunch robot_navigation audio_video_slave.launch enable_image_view:=false
# 树莓派机器人端（无本地相机时关闭 lower 摄像头采集），仍接收并显示医生视频
roslaunch robot_navigation audio_video_master.launch start_lower_cam:=false
```
默认已改 cam_pixel_format=mjpeg；640x480@25。视频单独测试两端加 enable_audio:=false。
浏览器代替弹窗：接收端 enable_display:=false enable_http:=true，访问 http://接收端IP:8080/stream。
enable_browser 仅保留8081兼容入口；通常不要开第二条输出。

## 观测
医生端 VM 本地相机：/upper_cam/upper_usb_cam/image_raw
跨网络压缩：/upper_cam/network/compressed
树莓派机器人屏显示医生端视频：/camera/upper_display
只有VM1有真实相机，不应拿lower_cam的25Hz作VM1相机证据。
不要在远端额外订阅原始图像测速，否则又会产生原图跨网负载。
日志含处理fps、max_work；音频含丢包/队列/播放写入耗时，不是端到端延迟。

## 验收
- 音频单向、反向、双向各5分钟；记录特征点实测延迟和断续次数。
- 摄像头断开、/Ready 0/1十次、Ctrl+C可退出，后台pacat不残留。
- 用同一外部录音/视频记录声光事件，避免直接相减未同步的VM时间。
- 记录视频实际新画面变化，rostopic hz只代表消息频率，可能含重复帧。
- 比较纯音频和音视频同时运行。宿主机/虚拟音频固定延迟不受应用队列上限保证。
- 未做硬件端到端实测，不承诺低于某一毫秒值。
