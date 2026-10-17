# 全流程启动、通话与显示（2026-10-17更新）

当前入口：`roslaunch robot_navigation mission_complete.launch`。
本次依据实机反馈调整摄像头、持续音视频、停靠阶段和独立体征显示。最新行为以本篇为准。

## 默认行为

1. 启动launch即打开机器人通话摄像头与音频；VM运行audio_video_slave.launch。两端更新后，无需PC1或任务Ready即可通话，任务完成或急停也不会关闭通话。
2. ESP32桥接启动后随时接收、显示心率体温，不等待到病床；体征无效或不测量不会阻止离床。
3. PC1只开始机器人任务：前往护士台→二维码决定首床/首箱→床前路径停靠→车体入圈和前向测距检查→床头条形码→指定药箱放药与播报→床旁音视频交流确认→另一床→回归。
4. 二维码与条形码始终用同一个扫码摄像头、同一个 `/usb_cam/image_raw`。右侧始终保留医生视频；左下角只在扫码阶段显示独立取景，退出扫码便隐藏。
5. 位置图继续显示 `/odom` 估计位置、朝向、轨迹及当前目标；不是独立定位系统，也不推测未观测障碍。

## 摄像头角色（按本次实机反馈修正）

| 机器/用途 | 默认设备 | 输出 |
| --- | --- | --- |
| Pi 扫码（二维码+条形码） | /dev/video2 | /usb_cam/image_raw |
| Pi 通话摄像头 | /dev/video0 | /lower_cam/lower_usb_cam/image_raw → /lower_cam/network/compressed |
| VM 医生摄像头 | /dev/video0（VM本机） | /upper_cam/upper_usb_cam/image_raw → /upper_cam/network/compressed |

摄像头默认值统一在 `config/camera_devices.launch` 编辑：

```xml
<!-- scan_camera_device 对应此项，二维码与条形码共用 -->
<arg name="scan_device" default="/dev/video2"/>
<!-- Pi 的 call_camera_device 对应此项 -->
<arg name="robot_audio_device" default="/dev/video0"/>
<!-- VM 医生相机 -->
<arg name="doctor_audio_device" default="/dev/video0"/>
```

Linux设备编号可能随插拔变化，设备变化时改这个配置文件即可，不需要改启动命令。可填写可用的稳定by-id路径；video1/3可能只是同一设备的另一接口，不能直接认定是另一只摄像头。

```bash
# Pi：先在工作空间编译并加载环境
catkin_make
source devel/setup.bash
roslaunch robot_navigation mission_complete.launch

# VM：同步本次代码，编译并加载环境后运行
roslaunch robot_navigation audio_video_slave.launch
```

不要同时另开Pi的audio_video_master.launch或默认status_display.launch；全流程已包含它们的相关节点。
默认IP沿用本机10.161.170.93、VM10.161.170.41；本次没有改变网络配置。两端必须使用一致、可访问的ROS_MASTER_URI，并正确声明各自ROS_IP。

## 床前一直显示“微调”的处理

与用户指定的1b9c8eac18c935de9dd090112704057a3f4ae92d相比，fine_tuning控制器、250mm目标以及底盘路径/规划/里程计/避障实现均没有变化。旧的完整任务会在路径到达后再执行测距微调，再校验车体投影；界面原来把整个阶段统称为微调，无法区分具体等待项。

默认改为enable_bedside_distance_tuning=false：采用已验证路径的停靠位姿，不再强制执行额外的250mm前向修正，但仍校验新鲜里程计、整车底盘入圈和有效前向测距。没有放宽圈半径、车体尺寸或安全条件。若定位与圈坐标不一致，仍会停止等待并给出明确提示，不会自动开箱。

车体半径默认0.25m、圆圈半径0.3m、余量0.02m，因此中心误差必须不超过0.03m；旧微调最多可移动0.05m，二者可能冲突。这是可确认的配置风险，未收到这次床旁日志，不能断言其为此次唯一根因。

如已标定传感器安装/床柜位置，可以显式恢复额外测距修正：

```bash
roslaunch robot_navigation mission_complete.launch enable_bedside_distance_tuning:=true
```

GUI会具体显示“等待前向测距有效且新鲜”“等待车体完整进入停靠圈”等。若仍不通过，请记录：

```bash
rostopic echo -n 1 /mission/state
rostopic echo -n 1 /fine_tuning/status
rostopic echo -n 1 /front/range
rostopic echo -n 1 /odom
```

同时保留 `[mission_controller] 投影校验未通过` 的center_error和usable_radius值。默认路径停靠模式没有启动fine_tuning/status更新时，其旧值不能代表当前正在微调，应以mission/state和界面入圈等待提示为准。

## 音视频全程开启与连通检查

两端always_on默认true。新增audio_video_ready_node只发布媒体专用 `/audio_video/ready=1`（锁存），不会启动底盘。所有音视频组件使用该媒体话题，摄像头不再等待PC1；任务节点的旧 `/Ready=0/1` 不影响媒体。

两端必须同步本次更改并编译，尤其新增脚本需经过catkin安装/包装。单独需要兼容旧Ready门控时，可以对音视频wrapper传always_on:=false。

ping通只说明IP层可达，不证明UDP5004音频、ROS节点反向连接和视频话题可达。Pi和VM各检查：

```bash
rostopic echo -n 1 /audio_video/ready
rostopic echo -n 1 /audio_chat/master/active
rostopic echo -n 1 /audio_chat/slave/active
rostopic echo -n 1 /audio_chat/master/peer_ready
rostopic echo -n 1 /audio_chat/slave/peer_ready
rostopic hz /upper_cam/network/compressed
rostopic hz /lower_cam/network/compressed
```

active表示本地音频线程已运行；peer_ready表示本机收到了有效对端Opus数据。GUI会区分“尚未启动/启动失败”“运行中但未收到医生端有效UDP”“本机运行/已接收医生音频”。若无UDP，需保留VM音频日志，检查默认声卡/麦克风及UDP5004、防火墙、虚拟机网络和remote_ip；本次不擅自修改网络。

## 心率体温独立测量

ESP32当前VITALS_AUTO_START=true，启动后连续采集；USB桥接和GUI持续显示有效数值。没有增加ROS向ESP32下发启停的指令。enable_patient_measurement=true控制是否启动桥接；require_bedside_measurement=false控制床旁服务不等待体征，二者已分开。

床旁仍保留默认5秒音视频接收确认，以支持两床交流流程；不再要求体征连续有效2秒才能继续。不测量、手指离开或体征暂时无效，仅影响显示，不影响任务推进。

## 显示与单独扫码

当前任务按二维码显示实际床号、药箱号，条码独立留存。扫码阶段左下小窗显示真实取景，右侧医生画面不切换。PPM图像直接以二进制交给Tk，修复旧版Base64导致的couldn't recognize image data。

退出全流程后，可只测试扫码，不启动底盘/机械臂：

```bash
roslaunch robot_navigation scan_and_display.launch
```

单独查看显示、避免重复占用已运行桥接：

```bash
roslaunch robot_navigation status_display.launch enable_vitals_bridge:=false fullscreen:=false
```

show_map:=false隐藏位置图。arena_config和paths_file控制场地示意及停靠点；全流程使用与路径管理器相同的paths_file。

## 急停和剩余实机验收

PC1不解除锁存急停。确认物理急停松开、故障修复后：

```bash
rosservice call /system_health/reset "{}"
rostopic echo -n 1 /robot_state
```

服务成功只表示发出了复位请求；还要确认底盘接受并回到CACHE_READY。恢复后PC1开始新一轮任务，不是断点续跑；调试时优先回起点重新启动以保持里程计原点一致。

药品落入直径100mm圆、体征精度、整车入圈、回归白框与真实双向通话仍需实机验收。测距微调默认停用不等于已证明车体合规；home_zone_vertices等占位尺寸仍需实测。软件180秒任务总超时为项目配置，不据此解释为比赛规定的全程时限。

## 验证范围

新增ROS回归fixture mission_waypoint_position.test，用真实任务节点验证四种二维码顺序、未入圈不放药、路径停靠模式不调用测距服务；须在Ubuntu构建后运行。离线还编译运行了从当前源码提取的停靠入口/完成判定C++函数，验证关闭距离修正时不调用服务、投影/前向测距失败时不推进、合格后推进以及开启修正时仍等待完成。

本次离线解析检查不能代替Ubuntu全量catkin构建或实机UDP、摄像头、测距验收。请在目标环境运行：

```bash
catkin_make run_tests_robot_navigation
catkin_test_results
```


## 2026-10-17：横向视频、六路测距与寻路回归排查

本次基线为当前1aa44a3；与1b9c8ea和95532a1比较，CAN/规划/路径/里程计/微调/避障算法和路径坐标没有改变。使用实际roslaunch解析旧版与当前mission_complete.launch，底盘相关101个生效参数完全相同。不能据此把实机“起步横移后停住”断言为某项路径参数被改，也没有证据宣称已修复该实机故障。

新增的视频、GUI、体征工作会增加整机运行负载，本次限制OpenCV线程为1，采集保留640×480@25，但发送及接收处理降为10Hz、GUI显示10Hz、地图4Hz。该调整减少媒体处理争用，是缓解措施；尚无这次实机CPU/延迟日志证明资源争用就是根因。速度、避障距离、测距有效性和急停门槛均未放宽。

### 横向画面

画面输出默认横向640×480（4:3），保留原比例并用黑边填充，不拉伸人物。所有默认尺寸集中在 `config/audio_video_defaults.launch`：

```xml
<!-- 摄像头采集宽、高 -->
<arg name="cam_width" default="640" />
<arg name="cam_height" default="480" />
<!-- 发送、接收输出宽、高：640×480是4:3；改成640×360就是16:9 -->
<arg name="stream_width" default="640" />
<arg name="stream_height" default="480" />
<arg name="stream_framerate" default="10.0" />
```

Pi和VM同步这个配置文件后，分别用不带附加参数的原命令启动即可。全流程入口不再写死输出宽高来覆盖本文件；GUI会自动读取接收节点配置的宽高，避免画面已改16:9而GUI仍固定4:3。`status_display.yaml`不再重复维护比例。

若摄像头确实是竖向源，默认保持人物直立、两侧留黑。GUI的 `rotate_portrait_video` 仍可在 `config/status_display.yaml` 中设置；不要为凑横屏盲目旋转已直立的人物。

### 只有声音没有画面

音频UDP通不能证明ROS/TCP视频通。Pi GUI增加接收链路判断：没收到医生压缩视频、医生端无信号帧、有压缩视频但显示节点无输出、Tk渲染错误。GUI不额外订阅远程视频来诊断，复用视频接收节点每秒发布的轻量JSON状态，避免重复网络流量。

发送节点 /video_sender_master/status 和 /video_sender_slave/status、接收节点 /video_stream_master/status 和 /video_stream_slave/status 提供input_topic、input_age_s、signal_age_s、received_frames、decode_errors、no_signal、ready。负age表示从未收到。重点先看VM的video_sender_slave/status有无真实采集，再看Pi的video_stream_master/status有无收到。

黑帧现在携带no_signal标记；不再把“等待摄像头”黑帧当成真实画面或床旁通话成功。

### 起步横移/床前卡住

横移可能是RangeDetour根据前向距离触发SIDESTEP；只有日志才能确认。path_tracker在绕障状态变化时记录pose、target、四向车体边缘净空、有效位和数据年龄；GUI同时显示路径状态、选中运动源、距离安全状态。接受PC1时记录当前里程计起点，如不在P0则提示。手动搬动车辆不会重置里程计，重试也不是断点续跑。

**没有**禁用避障，也没有把无效距离当成通路。默认仍使用已跑通路径停靠，enable_bedside_distance_tuning=false，不再强制向250mm目标移动。车体投影或前向测距校验未通过时仍拒绝开箱。

左下固定显示6路VL53：前、左、右、后、臂左、臂右，单位mm，失效显示“无效”，超过0.5秒显示“已过期”，无消息显示“未收到”。显示的是原始传感器距离；日志中的body_clearance是减去安装内缩后的车体净空，两者不同。

若当前仍不能行至护士台，先停止本轮并保留终端中失败前后的日志；不要用反复按PC1清除急停。可在运行launch期间另开终端收集只读快照（不发运动、不复位）：

```bash
rosrun robot_navigation diagnose_mission.py --output ~/mission_diagnostics.txt
```

脚本仅并行读取话题、节点信息与ROS网络地址，每项最多等4秒；不是同一时间戳的同步录制。需要把任务从PC1启动直到自动急停的一段终端日志和这个快照一起检查，才能确认到底是绕障STALLED、传感器失效、命令过期、CAN故障还是阶段超时。Pi和VM都更新并编译本次代码后再比较视频状态。

本次89项Python/launch离线回归通过，横向视频尺寸C++断言编译运行通过；Tk布局检查含六路距离和扫码窗口。完整ROS C++视频节点尚需Ubuntu catkin编译。没有实机视频或导航跑通结论。


## 配置文件默认值回归验证

2026-10-17新增回归：在临时工程中只编辑camera_devices.launch的三路设备默认值，以及audio_video_defaults.launch的输出高度，使用不带设备/尺寸参数的完整任务和VM launch进行真实roslaunch离线解析，验证两个摄像头、VM摄像头、发送和接收尺寸全部跟随配置。另验证GUI在master/slave角色下都自动采用接收节点的16:9比例。

这次针对配置默认值调整的launch回归30项、GUI/体征回归27项通过；没有连接硬件或执行实际摄像头采集。
