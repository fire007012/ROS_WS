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

共享路由在config/camera_devices.launch。默认对调依据用户反馈：原/视频0实际是通话摄像头。Linux设备编号可能随插拔变化，必要时用启动参数指定实际设备，建议使用可用的稳定by-id路径；video1/3可能只是同一设备的另一接口，不能直接认定是另一只摄像头。

```bash
# Pi：先在工作空间编译并加载环境
catkin_make
source devel/setup.bash
roslaunch robot_navigation mission_complete.launch \
  scan_camera_device:=/dev/video2 \
  call_camera_device:=/dev/video0

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
roslaunch robot_navigation scan_and_display.launch video_device:=/dev/video2
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
