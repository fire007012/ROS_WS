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
# VM1
roslaunch robot_navigation audio_video_master.launch enable_video:=false
# VM2
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
# VM1（相机），关闭本机接收显示和HTTP，因为VM2没有相机
roslaunch robot_navigation audio_video_master.launch enable_display:=false enable_http:=false
# VM2（看VM1），不启动相机，仍接收压缩视频
roslaunch robot_navigation audio_video_slave.launch start_lower_cam:=false enable_http:=false
```
默认已改 cam_pixel_format=mjpeg；720p25。视频单独测试两端加 enable_audio:=false。
浏览器代替弹窗：接收端 enable_display:=false enable_http:=true，访问 http://接收端IP:8080/stream。
enable_browser 仅保留8081兼容入口；通常不要开第二条输出。

## 观测
VM1 本地相机：/upper_cam/upper_usb_cam/image_raw
跨网络压缩：/upper_cam/network/compressed
VM2 本地显示：/camera/upper_display
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
