# 放药释放事件与语音启动

`/arm_place_medicine` 的服务成功响应与 `/medicine/placed` 是两个不同条件。
事件为 `std_msgs/Int8`：data=1/3 表示释放药品的床号。消息不是药箱号，也不是
`/medicine_release_done`（整个机械臂流程完成的 Bool）。任务收到匹配床号的
释放事件才调用 `/speak`，使语音可与机械臂回位并行；机械臂服务成功且语音成功
后才进入床旁流程，床旁完成后才解锁底盘继续导航。

真实发布点位于 `arm_and_gripper/src/arm_and_gripper_node.cpp` 的
`executePlaceSequence()`：机械臂就位、对应药箱开箱指令 ACK 成功、释放等待结束
后发布事件，再关箱和回位。ACK 或等待失败/取消时不伪造释放事件。事件表示
ACK 加定时流程的释放点，尚无药品实际落下的传感器反馈。

此前失败有以下原因：

1. `test_mission_routes.py`、`test_mission_cancellation.py`、
   `test_mission_closed_loop.py` 的模拟放药服务只返回成功，没有模拟释放事件。
   所以 `speech_started_` 始终为 false；旧状态机在语音入口立即失败。
   路线/闭环夹具还缺少后续 `/bedside/complete` 服务。
2. 旧 `medicinePlacedCallback()` 仅在 `PLACE_MEDICINE_A/B` 接受事件。
   服务响应和话题走不同连接，响应先到时状态已经变成 `VOICE_A/B`，稍后到达的
   合法释放事件会被拒收。新顺序测试可复现该问题。
3. 任务取消后，重启仅检查旧动作服务，未检查并行的旧语音服务。
   现已等待旧语音服务结束并丢弃结果，再允许显式启动新任务。

现在在发起本轮机械臂服务前登记预期床号和任务代次。允许在放药状态及服务成功
后的语音等待状态接收一次匹配事件；重复、错误床号、超时、取消后或其他状态中的
事件不会启动语音。服务响应先到时，底盘保持锁定并等待事件，默认最多 1 秒。
使用单调时钟计算事件接收期限，超时进入失败停车状态，日志明确给出缺失的话题
和预期床号。语音失败或机械臂失败也保持停车。

配置位于 `config/mission_params.yaml`：

```yaml
mission:
  medicine_placed_event_timeout_sec: 1.0
```

该参数是“服务成功响应之后”的额外事件等待期限；放药服务执行期间仍受原任务
阶段/全局超时保护。参数须为正且有限。

排查实际接入时可同时观察：

```bash
rostopic echo /mission/state
rostopic echo /medicine/placed
rostopic echo /chassis_lock
rosservice info /arm_place_medicine
rosservice info /speak
```

完整任务应调用 `/arm_place_medicine` 并传 bed_id=1/3、box_id=1/3。
旧 `/place_medicine` 别名没有床号，不能作为完整任务的放药事件来源。
若事件未发布，检查机械臂/药箱 ACK、释放等待、取消及急停日志；若事件已发布，
检查床号、当前任务阶段、等待期限和 `/speak` 的服务结果。
不要把手工发布事件当作真实药品释放的验证。

本次修改：任务状态机源码/头文件、任务参数、三个旧测试夹具、CMake 测试注册，
新增 `mission_medicine_event.test`/`test_mission_medicine_event.py`，以及
机械臂原有测试中的实际发布点验证。

ROS Noetic 编译通过，以下六套测试共 11 项测试通过：

- `mission_routes.test`：四种床号/药箱组合、正确语音床号、回归保持与缺失路线停车。
- `mission_cancellation.test`：慢放药/语音服务取消、旧服务重启门控、阶段/全局超时。
- `mission_closed_loop.test`：两条完整路线通过真实导航/速度/安全控制链路。
- `mission_medicine_event.test`：事件先到/响应先到、两个床号、重复/错误床号、缺失、
  超时后/取消后事件、语音失败、机械臂失败，验证底盘保持锁定。
- `arm_cancellation.test`：6 项，包含真实机械臂控制代码在两种药箱上发布床号事件、
  在回位完成前启动事件，以及取消/ACK 失败保护。
- `mission_upper_alignment.test`：原横移阶段入口回归。

所有机械臂传输、ACK、语音与底盘运动均为模拟测试，没有连接真实 CAN 或执行
实车放药/横移。
