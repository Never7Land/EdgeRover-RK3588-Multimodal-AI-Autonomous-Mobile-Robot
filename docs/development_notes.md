# EdgeRover 开发笔记（关键技术决策）

> 更新：2026-11-21。只记录今后开发会再用到的结论；普通代码改动不记。
> 接口细节见 [`interfaces.md`](interfaces.md)，架构见 [`architecture.md`](architecture.md)。

**状态标记**

| 标记 | 含义 |
|---|---|
| **[已解决]** | 问题已处理并实测 |
| **[已验证]** | 实测确认 |
| **[依据代码]** | 只读代码得出，未实测 |
| **[待验证]** | 需上板确认 |
| **[未解决]** | 已知缺口 |

**目录**

1. [RK3588 系统与 SDK 环境](#1-rk3588-系统与-sdk-环境)
2. [ROS2 工作空间与构建部署](#2-ros2-工作空间与构建部署)
3. [运行库版本与兼容性注意事项](#3-运行库版本与兼容性注意事项)
4. [摄像头、共享内存、ZMQ 的技术选型](#4-摄像头共享内存zmq-的技术选型)
5. [已经解决的关键问题](#5-已经解决的关键问题)
6. [已经验证失败的方案](#6-已经验证失败的方案)
7. [尚未解决的技术问题](#7-尚未解决的技术问题)

---

## 1. RK3588 系统与 SDK 环境

| 项 | 结论 | 状态 |
|---|---|---|
| 平台 | RK3588 开发板，用户 `lyfan`，主机名 `rk3588` | [已验证] |
| ROS2 | Humble，`/opt/ros/humble` | [已验证] |
| Ubuntu 版本 | Ubuntu 22.04.5 LTS，内核 6.12.69 aarch64（SSH 登录提示） | [已验证] 2026-11-20 |
| Docker | 板上**没有安装**（`docker: command not found`） | [已验证] |
| NPU 运行时 | librknnrt（YOLO、VLM 视觉编码器）+ librkllmrt（LLM） | [依据代码] |
| 2D 加速 | librga（RGA） | [依据代码] |

确认版本时在板上执行以下命令：

```bash
lsb_release -a
strings /usr/lib/librknnrt.so | grep -i "librknnrt version"
strings ~/edge_rover/ros2_workspace/src/edgerover_vlm/third_party/lib/librknnrt.so | grep -i "librknnrt version"
strings ~/edge_rover/ros2_workspace/src/edgerover_vlm/third_party/lib/librkllmrt.so | grep -i version
dpkg -l | grep -E 'librga|libzmq|nlohmann|opencv'
```

---

## 2. ROS2 工作空间与构建部署

### 2.1 目录

| 路径（板上） | 内容 |
|---|---|
| `~/edge_rover/ros2_workspace` | colcon 工作空间。自研包在 `src/`，第三方包在 `src/third_party/` |
| `~/edge_rover/model/yolov5s_rk3588.rknn` | YOLO 模型 |
| `~/edge_rover/model/qwen3vl/qwen3-vl-2b_vision_rk3588.rknn` | VLM 视觉编码器 |
| `~/edge_rover/model/qwen3vl/qwen3-vl-2b-instruct_w8a8_rk3588.rkllm` | VLM 语言模型 |

Windows 侧挂载：`Z:\edge_rover` 对应板上的 `~/edge_rover`。

### 2.2 构建

- 在板上原生用 colcon 构建，没有交叉编译。
- supervisor 的默认路径是 `$HOME/edge_rover/ros2_workspace/install` 和 `$HOME/edge_rover/model`，两者都可以用参数覆盖。改路径时要同步修改 supervisor 参数。[依据代码]

### 2.3 部署与自启

| 组件 | 方式 | 状态 |
|---|---|---|
| 视觉三进程 | `edgerover.service` → supervisor → camera / yolo / vlm | [已验证] 重启后自启 |
| ros_bridge | `edgerover-ros-bridge.service`（`User=lyfan`，`After=edgerover.service`）→ `ros2 launch`，respawn 2 s | [已验证] 重启后自启 |
| micro_ros_agent | 工作空间内原生编译（`src/third_party/micro-ROS-Agent`）；`edgerover-uros-agent.service`（`User=lyfan`，`Restart=always`）→ `ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4` | [已验证] 2026-11-20 重启后自启，底盘自动连上（`session established`）；能断线重连 |
| TF 树 | `edgerover-tf.service`（`User=lyfan`）→ `ros2 launch pkg_edgerover_odom_tf tf_bringup.launch.py` | [已验证] 2026-11-20 重启后 `view_frames` 完整，`odom → base_footprint` 约 43 Hz |
| systemd 单元文件 | 仓库副本在 `deploy/systemd/`（4 个：`edgerover`、`edgerover-ros-bridge` 为已安装单元的拷贝；`edgerover-uros-agent`、`edgerover-tf` 为新建）；安装位置 `/etc/systemd/system/` | [已验证] 已纳入仓库；[已验证] 2026-11-20 `sudo diff` 核对 `/etc` 下安装版本：`edgerover-tf.service` 仅注释行不同，`edgerover-uros-agent.service` 完全一致，无需重装 |

手动启动 micro_ros_agent（调试用；自启服务在运行时先 `sudo systemctl stop edgerover-uros-agent`，避免端口 8888 冲突）：

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v4
```

注意事项：

- `After=` 只约束启动顺序。bridge 会自行重试 FrameRing 和 ZMQ 的连接，所以不依赖视觉进程先就绪。[依据代码]
- 底盘连接的 agent 地址在底盘网页里配置，保存在 ESP32 的 NVS 中，默认 `192.168.31.214:8888`。RK3588 的 IP 变了要去网页重设。[依据代码]

---

## 3. 运行库版本与兼容性注意事项

| 库 | 链接方式 | 注意 |
|---|---|---|
| librga | perception 链接系统库 `/usr/lib/aarch64-linux-gnu/librga.so.2`；头文件随包放在 `edgerover_perception/third_party/rga/include`（`include/` 和 `include/rga/` 两份） | 头文件与系统 .so 的版本一致性未核对 [待验证]。换系统镜像或升级 librga 后先检查 RGA 调用是否报错 |
| librknnrt（YOLO） | `yolo_proc` 链接系统库 `/usr/lib/librknnrt.so` | — |
| librknnrt / librkllmrt（VLM） | `edgerover_vlm/third_party/lib/` 自带一份 .so 和头文件（`rkllm.h`、`rknn_api.h`、`rknn_custom_op.h`、`rknn_matmul_api.h`） | **系统和自带的 librknnrt 有两份**，两个进程可能加载不同版本 [待验证]。`.rknn` / `.rkllm` 模型必须与运行时版本匹配，升级运行时后要重新转换模型 |
| 头文件版本宏 | 现有头文件里没有可读的版本宏 | 只能用 `strings` 查 .so |
| 其他 | libzmq、nlohmann_json（bridge）、OpenCV core/imgproc/imgcodecs（仅 `frame_dump` 调试工具） | 版本未记录 [待验证] |

---

## 4. 摄像头、共享内存、ZMQ 的技术选型

### 4.1 摄像头

- `/dev/video44`，IMX415，3840x2160 NV12 采集。[已验证] 正常运行
- 用 RGA 做 letterbox 缩放并转色，输出 640x640 RGB888。缩放和转色都在 RGA 上完成，不占 CPU。[依据代码]
- 检测坐标基于 letterbox 640。帧里附带原图尺寸，下游需要时自行反算。

### 4.2 共享内存 FrameRing

- 名称 `/edgerover_frames`，4 个 slot，seqlock，**单写多读**（camera 写，yolo / vlm / bridge 读）。
- 选型理由：图像数据量大，避免拷贝和序列化；读方读到撕裂帧（`kTorn`）时直接重读，不加锁，写方不被阻塞。
- 读方要自己处理写方重启：`vlm_proc` 打不开共享内存时每 1 s 重试；`ros_bridge` 超过 3 s 没有新帧就重新打开。[依据代码]

### 4.3 ZMQ（`ipc://`）

| 端点 | 模式 | 用途 |
|---|---|---|
| `ipc:///tmp/edgerover_det` | PUB/SUB，HWM=2 | 检测 JSON |
| `ipc:///tmp/edgerover_vlm` | REQ/REP | VLM 问答 |
| `ipc:///tmp/edgerover_state` | PUB/SUB | VLM 忙闲状态 |

- 选型理由：视觉进程**不依赖 ROS**，可以单独运行调试；只有 bridge 把它们接入 ROS2。
- VLM 回复的第一行格式如下，排查时先看这一行：

  ```text
  OK frame=.. age=.. enc=.. ttft=.. tokens=.. tps=.. total=..
  ```

- 错误回复：`ERR no frame`、`ERR stale frame`（帧龄超过 500 ms）、`ERR encoder`、`ERR llm`。

> REQ/REP 是严格的一问一答：客户端超时后必须重建 REQ socket，否则状态机会卡住。（ZMQ 通用约束，写新客户端时注意）

### 4.4 NPU 资源仲裁

- YOLO 固定在 NPU CORE_0，VLM 视觉编码器默认用 3 核，rkllm 参数为 max_new=64、ctx=4096。[依据代码]
- VLM 忙时发布 busy，YOLO 降到 5 fps；busy 超过 60 s 视为 VLM 异常，YOLO 恢复全速。[依据代码] 降帧效果 [待验证]

> ⚠️ `vlm_proc` 的 usage 文字写 max_new 为 128，代码默认值是 64，两者不一致。调参时以代码为准。

### 4.5 底盘（micro-ROS）

- ESP32 固件经 Wi-Fi UDP 连接 agent，节点名 `leap_low_driver`。MCU 资源受限，所以用 micro-ROS。
- 固件内部用 20 ms 定时器：odom、imu 标称 50 Hz；scan、battery、ultrasonic 每 5 个周期发一次，标称 10 Hz。[依据代码]

QoS 汇总 [依据代码]：

| Topic | QoS |
|---|---|
| `/odom`、`/scan` | RELIABLE |
| `/imu`、`/battery_state`、`/ultrasonic` | BEST_EFFORT |
| `/cmd_vel`（订阅） | BEST_EFFORT |

> 用 `ros2 topic echo/hz` 看 BEST_EFFORT 话题时要加 `--qos-reliability best_effort`，否则收不到。

### 4.6 全屏摄像头预览（camera_view，未实现）

开机后屏幕停在桌面/登录界面，看不到摄像头画面。需求是「开机就在屏幕上看到画面」。

约束与结论（依据代码 + 环境推断，未实现）：

- **不能另开 V4L2**：`camera_proc` 已用 `open(VIDEO_DEVICE, O_RDWR)` 独占 `/dev/video44`（`dev_Camera.c:31`），第二个进程开同一节点会和它抢 buffer。所以预览进程必须读 **FrameRing**（`/edgerover_frames`，640x640 RGB888），不能自己采图。
- **不走 GStreamer**：GStreamer 要自己打开 V4L2 或用 `v4l2src`，同样冲突；而且 RK3588 上 `v4l2src` 拿不到已经 mmap 的 buffer。走 FrameRing 只需要一个 `FrameRingReader`，和 `frame_dump` 一样，代码量最小。
- **也不要用 X11/桌面**：仓库里 `common.h:21` 已经留了 `DRM_DEVICE "/dev/dri/card0"`，`vendor_c/` 里没有 DRM 显示代码。纯 DRM/KMS + GLES2 纹理是最小依赖方案，但需要自己写 page flip / atomic commit，代码量比想象大。
- **最快可行的两条路**（都未验证）：

  | 方案 | 依赖 | 代码量 | 备注 |
  |---|---|---|---|
  | A. Qt5 `eglfs` + `QLabel` 显示 QImage | `qtbase5-dev`；板上是否已装**待确认** | 约 60 行（读帧 → `QImage` → `QPixmap` → `setPixmap`），全屏 `showFullScreen()` | EGLFS 直接走 KMS，不需要桌面；Qt 自动接管 `card0`，所以必须 **停掉/退出桌面**（轻量桌面如 `lightdm` 占着 DRM master 会冲突） |
  | B. 纯 DRM + GLES2 | `libdrm-dev`、`libgles2-mesa-dev`（板上有 `librga` 但没验证 GLES） | 约 250–400 行（dumb buffer / atomic commit + 纹理） | 代码多，但不引入 Qt |

- **建议先选 A**：项目目标是简历展示完整度，不追求零依赖；Qt 代码短、调试快。若板上没有 Qt，则先确认 `apt list --installed | grep -i qt5`。
- **自启与桌面二选一**：要么关掉桌面（`sudo systemctl set-default multi-user.target`），要么让预览进程跑在桌面里（`X11` 下 `QLabel` 全屏 + 自动登录）。前者更稳，开机直接出画面。
- **登录界面**：若保留桌面，需要开自动登录（`/etc/lightdm/lightdm.conf` 的 `autologin-user=lyfan`，文件位置**待确认**，不同发行版配置目录不同）。
- 退出：无键盘，不设计按键退出；靠 `systemctl stop <unit>` 或断电。

---

## 5. 已经解决的关键问题

| 问题 | 原因 | 处理 | 状态 |
|---|---|---|---|
| 底盘没有 `/odom` | 板上没有 Docker，agent 根本没运行 | 改用工作空间内原生编译的 micro_ros_agent | [已解决] `/odom` 实测 50.5 Hz |
| `/cmd_vel` 停发后底盘一直按最后指令运动；agent 断开时电机不停 | 固件没有指令超时停车 | 在固件 `main/tasks/control/motion_task.cpp` 加 500 ms 看门狗（`kCmdVelTimeoutTicks`），只针对 micro-ROS 来源的指令 | [已解决] 停发、`--once`、杀 agent 三种情况都在约 0.5 s 内停车，之后能自动重连 |
| 运动方向与坐标约定 | — | 前进时 `linear.x` 为正；`angular.z` 为正时逆时针 | [已验证] 2026-11-18 |
| 视觉栈与 bridge 开机自启 | — | 两个 systemd 单元 | [已验证] 重启后正常 |
| 固件不发 TF，TF 树断开 | 固件只发 `/odom` 消息 | 上位机 `pkg_edgerover_odom_tf`：`tf_bringup.launch.py` = `odom_tf` + robot_state_publisher + joint_state_publisher | [已解决] 2026-11-20 `view_frames` 完整树（含左右轮） |
| micro_ros_agent、TF 没有自启 | — | `edgerover-uros-agent.service`、`edgerover-tf.service` | [已验证] 2026-11-20 `sudo reboot` 后无需手动操作 |
| `/scan` 内容不明 | 固件用 `xQueuePeek` 重复发布、雷达有大量空桶 | 脚本实测：360 点 / 1°、8.6–8.8 Hz、整帧实际约 6 Hz、约 35% 桶无效、0° 朝车头 | [已验证] 2026-11-20；左右方向、225°/315° 固定回波未定 |
| ROS 桥接 | — | `edgerover_ros_bridge` | [已验证] 8 项检查全部通过 |
| `chuyun_navigation2/slam_toolbox_mapping.launch.py` 不能用 | 写死 `/root/slam_ws` 源码版 slam_toolbox，第三方包不改 | 新建自研包 `edgerover_navigation`，`slam_mapping.launch.py` include apt 版 `online_async_launch.py`，参数用 `config/slam_toolbox.yaml` | [已验证] 2026-11-21 |
| `map_saver_cli` 报 `Failed to spin map subscription` | 默认等待 2 s，`/map` 每 5 s 才发一次 | 加 `--ros-args -p save_map_timeout:=10.0` | [已验证] 2026-11-21 |
| SLAM 地图墙体多重旋转重影、位姿图分团 | 建图过程中同时做了快速原地转圈等测试，错误匹配进入位姿图 | 重启 SLAM 从空图开始，慢速遥控（≤0.15 m/s、≤0.3 rad/s），建图期间不做其他测试 | [已验证] 2026-11-21 重建后结构清晰 |
| `/scan` 左右方向 | — | 车左侧放箱，约 90° 处读到箱距：正角逆时针，固件镜像正确 | [已验证] 2026-11-21 |

---

## 6. 已经验证失败的方案

| 方案 | 失败原因 | 结论 |
|---|---|---|
| 用 `docker run microros/micro-ros-agent:humble` 启动 agent | 板上没有 Docker | 不再使用；用工作空间内的原生 agent。今后加新组件前先检查 `src/third_party` 是否已有 |

（其他方案没有失败的实测记录，不列。）

---

## 7. 尚未解决的技术问题

### 7.1 [未解决]

- [ ] Nav2 窄通道受限：方案与状态见 `interfaces.md` 第 11 节。目前开阔区域可用，可通行宽度约 0.6 m 的通道会卡住（膨胀区合拢 + odom 转角误差 + `/scan` 35% 无效点）。可选措施：`global_costmap.inflation_radius` 0.29 → 0.18（未验证）、关掉实时障碍层改回纯静态地图、或把速度再降。
- [ ] `/odom` 航向偏小约 10%：航向只靠编码器积分。2026-11-21 原地转 3 圈 odom 978.4°（目测 1080°）；直行 1 m odom 0.996 m（轮径准确，误差只在航向）。**固件 `main/control/motion_controller.cpp:12` 的 `kTrackWidth` 已由 131.7 mm 改为 119.3 mm（磁盘文件时间 2026-11-21 17:25），但该修改尚未重新烧录、未复测**，所以 10% 误差是否消除未确认 [待验证]。IMU 比例误差未标定，暂不作为真值。对 SLAM 影响可被扫描匹配吸收；Nav2 的 AMCL 运动模型会直接承受这部分误差——实测定位跳变时，先把 `alpha1`/`alpha4`（odom 旋转噪声）从 0.05 提到 0.1~0.2。
- [ ] 开机后约 30 s 内 `odom_tf` 报 `odom stamp skew 4.6~4.7s vs host clock`（2026-11-20 重启实测 3 次，17:29:23–17:29:47），底盘重连（agent 日志 `session closed` → `session established`）后消失，之后不再出现；期间 `/odom` 照常约 43 Hz。推测：底盘同步 epoch 时板上时钟尚未被 NTP（`systemd-timesyncd`）校正，校正后底盘时间戳整体偏 ~4.7 s，直到重连重新同步。未在 journal 中看到时钟跳变记录（普通用户看不到系统日志，需 `sudo journalctl -b -u systemd-timesyncd` 核实）。影响：SLAM / Nav2 查 TF 时会外推失败；候选措施：agent 与 TF 服务加 `After=time-sync.target`（并启用 `systemd-time-wait-sync`）。
- [ ] 路线图阶段 9：长时间稳定性测试和异常恢复测试还没做。

### 7.2 [待验证]

| 项 | 验证方法 |
|---|---|
| supervisor 退避重启、camera 连带重启下游 | 分别 `kill` 子进程，看日志里的重启时间间隔 |
| bridge 的 launch respawn | `kill` ros_bridge，看是否在 2 s 后重启 |
| VLM busy 时 YOLO 降帧的效果 | 提问期间 `ros2 topic hz /edgerover/detections` |
| 两份 librknnrt、librga 头文件与 .so 的版本一致性 | 第 1 节的 `strings` / `dpkg` 命令 |
| `/imu`、`/scan`、`/battery_state`、`/ultrasonic` 实测频率与内容 | `ros2 topic hz/echo`（BEST_EFFORT 话题加 QoS 参数） |
| PID 服务 `/set_speed_pid`、`/get_speed_pid` | `ros2 service call` |
| `kTrackWidth` 改 119.3 mm 后 `/odom` 转角精度 | 原地转 3 圈，`/odom` yaw 累计与目测 1080° 比较 |
| `mission_node` 的 DESCRIBE 环节 | `ros2 service call /edgerover/vlm/ask ...`，再跑一次完整任务 |
