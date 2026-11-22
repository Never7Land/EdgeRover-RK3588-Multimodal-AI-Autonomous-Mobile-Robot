# EdgeRover — Claude Code 项目规则

## 项目简介

EdgeRover 是运行在 RK3588 上的边缘智能小车：
- 视觉流水线：camera → YOLO → VLM，非 ROS 进程，由 supervisor 管理；
- `ros_bridge`：把视觉结果接入 ROS2；
- 底盘：ESP32 固件经 micro-ROS 接入 ROS2。

## 路径与环境

| 项 | 内容 |
|---|---|
| 板上项目根 | `~/edge_rover`（用户 `lyfan`，主机 `rk3588`） |
| Windows 侧 | `Z:\edge_rover`，挂载自板上 `~/edge_rover`；只用来编辑，不在 Windows 上构建 |
| 系统 / ROS2 | RK3588，ROS2 Humble（`/opt/ros/humble`）；Ubuntu 版本未确认 |
| colcon 工作空间 | `ros2_workspace/`：自研包在 `src/`，第三方包在 `src/third_party/` |
| 模型 | `model/`（`.rknn` / `.rkllm`） |
| 其他 | `config/`、`maps/`（SLAM 配置与地图） |
| 底盘固件 | 不在本仓库（ESP-IDF 工程） |

文档没有写明的环境信息（版本号、IP、单元文件内容等）不要猜。需要时说明“未确认”，并给出可在板上执行的核实命令。

## 代码修改规范

- 只改当前任务涉及的文件，不顺手重构，也不改格式。
- 以下内容不要修改，除非任务明确要求：
  - `src/third_party/` 下的包；
  - `edgerover_perception/third_party/`、`edgerover_perception/vendor_c/`；
  - `edgerover_vlm/third_party/`（自带的头文件和 `.so`）。
- 不要修改或删除 `ros2_workspace/build|install|log/`、`model/` 下的文件。
- 自研包统一使用 C++17。保持各包 `CMakeLists.txt` 里现有的警告选项，新代码不能引入新的警告。
- 修改 ZMQ 消息、FrameRing 结构、ROS2 话题/服务/消息、`bridge.yaml` 参数时，要同步检查所有收发两端，并更新 `docs/interfaces.md`。
- 修改模型或安装路径时，要同步修改 supervisor 参数（它有默认路径）。
- 涉及 NPU 运行时或 librga 的改动要注意版本匹配，细节见 `docs/development_notes.md`。
- 未在板上编译和运行过的改动，要明确标注“未验证”，不要写成已完成。
- 除非用户明确要求，Claude 不直接修改代码：只给实现方案，写明要改的文件和行号，附必要的解释，由用户自己动手。
- 给用户的回答里：
  - 每次出现的 frame（坐标系）、话题、节点、包等专有名词，后面都用括号加一句简短注释，例如 `base_footprint`（车体在地面的投影点）、`/odom`（底盘发布的里程计话题）；
  - 实现方案要给出具体要改的代码，并写明文件完整路径（如 `Z:\edge_rover\ros2_workspace\src\pkg_edgerover_odom_tf\src\odom_tf_node.cpp`）和行号；代码块里只写要新增/替换成的代码，不要用 `+`/`-` 或行号前缀；定位信息（插在第几行前后、替换第几行、前后原代码是什么）写在代码块外面的文字里；
  - 给出的每段待修改代码都附一句中文注释，说明这行做什么、为什么这样改。
  - 给出的Shell命令给出注释，表面是在干什么

## 构建与部署（实际方式）

- 在 RK3588 板上用 colcon 原生构建，工作目录为 `~/edge_rover/ros2_workspace`。不使用交叉编译，板上也没有 Docker。
- 开机自启由两个 systemd 服务负责：
  - `edgerover.service` → supervisor → camera / yolo / vlm；
  - `edgerover-ros-bridge.service` → `ros2 launch`（respawn）→ `ros_bridge`。
- 这两个 systemd 单元文件不在仓库中，不要假设其内容。
- `micro_ros_agent` 需要手动启动，没有自启。启动命令见 `docs/development_notes.md`。
- Claude 在 Windows 侧无法构建或运行。需要上板执行的命令应交给用户运行，并说明预期结果。

## 文档索引（按需读取）

| 文档 | 何时读取 |
|---|---|
| `docs/architecture.md` | 涉及进程/包职责、数据流、通信分工、systemd 与 supervisor 进程管理时 |
| `docs/interfaces.md` | 修改或调用 ROS2 话题/服务、ZMQ 消息、FrameRing、`bridge.yaml`、底盘 micro-ROS 接口时 |
| `docs/development_notes.md` | 处理环境/SDK、构建部署、运行库版本、技术选型，或排查已知问题、失败方案时 |

读取规则：
- 只读当前任务需要的文档，并优先按标题定位到相关章节读取，**不要每次加载全部技术文档**。
- 简单、局部的代码修改，如果不涉及上述主题，就不需要读文档。
- 本文件只放长期规则和索引。技术细节以上述文档为准，不要复制到这里。
- 任务产生了经过验证的新结论或接口变化时，更新对应文档，并保留文档里已有的状态标记（✅/⚠️/🚧、[已验证] 等）。
