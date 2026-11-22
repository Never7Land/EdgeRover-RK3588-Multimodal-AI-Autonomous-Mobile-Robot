An RK3588-Based Multimodal AI Autonomous Mobile Robot

基于 Rockchip RK3588 的边缘 AI 移动机器人项目，融合 YOLO 目标检测、Qwen3-VL 多模态视觉理解与 ROS 2 机器人软件框架，实现环境感知、视觉语义理解、机器人定位与自主导航。

1. 项目介绍（Introduction）

EdgeRover 是一个基于 RK3588 边缘计算平台开发的多模态 AI 移动机器人系统。

项目以 RK3588 为核心计算平台，通过 NPU 加速视觉模型和多模态大模型推理，并使用 ROS 2 构建机器人软件系统。

系统主要包含以下能力：

视觉感知： 使用 YOLO 实现目标检测。

多模态理解： 使用 Qwen3-VL 对视觉图像进行语义理解。

机器人通信： 基于 ROS 2 和 DDS 实现模块间通信。

自主定位： 使用 SLAM Toolbox 进行环境建图与定位。

自主导航： 基于 Nav2 实现路径规划与导航控制。

底盘控制： 通过 ROS 2 与底盘控制系统进行通信，实现移动控制。

项目重点探索边缘计算平台上的 AI 模型部署、多模块机器人系统集成，以及视觉感知与机器人运动控制的协同。

2. 系统架构（System Architecture）

                  EdgeRover
                      |
              RK3588 Computing
                      |
        +-------------+-------------+
        |             |             |
    AI Perception  ROS 2 System   Robot Control
        |             |             |
    +---+---+     +---+---+     +---+---+
    |       |     |       |     |       |
   YOLO  Qwen3-VL SLAM   Nav2  micro-ROS Chassis
    |       |     |       |     |       |
    +---+---+     +---+---+     +---+---+
        |             |             |
        +-------------+-------------+
                      |
               Mobile Robot

