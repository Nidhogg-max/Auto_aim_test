# RoboMaster 自瞄系统

本项目是一个基于 ROS2 Humble 的 RoboMaster 机器人装甲板自动瞄准系统，**根据原作者 [陈军](https://github.com/chenjunnn) 的开源方案改进与实现**，由 **Nidhogg-max** 修改和维护。系统能够完成装甲板的实时检测、数字识别、三维位姿解算以及目标的跟踪滤波，输出高鲁棒性的敌方装甲板状态。

## 📸 效果预览

| 原图 | 二值化 | 灯条提取 |
|------|--------|----------|
| ![原图](docs/raw.png) | ![二值化](docs/gray_bin.png) | ![灯条](docs/red.png) |

| 数字矫正 | 分类结果 | 识别输出 |
|----------|----------|----------|
| ![数字](docs/num_bin.png) | ![结果](docs/result.png) | ![](docs/rm_vision.svg) |

## ✨ 主要特性

- **完整的自瞄流水线**  
  从相机图像采集 → 装甲板检测 → 数字识别 → PnP 位姿解算 → 目标跟踪 → 发布目标状态，一应俱全。

- **轻量化数字分类网络**  
  使用 ONNX 格式的多层感知机（MLP）对数字灯条进行分类，模型小、推理快，可直接代替传统特征匹配。

- **在线扩展卡尔曼滤波（EKF）跟踪**  
  对目标在三维空间的位置、速度以及装甲板旋转状态进行实时估计，处理遮挡、跳跃等复杂情况。

- **即插即用的 ROS2 节点**  
  各模块独立封装，参数可通过 launch 文件灵活配置，支持 `debug` 调试输出与 RViz 可视化。

- **支持多种装甲板类型与数目**  
  自动识别小装甲板（常规4块）、大装甲板（平衡步兵/哨兵2块）、前哨站（3块）等不同结构。

## 📁 软件架构

```
rm_auto_aim
├── auto_aim_interfaces   # 自定义 ROS2 消息接口定义
├── armor_detector        # 装甲板检测与数字识别节点
├── armor_tracker         # 目标跟踪与状态估计节点
└── rm_vision             # 主启动（launch）与参数配置
```

- **armor_detector**  
  订阅相机图像 (`/image_raw`) 和相机信息 (`/camera_info`)，检测装甲板并识别数字，输出 `<Armors>` 消息。
- **armor_tracker**  
  订阅 `/detector/armors`，对多帧检测结果进行关联与跟踪，通过 EKF 滤波输出目标的精确位置与速度。
- **auto_aim_interfaces**  
  定义装甲板 (`Armor`)、目标 (`Target`)、调试信息等消息类型，供 detector 和 tracker 使用。

## 🚀 快速开始

### 环境要求

- 操作系统：Ubuntu 22.04
- ROS2 版本：[Humble](https://docs.ros.org/en/humble/Installation.html)
- 编译工具：`colcon`、`ament_cmake`
- 依赖库：OpenCV、Eigen3、tf2 等

### 安装依赖

```bash
# 克隆项目到工作空间
cd ~/ros2_ws/src
git clone https://github.com/your_username/your_repo.git  # 请替换为实际地址
cd ..

# 安装 ROS 依赖
rosdep install --from-paths src --ignore-src -r -y
```

### 编译

```bash
colcon build --symlink-install --packages-up-to auto_aim_bringup
```

### 运行

```bash
source install/setup.bash
ros2 launch auto_aim_bringup bringup.launch.py
```

## 🧠 算法详解

### 1. 装甲板检测（Detector）

- **预处理**：将彩色图转为灰度图并进行二值化，通过亮度直接提取灯条轮廓。
- **灯条提取**：利用轮廓形状（长宽比、倾斜角）过滤噪声，并通过 ROI 内像素的 R/B 通道和判断颜色（红/蓝）。
- **灯条配对**：将同色灯条按照几何约束（距离、长度比、倾斜角）组成装甲板候选。
- **数字识别**：对候选区域进行透视变换、Ostu 二值化，送入 MLP 网络（结构见 `docs/model.svg`）得到数字类别和置信度。

### 2. PnP 位姿解算

采用 OpenCV 的 `SOLVEPNP_IPPE` 方法，根据装甲板的四个角点与已知物理尺寸解算 3D 位姿，返回相机坐标系下的平移和旋转。

### 3. 目标跟踪（Tracker）

- **运动模型**：假设目标在固定惯性系中做匀速直线运动，状态量包括位置、速度、装甲板旋转半径等。
- **扩展卡尔曼滤波**：预测‑更新循环，利用当前帧检测结果修正状态估计。
- **状态机**：支持 `LOST` → `DETECTING` → `TRACKING` → `TEMP_LOST` 的切换逻辑，有效处理遮挡与短暂丢失。
- **数据关联**：基于预测位置与检测位置的欧氏距离进行匹配，并处理装甲板“跳跃”情况（如目标旋转导致编号切换）。

## 📊 可视化与调试

启动 launch 文件时设置 `debug:=true` 可发布：

- 二值化图像
- 提取的数字图像
- 带标注的结果图像
- 灯条与装甲板的详细信息
- RViz 三维标记（位置、速度矢量、装甲板模型）

使用 RViz 订阅相关话题即可进行在线调试。

## ⚙️ 参数说明

详见各包目录下的 `config` 文件夹与 launch 文件。核心参数包括：

| 参数名 | 说明 | 默认值 |
|--------|------|--------|
| `binary_thres` | 灰度二值化阈值 | 160 |
| `detect_color` | 目标颜色（0‑红，1‑蓝） | 红 |
| `classifier_threshold` | 分类置信度阈值 | 0.7 |
| `tracker.max_match_distance` | 数据关联最大距离 | 0.15 m |
| `ekf.sigma2_q_xyz` | 过程噪声协方差 | 20.0 |

## 📄 许可证与致谢

本项目基于 [MIT License](LICENSE) 开源。

- 原项目：[rm_auto_aim by Chen Jun](https://github.com/chenjunnn/rm_auto_aim)
- 修改与扩展：**Nidhogg-max**

感谢原作者陈军提供的高质量开源方案，本仓库在此基础上进行了工程化优化与功能增强。欢迎大家 Star ⭐ 与贡献代码。

---

*如果在使用中遇到问题，欢迎提交 Issue 或 Pull Request。*
