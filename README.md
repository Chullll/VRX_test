# VRX 船舶仿真项目

这是一个基于ROS 2和Gazebo的虚拟海洋竞赛(Virtual RoboticX, VRX)仿真项目。项目包含船舶模型(WAM-V)、传感器配置、控制系统和仿真环境。

## 📋 项目结构

```
vrx_ws/
├── src/
│   ├── vrx/                 # VRX仿真核心包
│   │   ├── vrx_urdf/       # URDF/Xacro机器人描述文件
│   │   │   ├── wamv_description/  # 船舶基础模型
│   │   │   ├── wamv_gazebo/       # Gazebo传感器和动力学
│   │   │   └── vrx_gazebo/        # VRX特定配置
│   │   ├── vrx_gz/         # Gazebo集成和桥接
│   │   └── vrx_ros/        # ROS核心功能
│   └── wamv_control/       # 控制模块（Python）
├── build/                  # 编译输出目录
├── install/               # 安装输出目录
└── data/                  # 数据文件

```

## ⚙️ 系统要求

- **操作系统**: Ubuntu 24.04 LTS
- **ROS版本**: ROS 2 Jazzy
- **Python**: Python 3.12+
- **Gazebo**: Gazebo Harmonic

## 🚀 快速开始

### 1. 环境配置

#### 安装ROS 2 Jazzy（如果还未安装）
```bash
curl -sSL https://repo.ros2.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://repo.ros2.org/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

#### 安装Gazebo（推荐 Gazebo Harmonic）
```bash
sudo apt-get install -y gz-harmonic
```

#### 安装依赖
```bash
sudo apt install -y \
  ros-jazzy-gazebo-msgs \
  ros-jazzy-gazebo-plugins \
  ros-jazzy-gazebo-ros \
  ros-jazzy-ros-gz \
  python3-colcon-common-extensions
```

### 2. 克隆并构建项目

```bash
# 克隆项目
git clone https://github.com/Chullll/VRX_test.git
cd VRX_test

# 构建项目
colcon build

# Source环境（每次新终端需要做）
source install/setup.bash
```

### 3. 启动VRX仿真

#### 启动基础仿真环境
```bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

**可用的world选项**:
- `sydney_regatta` - 悉尼水面环境
- 其他可用环境请参考 `src/vrx/vrx_gazebo/` 中的定义

#### 启动时启用特定传感器

在launch命令后添加参数：
```bash
# 启用IMU
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta imu_enabled:=true

# 启用相机
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta camera_enabled:=true

# 启用GPS
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta gps_enabled:=true

# 启用LIDAR
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta lidar_enabled:=true

# 同时启用多个传感器
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta \
  imu_enabled:=true \
  camera_enabled:=true \
  gps_enabled:=true
```

## 📡 ROS 2 Topic和Service

### IMU传感器
```bash
# 订阅IMU数据
ros2 topic echo /sensors/imu/imu_wamv/imu/data
```

### 推进器控制
```bash
# 发送推进器命令
ros2 topic pub /thrusters/commands std_msgs/msg/Float64MultiArray \
  "data: [100.0, 100.0]"
```

## 🔧 关键配置文件

| 文件路径 | 说明 |
|---------|------|
| `src/vrx/vrx_urdf/wamv_gazebo/urdf/components/wamv_imu.xacro` | IMU传感器定义 |
| `src/vrx/vrx_urdf/wamv_gazebo/urdf/wamv_gazebo.urdf.xacro` | 船舶Gazebo配置 |
| `src/vrx/vrx_ros/launch/competition.launch.py` | 主启动文件 |
| `src/wamv_control/wamv_control/simple_thruster_control.py` | 推进器控制模块 |

## 📝 常见问题

### Q: 编译失败怎么办？
**A**: 确保已source设置文件并重新构建：
```bash
source /opt/ros/humble/setup.bash
cd /home/chul/vrx_ws
colcon build --symlink-install
```

### Q: 仿真运行缓慢
**A**: 检查以下几点：
- 确保GPU驱动正确安装
- 关闭不需要的传感器
- 减少仿真环境的视觉效果质量

### Q: IMU数据异常
**A**: 如果IMU数据出现异常（如固定关节导致的旋转问题），参考项目中的IMU关节配置文件。

### Q: 如何修改控制参数？
**A**: 编辑 `src/wamv_control/wamv_control/simple_thruster_control.py` 中的参数，然后重新构建。

## 🔄 工作流程

1. **修改配置或代码**
   ```bash
   # 编辑相关文件...
   ```

2. **重新构建**
   ```bash
   colcon build
   ```

3. **Source新的环境**
   ```bash
   source install/setup.bash
   ```

4. **启动仿真**
   ```bash
   ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
   ```
