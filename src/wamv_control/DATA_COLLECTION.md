# WAM-V 数据采集说明

## 采集入口

控制脚本启动后会先询问：

```bash
是否在控制开始前启用 USV 数据采集？[y/N]:
```

输入 `y`、`yes`、`是` 或 `是的` 后，控制节点会创建
`wamv_control.data_collector.USVDataCollector`，并在推进器控制计时开始
前启动数据采集。

注意：`ros2 run` 执行的是 `~/vrx_ws/install` 中已安装的包，不会直接执行
`src` 目录里的源码。修改代码后需要重新编译并 source：

```bash
cd ~/vrx_ws
colcon build --packages-select wamv_control --merge-install
source install/setup.bash
```

建议在运行控制脚本时启用仿真时间：

```bash
ros2 run wamv_control simple_thruster_control --ros-args -p use_sim_time:=true
```

## 数据来源

- 全局位姿来自 Gazebo Transport：
  `/world/sydney_regatta/dynamic_pose/info`
- 采集脚本通过 `gz topic -e -t` 读取原始文本，并筛选
  `name: "wamv"` 的 `pose`。
- IMU 数据来自 ROS 2：
  `/wamv/sensors/imu/imu/data`
- IMU 消息类型为 `sensor_msgs/msg/Imu`。

当前环境没有可用的 `ros_gz_interfaces/msg/EntityPoses`，并且将 Gazebo
位姿桥接为 `tf2_msgs/msg/TFMessage` 后会丢失实体名称。因此本采集脚本
直接解析 Gazebo 原始输出，避免依赖 transform 顺序猜测 `wamv`。

## 输出文件

默认输出目录：

```text
~/vrx_ws/data
```

文件名格式：

```text
usv_data_YYYYMMDD_HHMMSS.csv
```

可通过 ROS 2 参数覆盖：

```bash
ros2 run wamv_control simple_thruster_control --ros-args \
  -p use_sim_time:=true \
  -p data_output_dir:=/tmp/wamv_data \
  -p data_sample_period:=0.1 \
  -p data_flush_every_n:=10
```

## 代码结构

`wamv_control/data_collector.py` 按职责拆成以下组件：

- `GazeboPoseReader`：只负责启动 `gz topic` 并解析 `wamv` 的全局 pose。
- `ImuReader`：只负责订阅 `/wamv/sensors/imu/imu/data`。
- `StateEstimator`：只负责四元数转欧拉角、全局线速度差分、角加速度差分。
- `CsvLogger`：只负责 CSV 文件创建、写入和按批次 flush。
- `USVDataCollector`：只负责组织上述组件，并由控制脚本调用。

## CSV 字段

| 字段 | 含义 |
| --- | --- |
| `sim_time_sec` | ROS clock 时间；启用 `use_sim_time` 后为 Gazebo 仿真时间 |
| `elapsed_time_sec` | 采集开始后的仿真经过时间 |
| `global_x/y/z` | Gazebo 中 `wamv` 的全局位置 |
| `global_qx/qy/qz/qw` | Gazebo 中 `wamv` 的全局姿态四元数 |
| `roll/pitch/yaw` | 由全局四元数转换得到的欧拉角，单位 rad |
| `global_vx/vy/vz` | 对全局位置做差分得到的全局线速度 |
| `imu_angular_velocity_x/y/z` | IMU 测得的角速度，通常单位 rad/s |
| `imu_linear_acceleration_x/y/z` | IMU 测得的线加速度，通常单位 m/s^2 |
| `imu_angular_acceleration_x/y/z` | 对 IMU 角速度做差分得到的角加速度 |

## 注意事项

- 启动采集前需要先启动 VRX/Gazebo 仿真，确保 Gazebo 位姿话题存在。
- 若需要严格记录仿真时间，运行控制脚本时应设置 `use_sim_time:=true`。
- `gz topic -e -t /world/sydney_regatta/dynamic_pose/info` 应能持续输出
  `name: "wamv"` 的 `pose`。
- CSV 默认每 10 行 flush 一次，可用 `data_flush_every_n` 调整；退出采集时
  会强制 flush 并关闭文件。
- IMU 线加速度位于 IMU/body 坐标系，若需要全局坐标系加速度，需要额外
  用全局姿态四元数做坐标变换。
- 差分得到的速度和角加速度对采样间隔及噪声敏感，如后续用于训练或辨识，
  建议再做滤波或平滑处理。
