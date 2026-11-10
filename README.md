# hero_chassis_controller

用于 Gazebo 英雄机器人麦克纳姆轮底盘的 ROS Noetic 控制器。

控制器通过 `hardware_interface::EffortJointInterface` 输出四个轮子的力矩。它接收 `/cmd_vel`，经过加速度限制和麦克纳姆轮逆运动学计算目标轮速，再使用四个独立的 `control_toolbox::Pid` 进行速度闭环控制。

实际轮速会用于正运动学和里程计积分，结果发布到 `/odom`，同时广播 `odom → base_link` 的 TF。

本项目以 [`rm_template`](https://github.com/gdut-dynamic-x/rm_template) 为模板创建。

## 已实现的功能

- 四轮独立 PID 速度控制；
- 从 YAML 读取 PID、底盘几何尺寸和运行参数；
- 使用 `dynamic_reconfigure` 在线修改 PID；
- 提供 `wheel_debug` 数据，配合 `rqt_plot` 观察目标轮速、实际轮速和 PID 输出；
- 接收 `geometry_msgs/Twist`，完成麦克纳姆轮逆运动学；
- 根据实际轮速计算底盘速度并积分得到里程计；
- 发布 `/odom`，广播 `odom → base_link`；
- 可配置底盘坐标系速度模式或全局坐标系速度模式；
- 全局模式下使用 `tf::TransformListener::transformVector()` 转换线速度；
- 线速度和角速度加速度限制；
- `/cmd_vel` 超时后自动清零命令。

当前代码已经通过编译，并完成控制器加载、基础前进命令、`wheel_debug` 和 `/odom` 输出检查。默认 PID 参数只是起始值，不能当作已经调好的最终参数。

## 开发环境

- Ubuntu 20.04 Docker 容器；
- ROS Noetic；
- Gazebo 11；
- catkin-tools；
- C++14；
- `rm_description_for_task` 中的英雄机器人模型。

宿主机系统版本不影响包本身的构建，但 ROS、Gazebo 和依赖需要正确安装在运行环境中。

## 包结构

```text
hero_chassis_controller/
├── cfg/
│   └── Pid.cfg
├── config/
│   └── hero_chassis_controller.yaml
├── include/
│   └── hero_chassis_controller/
│       └── hero_chassis_controller.h
├── launch/
│   └── hero_chassis_controller.launch
├── src/
│   └── hero_chassis_controller.cpp
├── CMakeLists.txt
├── package.xml
├── hero_chassis_controller_plugins.xml
└── README.md
```

几个文件的分工如下：

| 文件 | 作用 |
| --- | --- |
| `hero_chassis_controller.h` | 控制器类、关节句柄、PID、TF 和里程计成员声明 |
| `hero_chassis_controller.cpp` | 控制器生命周期、运动学、PID、里程计和坐标变换实现 |
| `Pid.cfg` | 生成 `dynamic_reconfigure` 参数接口 |
| `hero_chassis_controller.yaml` | 关节名称、底盘尺寸、PID 和运行参数 |
| `hero_chassis_controller.launch` | 加载参数并启动控制器 |
| `hero_chassis_controller_plugins.xml` | 声明 pluginlib 控制器插件 |

`PidConfig.h` 不需要手写。它由 `cfg/Pid.cfg` 在构建过程中自动生成。

## 依赖

主要 ROS 依赖包括：

- `roscpp`
- `controller_interface`
- `hardware_interface`
- `controller_manager`
- `joint_state_controller`
- `pluginlib`
- `control_toolbox`
- `dynamic_reconfigure`
- `geometry_msgs`
- `nav_msgs`
- `std_msgs`
- `tf`

运行仿真还需要 `rm_description_for_task`。调试和演示时可以使用：

- `rqt_plot`
- `rqt_reconfigure`
- `teleop_twist_keyboard`

## 获取与构建

将本包和 `rm_description_for_task` 放在同一个 catkin 工作空间的 `src` 目录下。

如果还没有获取本仓库：

```bash
cd ~/catkin_ws/src
git clone https://github.com/lingming2875/hero_chassis_controller.git
```

安装依赖并构建：

```bash
cd ~/catkin_ws
rosdep install --from-paths src/hero_chassis_controller --ignore-src -r -y
catkin build hero_chassis_controller
source devel/setup.bash
```

本项目使用 catkin-tools，不使用 `catkin_make`。

## 启动仿真和控制器

先启动机器人模型和 Gazebo：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch rm_description_for_task hero.launch
```

再打开一个终端，加载并启动控制器：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch hero_chassis_controller hero_chassis_controller.launch
```

`hero_chassis_controller.launch` 只负责加载参数和控制器，不会启动第二份 Gazebo。

不要同时启动 `simple_chassis_controller` 和本控制器。两者都要占用四个轮子的 `EffortJointInterface`，同时运行会发生资源冲突。

检查控制器状态：

```bash
rosservice call /controller_manager/list_controllers "{}"
```

正常情况下，`joint_state_controller` 和 `hero_chassis_controller` 都应为 `running`。

## 坐标系和轮子顺序

底盘采用 ROS 常用坐标约定：

- `x` 轴指向底盘前方；
- `y` 轴指向底盘左方；
- 绕 `z` 轴逆时针旋转为正。

四个轮子在程序中的顺序固定为：

| 索引 | 轮子 | Gazebo 关节名称 |
| --- | --- | --- |
| 0 | 左前轮 LF | `left_front_wheel_joint` |
| 1 | 右前轮 RF | `right_front_wheel_joint` |
| 2 | 左后轮 LB | `left_back_wheel_joint` |
| 3 | 右后轮 RB | `right_back_wheel_joint` |

修改 YAML 中的关节顺序时，必须同时考虑逆运动学、正运动学和调试数据的顺序，不能只修改关节名称。

## 底盘速度控制

控制器订阅：

| Topic | 类型 | 含义 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度命令 |

使用的三个速度分量为：

- `linear.x`：前后速度 `vx`；
- `linear.y`：左右速度 `vy`；
- `angular.z`：偏航角速度 `wz`。

下面的命令以 10 Hz 发布前进命令：

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

修改其中的 `x`、`y` 和 `z` 即可测试其他运动：

| 运动 | `linear.x` | `linear.y` | `angular.z` |
| --- | ---: | ---: | ---: |
| 前进 | 0.2 | 0.0 | 0.0 |
| 后退 | -0.2 | 0.0 | 0.0 |
| 左移 | 0.0 | 0.2 | 0.0 |
| 右移 | 0.0 | -0.2 | 0.0 |
| 逆时针旋转 | 0.0 | 0.0 | 0.5 |
| 顺时针旋转 | 0.0 | 0.0 | -0.5 |

按 `Ctrl+C` 停止发布。超过 `cmd_timeout` 没有收到新命令后，控制器会将目标速度清零。

## 逆运动学

设：

```text
r = 轮半径
k = (wheel_base + track_width) / 2
```

轮速单位为 `rad/s`，按照 LF、RF、LB、RB 的顺序，逆运动学为：

```text
ω0 = (vx - vy - k·wz) / r
ω1 = (vx + vy + k·wz) / r
ω2 = (vx + vy - k·wz) / r
ω3 = (vx - vy + k·wz) / r
```

得到四个目标轮速后，每个轮子分别计算：

```text
error = desired_velocity - actual_velocity
effort = PID(error)
```

PID 输出通过 `EffortJointInterface::setCommand()` 写入 Gazebo 关节。



## 四轮独立 PID

四个轮子各有一个 `control_toolbox::Pid`，不是四个轮子共用一个 PID 对象。

YAML 中使用的参数前缀为：

```text
front_left_
front_right_
back_left_
back_right_
```

每个轮子都有四个参数：

```text
p
i
d
i_clamp
```

例如左前轮：

```yaml
front_left_p: 2.0
front_left_i: 0.0
front_left_d: 0.0
front_left_i_clamp: 1.0
```

修改 YAML 后，需要重新加载控制器。通过 `rqt_reconfigure` 修改的是运行时参数，默认不会自动写回 YAML。

打开在线调参界面：

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

在界面中选择：

```text
/hero_chassis_controller
```

调参时结合 `rqt_plot` 看响应，不要只凭机器人“看起来能动”判断参数是否合适。可以先调 P，再根据超调和振荡情况调整 D；确有稳态误差时再加入少量 I。

## wheel_debug 与 rqt_plot

控制器发布：

| Topic | 类型 |
| --- | --- |
| `/hero_chassis_controller/wheel_debug` | `std_msgs/Float64MultiArray` |

数组共有 12 个元素：

| 下标 | 数据 |
| --- | --- |
| `data[0..3]` | 四轮目标角速度 |
| `data[4..7]` | 四轮实际角速度 |
| `data[8..11]` | 四轮 PID 输出力矩 |

每一组内部都按照 LF、RF、LB、RB 排列。

绘制左前轮的目标速度、实际速度和 PID 输出：

```bash
rosrun rqt_plot rqt_plot \
'/hero_chassis_controller/wheel_debug/data[0]' \
'/hero_chassis_controller/wheel_debug/data[4]' \
'/hero_chassis_controller/wheel_debug/data[8]'
```

其他轮子的对应关系为：

| 轮子 | 目标速度 | 实际速度 | PID 输出 |
| --- | --- | --- | --- |
| LF | `data[0]` | `data[4]` | `data[8]` |
| RF | `data[1]` | `data[5]` | `data[9]` |
| LB | `data[2]` | `data[6]` | `data[10]` |
| RB | `data[3]` | `data[7]` | `data[11]` |

## 正运动学与里程计

控制器读取四个轮子的实际速度，通过正运动学计算机体速度：

```text
vx = r/4 · (ω0 + ω1 + ω2 + ω3)
vy = r/4 · (-ω0 + ω1 + ω2 - ω3)
wz = r/(4k) · (-ω0 + ω1 - ω2 + ω3)
```

位置和偏航角通过离散积分更新。平移积分使用中点偏航角：

```text
midpoint_yaw = yaw + wz·dt/2
```

随后发布：

| 数据 | 内容 |
| --- | --- |
| `/odom` | `nav_msgs/Odometry` |
| TF | `odom → base_link` |

查看里程计：

```bash
rostopic echo /odom
```

检查 TF：

```bash
rosrun tf tf_echo odom base_link
```

控制器每次进入 `starting()` 时会将内部里程计位置清零。

这里的里程计完全来自轮速。轮子打滑、地面接触变化和几何参数误差都会影响结果，麦克纳姆轮横移时尤其明显。它不是绝对定位。

## 底盘与全局速度模式

配置文件中的 `global_mode` 用于切换 `/cmd_vel` 的解释方式。

### 底盘坐标系模式

```yaml
global_mode: false
```

`linear.x` 和 `linear.y` 直接视为底盘坐标系下的速度。

### 全局坐标系模式

```yaml
global_frame: "odom"
global_mode: true
```

控制器将全局坐标系下的线速度向量通过 `transformVector()` 转换到 `base_link`，再执行逆运动学。

这里变换的是速度向量，不是位置点，因此不应把 `odom` 原点的平移量加到速度上。`angular.z` 不参与平面线速度向量变换。

如果 TF 查询失败，控制器会将线速度命令清零，避免把未变换的全局速度直接当成底盘速度执行。

修改模式后重新加载控制器，再检查 `/odom` 和 TF 是否正常。

## 加速度限制与命令超时

默认参数：

```yaml
cmd_timeout: 0.5
max_linear_accel: 1.0
max_angular_accel: 2.0
```

加速度限制按控制周期计算允许的最大变化量：

```text
max_delta = max_accel · dt
```

它分别作用于：

- `linear.x`
- `linear.y`
- `angular.z`

这样可以避免目标速度阶跃变化直接传到轮速 PID。限制的是命令变化率，不是 Gazebo 中机器人的绝对加速度上限。

`cmd_timeout` 用于处理命令发布者退出或通信中断。没有收到过命令，或者命令超过超时时间未更新，控制器都会将命令清零。

## 主要配置参数

配置位于 `config/hero_chassis_controller.yaml`。

| 参数 | 当前值 | 含义 |
| --- | ---: | --- |
| `wheel_radius` | 0.07625 | 轮半径，单位 m |
| `wheel_base` | 0.4 | 前后轮中心距，单位 m |
| `track_width` | 0.4 | 左右轮中心距，单位 m |
| `cmd_timeout` | 0.5 | 命令超时时间，单位 s |
| `max_linear_accel` | 1.0 | 线速度命令变化率限制，单位 m/s² |
| `max_angular_accel` | 2.0 | 角速度命令变化率限制，单位 rad/s² |
| `odom_frame` | `odom` | 里程计父坐标系 |
| `base_frame` | `base_link` | 底盘坐标系 |
| `global_frame` | `odom` | 全局速度命令所在坐标系 |
| `global_mode` | `false` | 是否启用全局速度模式 |

`wheel_base` 和 `track_width` 应与机器人实际模型一致。只凭轮子在转不能证明几何参数正确，横移和旋转测试更容易暴露尺寸或轮序问题。

## 控制器如何被加载

本包不是通过 `rosrun` 启动一个独立控制节点，而是作为 `ros_control` 控制器插件运行。

加载过程为：

```text
PLUGINLIB_EXPORT_CLASS
→ hero_chassis_controller_plugins.xml
→ package.xml 中的 controller_interface export
→ YAML 中的控制器 type
→ controller_manager/spawner
→ Gazebo 中的 gzserver 调用控制器 update()
```

插件声明为：

```text
hero_chassis_controller/HeroChassisController
```

控制器继承：

```cpp
controller_interface::Controller<
    hardware_interface::EffortJointInterface>
```

因此，控制器列表为空时应先检查参数是否加载、插件是否导出以及 spawner 是否运行，而不是反复执行 `rosrun`。

## 键盘控制

安装并运行 `teleop_twist_keyboard` 后，可以通过键盘向 `/cmd_vel` 发布速度：

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

键盘节点只负责发布速度命令，轮速计算、PID 和里程计仍由本控制器完成。

## 当前验证记录

已经完成：

- `catkin build hero_chassis_controller` 构建检查；
- pluginlib 加载和 controller_manager 启动检查；
- 基础 `/cmd_vel` 前进命令检查；
- `wheel_debug` 数据输出检查；
- `/odom` 数据输出检查；
- 停止持续发布命令后的基础停车检查。

后续演示前还应分别补测并记录：

- 后退、左右横移和原地旋转；
- `rqt_plot` 轮速响应曲线；
- `rqt_reconfigure` 在线修改四轮 PID；
- `tf_echo odom base_link`；
- `global_mode: true` 下的全局速度控制；
- 键盘控制；
- 加速度限制和命令超时的实际效果。

代码中存在某个功能，不等于该功能已经实测通过。完成一项测试后，应同步更新本节和对应的 Git 提交。

## 常见问题

### `Unable to communicate with master`

ROS Master 或 Gazebo 仿真尚未运行。先启动 `rm_description_for_task` 的 `hero.launch`，再启动控制器。

### 控制器列表为空

`hero.launch` 只负责机器人描述和 Gazebo 模型，不会自动加载本包控制器。还需要运行：

```bash
roslaunch hero_chassis_controller hero_chassis_controller.launch
```

### 启动时提示关节资源冲突

检查 `simple_chassis_controller` 是否仍在运行。同一组 `EffortJointInterface` 关节不能同时交给两个控制器。

### 修改 PID 后重新启动又恢复原值

`rqt_reconfigure` 不会自动保存到 YAML。确认参数合适后，需要手动写回 `config/hero_chassis_controller.yaml`。

## 模板与参考

- [rm_template](https://github.com/gdut-dynamic-x/rm_template)
- [simple_chassis_controller](https://github.com/gdut-dynamic-x/simple_chassis_controller)
- [rm_description_for_task](https://github.com/gdut-dynamic-x/rm_description_for_task)
- [ros_control](http://wiki.ros.org/ros_control)
- [pluginlib](http://wiki.ros.org/pluginlib)
- [control_toolbox](http://wiki.ros.org/control_toolbox)
- [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure)
- [tf](http://wiki.ros.org/tf)

## 版本管理

开发过程使用 Git 记录。包初始化、文档修改、分支合并和 README 冲突解决均保留在提交历史中。

查看历史：

```bash
git log --oneline --graph --decorate --all
```

后续修改应拆分成含义明确的提交，例如：

```text
Add mecanum inverse kinematics.
Add wheel odometry and TF.
Verify chassis and global velocity modes.
Update controller usage and test results.
```
