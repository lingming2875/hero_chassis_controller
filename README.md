# hero_chassis_controller

这个 ROS 包用于控制 Gazebo 中的英雄机器人麦克纳姆轮底盘。示例控制器只是周期性输出几组固定力矩，这里则把 `/cmd_vel` 转换成四个轮子的目标转速，再用四套独立的 PID 闭环跟踪。PID 的计算结果通过 `hardware_interface::EffortJointInterface` 写入关节。

控制器还会读取实际轮速，计算机体速度和里程计。结果一方面发布到 `/odom`，另一方面用于广播 `odom → base_link` 的 TF，因此后续做全局速度控制时也能使用这组坐标变换。

项目从 [`rm_template`](https://github.com/gdut-dynamic-x/rm_template) 创建，开发环境为 ROS Noetic 和 Gazebo 11。

## 功能与当前状态

代码里已经包含这些功能：

- 四个轮子分别使用一个 `control_toolbox::Pid`；
- PID、轮距、轴距等参数放在 YAML 中；
- 可以通过 `dynamic_reconfigure` 在线改参；
- `wheel_debug` 同时输出了目标轮速、实际轮速和 PID 力矩，方便接入 `rqt_plot`；
- 接收 `geometry_msgs/Twist`，完成麦克纳姆轮逆运动学；
- 根据实际轮速进行正运动学计算和里程计积分；
- 发布 `/odom` 和 `odom → base_link` TF；
- 支持底盘坐标系、全局坐标系两种速度模式；
- 全局模式下调用 `tf::TransformListener::transformVector()`；
- 对速度命令做加速度限制，并在 `/cmd_vel` 超时后停车。

目前已经做过构建、控制器加载、基础前进、`wheel_debug` 输出和 `/odom` 输出检查。YAML 里的 PID 数值只是暂时可用的起点，还不是完整调参后的结果。横移、旋转、全局速度模式等功能虽然已有代码，仍需要分别实测。

## 开发环境

实际开发时使用 Ubuntu 20.04 Docker 容器，里面安装了 ROS Noetic 和 Gazebo 11。代码按 C++14 构建，构建工具是 catkin-tools，机器人模型来自 `rm_description_for_task`。


## 文件结构

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

控制器的主要逻辑放在 `src/hero_chassis_controller.cpp`，成员声明位于对应的头文件。`cfg/Pid.cfg` 负责生成在线调参接口，`config/hero_chassis_controller.yaml` 保存关节名称、底盘尺寸和默认参数。

启动文件只加载参数并请求 `controller_manager` 启动控制器。插件名称和导出关系则写在 `hero_chassis_controller_plugins.xml` 与 `package.xml` 中。

构建时可能在编辑器里看到 `PidConfig.h` 不存在的提示。这个头文件由 `Pid.cfg` 自动生成，不需要手动创建。

## 依赖

包直接使用的 ROS 组件有：

```text
roscpp
controller_interface
hardware_interface
controller_manager
joint_state_controller
pluginlib
control_toolbox
dynamic_reconfigure
geometry_msgs
nav_msgs
std_msgs
tf
```

仿真还需要 `rm_description_for_task`。如果要完成调参和键盘演示，可以再准备 `rqt_plot`、`rqt_reconfigure` 与 `teleop_twist_keyboard`。

## 获取与构建

本包和 `rm_description_for_task` 应放在同一个 catkin 工作空间的 `src` 下。还没有克隆本仓库时，可以执行：

```bash
cd ~/catkin_ws/src
git clone https://github.com/lingming2875/hero_chassis_controller.git
```

随后安装依赖并构建：

```bash
cd ~/catkin_ws
rosdep install --from-paths src/hero_chassis_controller --ignore-src -r -y
catkin build hero_chassis_controller
source devel/setup.bash
```

这里使用 catkin-tools，不使用 `catkin_make`。

## 启动仿真和控制器

启动顺序不要反过来。先让 ROS Master、Gazebo 和机器人模型运行起来：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch rm_description_for_task hero.launch
```

另开一个终端，再加载控制器：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch hero_chassis_controller hero_chassis_controller.launch
```

第二个 launch 不会创建新的 Gazebo 世界，它只加载 YAML，并通过 spawner 启动 `joint_state_controller` 和 `hero_chassis_controller`。

如果之前启动过 `simple_chassis_controller`，不要让它继续占用这四个轮子。两个控制器同时申请相同的 `EffortJointInterface` 关节会发生资源冲突。

控制器是否启动成功，可以直接查：

```bash
rosservice call /controller_manager/list_controllers "{}"
```

看到两个控制器均为 `running`，才说明已经进入正常的控制循环。

## 坐标系和轮子顺序

底盘坐标系采用 ROS 的常见约定：前方是 `x` 正方向，左方是 `y` 正方向，绕 `z` 轴逆时针旋转为正。

程序中的轮序不能随意调换：

| 索引 | 轮子 | Gazebo 关节名称 |
| --- | --- | --- |
| 0 | 左前轮 LF | `left_front_wheel_joint` |
| 1 | 右前轮 RF | `right_front_wheel_joint` |
| 2 | 左后轮 LB | `left_back_wheel_joint` |
| 3 | 右后轮 RB | `right_back_wheel_joint` |

逆运动学、正运动学、PID 参数和 `wheel_debug` 都使用这个顺序。只修改 YAML 中的关节排列，而不修改其他对应关系，得到的结果会错位。

## 底盘速度控制

速度命令从 `/cmd_vel` 输入，消息类型为 `geometry_msgs/Twist`。平面运动只使用三个分量：

| 消息字段 | 程序变量 | 作用 |
| --- | --- | --- |
| `linear.x` | `vx` | 前进、后退 |
| `linear.y` | `vy` | 左右横移 |
| `angular.z` | `wz` | 原地旋转 |

例如，以 10 Hz 持续发布一个较慢的前进命令：

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

其他运动只需修改这三个数值：

| 运动 | `linear.x` | `linear.y` | `angular.z` |
| --- | ---: | ---: | ---: |
| 前进 | 0.2 | 0.0 | 0.0 |
| 后退 | -0.2 | 0.0 | 0.0 |
| 左移 | 0.0 | 0.2 | 0.0 |
| 右移 | 0.0 | -0.2 | 0.0 |
| 逆时针旋转 | 0.0 | 0.0 | 0.5 |
| 顺时针旋转 | 0.0 | 0.0 | -0.5 |

测试时按 `Ctrl+C` 结束发布。控制器超过 `cmd_timeout` 没有收到新消息，会把速度命令清零，不需要再补发一条全零消息才能停车。

## 逆运动学

记轮半径为 `r`，并定义：

```text
k = (wheel_base + track_width) / 2
```

按照 LF、RF、LB、RB 的顺序，四个目标轮速为：

```text
ω0 = (vx - vy - k·wz) / r
ω1 = (vx + vy + k·wz) / r
ω2 = (vx + vy - k·wz) / r
ω3 = (vx - vy + k·wz) / r
```

这里的轮速单位是 `rad/s`。得到目标值以后，每个轮子独立计算速度误差：

```text
error = desired_velocity - actual_velocity
effort = PID(error)
```

最后由 `EffortJointInterface::setCommand()` 把 PID 输出写入对应的 Gazebo 关节。机器人能否按预期运动，不只取决于 PID，还和轮序、轮半径、`wheel_base`、`track_width` 是否填写正确有关。



## 四轮 PID 与在线调参

四个轮子各有一套 `control_toolbox::Pid`。这样写并不是为了增加代码量：受负载、摩擦和接触状态影响，四个轮子的响应不会完全一样，共用一个 PID 对象反而难以分别调整。

YAML 使用下面的前缀区分轮子：

```text
front_left_
front_right_
back_left_
back_right_
```

每个前缀下都有 `p`、`i`、`d` 和 `i_clamp`。以左前轮为例：

```yaml
front_left_p: 2.0
front_left_i: 0.0
front_left_d: 0.0
front_left_i_clamp: 1.0
```

控制器运行时，可以打开动态调参界面：

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

在参数树中找到：

```text
/hero_chassis_controller
```

这里修改的是当前运行参数，不会替你编辑 YAML。找到比较合适的数值后，还要手动写回配置文件，否则下次启动仍会使用原来的值。

调参时建议同时打开轮速曲线。先改 P，观察响应是否过慢或者开始振荡，再决定是否需要 D；确实存在稳态误差时，再加入少量 I。只看机器人能动起来，很难判断参数是否合适。

## 用 wheel_debug 查看轮速

为了方便调参，控制器额外发布了：

| Topic | 类型 |
| --- | --- |
| `/hero_chassis_controller/wheel_debug` | `std_msgs::Float64MultiArray` |

一条消息包含 12 个数值：

| 下标 | 含义 |
| --- | --- |
| `data[0..3]` | 目标轮速 |
| `data[4..7]` | 实际轮速 |
| `data[8..11]` | PID 输出力矩 |

三组数据都按 LF、RF、LB、RB 排列。比如观察左前轮，可以画 `data[0]`、`data[4]` 和 `data[8]`：

```bash
rosrun rqt_plot rqt_plot \
'/hero_chassis_controller/wheel_debug/data[0]' \
'/hero_chassis_controller/wheel_debug/data[4]' \
'/hero_chassis_controller/wheel_debug/data[8]'
```

其余轮子的下标如下：

| 轮子 | 目标速度 | 实际速度 | PID 输出 |
| --- | --- | --- | --- |
| LF | `data[0]` | `data[4]` | `data[8]` |
| RF | `data[1]` | `data[5]` | `data[9]` |
| LB | `data[2]` | `data[6]` | `data[10]` |
| RB | `data[3]` | `data[7]` | `data[11]` |

目标速度和实际速度适合放在一起看。目标值突然变化而实际值迟迟跟不上，可能是增益偏小、输出不足，也可能是命令变化本身太快；只盯着力矩曲线反而容易漏掉问题。

## 正运动学与里程计

里程计没有直接读取 Gazebo 中机器人的真实位置，而是使用四个轮子的实际转速计算。这样更接近实际机器人上能够获得的测量数据。

正运动学公式为：

```text
vx = r/4 · (ω0 + ω1 + ω2 + ω3)
vy = r/4 · (-ω0 + ω1 + ω2 - ω3)
wz = r/(4k) · (-ω0 + ω1 - ω2 + ω3)
```

得到机体速度后，程序按控制周期进行积分。平移部分使用中点偏航角：

```text
midpoint_yaw = yaw + wz·dt/2
```

每个周期都会更新 `x`、`y` 和 `yaw`，然后发布两类数据：

| 数据 | 内容 |
| --- | --- |
| `/odom` | `nav_msgs/Odometry` |
| TF | `odom → base_link` |

查看消息：

```bash
rostopic echo /odom
```

也可以直接检查坐标变换：

```bash
rosrun tf tf_echo odom base_link
```

每次调用控制器的 `starting()`，内部位置都会重新从零开始。

轮速里程计并不等同于真实位置。轮子打滑、模型尺寸误差和地面接触变化都会带来偏差，横移时尤其明显。短时间演示通常足够，但不能把它当作绝对定位结果。

## 两种速度模式

`global_mode` 决定 `/cmd_vel` 应该在哪一个坐标系下解释。

### 底盘坐标系模式

```yaml
global_mode: false
```

这是当前默认配置。`linear.x` 始终沿底盘前后方向，`linear.y` 沿左右方向。机器人转过一定角度以后，相同的速度命令也会跟着底盘一起改变方向。

### 全局坐标系模式

```yaml
global_frame: "odom"
global_mode: true
```

切换到该模式后，线速度先被看作 `odom` 下的向量，再通过 `transformVector()` 转到 `base_link`。转换完成后，才进入逆运动学和轮速 PID。

这里处理的是速度向量，不是一个位置点，所以不能把两个坐标系原点之间的平移量加到速度上。`angular.z` 也不随这次平面线速度变换修改。

TF 查询失败时，程序会把线速度清零。未经转换的全局速度不能直接当成底盘速度执行，否则机器人可能朝错误方向运动。

修改模式后需要重新加载控制器。测试前还应确认 `/odom` 和 `odom → base_link` TF 都在正常更新。

## 加速度限制与命令超时

目前的相关配置为：

```yaml
cmd_timeout: 0.5
max_linear_accel: 1.0
max_angular_accel: 2.0
```

加速度限制不是简单地截断最大速度，而是限制每个控制周期内命令能改变多少：

```text
max_delta = max_accel · dt
```

`linear.x`、`linear.y` 和 `angular.z` 分别进行这项处理。这样，键盘或测试命令中的速度阶跃不会立刻变成轮速 PID 的阶跃输入。

需要注意，这里限制的是命令变化率，并不保证 Gazebo 中机体的实际加速度永远不超过该数值。实际响应还会受到 PID、轮地接触和仿真物理参数影响。

`cmd_timeout` 用来处理命令发布者退出的情况。控制器从未收到命令，或者最近一条命令已经超过超时时间，都会将命令清零。

## 配置文件里的主要参数

这些参数位于 `config/hero_chassis_controller.yaml`：

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

轮半径、轴距和轮距最好从模型中核对。前进时四个轮子都在转，并不能证明这些参数正确；到了横移和旋转测试，尺寸误差和轮序错误通常才会明显暴露出来。

## 控制器是怎样进入 Gazebo 的

这个包没有一个需要通过 `rosrun` 启动的独立控制节点。真正的可执行控制器以插件形式编译成动态库，再由 Gazebo 中的 `controller_manager` 加载。

整条加载链是：

```text
PLUGINLIB_EXPORT_CLASS
→ hero_chassis_controller_plugins.xml
→ package.xml 中的 controller_interface export
→ YAML 中的控制器 type
→ controller_manager/spawner
→ gzserver 周期调用控制器 update()
```

YAML 中指定的插件名称为：

```text
hero_chassis_controller/HeroChassisController
```

类本身继承自：

```cpp
controller_interface::Controller<
    hardware_interface::EffortJointInterface>
```

所以控制器列表为空时，反复执行 `rosrun` 没有用。应该检查插件是否导出、参数是否加载、spawner 是否运行，以及 `gzserver` 终端有没有相关错误。

## 键盘控制

安装 `teleop_twist_keyboard` 后，可以用键盘发布 `/cmd_vel`：

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

键盘节点只负责产生速度消息，不直接驱动轮子。后面的坐标变换、逆运动学、四轮 PID 和里程计仍由本控制器处理。

## 验证记录

目前已经完成的检查包括：

- `catkin build hero_chassis_controller` 构建通过；
- pluginlib 能加载插件，controller_manager 能启动控制器；
- 基础前进命令可以执行；
- `wheel_debug` 有数据输出；
- `/odom` 有数据输出；
- 停止持续发布命令后，基础停车流程可以执行。

正式录视频前，还需要把下面几项分别跑一遍，并保留曲线或终端结果：

- 后退、左右横移和原地旋转；
- `rqt_plot` 中的目标轮速和实际轮速；
- `rqt_reconfigure` 在线修改 PID；
- `tf_echo odom base_link`；
- `global_mode: true` 时的全局速度控制；
- `teleop_twist_keyboard` 键盘操作；
- 加速度限制和命令超时的实际表现。

代码里写了某个功能，只能说明实现已经加入，不能代替运行验证。后续每完成一项测试，都应该更新这里，避免 README 与仓库实际状态脱节。

## 常见问题

### 提示 `Unable to communicate with master`

通常是 ROS Master 或 Gazebo 还没启动。先运行 `rm_description_for_task` 的 `hero.launch`，再启动控制器，不必因此重新编译代码。

### 查询不到控制器

`hero.launch` 不负责加载本包。模型出现以后，还需要另外执行：

```bash
roslaunch hero_chassis_controller hero_chassis_controller.launch
```

### 启动时发生关节资源冲突

检查示例 `simple_chassis_controller` 是否仍在运行。同一组 `EffortJointInterface` 关节不能同时被两个控制器占用。

### 在线调好的 PID 重启后消失

这是正常现象。`rqt_reconfigure` 修改的是运行参数，不会自动保存文件。确认数值后，手动写回 `config/hero_chassis_controller.yaml`。

## 模板与参考资料

- [rm_template](https://github.com/gdut-dynamic-x/rm_template)
- [simple_chassis_controller](https://github.com/gdut-dynamic-x/simple_chassis_controller)
- [rm_description_for_task](https://github.com/gdut-dynamic-x/rm_description_for_task)
- [ros_control](http://wiki.ros.org/ros_control)
- [pluginlib](http://wiki.ros.org/pluginlib)
- [control_toolbox](http://wiki.ros.org/control_toolbox)
- [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure)
- [tf](http://wiki.ros.org/tf)

## Git 开发记录

这个包的初始化、文档修改、分支合并以及 README 冲突解决都保留在 Git 历史中。可以通过下面的命令查看：

```bash
git log --oneline --graph --decorate --all
```

后续提交尽量按实际修改内容拆分，不要把运动学、里程计、调参记录和文档更新全部塞进同一个 commit。例如：

```text
Add mecanum inverse kinematics.
Add wheel odometry and TF.
Verify chassis and global velocity modes.
Update controller usage and test results.
```

