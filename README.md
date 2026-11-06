# hero_chassis_controller

基于 `rm_template` 创建的英雄机器人麦克纳姆轮底盘控制器 ROS 包。

## 开发环境

- ROS Noetic
- Gazebo 11
- catkin-tools
- `rm_description_for_task`
- `hardware_interface::EffortJointInterface`

## 当前进度

完成 ROS 包的基本配置，包括：

- `package.xml` 中的包信息和依赖；
- `CMakeLists.txt` 中的编译、动态调参与安装配置；
- 控制器插件和核心功能正在开发中。

## 构建方式

核心源码完成后，在工作空间中使用 catkin-tools 构建：

```bash
cd /root/catkin_ws
catkin build hero_chassis_controller
source /root/catkin_ws/devel/setup.bash
