<p align="center"><strong>tita_description</strong></p>
<p align="center"><a href="https://github.com/${YOUR_GIT_REPOSITORY}/blob/main/LICENSE"><img alt="License" src="https://img.shields.io/badge/License-Apache%202.0-orange"/></a>
<img alt="language" src="https://img.shields.io/badge/language-c++-red"/>
<img alt="platform" src="https://img.shields.io/badge/platform-linux-l"/>
</p>
<p align="center">
    语言：<a href="./docs/docs_en/README_EN.md"><strong>English</strong></a> / <strong>中文</strong>
</p>

主要存储机器人描述文件，包括单不限于tita的模型，同时存在launch文件来可视化机器人模型
将以下两个文件夹复制到指定位置, 便于webots及rviz使用

```
sudo mkdir -p /usr/share/robot_description
sudo cp -r src/tita_locomotion/tita_description/tita /usr/share/robot_description/
sudo cp -r src/tita_locomotion/tita_description/tower /usr/share/robot_description/
```

## Basic Information

| Installation method | Supported platform[s]      |
| ------------------- | -------------------------- |
| Source              | Jetpack 6.0 , ros-humble |

------

## Build Package

```bash
# if have extra dependencies
# apt install ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui
colcon build --packages-up-to tita_description
source install/setup.bash
# display tita with rviz
ros2 launch tita_description tita_display.launch.py
# display airbot with rviz
ros2 launch tita_description airbot_play_display.launch.py
```

### some help:
```
ros2 run tf2_tools view_frames #查看tf树，会生成frams.pdf，打开该文件查看TF树
ros2 node list | grep tf #用以下命令查看正在运行的TF发布节点
```
注意urdf和xacro里的joint_limit要写完整，不然rviz显示会有问题（报错：No transform from [left_wheel_link] to [base_link]，例如这种）
