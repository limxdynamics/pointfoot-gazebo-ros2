# pointfoot-gazebo-ros2



## 1. 搭建开发环境

安装ROS 2 Iron：在 Ubuntu 22.04 操作系统上建立基于 ROS 2 Iron 的算法开发环境。 安装请参考文档： https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html ，选择“ros-iron-desktop”进行安装。ROS 2 Iron 安装完成后，Bash 终端输入以下 Shell 命令，安装开发环境所依赖的库：

```
sudo apt update
sudo apt install ros-iron-urdf \
             ros-iron-urdfdom \
             ros-iron-urdfdom-headers \
             ros-iron-kdl-parser \
             ros-iron-hardware-interface \
             ros-iron-controller-manager \
             ros-iron-controller-interface \
             ros-iron-controller-manager-msgs \
             ros-iron-control-msgs \
             ros-iron-controller-interface \
             ros-iron-gazebo-* \
             ros-iron-rviz* \
             ros-iron-rqt-gui \
             ros-iron-rqt-robot-steering \
             ros-iron-plotjuggler* \
             ros-iron-control-toolbox \
             ros-iron-ros2-control \
             ros-iron-ros2-controllers \
             ros-dev-tools \
             cmake build-essential libpcl-dev libeigen3-dev libopencv-dev libmatio-dev \
             python3-pip libboost-all-dev libtbb-dev liburdfdom-dev liborocos-kdl-dev -y
```

## 2. 创建工作空间

可以按照以下步骤，创建一个算法开发工作空间：

- 打开一个 Bash 终端。

- 创建一个新目录来存放工作空间。例如，可以在用户的主目录下创建一个名为“limx_ws”的目录：

  ```
  mkdir -p ~/limx_ws/src
  ```

- 下载机器人模型描述文件

  ```
  cd ~/limx_ws/src
  git clone https://github.com/limxdynamics/robot-description.git
  ```

- 下载运动控制开发接口：

  ```
  cd ~/limx_ws/src
  git clone https://github.com/limxdynamics/pointfoot-sdk-lowlevel.git
  ```
- 下载可视化工具
    ```Bash
    cd ~/limx_ws/src
    git clone https://github.com/limxdynamics/robot-visualization.git
    ```

- 下载 Gazebo 仿真器：

  ```
  cd ~/limx_ws/src
  git clone https://github.com/limxdynamics/pointfoot-gazebo-ros2.git
  ```

- 编译工程：

  ```
  cd ~/limx_ws
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
  ```

- 选择机器人类型

  - 通过 Shell 命令 `tree -L 1 src/robot-description/pointfoot ` 列出可用的机器人类型：

    ```
    src/robot-description/pointfoot
    ├── PF_P441A
    ├── PF_P441B
    ├── PF_P441C
    └── PF_P441C2
    ```

  - 以`PF_P441C`（请根据实际机器人类型进行替换）为例，设置机器人型号类型：

    ```
    echo 'export ROBOT_TYPE=PF_P441C' >> ~/.bashrc && source ~/.bashrc
    ```

- 运行仿真：可以设置empty_world.launch.py文件的`use_support`参数为 `true`，执行下面Shell命令运行仿真：

  ```
  source install/setup.bash
  ros2 launch pointfoot_gazebo empty_world.launch.py
  ```

- 运行控制例程，确保仿真器中机器人有运动，说明仿真环境搭建完成：

  ```
  source install/setup.bash
  ros2 run pointfoot_sdk_lowlevel pf_groupJoints_move 127.0.0.1
  ```
  ![](doc/simulator.gif) 

  
