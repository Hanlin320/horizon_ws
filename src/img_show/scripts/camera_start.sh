#!/bin/bash

# 设置工作空间路径（请根据实际情况修改）
WORKSPACE_DIR=~/Desktop/horizon_ws  # 你可以根据实际路径修改这里

# 进入工作空间目录
cd $WORKSPACE_DIR

# 设置 ROS 2 环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 运行 camera_catch 节点
echo "Running camera_catch node..."
ros2 run img_show camera_catch
