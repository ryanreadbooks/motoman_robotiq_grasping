# 安川MA2010机械臂 + Robotiq 2F-140夹爪视觉抓取 (持续更新...)

## 环境
**Ubuntu16.04 + ROS kinetic**
## 安装

### 前置条件
  1. 安装[ROS](https://wiki.ros.org/kinetic/Installation/Ubuntu)
  2. 安装[realsense-ros](https://github.com/IntelRealSense/realsense-ros)

### 安装本仓库
1. 创建工作空间
    ```bash
    mkdir -p demo_ws/src
    cd demo/src
    catkin_init_workspace
2. 将项目clone下来
   ```bash
   git clone --recursive https://github.com/ryanreadbooks/motoman_robotiq_grasping.git
3. 相关依赖的安装
    ```bash
    cd ..
    rosdep install --from-paths src --ignore-src -y -r
4. 使用catkin build编译
    ```bash
    # 由于项目使用python3,所以需要指定python3路径进行编译，向下面这样，如果你是其它路径，则替换下面三个路径
    catkin config -DPYTHON_EXECUTABLE=/home/ryan/Apps/miniconda3/bin/python -DPYTHON_INCLUDE_DIR=/home/ryan/Apps/miniconda3/include/python3.7m -DPYTHON_LIBRARY=/home/ryan/Apps/miniconda3/lib/libpython3.7m.so
    # 指定不进行安装
    catkin config --no-install 
    # 用catkin build进行编译
    catkin build -DSETUPTOOLS_DEB_LAYOUT=OFF    
    