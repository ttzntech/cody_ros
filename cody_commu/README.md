# cody_commu

@pluto

## 简介

Can设备负责不同设备之间的通信,通信方式是在不同的波(id)上发送固定长度的二进制串.
本节点是基于can转串口设备USB-CAN-V3开发的ros1 noetic驱动程序,实现上层控制接口与下层电控接口的对接。

## package架构

```
cody_commu
.
├── chassis_driver
│   ├── CMakeLists.txt
│   ├── include
│   │   └── chassis_driver
│   │       ├── cody_driver.h
│   │       └── utils.h
│   ├── launch
│   │   └── cody.launch
│   ├── package.xml
│   ├── rviz
│   │   └── my_rviz.rviz
│   ├── src
│   │   ├── chassis_driver.cpp
│   │   └── chassis_driver_node.cpp
│   └── third_party
│       └── CSerialPort-4.3.1
├── move_base_bridge
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src
│       └── move_base_bridge.cpp
├── README.md
└── topic_pub.sh
```

## ID通道

+ **CAN1**

## 消息格式

vel_sub = nh.subscribe<cody_msgs::CtrlCmd>("/cmd_vel", 10, &CODY::sub_command,this);

cmd_feed_pub = nh.advertise<cody_msgs::CtrlCmd>("/cmd_feed", 1);

odom_feed_pub = nh.advertise<cody_msgs::OdomFb>("/odom_feed", 1);

motor_feed_pub = nh.advertise<cody_msgs::MotorMotionFb>("/motor_feed", 1);

pose_feed_pub = nh.advertise<nav_msgs::Odometry>("/pose_feed", 1);

### 话题与消息格式

| 序号  | 消息格式                    | 话题名                | 描述          |
| --- | ----------------------- | ------------------ | ----------- |
| 1   | cody_msgs/CtrlCmd       | /cmd_vel           | 订阅控制节点发布的命令 |
| 2   | cody_msgs/IoCmd         | /cmd_io            | 订阅上层选定的灯光   |
| 3   | cody_msgs/SystemStateFb | /sys_feed          | 显示回馈的状态信息   |
| 4   | cody_msgs/OdomFb        | /odom_feed         | 显示反馈的里程信息   |
| 5   | cody_msgs/BmsFb         | /bms_feed          | 显示反馈的bms信息  |
| 6   | cody_msgs/MotorMotionFb | /motor_motion_feed | 显示反馈的电机信息   |
| 7   | cody_msgs/MotorSpeed    | /motor_speed_feed  | 显示反馈的电机速度   |

## Note

## dependence

- ubuntu 22.04 
- ros humble
- CSerialPort
- yaml-cpp

#### 安装CSerialPort

```
cd catkin_ws/src/can_ros2/third_party/CSerialPort-4.3.1
mkdir build&&cd build
cmake ..
make
sudo make install
```

#### 安装yaml,从源码编译

```
进入https://github.com/jbeder/yaml-cpp/releases

下载最新的yaml-cpp源码
解压
cd yaml-0.8.0
mkdir build&&cd build
cmake ..
make
sudo make install

check一下libyaml.a是不是在/usr/local/lib路径下。
```

## 运行

### 编译

```
cd ~/catkin_ws
colcon build --packages-select can_ros2
```

### 运行节点

```
cd ~/catkin_ws
source ./install/setup.zsh(setup.bash)
ros2 run can_ros2 can_ros2_node
```

```
设置单次串口权限
sudo chmod 777 /dev/ttyCH341USB0

设置永久串口权限
whoami
sudo usermod -aG dialout ${your_name}
```

## 配置

| 序号  | 参数                | 描述               |
| --- | ----------------- | ---------------- |
| 1   | pose_cal_use_odom | 积分计算位姿或轮速里程计计算位姿 |
|     |                   |                  |
|     |                   |                  |
|     |                   |                  |
