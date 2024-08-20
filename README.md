# ROS Packages for Cody Mobile Base
```
                          $$\                                               
                          $$ |                                              
 $$$$$$$\  $$$$$$\   $$$$$$$ |$$\   $$\        $$$$$$\   $$$$$$\   $$$$$$$\ 
$$  _____|$$  __$$\ $$  __$$ |$$ |  $$ |      $$  __$$\ $$  __$$\ $$  _____|
$$ /      $$ /  $$ |$$ /  $$ |$$ |  $$ |      $$ |  \__|$$ /  $$ |\$$$$$$\  
$$ |      $$ |  $$ |$$ |  $$ |$$ |  $$ |      $$ |      $$ |  $$ | \____$$\ 
\$$$$$$$\ \$$$$$$  |\$$$$$$$ |\$$$$$$$ |      $$ |      \$$$$$$  |$$$$$$$  |
 \_______| \______/  \_______| \____$$ |      \__|       \______/ \_______/ 
                              $$\   $$ |                                    
                              \$$$$$$  |                                    
                               \______/  
```
[English Version](./READNE-en.md)

## 功能包
- `cody_chassis` : cody 底盘控制功能包，主要接受和发布底层CAN信息。并发布自身里程计到 `/odom`话题下。 
- `cody_control` : cody 控制功能包，实现一些控制算法。将控制命令发布到`/cmd_vel`话题下。
- `cody_msgs` : cody 自定义消息类型。
- `cody_startup` : 一些 launch file 和其他相关启动配置。

## 依赖及前置配置
- 请确保在运行项目之前安装上 `ttzn_sdk`底层驱动。若没安装，请参考该 [README](https://github.com/ttzntech/ttzn_sdk/#安装底盘驱动) 进行该驱动的安装。
- 请确保初始化了CAN通讯设备，具体请参考 [CAN 设备初始化](https://github.com/ttzntech/ttzn_sdk/#can-设备初始化) 进行初始化。

## 发布接受话题及参数
### 发布话题
以下具体消息类型请查阅 `cody_msgs` 功能包。各参数含义请查阅 cody 用户使用手册。
- `/cody/sys_status_fb` : 发布CAN底层反馈的系统状态。
- `/cody/move_ctrl_fb` : 发布CAN底层反馈的速度、转角。
- `/cody/motor_info_fb` : 发布CAN底层返回的左右电机状态。
- `/cody/odom_fb` : 发布CAN底层反馈的左右电机里程计。\
*注意 : 该odom是左右轮的总行程，不为常规 odom。* 
- `/odom` : 根据速度等信息和自身模型，计算出 odom 并发布。

### 接受话题
- `/cmd_vel` : 接受控制信息，并通过CAN下发底盘。\
*注意 : 上述的 `/cmd_vel` 接收的消息类型为自定义的 `cody_msgs/MoveCtrl`。* 
- `cody/mode_ctrl` : 接受运动模式信息，并通过CAN下发底盘。
- `cody/light_ctrl` : 接受前后灯控制信息，并通过CAN下发底盘。

### 参数
- `cody/pub_tf` : 是否发布 tf 变换。
- `cody/base_frame` : tf 变换 base frame 名称，默认 `map`。
- `cody/odom_frame` : tf 变换 odom frame 名称，默认 `odom`。

## 基础使用方法
### 编译该项目
```bash
cd <your_catkin_ws>/src
git clone https://github.com/ttzntech/cody_ros.git
cd ..
catkin_make
```
### 设置环境变量
```bash
source devel/setup.bash
```
### 示例
1. 启动 cody_chassis 节点
```bash
roslaunch cody_chassis cody_chassis.launch
```
2. 启动 cody_chassis 节点和键盘控制节点 teleop 
```bash
roslaunch cody_startup cody_teleop.launch
```
---
Copyright &copy; 2023 [威海天特智能科技有限公司](http://ttzntech.com/)