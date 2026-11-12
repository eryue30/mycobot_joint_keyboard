# myCobot 280 M5 六关节键盘控制

适用于 Ubuntu 22.04、ROS 2 Humble 和已有 `mycobot_ros2` 工作区。
这是一个独立的 ROS 2 Python 功能包，用于替代原来的 `teleop_keyboard` 客户端。

详细安装、原驱动零位反馈修复、方向调整和常见问题请阅读 **[中文操作说明](README_中文.md)**。

## 按键

| 关节 | 顺时针 | 逆时针 |
| --- | --- | --- |
| J1 | q | a |
| J2 | w | s |
| J3 | e | d |
| J4 | r | f |
| J5 | t | g |
| J6 | y | h |

- `1` + 回车：前往 init，默认六个关节均为 0°。
- `2` + 回车：前往 home；首次没有保存文件时使用启动姿态。
- `3`：把停稳的当前姿态保存为 home。
- `4`：退出客户端。

默认每次点按 1°，速度参数 10。顺时针键默认使对应关节读数减小；
实际观察方向应逐关节小步核对，可通过 `cw_signs` 单独反转。

## 在 Ubuntu 下载与编译

下面命令将仓库下载到现有 colcon 工作区。目标文件夹若已存在，不要删除已有内容，
先检查它是否就是本仓库的克隆。

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/eryue30/mycobot_joint_keyboard.git

source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash

# 先停稳机械臂并关闭旧实机 launch；此脚本只修补源码并备份，不发送运动。
python3 ~/colcon_ws/src/mycobot_joint_keyboard/tools/patch_zero_feedback.py \
  ~/colcon_ws/src/mycobot_ros2/mycobot_280/mycobot_280/mycobot_280/listen_real_service.py

cd ~/colcon_ws
colcon build --packages-select mycobot_280 mycobot_joint_keyboard
source ~/colcon_ws/install/setup.bash
```

原驱动路径若不同，请按中文说明定位实际文件，不要修改其他机型的驱动。

## 使用现有实机驱动

本包通过 `/set_angles` 下发目标，通过 `/joint_states` 检查真实到位，
不额外建立机械臂串口连接，也不发布模拟关节状态。

安装前需要已有的 `mycobot_interfaces` 与 `mycobot_description`。
初次使用应按中文说明修复旧驱动在零位不发布反馈的问题，再编译本包。

终端 A，启动唯一实机驱动：

```bash
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 launch mycobot_280 teleop_keyboard.launch.py \
  port:=/dev/ttyACM0 baud:=115200
```

终端 B，启动新键盘客户端：

```bash
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 run mycobot_joint_keyboard joint_keyboard
```

## 验证与限制

本包包含 43 项离线测试，覆盖按键映射、角度范围、反馈时间与服务异常等情况。
测试中的 ROS 对象使用内存替身，不表示已通过真实 ROS 通信或硬件验证。

程序包含软件范围检查、真实反馈到位检查和回位分段，但不包含碰撞规划。
`4` 和 Ctrl+C 仅退出客户端，不能保证停止已下发的运动。

在包目录运行离线测试：

```bash
python3 -m unittest discover -s tests -v
```
