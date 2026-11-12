# myCobot 280 M5 六关节键盘控制包

本包替代键盘客户端 `ros2 run mycobot_280 teleop_keyboard`。继续使用已经验证过的
`teleop_keyboard.launch.py` 启动唯一实机驱动、真实 JointState、TF 与 RViz。

本包只订阅 `/joint_states`、调用 `/set_angles`，不打开串口、不发布关节状态。
代码有中文注释；默认点按一次 1°、速度参数 10，等真实反馈到位并停稳后才能继续。

## 1. 按键

在 Ubuntu 键盘控制终端中点按，不需要回车。使用英文输入法。

| 顺时针键 | 逆时针键 | 控制对象 |
| --- | --- | --- |
| q | a | 关节 J1 |
| w | s | 关节 J2 |
| e | d | 关节 J3 |
| r | f | 关节 J4 |
| t | g | 关节 J5 |
| y | h | 关节 J6 |

| 按键 | 功能 |
| --- | --- |
| 1 | Go to init pose：默认六关节全零；5 秒内按回车确认执行 |
| 2 | Go to home pose：5 秒内按回车确认执行 |
| 3 | Resave home pose：将已停稳的当前真实六关节角保存为 home，不移动机械臂 |
| 4 | Quit：退出本客户端；不会关闭驱动或释放电机 |

**q 现在是 J1 点动键；g、h 现在也是关节点动键，不是夹爪开关。**
本程序没有继承旧客户端的夹爪、吸泵按键，以免与用户指定的键位冲突。

方向定义：程序默认 `cw_signs=[-1,-1,-1,-1,-1,-1]`，即顺时针键让相应的
关节角读数减小，逆时针键让读数增大。“顺时针”取决于观察转轴的方向；
通常从关节正轴端看向关节原点描述旋转。首次使用必须逐关节以小步核对。
如果与你规定的观察方向相反，仅将该关节的 cw_signs 元素从 -1 改为 +1。
没有在实机核对过的方向不能被当作已经验证。

## 2. 文件放置：Windows -> Ubuntu

现在也可以直接从公开 GitHub 仓库下载，不必使用 VMware 文件复制：

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/eryue30/mycobot_joint_keyboard.git
```

如果目标目录已经存在，先检查内容，不要覆盖或删除你已有的修改。
已通过 git clone 下载且没有本地改动的副本，可以用
`git -C ~/colcon_ws/src/mycobot_joint_keyboard pull --ff-only` 更新，然后重新编译。

Windows 源目录：`E:\codex_workarea\mycobot\mycobot_joint_keyboard`。

将整个 `mycobot_joint_keyboard` 文件夹通过你已可用的 VMware 共享文件夹或文件复制方式，
复制到 Ubuntu 的 `~/colcon_ws/src/`，最终应为：

```text
~/colcon_ws/src/mycobot_joint_keyboard/
  package.xml
  setup.py
  setup.cfg
  README_中文.md
  config/keyboard.yaml
  resource/mycobot_joint_keyboard
  mycobot_joint_keyboard/
    __init__.py
    core.py
    keyboard_node.py
    terminal.py
  tools/patch_zero_feedback.py
  tests/...
```

不要把 package.xml 多套一层目录。Ubuntu 检查：

```bash
ls "$HOME/colcon_ws/src/mycobot_joint_keyboard/package.xml"
```

## 3. 修复原驱动的“零位不发布”条件

这是为了让按 1 回到全零姿态之后仍有真实反馈，能够继续点动和生成 TF。
你工作区现有 `listen_real_service.py` 会跳过前三关节全为零的反馈。
本包不使用 `/get_angles` 的返回值替代连续反馈，因为旧 GetAngles 服务没有有效状态字段。

先让实机停稳，按 4 关闭新客户端；如旧客户端仍在运行也要退出。然后 Ctrl+C 关闭
原来的实机 launch。以下操作只修改源码，不连接机械臂：

```bash
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash

export JOINT_KEYBOARD_DIR="$HOME/colcon_ws/src/mycobot_joint_keyboard"
export MYCOBOT_DRIVER="$HOME/colcon_ws/src/mycobot_ros2/mycobot_280/mycobot_280/mycobot_280/listen_real_service.py"

ls "$MYCOBOT_DRIVER"
python3 "$JOINT_KEYBOARD_DIR/tools/patch_zero_feedback.py" "$MYCOBOT_DRIVER"
```

补丁只删除 `angles[0:3] == [0.0, 0.0, 0.0]` 这一项过滤，保留原来的空值、类型与长度检查。
它会生成同目录备份 `listen_real_service.py.before_joint_keyboard.bak`，并读回验证。
重复运行时会提示已经修复。版本不匹配、目标不唯一或备份冲突时拒绝修改；不要强行替换。

若 Ubuntu 源码不在上面的路径，先定位：

```bash
find "$HOME/colcon_ws/src" -path '*/mycobot_280/mycobot_280/listen_real_service.py' -print
```

此时把 `MYCOBOT_DRIVER` 改为实际的 M5 驱动路径，不要误用 pi、jn 或其他机型文件。
本次交付未直接修改 Windows 上的原驱动；运行修补脚本才会修改你指定的目标文件。

## 4. 编译

```bash
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
cd ~/colcon_ws

# 原来的 mycobot_interfaces、mycobot_description 应已安装在当前工作区。
ros2 pkg prefix mycobot_interfaces
ros2 pkg prefix mycobot_description

colcon build --packages-select mycobot_280 mycobot_joint_keyboard
source ~/colcon_ws/install/setup.bash
ros2 pkg executables mycobot_joint_keyboard
```

最后应包含：`mycobot_joint_keyboard joint_keyboard`。
如提示找不到依赖，先解决原工作区的接口包或描述包构建问题，再编译本包。

## 5. 终端 A：启动唯一实机驱动

不要同时运行旧键盘客户端、slider、simple_gui、MoveIt 实机执行节点或独立串口脚本。
你已验证的串口是 `/dev/ttyACM0`，波特率 115200：

```bash
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash

ros2 launch mycobot_280 teleop_keyboard.launch.py \
  port:=/dev/ttyACM0 baud:=115200
```

这个 launch 继续保留；本次替换的是另外运行的键盘客户端，不是驱动 launch。

## 6. 终端 B：运行新键盘客户端

```bash
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash

ros2 run mycobot_joint_keyboard joint_keyboard
```

启动不会移动机械臂，会先检查真实反馈与服务来源，随后打印当前角度、home 与按键表。
点击终端 B 获得焦点；单按 q，等打印“真实反馈已到位并停稳”后再按其他键。
首次请逐轴核对 q/a、w/s、e/d、r/f、t/g、y/h 的实际方向，确认相机线缆有余量。

这是点按点动，不是按住持续转动。执行中输入的运动键会丢弃；长按的自动重复也会受到抑制。
手眼采样时，在本工具显示到位之后仍按原采样要求停稳至少 2 秒，再到图像窗口冻结/保存。

所有节点应使用相同的 ROS_DOMAIN_ID 与 RMW_IMPLEMENTATION。不要只改变其中一个终端。

## 7. 调整步长、速度与方向

以下示例每次 0.5°、速度参数 10：

```bash
ros2 run mycobot_joint_keyboard joint_keyboard --ros-args \
  -p step_deg:=0.5 \
  -p speed:=10
```

如仅 J2 观察到的顺时针与预期相反，将第二项改为 +1：

```bash
ros2 run mycobot_joint_keyboard joint_keyboard --ros-args \
  -p step_deg:=0.5 \
  -p 'cw_signs:=[-1,1,-1,-1,-1,-1]'
```

退出前一次客户端再运行新命令，不要并开两个。方向配置通过命令行传入时只对本次运行有效。
需要长期保存，可编辑本包 `config/keyboard.yaml`，重新构建后使用：

```bash
ros2 run mycobot_joint_keyboard joint_keyboard --ros-args \
  --params-file "$(ros2 pkg prefix mycobot_joint_keyboard)/share/mycobot_joint_keyboard/config/keyboard.yaml"
```

本工具接受步长 0.1～5.0°、速度整数 1～30；参数速度是固件接口值，不代表度/秒。
当前 ROS 参数不支持在运行中热更新，修改后应退出重启。

## 8. init 与 home

默认 init 为 `[0,0,0,0,0,0]`；它只是关节零角姿态，不是寻找机械限位或编码器校零。
若相机支架或夹爪使该姿态不合适，不要按 1；可以明确配置已验证的 init_angles：

```bash
# 这里仅演示参数格式：六个数必须换成你已验证可达且路径无碰撞的姿态。
ros2 run mycobot_joint_keyboard joint_keyboard --ros-args \
  -p 'init_angles:=[0.0,0.0,0.0,0.0,0.0,0.0]'
```

没有 home 文件时，home 使用这一次启动时停稳的实际姿态；不会套用旧客户端的固定 home。
将机械臂调整到希望保存的位置，停稳后按 3，保存到：

```text
~/.ros/mycobot_joint_keyboard/home.json
```

下次启动自动读这个文件，按 2 回该姿态。按 3 会替换以前保存的 home。
文件异常或角度越界时拒绝使用，不能静默变成六个零。

按 1/2 会先显示目标，5 秒内按回车执行。中途按其他功能键会取消确认。
大幅回位使用回车单独确认，避免将长按 1/2 产生的键盘自动重复误认为主动确认。
回位按相邻分段目标每关节最多 2° 插值，每段都等待真实反馈；但它仍是关节空间插值，
不包含碰撞检测，不保证夹爪或相机的空间路径是直线。先确认整条动作的活动空间。

## 9. 软件拒绝条件和退出的含义

- `/joint_states` 必须只有一个发布者，默认名为 `listen_real_service`，且包含六个正确的关节名。
- `/set_angles` 必须由同一驱动唯一提供。新客户端自身不发布 `/joint_states`。
- 反馈接收时间和源时间必须新鲜；运动前、每段到位时都要看到连续停稳反馈。
- 目标必须在 M5 URDF 与本工具附加范围的交集内，并留出默认 2° 余量。软件范围不替代固件限位。
- 不用模 360° 绕回关节角，不会把越界目标裁剪到另一端。
- 服务失败、超时、动作中反馈丢失或不到位时，停止发送后续段并锁住新的运动请求。
  检查实机停止和反馈恢复后，按 4 退出并重新启动；程序不会自动重发。

**4 和 Ctrl+C 仅退出客户端，无法保证停止已下发的运动。** 现有驱动没有提供停止服务，
本包也不会另开串口抢占控制或通过释放电机充当停止。需要停止实机时使用你已经确认的硬件停止方式。

如启动提示当前位置已超出本工具保守范围，先使用你已验证的控制方式回到允许范围，再启用本工具；
不要为了启动成功盲目放宽范围。若反馈名因启动方式而变更，应先确认仍是唯一真实反馈源，
再设置 `expected_feedback_node`；不要指向 GUI 或模拟器。

## 10. 离线检查与现场验收

在包目录运行离线测试，不连接硬件，也不会调用真实 ROS 服务：

```bash
cd ~/colcon_ws/src/mycobot_joint_keyboard
python3 -m unittest discover -s tests -v
```

测试覆盖按键与六关节对应关系、方向翻转、角度范围、分段、home、源时间戳、
反馈中断和移动状态，以及使用内存 ROS 替身验证服务失败/超时不重发、flag=True 不等于到位。
它不等同于 ROS Humble 真正的进程通信、终端、串口或硬件试验。

现场先做每关节一次小步及其反向一步，核对实机与 RViz，再检查：

```bash
ros2 topic info /joint_states --verbose
ros2 run tf2_ros tf2_echo g_base joint6_flange
```

应仍只有真实驱动一个关节状态发布者，TF 随实机变化。验证 init/home 前先确认完整路径的空间。
本次本地验证不声称已在你的 Ubuntu、VMware 或机械臂上执行。

2026-11-12 本地验证记录：43 项离线测试通过；Python 源码语法检查和 setuptools 包名/版本检查通过。
测试中的 ROS 对象是内存替身，未执行真正的 colcon/ROS 进程通信或实机运动。
