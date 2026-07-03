# AP + Gazebo + MAVROS 仿真安装操作清单

参考资料：

- DroneChina 帖子：<https://dronechina.net/t/topic/1046/4>
- ArduPilot 官方 Gazebo Classic legacy 文档：<https://ardupilot.org/dev/docs/sitl-with-gazebo-legacy.html>

目标：

```text
机载电脑 Ubuntu 20.04 + ROS Noetic + MAVROS
安装 ArduPilot SITL + Gazebo Classic 11 + ardupilot_gazebo 插件
用于测试 AP Guided 模式 Python 控制脚本
```

---

## 0. 先确认环境

在机载电脑终端执行：

```bash
lsb_release -a
rosversion -d
rospack find mavros
df -h
free -h
gazebo --version
```

期望：

```text
Ubuntu: 20.04
ROS: noetic
MAVROS: 能找到路径
Gazebo: 如果没装会提示 command not found；如果装了，一般是 Gazebo 11.x
磁盘: 建议至少空 10GB
内存: 建议至少 4GB；更低也能试，但编译容易慢或被杀
```

你当前机载电脑检查结果：

```text
Ubuntu: 20.04.6 LTS focal
ROS: noetic
MAVROS: /opt/ros/noetic/share/mavros
Gazebo: 11.11.0
磁盘: / 分区 468G，总空余约 89G
内存: 15Gi，总可用约 13Gi
交换分区: 2.0Gi
```

结论：

```text
Gazebo 本体已经装好，不需要重复安装 gazebo11。
下一步重点是安装 ArduPilot SITL 和 ardupilot_gazebo 插件。
这台机载电脑的磁盘和内存比之前虚拟机宽裕很多，可以正常编译，但仍建议 make -j2 起步。
```

---

## 1. 安装基础工具和 Gazebo 依赖

```bash
sudo apt update
sudo apt install -y git wget curl build-essential cmake python3-pip python3-rosdep python3-catkin-tools
sudo apt install -y gazebo11 libgazebo11-dev ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

测试 Gazebo：

```bash
gazebo --version
gazebo --verbose
```

如果弹出一个空世界，说明 Gazebo 本体正常。测试完可以关闭 Gazebo。

---

## 2. 安装 ArduPilot 源码和 SITL 环境

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git --recursive
cd ~/ardupilot
git submodule update --init --recursive
```

安装 ArduPilot 依赖：

```bash
cd ~/ardupilot/Tools/environment_install
./install-prereqs-ubuntu.sh -y
```

执行完以后，注销重登或重启。推荐直接重启：

```bash
sudo reboot
```

重启后测试 SITL：

```bash
source ~/.bashrc
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map
```

如果能看到 MAVProxy 控制台、地图，说明 ArduPilot SITL 基础环境正常。

退出：

```text
在 MAVProxy 里输入：exit
```

---

## 3. 安装 ardupilot_gazebo 插件

DroneChina 帖子使用 SwiftGust 仓库，示例多；官方文档使用 khancyr 仓库，插件本质相同。这里先采用帖子里的 SwiftGust 路线，方便后续直接跑示例世界。

```bash
cd ~
git clone https://github.com/SwiftGust/ardupilot_gazebo.git
cd ~/ardupilot_gazebo
mkdir -p build
cd build
cmake ..
make -j2
sudo make install
```

说明：

```text
如果机载电脑内存较小，make 不要用 -j8 或 -j16。
先用 -j2，报内存不够再改成 -j1。
```

---

## 4. 写入 Gazebo 环境变量

打开 bashrc：

```bash
gedit ~/.bashrc
```

或者用 nano：

```bash
nano ~/.bashrc
```

在文件末尾加入：

```bash
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models_gazebo:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH}
```

保存后执行：

```bash
source ~/.bashrc
```

检查变量：

```bash
echo $GAZEBO_MODEL_PATH
echo $GAZEBO_RESOURCE_PATH
echo $GAZEBO_PLUGIN_PATH
```

---

## 5. 测试 Gazebo + ArduPilot SITL

打开两个终端。

### 终端 1：启动 ArduPilot SITL

```bash
source ~/.bashrc
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

### 终端 2：启动 Gazebo 世界

```bash
source ~/.bashrc
cd ~/ardupilot_gazebo/worlds
gazebo --verbose iris_ardupilot.world
```

如果窗口里出现 Iris 飞机，SITL 控制台没有持续报错，说明 Gazebo 和 ArduPilot 插件基本打通。

---

## 6. 启动 MAVROS 连接 SITL

再开第三个终端：

```bash
source ~/.bashrc
roslaunch mavros_control_demo ap_sitl_mavros.launch
```

如果这个 launch 还没创建，可临时直接运行：

```bash
source ~/.bashrc
roslaunch mavros apm.launch fcu_url:=udp://:14550@
```

如果 `/mavros/state` 不显示 connected，再试：

```bash
roslaunch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14555
```

检查连接：

```bash
rostopic echo -n 1 /mavros/state
```

看到：

```text
connected: True
```

就说明 MAVROS 已经和仿真飞控通了。

---

## 7. 跑我们自己的测试脚本

### 终端 4：读取坐标/GPS/RTK/状态

```bash
source ~/.bashrc
rosrun mavros_control_demo ap_position_reader.py
```

### 终端 5：发送 AP Guided 位置 setpoint

先保守测试：

```bash
source ~/.bashrc
rosrun mavros_control_demo ap_guided_position_setpoint.py _x:=0 _y:=0 _z:=2
```

确认没有异常后，再让脚本自动切 GUIDED、解锁、起飞：

```bash
source ~/.bashrc
rosrun mavros_control_demo ap_guided_position_setpoint.py _x:=2 _y:=0 _z:=2 _auto_guided:=true _auto_arm:=true _auto_takeoff:=true
```

---

## 8. 常见问题

### 0. GitHub 克隆 ArduPilot 失败

典型报错：

```text
GnuTLS recv error (-54): Error in the pull function
fatal: 远端意外挂断了
fatal: 过早的文件结束符 EOF
fatal: index-pack 失败
```

这通常不是命令写错，而是机载电脑到 GitHub 的 HTTPS 连接不稳定。

先测试 GitHub：

```bash
curl -I https://github.com
```

如果主机开了代理，需要让机载电脑也走主机代理。示例：

```bash
export http_proxy=http://主机IP:代理端口
export https_proxy=http://主机IP:代理端口
export all_proxy=socks5://主机IP:socks端口
```

测试代理是否通：

```bash
curl -I https://github.com
```

临时给 git 设置代理：

```bash
git config --global http.proxy http://主机IP:代理端口
git config --global https.proxy http://主机IP:代理端口
```

如果用的是 socks5：

```bash
git config --global http.proxy socks5h://主机IP:socks端口
git config --global https.proxy socks5h://主机IP:socks端口
```

然后用浅克隆：

```bash
cd ~
rm -rf ~/ardupilot
git clone --depth 1 --filter=blob:none --recurse-submodules --shallow-submodules https://github.com/ArduPilot/ardupilot.git
```

如果后续不想让 git 一直走代理，可以清除：

```bash
git config --global --unset http.proxy
git config --global --unset https.proxy
```

如果机载电脑始终连 GitHub 不稳定，可以改用主机下载后离线传输。

#### 方案 A：Windows 主机下载，打包传到机载电脑

注意：不要直接点 GitHub 的 `Download ZIP`。那个 zip 通常不包含完整子模块，ArduPilot 后面会缺依赖。

在 Windows 主机打开 PowerShell，执行：

```powershell
mkdir C:\ardupilot_transfer
cd C:\ardupilot_transfer
git clone --depth 1 --filter=blob:none https://github.com/ArduPilot/ardupilot.git
cd C:\ardupilot_transfer\ardupilot
git submodule update --init --recursive --depth 1 --jobs 4
cd C:\ardupilot_transfer
tar -czf ardupilot.tar.gz ardupilot
```

然后把这个文件传到机载电脑：

```text
C:\ardupilot_transfer\ardupilot.tar.gz
```

可以用 NoMachine 文件传输、U 盘、局域网共享，或者 `scp`。

在机载电脑上解压：

```bash
cd ~
tar -xzf ~/下载/ardupilot.tar.gz -C ~
```

如果文件在桌面：

```bash
cd ~
tar -xzf ~/桌面/ardupilot.tar.gz -C ~
```

确认目录：

```bash
cd ~/ardupilot
git status
git submodule status --recursive | head
```

如果后面提示某些脚本没有执行权限，执行：

```bash
cd ~/ardupilot
git ls-files -s | awk '$1=="100755" {print $4}' | xargs -r chmod +x
git submodule foreach --recursive 'git ls-files -s | awk '\''$1=="100755" {print $4}'\'' | xargs -r chmod +x'
```

然后继续安装 ArduPilot 依赖：

```bash
cd ~/ardupilot/Tools/environment_install
./install-prereqs-ubuntu.sh -y
```

### 0.1 `install-prereqs-ubuntu.sh` 提示 Ubuntu 20.04 不再支持

典型报错：

```text
ArduPilot no longer supports developing on this operating system that has reached end of standard support.
```

含义：

```text
当前 ArduPilot master 分支的依赖安装脚本已经不再支持 Ubuntu 20.04 focal。
这不是仓库坏了，也不是 Gazebo 问题。
因为 ROS Noetic 主要搭配 Ubuntu 20.04，所以当前阶段不建议直接升级系统。
更适合的办法是切到较早的 ArduPilot 稳定分支，例如 Copter-4.5 或 Copter-4.4。
```

推荐处理：

```bash
cd ~/ardupilot
git fetch --depth 1 origin Copter-4.5
git checkout -B Copter-4.5-local FETCH_HEAD
git submodule sync --recursive
git submodule update --init --recursive --depth 1 --jobs 1
cd ~/ardupilot/Tools/environment_install
./install-prereqs-ubuntu.sh -y
```

如果 `Copter-4.5` 分支不存在或下载失败，再试：

```bash
cd ~/ardupilot
git fetch --depth 1 origin Copter-4.4
git checkout -B Copter-4.4-local FETCH_HEAD
git submodule sync --recursive
git submodule update --init --recursive --depth 1 --jobs 1
cd ~/ardupilot/Tools/environment_install
./install-prereqs-ubuntu.sh -y
```

### 1. Gazebo 打不开或很卡

检查显卡/桌面转发。NoMachine 远程桌面下 Gazebo 有时会比较吃力，可以先用：

```bash
gazebo --verbose
```

确认本体能启动。

### 2. 编译 ardupilot_gazebo 时被 killed

一般是内存不够：

```bash
make -j1
```

同时检查：

```bash
free -h
```

### 3. MAVROS 没有 connected True

检查 SITL 是否已经启动，再检查 UDP 端口：

```bash
rostopic echo -n 1 /mavros/state
```

尝试：

```bash
roslaunch mavros apm.launch fcu_url:=udp://:14550@
roslaunch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14555
```

### 4. QGC 和 MAVROS 可能抢端口

DroneChina 帖子里也提醒：QGC 和 MAVROS 不要乱抢同一个 MAVLink 输出。排查时可以先关闭 QGC，只保留 SITL + Gazebo + MAVROS。

### 5. `Main loop slow`

仿真性能不足时可能出现。先不急着改参数，先确认 CPU/内存/图形性能。如果只是偶发警告可继续测试；如果影响解锁，再考虑临时关闭部分 arming check。

---

## 9. 最小命令流程

安装完成后，最小测试流程是：

```bash
# 终端 1
source ~/.bashrc
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map

# 终端 2
source ~/.bashrc
cd ~/ardupilot_gazebo/worlds
gazebo --verbose iris_ardupilot.world

# 终端 3
source ~/.bashrc
roslaunch mavros_control_demo ap_sitl_mavros.launch

# 终端 4
source ~/.bashrc
rostopic echo -n 1 /mavros/state
rosrun mavros_control_demo ap_position_reader.py

# 终端 5
source ~/.bashrc
rosrun mavros_control_demo ap_guided_position_setpoint.py _x:=2 _y:=0 _z:=2 _auto_guided:=true _auto_arm:=true _auto_takeoff:=true
```
