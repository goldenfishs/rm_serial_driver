# rm_serial_driver

ROS2: 适用于RM的简易上位机串口通信程序。

## 使用方式

### 1. 安装依赖

首先，安装所需的依赖包：

```bash
sudo apt install ros-humble-serial-driver
```

### 2. 编译并安装串口库

克隆串口库源码并编译安装：

```bash
git clone https://github.com/ZhaoXiangBox/serial.git
cd serial
mkdir build
cd build
cmake ..
make
sudo make install
```

**注意**：安装完成后可能需要重启系统。

### 3. 下载并编译项目

将项目克隆到你的ROS2工作空间中：

```bash
cd ~/ros2_ws/src
git clone https://github.com/goldenfishs/rm_serial_driver.git
git clone https://github.com/goldenfishs/rm_msg.git  # 包含所需的自定义话题消息，必须一起使用
```

回到工作空间并编译：

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 4. 运行程序

使用以下命令启动串口驱动：

```bash
ros2 launch rm_serial_driver rm_serial_driver.launch.py
```
使用时请将上位机控制命令发送到data_ai话题，即可将数据传给下位机，下位机发送上来的数据会发布到data_mcu和data_ref话题，其他程序可直接哪去

### 5. 自定义通信协议

如果需要自定义通信协议，可以参考 `protocol.h` 文件，修改对应的协议。同时，修改 `rm_msgs` 包中的自定义消息，注意数据顺序和数据类型。修改完成后，重新编译即可。

```bash
colcon build
source install/setup.bash
```

## 注意

ubuntu22需要卸载brltty

```bash
sudo apt remove brltty
```

## 注意事项

- 确保在修改协议和消息后重新编译项目。
- 如果遇到问题，请检查串口连接和权限设置。
