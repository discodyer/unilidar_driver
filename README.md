# unilidar_driver

Open source C++ driver for Unitree Lidar L2.

宇树 L2 激光雷达的开源 C++ 驱动程序

## **⚠️ 免责声明 / Disclaimer**

**本仓库代码仅供学习和研究使用。**

- 原官方仓库地址：[仓库地址](https://github.com/unitreerobotics/unilidar_sdk2)
- 本驱动为非官方实现，未经设备制造商授权
- 使用本驱动程序可能导致设备损坏、保修失效或其他不可预见的问题
- **使用者需自行承担所有风险和后果，开发者不对任何损失负责**
- 建议在非生产环境中谨慎测试

---

## TODO List

- [x] 实现数据包的发送和接收
- [x] 测试 Linux 下的 UDP 协议通信
- [x] 测试 Windows 下的 UDP 协议通信
- [ ] 测试 Linux 下的 Serial 协议通信
- [ ] 测试 Windows 下的 Serial 协议通信

## 编译说明

### 系统要求

- Linux / Windows
- GCC/G++ 编译器
- CMake 3.10 或更高版本

### 编译步骤

```bash
# Cloning repository
git clone https://github.com/discodyer/unilidar_driver.git
cd unilidar_driver

# Create build directory
mkdir build
cd build

# Compilation
cmake ..
make

# Installation (optional)
sudo make install
```

## 协议文档

### 数据包结构

雷达通信采用自定义二进制协议，每个数据包结构如下：

| 字节序号 | 字段名称 | 大小(字节) | 说明 |
|---------|---------|-----------|------|
| 0-3 | 帧头 | 4 | 固定值：`55 AA 05 0A` |
| 4-7 | 包类型 | 4 | 标识数据包类型（小端序） |
| 8-11 | 包大小 | 4 | 数据段长度（小端序） |
| 12-n | 数据段 | 可变 | 具体数据内容 |
| n+1 ~ n+4 | CRC32 | 4 | 数据校验和 |
| n+5 ~ n+8 | msg_type_check | 4 | 消息类型校验 |
| n+9 ~ n+10 | reserve | 2 | 保留字段 |
| n+11 ~ n+12 | 帧尾 | 2 | 固定值：`00 FF` |

## 已知命令列表

### 命令对照表

| 功能函数 | 包类型名称 | 包类型ID | 包数据类型 | 命令类型ID | 命令值 | 说明 |
|---------|-----------|---------|------------|-----------|--------|------|
| `setLidarWorkMode` | - | `0x7d2` | LidarWorkModeConfigPacket | - | - | 设置雷达工作模式 |
| `syncLidarTimeStamp` | LIDAR_TIME_STAMP_PACKET_TYPE | `0x6a` | LidarTimeStampPacket | - | - | 同步雷达时间戳 |
| `sendUserCtrlCmd` | LIDAR_USER_CMD_PACKET_TYPE | `0x64` | LidarUserCtrlCmdPacket | varies | varies | 发送用户控制命令 |
| `setLidarIpAddressConfig` | LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE | `0x6c` | LidarIpAddressConfigPacket | - | - | 配置雷达IP地址 |
| `setLidarMacAddressConfig` | LIDAR_MAC_ADDRESS_CONFIG_PACKET_TYPE | `0x6d` | LidarMacAddressConfigPacket | - | - | 配置雷达MAC地址 |
| `startLidarRotation` | LIDAR_USER_CMD_PACKET_TYPE | `0x64` | LidarUserCtrlCmdPacket | `2` | `0` | 启动雷达旋转 |
| `resetLidar` | LIDAR_USER_CMD_PACKET_TYPE | `0x64` | LidarUserCtrlCmdPacket | `1` | `1` | 重置雷达 |
| `stopLidarRotation` | LIDAR_USER_CMD_PACKET_TYPE | `0x64` | LidarUserCtrlCmdPacket | `2` | `1` | 停止雷达旋转 |
| `setLidarParamData` | LIDAR_PARAM_DATA_PACKET_TYPE | `0x7d1` | LidarParamDataPacket | - | - | 设置雷达参数 |
| `sendRequestOfLidarParam` | LIDAR_COMMAND_PACKET_TYPE | `0x7d0` | - | `3` | `3` | 请求雷达参数 |
| `sendRequestOfLidarVersion` | LIDAR_COMMAND_PACKET_TYPE | `0x7d0` | - | `4` | `0` | 请求雷达版本 |
| `sendRequestOfTimeDelay` | LIDAR_COMMAND_PACKET_TYPE | `0x7d0` | - | `6` | counter+1 | 请求时间延迟（响应类型：LIDAR_ACK_DATA_PACKET_TYPE） |

## License

BSD 3-Clause License

## 贡献

欢迎提交 Issue 和 Pull Request。
