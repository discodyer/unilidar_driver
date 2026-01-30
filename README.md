# unilidar_driver

Open source C++ driver for Unitree Lidar L2.

宇树 L2 激光雷达的开源 C++ 驱动程序

[【中文版 | Chinese】](README_CN.md)

---

## ⚠️ Disclaimer

**The code in this repository is for learning and research purposes only.**

- Original official repository: [Repository Link](https://github.com/unitreerobotics/unilidar_sdk2)
- This driver is an unofficial implementation and is not authorized by the device manufacturer.
- Using this driver may cause device damage, warranty invalidation, or other unforeseen issues.
- **Users assume all risks and consequences. The developer is not responsible for any losses.**
- It is recommended to test cautiously in non-production environments.

---

## TODO List

- [x] Implement data packet transmission and reception
- [x] Test UDP protocol communication on Linux
- [x] Test UDP protocol communication on Windows
- [ ] Test Serial protocol communication on Linux
- [ ] Test Serial protocol communication on Windows

## Build Instructions

### System Requirements

- Linux / Windows
- GCC/G++ compiler
- CMake 3.10 or higher

### Build Steps

```bash
# Cloning repository
git clone https://github.com/discodyer/unilidar_driver.git --recurse-submodules
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

## Protocol Documentation

### Packet Structure

The lidar communication uses a custom binary protocol. Each data packet has the following structure:

| Byte Index | Field Name | Size (Bytes) | Description |
|------------|------------|--------------|-------------|
| 0-3 | Header | 4 | Fixed value `55 AA 05 0A` |
| 4-7 | Packet Type | 4 | Identifies packet type (Little Endian) |
| 8-11 | Packet Size | 4 | Length of data segment (Little Endian) |
| 12-n | Data Segment | Variable | Specific data content |
| n+1 ~ n+4 | CRC32 | 4 | Data checksum |
| n+5 ~ n+8 | msg_type_check | 4 | Message type verification |
| n+9 ~ n+10 | Reserve | 2 | Reserved field |
| n+11 ~ n+12 | Footer | 2 | Fixed value `00 FF` |

## Known Command List

### Command Reference Table

| Function | Packet Type Name | Packet Type ID | Packet Data Type | Command Type ID | Command Value | Description |
|----------|-----------------|----------------|------------------|-----------------|---------------|-------------|
| `setLidarWorkMode` | - | `0x7d2` | LidarWorkModeConfigPacket | - | - | Set lidar work mode |
| `syncLidarTimeStamp` | LIDAR_TIME_STAMP_PACKET_TYPE | `0x6a` | LidarTimeStampPacket | - | - | Synchronize lidar timestamp |
| `sendUserCtrlCmd` | LIDAR_USER_CMD_PACKET_TYPE | `0x64` | LidarUserCtrlCmdPacket | varies | varies | Send user control command |
| `setLidarIpAddressConfig` | LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE | `0x6c` | LidarIpAddressConfigPacket | - | - | Configure lidar IP address |
| `setLidarMacAddressConfig` | LIDAR_MAC_ADDRESS_CONFIG_PACKET_TYPE | `0x6d` | LidarMacAddressConfigPacket | - | - | Configure lidar MAC address |
| `startLidarRotation` | LIDAR_USER_CMD_PACKET_TYPE | `0x64` | LidarUserCtrlCmdPacket | `2` | `0` | Start lidar rotation |
| `resetLidar` | LIDAR_USER_CMD_PACKET_TYPE | `0x64` | LidarUserCtrlCmdPacket | `1` | `1` | Reset lidar |
| `stopLidarRotation` | LIDAR_USER_CMD_PACKET_TYPE | `0x64` | LidarUserCtrlCmdPacket | `2` | `1` | Stop lidar rotation |
| `setLidarParamData` | LIDAR_PARAM_DATA_PACKET_TYPE | `0x7d1` | LidarParamDataPacket | - | - | Set lidar parameters |
| `sendRequestOfLidarParam` | LIDAR_COMMAND_PACKET_TYPE | `0x7d0` | - | `3` | `3` | Request lidar parameters |
| `sendRequestOfLidarVersion` | LIDAR_COMMAND_PACKET_TYPE | `0x7d0` | - | `4` | `0` | Request lidar version |
| `sendRequestOfTimeDelay` | LIDAR_COMMAND_PACKET_TYPE | `0x7d0` | - | `6` | counter+1 | Request time delay (Response type: LIDAR_ACK_DATA_PACKET_TYPE) |

## License

BSD 3-Clause License

## Contributing

Issues and Pull Requests are welcome.