#include "unitree_lidar_reader_imp.h"
#include "serial/serial.h"
#include <cstring>
#include <cmath>
#include <ctime>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <iomanip>

namespace unilidar_sdk2 {

// Factory function implementation
UnitreeLidarReader* createUnitreeLidarReader() {
    return new UnitreeLidarReaderImp();
}

// Constructor
UnitreeLidarReaderImp::UnitreeLidarReaderImp()
    : m_connection_type(COMM_NONE)
    , m_serial(nullptr)
    , m_serial_port("")
    , m_baudrate(0)
    , m_udp_handler(nullptr)
    , m_local_ip("192.168.1.100")
    , m_local_port(5000)
    , m_lidar_ip("192.168.1.101")
    , m_lidar_port(5000)
    , m_cloud_scan_num(16)
    , m_enable_imu_data(true)
    , m_version_received(false)
    , m_param_received(false)
    , m_time_delay(0.0)
    , m_time_delay_request_time(0.0)
    , m_time_delay_last_valid(-1.0)
    , m_time_delay_counter(100)
    , m_dirty_percentage(0.0f)
    , m_dirty_sample_count(0)
    , m_dirty_valid(false)
    , m_range_min(0.0f)
    , m_range_max(0.0f)
    , m_last_version_request_time(-1.0)
    , m_last_param_request_time(-1.0)
    , m_last_param_set_time(0.0)
    , m_sdk_version("0.0.0")
{
    memset(m_udp_recv_buffer, 0, sizeof(m_udp_recv_buffer));
    
    // Initialize accumulated cloud
    m_accumulated_cloud.stamp = 0.0;
    m_accumulated_cloud.id = 0;
    m_accumulated_cloud.ringNum = 0;
    m_accumulated_cloud.points.clear();
}

// Destructor
UnitreeLidarReaderImp::~UnitreeLidarReaderImp() {
    // Close connections if open
    if (m_connection_type == COMM_SERIAL) {
        closeSerial();
    } else if (m_connection_type == COMM_UDP) {
        closeUDP();
    }
}

// Basic initialization helper
int UnitreeLidarReaderImp::initializeBasic(float range_min, float range_max, 
                                           uint16_t cloud_scan_num, bool use_system_timestamp) {
    m_cloud_scan_num = cloud_scan_num;
    m_dirty_valid = false;
    m_time_delay = 0.0;
    m_dirty_percentage = 0.0f;
    m_range_min = range_min;
    m_range_max = range_max;
    m_sdk_version = "2.0.9";
    m_enable_imu_data = use_system_timestamp;
    return 0;
}

// Serial initialization
int UnitreeLidarReaderImp::initializeSerial(std::string port, 
                                            uint32_t baudrate,
                                            uint16_t cloud_scan_num,
                                            bool use_system_timestamp,
                                            float range_min,
                                            float range_max) {
    m_serial_port = port;
    m_baudrate = baudrate;
    
    // Check if serial port exists
    if (access(port.c_str(), F_OK) == -1) {
        m_connection_type = COMM_NONE;
        std::cerr << "[ERROR] Unilidar failed to initialize! Serial port \"" 
                  << port << "\" does not exist!" << std::endl;
        return -1;
    }
    
    try {
        // Create serial connection
        m_serial = std::make_shared<serial::Serial>(
            port, baudrate,
            serial::Timeout::simpleTimeout(1000)
        );
        
        if (m_serial->isOpen()) {
            m_serial->flushInput();
        }
        
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        m_serial->setTimeout(timeout);
        
        m_connection_type = COMM_SERIAL;
        initializeBasic(range_min, range_max, cloud_scan_num, use_system_timestamp);
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to initialize serial: " << e.what() << std::endl;
        return -1;
    }
}

// UDP initialization
int UnitreeLidarReaderImp::initializeUDP(unsigned short lidar_port,
                                         std::string lidar_ip,
                                         unsigned short local_port,
                                         std::string local_ip,
                                         uint16_t cloud_scan_num,
                                         bool use_system_timestamp,
                                         float range_min,
                                         float range_max) {
    m_lidar_ip = lidar_ip;
    m_lidar_port = lidar_port;
    m_local_ip = local_ip;
    m_local_port = local_port;
    
    try {
        // Create UDP handler
        m_udp_handler = std::make_shared<UDPHandler>(local_port);
        
        if (m_udp_handler->CreateSocket() != 0) {
            return -1;
        }
        
        if (m_udp_handler->Bind() != 0) {
            return -1;
        }
        
        // Set a short timeout (1ms) so recvfrom returns quickly when no data available
        // Note: SetRecvTimeout parameter is in seconds, but gets converted to milliseconds on Windows
        if (m_udp_handler->SetRecvTimeout(1) != 0) {
            return -1;
        }
        
        m_connection_type = COMM_UDP;
        memset(m_udp_recv_buffer, 0, sizeof(m_udp_recv_buffer));
        initializeBasic(range_min, range_max, cloud_scan_num, use_system_timestamp);
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Failed to initialize UDP: " << e.what() << std::endl;
        return -1;
    }
}

// Close serial connection
bool UnitreeLidarReaderImp::closeSerial() {
    if (m_connection_type != COMM_SERIAL) {
        std::cout << "Serial connection does not exist!" << std::endl;
        return false;
    }
    
    m_connection_type = COMM_NONE;
    
    if (m_serial) {
        m_serial->close();
        m_serial.reset();
    }
    
    std::cout << "Serial connection closed!" << std::endl;
    return true;
}

// Close UDP connection
bool UnitreeLidarReaderImp::closeUDP() {
    if (m_connection_type != COMM_UDP) {
        std::cout << "UDP connection does not exist!" << std::endl;
        return false;
    }
    
    m_connection_type = COMM_NONE;
    
    if (m_udp_handler) {
        m_udp_handler->Close();
        m_udp_handler.reset();
    }
    
    std::cout << "UDP connection closed!" << std::endl;
    return true;
}

// Clear buffer
void UnitreeLidarReaderImp::clearBuffer() {
    m_data_buffer.clear();
    m_read_buffer.clear();
    m_temp_points.clear();
}

// Get buffer cached size
size_t UnitreeLidarReaderImp::getBufferCachedSize() const {
    return m_data_buffer.size();
}

// Get buffer read size
size_t UnitreeLidarReaderImp::getBufferReadSize() const {
    return m_read_buffer.size();
}

// Get IMU data
bool UnitreeLidarReaderImp::getImuData(LidarImuData& imu_data) const {
    // Copy IMU data from internal storage to output parameter
    // The IMU data is stored in m_imu_data.data field
    if (m_enable_imu_data) {
        imu_data = m_imu_data.data;
        return true;
    }
    return false;
}

// Get time delay
bool UnitreeLidarReaderImp::getTimeDelay(double& time_delay) const {
    if (m_time_delay_last_valid > 0.0) {
        time_delay = m_time_delay;
        return true;
    }
    return false;
}

// Get dirty percentage
bool UnitreeLidarReaderImp::getDirtyPercentage(float& dirty_percentage) const {
    if (m_dirty_valid) {
        dirty_percentage = m_dirty_percentage;
        return true;
    }
    return false;
}

// Get firmware version
bool UnitreeLidarReaderImp::getVersionOfLidarFirmware(std::string& version) const {
    if (m_firmware_version.empty()) {
        return false;
    }
    version = m_firmware_version;
    return true;
}

// Get hardware version
bool UnitreeLidarReaderImp::getVersionOfLidarHardware(std::string& version) const {
    if (m_hardware_version.empty()) {
        return false;
    }
    version = m_hardware_version;
    return true;
}

// Get SDK version
bool UnitreeLidarReaderImp::getVersionOfSDK(std::string& version) const {
    version = m_sdk_version;
    return true;
}

// Packet access methods
const LidarPointDataPacket& UnitreeLidarReaderImp::getLidarPointDataPacket() const {
    return m_point_cloud_3d;
}

const Lidar2DPointDataPacket& UnitreeLidarReaderImp::getLidar2DPointDataPacket() const {
    return m_point_cloud_2d;
}

const LidarImuDataPacket& UnitreeLidarReaderImp::getLidarImuDataPacket() const {
    return m_imu_data;
}

const LidarVersionDataPacket& UnitreeLidarReaderImp::getLidarVersionDataPacket() const {
    return m_version_data;
}

// Get lidar parameter data packet
const LidarParamDataPacket& UnitreeLidarReaderImp::getLidarParamDataPacket() const {
    return m_param_data;
}

// Start lidar rotation - ! not sure
void UnitreeLidarReaderImp::startLidarRotation() {
    // Create user control command to start rotation
    LidarUserCtrlCmd cmd;
    cmd.cmd_type = USER_CMD_STANDBY_TYPE;   // Command type for rotation control
    cmd.cmd_value = 0;  // Value 0 = start rotation

    // Send the command
    sendUserCtrlCmd(cmd);
}

// Stop lidar rotation
void UnitreeLidarReaderImp::stopLidarRotation() {
    // Create user control command to stop rotation
    LidarUserCtrlCmd cmd;
    cmd.cmd_type = USER_CMD_STANDBY_TYPE;   // Command type for rotation control
    cmd.cmd_value = 1;  // Value 1 = stop rotation
    
    // Send the command
    sendUserCtrlCmd(cmd);
}

// Reset lidar
void UnitreeLidarReaderImp::resetLidar() {
    // Create user control command to reset lidar
    LidarUserCtrlCmd cmd;
    cmd.cmd_type = USER_CMD_RESET_TYPE;   // Command type for reset/standby
    cmd.cmd_value = 1;  // Value 1 = reset
    
    // Send the command
    sendUserCtrlCmd(cmd);
}

// Set lidar work mode
void UnitreeLidarReaderImp::setLidarWorkMode(uint32_t mode) {
    // Create work mode config packet
    LidarWorkModeConfigPacket packet;
    
    // Set header
    packet.header.header[0] = FRAME_HEADER_ARRAY_0;  // 0x55
    packet.header.header[1] = FRAME_HEADER_ARRAY_1;  // 0xAA
    packet.header.header[2] = FRAME_HEADER_ARRAY_2;  // 0x05
    packet.header.header[3] = FRAME_HEADER_ARRAY_3;  // 0x0A
    packet.header.packet_type = 0x7d2;  // 2002 (0x7d2)
    packet.header.packet_size = sizeof(packet);   // 28 bytes
    
    // Set mode data (masked to 22 bits)
    packet.data.mode = mode & 0x3FFFFF;
    
    // Calculate CRC32
    uint32_t crc = crc32(reinterpret_cast<const uint8_t*>(&packet.data), sizeof(packet.data));
    
    // Set tail
    packet.tail.crc32 = crc;
    packet.tail.msg_type_check = 0;
    packet.tail.reserve[0] = 0;
    packet.tail.reserve[1] = 0;
    packet.tail.tail[0] = FRAME_TAIL_ARRAY_0;  // 0x00
    packet.tail.tail[1] = FRAME_TAIL_ARRAY_1;  // 0xFF
        
    // Send packet
    if (m_connection_type == COMM_SERIAL && m_serial) {
        m_serial->flush();
        m_serial->write(reinterpret_cast<const uint8_t*>(&packet), 
                        sizeof(packet));
    } else if (m_connection_type == COMM_UDP && m_udp_handler) {
        m_udp_handler->Send(reinterpret_cast<const char*>(&packet),
                            sizeof(packet),
                            const_cast<char*>(m_lidar_ip.c_str()),
                            m_lidar_port);
    }
}

// Sync lidar timestamp
void UnitreeLidarReaderImp::syncLidarTimeStamp() {
    // Create timestamp sync packet
    LidarTimeStampPacket packet;
    
    // Set header
    packet.header.header[0] = FRAME_HEADER_ARRAY_0;  // 0x55
    packet.header.header[1] = FRAME_HEADER_ARRAY_1;  // 0xAA
    packet.header.header[2] = FRAME_HEADER_ARRAY_2;  // 0x05
    packet.header.header[3] = FRAME_HEADER_ARRAY_3;  // 0x0A
    packet.header.packet_type = LIDAR_TIME_STAMP_PACKET_TYPE;  // 106 (0x6A)
    packet.header.packet_size = sizeof(packet);  // 32 bytes
    
    // Get current system timestamp
    TimeStamp current_time;
    getSystemTimeStamp(current_time);
    
    // Set timestamp data
    packet.data = current_time;
    
    // Calculate CRC32
    uint32_t crc = crc32(reinterpret_cast<const uint8_t*>(&packet.data), sizeof(packet.data));
    
    // Set tail
    packet.tail.crc32 = crc;
    packet.tail.msg_type_check = 0;
    packet.tail.reserve[0] = 0;
    packet.tail.reserve[1] = 0;
    packet.tail.tail[0] = FRAME_TAIL_ARRAY_0;  // 0x00
    packet.tail.tail[1] = FRAME_TAIL_ARRAY_1;  // 0xFF
    
    // Send packet
    if (m_connection_type == COMM_SERIAL && m_serial) {
        m_serial->flush();
        m_serial->write(reinterpret_cast<const uint8_t*>(&packet), 
                        sizeof(packet));
    } else if (m_connection_type == COMM_UDP && m_udp_handler) {
        m_udp_handler->Send(reinterpret_cast<const char*>(&packet),
                            sizeof(packet),
                            const_cast<char*>(m_lidar_ip.c_str()),
                            m_lidar_port);
    }
}

// Send user control command
void UnitreeLidarReaderImp::sendUserCtrlCmd(LidarUserCtrlCmd cmd) {
    // Create user control command packet
    LidarUserCtrlCmdPacket packet;
    
    // Set header
    packet.header.header[0] = FRAME_HEADER_ARRAY_0;  // 0x55
    packet.header.header[1] = FRAME_HEADER_ARRAY_1;  // 0xAA
    packet.header.header[2] = FRAME_HEADER_ARRAY_2;  // 0x05
    packet.header.header[3] = FRAME_HEADER_ARRAY_3;  // 0x0A
    packet.header.packet_type = LIDAR_USER_CMD_PACKET_TYPE;  // 100 (0x64)
    packet.header.packet_size = sizeof(packet);   // 32 bytes

    // Set command data
    packet.data = cmd;
    // Calculate CRC32
    uint32_t crc = crc32(reinterpret_cast<const uint8_t*>(&packet.data), sizeof(packet.data));

    // Set tail
    packet.tail.crc32 = crc;
    packet.tail.msg_type_check = 0;
    packet.tail.reserve[0] = 0;
    packet.tail.reserve[1] = 0;
    packet.tail.tail[0] = FRAME_TAIL_ARRAY_0;  // 0x00
    packet.tail.tail[1] = FRAME_TAIL_ARRAY_1;  // 0xFF
        
    // Send packet
    if (m_connection_type == COMM_SERIAL && m_serial) {
        m_serial->flush();
        m_serial->write(reinterpret_cast<const uint8_t*>(&packet), 
                        sizeof(packet));
    } else if (m_connection_type == COMM_UDP && m_udp_handler) {
        m_udp_handler->Send(reinterpret_cast<const char*>(&packet),
                            sizeof(packet),
                            const_cast<char*>(m_lidar_ip.c_str()),
                            m_lidar_port);
    }
}

// Send lidar command
void UnitreeLidarReaderImp::sendLidarCommand(LidarCommand cmd) {
    // Create user control command packet
    LidarCommandPacket packet;
    memset(&packet, 0, sizeof(packet));

    // Header
    packet.header.header[0] = FRAME_HEADER_ARRAY_0;
    packet.header.header[1] = FRAME_HEADER_ARRAY_1;
    packet.header.header[2] = FRAME_HEADER_ARRAY_2;
    packet.header.header[3] = FRAME_HEADER_ARRAY_3;

    packet.header.packet_type = LIDAR_COMMAND_PACKET_TYPE; // 2000
    packet.header.packet_size = sizeof(packet);

    // Data
    packet.data = cmd;

    // CRC32
    uint32_t crc = crc32(reinterpret_cast<const uint8_t*>(&packet.data), sizeof(packet.data));
    
    // Set tail
    packet.tail.crc32 = crc;
    packet.tail.msg_type_check = 0;
    packet.tail.reserve[0] = 0;
    packet.tail.reserve[1] = 0;
    packet.tail.tail[0] = FRAME_TAIL_ARRAY_0;  // 0x00
    packet.tail.tail[1] = FRAME_TAIL_ARRAY_1;  // 0xFF
    
    // Send packet via Serial or UDP
    if (m_connection_type == COMM_SERIAL && m_serial) {
        m_serial->flush();
        m_serial->write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    } else if (m_connection_type == COMM_UDP && m_udp_handler) {
        m_udp_handler->Send(reinterpret_cast<char*>(&packet),
                            sizeof(packet),
                            const_cast<char*>(m_lidar_ip.c_str()),
                            m_lidar_port);
    }
}

// Set lidar IP address configuration
void UnitreeLidarReaderImp::setLidarIpAddressConfig(LidarIpAddressConfig config) {
    // Create IP address configuration packet
    LidarIpAddressConfigPacket packet;
    
    // Set header
    packet.header.header[0] = FRAME_HEADER_ARRAY_0;  // 0x55
    packet.header.header[1] = FRAME_HEADER_ARRAY_1;  // 0xAA
    packet.header.header[2] = FRAME_HEADER_ARRAY_2;  // 0x05
    packet.header.header[3] = FRAME_HEADER_ARRAY_3;  // 0x0A
    packet.header.packet_type = LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE;  // 108 (0x6C)
    packet.header.packet_size = sizeof(packet);   // 44 bytes
    
    // Set IP configuration data
    packet.data = config;
    
    // Calculate CRC32
    uint32_t crc = crc32(reinterpret_cast<const uint8_t*>(&packet.data), sizeof(packet.data));
    
    // Set tail
    packet.tail.crc32 = crc;
    packet.tail.msg_type_check = 0;
    packet.tail.reserve[0] = 0;
    packet.tail.reserve[1] = 0;
    packet.tail.tail[0] = FRAME_TAIL_ARRAY_0;  // 0x00
    packet.tail.tail[1] = FRAME_TAIL_ARRAY_1;  // 0xFF
    
    // Send packet
    if (m_connection_type == COMM_SERIAL && m_serial) {
        m_serial->flush();
        m_serial->write(reinterpret_cast<const uint8_t*>(&packet), 
                        sizeof(packet));
    } else if (m_connection_type == COMM_UDP && m_udp_handler) {
        m_udp_handler->Send(reinterpret_cast<const char*>(&packet),
                              sizeof(packet),
                              const_cast<char*>(m_lidar_ip.c_str()),
                              m_lidar_port);
    }
}

// Set lidar MAC address configuration
void UnitreeLidarReaderImp::setLidarMacAddressConfig(LidarMacAddressConfig config) {
    // Create MAC address configuration packet
    LidarMacAddressConfigPacket packet;
    
    // Set header
    packet.header.header[0] = FRAME_HEADER_ARRAY_0;  // 0x55
    packet.header.header[1] = FRAME_HEADER_ARRAY_1;  // 0xAA
    packet.header.header[2] = FRAME_HEADER_ARRAY_2;  // 0x05
    packet.header.header[3] = FRAME_HEADER_ARRAY_3;  // 0x0A
    packet.header.packet_type = LIDAR_MAC_ADDRESS_CONFIG_PACKET_TYPE;  // 109 (0x6d)
    packet.header.packet_size = sizeof(packet);   // 32 bytes
    
    // Set MAC configuration data
    packet.data = config;
    
    // Calculate CRC32
    uint32_t crc = crc32(reinterpret_cast<const uint8_t*>(&packet.data), sizeof(packet.data));
    
    // Set tail
    packet.tail.crc32 = crc;
    packet.tail.msg_type_check = 0;
    packet.tail.reserve[0] = 0;
    packet.tail.reserve[1] = 0;
    packet.tail.tail[0] = FRAME_TAIL_ARRAY_0;  // 0x00
    packet.tail.tail[1] = FRAME_TAIL_ARRAY_1;  // 0xFF
    
    // Send packet
    if (m_connection_type == COMM_SERIAL && m_serial) {
        m_serial->flush();
        m_serial->write(reinterpret_cast<const uint8_t*>(&packet), 
                        sizeof(packet));
    } else if (m_connection_type == COMM_UDP && m_udp_handler) {
        m_udp_handler->Send(reinterpret_cast<const char*>(&packet),
                            sizeof(packet),
                            const_cast<char*>(m_lidar_ip.c_str()),
                            m_lidar_port);
    }
}

// Send request of lidar parameter
void UnitreeLidarReaderImp::sendRequestOfLidarParam() {
    LidarUserCtrlCmdPacket packet;
    memset(&packet, 0, sizeof(packet));

    // Header
    packet.header.header[0] = FRAME_HEADER_ARRAY_0;
    packet.header.header[1] = FRAME_HEADER_ARRAY_1;
    packet.header.header[2] = FRAME_HEADER_ARRAY_2;
    packet.header.header[3] = FRAME_HEADER_ARRAY_3;

    packet.header.packet_type = LIDAR_COMMAND_PACKET_TYPE; // 2000
    packet.header.packet_size = sizeof(packet);

    // Data
    packet.data.cmd_type = CMD_PARAM_GET;
    packet.data.cmd_value = 0;

    // CRC32
    uint32_t crc = crc32(reinterpret_cast<const uint8_t*>(&packet.data), sizeof(packet.data));
    
    // Set tail
    packet.tail.crc32 = crc;
    packet.tail.msg_type_check = 0;
    packet.tail.reserve[0] = 0;
    packet.tail.reserve[1] = 0;
    packet.tail.tail[0] = FRAME_TAIL_ARRAY_0;  // 0x00
    packet.tail.tail[1] = FRAME_TAIL_ARRAY_1;  // 0xFF
    
    // Send packet via Serial or UDP
    if (m_connection_type == COMM_SERIAL && m_serial) {
        m_serial->flush();
        m_serial->write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    } else if (m_connection_type == COMM_UDP && m_udp_handler) {
        m_udp_handler->Send(reinterpret_cast<char*>(&packet),
                            sizeof(packet),
                            const_cast<char*>(m_lidar_ip.c_str()),
                            m_lidar_port);
    }
}

// Get lidar parameter data
void UnitreeLidarReaderImp::getLidarParamData(LidarParamData& param) const {
    if (m_param_received) {
        // Copy all 21 fields from internal storage to output parameter
        param = m_param_data.data;
    }
}

// Send request of lidar version
void UnitreeLidarReaderImp::sendRequestOfLidarVersion() {    
    LidarUserCtrlCmdPacket packet;
    memset(&packet, 0, sizeof(packet));

    // Header
    packet.header.header[0] = FRAME_HEADER_ARRAY_0;
    packet.header.header[1] = FRAME_HEADER_ARRAY_1;
    packet.header.header[2] = FRAME_HEADER_ARRAY_2;
    packet.header.header[3] = FRAME_HEADER_ARRAY_3;

    packet.header.packet_type = LIDAR_COMMAND_PACKET_TYPE; // 2000
    packet.header.packet_size = sizeof(packet);

    // Data
    packet.data.cmd_type = CMD_VERSION_GET;
    packet.data.cmd_value = 0;

    // CRC32
    uint32_t crc = crc32(reinterpret_cast<const uint8_t*>(&packet.data), sizeof(packet.data));
    
    // Set tail
    packet.tail.crc32 = crc;
    packet.tail.msg_type_check = 0;
    packet.tail.reserve[0] = 0;
    packet.tail.reserve[1] = 0;
    packet.tail.tail[0] = FRAME_TAIL_ARRAY_0;  // 0x00
    packet.tail.tail[1] = FRAME_TAIL_ARRAY_1;  // 0xFF
        
    // Send packet via Serial or UDP
    if (m_connection_type == COMM_SERIAL && m_serial) {
        m_serial->flush();
        m_serial->write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    } else if (m_connection_type == COMM_UDP && m_udp_handler) {
        m_udp_handler->Send(reinterpret_cast<char*>(&packet),
                            sizeof(packet),
                            const_cast<char*>(m_lidar_ip.c_str()),
                            m_lidar_port);
    } else {
        std::cout << "ERROR: No valid connection for sending!" << std::endl;
    }
}

// Send request of time delay
void UnitreeLidarReaderImp::sendRequestOfTimeDelay() {
    LidarUserCtrlCmdPacket packet;
    memset(&packet, 0, sizeof(packet));

    // Header
    packet.header.header[0] = FRAME_HEADER_ARRAY_0;
    packet.header.header[1] = FRAME_HEADER_ARRAY_1;
    packet.header.header[2] = FRAME_HEADER_ARRAY_2;
    packet.header.header[3] = FRAME_HEADER_ARRAY_3;

    packet.header.packet_type = LIDAR_COMMAND_PACKET_TYPE; // 2000
    packet.header.packet_size = sizeof(packet);

    // Data
    packet.data.cmd_type = CMD_LATENCY_TYPE;
    packet.data.cmd_value = ++m_time_delay_counter;

    // CRC32
    uint32_t crc = crc32(reinterpret_cast<const uint8_t*>(&packet.data), sizeof(packet.data));
    
    // Set tail
    packet.tail.crc32 = crc;
    packet.tail.msg_type_check = 0;
    packet.tail.reserve[0] = 0;
    packet.tail.reserve[1] = 0;
    packet.tail.tail[0] = FRAME_TAIL_ARRAY_0;  // 0x00
    packet.tail.tail[1] = FRAME_TAIL_ARRAY_1;  // 0xFF
    
    // Send packet via Serial or UDP
    if (m_connection_type == COMM_SERIAL && m_serial) {
        m_serial->flush();
        m_serial->write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    } else if (m_connection_type == COMM_UDP && m_udp_handler) {
        m_udp_handler->Send(reinterpret_cast<char*>(&packet),
                            sizeof(packet),
                            const_cast<char*>(m_lidar_ip.c_str()),
                            m_lidar_port);
    }
    
    // Record request time
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    m_time_delay_request_time = (double)ts.tv_sec + (double)ts.tv_nsec / 1000000000.0;
}

// Get point cloud
bool UnitreeLidarReaderImp::getPointCloud(PointCloudUnitree& cloud) const {
    // Check if we have enough scans accumulated
    if (m_accumulated_cloud.ringNum < m_cloud_scan_num) {
        return false;
    }
    
    // Copy accumulated cloud to output
    cloud.stamp = m_accumulated_cloud.stamp;
    cloud.id = m_accumulated_cloud.id;
    cloud.ringNum = m_accumulated_cloud.ringNum;
    cloud.points = m_accumulated_cloud.points;
    
    return true;
}

// Append one point cloud
void UnitreeLidarReaderImp::appendOnePointCloud(PointCloudUnitree& target_cloud,
                                                const PointCloudUnitree& source_cloud,
                                                uint32_t max_scan_num) {
    double stamp_diff = source_cloud.stamp - target_cloud.stamp;
    
    // If target has fewer scans than max, append points
    if (target_cloud.ringNum < max_scan_num) {
        // Adjust z-coordinate based on timestamp difference
        for (const auto& point : source_cloud.points) {
            PointUnitree adjusted_point = point;
            adjusted_point.z += stamp_diff;
            target_cloud.points.push_back(adjusted_point);
        }
        target_cloud.ringNum++;
    } else {
        // Reset and start new accumulation
        target_cloud.id++;
        target_cloud.ringNum = 1;
        target_cloud.stamp = source_cloud.stamp;
        target_cloud.points = source_cloud.points;
    }
}

// Main parsing function
// This is the most complex function that handles all packet parsing
int UnitreeLidarReaderImp::runParse() {
    
    // Check if connection is initialized
    if (m_connection_type == COMM_NONE) {
        std::cout << "[WARNING] Unilidar is not initialized!" << std::endl;
        sleep(1);
        return 0;
    }
        
    // Get current time
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    double current_time = (double)ts.tv_sec + (double)ts.tv_nsec / 1000000000.0;
    
    // Read data from connection if buffer is empty
    if (m_data_buffer.empty()) {        
        if (m_connection_type == COMM_SERIAL) {
            // Serial connection
            if (m_serial && m_serial->waitReadable()) {
                size_t available = m_serial->available();
                if (available > 0) {
                    m_read_buffer.resize(available);
                    m_serial->read(m_read_buffer.data(), available);
                    m_data_buffer.insert(m_data_buffer.end(), 
                                        m_read_buffer.begin(), 
                                        m_read_buffer.end());
                }
            } else {
                std::cout << "[DEBUG] Serial not readable" << std::endl;
            }
        } else if (m_connection_type == COMM_UDP) {
            // UDP connection
            int recv_len = m_udp_handler->Recv((char*)m_udp_recv_buffer, 2000, nullptr);
            
            if (recv_len > 0) {
                m_read_buffer.assign(m_udp_recv_buffer, m_udp_recv_buffer + recv_len);
                m_data_buffer.insert(m_data_buffer.end(),
                                    m_read_buffer.begin(),
                                    m_read_buffer.end());
                
            } else if (recv_len == 0) {
                std::cout << "[DEBUG] UDP Recv returned 0 (no data)" << std::endl;
            } else {
                // std::cout << "[DEBUG] UDP Recv error: " << recv_len << std::endl;
            }
        }
    } else {
        // std::cout << "[DEBUG] Buffer not empty, skipping read" << std::endl;
    }
        
    // Request version if not received and enough time has passed
    if (!m_version_received && (current_time - m_last_version_request_time > 0.5)) {
        sendRequestOfLidarVersion();
        m_last_version_request_time = current_time;
    }
    
    // Request parameters if not received and enough time has passed
    if (!m_param_received && (current_time - m_last_param_request_time > 0.5)) {
        sendRequestOfLidarParam();
        m_last_param_request_time = current_time;
    }
    
    // Request time delay periodically
    if (current_time - m_time_delay_request_time > 1.0) {
        sendRequestOfTimeDelay();
    }
    
    // Parse packets from buffer using packet parser
    int parse_result = 0;

    // Process buffer byte by byte until we get a complete packet or run out of bytes
    while (!m_data_buffer.empty()) {
        uint8_t byte = m_data_buffer.front();
        m_data_buffer.pop_front();
        
        parse_result = m_packet_parser.parseByte(byte, m_read_buffer);
        
        if (parse_result != 0) {
            // Packet parsed successfully
            // std::cout << "[DEBUG] Packet parsed! Type: " << parse_result << " (0x" << std::hex << parse_result << std::dec << ")" << std::endl;
            break;
        }
    }
    
    // Handle parsed packet based on type
    switch (parse_result) {
    case LIDAR_POINT_DATA_PACKET_TYPE:
        // std::cout << "[DEBUG] Processing LIDAR_POINT_DATA_PACKET_TYPE" << std::endl;
        {
            // 3D point cloud packet
            if (m_read_buffer.size() >= sizeof(LidarPointDataPacket)) {
                // Copy raw bytes into the struct member
                std::memcpy(&m_point_cloud_3d, m_read_buffer.data(), sizeof(LidarPointDataPacket));

                // Process Dirty Percentage
                float current_dirty = m_point_cloud_3d.data.state.dirty_index;
                if (m_dirty_sample_count < 200) {
                    // Initial accumulation
                    m_dirty_percentage = (m_dirty_percentage * m_dirty_sample_count + current_dirty) / (m_dirty_sample_count + 1);
                    m_dirty_sample_count++;
                } else {
                    // Running average (0.01 weight for new, 0.99 for old)
                    m_dirty_percentage = m_dirty_percentage * 0.99f + current_dirty * 0.01f;
                }
                m_dirty_valid = (m_dirty_sample_count > 199);

                // Convert packet to PointCloud
                PointCloudUnitree cloud_unitree;
                parseFromPacketToPointCloud(cloud_unitree, m_point_cloud_3d, true, m_range_min, m_range_max);
                
                // Accumulate (stitch) point clouds
                appendOnePointCloud(m_accumulated_cloud, cloud_unitree, m_cloud_scan_num);
            }
        }
        break;
        
    case LIDAR_2D_POINT_DATA_PACKET_TYPE:
        // std::cout << "[DEBUG] Processing LIDAR_2D_POINT_DATA_PACKET_TYPE" << std::endl;
        {
            // 2D point cloud packet
            if (m_read_buffer.size() >= sizeof(Lidar2DPointDataPacket)) {
                std::memcpy(&m_point_cloud_2d, m_read_buffer.data(), sizeof(Lidar2DPointDataPacket));
                
                // Note: If you have a member for accumulated 2D cloud, you would process it here
                // typically using parseFromPacketPointCloud2D
            }        
        }
        break;

    case LIDAR_IMU_DATA_PACKET_TYPE:
        // std::cout << "[DEBUG] Processing LIDAR_IMU_DATA_PACKET_TYPE" << std::endl;
        {
            // IMU data packet
            if (m_read_buffer.size() >= sizeof(LidarImuDataPacket)) {
                std::memcpy(&m_imu_data, m_read_buffer.data(), sizeof(LidarImuDataPacket));
                
                // The IMU data is now updated in the member variable m_imu_data
                // If timestamps need alignment, it would happen here, but standard use assumes latest data
            }
        }
        break;

    case LIDAR_VERSION_PACKET_TYPE:
        // std::cout << "[DEBUG] Processing LIDAR_VERSION_PACKET_TYPE" << std::endl;
        {
            // Version data packet
            if (m_read_buffer.size() >= sizeof(LidarVersionDataPacket)) {
                std::memcpy(&m_version_data, m_read_buffer.data(), sizeof(LidarVersionDataPacket));
                m_version_received = true;

                // Format version strings (HW & SW)
                char version_buffer[64];
                
                // Format Hardware Version
                snprintf(version_buffer, sizeof(version_buffer), "%d.%d.%d.%d",
                        m_version_data.data.hw_version[0], m_version_data.data.hw_version[1],
                        m_version_data.data.hw_version[2], m_version_data.data.hw_version[3]);
                m_hardware_version = std::string(version_buffer);

                // Format Software/Firmware Version
                snprintf(version_buffer, sizeof(version_buffer), "%d.%d.%d.%d",
                        m_version_data.data.sw_version[0], m_version_data.data.sw_version[1],
                        m_version_data.data.sw_version[2], m_version_data.data.sw_version[3]);
                m_firmware_version = std::string(version_buffer);
            }
        }
        break;

    case LIDAR_PARAM_DATA_PACKET_TYPE:
        // std::cout << "[DEBUG] Processing LIDAR_PARAM_DATA_PACKET_TYPE" << std::endl;
        {
            // Parameter data packet
            if (m_read_buffer.size() >= sizeof(LidarParamDataPacket)) {
                std::memcpy(&m_param_data, m_read_buffer.data(), sizeof(LidarParamDataPacket));
                m_param_received = true;
                m_last_param_set_time = current_time;
            }
        }
        break;

    case LIDAR_ACK_DATA_PACKET_TYPE:
        // std::cout << "[DEBUG] Processing LIDAR_ACK_DATA_PACKET_TYPE" << std::endl;
        {
            // Time delay response packet (or generic ACK)
            if (m_read_buffer.size() >= sizeof(LidarAckDataPacket)) {
                LidarAckDataPacket ack_packet;
                std::memcpy(&ack_packet, m_read_buffer.data(), sizeof(LidarAckDataPacket));

                // Check if this ACK is for the Latency command (CMD_LATENCY_TYPE = 6)
                if (ack_packet.data.cmd_type == CMD_LATENCY_TYPE) {
                    // Calculate Round Trip Time (RTT)
                    double rtt = current_time - m_time_delay_request_time;

                    // Apply Low-Pass Filter: New * 0.1 + Old * 0.9
                    if (m_time_delay_counter == 0) {
                        m_time_delay = rtt;
                    } else {
                        m_time_delay = rtt * 0.1 + m_time_delay * 0.9;
                    }
                    m_time_delay_counter++;
                    m_time_delay_last_valid = current_time;
                }
            }
        }
        break;

    default:
        if (parse_result != 0) {
            // std::cout << "[DEBUG] Unknown packet type: " << parse_result << std::endl;
        }
        break;
    }

    return parse_result;
}

} // namespace unilidar_sdk2
