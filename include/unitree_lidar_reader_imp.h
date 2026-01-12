#ifndef UNITREE_LIDAR_READER_IMP_H
#define UNITREE_LIDAR_READER_IMP_H

#include "unitree_lidar_sdk.h"
#include "unitree_lidar_packet_parser.h"
#include "udp_handler.h"
#include <string>
#include <deque>
#include <vector>
#include <memory>

// Forward declarations
namespace serial {
    class Serial;
}

namespace unilidar_sdk2 {

class UnitreeLidarReaderImp : public UnitreeLidarReader {
public:
    UnitreeLidarReaderImp();
    virtual ~UnitreeLidarReaderImp();

    // Initialization methods - match base class signatures exactly
    virtual int initializeSerial(
        std::string port = "/dev/ttyACM0",
        uint32_t baudrate = 4000000,
        uint16_t cloud_scan_num = 18,
        bool use_system_timestamp = true,
        float range_min = 0,
        float range_max = 100
    ) override;
    
    virtual int initializeUDP(
        unsigned short lidar_port = 6101,
        std::string lidar_ip = "192.168.1.62",
        unsigned short local_port = 6201,
        std::string local_ip = "192.168.1.2",
        uint16_t cloud_scan_num = 18,
        bool use_system_timestamp = true,
        float range_min = 0,
        float range_max = 100
    ) override;

    // Connection management
    virtual bool closeSerial() override;
    virtual bool closeUDP() override;

    // Data retrieval methods
    virtual bool getPointCloud(PointCloudUnitree& cloud) const override;
    virtual bool getImuData(LidarImuData& imu_data) const override;
    virtual bool getTimeDelay(double& time_delay) const override;
    virtual bool getDirtyPercentage(float& dirty_percentage) const override;
    
    // Version information
    virtual bool getVersionOfLidarFirmware(std::string& version) const override;
    virtual bool getVersionOfLidarHardware(std::string& version) const override;
    virtual bool getVersionOfSDK(std::string& version) const override;

    // Buffer status
    virtual size_t getBufferCachedSize() const override;
    virtual size_t getBufferReadSize() const override;
    virtual void clearBuffer() override;

    // Lidar control commands
    virtual void startLidarRotation() override;
    virtual void stopLidarRotation() override;
    virtual void resetLidar() override;
    virtual void setLidarWorkMode(uint32_t mode) override;
    virtual void syncLidarTimeStamp() override;

    // Configuration methods
    virtual void setLidarIpAddressConfig(LidarIpAddressConfig config) override;
    virtual void setLidarMacAddressConfig(LidarMacAddressConfig config) override;
    virtual void sendUserCtrlCmd(LidarUserCtrlCmd cmd) override;

    // Packet access methods (const)
    virtual const LidarPointDataPacket& getLidarPointDataPacket() const override;
    virtual const Lidar2DPointDataPacket& getLidar2DPointDataPacket() const override;
    virtual const LidarImuDataPacket& getLidarImuDataPacket() const override;
    virtual const LidarVersionDataPacket& getLidarVersionDataPacket() const override;
    const LidarParamDataPacket& getLidarParamDataPacket() const;

    // Parameter data access
    void getLidarParamData(LidarParamData& param) const;

    // Main parsing function
    virtual int runParse() override;

private:
    // Helper methods
    int initializeBasic(float range_min, float range_max, 
                       uint16_t cloud_scan_num, bool use_system_timestamp);
    void appendOnePointCloud(PointCloudUnitree& target_cloud,
                            const PointCloudUnitree& source_cloud,
                            uint32_t max_scan_num);
    
    void sendLidarCommand(LidarCommand cmd);
    void sendRequestOfLidarParam();
    void sendRequestOfLidarVersion();
    void sendRequestOfTimeDelay();

    // Member variables - organized by functionality
    
    // Connection type: 0=none, 1=serial, 2=UDP
    uint8_t m_connection_type;
    
    // Serial connection
    std::shared_ptr<serial::Serial> m_serial;
    std::string m_serial_port;
    uint32_t m_baudrate;
    
    // UDP connection
    std::shared_ptr<UDPHandler> m_udp_handler;
    std::string m_local_ip;
    uint16_t m_local_port;
    std::string m_lidar_ip;
    uint16_t m_lidar_port;
    uint8_t m_udp_recv_buffer[2000];
    
    // Data buffers
    std::deque<uint8_t> m_data_buffer;
    std::vector<uint8_t> m_read_buffer;
    
    // Packet parser
    LidarPacketParser m_packet_parser;
    
    // Point cloud data packets
    LidarPointDataPacket m_point_cloud_3d;
    Lidar2DPointDataPacket m_point_cloud_2d;
    std::vector<PointUnitree> m_temp_points;
    
    // Accumulated point cloud
    PointCloudUnitree m_accumulated_cloud;
    uint16_t m_cloud_scan_num;
    
    // IMU data packet
    LidarImuDataPacket m_imu_data;
    bool m_enable_imu_data;
    
    // Version data packet
    LidarVersionDataPacket m_version_data;
    std::string m_firmware_version;
    std::string m_hardware_version;
    bool m_version_received;
    
    // Parameter data packet
    LidarParamDataPacket m_param_data;
    bool m_param_received;
    
    // Time delay measurement
    double m_time_delay;
    double m_time_delay_request_time;
    double m_time_delay_last_valid;
    uint32_t m_time_delay_counter;
    
    // Dirty percentage
    float m_dirty_percentage;
    uint32_t m_dirty_sample_count;
    bool m_dirty_valid;
    
    // Range filtering
    float m_range_min;
    float m_range_max;
    
    // Timestamps
    double m_last_version_request_time;
    double m_last_param_request_time;
    double m_last_param_set_time;
    
    // SDK version
    std::string m_sdk_version;
};

// Factory function
UnitreeLidarReader* createUnitreeLidarReader();

} // namespace unilidar_sdk2

#endif // UNITREE_LIDAR_READER_IMP_H
