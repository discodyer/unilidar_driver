#ifndef UNITREE_LIDAR_PACKET_PARSER_H
#define UNITREE_LIDAR_PACKET_PARSER_H

#include "unitree_lidar_protocol.h"
#include "unitree_lidar_utilities.h"
#include <vector>
#include <cstring>
#include <algorithm>

#ifndef __EXTERN_C__
namespace unilidar_sdk2{
#endif

class LidarPacketParser {
private:
    uint32_t state_counter;
    std::vector<uint8_t> header_pattern;
    std::vector<uint8_t> tail_pattern;
    std::vector<uint8_t> buffer;
    uint8_t* current_pos;
    uint8_t* buffer_end;
    uint64_t packet_type_temp;
    uint32_t packet_size;
    uint32_t crc32_value;
    uint32_t msg_type_check;
    uint32_t max_packet_size;

public:
    LidarPacketParser();
    int parseByte(const uint8_t& byte, std::vector<uint8_t>& output_packet);
};

#ifndef __EXTERN_C__
} // namespace unilidar_sdk2
#endif

#endif
