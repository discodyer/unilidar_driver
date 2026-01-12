#include "unitree_lidar_packet_parser.h"

#ifndef __EXTERN_C__
namespace unilidar_sdk2{
#endif

// Constructor
LidarPacketParser::LidarPacketParser() {
    state_counter = 0;
    
    // Initialize header pattern
    header_pattern.clear();
    header_pattern.push_back(FRAME_HEADER_ARRAY_0);  // 0x55
    header_pattern.push_back(FRAME_HEADER_ARRAY_1);  // 0xAA
    header_pattern.push_back(FRAME_HEADER_ARRAY_2);  // 0x05
    header_pattern.push_back(FRAME_HEADER_ARRAY_3);  // 0x0A
    
    // Initialize tail pattern
    tail_pattern.clear();
    tail_pattern.push_back(FRAME_TAIL_ARRAY_0);      // 0x00
    tail_pattern.push_back(FRAME_TAIL_ARRAY_1);      // 0xFF
    
    // Initialize buffer
    buffer.clear();
    current_pos = buffer.data();
    buffer_end = buffer.data();
    
    packet_type_temp = 0;
    packet_size = 0;
    crc32_value = 0;
    msg_type_check = 0;
    max_packet_size = 10000;  // 0x2710
}

// Main parsing function
int LidarPacketParser::parseByte(const uint8_t& byte, std::vector<uint8_t>& output_packet) {
    // Add byte to buffer using pointer logic (matching official implementation)
    if (current_pos >= buffer_end) {
        buffer.push_back(byte);
        current_pos = buffer.data() + buffer.size();
        buffer_end = buffer.data() + buffer.capacity();
    } else {
        *current_pos = byte;
        current_pos++;
    }
    
    size_t current_size = current_pos - buffer.data();
    
    // Need at least 4 bytes for header check
    if (current_size <= 4) {
        // Check if the byte matches expected header pattern
        if (buffer[current_size - 1] != header_pattern[current_size - 1]) {
            // Reset if mismatch - only reset pointer, not buffer
            current_pos = buffer.data();
        }
        return 0;
    }
    
    // At 12 bytes, we have the complete header
    if (current_size == 12) {
        // Extract header info
        FrameHeader* header = reinterpret_cast<FrameHeader*>(buffer.data());
        packet_type_temp = header->packet_type;
        packet_size = header->packet_size;
                
        // Validate packet size
        if (packet_size <= 12 || packet_size > max_packet_size) {
            // Invalid packet size, reset pointer only
            current_pos = buffer.data();
            return 0;
        }
    }
    
    // Check if we have a complete packet
    if (current_size == packet_size && packet_size > 0) {
        // Extract tail info
        FrameTail* tail = reinterpret_cast<FrameTail*>(buffer.data() + packet_size - 12);
        crc32_value = tail->crc32;
        msg_type_check = tail->msg_type_check;
        
        // Verify tail pattern
        if (tail->tail[0] != tail_pattern[0] || tail->tail[1] != tail_pattern[1]) {
            // Invalid tail, reset pointer only
            // printf("[PARSER DEBUG] Invalid tail pattern, resetting\n");
            current_pos = buffer.data();
            return 0;
        }

        // Calculate CRC32 on DATA ONLY (excluding 12-byte header and 12-byte tail)
        // The lidar calculates CRC on bytes 12 to (packet_size - 13)
        uint32_t calculated_crc = crc32(buffer.data() + 12, packet_size - 24);
                
        if (calculated_crc != crc32_value) {
            // CRC mismatch, reset pointer only
            printf("[PARSER DEBUG] CRC mismatch, resetting\n");
            current_pos = buffer.data();
            return 0;
        }
        
        // Valid packet found, copy to output
        output_packet.assign(buffer.begin(), buffer.begin() + packet_size);
        
        // printf("[PARSER DEBUG] Valid packet parsed! Type=0x%X (%u)\n", 
        //        packet_type_temp, packet_type_temp);
        
        // Reset pointer for next packet (matching official: only reset pointer)
        current_pos = buffer.data();
        
        // Return packet type
        return packet_type_temp;
    }
    
    return 0;
}

#ifndef __EXTERN_C__
} // namespace unilidar_sdk2
#endif
