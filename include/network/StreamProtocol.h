#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <memory>
#include <chrono>

namespace CloudStream {

/**
 * @brief Protocol constants and message types for point cloud streaming
 */
namespace Protocol {
    
    constexpr uint32_t MAGIC_NUMBER = 0x434C5354; // "CLST" in hex
    constexpr uint16_t VERSION = 0x0100; // Version 1.0
    constexpr size_t MAX_PACKET_SIZE = 65536; // 64KB max packet size
    constexpr size_t HEADER_SIZE = 24; // Fixed header size
    
    enum class MessageType : uint8_t {
        HANDSHAKE = 0x01,
        HANDSHAKE_ACK = 0x02,
        POINT_CLOUD_HEADER = 0x10,
        POINT_CLOUD_DATA = 0x11,
        POINT_CLOUD_END = 0x12,
        CONTROL_COMMAND = 0x20,
        STATUS_UPDATE = 0x30,
        ERROR = 0xFF
    };
    
    enum class CompressionType : uint8_t {
        NONE = 0x00,
        ZLIB = 0x01,
        LZ4 = 0x02,
        CUSTOM = 0xFF
    };
    
    enum class ControlCommand : uint8_t {
        START_STREAM = 0x01,
        STOP_STREAM = 0x02,
        PAUSE_STREAM = 0x03,
        RESUME_STREAM = 0x04,
        REQUEST_KEYFRAME = 0x10,
        SET_QUALITY = 0x20,
        SET_FPS = 0x21
    };
    
#pragma pack(push, 1)
    /**
     * @brief Fixed-size header for all protocol messages
     */
    struct MessageHeader {
        uint32_t magic;
        uint16_t version;
        MessageType type;
        uint8_t flags;
        uint32_t sequence_number;
        uint32_t payload_size;
        uint32_t checksum;
        uint64_t timestamp;
        
        MessageHeader() = default;
        MessageHeader(MessageType msg_type, uint32_t payload_len);
        
        bool validate() const;
        void updateChecksum(const uint8_t* payload, size_t size);
    };
    
    /**
     * @brief Point cloud header containing metadata
     */
    struct PointCloudHeader {
        uint32_t total_points;
        uint32_t chunk_count;
        CompressionType compression;
        uint8_t has_colors : 1;
        uint8_t has_normals : 1;
        uint8_t has_intensity : 1;
        uint8_t reserved : 5;
        uint16_t point_stride;
        float bounding_box_min[3];
        float bounding_box_max[3];
        char description[64];
    };
    
    /**
     * @brief Point cloud data chunk
     */
    struct PointCloudChunk {
        uint32_t chunk_index;
        uint32_t point_count;
        uint32_t compressed_size;
        uint32_t uncompressed_size;
        // Followed by point data
    };
    
    /**
     * @brief Status update message
     */
    struct StatusUpdate {
        uint32_t points_sent;
        uint32_t points_received;
        float bandwidth_mbps;
        float fps;
        uint32_t dropped_frames;
        uint32_t latency_ms;
    };
    
#pragma pack(pop)
    
} // namespace Protocol

/**
 * @brief Message builder for creating protocol messages
 */
class MessageBuilder {
public:
    MessageBuilder();
    
    std::vector<uint8_t> buildHandshake(const std::string& client_id);
    std::vector<uint8_t> buildHandshakeAck(bool accepted, const std::string& reason = "");
    std::vector<uint8_t> buildPointCloudHeader(const Protocol::PointCloudHeader& header);
    std::vector<uint8_t> buildPointCloudChunk(uint32_t chunk_index, 
                                              const std::vector<uint8_t>& data,
                                              bool compressed = false);
    std::vector<uint8_t> buildControlCommand(Protocol::ControlCommand cmd, 
                                            const std::vector<uint8_t>& params = {});
    std::vector<uint8_t> buildStatusUpdate(const Protocol::StatusUpdate& status);
    std::vector<uint8_t> buildError(uint32_t error_code, const std::string& message);
    
    // Public access to createMessage for custom messages
    std::vector<uint8_t> createMessage(Protocol::MessageType type, 
                                      const std::vector<uint8_t>& payload);
    
private:
    uint32_t sequence_number_{0};
};

/**
 * @brief Message parser for decoding protocol messages
 */
class MessageParser {
public:
    struct ParseResult {
        bool success{false};
        Protocol::MessageType type{Protocol::MessageType::ERROR};
        std::vector<uint8_t> payload;
        std::string error_message;
    };
    
    MessageParser();
    
    void addData(const uint8_t* data, size_t size);
    bool hasCompleteMessage() const;
    ParseResult parseNextMessage();
    void reset();
    
    // Specific message parsers
    static bool parseHandshake(const std::vector<uint8_t>& payload, std::string& client_id);
    static bool parsePointCloudHeader(const std::vector<uint8_t>& payload, 
                                     Protocol::PointCloudHeader& header);
    static bool parsePointCloudChunk(const std::vector<uint8_t>& payload,
                                    uint32_t& chunk_index,
                                    std::vector<uint8_t>& data);
    static bool parseStatusUpdate(const std::vector<uint8_t>& payload,
                                 Protocol::StatusUpdate& status);
    
private:
    std::vector<uint8_t> buffer_;
    size_t parse_offset_{0};
    
    bool findMessageBoundary();
};

} // namespace CloudStream 