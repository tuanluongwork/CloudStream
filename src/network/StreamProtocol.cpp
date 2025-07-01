#include "network/StreamProtocol.h"
#include <cstring>
#include <algorithm>

namespace CloudStream {

// MessageHeader implementation
Protocol::MessageHeader::MessageHeader(MessageType msg_type, uint32_t payload_len)
    : magic(MAGIC_NUMBER)
    , version(VERSION)
    , type(msg_type)
    , flags(0)
    , sequence_number(0)
    , payload_size(payload_len)
    , checksum(0)
    , timestamp(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count()) {
}

bool Protocol::MessageHeader::validate() const {
    return magic == MAGIC_NUMBER && version == VERSION && payload_size <= MAX_PACKET_SIZE;
}

void Protocol::MessageHeader::updateChecksum(const uint8_t* payload, size_t size) {
    // Simple CRC32 would be better, but for demo purposes use a basic checksum
    uint32_t sum = 0;
    for (size_t i = 0; i < size; ++i) {
        sum += payload[i];
        sum = (sum << 1) | (sum >> 31); // Rotate left
    }
    checksum = sum;
}

// MessageBuilder implementation
MessageBuilder::MessageBuilder() : sequence_number_(0) {
}

std::vector<uint8_t> MessageBuilder::buildHandshake(const std::string& client_id) {
    std::vector<uint8_t> payload;
    payload.reserve(client_id.size() + 4);
    
    // Write client ID length and data
    uint32_t id_len = static_cast<uint32_t>(client_id.size());
    payload.insert(payload.end(), reinterpret_cast<uint8_t*>(&id_len), 
                   reinterpret_cast<uint8_t*>(&id_len) + sizeof(id_len));
    payload.insert(payload.end(), client_id.begin(), client_id.end());
    
    return createMessage(Protocol::MessageType::HANDSHAKE, payload);
}

std::vector<uint8_t> MessageBuilder::buildHandshakeAck(bool accepted, const std::string& reason) {
    std::vector<uint8_t> payload;
    payload.push_back(accepted ? 1 : 0);
    
    uint32_t reason_len = static_cast<uint32_t>(reason.size());
    payload.insert(payload.end(), reinterpret_cast<uint8_t*>(&reason_len),
                   reinterpret_cast<uint8_t*>(&reason_len) + sizeof(reason_len));
    payload.insert(payload.end(), reason.begin(), reason.end());
    
    return createMessage(Protocol::MessageType::HANDSHAKE_ACK, payload);
}

std::vector<uint8_t> MessageBuilder::buildPointCloudHeader(const Protocol::PointCloudHeader& header) {
    std::vector<uint8_t> payload;
    payload.resize(sizeof(Protocol::PointCloudHeader));
    std::memcpy(payload.data(), &header, sizeof(header));
    
    return createMessage(Protocol::MessageType::POINT_CLOUD_HEADER, payload);
}

std::vector<uint8_t> MessageBuilder::buildPointCloudChunk(uint32_t chunk_index,
                                                         const std::vector<uint8_t>& data,
                                                         bool compressed) {
    Protocol::PointCloudChunk chunk;
    chunk.chunk_index = chunk_index;
    chunk.point_count = 0; // Should be calculated based on data
    chunk.compressed_size = compressed ? static_cast<uint32_t>(data.size()) : 0;
    chunk.uncompressed_size = static_cast<uint32_t>(data.size());
    
    std::vector<uint8_t> payload;
    payload.reserve(sizeof(chunk) + data.size());
    
    // Write chunk header
    payload.insert(payload.end(), reinterpret_cast<uint8_t*>(&chunk),
                   reinterpret_cast<uint8_t*>(&chunk) + sizeof(chunk));
    
    // Write point data
    payload.insert(payload.end(), data.begin(), data.end());
    
    return createMessage(Protocol::MessageType::POINT_CLOUD_DATA, payload);
}

std::vector<uint8_t> MessageBuilder::buildControlCommand(Protocol::ControlCommand cmd,
                                                        const std::vector<uint8_t>& params) {
    std::vector<uint8_t> payload;
    payload.push_back(static_cast<uint8_t>(cmd));
    payload.insert(payload.end(), params.begin(), params.end());
    
    return createMessage(Protocol::MessageType::CONTROL_COMMAND, payload);
}

std::vector<uint8_t> MessageBuilder::buildStatusUpdate(const Protocol::StatusUpdate& status) {
    std::vector<uint8_t> payload;
    payload.resize(sizeof(Protocol::StatusUpdate));
    std::memcpy(payload.data(), &status, sizeof(status));
    
    return createMessage(Protocol::MessageType::STATUS_UPDATE, payload);
}

std::vector<uint8_t> MessageBuilder::buildError(uint32_t error_code, const std::string& message) {
    std::vector<uint8_t> payload;
    payload.reserve(sizeof(error_code) + sizeof(uint32_t) + message.size());
    
    // Write error code
    payload.insert(payload.end(), reinterpret_cast<uint8_t*>(&error_code),
                   reinterpret_cast<uint8_t*>(&error_code) + sizeof(error_code));
    
    // Write message
    uint32_t msg_len = static_cast<uint32_t>(message.size());
    payload.insert(payload.end(), reinterpret_cast<uint8_t*>(&msg_len),
                   reinterpret_cast<uint8_t*>(&msg_len) + sizeof(msg_len));
    payload.insert(payload.end(), message.begin(), message.end());
    
    return createMessage(Protocol::MessageType::ERROR, payload);
}

std::vector<uint8_t> MessageBuilder::createMessage(Protocol::MessageType type,
                                                   const std::vector<uint8_t>& payload) {
    Protocol::MessageHeader header(type, static_cast<uint32_t>(payload.size()));
    header.sequence_number = sequence_number_++;
    header.updateChecksum(payload.data(), payload.size());
    
    std::vector<uint8_t> message;
    message.reserve(sizeof(header) + payload.size());
    
    // Write header
    message.insert(message.end(), reinterpret_cast<uint8_t*>(&header),
                   reinterpret_cast<uint8_t*>(&header) + sizeof(header));
    
    // Write payload
    message.insert(message.end(), payload.begin(), payload.end());
    
    return message;
}

// MessageParser implementation
MessageParser::MessageParser() : parse_offset_(0) {
    buffer_.reserve(Protocol::MAX_PACKET_SIZE * 2);
}

void MessageParser::addData(const uint8_t* data, size_t size) {
    buffer_.insert(buffer_.end(), data, data + size);
}

bool MessageParser::hasCompleteMessage() const {
    if (buffer_.size() - parse_offset_ < sizeof(Protocol::MessageHeader)) {
        return false;
    }
    
    const Protocol::MessageHeader* header = 
        reinterpret_cast<const Protocol::MessageHeader*>(buffer_.data() + parse_offset_);
    
    return header->validate() && 
           (buffer_.size() - parse_offset_ >= sizeof(Protocol::MessageHeader) + header->payload_size);
}

MessageParser::ParseResult MessageParser::parseNextMessage() {
    ParseResult result;
    
    if (!hasCompleteMessage()) {
        result.error_message = "Incomplete message";
        return result;
    }
    
    const Protocol::MessageHeader* header = 
        reinterpret_cast<const Protocol::MessageHeader*>(buffer_.data() + parse_offset_);
    
    if (!header->validate()) {
        result.error_message = "Invalid message header";
        parse_offset_ = buffer_.size(); // Skip invalid data
        return result;
    }
    
    // Extract payload
    size_t payload_start = parse_offset_ + sizeof(Protocol::MessageHeader);
    result.payload.assign(buffer_.begin() + payload_start,
                         buffer_.begin() + payload_start + header->payload_size);
    
    // Verify checksum
    Protocol::MessageHeader temp_header = *header;
    temp_header.checksum = 0;
    temp_header.updateChecksum(result.payload.data(), result.payload.size());
    
    if (temp_header.checksum != header->checksum) {
        result.error_message = "Checksum mismatch";
        parse_offset_ += sizeof(Protocol::MessageHeader) + header->payload_size;
        return result;
    }
    
    result.success = true;
    result.type = header->type;
    
    // Move parse offset
    parse_offset_ += sizeof(Protocol::MessageHeader) + header->payload_size;
    
    // Clean up buffer if needed
    if (parse_offset_ > Protocol::MAX_PACKET_SIZE) {
        buffer_.erase(buffer_.begin(), buffer_.begin() + parse_offset_);
        parse_offset_ = 0;
    }
    
    return result;
}

void MessageParser::reset() {
    buffer_.clear();
    parse_offset_ = 0;
}

bool MessageParser::parseHandshake(const std::vector<uint8_t>& payload, std::string& client_id) {
    if (payload.size() < sizeof(uint32_t)) {
        return false;
    }
    
    uint32_t id_len;
    std::memcpy(&id_len, payload.data(), sizeof(id_len));
    
    if (payload.size() < sizeof(id_len) + id_len) {
        return false;
    }
    
    client_id.assign(payload.begin() + sizeof(id_len), 
                     payload.begin() + sizeof(id_len) + id_len);
    return true;
}

bool MessageParser::parsePointCloudHeader(const std::vector<uint8_t>& payload,
                                         Protocol::PointCloudHeader& header) {
    if (payload.size() < sizeof(Protocol::PointCloudHeader)) {
        return false;
    }
    
    std::memcpy(&header, payload.data(), sizeof(header));
    return true;
}

bool MessageParser::parsePointCloudChunk(const std::vector<uint8_t>& payload,
                                        uint32_t& chunk_index,
                                        std::vector<uint8_t>& data) {
    if (payload.size() < sizeof(Protocol::PointCloudChunk)) {
        return false;
    }
    
    Protocol::PointCloudChunk chunk;
    std::memcpy(&chunk, payload.data(), sizeof(chunk));
    
    chunk_index = chunk.chunk_index;
    data.assign(payload.begin() + sizeof(chunk), payload.end());
    
    return true;
}

bool MessageParser::parseStatusUpdate(const std::vector<uint8_t>& payload,
                                     Protocol::StatusUpdate& status) {
    if (payload.size() < sizeof(Protocol::StatusUpdate)) {
        return false;
    }
    
    std::memcpy(&status, payload.data(), sizeof(status));
    return true;
}

bool MessageParser::findMessageBoundary() {
    // Look for magic number in buffer
    for (size_t i = parse_offset_; i <= buffer_.size() - sizeof(uint32_t); ++i) {
        uint32_t magic;
        std::memcpy(&magic, buffer_.data() + i, sizeof(magic));
        
        if (magic == Protocol::MAGIC_NUMBER) {
            parse_offset_ = i;
            return true;
        }
    }
    
    // No valid message found, clear buffer
    buffer_.clear();
    parse_offset_ = 0;
    return false;
}

} // namespace CloudStream 