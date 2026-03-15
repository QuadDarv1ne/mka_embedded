/**
 * @file telemetry.hpp
 * @brief System of telemetry and commands for MKA
 *
 * Components:
 * - Telemetry frame generator (CCSDS compatible)
 * - Command handler with validation
 * - Parameter manager
 * - CRC computation
 */

#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>
#include <functional>
#include <type_traits>

#include "utils/callback.hpp"

namespace mka {
namespace telemetry {

// ============================================================================
// Constants and types
// ============================================================================

/// Maximum telemetry frame size
constexpr size_t MAX_FRAME_SIZE = 256;

/// Maximum command payload size
constexpr size_t MAX_COMMAND_PAYLOAD = 240;

/// Number of telemetry channels
constexpr size_t MAX_TELEMETRY_CHANNELS = 32;

/// CRC-16 size
constexpr size_t CRC_SIZE = 2;

// ============================================================================
// CCSDS Frame Format (simplified)
// ============================================================================

/**
 * @brief Telemetry header (Primary Header)
 */
struct TelemetryHeader {
    uint16_t word0;  // Version + Type + SH + APID
    uint16_t word1;  // Flags + Sequence Count
    uint16_t length; // Packet Length (data only, minus 1)

    static constexpr uint16_t VERSION = 0b000;
    static constexpr uint16_t TYPE_TM = 0b0;
    static constexpr uint16_t TYPE_TC = 0b1;

    void set(uint16_t apid, uint16_t sequenceCount, size_t dataLength,
             bool lastFrame = true) {
        word0 = (VERSION << 13) | (TYPE_TM << 12) | (0 << 11) | (apid & 0x7FF);
        
        uint16_t sequenceFlags = lastFrame ? 0b11 : 0b01;
        word1 = (sequenceFlags << 14) | (sequenceCount & 0x3FFF);
        
        length = static_cast<uint16_t>(dataLength + CRC_SIZE - 1);
    }
};

static_assert(sizeof(TelemetryHeader) == 6, "Header must be 6 bytes");

/**
 * @brief Command header
 */
struct CommandHeader {
    uint16_t word0;  // Version + Type + SH + APID
    uint16_t word1;  // Flags + Sequence Count
    uint8_t commandId;
    uint8_t payloadLength;

    void set(uint16_t apid, uint16_t sequenceCount, uint8_t cmdId,
             uint8_t payloadLen) {
        word0 = (0b000 << 13) | (TYPE_TC << 12) | (0 << 11) | (apid & 0x7FF);
        word1 = 0x0000 | (sequenceCount & 0x3FFF);
        commandId = cmdId;
        payloadLength = payloadLen;
    }

private:
    static constexpr uint16_t TYPE_TC = 0b1;
};

// CommandHeader is 6 bytes: 2 + 2 + 1 + 1
static_assert(sizeof(CommandHeader) == 6, "Command header must be 6 bytes");

// ============================================================================
// CRC computation
// ============================================================================

/**
 * @brief CRC-16-CCITT for integrity check
 */
class CRC16 {
public:
    static uint16_t calculate(const uint8_t* data, size_t length,
                              uint16_t initialValue = 0xFFFF) {
        uint16_t crc = initialValue;
        
        for (size_t i = 0; i < length; ++i) {
            crc ^= static_cast<uint16_t>(data[i]) << 8;
            for (uint8_t j = 0; j < 8; ++j) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
        
        return crc;
    }

    static bool verify(const uint8_t* data, size_t length) {
        if (length < CRC_SIZE) return false;
        
        uint16_t expected = (static_cast<uint16_t>(data[length - 2]) << 8) |
                            data[length - 1];
        uint16_t calculated = calculate(data, length - CRC_SIZE);
        
        return expected == calculated;
    }
};

// ============================================================================
// Telemetry data types
// ============================================================================

enum class TelemetryType : uint8_t {
    UINT8,
    UINT16,
    UINT32,
    UINT64,
    INT8,
    INT16,
    INT32,
    INT64,
    FLOAT,
    DOUBLE,
    BOOLEAN,
    ENUM,
    RAW_BYTES
};

struct TelemetryChannel {
    const char* name;
    TelemetryType type;
    uint8_t id;
    float scaleFactor;
    float offset;
    const char* unit;
    float minValid;
    float maxValid;
    bool enabled;
};

union TelemetryValue {
    uint8_t u8;
    uint16_t u16;
    uint32_t u32;
    uint64_t u64;
    int8_t i8;
    int16_t i16;
    int32_t i32;
    int64_t i64;
    float f;
    double d;
    bool b;
    uint8_t raw[8];

    TelemetryValue() : u64(0) {}
};

struct TelemetryPoint {
    uint8_t channelId;
    TelemetryValue value;
    uint32_t timestamp;
    uint8_t status;
};

// ============================================================================
// Telemetry Generator
// ============================================================================

class TelemetryGenerator {
public:
    enum class FrameStatus {
        OK,
        BUFFER_TOO_SMALL,
        NO_DATA,
        INVALID_CHANNEL,
        CRC_ERROR
    };

    TelemetryGenerator(uint8_t* buffer, size_t bufferSize)
        : buffer_(buffer)
        , bufferSize_(bufferSize)
        , sequenceCount_(0)
        , currentApId_(0)
    {}

    uint8_t registerChannel(const TelemetryChannel& channel) {
        if (channelCount_ >= MAX_TELEMETRY_CHANNELS) {
            return 0xFF;
        }
        
        channels_[channelCount_] = channel;
        channels_[channelCount_].id = static_cast<uint8_t>(channelCount_);
        return channelCount_++;
    }

    template<typename T>
    FrameStatus updateValue(uint8_t channelId, const T& value,
                            uint32_t timestamp = 0) {
        if (channelId >= channelCount_) {
            return FrameStatus::INVALID_CHANNEL;
        }

        for (size_t i = 0; i < MAX_RECENT_VALUES; ++i) {
            if (recentValues_[i].channelId == channelId ||
                recentValues_[i].channelId == 0xFF) {
                recentValues_[i].channelId = channelId;
                recentValues_[i].timestamp = timestamp;
                recentValues_[i].status = 0;
                copyValue(recentValues_[i].value, value);
                return FrameStatus::OK;
            }
        }

        recentValues_[nextReplace_].channelId = channelId;
        recentValues_[nextReplace_].timestamp = timestamp;
        recentValues_[nextReplace_].status = 0;
        copyValue(recentValues_[nextReplace_].value, value);
        nextReplace_ = (nextReplace_ + 1) % MAX_RECENT_VALUES;

        return FrameStatus::OK;
    }

    FrameStatus generateFrame(uint16_t apid, uint32_t /*includeChannels*/ = 0xFFFFFFFF,
                              size_t* frameSize = nullptr) {
        currentApId_ = apid;
        
        TelemetryHeader* header = reinterpret_cast<TelemetryHeader*>(buffer_);
        header->set(apid, sequenceCount_, 0, true);
        
        size_t offset = sizeof(TelemetryHeader);
        size_t dataStart = offset;
        
        uint8_t channelsAdded = 0;
        for (size_t i = 0; i < channelCount_ && offset < bufferSize_; ++i) {
            if (!channels_[i].enabled) continue;
            
            TelemetryValue* valuePtr = findRecentValue(static_cast<uint8_t>(i));
            if (!valuePtr) continue;
            
            if (offset + 1 >= bufferSize_) {
                return FrameStatus::BUFFER_TOO_SMALL;
            }
            buffer_[offset++] = static_cast<uint8_t>(i);
            
            FrameStatus status = writeValueByType(offset, static_cast<uint8_t>(i), *valuePtr);
            if (status != FrameStatus::OK) {
                return status;
            }
            
            channelsAdded++;
        }

        if (channelsAdded == 0) {
            return FrameStatus::NO_DATA;
        }

        uint16_t crc = CRC16::calculate(buffer_ + dataStart, offset - dataStart);
        if (offset + CRC_SIZE > bufferSize_) {
            return FrameStatus::BUFFER_TOO_SMALL;
        }
        
        buffer_[offset++] = static_cast<uint8_t>(crc >> 8);
        buffer_[offset++] = static_cast<uint8_t>(crc & 0xFF);

        header->set(apid, sequenceCount_++, offset - dataStart, true);

        if (frameSize) {
            *frameSize = offset;
        }

        return FrameStatus::OK;
    }

    void setChannelEnabled(uint8_t channelId, bool enabled) {
        if (channelId < channelCount_) {
            channels_[channelId].enabled = enabled;
        }
    }

    size_t getChannelCount() const { return channelCount_; }
    void resetSequenceCount() { sequenceCount_ = 0; }

private:
    static constexpr size_t MAX_RECENT_VALUES = 64;
    
    uint8_t* buffer_;
    size_t bufferSize_;
    uint16_t sequenceCount_;
    uint16_t currentApId_;
    
    std::array<TelemetryChannel, MAX_TELEMETRY_CHANNELS> channels_{};
    size_t channelCount_ = 0;
    
    std::array<TelemetryPoint, MAX_RECENT_VALUES> recentValues_{};
    size_t nextReplace_ = 0;

    template<typename T>
    void copyValue(TelemetryValue& dest, const T& src) {
        if constexpr (sizeof(T) <= sizeof(TelemetryValue)) {
            dest.u64 = 0;
            // Use union member assignment to avoid -Wclass-memaccess
            if constexpr (std::is_same<T, float>::value) {
                dest.f = src;
            } else if constexpr (std::is_same<T, double>::value) {
                dest.d = src;
            } else if constexpr (std::is_same<T, uint8_t>::value) {
                dest.u8 = src;
            } else if constexpr (std::is_same<T, uint16_t>::value) {
                dest.u16 = src;
            } else if constexpr (std::is_same<T, uint32_t>::value) {
                dest.u32 = src;
            } else if constexpr (std::is_same<T, uint64_t>::value) {
                dest.u64 = src;
            } else if constexpr (std::is_same<T, int8_t>::value) {
                dest.i8 = static_cast<int8_t>(src);
            } else if constexpr (std::is_same<T, int16_t>::value) {
                dest.i16 = static_cast<int16_t>(src);
            } else if constexpr (std::is_same<T, int32_t>::value) {
                dest.i32 = static_cast<int32_t>(src);
            } else if constexpr (std::is_same<T, int64_t>::value) {
                dest.i64 = static_cast<int64_t>(src);
            } else if constexpr (std::is_same<T, bool>::value) {
                dest.b = src;
            } else {
                // Fallback for other types
                std::memcpy(&dest, &src, sizeof(T));
            }
        }
    }

    TelemetryValue* findRecentValue(uint8_t channelId) {
        for (size_t i = 0; i < MAX_RECENT_VALUES; ++i) {
            if (recentValues_[i].channelId == channelId) {
                return &recentValues_[i].value;
            }
        }
        return nullptr;
    }

    FrameStatus writeValueByType(size_t& offset, uint8_t channelId,
                                 const TelemetryValue& value) {
        if (channelId >= channelCount_) {
            return FrameStatus::INVALID_CHANNEL;
        }

        const auto& channel = channels_[channelId];
        
        switch (channel.type) {
            case TelemetryType::UINT8:
                if (offset + 1 > bufferSize_) return FrameStatus::BUFFER_TOO_SMALL;
                buffer_[offset++] = value.u8;
                break;
                
            case TelemetryType::UINT16:
                if (offset + 2 > bufferSize_) return FrameStatus::BUFFER_TOO_SMALL;
                buffer_[offset++] = value.u16 & 0xFF;
                buffer_[offset++] = (value.u16 >> 8) & 0xFF;
                break;
                
            case TelemetryType::UINT32:
            case TelemetryType::FLOAT:
                if (offset + 4 > bufferSize_) return FrameStatus::BUFFER_TOO_SMALL;
                buffer_[offset++] = value.u32 & 0xFF;
                buffer_[offset++] = (value.u32 >> 8) & 0xFF;
                buffer_[offset++] = (value.u32 >> 16) & 0xFF;
                buffer_[offset++] = (value.u32 >> 24) & 0xFF;
                break;
                
            case TelemetryType::UINT64:
            case TelemetryType::DOUBLE:
                if (offset + 8 > bufferSize_) return FrameStatus::BUFFER_TOO_SMALL;
                for (int i = 0; i < 8; ++i) {
                    buffer_[offset++] = (value.u64 >> (i * 8)) & 0xFF;
                }
                break;
                
            default:
                if (offset + 4 > bufferSize_) return FrameStatus::BUFFER_TOO_SMALL;
                buffer_[offset++] = value.u32 & 0xFF;
                buffer_[offset++] = (value.u32 >> 8) & 0xFF;
                buffer_[offset++] = (value.u32 >> 16) & 0xFF;
                buffer_[offset++] = (value.u32 >> 24) & 0xFF;
                break;
        }

        return FrameStatus::OK;
    }
};

// ============================================================================
// Command Manager
// ============================================================================

enum class CommandResult : uint8_t {
    OK = 0,
    ERROR = 1,
    INVALID_COMMAND = 2,
    INVALID_PAYLOAD = 3,
    CRC_ERROR = 4,
    TIMEOUT = 5,
    BUSY = 6,
    NOT_AUTHORIZED = 7
};

struct Command {
    uint8_t id;
    const uint8_t* payload;
    size_t payloadSize;
    uint32_t timestamp;
    uint16_t sourceApId;
};

using CommandHandler = std::function<CommandResult(const Command&)>;

class CommandManager {
public:
    static constexpr size_t MAX_COMMANDS = 16;

    CommandManager() = default;

    bool registerHandler(uint8_t commandId, CommandHandler handler) {
        if (handlerCount_ >= MAX_COMMANDS) {
            return false;
        }

        handlers_[handlerCount_] = {commandId, handler};
        handlerCount_++;
        return true;
    }

    CommandResult processCommand(const uint8_t* data, size_t length,
                                 CommandResult* result = nullptr) {
        if (length < sizeof(CommandHeader) + CRC_SIZE) {
            return CommandResult::INVALID_PAYLOAD;
        }

        if (!CRC16::verify(data, length)) {
            return CommandResult::CRC_ERROR;
        }

        const CommandHeader* header = reinterpret_cast<const CommandHeader*>(data);
        
        CommandHandler handler = findHandler(header->commandId);
        if (!handler) {
            return CommandResult::INVALID_COMMAND;
        }

        Command cmd;
        cmd.id = header->commandId;
        cmd.payload = data + sizeof(CommandHeader);
        cmd.payloadSize = header->payloadLength;
        cmd.timestamp = 0;
        cmd.sourceApId = header->word0 & 0x7FF;

        CommandResult execResult = handler(cmd);
        
        if (result) {
            *result = execResult;
        }

        return execResult;
    }

    bool buildAcknowledgment(uint8_t commandId, CommandResult result,
                             uint8_t* buffer, size_t bufferSize, size_t* frameSize) {
        if (bufferSize < 6) {
            return false;
        }

        buffer[0] = commandId;
        buffer[1] = static_cast<uint8_t>(result);
        buffer[2] = 0x00;
        buffer[3] = 0x00;

        uint16_t crc = CRC16::calculate(buffer, 4);
        buffer[4] = static_cast<uint8_t>(crc >> 8);
        buffer[5] = static_cast<uint8_t>(crc & 0xFF);

        if (frameSize) {
            *frameSize = 6;
        }

        return true;
    }

    void setTimestampThreshold(uint32_t maxAgeSeconds) {
        timestampThreshold_ = maxAgeSeconds;
        useTimestampValidation_ = true;
    }

    void disableTimestampValidation() {
        useTimestampValidation_ = false;
    }

private:
    struct HandlerEntry {
        uint8_t commandId;
        CommandHandler handler;
    };

    std::array<HandlerEntry, MAX_COMMANDS> handlers_{};
    size_t handlerCount_ = 0;
    
    uint32_t timestampThreshold_ = 60;
    bool useTimestampValidation_ = false;

    CommandHandler findHandler(uint8_t commandId) {
        for (size_t i = 0; i < handlerCount_; ++i) {
            if (handlers_[i].commandId == commandId) {
                return handlers_[i].handler;
            }
        }
        return nullptr;
    }
};

// ============================================================================
// Parameter Store
// ============================================================================

enum class ParameterType : uint8_t {
    UINT8,
    UINT16,
    UINT32,
    INT8,
    INT16,
    INT32,
    FLOAT,
    BOOLEAN,
    ENUM
};

struct ParameterAttributes {
    const char* name;
    ParameterType type;
    uint16_t id;
    bool persistent;
    float minValue;
    float maxValue;
    float defaultValue;
};

class ParameterStore {
public:
    static constexpr size_t MAX_PARAMETERS = 64;

    ParameterStore() = default;

    uint16_t registerParameter(const ParameterAttributes& attrs,
                               float initialValue = 0.0f) {
        if (paramCount_ >= MAX_PARAMETERS) {
            return 0xFFFF;
        }

        attributes_[paramCount_] = attrs;
        values_[paramCount_] = initialValue;
        return static_cast<uint16_t>(paramCount_++);
    }

    bool getParameter(uint16_t id, float& value) const {
        if (id >= paramCount_) {
            return false;
        }

        value = values_[id];
        return true;
    }

    bool setParameter(uint16_t id, float value, bool validate = true) {
        if (id >= paramCount_) {
            return false;
        }

        if (validate) {
            const auto& attrs = attributes_[id];
            if (value < attrs.minValue || value > attrs.maxValue) {
                return false;
            }
        }

        values_[id] = value;
        return true;
    }

    bool resetParameter(uint16_t id) {
        if (id >= paramCount_) {
            return false;
        }

        values_[id] = attributes_[id].defaultValue;
        return true;
    }

    void resetAllParameters() {
        for (size_t i = 0; i < paramCount_; ++i) {
            values_[i] = attributes_[i].defaultValue;
        }
    }

    size_t getParameterCount() const { return paramCount_; }

    const ParameterAttributes* getParameterAttributes(uint16_t id) const {
        if (id >= paramCount_) {
            return nullptr;
        }
        return &attributes_[id];
    }

private:
    std::array<ParameterAttributes, MAX_PARAMETERS> attributes_{};
    std::array<float, MAX_PARAMETERS> values_{};
    size_t paramCount_ = 0;
};

} // namespace telemetry
} // namespace mka

#endif // TELEMETRY_HPP
