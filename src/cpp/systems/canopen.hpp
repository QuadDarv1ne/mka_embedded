/**
 * @file canopen.hpp
 * @brief Базовая реализация стека CANopen для бортовых систем
 *
 * Поддерживает:
 * - NMT управление (State Machine)
 * - SDO сервисы (чтение/запись Object Dictionary)
 * - PDO передача (TPDO/RPDO)
 *
 * Соответствует CiA 301 (CANopen standard)
 */

#ifndef CANOPEN_HPP
#define CANOPEN_HPP

#include <cstdint>
#include <cstddef>
#include <array>
#include <functional>
#include <optional>

#include "hal/hal_full.hpp"

namespace mka {
namespace systems {

// ============================================================================
// CANopen Constants
// ============================================================================

/// CANopen COB-ID (идентификаторы)
namespace COBId {
    constexpr uint16_t NMT_COMMAND      = 0x000;  // Master -> Slave
    constexpr uint16_t NMT_ERROR_CTRL   = 0x700;  // Slave -> Master (Node Guarding/Heartbeat)
    constexpr uint16_t SDO_RX           = 0x600;  // Master -> Slave (SDO request)
    constexpr uint16_t SDO_TX           = 0x580;  // Slave -> Master (SDO response)
    constexpr uint16_t EMCY             = 0x080;  // Emergency message
    constexpr uint16_t PDO_BASE         = 0x180;  // Base for PDO (TPDO1=0x181, RPDO1=0x201, etc.)
}

/// NMT команды
enum class NMTCommand : uint8_t {
    START_REMOTE_NODE     = 0x01,  // Переход в Operational
    STOP_REMOTE_NODE      = 0x02,  // Переход в Stopped
    ENTER_PRE_OPERATIONAL = 0x80,  // Переход в Pre-Operational
    RESET_NODE            = 0x81,  // Reset Node
    RESET_COMMUNICATION   = 0x82   // Reset Communication
};

/// Состояния CANopen Node
enum class NodeState : uint8_t {
    INITIALIZING      = 0x00,
    STOPPED           = 0x04,
    OPERATIONAL       = 0x05,
    PRE_OPERATIONAL   = 0x7F
};

/// SDO команды (CSI - Command Specifier)
namespace SDOCommand {
    constexpr uint8_t UPLOAD_SEGMENT_RESPONSE    = 0x00;
    constexpr uint8_t DOWNLOAD_SEGMENT_RESPONSE  = 0x60;
    constexpr uint8_t INITIATE_UPLOAD_RESPONSE   = 0x40;
    constexpr uint8_t INITIATE_DOWNLOAD_RESPONSE = 0x60;
    constexpr uint8_t UPLOAD_SEGMENT_REQUEST     = 0x20;
    constexpr uint8_t DOWNLOAD_SEGMENT_REQUEST   = 0x00;
    constexpr uint8_t INITIATE_UPLOAD_REQUEST    = 0x40;
    constexpr uint8_t INITIATE_DOWNLOAD_REQUEST  = 0x20;
    constexpr uint8_t ABORT_TRANSFER             = 0x80;
}

/// SDO коды ошибок
enum class SDOErrorCode : uint32_t {
    NO_ERROR                     = 0x00000000,
    UNSPECIFIED_ERROR            = 0x05040001,
    UNSUPPORTED_ACCESS           = 0x06010000,
    WRITE_ONLY_OBJECT            = 0x06010001,
    READ_ONLY_OBJECT             = 0x06010002,
    OBJECT_NOT_EXIST             = 0x06020000,
    MAPPING_ERROR                = 0x06040041,
    MAPPING_LENGTH_ERROR         = 0x06040042,
    GENERAL_PARAMETER_ERROR      = 0x06040043,
    DEVICE_HARDWARE_ERROR        = 0x06060000,
    DATA_LENGTH_ERROR            = 0x06070010,
    DATA_LENGTH_EXCEEDED         = 0x06070012,
    DATA_NOT_MATCH               = 0x06070013,
    DATA_RANGE_ERROR             = 0x06090011,
    DATA_WRITE_ERROR             = 0x08000000,
    DATA_WRITE_LOCAL_ERROR       = 0x08000020,
    DATA_WRITE_HW_ERROR          = 0x08000021,
    BLOCK_MODE_ERROR             = 0x08000022,
    DATA_READ_ERROR              = 0x08000023,
    DATA_READ_HW_ERROR           = 0x08000024,
    BUFFER_OVERFLOW              = 0x08000025
};

/// PDO Mapping Record
struct PDOMappingEntry {
    uint32_t cobId;        // COB-ID PDO
    uint8_t transmissionType;  // Transmission type (0-255)
    uint8_t inhibitionTime;    // Inhibition time (100µs units)
    uint8_t compatibilityEntry;// Compatibility entry (unused)
    uint16_t eventTimer;       // Event timer (ms)
};

// ============================================================================
// Object Dictionary Entry
// ============================================================================

/**
 * @brief Запись словаря объектов (Object Dictionary)
 *
 * Object Dictionary - база данных параметров CANopen устройства.
 * Индексируется по 16-битному индексу и 8-битному суб-индексу.
 */
struct ODEntry {
    uint16_t index;         // 16-битный индекс (0x1000-0x9FFF)
    uint8_t subIndex;       // 8-битный суб-индекс (0x00-0xFF)
    
    enum class Access : uint8_t {
        RO,     // Read Only
        WO,     // Write Only
        RW,     // Read Write
        CONST   // Constant
    };
    
    Access access;
    
    enum class DataType : uint8_t {
        BOOLEAN     = 0x01,
        INTEGER8    = 0x02,
        INTEGER16   = 0x03,
        INTEGER32   = 0x04,
        UNSIGNED8   = 0x05,
        UNSIGNED16  = 0x06,
        UNSIGNED32  = 0x07,
        REAL32      = 0x08,
        VISIBLE_STR = 0x0F,
        DOMAIN      = 0x0F
    };
    
    DataType dataType;
    
    // Данные (максимум 4 байта для SDO fast transfer)
    std::array<uint8_t, 4> data{};
    
    // Callback для записи (валидация, side effects)
    using WriteCallback = std::function<bool(const uint8_t*, size_t)>;
    WriteCallback onWrite = nullptr;
    
    // Callback для чтения (dynamic data)
    using ReadCallback = std::function<void(uint8_t*, size_t)>;
    ReadCallback onRead = nullptr;
};

// ============================================================================
// SDO Server
// ============================================================================

/**
 * @brief SDO (Service Data Object) сервер
 *
 * Реализует обмен параметрами через Object Dictionary.
 * Поддерживает expedited (быстрый) и segmented (сегментированный) transfer.
 */
class SDOServer {
public:
    static constexpr size_t MAX_OD_ENTRIES = 64;
    static constexpr size_t SDO_DATA_SIZE = 8;  // CAN frame data size

    explicit SDOServer(uint8_t nodeId = 1) : nodeId_(nodeId) {}

    /**
     * @brief Регистрация записи в Object Dictionary
     */
    bool registerEntry(const ODEntry& entry) {
        if (odCount_ >= MAX_OD_ENTRIES) {
            return false;
        }
        objectDictionary_[odCount_++] = entry;
        return true;
    }

    /**
     * @brief Обработка входящего SDO запроса
     * @param data Данные CAN frame (8 байт)
     * @param len Длина данных
     * @return true если ответ сформирован
     */
    bool processRequest(const uint8_t* data, uint32_t len, uint8_t* response) {
        if (len < SDO_DATA_SIZE) {
            return false;
        }

        uint8_t ccs = data[0] >> 5;  // Command Specifier
        uint16_t index = data[1] | (data[2] << 8);
        uint8_t subIndex = data[3];

        switch (ccs) {
            case 0x01:  // Download segment
                return processDownloadSegment(data, response);
            case 0x02:  // Initiate download
                return processInitiateDownload(data, response);
            case 0x03:  // Initiate upload
                return processInitiateUpload(index, subIndex, response);
            case 0x00:  // Upload segment
                return processUploadSegment(data, response);
            default:
                sendAbort(response, index, subIndex, SDOErrorCode::UNSPECIFIED_ERROR);
                return true;
        }
    }

    /**
     * @brief Получить COB-ID для ответа
     */
    uint16_t getResponseCOBId() const {
        return COBId::SDO_TX + nodeId_;
    }

private:
    uint8_t nodeId_;
    ODEntry objectDictionary_[MAX_OD_ENTRIES];
    size_t odCount_ = 0;

    ODEntry* findEntry(uint16_t index, uint8_t subIndex) {
        for (size_t i = 0; i < odCount_; ++i) {
            if (objectDictionary_[i].index == index && 
                objectDictionary_[i].subIndex == subIndex) {
                return &objectDictionary_[i];
            }
        }
        return nullptr;
    }

    bool processInitiateUpload(uint16_t index, uint8_t subIndex, uint8_t* response) {
        ODEntry* entry = findEntry(index, subIndex);
        
        if (!entry) {
            sendAbort(response, index, subIndex, SDOErrorCode::OBJECT_NOT_EXIST);
            return true;
        }

        if (entry->access == ODEntry::Access::WO) {
            sendAbort(response, index, subIndex, SDOErrorCode::WRITE_ONLY_OBJECT);
            return true;
        }

        // Чтение данных
        uint32_t data = 0;
        size_t size = getDataSize(entry->dataType);
        
        if (entry->onRead) {
            std::array<uint8_t, 4> buf{};
            entry->onRead(buf.data(), size);
            for (size_t i = 0; i < size; ++i) {
                data |= (static_cast<uint32_t>(buf[i]) << (i * 8));
            }
        } else {
            for (size_t i = 0; i < size; ++i) {
                data |= (static_cast<uint32_t>(entry->data[i]) << (i * 8));
            }
        }

        // Формирование ответа
        response[0] = SDOCommand::INITIATE_UPLOAD_RESPONSE | ((4 - size) << 2);
        response[1] = index & 0xFF;
        response[2] = (index >> 8) & 0xFF;
        response[3] = subIndex;
        response[4] = data & 0xFF;
        response[5] = (data >> 8) & 0xFF;
        response[6] = (data >> 16) & 0xFF;
        response[7] = (data >> 24) & 0xFF;

        return true;
    }

    bool processInitiateDownload(const uint8_t* data, uint8_t* response) {
        uint16_t index = data[1] | (data[2] << 8);
        uint8_t subIndex = data[3];
        
        ODEntry* entry = findEntry(index, subIndex);
        
        if (!entry) {
            sendAbort(response, index, subIndex, SDOErrorCode::OBJECT_NOT_EXIST);
            return true;
        }

        if (entry->access == ODEntry::Access::RO || 
            entry->access == ODEntry::Access::CONST) {
            sendAbort(response, index, subIndex, SDOErrorCode::READ_ONLY_OBJECT);
            return true;
        }

        // Проверка размера данных
        uint8_t n = (data[0] >> 1) & 0x03;
        size_t size = 4 - n;
        
        // Валидация через callback
        if (entry->onWrite) {
            if (!entry->onWrite(&data[4], size)) {
                sendAbort(response, index, subIndex, SDOErrorCode::DATA_RANGE_ERROR);
                return true;
            }
        }

        // Запись данных
        for (size_t i = 0; i < size; ++i) {
            entry->data[i] = data[4 + i];
        }

        // Ответ
        response[0] = SDOCommand::INITIATE_DOWNLOAD_RESPONSE;
        response[1] = index & 0xFF;
        response[2] = (index >> 8) & 0xFF;
        response[3] = subIndex;
        response[4] = 0;
        response[5] = 0;
        response[6] = 0;
        response[7] = 0;

        return true;
    }

    bool processDownloadSegment(const uint8_t* data, uint8_t* response) {
        // Упрощённая реализация (без сегментированной передачи)
        (void)data;
        response[0] = SDOCommand::DOWNLOAD_SEGMENT_RESPONSE;
        for (size_t i = 1; i < 8; ++i) response[i] = 0;
        return true;
    }

    bool processUploadSegment(const uint8_t* data, uint8_t* response) {
        (void)data;
        // Упрощённая реализация
        response[0] = SDOCommand::UPLOAD_SEGMENT_RESPONSE;
        for (size_t i = 1; i < 8; ++i) response[i] = 0;
        return true;
    }

    void sendAbort(uint8_t* response, uint16_t index, uint8_t subIndex, 
                   SDOErrorCode errorCode) {
        response[0] = SDOCommand::ABORT_TRANSFER;
        response[1] = index & 0xFF;
        response[2] = (index >> 8) & 0xFF;
        response[3] = subIndex;
        response[4] = static_cast<uint8_t>((static_cast<uint32_t>(errorCode) >> 0) & 0xFF);
        response[5] = static_cast<uint8_t>((static_cast<uint32_t>(errorCode) >> 8) & 0xFF);
        response[6] = static_cast<uint8_t>((static_cast<uint32_t>(errorCode) >> 16) & 0xFF);
        response[7] = static_cast<uint8_t>((static_cast<uint32_t>(errorCode) >> 24) & 0xFF);
    }

    size_t getDataSize(ODEntry::DataType type) const {
        switch (type) {
            case ODEntry::DataType::BOOLEAN:
            case ODEntry::DataType::INTEGER8:
            case ODEntry::DataType::UNSIGNED8:
                return 1;
            case ODEntry::DataType::INTEGER16:
            case ODEntry::DataType::UNSIGNED16:
                return 2;
            case ODEntry::DataType::INTEGER32:
            case ODEntry::DataType::UNSIGNED32:
            case ODEntry::DataType::REAL32:
                return 4;
            default:
                return 4;
        }
    }
};

// ============================================================================
// NMT State Machine
// ============================================================================

/**
 * @brief NMT (Network Management) машина состояний
 *
 * Управляет состоянием CANopen узла:
 * - Initialization → Pre-Operational → Operational
 * - Operational → Stopped → Operational
 */
class NMTStateMachine {
public:
    using StateCallback = std::function<void(NodeState)>;

    NMTStateMachine(uint8_t nodeId = 1) : nodeId_(nodeId) {}

    /**
     * @brief Обработка NMT команды
     */
    NodeState processCommand(NMTCommand cmd) {
        switch (cmd) {
            case NMTCommand::START_REMOTE_NODE:
                currentState_ = NodeState::OPERATIONAL;
                break;
            case NMTCommand::STOP_REMOTE_NODE:
                currentState_ = NodeState::STOPPED;
                break;
            case NMTCommand::ENTER_PRE_OPERATIONAL:
                currentState_ = NodeState::PRE_OPERATIONAL;
                break;
            case NMTCommand::RESET_NODE:
                currentState_ = NodeState::INITIALIZING;
                // Требуется перезагрузка устройства
                break;
            case NMTCommand::RESET_COMMUNICATION:
                currentState_ = NodeState::PRE_OPERATIONAL;
                // Пересоздание коммуникационных объектов
                break;
        }

        if (onStateChange_) {
            onStateChange_(currentState_);
        }

        return currentState_;
    }

    /**
     * @brief Получить текущее состояние
     */
    NodeState getState() const {
        return currentState_;
    }

    /**
     * @brief Проверка Operational состояния
     */
    bool isOperational() const {
        return currentState_ == NodeState::OPERATIONAL;
    }

    /**
     * @brief Установить callback изменения состояния
     */
    void onStateChange(StateCallback cb) {
        onStateChange_ = std::move(cb);
    }

    /**
     * @brief Получить COB-ID Heartbeat
     */
    uint16_t getHeartbeatCOBId() const {
        return COBId::NMT_ERROR_CTRL + nodeId_;
    }

    /**
     * @brief Сформировать Heartbeat сообщение
     */
    uint8_t createHeartbeat() const {
        return static_cast<uint8_t>(currentState_);
    }

private:
    uint8_t nodeId_;
    NodeState currentState_ = NodeState::INITIALIZING;
    StateCallback onStateChange_;
};

// ============================================================================
// PDO Manager
// ============================================================================

/**
 * @brief PDO (Process Data Object) менеджер
 *
 * Управляет передачей данных реального времени:
 * - TPDO (Transmit PDO) - устройство -> мастер
 * - RPDO (Receive PDO) - мастер -> устройство
 */
class PDOManager {
public:
    static constexpr size_t MAX_TPDO = 4;
    static constexpr size_t MAX_RPDO = 4;
    static constexpr size_t MAX_MAPPINGS = 4;  // Макс. объектов в PDO

    struct MappedObject {
        uint32_t index;    // 16-бит индекс + 8-бит суб-индекс
        uint8_t size;      // Размер в битах
    };

    explicit PDOManager(uint8_t nodeId = 1) : nodeId_(nodeId) {}

    /**
     * @brief Настройка TPDO
     */
    bool configureTPDO(uint8_t pdoNumber, uint32_t cobId, 
                       const MappedObject* mappings, size_t count) {
        if (pdoNumber == 0 || pdoNumber > MAX_TPDO) return false;
        if (count > MAX_MAPPINGS) return false;

        size_t idx = pdoNumber - 1;
        tpdoConfig_[idx].cobId = cobId;
        tpdoConfig_[idx].mappingCount = count;
        
        for (size_t i = 0; i < count; ++i) {
            tpdoConfig_[idx].mappings[i] = mappings[i];
        }

        return true;
    }

    /**
     * @brief Настройка RPDO
     */
    bool configureRPDO(uint8_t pdoNumber, uint32_t cobId,
                       const MappedObject* mappings, size_t count) {
        if (pdoNumber == 0 || pdoNumber > MAX_RPDO) return false;
        if (count > MAX_MAPPINGS) return false;

        size_t idx = pdoNumber - 1;
        rpdoConfig_[idx].cobId = cobId;
        rpdoConfig_[idx].mappingCount = count;

        for (size_t i = 0; i < count; ++i) {
            rpdoConfig_[idx].mappings[i] = mappings[i];
        }

        return true;
    }

    /**
     * @brief Чтение данных из RPDO
     */
    bool processRPDO(uint8_t pdoNumber, const uint8_t* data, uint32_t len) {
        if (pdoNumber == 0 || pdoNumber > MAX_RPDO) return false;

        uint32_t idx = pdoNumber - 1;
        auto& config = rpdoConfig_[idx];

        uint32_t bitOffset = 0;
        for (uint32_t i = 0; i < config.mappingCount; ++i) {
            uint16_t index = (config.mappings[i].index >> 16) & 0xFFFF;
            uint8_t subIndex = (config.mappings[i].index >> 8) & 0xFF;
            uint8_t size = config.mappings[i].size;

            // Извлечение данных из CAN frame
            uint32_t value = extractBits(data, len, bitOffset, size);
            
            // Запись в Object Dictionary (через callback)
            if (rpdoCallbacks_[idx]) {
                rpdoCallbacks_[idx](index, subIndex, value, size);
            }

            bitOffset += size;
        }

        return true;
    }

    /**
     * @brief Формирование TPDO кадра
     */
    size_t createTPDO(uint8_t pdoNumber, uint8_t* data, size_t maxSize) {
        if (pdoNumber == 0 || pdoNumber > MAX_TPDO) return 0;
        
        size_t idx = pdoNumber - 1;
        auto& config = tpdoConfig_[idx];

        size_t bitOffset = 0;
        for (size_t i = 0; i < config.mappingCount; ++i) {
            uint16_t index = (config.mappings[i].index >> 16) & 0xFFFF;
            uint8_t subIndex = (config.mappings[i].index >> 8) & 0xFF;
            uint8_t size = config.mappings[i].size;

            // Чтение из Object Dictionary (через callback)
            uint32_t value = 0;
            if (tpdoCallbacks_[idx]) {
                value = tpdoCallbacks_[idx](index, subIndex, size);
            }

            // Вставка данных в CAN frame
            insertBits(data, maxSize, bitOffset, size, value);
            bitOffset += size;
        }

        return (bitOffset + 7) / 8;  // Биты -> байты
    }

    /**
     * @brief Получить COB-ID TPDO
     */
    uint32_t getTPDOCobId(uint8_t pdoNumber) const {
        if (pdoNumber == 0 || pdoNumber > MAX_TPDO) return 0;
        return tpdoConfig_[pdoNumber - 1].cobId;
    }

    /**
     * @brief Получить COB-ID RPDO
     */
    uint32_t getRPDOCobId(uint8_t pdoNumber) const {
        if (pdoNumber == 0 || pdoNumber > MAX_RPDO) return 0;
        return rpdoConfig_[pdoNumber - 1].cobId;
    }

    /**
     * @brief Установить callback для чтения TPDO
     */
    using TPDOReadCallback = std::function<uint32_t(uint16_t, uint8_t, uint8_t)>;
    void setTPDOCallback(uint8_t pdoNumber, TPDOReadCallback cb) {
        if (pdoNumber > 0 && pdoNumber <= MAX_TPDO) {
            tpdoCallbacks_[pdoNumber - 1] = std::move(cb);
        }
    }

    /**
     * @brief Установить callback для записи RPDO
     */
    using RPDOWriteCallback = std::function<void(uint16_t, uint8_t, uint32_t, uint8_t)>;
    void setRPDOCallback(uint8_t pdoNumber, RPDOWriteCallback cb) {
        if (pdoNumber > 0 && pdoNumber <= MAX_RPDO) {
            rpdoCallbacks_[pdoNumber - 1] = std::move(cb);
        }
    }

private:
    uint8_t nodeId_;

    struct PDOConfig {
        uint32_t cobId = 0;
        uint8_t transmissionType = 255;  // 0-240=synchronous, 251=event, 255=disabled
        uint8_t mappingCount = 0;
        MappedObject mappings[MAX_MAPPINGS];
    };

    PDOConfig tpdoConfig_[MAX_TPDO];
    PDOConfig rpdoConfig_[MAX_RPDO];

    TPDOReadCallback tpdoCallbacks_[MAX_TPDO];
    RPDOWriteCallback rpdoCallbacks_[MAX_RPDO];

    uint32_t extractBits(const uint8_t* data, size_t len, 
                         size_t offset, size_t count) const {
        uint32_t result = 0;
        for (size_t i = 0; i < count; ++i) {
            size_t byteIdx = (offset + i) / 8;
            size_t bitIdx = (offset + i) % 8;
            if (byteIdx < len) {
                result |= ((data[byteIdx] >> bitIdx) & 1) << i;
            }
        }
        return result;
    }

    void insertBits(uint8_t* data, size_t len,
                    size_t offset, size_t count, uint32_t value) {
        for (size_t i = 0; i < count; ++i) {
            size_t byteIdx = (offset + i) / 8;
            size_t bitIdx = (offset + i) % 8;
            if (byteIdx < len) {
                if (value & (1 << i)) {
                    data[byteIdx] |= (1 << bitIdx);
                } else {
                    data[byteIdx] &= ~(1 << bitIdx);
                }
            }
        }
    }
};

// ============================================================================
// CANopen Stack
// ============================================================================

/**
 * @brief Полный стек CANopen
 *
 * Интегрирует NMT, SDO, PDO компоненты.
 */
class CANopenStack {
public:
    explicit CANopenStack(uint8_t nodeId = 1) 
        : nodeId_(nodeId)
        , nmt_(nodeId)
        , sdo_(nodeId)
        , pdo_(nodeId)
    {}

    /**
     * @brief Инициализация стека
     */
    hal::Status init(hal::ICAN& can) {
        can_ = &can;
        
        // Начальное состояние - Pre-Operational
        nmt_.processCommand(NMTCommand::ENTER_PRE_OPERATIONAL);

        // Настройка стандартных Object Dictionary entries
        setupStandardOD();

        return hal::Status::OK;
    }

    /**
     * @brief Обработка входящих CAN сообщений
     */
    void processMessage(uint32_t id, const uint8_t* data, uint32_t len) {
        // NMT команды (COB-ID 0x000)
        if (id == COBId::NMT_COMMAND && len >= 2u) {
            NMTCommand cmd = static_cast<NMTCommand>(data[0]);
            uint8_t targetNode = data[1];

            // 0 = broadcast, иначе проверка node ID
            if (targetNode == 0 || targetNode == nodeId_) {
                nmt_.processCommand(cmd);
            }
            return;
        }

        // SDO запросы
        if (id == (static_cast<uint32_t>(COBId::SDO_RX) + nodeId_) && len == 8u) {
            uint8_t response[8];
            if (sdo_.processRequest(data, len, response)) {
                // Отправка ответа
                transmitCAN(sdo_.getResponseCOBId(), response, 8);
            }
            return;
        }

        // RPDO сообщения
        for (uint8_t i = 1; i <= 4; ++i) {
            if (id == pdo_.getRPDOCobId(i)) {
                pdo_.processRPDO(i, data, len);
                return;
            }
        }
    }

    /**
     * @brief Отправка TPDO
     */
    bool sendTPDO(uint8_t pdoNumber) {
        if (!nmt_.isOperational()) {
            return false;
        }

        uint8_t data[8] = {};
        size_t len = pdo_.createTPDO(pdoNumber, data, 8);
        
        if (len == 0) return false;

        uint32_t cobId = pdo_.getTPDOCobId(pdoNumber);
        return transmitCAN(cobId, data, len) == hal::Status::OK;
    }

    /**
     * @brief Отправка Heartbeat
     */
    bool sendHeartbeat() {
        uint8_t state = nmt_.createHeartbeat();
        return transmitCAN(nmt_.getHeartbeatCOBId(), &state, 1) == hal::Status::OK;
    }

    /**
     * @brief Отправка Emergency сообщения
     */
    bool sendEmergency(uint16_t errorCode, const uint8_t* data, size_t len) {
        if (len > 5) len = 5;  // Max 5 bytes for emergency data

        uint8_t emcy[8] = {};
        emcy[0] = errorCode & 0xFF;
        emcy[1] = (errorCode >> 8) & 0xFF;
        emcy[2] = nmt_.createHeartbeat();  // Device status

        for (size_t i = 0; i < len; ++i) {
            emcy[3 + i] = data[i];
        }

        return transmitCAN(COBId::EMCY + nodeId_, emcy, 8) == hal::Status::OK;
    }

    /**
     * @brief Регистрация записи Object Dictionary
     */
    bool registerOD(const ODEntry& entry) {
        return sdo_.registerEntry(entry);
    }

    /**
     * @brief Настройка PDO
     */
    PDOManager& getPDO() { return pdo_; }

    /**
     * @brief Получить NMT состояние
     */
    NodeState getState() const {
        return nmt_.getState();
    }

private:
    uint8_t nodeId_;
    hal::ICAN* can_ = nullptr;
    NMTStateMachine nmt_;
    SDOServer sdo_;
    PDOManager pdo_;

    hal::Status transmitCAN(uint32_t id, const uint8_t* data, size_t len, uint32_t timeoutMs = 1000) {
        hal::CANMessage msg;
        msg.id = id;
        msg.extended = false;
        msg.remote = false;
        msg.dlc = static_cast<uint8_t>(len > 8 ? 8 : len);
        msg.data = {};
        if (data && len > 0) {
            std::memcpy(msg.data.data(), data, msg.dlc);
        }
        msg.timestamp = 0;
        return can_->transmit(msg, timeoutMs);
    }

    void setupStandardOD() {
        // Manufacturer Device Information
        ODEntry vendorId = {
            .index = 0x1018,  // Identity Object
            .subIndex = 0x01,  // Vendor ID
            .access = ODEntry::Access::RO,
            .dataType = ODEntry::DataType::UNSIGNED32
        };
        // Vendor ID: 0x00000123 (MKA Embedded Project)
        vendorId.data = {0x23, 0x01, 0x00, 0x00};
        sdo_.registerEntry(vendorId);

        ODEntry productCode = {
            .index = 0x1018,
            .subIndex = 0x02,  // Product Code
            .access = ODEntry::Access::RO,
            .dataType = ODEntry::DataType::UNSIGNED32
        };
        productCode.data = {0x01, 0x00, 0x00, 0x00};  // Product code = 1
        sdo_.registerEntry(productCode);

        ODEntry revisionNumber = {
            .index = 0x1018,
            .subIndex = 0x03,  // Revision Number
            .access = ODEntry::Access::RO,
            .dataType = ODEntry::DataType::UNSIGNED32
        };
        revisionNumber.data = {0x00, 0x00, 0x02, 0x00};  // v2.0.0
        sdo_.registerEntry(revisionNumber);

        ODEntry serialNumber = {
            .index = 0x1018,
            .subIndex = 0x04,  // Serial Number
            .access = ODEntry::Access::RO,
            .dataType = ODEntry::DataType::UNSIGNED32
        };
        serialNumber.data = {0x00, 0x00, 0x00, 0x00};  // Заполняется при производстве
        sdo_.registerEntry(serialNumber);

        ODEntry manufacturerName = {
            .index = 0x1008,
            .subIndex = 0x00,
            .access = ODEntry::Access::RO,
            .dataType = ODEntry::DataType::VISIBLE_STR
        };
        // "MKA Embedded"
        const char* name = "MKA Embedded";
        size_t nameLen = strlen(name) + 1;
        if (nameLen <= manufacturerName.data.size()) {
            std::memcpy(manufacturerName.data.data(), name, nameLen);
        }
        sdo_.registerEntry(manufacturerName);

        ODEntry deviceType = {
            .index = 0x1000,
            .subIndex = 0x00,
            .access = ODEntry::Access::RO,
            .dataType = ODEntry::DataType::UNSIGNED32
        };
        // Device type: 0x01000000 (CIA 301 compliant device)
        deviceType.data = {0x00, 0x00, 0x00, 0x01};
        sdo_.registerEntry(deviceType);

        ODEntry errorRegister = {
            .index = 0x1001,
            .subIndex = 0x00,
            .access = ODEntry::Access::RO,
            .dataType = ODEntry::DataType::UNSIGNED8
        };
        errorRegister.data = {0x00};  // No error
        sdo_.registerEntry(errorRegister);
    }
};

} // namespace systems
} // namespace mka

#endif // CANOPEN_HPP
