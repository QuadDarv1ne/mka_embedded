/**
 * @file param_store.hpp
 * @brief Parameter Store for MKA
 * 
 * Система хранения параметров конфигурации с поддержкой:
 * - Сохранения во Flash/FRAM
 * - Горячей замены значений
 * - Валидации при записи
 * - Контрольных сумм
 */

#ifndef PARAM_STORE_HPP
#define PARAM_STORE_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <functional>
#include <array>
#include <span>
#include <optional>

namespace mka {
namespace param {

// ============================================================================
// Типы параметров
// ============================================================================

enum class ParamType : uint8_t {
    UINT8,
    INT8,
    UINT16,
    INT16,
    UINT32,
    INT32,
    UINT64,
    INT64,
    FLOAT,
    DOUBLE,
    BOOL,
    STRING,
    BLOB
};

// ============================================================================
// Атрибуты параметра
// ============================================================================

struct ParamAttributes {
    bool readable : 1;
    bool writable : 1;
    bool persistent : 1;
    bool requiresReset : 1;     // Требует сброса для применения
    bool readOnlyAfterInit : 1; // Только чтение после инициализации
    uint8_t reserved : 3;
};

// ============================================================================
// Идентификаторы параметров
// ============================================================================

namespace ids {

// Системные параметры (0x0000-0x00FF)
constexpr uint16_t SYSTEM_VERSION        = 0x0000;
constexpr uint16_t SYSTEM_UPTIME         = 0x0001;
constexpr uint16_t SYSTEM_RESET_COUNT    = 0x0002;
constexpr uint16_t SYSTEM_LAST_RESET     = 0x0003;

// Параметры связи (0x0100-0x01FF)
constexpr uint16_t COMM_BAUD_RATE        = 0x0100;
constexpr uint16_t COMM_TX_POWER         = 0x0101;
constexpr uint16_t COMM_TX_INTERVAL      = 0x0102;
constexpr uint16_t COMM_FREQUENCY        = 0x0103;
constexpr uint16_t COMM_CALLSIGN         = 0x0104;

// Параметры ADCS (0x0200-0x02FF)
constexpr uint16_t ADCS_MODE             = 0x0200;
constexpr uint16_t ADCS_UPDATE_RATE      = 0x0201;
constexpr uint16_t ADCS_P_GAIN           = 0x0202;
constexpr uint16_t ADCS_I_GAIN           = 0x0203;
constexpr uint16_t ADCS_D_GAIN           = 0x0204;
constexpr uint16_t ADCS_MAX_RATE         = 0x0205;

// Параметры EPS (0x0300-0x03FF)
constexpr uint16_t EPS_BATTERY_LOW       = 0x0300;
constexpr uint16_t EPS_BATTERY_CRITICAL  = 0x0301;
constexpr uint16_t EPS_SAFE_MODE_THRESH  = 0x0302;

// Параметры миссии (0x0400-0x04FF)
constexpr uint16_t MISSION_MODE          = 0x0400;
constexpr uint16_t MISSION_PHASE         = 0x0401;
constexpr uint16_t MISSION_TARGET_LAT    = 0x0402;
constexpr uint16_t MISSION_TARGET_LON    = 0x0403;

} // namespace ids

// ============================================================================
// Значение параметра
// ============================================================================

union ParamValue {
    uint8_t u8;
    int8_t i8;
    uint16_t u16;
    int16_t i16;
    uint32_t u32;
    int32_t i32;
    uint64_t u64;
    int64_t i64;
    float f;
    double d;
    bool b;
    uint8_t bytes[8];
};

// ============================================================================
// Запись параметра
// ============================================================================

struct ParamEntry {
    uint16_t id;
    ParamType type;
    ParamAttributes attributes;
    uint8_t size;               // Размер значения в байтах
    ParamValue defaultValue;
    ParamValue minValue;
    ParamValue maxValue;
    const char* name;
    const char* description;
    const char* unit;
};

// ============================================================================
// Хранилище параметров
// ============================================================================

/**
 * @brief Менеджер параметров конфигурации
 */
class ParameterStore {
public:
    static constexpr size_t MAX_PARAMS = 128;
    
    using ChangeCallback = std::function<void(uint16_t, ParamValue)>;
    
    ParameterStore() = default;
    
    // ========================================================================
    // Регистрация параметров
    // ========================================================================
    
    /**
     * @brief Регистрация параметра
     */
    bool registerParam(const ParamEntry& entry) {
        if (paramCount_ >= MAX_PARAMS) return false;
        
        // Проверка дубликата
        for (size_t i = 0; i < paramCount_; i++) {
            if (entries_[i].id == entry.id) return false;
        }
        
        entries_[paramCount_] = entry;
        values_[paramCount_] = entry.defaultValue;
        modified_[paramCount_] = false;
        paramCount_++;
        
        return true;
    }
    
    // ========================================================================
    // Доступ к параметрам
    // ========================================================================
    
    /**
     * @brief Получение значения параметра
     */
    std::optional<ParamValue> get(uint16_t id) const {
        for (size_t i = 0; i < paramCount_; i++) {
            if (entries_[i].id == id) {
                return values_[i];
            }
        }
        return std::nullopt;
    }
    
    /**
     * @brief Установка значения параметра
     */
    bool set(uint16_t id, ParamValue value) {
        for (size_t i = 0; i < paramCount_; i++) {
            if (entries_[i].id == id) {
                if (!entries_[i].attributes.writable) {
                    return false;
                }
                
                // Валидация диапазона
                if (!validateRange(entries_[i], value)) {
                    return false;
                }
                
                values_[i] = value;
                modified_[i] = true;
                
                // Уведомление
                if (changeCallback_) {
                    changeCallback_(id, value);
                }
                
                return true;
            }
        }
        return false;
    }
    
    /**
     * @brief Сброс к значению по умолчанию
     */
    bool reset(uint16_t id) {
        for (size_t i = 0; i < paramCount_; i++) {
            if (entries_[i].id == id) {
                values_[i] = entries_[i].defaultValue;
                modified_[i] = false;
                return true;
            }
        }
        return false;
    }
    
    /**
     * @brief Сброс всех параметров к значениям по умолчанию
     */
    void resetAll() {
        for (size_t i = 0; i < paramCount_; i++) {
            values_[i] = entries_[i].defaultValue;
            modified_[i] = false;
        }
    }
    
    // ========================================================================
    // Информация о параметрах
    // ========================================================================
    
    /**
     * @brief Получение описания параметра
     */
    const ParamEntry* getEntry(uint16_t id) const {
        for (size_t i = 0; i < paramCount_; i++) {
            if (entries_[i].id == id) {
                return &entries_[i];
            }
        }
        return nullptr;
    }
    
    /**
     * @brief Проверка существования параметра
     */
    bool exists(uint16_t id) const {
        for (size_t i = 0; i < paramCount_; i++) {
            if (entries_[i].id == id) return true;
        }
        return false;
    }
    
    /**
     * @brief Количество зарегистрированных параметров
     */
    size_t count() const { return paramCount_; }
    
    /**
     * @brief Получение списка ID всех параметров
     */
    size_t getParamIds(std::span<uint16_t> ids) const {
        size_t count = std::min(paramCount_, ids.size());
        for (size_t i = 0; i < count; i++) {
            ids[i] = entries_[i].id;
        }
        return count;
    }
    
    // ========================================================================
    // Персистентность
    // ========================================================================
    
    /**
     * @brief Проверка наличия несохранённых изменений
     */
    bool hasUnsavedChanges() const {
        for (size_t i = 0; i < paramCount_; i++) {
            if (modified_[i] && entries_[i].attributes.persistent) {
                return true;
            }
        }
        return false;
    }
    
    /**
     * @brief Сериализация параметров для сохранения
     */
    size_t serialize(std::span<uint8_t> buffer) const {
        size_t offset = 0;
        
        // Заголовок
        if (offset + 4 > buffer.size()) return 0;
        buffer[offset++] = 'P';
        buffer[offset++] = 'A';
        buffer[offset++] = 'R';
        buffer[offset++] = 0x01;  // Version
        
        // Количество параметров
        if (offset + 2 > buffer.size()) return 0;
        buffer[offset++] = (paramCount_ >> 8) & 0xFF;
        buffer[offset++] = paramCount_ & 0xFF;
        
        // Параметры
        for (size_t i = 0; i < paramCount_; i++) {
            if (entries_[i].attributes.persistent) {
                if (offset + 2 + entries_[i].size > buffer.size()) {
                    return 0;
                }
                
                // ID
                buffer[offset++] = (entries_[i].id >> 8) & 0xFF;
                buffer[offset++] = entries_[i].id & 0xFF;
                
                // Значение
                std::memcpy(&buffer[offset], &values_[i], entries_[i].size);
                offset += entries_[i].size;
            }
        }
        
        // CRC32 (placeholder)
        if (offset + 4 > buffer.size()) return 0;
        buffer[offset++] = 0;
        buffer[offset++] = 0;
        buffer[offset++] = 0;
        buffer[offset++] = 0;
        
        return offset;
    }
    
    /**
     * @brief Десериализация параметров
     */
    bool deserialize(std::span<const uint8_t> buffer) {
        size_t offset = 0;
        
        // Проверка заголовка
        if (buffer.size() < 6) return false;
        if (buffer[0] != 'P' || buffer[1] != 'A' || buffer[2] != 'R') {
            return false;
        }
        
        offset = 4;
        
        // Количество параметров
        uint16_t count = (buffer[offset] << 8) | buffer[offset + 1];
        offset += 2;
        
        // Загрузка значений
        for (uint16_t i = 0; i < count && offset + 2 <= buffer.size(); i++) {
            uint16_t id = (buffer[offset] << 8) | buffer[offset + 1];
            offset += 2;
            
            // Поиск параметра
            for (size_t j = 0; j < paramCount_; j++) {
                if (entries_[j].id == id) {
                    if (offset + entries_[j].size <= buffer.size()) {
                        std::memcpy(&values_[j], &buffer[offset], entries_[j].size);
                        offset += entries_[j].size;
                        modified_[j] = false;
                    }
                    break;
                }
            }
        }
        
        return true;
    }
    
    /**
     * @brief Установка callback для отслеживания изменений
     */
    void setChangeCallback(ChangeCallback callback) {
        changeCallback_ = callback;
    }
    
private:
    std::array<ParamEntry, MAX_PARAMS> entries_{};
    std::array<ParamValue, MAX_PARAMS> values_{};
    std::array<bool, MAX_PARAMS> modified_{};
    size_t paramCount_ = 0;
    
    ChangeCallback changeCallback_;
    
    bool validateRange(const ParamEntry& entry, ParamValue value) const {
        switch (entry.type) {
            case ParamType::UINT8:
                return value.u8 >= entry.minValue.u8 && value.u8 <= entry.maxValue.u8;
            case ParamType::INT8:
                return value.i8 >= entry.minValue.i8 && value.i8 <= entry.maxValue.i8;
            case ParamType::UINT16:
                return value.u16 >= entry.minValue.u16 && value.u16 <= entry.maxValue.u16;
            case ParamType::INT16:
                return value.i16 >= entry.minValue.i16 && value.i16 <= entry.maxValue.i16;
            case ParamType::UINT32:
                return value.u32 >= entry.minValue.u32 && value.u32 <= entry.maxValue.u32;
            case ParamType::INT32:
                return value.i32 >= entry.minValue.i32 && value.i32 <= entry.maxValue.i32;
            case ParamType::FLOAT:
                return value.f >= entry.minValue.f && value.f <= entry.maxValue.f;
            default:
                return true;
        }
    }
};

// ============================================================================
// Макросы для упрощения регистрации параметров
// ============================================================================

#define PARAM_UINT8(id, name, def, min, max, desc, unit) \
    mka::param::ParamEntry { \
        .id = id, .type = mka::param::ParamType::UINT8, \
        .attributes = {true, true, true, false, false, 0}, \
        .size = 1, .defaultValue = {.u8 = def}, \
        .minValue = {.u8 = min}, .maxValue = {.u8 = max}, \
        .name = name, .description = desc, .unit = unit \
    }

#define PARAM_FLOAT(id, name, def, min, max, desc, unit) \
    mka::param::ParamEntry { \
        .id = id, .type = mka::param::ParamType::FLOAT, \
        .attributes = {true, true, true, false, false, 0}, \
        .size = 4, .defaultValue = {.f = def}, \
        .minValue = {.f = min}, .maxValue = {.f = max}, \
        .name = name, .description = desc, .unit = unit \
    }

#define PARAM_READONLY(id, name, def, desc, unit) \
    mka::param::ParamEntry { \
        .id = id, .type = mka::param::ParamType::UINT32, \
        .attributes = {true, false, false, false, false, 0}, \
        .size = 4, .defaultValue = {.u32 = def}, \
        .minValue = {.u32 = 0}, .maxValue = {.u32 = 0}, \
        .name = name, .description = desc, .unit = unit \
    }

} // namespace param
} // namespace mka

#endif // PARAM_STORE_HPP
