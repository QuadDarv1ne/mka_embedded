/**
 * @file command_handler.hpp
 * @brief Command Handler System for MKA
 * 
 * Система обработки команд от НКУ (наземного комплекса управления).
 * Поддерживает:
 * - Регистрацию обработчиков команд
 * - Валидацию параметров
 * - Очередь команд
 * - Ответы с кодами результата
 */

#ifndef COMMAND_HANDLER_HPP
#define COMMAND_HANDLER_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>
#include <optional>

#include "utils/callback.hpp"
#include "utils/span.hpp"

namespace mka {
namespace cmd {

// ============================================================================
// Типы и константы
// ============================================================================

/// Максимальный размер команды
constexpr size_t MAX_COMMAND_SIZE = 256;

/// Максимальный размер ответа
constexpr size_t MAX_RESPONSE_SIZE = 256;

/// Максимальное количество команд в очереди
constexpr size_t COMMAND_QUEUE_SIZE = 16;

/// Код результата выполнения команды
enum class ResultCode : uint8_t {
    OK = 0,                    // Успешно
    EXECUTED = 1,              // Выполнено
    QUEUED = 2,                // Поставлено в очередь
    BUSY = 3,                  // Занято, попробуйте позже
    
    // Ошибки
    UNKNOWN_COMMAND = 0x80,    // Неизвестная команда
    INVALID_PARAMS = 0x81,     // Неверные параметры
    INVALID_LENGTH = 0x82,     // Неверная длина
    CHECKSUM_ERROR = 0x83,     // Ошибка контрольной суммы
    NOT_AUTHORIZED = 0x84,     // Нет прав
    TIMEOUT = 0x85,            // Таймаут
    EXECUTION_ERROR = 0x86,    // Ошибка выполнения
    QUEUE_FULL = 0x87,         // Очередь полна
    HARDWARE_ERROR = 0x88,     // Аппаратная ошибка
};

/// Приоритет команды
enum class CommandPriority : uint8_t {
    LOW = 0,
    NORMAL = 1,
    HIGH = 2,
    CRITICAL = 3
};

// ============================================================================
// Структуры данных
// ============================================================================

/**
 * @brief Заголовок команды
 */
struct CommandHeader {
    uint16_t commandId;        // Идентификатор команды
    uint16_t sequenceNumber;   // Порядковый номер
    uint8_t priority;          // Приоритет
    uint8_t flags;             // Флаги
    uint16_t dataLength;       // Длина данных
};

/**
 * @brief Полная команда
 */
struct Command {
    CommandHeader header;
    std::array<uint8_t, 240> data{};  // MAX_COMMAND_SIZE - sizeof(CommandHeader) = 256 - 16 = 240
    
    // Метаданные
    uint32_t timestamp;        // Время получения
    uint8_t source;            // Источник команды
    bool valid;                // Валидность
};

/**
 * @brief Ответ на команду
 */
struct CommandResponse {
    uint16_t commandId;
    uint16_t sequenceNumber;
    ResultCode resultCode;
    uint16_t responseLength;
    std::array<uint8_t, MAX_RESPONSE_SIZE> data{};
    
    // Время выполнения
    uint32_t executionTimeMs;
};

// ============================================================================
// Идентификаторы команд
// ============================================================================

namespace commands {

// Системные команды (0x00xx)
constexpr uint16_t NOOP                = 0x0000;  // No operation
constexpr uint16_t RESET               = 0x0001;  // Сброс системы
constexpr uint16_t GET_STATUS          = 0x0002;  // Получить статус
constexpr uint16_t GET_TIME            = 0x0003;  // Получить время
constexpr uint16_t SET_TIME            = 0x0004;  // Установить время
constexpr uint16_t GET_VERSION         = 0x0005;  // Получить версию ПО
constexpr uint16_t GET_MEMORY          = 0x0006;  // Статистика памяти

// Команды управления режимами (0x01xx)
constexpr uint16_t SET_MODE            = 0x0100;  // Установить режим
constexpr uint16_t GET_MODE            = 0x0101;  // Получить режим
constexpr uint16_t ENTER_SAFE_MODE     = 0x0102;  // Безопасный режим

// Команды подсистем (0x02xx - EPS)
constexpr uint16_t EPS_GET_STATUS      = 0x0200;  // Статус EPS
constexpr uint16_t EPS_SET_POWER_RAIL  = 0x0201;  // Управление питанием
constexpr uint16_t EPS_GET_BATTERY     = 0x0202;  // Статус батареи

// Команды подсистем (0x03xx - ADCS)
constexpr uint16_t ADCS_GET_STATUS     = 0x0300;  // Статус ADCS
constexpr uint16_t ADCS_SET_MODE       = 0x0301;  // Режим ADCS
constexpr uint16_t ADCS_GET_ATTITUDE   = 0x0302;  // Ориентация
constexpr uint16_t ADCS_SET_TARGET     = 0x0303;  // Целевая ориентация

// Команды подсистем (0x04xx - COMM)
constexpr uint16_t COMM_GET_STATUS     = 0x0400;  // Статус COMM
constexpr uint16_t COMM_SET_TX_POWER   = 0x0401;  // Мощность передатчика
constexpr uint16_t COMM_GET_STATS      = 0x0402;  // Статистика связи

// Команды телеметрии (0x05xx)
constexpr uint16_t TM_GET_HK           = 0x0500;  // Housekeeping
constexpr uint16_t TM_SET_RATE         = 0x0501;  // Частота телеметрии
constexpr uint16_t TM_REQUEST_DUMP     = 0x0502;  // Запрос дампа

// Команды полезной нагрузки (0x06xx)
constexpr uint16_t PAYLOAD_ON          = 0x0600;  // Включить payload
constexpr uint16_t PAYLOAD_OFF         = 0x0601;  // Выключить payload
constexpr uint16_t PAYLOAD_GET_DATA    = 0x0602;  // Получить данные

// Команды файловой системы (0x07xx)
constexpr uint16_t FS_LIST             = 0x0700;  // Список файлов
constexpr uint16_t FS_READ             = 0x0701;  // Чтение файла
constexpr uint16_t FS_WRITE            = 0x0702;  // Запись файла
constexpr uint16_t FS_DELETE           = 0x0703;  // Удаление файла

// OTA команды (0x08xx)
constexpr uint16_t OTA_START           = 0x0800;  // Начало обновления
constexpr uint16_t OTA_DATA            = 0x0801;  // Данные обновления
constexpr uint16_t OTA_COMMIT          = 0x0802;  // Применить обновление
constexpr uint16_t OTA_ABORT           = 0x0803;  // Отменить обновление

} // namespace commands

// ============================================================================
// Обработчик команды
// ============================================================================

/**
 * @brief Функция обработки команды
 * @param cmd Входная команда
 * @param response Ответ для заполнения
 * @return Код результата
 */
using CommandHandler = Callback<ResultCode(const Command& cmd, CommandResponse& response)>;

/**
 * @brief Валидатор параметров команды
 */
using ParamValidator = Callback<bool(const uint8_t* params, size_t size)>;

// ============================================================================
// Регистр команд
// ============================================================================

struct CommandRegistration {
    uint16_t commandId;
    const char* name;
    CommandHandler handler;
    ParamValidator validator;
    uint32_t timeoutMs;
    bool requiresAuth;
    uint8_t minParams;
    uint8_t maxParams;
};

// ============================================================================
// Менеджер команд
// ============================================================================

/**
 * @brief Центральный менеджер обработки команд
 */
class CommandManager {
public:
    static constexpr size_t MAX_COMMANDS = 128;
    
    CommandManager() = default;
    
    /**
     * @brief Регистрация обработчика команды
     */
    bool registerCommand(const CommandRegistration& reg) {
        if (commandCount_ >= MAX_COMMANDS) return false;
        
        // Проверка дубликата
        for (size_t i = 0; i < commandCount_; i++) {
            if (registrations_[i].commandId == reg.commandId) {
                registrations_[i] = reg;  // Обновление
                return true;
            }
        }
        
        registrations_[commandCount_++] = reg;
        return true;
    }
    
    /**
     * @brief Обработка входящей команды
     */
    CommandResponse processCommand(const Command& cmd) {
        CommandResponse response{};
        response.commandId = cmd.header.commandId;
        response.sequenceNumber = cmd.header.sequenceNumber;
        response.resultCode = ResultCode::UNKNOWN_COMMAND;
        response.responseLength = 0;
        response.executionTimeMs = 0;

        // Поиск обработчика
        const CommandRegistration* reg = findRegistration(cmd.header.commandId);

        if (!reg) {
            return response;
        }

        // Валидация длины параметров
        if (cmd.header.dataLength > cmd.data.size()) {
            response.resultCode = ResultCode::INVALID_LENGTH;
            return response;
        }

        if (cmd.header.dataLength < reg->minParams ||
            cmd.header.dataLength > reg->maxParams) {
            response.resultCode = ResultCode::INVALID_LENGTH;
            return response;
        }

        // Валидация параметров
        if (reg->validator) {
            if (!reg->validator(cmd.data.data(), cmd.header.dataLength)) {
                response.resultCode = ResultCode::INVALID_PARAMS;
                return response;
            }
        }

        // Выполнение команды
        uint32_t startTime = getTimestampMs ? getTimestampMs() : 0;

        if (reg->handler) {
            response.resultCode = reg->handler(cmd, response);
        } else {
            response.resultCode = ResultCode::EXECUTION_ERROR;
        }

        uint32_t endTime = getTimestampMs ? getTimestampMs() : 0;
        response.executionTimeMs = endTime - startTime;

        return response;
    }
    
    /**
     * @brief Проверка наличия команды
     */
    bool hasCommand(uint16_t commandId) const {
        return findRegistration(commandId) != nullptr;
    }

    /**
     * @brief unregister обработчика команды
     */
    bool unregisterCommand(uint16_t commandId) {
        for (size_t i = 0; i < commandCount_; i++) {
            if (registrations_[i].commandId == commandId) {
                // Сдвиг массива
                for (size_t j = i; j < commandCount_ - 1; j++) {
                    registrations_[j] = registrations_[j + 1];
                }
                commandCount_--;
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Получение информации о команде
     */
    const char* getCommandName(uint16_t commandId) const {
        const CommandRegistration* reg = findRegistration(commandId);
        return reg ? reg->name : "UNKNOWN";
    }

    /**
     * @brief Количество зарегистрированных команд
     */
    size_t getCommandCount() const { return commandCount_; }

    /**
     * @brief Сброс всех команд
     */
    void clear() { commandCount_ = 0; }
    
    /**
     * @brief Установка источника времени
     */
    void setTimestampSource(Callback<uint32_t()> source) {
        getTimestampMs = source;
    }

private:
    std::array<CommandRegistration, MAX_COMMANDS> registrations_{};
    size_t commandCount_ = 0;

    Callback<uint32_t()> getTimestampMs;
    
    const CommandRegistration* findRegistration(uint16_t commandId) const {
        for (size_t i = 0; i < commandCount_; i++) {
            if (registrations_[i].commandId == commandId) {
                return &registrations_[i];
            }
        }
        return nullptr;
    }
};

// ============================================================================
// Стандартные обработчики
// ============================================================================

/**
 * @brief Создание обработчика NOOP
 */
inline CommandHandler createNoopHandler() {
    return +[](const Command&, CommandResponse& resp) -> ResultCode {
        resp.data[0] = 'O';
        resp.data[1] = 'K';
        resp.responseLength = 2;
        return ResultCode::OK;
    };
}

/**
 * @brief Создание обработчика GET_VERSION
 */
inline CommandHandler createGetVersionHandler(uint8_t major, uint8_t minor, uint8_t patch) {
    // Используем глобальное хранилище для версий
    // Это ограничение Callback без захвата
    static uint8_t s_major = 0, s_minor = 0, s_patch = 0;
    s_major = major;
    s_minor = minor;
    s_patch = patch;
    
    return +[](const Command&, CommandResponse& resp) -> ResultCode {
        resp.data[0] = s_major;
        resp.data[1] = s_minor;
        resp.data[2] = s_patch;
        resp.responseLength = 3;
        return ResultCode::OK;
    };
}

/**
 * @brief Парсер команды из байтов
 */
inline std::optional<Command> parseCommand(const uint8_t* data, size_t dataSize) {
    if (!data || dataSize < sizeof(CommandHeader)) {
        return std::nullopt;
    }

    Command cmd{};
    std::memcpy(&cmd.header, data, sizeof(CommandHeader));

    // Проверка длины
    if (dataSize < sizeof(CommandHeader) + cmd.header.dataLength) {
        return std::nullopt;
    }

    // Копирование данных с проверкой границ
    size_t dataLen = std::min<size_t>(cmd.header.dataLength, cmd.data.size());
    std::memcpy(cmd.data.data(), data + sizeof(CommandHeader), dataLen);

    cmd.valid = true;
    return cmd;
}

/**
 * @brief Сериализация ответа в байты
 */
inline size_t serializeResponse(const CommandResponse& resp, uint8_t* buffer, size_t bufferSize) {
    if (!buffer) return 0;

    size_t totalSize = 7 + resp.responseLength;

    if (bufferSize < totalSize) {
        return 0;
    }

    size_t offset = 0;

    // Command ID
    buffer[offset++] = resp.commandId >> 8;
    buffer[offset++] = resp.commandId & 0xFF;

    // Sequence number
    buffer[offset++] = resp.sequenceNumber >> 8;
    buffer[offset++] = resp.sequenceNumber & 0xFF;

    // Result code
    buffer[offset++] = static_cast<uint8_t>(resp.resultCode);

    // Response length
    buffer[offset++] = resp.responseLength >> 8;
    buffer[offset++] = resp.responseLength & 0xFF;

    // Data
    std::memcpy(buffer + offset, resp.data.data(), resp.responseLength);
    offset += resp.responseLength;

    return offset;
}

} // namespace cmd
} // namespace mka

#endif // COMMAND_HANDLER_HPP
