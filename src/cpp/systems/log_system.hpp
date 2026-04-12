/**
 * @file log_system.hpp
 * @brief Structured Logging System for MKA
 * 
 * Система структурированного логирования для бортового ПО.
 * Поддерживает различные уровни важности, категории и вывод
 * в различные назначения (UART, файл, буфер).
 */

#ifndef LOG_SYSTEM_HPP
#define LOG_SYSTEM_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <array>
#include <mutex>

#include "utils/callback.hpp"
#include "utils/span.hpp"

namespace mka {
namespace log {

// ============================================================================
// Уровни логирования
// ============================================================================

enum class LogLevel : uint8_t {
    DEBUG = 0,
    INFO = 1,
    NOTICE = 2,
    WARNING = 3,
    ERROR = 4,
    CRITICAL = 5,
    OFF = 255
};

// ============================================================================
// Категории
// ============================================================================

enum class LogCategory : uint8_t {
    GENERAL = 0,
    OBC = 1,
    EPS = 2,
    ADCS = 3,
    COMM = 4,
    PAYLOAD = 5,
    TCS = 6,
    FDIR = 7,
    MISSION = 8
};

// ============================================================================
// Структура записи лога
// ============================================================================

struct LogEntry {
    uint32_t timestamp;         // Время (секунды от запуска)
    uint16_t milliseconds;      // Миллисекунды
    LogLevel level;             // Уровень
    LogCategory category;       // Категория
    uint16_t line;              // Номер строки
    const char* file;           // Файл
    const char* function;       // Функция
    char message[128];          // Сообщение
    uint8_t args[16];           // Бинарные аргументы (уменьшено)
    uint8_t argsLen;            // Длина аргументов
};

static_assert(sizeof(LogEntry) <= 192, "LogEntry too large");

// ============================================================================
// Выходной интерфейс
// ============================================================================

class ILogOutput {
public:
    virtual ~ILogOutput() = default;
    virtual void write(const LogEntry& entry, const char* formatted) = 0;
    virtual void flush() {}
};

// ============================================================================
// Конфигурация логгера
// ============================================================================

struct LoggerConfig {
    LogLevel minLevel = LogLevel::INFO;
    bool includeTimestamp = true;
    bool includeLocation = false;
    bool includeFunction = false;
    bool colorOutput = false;
    uint16_t bufferSize = 1024;
};

// ============================================================================
// Форматтер
// ============================================================================

class LogFormatter {
public:
    static void format(const LogEntry& entry, char* buffer, size_t size, bool color = false) {
        char* p = buffer;
        char* end = buffer + size - 1;
        
        // Уровень
        const char* levelStr = levelToString(entry.level);
        if (color) {
            const char* colorCode = levelColorCode(entry.level);
            p += snprintf(p, end - p, "%s", colorCode);
        }
        
        // Время
        if (entry.timestamp != 0 || entry.milliseconds != 0) {
            p += snprintf(p, end - p, "[%05u.%03u] ", 
                         entry.timestamp, entry.milliseconds);
        }
        
        // Уровень
        p += snprintf(p, end - p, "[%-8s] ", levelStr);
        
        // Категория
        p += snprintf(p, end - p, "[%s] ", categoryToString(entry.category));
        
        // Местоположение (опционально)
        if (entry.file && entry.line != 0) {
            p += snprintf(p, end - p, "(%s:%u) ", 
                         shortFileName(entry.file), entry.line);
        }
        
        // Сообщение
        p += snprintf(p, end - p, "%s", entry.message);
        
        // Reset color
        if (color) {
            p += snprintf(p, end - p, "\033[0m");
        }
        
        *p = '\0';
    }
    
    static const char* levelToString(LogLevel level) {
        switch (level) {
            case LogLevel::DEBUG:    return "DEBUG";
            case LogLevel::INFO:     return "INFO";
            case LogLevel::NOTICE:   return "NOTICE";
            case LogLevel::WARNING:  return "WARNING";
            case LogLevel::ERROR:    return "ERROR";
            case LogLevel::CRITICAL: return "CRITICAL";
            default: return "UNKNOWN";
        }
    }
    
    static const char* categoryToString(LogCategory cat) {
        switch (cat) {
            case LogCategory::GENERAL: return "GEN";
            case LogCategory::OBC:     return "OBC";
            case LogCategory::EPS:     return "EPS";
            case LogCategory::ADCS:    return "ADCS";
            case LogCategory::COMM:    return "COMM";
            case LogCategory::PAYLOAD: return "PAY";
            case LogCategory::TCS:     return "TCS";
            case LogCategory::FDIR:    return "FDIR";
            case LogCategory::MISSION: return "MSN";
            default: return "???";
        }
    }
    
    static const char* levelColorCode(LogLevel level) {
        switch (level) {
            case LogLevel::DEBUG:    return "\033[36m";    // Cyan
            case LogLevel::INFO:     return "\033[32m";    // Green
            case LogLevel::NOTICE:   return "\033[34m";    // Blue
            case LogLevel::WARNING:  return "\033[33m";    // Yellow
            case LogLevel::ERROR:    return "\033[31m";    // Red
            case LogLevel::CRITICAL: return "\033[35;1m";  // Bright Magenta
            default: return "\033[0m";
        }
    }
    
    static const char* shortFileName(const char* path) {
        const char* name = path;
        for (const char* p = path; *p; p++) {
            if (*p == '/' || *p == '\\') name = p + 1;
        }
        return name;
    }
};

// ============================================================================
// Кольцевой буфер лога
// ============================================================================

template<size_t Size>
class LogBuffer {
public:
    bool push(const LogEntry& entry) {
        std::lock_guard<std::mutex> lock(mutex_);
        entries_[head_] = entry;
        head_ = (head_ + 1) % Size;
        if (count_ < Size) count_++;
        return true;
    }

    bool pop(LogEntry& entry) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (count_ == 0) return false;
        size_t tail = (head_ + Size - count_) % Size;
        entry = entries_[tail];
        count_--;
        return true;
    }

    bool get(size_t index, LogEntry& entry) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (index >= count_) return false;
        size_t actualIndex = (head_ + Size - count_ + index) % Size;
        entry = entries_[actualIndex];
        return true;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return count_;
    }
    size_t capacity() const { return Size; }
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        count_ = 0; head_ = 0;
    }

private:
    std::array<LogEntry, Size> entries_{};
    size_t head_ = 0;
    size_t count_ = 0;
    mutable std::mutex mutex_;
};

// ============================================================================
// Основной логгер
// ============================================================================

class Logger {
public:
    static constexpr size_t MAX_OUTPUTS = 4;
    static constexpr size_t BUFFER_SIZE = 256;
    
    static Logger& instance() {
        static Logger logger;
        return logger;
    }
    
    void setConfig(const LoggerConfig& config) {
        std::lock_guard<std::mutex> lock(mutex_);
        config_ = config;
    }

    /**
     * @brief Сбросить состояние логгера (для тестирования)
     */
    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.clear();
        outputCount_ = 0;
        for (size_t i = 0; i < MAX_OUTPUTS; i++) {
            outputs_[i] = nullptr;
        }
        config_ = LoggerConfig{};
    }
    
    void addOutput(ILogOutput* output) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (outputCount_ < MAX_OUTPUTS) {
            outputs_[outputCount_++] = output;
        }
    }
    
    void setTimestampSource(Callback<uint32_t()> getSeconds,
                            Callback<uint16_t()> getMs) {
        getSeconds_ = getSeconds;
        getMs_ = getMs;
    }
    
    // Основной метод логирования
    void log(LogLevel level, LogCategory category,
             const char* file, int line, const char* function,
             const char* fmt, ...) {
        if (level < config_.minLevel) return;

        std::lock_guard<std::mutex> lock(mutex_);

        LogEntry entry{};

        // Время
        if (getSeconds_) entry.timestamp = getSeconds_();
        if (getMs_) entry.milliseconds = getMs_();

        entry.level = level;
        entry.category = category;
        entry.line = static_cast<uint16_t>(line);
        entry.file = file;
        entry.function = function;

        // Форматирование сообщения
        va_list args;
        va_start(args, fmt);
        vsnprintf(entry.message, sizeof(entry.message), fmt, args);
        va_end(args);

        // Добавление в буфер
        buffer_.push(entry);

        // Отправка на выходы
        char formatted[256];
        LogFormatter::format(entry, formatted, sizeof(formatted), config_.colorOutput);

        for (size_t i = 0; i < outputCount_; i++) {
            if (outputs_[i]) {
                outputs_[i]->write(entry, formatted);
            }
        }
    }
    
    // Удобные методы
    void debug(LogCategory cat, const char* file, int line, 
               const char* func, const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        char msg[128];
        vsnprintf(msg, sizeof(msg), fmt, args);
        va_end(args);
        log(LogLevel::DEBUG, cat, file, line, func, "%s", msg);
    }
    
    void info(LogCategory cat, const char* file, int line,
              const char* func, const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        char msg[128];
        vsnprintf(msg, sizeof(msg), fmt, args);
        va_end(args);
        log(LogLevel::INFO, cat, file, line, func, "%s", msg);
    }
    
    void warning(LogCategory cat, const char* file, int line,
                 const char* func, const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        char msg[128];
        vsnprintf(msg, sizeof(msg), fmt, args);
        va_end(args);
        log(LogLevel::WARNING, cat, file, line, func, "%s", msg);
    }
    
    void error(LogCategory cat, const char* file, int line,
               const char* func, const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        char msg[128];
        vsnprintf(msg, sizeof(msg), fmt, args);
        va_end(args);
        log(LogLevel::ERROR, cat, file, line, func, "%s", msg);
    }
    
    void critical(LogCategory cat, const char* file, int line,
                  const char* func, const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        char msg[128];
        vsnprintf(msg, sizeof(msg), fmt, args);
        va_end(args);
        log(LogLevel::CRITICAL, cat, file, line, func, "%s", msg);
    }
    
    // Доступ к буферу
    size_t getEntryCount() const { return buffer_.size(); }
    bool getEntry(size_t index, LogEntry& entry) const { return buffer_.get(index, entry); }
    void clearBuffer() { buffer_.clear(); }
    
private:
    Logger() = default;

    LoggerConfig config_;
    LogBuffer<BUFFER_SIZE> buffer_;
    ILogOutput* outputs_[MAX_OUTPUTS] = {};
    size_t outputCount_ = 0;

    Callback<uint32_t()> getSeconds_;
    Callback<uint16_t()> getMs_;
    std::mutex mutex_;  ///< Мьютекс для thread-safe логирования
};

// ============================================================================
// Макросы логирования
// ============================================================================

#define LOG_DEBUG(cat, fmt, ...) \
    mka::log::Logger::instance().debug(cat, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#define LOG_INFO(cat, fmt, ...) \
    mka::log::Logger::instance().info(cat, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#define LOG_WARNING(cat, fmt, ...) \
    mka::log::Logger::instance().warning(cat, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#define LOG_ERROR(cat, fmt, ...) \
    mka::log::Logger::instance().error(cat, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#define LOG_CRITICAL(cat, fmt, ...) \
    mka::log::Logger::instance().critical(cat, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

// ============================================================================
// Реализации выходов
// ============================================================================

/**
 * @brief Вывод в UART
 */
class UARTLogOutput : public ILogOutput {
public:
    using WriteFunc = Callback<void(const char*, size_t)>;

    UARTLogOutput(WriteFunc write) : write_(write) {}

    void write(const LogEntry& entry, const char* formatted) override {
        (void)entry;  // Форматирование уже выполнено
        size_t len = strlen(formatted);
        write_(formatted, len);
        write_("\r\n", 2);
    }

private:
    WriteFunc write_;
};

/**
 * @brief Вывод в кольцевой буфер (для последующей передачи)
 */
class BufferLogOutput : public ILogOutput {
public:
    static constexpr size_t BUFFER_SIZE = 4096;

    void write(const LogEntry& entry, const char* formatted) override {
        (void)entry;  // Форматирование уже выполнено
        size_t len = strlen(formatted);
        for (size_t i = 0; i < len && count_ < BUFFER_SIZE; i++) {
            buffer_[count_++] = formatted[i];
        }
        if (count_ < BUFFER_SIZE) {
            buffer_[count_++] = '\n';
        }
    }

    const char* getData() const { return buffer_.data(); }
    size_t getSize() const { return count_; }

    void clear() { count_ = 0; }

private:
    std::array<char, BUFFER_SIZE> buffer_{};
    size_t count_ = 0;
};

} // namespace log
} // namespace mka

#endif // LOG_SYSTEM_HPP
