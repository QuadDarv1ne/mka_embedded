/**
 * @file moscow_time.hpp
 * @brief Система Московского времени (МСК) для автоматической актуализации данных
 *
 * Обеспечивает:
 * - Конвертацию UTC ↔ МСК (UTC+3)
 * - Автоматическую актуализацию данных по расписанию
 * - Валидацию свежести данных (data freshness)
 * - Автоматические обновления при устаревании
 * - Интеграцию с SGP4, телеметрией, FDIR, health monitoring
 *
 * Московское время: UTC+3 (круглый год, без перехода на летнее время)
 */

#ifndef MOSCOW_TIME_HPP
#define MOSCOW_TIME_HPP

#include <cstdint>
#include <cmath>
#include <functional>
#include <chrono>
#include <cstdio>
#include <cstring>

namespace mka {
namespace systems {

// ============================================================================
// Константы Московского времени
// ============================================================================

namespace MoscowTimeConstants {
    constexpr int32_t UTC_OFFSET_SECONDS = 3 * 3600;  // МСК = UTC+3
    constexpr int32_t SECONDS_PER_DAY = 86400;
    constexpr int32_t SECONDS_PER_HOUR = 3600;
    constexpr int32_t MINUTES_PER_DAY = 1440;
}

// ============================================================================
// Структуры времени
// ============================================================================

/// Универсальная дата и время (UTC)
struct UTCDateTime {
    uint16_t year = 2000;
    uint8_t month = 1;
    uint8_t day = 1;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint16_t millisecond = 0;
    
    /// Получить Unix timestamp (секунды от epoch)
    uint64_t toUnixTimestamp() const {
        // Упрощённый расчёт дней от epoch (1970-01-01)
        // Для полной реализации нужен более сложный алгоритм
        uint64_t days = 0;
        int y = year;
        int m = month;
        
        // Коррекция для Января и Февраля
        if (m <= 2) {
            y--;
            m += 12;
        }
        
        // Дни от года
        days = 365 * y + y / 4 - y / 100 + y / 400;
        
        // Дни от месяца (приближённо)
        days += (153 * (m - 3) + 2) / 5;
        
        // Дни от дня
        days += day - 1;
        
        // Корректировка на Unix epoch (1970-01-01)
        days -= 719468;
        
        return days * 86400ULL + hour * 3600ULL + minute * 60ULL + second;
    }
    
    /// Установить из Unix timestamp
    void fromUnixTimestamp(uint64_t timestamp) {
        uint64_t days = timestamp / 86400;
        uint64_t seconds = timestamp % 86400;
        
        hour = seconds / 3600;
        minute = (seconds % 3600) / 60;
        second = seconds % 60;
        millisecond = 0;
        
        // Обратное преобразование дней в дату
        // Упрощённый алгоритм
        days += 719468;
        
        year = static_cast<uint16_t>((days * 400) / 146097 + 1);
        uint32_t dayOfYear = days - (365 * year + year / 4 - year / 100 + year / 400);
        
        month = 1;
        while (dayOfYear > 30) {
            dayOfYear -= 30;
            month++;
        }
        day = static_cast<uint8_t>(dayOfYear + 1);
    }
    
    /// Строковое представление "YYYY-MM-DD HH:MM:SS"
    const char* toString() const {
        static char buffer[32];
        snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d",
                 year, month, day, hour, minute, second);
        return buffer;
    }
};

/// МСК дата и время
struct MSKDateTime : public UTCDateTime {
    /// Конвертировать из UTC
    static MSKDateTime fromUTC(const UTCDateTime& utc) {
        MSKDateTime msk;
        uint64_t utcTimestamp = utc.toUnixTimestamp();
        uint64_t mskTimestamp = utcTimestamp + MoscowTimeConstants::UTC_OFFSET_SECONDS;
        msk.fromUnixTimestamp(mskTimestamp);
        return msk;
    }
    
    /// Конвертировать в UTC
    UTCDateTime toUTC() const {
        UTCDateTime utc;
        uint64_t mskTimestamp = toUnixTimestamp();
        uint64_t utcTimestamp = mskTimestamp - MoscowTimeConstants::UTC_OFFSET_SECONDS;
        utc.fromUnixTimestamp(utcTimestamp);
        return utc;
    }
    
    /// Получить текущее МСК время (если есть источник UTC)
    static MSKDateTime now() {
        // Получаем текущее время из системы
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        
        MSKDateTime msk;
        msk.fromUnixTimestamp(static_cast<uint64_t>(time_t));
        
        // Добавляем смещение МСК
        uint64_t mskTime = static_cast<uint64_t>(time_t) + MoscowTimeConstants::UTC_OFFSET_SECONDS;
        msk.fromUnixTimestamp(mskTime);
        
        return msk;
    }
    
    /// Проверка валидности
    bool isValid() const {
        return year >= 2000 && year <= 2100 &&
               month >= 1 && month <= 12 &&
               day >= 1 && day <= 31 &&
               hour < 24 && minute < 60 && second < 60;
    }
};

// ============================================================================
// Конвертер времени
// ============================================================================

/**
 * @brief Конвертер между UTC и МСК
 */
class MoscowTimeConverter {
public:
    /**
     * @brief Конвертировать UTC timestamp в МСК
     * @param utcTimestamp Unix timestamp в секундах (UTC)
     * @return МСК timestamp (Unix timestamp + 3 часа)
     */
    static uint64_t utcToMSK(uint64_t utcTimestamp) {
        return utcTimestamp + MoscowTimeConstants::UTC_OFFSET_SECONDS;
    }
    
    /**
     * @brief Конвертировать МСК timestamp в UTC
     * @param mskTimestamp МСК Unix timestamp
     * @return UTC timestamp
     */
    static uint64_t mskToUTC(uint64_t mskTimestamp) {
        return mskTimestamp - MoscowTimeConstants::UTC_OFFSET_SECONDS;
    }
    
    /**
     * @brief Получить смещение МСК в секундах
     */
    static int32_t getUTCOffset() {
        return MoscowTimeConstants::UTC_OFFSET_SECONDS;
    }
    
    /**
     * @brief Конвертировать UTC DateTime в МСК
     */
    static MSKDateTime utcToMSKDateTime(const UTCDateTime& utc) {
        return MSKDateTime::fromUTC(utc);
    }
    
    /**
     * @brief Получить текущее МСК время
     * @param utcSource Источник текущего UTC времени
     */
    static MSKDateTime getCurrentMSKTime(std::function<uint64_t()> utcSource) {
        if (utcSource) {
            uint64_t utcTime = utcSource();
            MSKDateTime msk;
            msk.fromUnixTimestamp(utcTime + MoscowTimeConstants::UTC_OFFSET_SECONDS);
            return msk;
        }
        return MSKDateTime::now();
    }
};

// ============================================================================
// Менеджер свежести данных
// ============================================================================

/// Статус свежести данных
enum class DataFreshnessStatus : uint8_t {
    FRESH = 0,          // Данные актуальны
    STALE = 1,          // Данные устарели (требуют обновления)
    EXPIRED = 2,        // Данные истекли (критическое устаревание)
    INVALID = 3         // Данные невалидны
};

/// Конфигурация свежести данных
struct DataFreshnessConfig {
    uint32_t freshThresholdSeconds;     // Порог свежести (до этого момента данные fresh)
    uint32_t staleThresholdSeconds;     // Порог устаревания (после этого stale)
    uint32_t expireThresholdSeconds;    // Порог истечения (после этого expired)
    bool autoUpdateOnStale;             // Автоматическое обновление при stale
    bool autoUpdateOnExpire;            // Автоматическое обновление при expire
};

/**
 * @brief Менеджер свежести данных с МСК временем
 * 
 * Мониторит актуальность данных и автоматически
 * инициирует обновление при устаревании.
 * 
 * Пример использования:
 * @code
 * DataFreshnessManager freshnessMgr;
 * freshnessMgr.init(utcTimeSource);
 * 
 * // Регистрация SGP4 данных
 * freshnessMgr.registerData("SGP4_TLE", {
 *     .freshThresholdSeconds = 3600,      // 1 час
 *     .staleThresholdSeconds = 7200,      // 2 часа
 *     .expireThresholdSeconds = 86400,    // 24 часа
 *     .autoUpdateOnStale = true,
 *     .autoUpdateOnExpire = true
 * });
 * 
 * // Проверка свежести
 * auto status = freshnessMgr.checkFreshNESS("SGP4_TLE", lastUpdateTime);
 * if (status == DataFreshnessStatus::STALE) {
 *     // Автоматическое обновление
 *     updateTLEData();
 * }
 * @endcode
 */
class DataFreshnessManager {
public:
    static constexpr size_t MAX_DATA_SOURCES = 32;
    
    DataFreshnessManager() = default;
    
    /**
     * @brief Инициализация менеджера
     * @param utcSource Источник текущего UTC времени (возвращает Unix timestamp)
     * @return true при успешной инициализации
     */
    bool init(std::function<uint64_t()> utcSource) {
        if (!utcSource) {
            return false;
        }
        utcSource_ = utcSource;
        return true;
    }
    
    /**
     * @brief Регистрация источника данных
     * @param name Имя источника
     * @param config Конфигурация свежести
     * @return true при успешной регистрации
     */
    bool registerDataSource(const char* name, const DataFreshnessConfig& config) {
        if (!name || sourceCount_ >= MAX_DATA_SOURCES) {
            return false;
        }
        
        names_[sourceCount_] = name;
        configs_[sourceCount_] = config;
        lastUpdateTimes_[sourceCount_] = 0;
        updateCounts_[sourceCount_] = 0;
        
        sourceCount_++;
        return true;
    }
    
    /**
     * @brief Проверка свежести данных
     * @param name Имя источника данных
     * @param lastUpdateTime Время последнего обновления (Unix timestamp)
     * @return Статус свежести
     */
    DataFreshnessStatus checkFreshNESS(const char* name, uint64_t lastUpdateTime) {
        if (!utcSource_) {
            return DataFreshnessStatus::INVALID;
        }
        
        // Находим источник
        int index = findSource(name);
        if (index < 0) {
            return DataFreshnessStatus::INVALID;
        }
        
        uint64_t currentTime = utcSource_();
        uint64_t age = currentTime - lastUpdateTime;
        
        lastUpdateTimes_[index] = lastUpdateTime;
        
        const auto& config = configs_[index];
        
        if (age <= config.freshThresholdSeconds) {
            return DataFreshnessStatus::FRESH;
        } else if (age <= config.staleThresholdSeconds) {
            return DataFreshnessStatus::STALE;
        } else if (age <= config.expireThresholdSeconds) {
            return DataFreshnessStatus::EXPIRED;
        } else {
            return DataFreshnessStatus::INVALID;
        }
    }
    
    /**
     * @brief Проверка свежести с автоматическим обновлением
     * @param name Имя источника данных
     * @param lastUpdateTime Время последнего обновления
     * @param updateCallback Callback для обновления данных
     * @return Статус свежести после обновления
     */
    DataFreshnessStatus checkAndAutoUpdate(const char* name, uint64_t lastUpdateTime,
                                           std::function<bool()> updateCallback) {
        DataFreshnessStatus status = checkFreshNESS(name, lastUpdateTime);
        
        bool shouldUpdate = false;
        
        switch (status) {
            case DataFreshnessStatus::STALE:
                shouldUpdate = getSourceConfig(name).autoUpdateOnStale;
                break;
            case DataFreshnessStatus::EXPIRED:
                shouldUpdate = getSourceConfig(name).autoUpdateOnExpire;
                break;
            case DataFreshnessStatus::INVALID:
                shouldUpdate = true;  // Всегда обновляем невалидные данные
                break;
            default:
                break;
        }
        
        if (shouldUpdate && updateCallback) {
            if (updateCallback()) {
                // Обновление успешно - записываем время в МСК
                recordUpdate(name, MoscowTimeConverter::utcToMSK(utcSource_()));
                return DataFreshnessStatus::FRESH;
            }
        }
        
        return status;
    }
    
    /**
     * @brief Записать время обновления (в МСК)
     * @param name Имя источника
     * @param mskUpdateTime МСК время обновления
     */
    void recordUpdate(const char* name, uint64_t mskUpdateTime) {
        int index = findSource(name);
        if (index >= 0) {
            lastUpdateTimes_[index] = MoscowTimeConverter::mskToUTC(mskUpdateTime);
            updateCounts_[index]++;
        }
    }
    
    /**
     * @brief Получить время последнего обновления (МСК)
     * @param name Имя источника
     * @return МСК timestamp или 0 если не найдено
     */
    uint64_t getLastUpdateTimeMSK(const char* name) const {
        int index = findSource(name);
        if (index >= 0) {
            return MoscowTimeConverter::utcToMSK(lastUpdateTimes_[index]);
        }
        return 0;
    }
    
    /**
     * @brief Получить количество обновлений
     * @param name Имя источника
     * @return Количество обновлений
     */
    uint32_t getUpdateCount(const char* name) const {
        int index = findSource(name);
        if (index >= 0) {
            return updateCounts_[index];
        }
        return 0;
    }
    
    /**
     * @brief Проверить все источникиы и вернуть наихудший статус
     * @return Наихудший статус среди всех источников
     */
    DataFreshnessStatus checkAllSources() {
        DataFreshnessStatus worstStatus = DataFreshnessStatus::FRESH;
        
        for (size_t i = 0; i < sourceCount_; i++) {
            if (lastUpdateTimes_[i] == 0) {
                worstStatus = DataFreshnessStatus::INVALID;
                break;
            }
            
            auto status = checkFreshNESS(names_[i], lastUpdateTimes_[i]);
            if (static_cast<uint8_t>(status) > static_cast<uint8_t>(worstStatus)) {
                worstStatus = status;
            }
        }
        
        return worstStatus;
    }
    
    /**
     * @brief Получить статистику по всем источникам
     * @param freshCount[out] Количество свежих источников
     * @param staleCount[out] Количество устаревших источников
     * @param expiredCount[out] Количество истёкших источников
     * @param invalidCount[out] Количество невалидных источников
     */
    void getFreshnessStats(uint8_t& freshCount, uint8_t& staleCount,
                           uint8_t& expiredCount, uint8_t& invalidCount) const {
        freshCount = 0;
        staleCount = 0;
        expiredCount = 0;
        invalidCount = 0;
        
        for (size_t i = 0; i < sourceCount_; i++) {
            if (lastUpdateTimes_[i] == 0) {
                invalidCount++;
                continue;
            }
            
            // Рассчитываем статус напрямую без вызова не-const метода
            uint64_t age = utcSource_() - lastUpdateTimes_[i];
            const auto& config = configs_[i];
            
            DataFreshnessStatus status;
            if (age <= config.freshThresholdSeconds) {
                status = DataFreshnessStatus::FRESH;
            } else if (age <= config.staleThresholdSeconds) {
                status = DataFreshnessStatus::STALE;
            } else if (age <= config.expireThresholdSeconds) {
                status = DataFreshnessStatus::EXPIRED;
            } else {
                status = DataFreshnessStatus::INVALID;
            }
            
            switch (status) {
                case DataFreshnessStatus::FRESH: freshCount++; break;
                case DataFreshnessStatus::STALE: staleCount++; break;
                case DataFreshnessStatus::EXPIRED: expiredCount++; break;
                case DataFreshnessStatus::INVALID: invalidCount++; break;
            }
        }
    }
    
private:
    std::function<uint64_t()> utcSource_;
    
    const char* names_[MAX_DATA_SOURCES];
    DataFreshnessConfig configs_[MAX_DATA_SOURCES];
    uint64_t lastUpdateTimes_[MAX_DATA_SOURCES];
    uint32_t updateCounts_[MAX_DATA_SOURCES];
    size_t sourceCount_ = 0;
    
    int findSource(const char* name) const {
        if (!name) return -1;
        for (size_t i = 0; i < sourceCount_; i++) {
            if (names_[i] && std::strcmp(names_[i], name) == 0) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }
    
    const DataFreshnessConfig& getSourceConfig(const char* name) const {
        static DataFreshnessConfig defaultConfig = {3600, 7200, 86400, false, false};
        int index = findSource(name);
        if (index >= 0) {
            return configs_[index];
        }
        return defaultConfig;
    }
};

// ============================================================================
// Автоматическая актуализация SGP4 данных
// ============================================================================

/**
 * @brief Менеджер актуализации орбитальных данных по МСК
 * 
 * Автоматически проверяет свежесть TLE данных и инициирует
 * обновление при устаревании.
 */
class SGP4DataFreshnessManager {
public:
    SGP4DataFreshnessManager() = default;
    
    /**
     * @brief Инициализация
     * @param utcSource Источник UTC времени
     * @return true при успехе
     */
    bool init(std::function<uint64_t()> utcSource) {
        return freshnessMgr_.init(utcSource);
    }
    
    /**
     * @brief Регистрация TLE данных
     * @param satelliteName Имя спутника
     * @param freshThresholdSeconds Порог свежести (по умолчанию 2 часа)
     * @param expireThresholdSeconds Порог истечения (по умолчанию 24 часа)
     * @return true при успехе
     */
    bool registerTLEData(const char* satelliteName, 
                         uint32_t freshThresholdSeconds = 7200,
                         uint32_t expireThresholdSeconds = 86400) {
        DataFreshnessConfig config = {
            .freshThresholdSeconds = freshThresholdSeconds,
            .staleThresholdSeconds = freshThresholdSeconds * 2,
            .expireThresholdSeconds = expireThresholdSeconds,
            .autoUpdateOnStale = true,
            .autoUpdateOnExpire = true
        };
        
        return freshnessMgr_.registerDataSource(satelliteName, config);
    }
    
    /**
     * @brief Проверить и автоматически обновить TLE данные
     * @param satelliteName Имя спутника
     * @param lastUpdateTime Время последнего обновления
     * @param updateCallback Callback для загрузки новых TLE
     * @return Статус свежести
     */
    DataFreshnessStatus checkAndUpdateTLE(const char* satelliteName,
                                          uint64_t lastUpdateTime,
                                          std::function<bool()> updateCallback) {
        return freshnessMgr_.checkAndAutoUpdate(satelliteName, lastUpdateTime, updateCallback);
    }
    
    /**
     * @brief Получить время последнего обновления TLE (МСК)
     */
    uint64_t getLastTLEUpdateTimeMSK(const char* satelliteName) const {
        return freshnessMgr_.getLastUpdateTimeMSK(satelliteName);
    }
    
    /**
     * @brief Получить менеджер свежести для доступа к статистике
     */
    DataFreshnessManager& getFreshnessManager() { return freshnessMgr_; }
    
private:
    DataFreshnessManager freshnessMgr_;
};

} // namespace systems
} // namespace mka

#endif // MOSCOW_TIME_HPP
