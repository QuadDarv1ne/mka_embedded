/**
 * @file eeprom_driver.hpp
 * @brief EEPROM/FRAM Driver for MKA
 * 
 * Драйверы для энергонезависимой памяти:
 * - I2C EEPROM (24LC256, AT24C256 и совместимые)
 * - SPI FRAM (FM25W256, MB85RS256 и совместимые)
 */

#ifndef EEPROM_DRIVER_HPP
#define EEPROM_DRIVER_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <functional>
#include <array>
#include <span>

#if !defined(STM32F4) && !defined(STM32F7) && !defined(STM32H7)
#include <thread>
#include <chrono>
#endif

#include "../utils/span.hpp"

namespace mka {
namespace nvm {

// ============================================================================
// Интерфейсы
// ============================================================================

/**
 * @brief Интерфейс I2C для EEPROM
 */
class II2CBus {
public:
    virtual ~II2CBus() = default;
    virtual bool write(uint8_t addr, uint16_t reg, std::span<const uint8_t> data) = 0;
    virtual bool read(uint8_t addr, uint16_t reg, std::span<uint8_t> data) = 0;
};

/**
 * @brief Интерфейс SPI для FRAM
 */
class ISPIBus {
public:
    virtual ~ISPIBus() = default;
    virtual bool transfer(std::span<const uint8_t> tx, std::span<uint8_t> rx) = 0;
    virtual void select() = 0;
    virtual void deselect() = 0;
};

// ============================================================================
// I2C EEPROM Driver (24LC256/AT24C256)
// ============================================================================

/**
 * @brief Драйвер I2C EEPROM
 * 
 * Типичные характеристики:
 * - Объём: 256 Kbit (32 KB)
 * - Адресация: 16-бит
 * - Скорость: до 1 MHz (Fast Mode Plus)
 * - Запись: 64 байта на страницу, 5 мс cycle time
 */
class EEPROM24LC256 {
public:
    // Параметры устройства
    static constexpr uint8_t DEFAULT_ADDRESS = 0x50;
    static constexpr size_t PAGE_SIZE = 64;
    static constexpr size_t TOTAL_SIZE = 32768;
    static constexpr uint32_t WRITE_CYCLE_MS = 5;
    
    explicit EEPROM24LC256(II2CBus& i2c, uint8_t address = DEFAULT_ADDRESS)
        : i2c_(i2c), address_(address) {}
    
    /**
     * @brief Инициализация
     */
    bool init() {
        // Проверка наличия устройства
        uint8_t data;
        return read(0, {&data, 1});
    }
    
    /**
     * @brief Чтение байта
     */
    bool readByte(uint16_t address, uint8_t& data) {
        return read(address, {&data, 1});
    }
    
    /**
     * @brief Запись байта
     */
    bool writeByte(uint16_t address, uint8_t data) {
        return write(address, {&data, 1});
    }
    
    /**
     * @brief Чтение массива данных
     */
    bool read(uint16_t address, std::span<uint8_t> data) {
        // Защита от integer overflow: используем безопасную проверку
        if (data.size() > TOTAL_SIZE || static_cast<uint32_t>(address) + data.size() > TOTAL_SIZE) {
            return false;
        }
        return i2c_.read(address_, address, data);
    }

    /**
     * @brief Запись массива данных (с учётом границ страниц)
     */
    bool write(uint16_t address, std::span<const uint8_t> data) {
        // Защита от integer overflow
        if (data.size() > TOTAL_SIZE || static_cast<uint32_t>(address) + data.size() > TOTAL_SIZE) {
            return false;
        }
        
        size_t offset = 0;
        
        while (offset < data.size()) {
            // Вычисление размера записи (до границы страницы)
            uint16_t pageOffset = address % PAGE_SIZE;
            size_t chunkSize = PAGE_SIZE - pageOffset;
            chunkSize = std::min(chunkSize, data.size() - offset);
            
            // Запись страницы
            if (!i2c_.write(address_, address, data.subspan(offset, chunkSize))) {
                return false;
            }
            
            // Ожидание завершения записи
            delayMs(WRITE_CYCLE_MS);
            
            address += chunkSize;
            offset += chunkSize;
        }
        
        return true;
    }
    
    /**
     * @brief Заполнение области памяти
     */
    bool fill(uint16_t address, size_t length, uint8_t value) {
        std::array<uint8_t, PAGE_SIZE> buffer;
        buffer.fill(value);
        
        while (length > 0) {
            size_t chunkSize = std::min(length, buffer.size());
            if (!write(address, {buffer.data(), chunkSize})) {
                return false;
            }
            address += chunkSize;
            length -= chunkSize;
        }
        
        return true;
    }
    
    /**
     * @brief Стирание всего чипа
     */
    bool eraseAll() {
        return fill(0, TOTAL_SIZE, 0xFF);
    }
    
    /**
     * @brief Проверка на пустоту
     */
    bool isEmpty(uint16_t address, size_t length) {
        uint8_t buffer[32];
        
        while (length > 0) {
            size_t chunkSize = std::min(length, sizeof(buffer));
            if (!read(address, {buffer, chunkSize})) {
                return false;
            }
            
            for (size_t i = 0; i < chunkSize; i++) {
                if (buffer[i] != 0xFF) {
                    return false;
                }
            }
            
            address += chunkSize;
            length -= chunkSize;
        }
        
        return true;
    }
    
    size_t getSize() const { return TOTAL_SIZE; }
    size_t getPageSize() const { return PAGE_SIZE; }

private:
    II2CBus& i2c_;
    uint8_t address_;

    void delayMs(uint32_t ms) {
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7)
        extern void HAL_Delay(uint32_t);
        HAL_Delay(ms);
#else
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
#endif
    }
};

// ============================================================================
// SPI FRAM Driver (FM25W256/MB85RS256)
// ============================================================================

/**
 * @brief Драйвер SPI FRAM
 * 
 * Преимущества FRAM:
 * - Мгновенная запись (нет write cycle time)
 * - Высокая износостойкость (10^14 циклов)
 * - Низкое энергопотребление
 * 
 * Характеристики FM25W256:
 * - Объём: 256 Kbit (32 KB)
 * - Адресация: 16-бит
 * - Скорость SPI: до 40 MHz
 */
class FRAM_FM25W256 {
public:
    // Opcodes
    static constexpr uint8_t OPCODE_WREN = 0x06;    // Write Enable
    static constexpr uint8_t OPCODE_WRDI = 0x04;    // Write Disable
    static constexpr uint8_t OPCODE_RDSR = 0x05;    // Read Status Register
    static constexpr uint8_t OPCODE_WRSR = 0x01;    // Write Status Register
    static constexpr uint8_t OPCODE_READ = 0x03;    // Read Memory
    static constexpr uint8_t OPCODE_FSTRD = 0x0B;   // Fast Read
    static constexpr uint8_t OPCODE_WRITE = 0x02;   // Write Memory
    static constexpr uint8_t OPCODE_RDID = 0x9F;    // Read Device ID
    
    // Параметры устройства
    static constexpr size_t TOTAL_SIZE = 32768;
    
    explicit FRAM_FM25W256(ISPIBus& spi) : spi_(spi) {}
    
    /**
     * @brief Инициализация и проверка устройства
     */
    bool init() {
        // Чтение ID устройства
        DeviceID id;
        if (!readDeviceID(id)) {
            return false;
        }
        
        // Проверка manufacturer ID (Cypress = 0x04, Fujitsu = 0x04)
        return id.manufacturerId != 0xFF && id.manufacturerId != 0x00;
    }
    
    /**
     * @brief Структура ID устройства
     */
    struct DeviceID {
        uint8_t manufacturerId;
        uint8_t productId;
        uint8_t productRev;
    };
    
    /**
     * @brief Чтение ID устройства
     */
    bool readDeviceID(DeviceID& id) {
        spi_.select();
        
        uint8_t tx[4] = {OPCODE_RDID, 0, 0, 0};
        uint8_t rx[4];
        
        bool success = spi_.transfer(tx, rx);
        spi_.deselect();
        
        if (success) {
            id.manufacturerId = rx[1];
            id.productId = rx[2];
            id.productRev = rx[3];
        }
        
        return success;
    }
    
    /**
     * @brief Чтение байта
     */
    bool readByte(uint16_t address, uint8_t& data) {
        return read(address, {&data, 1});
    }
    
    /**
     * @brief Запись байта
     */
    bool writeByte(uint16_t address, uint8_t data) {
        return write(address, {&data, 1});
    }
    
    /**
     * @brief Чтение массива данных
     */
    bool read(uint16_t address, std::span<uint8_t> data) {
        // Защита от integer overflow
        if (data.size() > TOTAL_SIZE || static_cast<uint32_t>(address) + data.size() > TOTAL_SIZE) {
            return false;
        }

        spi_.select();

        // Команда чтения + адрес
        uint8_t header[3] = {
            OPCODE_READ,
            static_cast<uint8_t>(address >> 8),
            static_cast<uint8_t>(address & 0xFF)
        };

        bool success = spi_.transfer(header, {});
        if (success) {
            success = spi_.transfer({}, data);
        }

        spi_.deselect();
        return success;
    }
    
    /**
     * @brief Быстрое чтение (с dummy byte)
     */
    bool fastRead(uint16_t address, std::span<uint8_t> data) {
        // Защита от integer overflow
        if (data.size() > TOTAL_SIZE || static_cast<uint32_t>(address) + data.size() > TOTAL_SIZE) {
            return false;
        }

        spi_.select();

        uint8_t header[4] = {
            OPCODE_FSTRD,
            static_cast<uint8_t>(address >> 8),
            static_cast<uint8_t>(address & 0xFF),
            0x00  // Dummy byte
        };

        bool success = spi_.transfer(header, {});
        if (success) {
            success = spi_.transfer({}, data);
        }

        spi_.deselect();
        return success;
    }
    
    /**
     * @brief Запись массива данных
     */
    bool write(uint16_t address, std::span<const uint8_t> data) {
        // Защита от integer overflow
        if (data.size() > TOTAL_SIZE || static_cast<uint32_t>(address) + data.size() > TOTAL_SIZE) {
            return false;
        }

        // Write Enable
        spi_.select();
        uint8_t wren = OPCODE_WREN;
        bool success = spi_.transfer({&wren, 1}, {});
        spi_.deselect();

        if (!success) {
            return false;
        }

        // Проверка что Write Enable прошёл успешно (чтение статусного регистра)
        uint8_t status = readStatusRegister();
        if (!(status & 0x02)) {  // WEL bit должен быть установлен
            return false;
        }

        // Write
        spi_.select();

        uint8_t header[3] = {
            OPCODE_WRITE,
            static_cast<uint8_t>(address >> 8),
            static_cast<uint8_t>(address & 0xFF)
        };

        success = spi_.transfer(header, {});
        if (success) {
            success = spi_.transfer(data, {});
        }

        spi_.deselect();

        // FRAM не требует ожидания!
        return success;
    }
    
    /**
     * @brief Чтение статусного регистра
     */
    uint8_t readStatusRegister() {
        spi_.select();
        
        uint8_t tx[2] = {OPCODE_RDSR, 0};
        uint8_t rx[2];
        
        spi_.transfer(tx, rx);
        spi_.deselect();
        
        return rx[1];
    }
    
    /**
     * @brief Запись статусного регистра
     */
    bool writeStatusRegister(uint8_t value) {
        // Write Enable
        spi_.select();
        uint8_t wren = OPCODE_WREN;
        spi_.transfer({&wren, 1}, {});
        spi_.deselect();
        
        // Write Status Register
        spi_.select();
        uint8_t tx[2] = {OPCODE_WRSR, value};
        spi_.transfer(tx, {});
        spi_.deselect();
        
        return true;
    }
    
    /**
     * @brief Установка защиты областей памяти
     */
    bool setBlockProtection(uint8_t blocks) {
        // Биты BP0, BP1 в статусном регистре
        uint8_t status = readStatusRegister();
        status = (status & ~0x0C) | ((blocks & 0x03) << 2);
        return writeStatusRegister(status);
    }
    
    size_t getSize() const { return TOTAL_SIZE; }
    
private:
    ISPIBus& spi_;
};

// ============================================================================
// NVM Manager
// ============================================================================

/**
 * @brief Менеджер энергонезависимой памяти
 * 
 * Предоставляет удобный интерфейс для хранения конфигурации
 * и калибровочных данных.
 */
template<typename NvmDriver>
class NVMManager {
public:
    static constexpr uint16_t CONFIG_START = 0x0000;
    static constexpr uint16_t CONFIG_SIZE = 0x0400;   // 1 KB for config
    static constexpr uint16_t CALIB_START = 0x0400;
    static constexpr uint16_t CALIB_SIZE = 0x0200;    // 512 B for calibration
    static constexpr uint16_t LOG_START = 0x0600;
    static constexpr uint16_t LOG_SIZE = 0x7A00;      // ~30 KB for logs
    
    struct ConfigHeader {
        uint32_t magic;
        uint16_t version;
        uint16_t size;
        uint16_t checksum;
        uint8_t reserved[6];
    };
    
    static constexpr uint32_t MAGIC_VALUE = 0x4D4B4131;  // "MKA1"
    
    explicit NVMManager(NvmDriver& driver) : driver_(driver) {}
    
    /**
     * @brief Сохранение конфигурации
     */
    bool saveConfig(std::span<const uint8_t> data, uint16_t version) {
        ConfigHeader header{};
        header.magic = MAGIC_VALUE;
        header.version = version;
        header.size = data.size();
        header.checksum = calculateChecksum(data);
        
        // Запись заголовка
        if (!driver_.write(CONFIG_START, 
                          {reinterpret_cast<uint8_t*>(&header), sizeof(header)})) {
            return false;
        }
        
        // Запись данных
        return driver_.write(CONFIG_START + sizeof(header), data);
    }
    
    /**
     * @brief Загрузка конфигурации
     */
    bool loadConfig(std::span<uint8_t> data, uint16_t& version) {
        ConfigHeader header;
        if (!driver_.read(CONFIG_START, 
                         {reinterpret_cast<uint8_t*>(&header), sizeof(header)})) {
            return false;
        }
        
        // Проверка magic
        if (header.magic != MAGIC_VALUE) {
            return false;
        }
        
        // Проверка размера
        if (header.size > data.size()) {
            return false;
        }
        
        // Чтение данных
        if (!driver_.read(CONFIG_START + sizeof(header), 
                         {data.data(), header.size})) {
            return false;
        }
        
        // Проверка контрольной суммы
        if (calculateChecksum({data.data(), header.size}) != header.checksum) {
            return false;
        }
        
        version = header.version;
        return true;
    }
    
    /**
     * @brief Сохранение калибровочных данных
     */
    bool saveCalibration(std::span<const uint8_t> data) {
        uint16_t checksum = calculateChecksum(data);
        
        // Запись контрольной суммы + данных
        uint8_t header[2] = {
            static_cast<uint8_t>(checksum >> 8),
            static_cast<uint8_t>(checksum & 0xFF)
        };
        
        if (!driver_.write(CALIB_START, header)) {
            return false;
        }
        
        return driver_.write(CALIB_START + 2, data);
    }
    
    /**
     * @brief Загрузка калибровочных данных
     */
    bool loadCalibration(std::span<uint8_t> data) {
        uint8_t header[2];
        if (!driver_.read(CALIB_START, header)) {
            return false;
        }
        
        uint16_t storedChecksum = (header[0] << 8) | header[1];
        
        if (!driver_.read(CALIB_START + 2, data)) {
            return false;
        }
        
        return calculateChecksum(data) == storedChecksum;
    }
    
private:
    NvmDriver& driver_;
    
    static uint16_t calculateChecksum(std::span<const uint8_t> data) {
        uint16_t crc = 0xFFFF;
        for (uint8_t byte : data) {
            crc ^= byte;
            for (int i = 0; i < 8; i++) {
                if (crc & 1) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }
};

} // namespace nvm
} // namespace mka

#endif // EEPROM_DRIVER_HPP
