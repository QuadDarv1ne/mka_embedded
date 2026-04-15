/**
 * @file block_device.hpp
 * @brief Block device abstraction layer for LittleFS
 *
 * Provides abstract interface for block devices (flash, EEPROM, RAM, etc.)
 * with mock implementation for host testing.
 */

#ifndef BLOCK_DEVICE_HPP
#define BLOCK_DEVICE_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <vector>
#include <functional>
#include <mutex>

#include "../utils/result.hpp"

namespace mka {
namespace filesystem {

// ============================================================================
// Block Device Status
// ============================================================================

enum class BlockDeviceStatus : uint8_t {
    OK = 0,
    ERROR = 1,
    NOT_INITIALIZED = 2,
    READ_ERROR = 3,
    WRITE_ERROR = 4,
    ERASE_ERROR = 5,
    OUT_OF_BOUNDS = 6
};

// ============================================================================
// Block Device Interface
// ============================================================================

/**
 * @brief Abstract block device interface
 *
 * All block devices must implement this interface for use with LittleFS.
 * Block devices operate on fixed-size blocks with read/program/erase operations.
 */
class IBlockDevice {
public:
    virtual ~IBlockDevice() = default;

    /**
     * @brief Initialize block device
     * @return Status
     */
    virtual Result<void, BlockDeviceStatus> init() = 0;

    /**
     * @brief Deinitialize block device
     * @return Status
     */
    virtual Result<void, BlockDeviceStatus> deinit() = 0;

    /**
     * @brief Read data from blocks
     * @param block Starting block number
     * @param offset Offset within block
     * @param buffer Output buffer
     * @param size Number of bytes to read
     * @return Bytes read or error
     */
    virtual Result<int, BlockDeviceStatus> read(
        uint32_t block, uint32_t offset, void* buffer, size_t size
    ) = 0;

    /**
     * @brief Program (write) data to blocks
     * @param block Starting block number
     * @param offset Offset within block
     * @param buffer Data to write
     * @param size Number of bytes to write
     * @return Bytes written or error
     */
    virtual Result<int, BlockDeviceStatus> program(
        uint32_t block, uint32_t offset, const void* buffer, size_t size
    ) = 0;

    /**
     * @brief Erase a block
     * @param block Block number to erase
     * @return Status
     */
    virtual Result<int, BlockDeviceStatus> erase(uint32_t block) = 0;

    /**
     * @brief Sync device (flush buffers)
     * @return Status
     */
    virtual Result<void, BlockDeviceStatus> sync() = 0;

    /**
     * @brief Get block size
     * @return Block size in bytes
     */
    virtual size_t getBlockSize() const = 0;

    /**
     * @brief Get total block count
     * @return Number of blocks
     */
    virtual size_t getBlockCount() const = 0;

    /**
     * @brief Get total size
     * @return Total size in bytes
     */
    virtual size_t getTotalSize() const {
        return getBlockSize() * getBlockCount();
    }

    /**
     * @brief Get read size (minimum read unit)
     * @return Read size in bytes
     */
    virtual size_t getReadSize() const = 0;

    /**
     * @brief Get program size (minimum program unit)
     * @return Program size in bytes
     */
    virtual size_t getProgSize() const = 0;

    /**
     * @brief Check if device is initialized
     */
    virtual bool isInitialized() const = 0;

    /**
     * @brief Get erase count for a block (for wear leveling stats)
     * @param block Block number
     * @return Erase count or 0 if not tracked
     */
    virtual uint32_t getBlockEraseCount(uint32_t block) const {
        (void)block;
        return 0;
    }

    /**
     * @brief Get total erase operations
     * @return Total erase count
     */
    virtual uint32_t getTotalEraseCount() const {
        return 0;
    }
};

// ============================================================================
// Memory Block Device (Mock for Testing)
// ============================================================================

/**
 * @brief In-memory block device for host testing
 *
 * Simulates flash behavior with RAM-backed storage.
 * Supports configurable block size and count.
 */
class MemoryBlockDevice : public IBlockDevice {
public:
    /**
     * @brief Constructor
     * @param block_size Block size in bytes (default 4096)
     * @param block_count Number of blocks (default 256)
     * @param read_size Minimum read size (default 1)
     * @param prog_size Minimum program size (default 4)
     */
    MemoryBlockDevice(
        size_t block_size = 4096,
        size_t block_count = 256,
        size_t read_size = 1,
        size_t prog_size = 4
    ) : block_size_(block_size)
    , block_count_(block_count)
    , read_size_(read_size)
    , prog_size_(prog_size)
    , initialized_(false)
    , total_erases_(0) {
        erase_counts_.resize(block_count, 0);
    }

    ~MemoryBlockDevice() override {
        deinit();
    }

    // Non-copyable
    MemoryBlockDevice(const MemoryBlockDevice&) = delete;
    MemoryBlockDevice& operator=(const MemoryBlockDevice&) = delete;

    Result<void, BlockDeviceStatus> init() override {
        if (initialized_) {
            return Ok<BlockDeviceStatus>();
        }

        // Allocate and zero-initialize memory
        try {
            storage_.resize(block_size_ * block_count_, 0xFF); // Flash is 0xFF when erased
            initialized_ = true;
            return Ok<BlockDeviceStatus>();
        } catch (...) {
            return Err<void, BlockDeviceStatus>(BlockDeviceStatus::ERROR);
        }
    }

    Result<void, BlockDeviceStatus> deinit() override {
        if (!initialized_) {
            return Ok<BlockDeviceStatus>();
        }

        storage_.clear();
        initialized_ = false;
        return Ok<BlockDeviceStatus>();
    }

    Result<int, BlockDeviceStatus> read(
        uint32_t block, uint32_t offset, void* buffer, size_t size
    ) override {
        if (!initialized_) {
            return Err<int, BlockDeviceStatus>(BlockDeviceStatus::NOT_INITIALIZED);
        }

        // Validate bounds
        if (block >= block_count_) {
            return Err<int, BlockDeviceStatus>(BlockDeviceStatus::OUT_OF_BOUNDS);
        }

        if (offset >= block_size_) {
            return Err<int, BlockDeviceStatus>(BlockDeviceStatus::OUT_OF_BOUNDS);
        }
        size_t addr = static_cast<size_t>(block) * block_size_ + offset;
        if (addr + size > storage_.size() || addr + size < addr) {
            return Err<int, BlockDeviceStatus>(BlockDeviceStatus::OUT_OF_BOUNDS);
        }

        std::memcpy(buffer, storage_.data() + addr, size);
        return Ok<int, BlockDeviceStatus>(static_cast<int>(size));
    }

    Result<int, BlockDeviceStatus> program(
        uint32_t block, uint32_t offset, const void* buffer, size_t size
    ) override {
        if (!initialized_) {
            return Err<int, BlockDeviceStatus>(BlockDeviceStatus::NOT_INITIALIZED);
        }

        // Validate bounds
        if (block >= block_count_) {
            return Err<int, BlockDeviceStatus>(BlockDeviceStatus::OUT_OF_BOUNDS);
        }
        if (offset >= block_size_) {
            return Err<int, BlockDeviceStatus>(BlockDeviceStatus::OUT_OF_BOUNDS);
        }
        size_t addr = static_cast<size_t>(block) * block_size_ + offset;
        if (addr + size > storage_.size() || addr + size < addr) {
            return Err<int, BlockDeviceStatus>(BlockDeviceStatus::OUT_OF_BOUNDS);
        }

        std::memcpy(storage_.data() + addr, buffer, size);
        return Ok<int, BlockDeviceStatus>(static_cast<int>(size));
    }

    Result<int, BlockDeviceStatus> erase(uint32_t block) override {
        if (!initialized_) {
            return Err<int, BlockDeviceStatus>(BlockDeviceStatus::NOT_INITIALIZED);
        }

        // Validate bounds
        if (block >= block_count_) {
            return Err<int, BlockDeviceStatus>(BlockDeviceStatus::OUT_OF_BOUNDS);
        }

        // Erase block (set to 0xFF)
        size_t addr = static_cast<size_t>(block) * block_size_;
        std::fill(storage_.begin() + addr, storage_.begin() + addr + block_size_, 0xFF);

        // Track erase count for wear leveling
        if (block < erase_counts_.size()) {
            erase_counts_[block]++;
        }
        total_erases_++;

        return Ok<int, BlockDeviceStatus>(1);
    }

    Result<void, BlockDeviceStatus> sync() override {
        if (!initialized_) {
            return Err<void, BlockDeviceStatus>(BlockDeviceStatus::NOT_INITIALIZED);
        }
        return Ok<BlockDeviceStatus>();
    }

    size_t getBlockSize() const override { return block_size_; }
    size_t getBlockCount() const override { return block_count_; }
    size_t getReadSize() const override { return read_size_; }
    size_t getProgSize() const override { return prog_size_; }
    bool isInitialized() const override { return initialized_; }

    uint32_t getBlockEraseCount(uint32_t block) const override {
        if (block < erase_counts_.size()) {
            return erase_counts_[block];
        }
        return 0;
    }

    uint32_t getTotalEraseCount() const override { return total_erases_; }

    /**
     * @brief Get raw storage data (for debugging)
     */
    const std::vector<uint8_t>& getStorage() const { return storage_; }

    /**
     * @brief Reset device to factory state
     */
    void reset() {
        deinit();
        total_erases_ = 0;
        std::fill(erase_counts_.begin(), erase_counts_.end(), 0);
        init();
    }

private:
    size_t block_size_;
    size_t block_count_;
    size_t read_size_;
    size_t prog_size_;
    bool initialized_;
    uint32_t total_erases_;

    std::vector<uint8_t> storage_;
    std::vector<uint32_t> erase_counts_;
};

// ============================================================================
// Callback-based Block Device
// ============================================================================

/**
 * @brief Block device wrapper for callback functions
 *
 * Allows using existing callback-based block device code with new interface.
 */
class CallbackBlockDevice : public IBlockDevice {
public:
    using ReadFunc = std::function<Result<int, BlockDeviceStatus>(uint32_t, void*, size_t)>;
    using ProgFunc = std::function<Result<int, BlockDeviceStatus>(uint32_t, const void*, size_t)>;
    using EraseFunc = std::function<Result<int, BlockDeviceStatus>(uint32_t)>;
    using SyncFunc = std::function<Result<void, BlockDeviceStatus>()>;

    CallbackBlockDevice(
        ReadFunc read_func,
        ProgFunc prog_func,
        EraseFunc erase_func,
        SyncFunc sync_func,
        size_t block_size = 4096,
        size_t block_count = 256,
        size_t read_size = 1,
        size_t prog_size = 4
    ) : read_func_(std::move(read_func))
    , prog_func_(std::move(prog_func))
    , erase_func_(std::move(erase_func))
    , sync_func_(std::move(sync_func))
    , block_size_(block_size)
    , block_count_(block_count)
    , read_size_(read_size)
    , prog_size_(prog_size)
    , initialized_(true) {}

    Result<void, BlockDeviceStatus> init() override {
        initialized_ = true;
        return Ok<BlockDeviceStatus>();
    }

    Result<void, BlockDeviceStatus> deinit() override {
        initialized_ = false;
        return Ok<BlockDeviceStatus>();
    }

    Result<int, BlockDeviceStatus> read(
        uint32_t block, uint32_t offset, void* buffer, size_t size
    ) override {
        (void)offset; // Callbacks don't use offset
        if (!initialized_) {
            return Err<int, BlockDeviceStatus>(BlockDeviceStatus::NOT_INITIALIZED);
        }
        return read_func_(block, buffer, size);
    }

    Result<int, BlockDeviceStatus> program(
        uint32_t block, uint32_t offset, const void* buffer, size_t size
    ) override {
        (void)offset; // Callbacks don't use offset
        if (!initialized_) {
            return Err<int, BlockDeviceStatus>(BlockDeviceStatus::NOT_INITIALIZED);
        }
        return prog_func_(block, buffer, size);
    }

    Result<int, BlockDeviceStatus> erase(uint32_t block) override {
        if (!initialized_) {
            return Err<int, BlockDeviceStatus>(BlockDeviceStatus::NOT_INITIALIZED);
        }
        return erase_func_(block);
    }

    Result<void, BlockDeviceStatus> sync() override {
        if (!initialized_) {
            return Err<void, BlockDeviceStatus>(BlockDeviceStatus::NOT_INITIALIZED);
        }
        return sync_func_();
    }

    size_t getBlockSize() const override { return block_size_; }
    size_t getBlockCount() const override { return block_count_; }
    size_t getReadSize() const override { return read_size_; }
    size_t getProgSize() const override { return prog_size_; }
    bool isInitialized() const override { return initialized_; }

private:
    ReadFunc read_func_;
    ProgFunc prog_func_;
    EraseFunc erase_func_;
    SyncFunc sync_func_;
    size_t block_size_;
    size_t block_count_;
    size_t read_size_;
    size_t prog_size_;
    bool initialized_;
};

// ============================================================================
// Block Device Status String
// ============================================================================

inline const char* blockDeviceStatusToString(BlockDeviceStatus status) {
    switch (status) {
        case BlockDeviceStatus::OK: return "OK";
        case BlockDeviceStatus::ERROR: return "ERROR";
        case BlockDeviceStatus::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case BlockDeviceStatus::READ_ERROR: return "READ_ERROR";
        case BlockDeviceStatus::WRITE_ERROR: return "WRITE_ERROR";
        case BlockDeviceStatus::ERASE_ERROR: return "ERASE_ERROR";
        case BlockDeviceStatus::OUT_OF_BOUNDS: return "OUT_OF_BOUNDS";
        default: return "UNKNOWN";
    }
}

} // namespace filesystem
} // namespace mka

#endif // BLOCK_DEVICE_HPP
