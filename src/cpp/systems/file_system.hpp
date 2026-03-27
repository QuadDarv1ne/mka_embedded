/**
 * @file file_system.hpp
 * @brief LittleFS-based file system for MKA embedded project
 *
 * Features:
 * - Power-loss resilient (journaling)
 * - Wear leveling
 * - Dynamic directory structure
 * - File attributes support
 * - Thread-safe (with FreeRTOS mutex)
 */

#ifndef FILE_SYSTEM_HPP
#define FILE_SYSTEM_HPP

#include <cstdint>
#include <cstddef>
#include <optional>
#include <functional>

#include "../hal/hal_full.hpp"
#include "../utils/result.hpp"
#include "../utils/span.hpp"

namespace mka {
namespace filesystem {

// ============================================================================
// Constants and Types
// ============================================================================

/// Maximum file path length
constexpr size_t MAX_PATH_LENGTH = 256;

/// Maximum file name length
constexpr size_t MAX_NAME_LENGTH = 255;

/// Maximum open files
constexpr size_t MAX_OPEN_FILES = 16;

/// Default block size (bytes)
constexpr size_t DEFAULT_BLOCK_SIZE = 4096;

/// Default read buffer size
constexpr size_t READ_BUFFER_SIZE = 512;

/// File system status
enum class FSStatus : uint8_t {
    OK = 0,
    ERROR = 1,
    NOT_FOUND = 2,
    EXISTS = 3,
    NOT_DIR = 4,
    IS_DIR = 5,
    NOT_EMPTY = 6,
    NO_SPACE = 7,
    NOT_MOUNTED = 8,
    INVALID_PATH = 9,
    NAME_TOO_LONG = 10,
    TOO_MANY_OPEN_FILES = 11,
    CORRUPT = 12,
    NOT_SUPPORTED = 13
};

/// File open modes
enum class FileMode : uint8_t {
    READ,           ///< Read only
    WRITE,          ///< Write (truncate)
    APPEND,         ///< Append
    READ_WRITE,     ///< Read + write
    WRITE_CREATE,   ///< Write + create if not exists
    APPEND_CREATE   ///< Append + create if not exists
};

/// File type
enum class FileType : uint8_t {
    NONE,
    FILE,
    DIRECTORY
};

/// File statistics
struct FileStats {
    FileType type = FileType::NONE;
    size_t size = 0;
    uint64_t created_time = 0;
    uint64_t modified_time = 0;
    uint32_t attributes_count = 0;
};

/// File system statistics
struct FSStats {
    size_t total_blocks = 0;
    size_t used_blocks = 0;
    size_t free_blocks = 0;
    size_t total_files = 0;
    size_t total_dirs = 0;
    size_t read_count = 0;
    size_t write_count = 0;
    size_t erase_count = 0;

    /// Get usage percentage
    float getUsagePercent() const {
        if (total_blocks == 0) return 0.0f;
        return (static_cast<float>(used_blocks) / total_blocks) * 100.0f;
    }

    /// Get total size in bytes
    size_t getTotalBytes() const {
        return total_blocks * DEFAULT_BLOCK_SIZE;
    }

    /// Get free size in bytes
    size_t getFreeBytes() const {
        return free_blocks * DEFAULT_BLOCK_SIZE;
    }
};

/// Wear leveling statistics
struct WearStats {
    uint32_t min_erases = 0;
    uint32_t max_erases = 0;
    float avg_erases = 0.0f;
    float wear_level = 0.0f;  /// 0 = perfect, 1 = max wear
};

// ============================================================================
// File Handle
// ============================================================================

/**
 * @brief File handle for read/write operations
 */
class FileHandle {
public:
    FileHandle() = default;
    ~FileHandle() { close(); }

    // Non-copyable
    FileHandle(const FileHandle&) = delete;
    FileHandle& operator=(const FileHandle&) = delete;

    // Movable
    FileHandle(FileHandle&& other) noexcept;
    FileHandle& operator=(FileHandle&& other) noexcept;

    /**
     * @brief Read data from file
     * @param buffer Output buffer
     * @param size Bytes to read
     * @return Bytes actually read or error
     */
    Result<int, FSStatus> read(void* buffer, size_t size);

    /**
     * @brief Write data to file
     * @param buffer Data to write
     * @param size Bytes to write
     * @return Bytes actually written or error
     */
    Result<int, FSStatus> write(const void* buffer, size_t size);

    /**
     * @brief Seek to position
     * @param offset Offset from whence
     * @param whence 0=start, 1=current, 2=end
     * @return New position or error
     */
    Result<int, FSStatus> seek(int32_t offset, int whence = 0);

    /**
     * @brief Get current position
     * @return Position in bytes
     */
    size_t tell() const { return position_; }

    /**
     * @brief Get file size
     * @return Size in bytes
     */
    size_t size() const { return size_; }

    /**
     * @brief Truncate file to size
     * @param size New size
     * @return Status
     */
    Result<FSStatus, FSStatus> truncate(size_t size);

    /**
     * @brief Flush buffers
     * @return Status
     */
    Result<FSStatus, FSStatus> flush();

    /**
     * @brief Close file
     * @return Status
     */
    Result<FSStatus, FSStatus> close();

    /**
     * @brief Check if file is open
     */
    bool isOpen() const { return is_open_; }

    /**
     * @brief Check if at end of file
     */
    bool eof() const { return position_ >= size_; }

private:
    friend class FileSystem;

    FileHandle(uint32_t id, const char* path, FileMode mode);

    uint32_t id_ = 0;
    char path_[MAX_PATH_LENGTH] = {};
    FileMode mode_ = FileMode::READ;
    size_t position_ = 0;
    size_t size_ = 0;
    bool is_open_ = false;
    void* internal_handle_ = nullptr;  // For real implementation
};

// ============================================================================
// File System Configuration
// ============================================================================

/**
 * @brief File system configuration
 */
struct FSConfig {
    size_t block_size = DEFAULT_BLOCK_SIZE;
    size_t block_count = 256;       ///< Number of blocks
    size_t name_max = MAX_NAME_LENGTH;
    size_t file_max = 2147483647;   ///< 2GB max file size
    size_t attr_max = 1022;
    size_t read_size = 1;
    size_t prog_size = 4;
    size_t lookahead_size = 8;
    uint32_t block_cycles = 512;    ///< Wear leveling cycles

    /// Get total size in bytes
    size_t getTotalSize() const {
        return block_size * block_count;
    }
};

// ============================================================================
// File System Interface
// ============================================================================

/**
 * @brief LittleFS-based file system interface
 *
 * Provides power-loss resilient file storage for embedded systems.
 * Supports wear leveling, directories, and file attributes.
 */
class FileSystem {
public:
    using BlockDeviceReadFunc = std::function<Result<int, FSStatus>(uint32_t block, void* buffer, size_t size)>;
    using BlockDeviceProgFunc = std::function<Result<int, FSStatus>(uint32_t block, const void* buffer, size_t size)>;
    using BlockDeviceEraseFunc = std::function<Result<int, FSStatus>(uint32_t block)>;
    using BlockDeviceSyncFunc = std::function<Result<FSStatus, FSStatus>()>;

    FileSystem();
    ~FileSystem();

    // Non-copyable
    FileSystem(const FileSystem&) = delete;
    FileSystem& operator=(const FileSystem&) = delete;

    /**
     * @brief Configure block device
     * @param read_func Block read function
     * @param prog_func Block program function
     * @param erase_func Block erase function
     * @param sync_func Sync function (optional)
     * @return Status
     */
    Result<FSStatus, FSStatus> configure(
        BlockDeviceReadFunc read_func,
        BlockDeviceProgFunc prog_func,
        BlockDeviceEraseFunc erase_func,
        BlockDeviceSyncFunc sync_func = nullptr
    );

    /**
     * @brief Configure with HAL flash interface
     * @param flash Flash interface
     * @param start_block First block address
     * @param block_count Number of blocks
     * @return Status
     */
    Result<FSStatus, FSStatus> configureFromFlash(
        hal::IFlash* flash,
        uint32_t start_block,
        size_t block_count
    );

    /**
     * @brief Format file system
     * @param config Configuration (optional, uses defaults if nullptr)
     * @return Status
     */
    Result<FSStatus, FSStatus> format(const FSConfig* config = nullptr);

    /**
     * @brief Mount file system
     * @return Status
     */
    Result<FSStatus, FSStatus> mount();

    /**
     * @brief Unmount file system
     * @return Status
     */
    Result<FSStatus, FSStatus> unmount();

    /**
     * @brief Check if file system is mounted
     */
    bool isMounted() const { return mounted_; }

    // ========================================================================
    // File Operations
    // ========================================================================

    /**
     * @brief Open file
     * @param path File path
     * @param mode Open mode
     * @return File handle or error
     */
    Result<FileHandle, FSStatus> open(const char* path, FileMode mode);

    /**
     * @brief Close file
     * @param handle File handle
     * @return Status
     */
    Result<FSStatus, FSStatus> close(FileHandle& handle);

    /**
     * @brief Remove file
     * @param path File path
     * @return Status
     */
    Result<FSStatus, FSStatus> remove(const char* path);

    /**
     * @brief Rename/move file
     * @param old_path Old path
     * @param new_path New path
     * @return Status
     */
    Result<FSStatus, FSStatus> rename(const char* old_path, const char* new_path);

    /**
     * @brief Get file statistics
     * @param path File path
     * @return Stats or error
     */
    Result<FileStats, FSStatus> stat(const char* path);

    /**
     * @brief Check if file exists
     * @param path File path
     * @return true if exists
     */
    bool exists(const char* path);

    /**
     * @brief Read entire file into buffer
     * @param path File path
     * @param buffer Output buffer
     * @param size Buffer size
     * @return Bytes read or error
     */
    Result<int, FSStatus> readFile(const char* path, void* buffer, size_t size);

    /**
     * @brief Write buffer to file (creates or truncates)
     * @param path File path
     * @param buffer Data buffer
     * @param size Data size
     * @return Status
     */
    Result<FSStatus, FSStatus> writeFile(const char* path, const void* buffer, size_t size);

    // ========================================================================
    // Directory Operations
    // ========================================================================

    /**
     * @brief Create directory
     * @param path Directory path
     * @return Status
     */
    Result<FSStatus, FSStatus> mkdir(const char* path);

    /**
     * @brief Remove empty directory
     * @param path Directory path
     * @return Status
     */
    Result<FSStatus, FSStatus> rmdir(const char* path);

    /**
     * @brief List directory contents
     * @param path Directory path
     * @param callback Callback for each entry (name, is_dir)
     * @return Status
     */
    Result<FSStatus, FSStatus> listdir(
        const char* path,
        std::function<void(const char* name, bool is_dir)> callback
    );

    /**
     * @brief Walk directory tree
     * @param path Root path
     * @param callback Callback for each directory (path, dirs, files)
     * @return Status
     */
    Result<FSStatus, FSStatus> walk(
        const char* path,
        std::function<void(const char* path, const char** dirs, size_t num_dirs,
                          const char** files, size_t num_files)> callback
    );

    // ========================================================================
    // File Attributes
    // ========================================================================

    /**
     * @brief Set file attribute
     * @param path File path
     * @param name Attribute name
     * @param value Attribute value
     * @param size Value size
     * @return Status
     */
    Result<FSStatus, FSStatus> setattr(const char* path, const char* name,
                                        const void* value, size_t size);

    /**
     * @brief Get file attribute
     * @param path File path
     * @param name Attribute name
     * @param buffer Output buffer
     * @param size Buffer size
     * @return Bytes read or error
     */
    Result<int, FSStatus> getattr(const char* path, const char* name,
                                   void* buffer, size_t size);

    // ========================================================================
    // Statistics
    // ========================================================================

    /**
     * @brief Get file system statistics
     * @return FS stats
     */
    FSStats getStats() const;

    /**
     * @brief Get wear leveling statistics
     * @return Wear stats
     */
    WearStats getWearStats() const;

    /**
     * @brief Get number of open files
     */
    size_t getOpenFileCount() const { return open_files_; }

    /**
     * @brief Get maximum open files
     */
    size_t getMaxOpenFiles() const { return MAX_OPEN_FILES; }

    // ========================================================================
    // System Operations
    // ========================================================================

    /**
     * @brief Sync file system (flush all buffers)
     * @return Status
     */
    Result<FSStatus, FSStatus> sync();

    /**
     * @brief Check and repair file system
     * @return true if repaired
     */
    bool checkAndRepair();

    /**
     * @brief Get last error
     */
    FSStatus getLastError() const { return last_error_; }

protected:
    // Internal methods for implementation
    Result<FSStatus, FSStatus> initBlockDevice();
    Result<FSStatus, FSStatus> deinitBlockDevice();

    int findFreeFileSlot() const;
    void updateStats();

    FSStatus last_error_ = FSStatus::OK;
    bool mounted_ = false;
    size_t open_files_ = 0;

    // Block device callbacks
    BlockDeviceReadFunc read_func_;
    BlockDeviceProgFunc prog_func_;
    BlockDeviceEraseFunc erase_func_;
    BlockDeviceSyncFunc sync_func_;

    // Configuration
    FSConfig config_;

    // Statistics
    FSStats stats_;
    WearStats wear_stats_;

    // Internal state (for real LittleFS implementation)
    void* lfs_handle_ = nullptr;
    void* block_device_ = nullptr;
};

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * @brief Convert FSStatus to string
 */
const char* fsStatusToString(FSStatus status);

/**
 * @brief Normalize file path (remove double slashes, handle relative paths)
 * @param path Input path
 * @param normalized Output normalized path
 * @param size Output buffer size
 * @return true if successful
 */
bool normalizePath(const char* path, char* normalized, size_t size);

/**
 * @brief Split path into directory and filename
 * @param path Full path
 * @param dir_path Output directory path
 * @param dir_size Directory buffer size
 * @param filename Output filename
 * @param fname_size Filename buffer size
 * @return true if successful
 */
bool splitPath(const char* path, char* dir_path, size_t dir_size,
               char* filename, size_t fname_size);

} // namespace filesystem
} // namespace mka

#endif // FILE_SYSTEM_HPP
