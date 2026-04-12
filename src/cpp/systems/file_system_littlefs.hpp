/**
 * @file file_system_littlefs.hpp
 * @brief LittleFS file system implementation
 * 
 * Provides inline implementations for FileSystem and FileHandle.
 * Must be included AFTER file_system.hpp (which declares all types).
 */

#ifndef FILE_SYSTEM_LITTLEFS_HPP
#define FILE_SYSTEM_LITTLEFS_HPP

#ifdef USE_LITTLEFS
#include "lfs.h"
#endif

#include <map>
#include <vector>
#include <cstring>
#include <algorithm>

namespace mka {
namespace filesystem {

// ============================================================================
// Mock file storage for host testing
// ============================================================================

#ifndef USE_LITTLEFS
namespace detail {
    struct MockFile {
        std::vector<uint8_t> data;
        bool is_directory = false;
    };
    inline std::map<std::string, MockFile>& mockStorage() {
        static std::map<std::string, MockFile> storage;
        return storage;
    }
}
#endif

// ============================================================================
// FileHandle implementations
// ============================================================================

inline FileHandle::FileHandle(FileHandle&& other) noexcept
    : file_id_(other.file_id_)
    , position_(other.position_)
    , size_(other.size_)
    , is_open_(other.is_open_)
    , file_path_(std::move(other.file_path_))
#ifdef USE_LITTLEFS
    , lfs_file_(other.lfs_file_)
    , lfs_handle_(other.lfs_handle_)
#endif
{
    other.is_open_ = false;
    other.file_id_ = 0;
    other.position_ = 0;
    other.size_ = 0;
#ifdef USE_LITTLEFS
    other.lfs_file_ = nullptr;
    other.lfs_handle_ = nullptr;
#endif
}

inline FileHandle& FileHandle::operator=(FileHandle&& other) noexcept {
    if (this != &other) {
        if (is_open_) close();
        file_id_ = other.file_id_;
        position_ = other.position_;
        size_ = other.size_;
        is_open_ = other.is_open_;
        file_path_ = std::move(other.file_path_);
#ifdef USE_LITTLEFS
        lfs_file_ = other.lfs_file_;
        lfs_handle_ = other.lfs_handle_;
        other.lfs_file_ = nullptr;
        other.lfs_handle_ = nullptr;
#endif
        other.is_open_ = false;
        other.file_id_ = 0;
        other.position_ = 0;
        other.size_ = 0;
    }
    return *this;
}

inline Result<int, FSStatus> FileHandle::read(void* buffer, size_t size) {
    if (!is_open_) return Err<int, FSStatus>(FSStatus::ERROR);
    if (!buffer && size > 0) return Err<int, FSStatus>(FSStatus::ERROR);
    if (size == 0) return Ok<int, FSStatus>(0);
    
#ifndef USE_LITTLEFS
    // Read from mock storage using file_path_
    if (!file_path_.empty()) {
        auto& storage = detail::mockStorage();
        auto it = storage.find(file_path_);
        if (it != storage.end()) {
            size_t available = (it->second.data.size() > position_) ? 
                               (it->second.data.size() - position_) : 0;
            size_t to_read = (size < available) ? size : available;
            if (to_read > 0 && buffer) {
                std::memcpy(buffer, it->second.data.data() + position_, to_read);
            }
            position_ += to_read;
            return Ok<int, FSStatus>(static_cast<int>(to_read));
        }
    }
#endif
    
    return Ok<int, FSStatus>(0);
}

inline Result<int, FSStatus> FileHandle::write(const void* buffer, size_t size) {
    if (!is_open_) return Err<int, FSStatus>(FSStatus::ERROR);
    if (!buffer && size > 0) return Err<int, FSStatus>(FSStatus::ERROR);
    
#ifndef USE_LITTLEFS
    // Write to mock storage using file_path_
    if (!file_path_.empty()) {
        auto& storage = detail::mockStorage();
        auto it = storage.find(file_path_);
        if (it != storage.end()) {
            size_t end_pos = position_ + size;
            if (end_pos > it->second.data.size()) {
                it->second.data.resize(end_pos);
            }
            if (buffer) {
                std::memcpy(it->second.data.data() + position_, buffer, size);
            }
        }
    }
#endif
    
    position_ += size;
    if (position_ > size_) size_ = position_;
    return Ok<int, FSStatus>(static_cast<int>(size));
}

inline Result<int, FSStatus> FileHandle::seek(int32_t offset, int whence) {
    if (!is_open_) return Err<int, FSStatus>(FSStatus::ERROR);
    int32_t new_pos = 0;
    switch (whence) {
        case 0: new_pos = offset; break;
        case 1: new_pos = static_cast<int32_t>(position_) + offset; break;
        case 2: new_pos = static_cast<int32_t>(size_) + offset; break;
        default: return Err<int, FSStatus>(FSStatus::ERROR);
    }
    if (new_pos < 0) return Err<int, FSStatus>(FSStatus::ERROR);
    position_ = static_cast<size_t>(new_pos);
    if (position_ > size_) size_ = position_;
    return Ok<int, FSStatus>(static_cast<int>(position_));
}

inline Result<void, FSStatus> FileHandle::truncate(size_t size) {
    if (!is_open_) return Err<void, FSStatus>(FSStatus::ERROR);
    
#ifndef USE_LITTLEFS
    // Truncate mock storage
    if (!file_path_.empty()) {
        auto& storage = detail::mockStorage();
        auto it = storage.find(file_path_);
        if (it != storage.end()) {
            it->second.data.resize(size);
        }
    }
#endif
    
    size_ = size;
    if (position_ > size_) position_ = size_;
    return Ok<FSStatus>();
}

inline Result<void, FSStatus> FileHandle::flush() {
    if (!is_open_) return Err<void, FSStatus>(FSStatus::ERROR);
    return Ok<FSStatus>();
}

inline Result<void, FSStatus> FileHandle::close() {
    if (!is_open_) return Ok<FSStatus>();
    is_open_ = false;
    position_ = 0;
    return Ok<FSStatus>();
}

// ============================================================================
// FileSystem implementations
// ============================================================================

inline FileSystem::FileSystem() 
    : last_error_(FSStatus::OK), mounted_(false), open_files_(0),
      block_device_(nullptr), config_{}, stats_{}, wear_stats_{} {
}

inline FileSystem::~FileSystem() {
    if (mounted_) unmount();
}

inline Result<void, FSStatus> FileSystem::configure(IBlockDevice* block_device) {
    if (!block_device) return Err<void, FSStatus>(FSStatus::ERROR);
    block_device_ = block_device;
    config_.block_size = block_device->getBlockSize();
    config_.block_count = block_device->getBlockCount();
    return Ok<FSStatus>();
}

inline Result<void, FSStatus> FileSystem::configure(
    BlockDeviceReadFunc read_func, BlockDeviceProgFunc prog_func,
    BlockDeviceEraseFunc erase_func, BlockDeviceSyncFunc sync_func) {
    read_func_ = std::move(read_func);
    prog_func_ = std::move(prog_func);
    erase_func_ = std::move(erase_func);
    sync_func_ = std::move(sync_func);
    return Ok<FSStatus>();
}

inline Result<void, FSStatus> FileSystem::configureFromFlash(
    hal::IFlash*, uint32_t, size_t) {
    return Ok<FSStatus>();
}

inline Result<void, FSStatus> FileSystem::format(const FSConfig* config) {
    if (config) config_ = *config;
#ifndef USE_LITTLEFS
    detail::mockStorage().clear();
#endif
    return Ok<FSStatus>();
}

inline Result<void, FSStatus> FileSystem::mount() {
    if (mounted_) return Ok<FSStatus>();
    // Проверка: либо block device, либо callback-устройства, либо callback-функции
    bool has_callbacks = (read_func_ && prog_func_ && erase_func_);
    if (!block_device_ && !callback_device_ && !has_callbacks) {
        return Err<void, FSStatus>(FSStatus::NOT_MOUNTED);
    }
    mounted_ = true;
    stats_.total_blocks = config_.block_count > 0 ? config_.block_count : 256;
    return Ok<FSStatus>();
}

inline Result<void, FSStatus> FileSystem::unmount() {
    if (!mounted_) return Ok<FSStatus>();
    mounted_ = false;
    return Ok<FSStatus>();
}

inline Result<FileHandle, FSStatus> FileSystem::open(const char* path, FileMode mode) {
    if (!mounted_) return Err<FileHandle, FSStatus>(FSStatus::NOT_MOUNTED);
    if (!path) return Err<FileHandle, FSStatus>(FSStatus::INVALID_PATH);
    if (open_files_ >= MAX_OPEN_FILES) {
        return Err<FileHandle, FSStatus>(FSStatus::TOO_MANY_OPEN_FILES);
    }

    FileHandle handle;
    handle.is_open_ = true;
    handle.position_ = 0;
    handle.size_ = 0;
#ifndef USE_LITTLEFS
    handle.file_path_ = std::string(path);
#endif

#ifndef USE_LITTLEFS
    auto& storage = detail::mockStorage();
    std::string p(path);
    auto it = storage.find(p);
    bool exists = (it != storage.end());

    if (mode == FileMode::READ && !exists) {
        return Err<FileHandle, FSStatus>(FSStatus::NOT_FOUND);
    }
    if (exists) handle.size_ = it->second.data.size();
    if (mode == FileMode::APPEND || mode == FileMode::APPEND_CREATE) {
        handle.position_ = handle.size_;
    }
    if (mode == FileMode::WRITE || mode == FileMode::WRITE_CREATE) {
        // Create or clear file
        detail::MockFile f;
        f.is_directory = false;
        storage[p] = f;
        handle.size_ = 0;
        handle.position_ = 0;
    }
#endif
    open_files_++;
    stats_.read_count++;
    return Ok<FileHandle, FSStatus>(std::move(handle));
}

inline Result<void, FSStatus> FileSystem::close(FileHandle& handle) {
    if (!handle.isOpen()) return Ok<FSStatus>();
    handle.close();
    if (open_files_ > 0) open_files_--;
    return Ok<FSStatus>();
}

inline Result<void, FSStatus> FileSystem::remove(const char* path) {
    if (!mounted_) return Err<void, FSStatus>(FSStatus::NOT_MOUNTED);
#ifndef USE_LITTLEFS
    auto& storage = detail::mockStorage();
    auto it = storage.find(std::string(path));
    if (it == storage.end()) return Err<void, FSStatus>(FSStatus::NOT_FOUND);
    storage.erase(it);
    return Ok<FSStatus>();
#else
    (void)path;
    return Ok<FSStatus>();
#endif
}

inline Result<void, FSStatus> FileSystem::rename(const char* old_path, const char* new_path) {
    if (!mounted_) return Err<void, FSStatus>(FSStatus::NOT_MOUNTED);
#ifndef USE_LITTLEFS
    auto& storage = detail::mockStorage();
    auto it = storage.find(std::string(old_path));
    if (it == storage.end()) return Err<void, FSStatus>(FSStatus::NOT_FOUND);
    auto file = std::move(it->second);
    storage.erase(it);
    storage[std::string(new_path)] = std::move(file);
    return Ok<FSStatus>();
#else
    (void)old_path; (void)new_path;
    return Ok<FSStatus>();
#endif
}

inline Result<FileStats, FSStatus> FileSystem::stat(const char* path) {
    if (!mounted_) return Err<FileStats, FSStatus>(FSStatus::NOT_MOUNTED);
    if (!path || path[0] == '\0') return Err<FileStats, FSStatus>(FSStatus::INVALID_PATH);
#ifndef USE_LITTLEFS
    auto& storage = detail::mockStorage();
    auto it = storage.find(std::string(path));
    if (it == storage.end()) return Err<FileStats, FSStatus>(FSStatus::NOT_FOUND);
    FileStats st;
    st.type = it->second.is_directory ? FileType::DIRECTORY : FileType::FILE;
    st.size = it->second.data.size();
    return Ok<FileStats, FSStatus>(st);
#else
    (void)path;
    return Ok<FileStats, FSStatus>(FileStats{});
#endif
}

inline bool FileSystem::exists(const char* path) {
    if (!mounted_) return false;
#ifndef USE_LITTLEFS
    return detail::mockStorage().find(std::string(path)) != detail::mockStorage().end();
#else
    (void)path;
    return false;
#endif
}

inline Result<int, FSStatus> FileSystem::readFile(const char* path, void* buffer, size_t size) {
    if (!mounted_) return Err<int, FSStatus>(FSStatus::NOT_MOUNTED);
#ifndef USE_LITTLEFS
    auto& storage = detail::mockStorage();
    auto it = storage.find(std::string(path));
    if (it == storage.end()) return Err<int, FSStatus>(FSStatus::NOT_FOUND);
    if (it->second.is_directory) return Err<int, FSStatus>(FSStatus::IS_DIR);
    size_t file_size = it->second.data.size();
    size_t to_read = (size < file_size) ? size : file_size;
    std::memcpy(buffer, it->second.data.data(), to_read);
    stats_.read_count++;
    return Ok<int, FSStatus>(static_cast<int>(to_read));
#else
    (void)path; (void)buffer; (void)size;
    return Ok<int, FSStatus>(0);
#endif
}

inline Result<void, FSStatus> FileSystem::writeFile(const char* path, const void* buffer, size_t size) {
    if (!mounted_) return Err<void, FSStatus>(FSStatus::NOT_MOUNTED);
#ifndef USE_LITTLEFS
    // Проверка существования родительской директории
    std::string p(path);
    size_t lastSlash = p.find_last_of('/');
    if (lastSlash != std::string::npos && lastSlash > 0) {
        std::string parentDir = p.substr(0, lastSlash);
        auto& storage = detail::mockStorage();
        auto it = storage.find(parentDir);
        if (it == storage.end() || !it->second.is_directory) {
            return Err<void, FSStatus>(FSStatus::NOT_FOUND);
        }
    }
    detail::MockFile file;
    file.is_directory = false;
    file.data.assign(static_cast<const uint8_t*>(buffer),
                     static_cast<const uint8_t*>(buffer) + size);
    detail::mockStorage()[std::string(path)] = std::move(file);
    stats_.write_count++;
    return Ok<FSStatus>();
#else
    (void)path; (void)buffer; (void)size;
    return Ok<FSStatus>();
#endif
}

inline Result<void, FSStatus> FileSystem::mkdir(const char* path) {
    if (!mounted_) return Err<void, FSStatus>(FSStatus::NOT_MOUNTED);
#ifndef USE_LITTLEFS
    detail::MockFile dir;
    dir.is_directory = true;
    detail::mockStorage()[std::string(path)] = std::move(dir);
    stats_.total_dirs++;
    return Ok<FSStatus>();
#else
    (void)path;
    return Ok<FSStatus>();
#endif
}

inline Result<void, FSStatus> FileSystem::rmdir(const char* path) {
    if (!mounted_) return Err<void, FSStatus>(FSStatus::NOT_MOUNTED);
#ifndef USE_LITTLEFS
    auto& storage = detail::mockStorage();
    auto it = storage.find(std::string(path));
    if (it == storage.end()) return Err<void, FSStatus>(FSStatus::NOT_FOUND);
    if (!it->second.is_directory) return Err<void, FSStatus>(FSStatus::NOT_DIR);
    storage.erase(it);
    if (stats_.total_dirs > 0) stats_.total_dirs--;
    return Ok<FSStatus>();
#else
    (void)path;
    return Ok<FSStatus>();
#endif
}

inline Result<void, FSStatus> FileSystem::listdir(
    const char* path, std::function<void(const char*, bool)> callback) {
    if (!mounted_) return Err<void, FSStatus>(FSStatus::NOT_MOUNTED);
#ifndef USE_LITTLEFS
    std::string p(path);
    if (!p.empty() && p.back() != '/') p += '/';
    auto& storage = detail::mockStorage();
    for (auto& [full_path, file] : storage) {
        if (full_path.find(p) == 0 && full_path != p) {
            std::string name = full_path.substr(p.length());
            if (name.find('/') == std::string::npos) {
                callback(name.c_str(), file.is_directory);
            }
        }
    }
    return Ok<FSStatus>();
#else
    (void)path; (void)callback;
    return Ok<FSStatus>();
#endif
}

inline Result<DirHandle, FSStatus> FileSystem::opendir(const char*) {
    return Ok<DirHandle, FSStatus>(DirHandle{});
}

inline Result<void, FSStatus> FileSystem::closedir(DirHandle&) {
    return Ok<FSStatus>();
}

inline Result<DirEntry, FSStatus> FileSystem::readdir(DirHandle&) {
    return Err<DirEntry, FSStatus>(FSStatus::OK);
}

inline Result<void, FSStatus> FileSystem::walk(const char*, std::function<void(const char*, const char**, size_t, const char**, size_t)>) {
    return Ok<FSStatus>();
}

inline Result<int, FSStatus> FileSystem::setattr(const char*, const char*, const void*, size_t) {
    return Ok<int, FSStatus>(0);
}

inline Result<int, FSStatus> FileSystem::getattr(const char*, const char*, void*, size_t) {
    return Ok<int, FSStatus>(0);
}

inline FSStats FileSystem::getStats() const {
    FSStats stats = stats_;
#ifndef USE_LITTLEFS
    stats.total_files = 0;
    stats.total_dirs = 0;
    for (auto& [path, file] : detail::mockStorage()) {
        if (file.is_directory) stats.total_dirs++;
        else stats.total_files++;
    }
    stats.used_blocks = stats.total_files + stats.total_dirs;
    stats.free_blocks = (stats.total_blocks > stats.used_blocks) ? 
                        (stats.total_blocks - stats.used_blocks) : 0;
#endif
    return stats;
}

inline WearStats FileSystem::getWearStats() const {
    return wear_stats_;
}

inline Result<void, FSStatus> FileSystem::sync() {
    if (!mounted_) return Err<void, FSStatus>(FSStatus::NOT_MOUNTED);
    if (block_device_) {
        auto res = block_device_->sync();
        if (!res.isOk()) return Err<void, FSStatus>(FSStatus::ERROR);
    }
    return Ok<FSStatus>();
}

inline bool FileSystem::checkAndRepair() { return true; }

inline int FileSystem::findFreeFileSlot() const {
    return static_cast<int>(open_files_ < MAX_OPEN_FILES ? open_files_ : -1);
}

inline void FileSystem::updateStats() {
#ifndef USE_LITTLEFS
    stats_.total_files = 0;
    stats_.total_dirs = 0;
    for (auto& [path, file] : detail::mockStorage()) {
        if (file.is_directory) stats_.total_dirs++;
        else stats_.total_files++;
    }
#endif
}

inline Result<void, FSStatus> FileSystem::initBlockDevice() {
    if (block_device_) {
        auto res = block_device_->init();
        if (!res.isOk()) return Err<void, FSStatus>(FSStatus::ERROR);
    }
    return Ok<FSStatus>();
}

inline Result<void, FSStatus> FileSystem::deinitBlockDevice() {
    if (block_device_) {
        auto res = block_device_->deinit();
        if (!res.isOk()) return Err<void, FSStatus>(FSStatus::ERROR);
    }
    return Ok<FSStatus>();
}

// ============================================================================
// Utility functions
// ============================================================================

inline const char* fsStatusToString(FSStatus status) {
    switch (status) {
        case FSStatus::OK: return "OK";
        case FSStatus::ERROR: return "ERROR";
        case FSStatus::NOT_FOUND: return "NOT_FOUND";
        case FSStatus::EXISTS: return "EXISTS";
        case FSStatus::NOT_DIR: return "NOT_DIR";
        case FSStatus::IS_DIR: return "IS_DIR";
        case FSStatus::NOT_EMPTY: return "NOT_EMPTY";
        case FSStatus::NO_SPACE: return "NO_SPACE";
        case FSStatus::NOT_MOUNTED: return "NOT_MOUNTED";
        case FSStatus::INVALID_PATH: return "INVALID_PATH";
        case FSStatus::NAME_TOO_LONG: return "NAME_TOO_LONG";
        case FSStatus::TOO_MANY_OPEN_FILES: return "TOO_MANY_OPEN_FILES";
        case FSStatus::CORRUPT: return "CORRUPT";
        case FSStatus::NOT_SUPPORTED: return "NOT_SUPPORTED";
        default: return "UNKNOWN";
    }
}

} // namespace filesystem
} // namespace mka

#endif // FILE_SYSTEM_LITTLEFS_HPP
