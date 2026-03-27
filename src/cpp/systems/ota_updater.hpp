/**
 * @file ota_updater.hpp
 * @brief Over-The-Air firmware update system for MKA
 *
 * Features:
 * - Dual-bank firmware (A/B slots)
 * - SHA256 integrity check
 * - Rollback on failure
 * - Golden image backup
 *
 * Flash Memory Layout:
 * ┌─────────────────────────────────────────────────────────────┐
 * │ Bootloader (16 KB)                                          │
 * │ - Integrity check                                           │
 * │ - Active slot selection                                     │
 * │ - Slot switching                                            │
 * ├─────────────────────────────────────────────────────────────┤
 * │ Image A (Application, ~200 KB)                              │
 * │ - Current firmware                                          │
 * │ - Metadata (version, CRC, signature)                        │
 * ├─────────────────────────────────────────────────────────────┤
 * │ Image B (Application, ~200 KB)                              │
 * │ - New firmware (downloading)                                │
 * │ - Metadata (version, CRC, signature)                        │
 * ├─────────────────────────────────────────────────────────────┤
 * │ Configuration & Storage (~64 KB)                            │
 * │ - System parameters                                         │
 * │ - Logs                                                      │
 * └─────────────────────────────────────────────────────────────┘
 */

#ifndef OTA_UPDATER_HPP
#define OTA_UPDATER_HPP

#include <cstdint>
#include <cstddef>
#include <functional>

#include "../hal/hal_full.hpp"
#include "../utils/result.hpp"
#include "file_system.hpp"

namespace mka {
namespace ota {

// ============================================================================
// Constants and Types
// ============================================================================

/// Magic number for firmware image
constexpr uint32_t IMAGE_MAGIC = 0x304B414D;  // "MAK0" in little-endian

/// Firmware header version
constexpr uint8_t HEADER_VERSION = 0x01;

/// Maximum firmware image size (bytes)
constexpr size_t MAX_IMAGE_SIZE = 256 * 1024;  // 256 KB

/// Header size (bytes)
constexpr size_t HEADER_SIZE = 64;

/// SHA256 hash size (bytes)
constexpr size_t SHA256_SIZE = 32;

/// Firmware header magic
constexpr uint8_t MAGIC_BYTES[8] = {'M', 'K', 'A', '_', 'F', 'W', '0', '1'};

/// Firmware update state
enum class OTAState : uint8_t {
    IDLE = 0,           ///< Idle, no update in progress
    RECEIVING = 1,      ///< Receiving firmware data
    VERIFYING = 2,      ///< Verifying firmware
    READY_TO_APPLY = 3, ///< Ready to apply (reboot required)
    APPLYING = 4,       ///< Applying update
    COMPLETE = 5,       ///< Update complete
    ERROR = 6           ///< Error occurred
};

/// OTA error codes
enum class OTAError : uint8_t {
    NONE = 0,
    INVALID_HEADER = 1,
    CRC_MISMATCH = 2,
    SIGNATURE_INVALID = 3,
    FLASH_WRITE_ERROR = 4,
    ERASE_ERROR = 5,
    VERSION_ROLLBACK = 6,
    SIZE_MISMATCH = 7,
    TIMEOUT = 8,
    ABORTED = 9,
    NOT_ENOUGH_SPACE = 10,
    ALREADY_ACTIVE = 11
};

/// Firmware slot (A/B bank)
enum class ImageSlot : uint8_t {
    SLOT_A = 0,
    SLOT_B = 1
};

/// Image flags
enum class ImageFlags : uint8_t {
    NONE = 0,
    VALID = 0x01,       ///< Image is valid
    ACTIVE = 0x02,      ///< Image is active (running)
    VERIFIED = 0x04,    ///< Image is verified
    NEW = 0x08,         ///< New image, never booted
    CORRUPT = 0x10      ///< Image is corrupt
};

/// Firmware version structure
struct FirmwareVersion {
    uint8_t major = 0;
    uint8_t minor = 0;
    uint8_t patch = 0;
    uint32_t build = 0;

    /// Compare versions
    bool operator<(const FirmwareVersion& other) const {
        if (major != other.major) return major < other.major;
        if (minor != other.minor) return minor < other.minor;
        if (patch != other.patch) return patch < other.patch;
        return build < other.build;
    }

    bool operator>(const FirmwareVersion& other) const {
        return other < *this;
    }

    bool operator<=(const FirmwareVersion& other) const {
        return !(*this > other);
    }

    bool operator>=(const FirmwareVersion& other) const {
        return !(*this < other);
    }

    /// Convert to string (returns pointer to static buffer)
    const char* toString() const {
        static char buffer[32];
        // Simple conversion - in real code use snprintf
        buffer[0] = '0' + major;
        buffer[1] = '.';
        buffer[2] = '0' + minor;
        buffer[3] = '.';
        buffer[4] = '0' + patch;
        buffer[5] = '.';
        buffer[6] = '0' + (build / 1000) % 10;
        buffer[7] = '0' + (build / 100) % 10;
        buffer[8] = '0' + (build / 10) % 10;
        buffer[9] = '0' + build % 10;
        buffer[10] = '\0';
        return buffer;
    }
};

/// Firmware image header (64 bytes)
struct ImageHeader {
    uint8_t magic[8] = MAGIC_BYTES;     ///< Magic number
    FirmwareVersion version;            ///< Firmware version
    uint32_t timestamp = 0;             ///< Unix timestamp
    uint32_t size = 0;                  ///< Image size (excluding header)
    uint32_t crc32 = 0;                 ///< CRC32 of image
    uint8_t flags = static_cast<uint8_t>(ImageFlags::NONE);
    uint8_t git_hash[20] = {};          ///< Git commit hash
    uint8_t board_rev = 0;              ///< Board revision
    uint8_t min_hardware = 0;           ///< Minimum hardware version
    uint8_t reserved[12] = {};          ///< Reserved
    uint8_t sha256[SHA256_SIZE] = {};   ///< SHA256 hash

    /// Check if header is valid
    bool isValid() const {
        for (size_t i = 0; i < 8; i++) {
            if (magic[i] != MAGIC_BYTES[i]) return false;
        }
        return true;
    }

    /// Get flags as enum
    ImageFlags getFlags() const {
        return static_cast<ImageFlags>(flags);
    }

    /// Check if image is valid
    bool isFlagSet(ImageFlags flag) const {
        return (flags & static_cast<uint8_t>(flag)) != 0;
    }
};

static_assert(sizeof(ImageHeader) == HEADER_SIZE, "ImageHeader must be 64 bytes");

/// OTA statistics
struct OTAStats {
    uint32_t updates_started = 0;
    uint32_t updates_completed = 0;
    uint32_t updates_failed = 0;
    uint32_t rollbacks = 0;
    uint32_t bytes_received = 0;
    uint32_t last_update_timestamp = 0;
    OTAError last_error = OTAError::NONE;
};

/// OTA configuration
struct OTAConfig {
    ImageSlot slot_a_addr = ImageSlot::SLOT_A;  ///< Slot A address/ID
    ImageSlot slot_b_addr = ImageSlot::SLOT_B;  ///< Slot B address/ID
    size_t slot_size = MAX_IMAGE_SIZE;          ///< Slot size
    bool verify_signature = true;                ///< Verify signatures
    bool auto_rollback = true;                   ///< Auto rollback on failure
    uint32_t watchdog_timeout_ms = 5000;         ///< Watchdog timeout
};

// ============================================================================
// CRC32 Calculation
// ============================================================================

/**
 * @brief CRC32 calculation (polynomial 0xEDB88320)
 */
class CRC32 {
public:
    /// Initialize CRC table
    static void initTable();

    /// Calculate CRC32
    static uint32_t calculate(const void* data, size_t size);

    /// Verify CRC32
    static bool verify(const void* data, size_t size, uint32_t expected);

private:
    static uint32_t table[256];
    static bool tableInitialized;
};

// ============================================================================
// SHA256 (simplified interface)
// ============================================================================

/**
 * @brief SHA256 hash calculation
 *
 * Note: Full implementation is large. For production use, consider:
 * - mbedTLS
 * - WolfSSL
 * - TinyCrypt
 */
class SHA256 {
public:
    /// Calculate SHA256 hash
    static Result<void, OTAError> calculate(
        const void* data,
        size_t size,
        uint8_t* output  // Must be SHA256_SIZE bytes
    );

    /// Verify hash
    static bool verify(
        const void* data,
        size_t size,
        const uint8_t* expected  // SHA256_SIZE bytes
    );
};

// ============================================================================
// OTA Updater
// ============================================================================

/**
 * @brief Over-The-Air firmware update manager
 *
 * Supports dual-bank updates with automatic rollback.
 */
class OTAUpdater {
public:
    /// Flash write callback
    using FlashWriteFunc = std::function<Result<void, OTAError>(
        ImageSlot slot,
        uint32_t offset,
        const void* data,
        size_t size
    )>;

    /// Flash read callback
    using FlashReadFunc = std::function<Result<void, OTAError>(
        ImageSlot slot,
        uint32_t offset,
        void* buffer,
        size_t size
    )>;

    /// Flash erase callback
    using FlashEraseFunc = std::function<Result<void, OTAError>(
        ImageSlot slot
    )>;

    /// Reboot callback
    using RebootFunc = std::function<void()>;

    /// Progress callback
    using ProgressFunc = std::function<void(uint8_t percent, OTAState state)>;

    OTAUpdater();
    ~OTAUpdater();

    /**
     * @brief Initialize OTA updater
     * @param config Configuration
     * @param write_func Flash write function
     * @param read_func Flash read function
     * @param erase_func Flash erase function
     * @param reboot_func System reboot function
     * @return Status
     */
    Result<void, OTAError> init(
        const OTAConfig& config,
        FlashWriteFunc write_func,
        FlashReadFunc read_func,
        FlashEraseFunc erase_func,
        RebootFunc reboot_func = nullptr
    );

    /**
     * @brief Initialize with file system backend
     * @param fs File system interface
     * @param slot_a_path Path to slot A image file
     * @param slot_b_path Path to slot B image file
     * @param reboot_func System reboot function
     * @return Status
     */
    Result<void, OTAError> initFromFS(
        filesystem::FileSystem* fs,
        const char* slot_a_path,
        const char* slot_b_path,
        RebootFunc reboot_func = nullptr
    );

    /**
     * @brief Start firmware update
     * @param image_size Expected image size (excluding header)
     * @return Status
     */
    Result<void, OTAError> startUpdate(size_t image_size);

    /**
     * @brief Receive firmware data chunk
     * @param data Data buffer
     * @param size Data size
     * @param bytes_written Bytes actually written (output)
     * @return Status
     */
    Result<void, OTAError> receiveData(
        const void* data,
        size_t size,
        size_t* bytes_written = nullptr
    );

    /**
     * @brief Finish firmware update
     * @return Status
     */
    Result<void, OTAError> finishUpdate();

    /**
     * @brief Abort firmware update
     * @return Status
     */
    Result<void, OTAError> abortUpdate();

    /**
     * @brief Apply firmware update (requires reboot)
     * @return Status
     */
    Result<void, OTAError> applyUpdate();

    /**
     * @brief Verify current firmware
     * @return Status
     */
    Result<void, OTAError> verifyCurrentFirmware();

    /**
     * @brief Rollback to previous firmware
     * @return Status
     */
    Result<void, OTAError> rollback();

    /**
     * @brief Get current state
     */
    OTAState getState() const { return state_; }

    /**
     * @brief Get last error
     */
    OTAError getLastError() const { return last_error_; }

    /**
     * @brief Get current slot
     */
    ImageSlot getCurrentSlot() const { return current_slot_; }

    /**
     * @brief Get inactive slot
     */
    ImageSlot getInactiveSlot() const;

    /**
     * @brief Get statistics
     */
    OTAStats getStats() const { return stats_; }

    /**
     * @brief Get current firmware version
     */
    FirmwareVersion getCurrentVersion() const;

    /**
     * @brief Get pending firmware version
     */
    FirmwareVersion getPendingVersion() const;

    /**
     * @brief Check if update is pending
     */
    bool isUpdatePending() const {
        return state_ == OTAState::READY_TO_APPLY ||
               state_ == OTAState::APPLYING;
    }

    /**
     * @brief Set progress callback
     */
    void setProgressCallback(ProgressFunc callback) {
        progress_callback_ = callback;
    }

    /**
     * @brief Mark current image as booted successfully
     * @return Status
     */
    Result<void, OTAError> markBootSuccessful();

    /**
     * @brief Check if this is first boot after update
     */
    bool isFirstBoot() const;

protected:
    // Internal methods
    Result<void, OTAError> validateHeader(const ImageHeader& header);
    Result<void, OTAError> verifyImage(ImageSlot slot);
    Result<void, OTAError> switchSlots();
    void updateProgress(uint8_t percent);
    void setState(OTAState state);

    OTAConfig config_;
    OTAState state_ = OTAState::IDLE;
    OTAError last_error_ = OTAError::NONE;
    ImageSlot current_slot_ = ImageSlot::SLOT_A;
    ImageSlot target_slot_ = ImageSlot::SLOT_B;

    size_t bytes_received_ = 0;
    size_t expected_size_ = 0;
    ImageHeader current_header_;

    // Callbacks
    FlashWriteFunc write_func_;
    FlashReadFunc read_func_;
    FlashEraseFunc erase_func_;
    RebootFunc reboot_func_;
    ProgressFunc progress_callback_;

    // File system backend (optional)
    filesystem::FileSystem* fs_ = nullptr;
    const char* slot_a_path_ = nullptr;
    const char* slot_b_path_ = nullptr;

    // Statistics
    OTAStats stats_;
};

// ============================================================================
// Bootloader Interface
// ============================================================================

/**
 * @brief Bootloader communication interface
 *
 * Used by application to communicate with bootloader
 * for slot switching and status reporting.
 */
class BootloaderInterface {
public:
    /// Bootloader command
    enum class Command : uint8_t {
        GET_STATUS = 0x01,
        SWITCH_SLOT = 0x02,
        MARK_VALID = 0x03,
        MARK_INVALID = 0x04,
        GET_VERSION = 0x05,
        REBOOT = 0x06
    };

    /// Bootloader status
    struct Status {
        ImageSlot active_slot = ImageSlot::SLOT_A;
        ImageSlot valid_slot = ImageSlot::SLOT_A;
        bool pending_switch = false;
        uint8_t boot_count = 0;
        uint32_t last_switch_timestamp = 0;
    };

    /**
     * @brief Get bootloader status
     * @return Status or error
     */
    static Result<Status, OTAError> getStatus();

    /**
     * @brief Request slot switch on next boot
     * @return Status
     */
    static Result<void, OTAError> requestSlotSwitch();

    /**
     * @brief Mark current slot as valid
     * @return Status
     */
    static Result<void, OTAError> markSlotValid();

    /**
     * @brief Mark current slot as invalid (force rollback)
     * @return Status
     */
    static Result<void, OTAError> markSlotInvalid();

    /**
     * @brief Reboot into bootloader
     * @return Status
     */
    static Result<void, OTAError> rebootToBootloader();

    /**
     * @brief Check if running from bootloader
     */
    static bool isInBootloader();
};

// ============================================================================
// Convenience Functions
// ============================================================================

/**
 * @brief Convert OTAState to string
 */
const char* otaStateToString(OTAState state);

/**
 * @brief Convert OTAError to string
 */
const char* otaErrorToString(OTAError error);

/**
 * @brief Convert ImageSlot to string
 */
const char* imageSlotToString(ImageSlot slot);

} // namespace ota
} // namespace mka

#endif // OTA_UPDATER_HPP
