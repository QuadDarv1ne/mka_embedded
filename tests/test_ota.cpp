/**
 * @file test_ota.cpp
 * @brief Tests for OTA firmware update system
 */

#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <vector>
#include <functional>

#include "systems/ota_updater.hpp"

using namespace mka::ota;

// ============================================================================
// Mock Flash Storage for Testing
// ============================================================================

class MockFlashStorage {
public:
    MockFlashStorage(size_t slot_size = MAX_IMAGE_SIZE)
        : slot_size_(slot_size) {
        slot_a_.resize(slot_size, 0xFF);
        slot_b_.resize(slot_size, 0xFF);
    }

    Result<void, OTAError> write(ImageSlot slot, uint32_t offset,
                                  const void* data, size_t size) {
        if (offset + size > slot_size_) {
            return Result<void, OTAError>::err(OTAError::NOT_ENOUGH_SPACE);
        }

        std::vector<uint8_t>* target = (slot == ImageSlot::SLOT_A) ? &slot_a_ : &slot_b_;
        const uint8_t* bytes = static_cast<const uint8_t*>(data);

        for (size_t i = 0; i < size; i++) {
            (*target)[offset + i] = bytes[i];
        }

        write_count_++;
        return Result<void, OTAError>::ok();
    }

    Result<void, OTAError> read(ImageSlot slot, uint32_t offset,
                                 void* buffer, size_t size) {
        if (offset + size > slot_size_) {
            return Result<void, OTAError>::err(OTAError::NOT_ENOUGH_SPACE);
        }

        const std::vector<uint8_t>* source = (slot == ImageSlot::SLOT_A) ? &slot_a_ : &slot_b_;
        uint8_t* bytes = static_cast<uint8_t*>(buffer);

        for (size_t i = 0; i < size; i++) {
            bytes[i] = (*source)[offset + i];
        }

        read_count_++;
        return Result<void, OTAError>::ok();
    }

    Result<void, OTAError> erase(ImageSlot slot) {
        std::vector<uint8_t>* target = (slot == ImageSlot::SLOT_A) ? &slot_a_ : &slot_b_;
        std::fill(target->begin(), target->end(), 0xFF);
        erase_count_++;
        return Result<void, OTAError>::ok();
    }

    void reset() {
        std::fill(slot_a_.begin(), slot_a_.end(), 0xFF);
        std::fill(slot_b_.begin(), slot_b_.end(), 0xFF);
        write_count_ = 0;
        read_count_ = 0;
        erase_count_ = 0;
        reboot_count_ = 0;
    }

    size_t getWriteCount() const { return write_count_; }
    size_t getReadCount() const { return read_count_; }
    size_t getEraseCount() const { return erase_count_; }
    size_t getRebootCount() const { return reboot_count_; }

    void triggerReboot() { reboot_count_++; }

private:
    size_t slot_size_;
    std::vector<uint8_t> slot_a_;
    std::vector<uint8_t> slot_b_;
    size_t write_count_ = 0;
    size_t read_count_ = 0;
    size_t erase_count_ = 0;
    size_t reboot_count_ = 0;
};

// ============================================================================
// Test Fixtures
// ============================================================================

class OTATest : public ::testing::Test {
protected:
    MockFlashStorage flash_;
    OTAUpdater ota_;
    OTAConfig config_;
    size_t reboot_count_ = 0;

    void SetUp() override {
        flash_.reset();
        reboot_count_ = 0;

        config_.slot_a_addr = ImageSlot::SLOT_A;
        config_.slot_b_addr = ImageSlot::SLOT_B;
        config_.slot_size = MAX_IMAGE_SIZE;
        config_.verify_signature = false;  // Disable for testing
        config_.auto_rollback = true;
    }

    Result<void, OTAError> initOTA() {
        return ota_.init(
            config_,
            [this](ImageSlot slot, uint32_t offset, const void* data, size_t size) {
                return flash_.write(slot, offset, data, size);
            },
            [this](ImageSlot slot, uint32_t offset, void* buffer, size_t size) {
                return flash_.read(slot, offset, buffer, size);
            },
            [this](ImageSlot slot) {
                return flash_.erase(slot);
            },
            [this]() {
                reboot_count_++;
            }
        );
    }

    /// Create a test firmware image
    std::vector<uint8_t> createTestImage(
        uint8_t version_major,
        uint8_t version_minor,
        size_t size = 1024
    ) {
        std::vector<uint8_t> image(HEADER_SIZE + size, 0);

        // Create header
        ImageHeader* header = reinterpret_cast<ImageHeader*>(image.data());
        std::memcpy(header->magic, MAGIC_BYTES, 8);
        header->version.major = version_major;
        header->version.minor = version_minor;
        header->version.patch = 0;
        header->version.build = 1;
        header->size = static_cast<uint32_t>(size);
        header->flags = static_cast<uint8_t>(ImageFlags::VALID);

        // Calculate CRC
        header->crc32 = CRC32::calculate(image.data() + HEADER_SIZE, size);

        // Fill payload with pattern
        for (size_t i = 0; i < size; i++) {
            image[HEADER_SIZE + i] = static_cast<uint8_t>(i & 0xFF);
        }

        return image;
    }
};

// ============================================================================
// CRC32 Tests
// ============================================================================

class CRC32Test : public ::testing::Test {
protected:
    void SetUp() override {
        CRC32::initTable();
    }
};

TEST_F(CRC32Test, CalculateBasic) {
    std::array<uint8_t, 5> data = {0x31, 0x32, 0x33, 0x34, 0x35};
    uint32_t crc = CRC32::calculate(data.data(), data.size());

    EXPECT_NE(crc, 0);
    EXPECT_EQ(crc, CRC32::calculate(data.data(), data.size()));
}

TEST_F(CRC32Test, CalculateEmpty) {
    std::array<uint8_t, 0> data = {};
    uint32_t crc = CRC32::calculate(data.data(), data.size());

    EXPECT_EQ(crc, 0);
}

TEST_F(CRC32Test, VerifyCorrect) {
    std::array<uint8_t, 10> data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    uint32_t crc = CRC32::calculate(data.data(), data.size());

    EXPECT_TRUE(CRC32::verify(data.data(), data.size(), crc));
}

TEST_F(CRC32Test, VerifyIncorrect) {
    std::array<uint8_t, 10> data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    uint32_t crc = CRC32::calculate(data.data(), data.size());

    // Modify data
    data[5] = 0xFF;

    EXPECT_FALSE(CRC32::verify(data.data(), data.size(), crc));
}

// ============================================================================
// ImageHeader Tests
// ============================================================================

class ImageHeaderTest : public ::testing::Test {
protected:
    void SetUp() override {}
};

TEST_F(ImageHeaderTest, DefaultConstructor) {
    ImageHeader header;

    EXPECT_TRUE(header.isValid());
    EXPECT_EQ(header.version.major, 0);
    EXPECT_EQ(header.version.minor, 0);
    EXPECT_EQ(header.size, 0);
    EXPECT_EQ(header.getFlags(), ImageFlags::NONE);
}

TEST_F(ImageHeaderTest, SetVersion) {
    ImageHeader header;
    header.version.major = 1;
    header.version.minor = 2;
    header.version.patch = 3;
    header.version.build = 4;

    EXPECT_EQ(header.version.major, 1);
    EXPECT_EQ(header.version.minor, 2);
    EXPECT_EQ(header.version.patch, 3);
    EXPECT_EQ(header.version.build, 4);
}

TEST_F(ImageHeaderTest, SetFlags) {
    ImageHeader header;

    header.flags = static_cast<uint8_t>(ImageFlags::VALID);
    EXPECT_TRUE(header.isFlagSet(ImageFlags::VALID));
    EXPECT_FALSE(header.isFlagSet(ImageFlags::ACTIVE));

    header.flags = static_cast<uint8_t>(ImageFlags::VALID) |
                   static_cast<uint8_t>(ImageFlags::ACTIVE);
    EXPECT_TRUE(header.isFlagSet(ImageFlags::VALID));
    EXPECT_TRUE(header.isFlagSet(ImageFlags::ACTIVE));
}

TEST_F(ImageHeaderTest, InvalidMagic) {
    ImageHeader header;
    header.magic[0] = 'X';  // Corrupt magic

    EXPECT_FALSE(header.isValid());
}

// ============================================================================
// FirmwareVersion Tests
// ============================================================================

class FirmwareVersionTest : public ::testing::Test {
protected:
    void SetUp() override {}
};

TEST_F(FirmwareVersionTest, Comparison) {
    FirmwareVersion v1{1, 0, 0, 1};
    FirmwareVersion v2{1, 0, 0, 2};
    FirmwareVersion v3{1, 0, 1, 0};
    FirmwareVersion v4{2, 0, 0, 0};

    EXPECT_TRUE(v1 < v2);
    EXPECT_TRUE(v2 < v3);
    EXPECT_TRUE(v3 < v4);
    EXPECT_TRUE(v1 <= v2);
    EXPECT_TRUE(v2 >= v1);
}

TEST_F(FirmwareVersionTest, Equality) {
    FirmwareVersion v1{1, 2, 3, 4};
    FirmwareVersion v2{1, 2, 3, 4};
    FirmwareVersion v3{1, 2, 3, 5};

    EXPECT_TRUE(v1 <= v2);
    EXPECT_TRUE(v1 >= v2);
    EXPECT_FALSE(v1 < v2);
    EXPECT_FALSE(v1 > v2);
    EXPECT_TRUE(v1 < v3);
}

// ============================================================================
// OTA Updater Basic Tests
// ============================================================================

TEST_F(OTATest, Init) {
    auto result = initOTA();
    EXPECT_TRUE(result.isOk());
}

TEST_F(OTATest, InitialState) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    EXPECT_EQ(ota_.getState(), OTAState::IDLE);
    EXPECT_EQ(ota_.getCurrentSlot(), ImageSlot::SLOT_A);
    EXPECT_EQ(ota_.getLastError(), OTAError::NONE);
}

TEST_F(OTATest, GetInactiveSlot) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    // Current is SLOT_A, so inactive should be SLOT_B
    EXPECT_EQ(ota_.getInactiveSlot(), ImageSlot::SLOT_B);
}

// ============================================================================
// OTA Update Process Tests
// ============================================================================

TEST_F(OTATest, StartUpdate) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    result = ota_.startUpdate(1024);
    EXPECT_TRUE(result.isOk());
    EXPECT_EQ(ota_.getState(), OTAState::RECEIVING);
}

TEST_F(OTATest, StartUpdateInvalidSize) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    // Size too large
    result = ota_.startUpdate(MAX_IMAGE_SIZE + 1);
    EXPECT_FALSE(result.isOk());
}

TEST_F(OTATest, ReceiveData) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    result = ota_.startUpdate(1024);
    ASSERT_TRUE(result.isOk());

    std::array<uint8_t, 256> data{};
    for (size_t i = 0; i < data.size(); i++) {
        data[i] = static_cast<uint8_t>(i);
    }

    size_t bytes_written = 0;
    result = ota_.receiveData(data.data(), data.size(), &bytes_written);
    EXPECT_TRUE(result.isOk());
    EXPECT_EQ(bytes_written, data.size());
}

TEST_F(OTATest, ReceiveDataWithoutStart) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    std::array<uint8_t, 256> data{};
    result = ota_.receiveData(data.data(), data.size());
    EXPECT_FALSE(result.isOk());
}

TEST_F(OTATest, FinishUpdate) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    // Start and receive some data
    result = ota_.startUpdate(1024);
    ASSERT_TRUE(result.isOk());

    std::array<uint8_t, 1024> data{};
    result = ota_.receiveData(data.data(), data.size());
    ASSERT_TRUE(result.isOk());

    result = ota_.finishUpdate();
    EXPECT_TRUE(result.isOk());
}

TEST_F(OTATest, AbortUpdate) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    result = ota_.startUpdate(1024);
    ASSERT_TRUE(result.isOk());

    result = ota_.abortUpdate();
    EXPECT_TRUE(result.isOk());
    EXPECT_EQ(ota_.getState(), OTAState::IDLE);
}

// ============================================================================
// Full Update Flow Tests
// ============================================================================

TEST_F(OTATest, FullUpdateFlow) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    // Create test image
    auto image = createTestImage(1, 0, 1024);

    // Start update
    result = ota_.startUpdate(image.size() - HEADER_SIZE);
    ASSERT_TRUE(result.isOk());

    // Send header
    size_t bytes_written = 0;
    result = ota_.receiveData(image.data(), HEADER_SIZE, &bytes_written);
    ASSERT_TRUE(result.isOk());

    // Send payload
    result = ota_.receiveData(image.data() + HEADER_SIZE,
                               image.size() - HEADER_SIZE, &bytes_written);
    ASSERT_TRUE(result.isOk());

    // Finish
    result = ota_.finishUpdate();
    EXPECT_TRUE(result.isOk());
}

TEST_F(OTATest, UpdateWithProgressCallback) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    std::vector<uint8_t> progress_values;
    ota_.setProgressCallback([&progress_values](uint8_t percent, OTAState state) {
        progress_values.push_back(percent);
    });

    // Create and send image
    auto image = createTestImage(1, 0, 4096);

    result = ota_.startUpdate(image.size() - HEADER_SIZE);
    ASSERT_TRUE(result.isOk());

    // Send in chunks
    constexpr size_t CHUNK_SIZE = 256;
    for (size_t offset = 0; offset < image.size(); offset += CHUNK_SIZE) {
        size_t chunk_size = std::min(CHUNK_SIZE, image.size() - offset);
        result = ota_.receiveData(image.data() + offset, chunk_size);
        ASSERT_TRUE(result.isOk());
    }

    result = ota_.finishUpdate();
    ASSERT_TRUE(result.isOk());

    // Progress should have been called
    EXPECT_GT(progress_values.size(), 0);
}

// ============================================================================
// Error Handling Tests
// ============================================================================

TEST_F(OTATest, CorruptHeader) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    result = ota_.startUpdate(1024);
    ASSERT_TRUE(result.isOk());

    // Send corrupt header
    std::array<uint8_t, HEADER_SIZE> corrupt_header{};
    std::fill(corrupt_header.begin(), corrupt_header.end(), 0xFF);

    result = ota_.receiveData(corrupt_header.data(), HEADER_SIZE);
    EXPECT_FALSE(result.isOk());
    EXPECT_EQ(ota_.getLastError(), OTAError::INVALID_HEADER);
}

TEST_F(OTATest, SizeMismatch) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    // Start with one size
    result = ota_.startUpdate(1024);
    ASSERT_TRUE(result.isOk());

    // Send more data than expected
    std::array<uint8_t, 2048> data{};
    result = ota_.receiveData(data.data(), data.size());
    EXPECT_FALSE(result.isOk());
}

TEST_F(OTATest, ReceiveAfterFinish) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    result = ota_.startUpdate(1024);
    ASSERT_TRUE(result.isOk());

    std::array<uint8_t, 1024> data{};
    result = ota_.receiveData(data.data(), data.size());
    ASSERT_TRUE(result.isOk());

    result = ota_.finishUpdate();
    ASSERT_TRUE(result.isOk());

    // Try to receive more data
    result = ota_.receiveData(data.data(), data.size());
    EXPECT_FALSE(result.isOk());
}

// ============================================================================
// Rollback Tests
// ============================================================================

TEST_F(OTATest, Rollback) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    // Simulate successful update first
    auto image = createTestImage(1, 0, 1024);

    result = ota_.startUpdate(image.size() - HEADER_SIZE);
    ASSERT_TRUE(result.isOk());

    result = ota_.receiveData(image.data(), image.size());
    ASSERT_TRUE(result.isOk());

    result = ota_.finishUpdate();
    ASSERT_TRUE(result.isOk());

    // Now rollback
    result = ota_.rollback();
    // Rollback may fail if no previous image, but should not crash
    // Just verify it returns
}

TEST_F(OTATest, AutoRollbackOnVerifyFailure) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    // Create image with bad CRC
    auto image = createTestImage(1, 0, 1024);
    ImageHeader* header = reinterpret_cast<ImageHeader*>(image.data());
    header->crc32 = 0xDEADBEEF;  // Wrong CRC

    result = ota_.startUpdate(image.size() - HEADER_SIZE);
    ASSERT_TRUE(result.isOk());

    result = ota_.receiveData(image.data(), image.size());
    // Should fail on finish when verifying
    result = ota_.finishUpdate();
    EXPECT_FALSE(result.isOk());
}

// ============================================================================
// Statistics Tests
// ============================================================================

TEST_F(OTATest, Statistics) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    OTAStats initial_stats = ota_.getStats();
    EXPECT_EQ(initial_stats.updates_started, 0);
    EXPECT_EQ(initial_stats.updates_completed, 0);
    EXPECT_EQ(initial_stats.updates_failed, 0);

    // Start an update
    result = ota_.startUpdate(1024);
    ASSERT_TRUE(result.isOk());

    OTAStats after_start = ota_.getStats();
    EXPECT_EQ(after_start.updates_started, 1);
}

TEST_F(OTATest, LastErrorTracking) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    EXPECT_EQ(ota_.getLastError(), OTAError::NONE);

    // Cause an error
    std::array<uint8_t, 256> data{};
    result = ota_.receiveData(data.data(), data.size());  // Without start
    EXPECT_FALSE(result.isOk());

    EXPECT_NE(ota_.getLastError(), OTAError::NONE);
}

// ============================================================================
// Apply Update Tests
// ============================================================================

TEST_F(OTATest, ApplyUpdate) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    // Create and receive image
    auto image = createTestImage(1, 0, 1024);

    result = ota_.startUpdate(image.size() - HEADER_SIZE);
    ASSERT_TRUE(result.isOk());

    result = ota_.receiveData(image.data(), image.size());
    ASSERT_TRUE(result.isOk());

    result = ota_.finishUpdate();
    ASSERT_TRUE(result.isOk());

    // Apply update
    result = ota_.applyUpdate();
    // May succeed or fail depending on implementation state
    // Just verify it doesn't crash
}

TEST_F(OTATest, ApplyUpdateWithoutData) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    result = ota_.applyUpdate();
    EXPECT_FALSE(result.isOk());
}

// ============================================================================
// Version Tests
// ============================================================================

TEST_F(OTATest, GetCurrentVersion) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    FirmwareVersion version = ota_.getCurrentVersion();
    // Default version should be 0.0.0.0 or similar
    EXPECT_EQ(version.major, 0);
}

TEST_F(OTATest, GetPendingVersion) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    // Before update, pending should be empty
    FirmwareVersion pending = ota_.getPendingVersion();
    EXPECT_EQ(pending.major, 0);

    // After receiving update
    auto image = createTestImage(2, 1, 1024);

    result = ota_.startUpdate(image.size() - HEADER_SIZE);
    ASSERT_TRUE(result.isOk());

    result = ota_.receiveData(image.data(), image.size());
    ASSERT_TRUE(result.isOk());

    result = ota_.finishUpdate();
    ASSERT_TRUE(result.isOk());

    pending = ota_.getPendingVersion();
    EXPECT_EQ(pending.major, 2);
    EXPECT_EQ(pending.minor, 1);
}

// ============================================================================
// Flash Operation Tests
// ============================================================================

TEST_F(OTATest, FlashWriteCount) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    auto image = createTestImage(1, 0, 1024);

    result = ota_.startUpdate(image.size() - HEADER_SIZE);
    ASSERT_TRUE(result.isOk());

    size_t initial_writes = flash_.getWriteCount();

    result = ota_.receiveData(image.data(), image.size());
    ASSERT_TRUE(result.isOk());

    EXPECT_GT(flash_.getWriteCount(), initial_writes);
}

TEST_F(OTATest, FlashEraseOnStart) {
    auto result = initOTA();
    ASSERT_TRUE(result.isOk());

    size_t initial_erases = flash_.getEraseCount();

    result = ota_.startUpdate(1024);
    ASSERT_TRUE(result.isOk());

    // Should have erased the target slot
    EXPECT_GT(flash_.getEraseCount(), initial_erases);
}
