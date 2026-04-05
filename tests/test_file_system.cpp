/**
 * @file test_file_system.cpp
 * @brief Tests for LittleFS-based file system
 */

#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <vector>
#include <string>

#include "systems/file_system.hpp"
#include "utils/result.hpp"

using namespace mka;
using namespace mka::filesystem;

// ============================================================================
// Mock Block Device for Testing
// ============================================================================

class MockBlockDevice {
public:
    MockBlockDevice(size_t block_size = 4096, size_t block_count = 256)
        : block_size_(block_size), block_count_(block_count) {
        storage_.resize(block_size * block_count, 0);
    }

    Result<int, FSStatus> read(uint32_t block, void* buffer, size_t size) {
        if (block >= block_count_) return Err<int, FSStatus>(FSStatus::NO_SPACE);
        if (size > block_size_) size = block_size_;

        std::memcpy(buffer, storage_.data() + block * block_size_, size);
        read_count_++;
        return Ok<int, FSStatus>(static_cast<int>(size));
    }

    Result<int, FSStatus> prog(uint32_t block, const void* buffer, size_t size) {
        if (block >= block_count_) return Err<int, FSStatus>(FSStatus::NO_SPACE);
        if (size > block_size_) size = block_size_;

        std::memcpy(storage_.data() + block * block_size_, buffer, size);
        write_count_++;
        return Ok<int, FSStatus>(static_cast<int>(size));
    }

    Result<int, FSStatus> erase(uint32_t block) {
        if (block >= block_count_) return Err<int, FSStatus>(FSStatus::NO_SPACE);

        std::memset(storage_.data() + block * block_size_, 0xFF, block_size_);
        erase_count_++;
        return Ok<int, FSStatus>(0);
    }

    Result<void, FSStatus> sync() {
        sync_count_++;
        return Ok<FSStatus>();
    }

    size_t getReadCount() const { return read_count_; }
    size_t getWriteCount() const { return write_count_; }
    size_t getEraseCount() const { return erase_count_; }

    void reset() {
        std::fill(storage_.begin(), storage_.end(), 0);
        read_count_ = 0;
        write_count_ = 0;
        erase_count_ = 0;
        sync_count_ = 0;
    }

private:
    size_t block_size_;
    size_t block_count_;
    std::vector<uint8_t> storage_;
    size_t read_count_ = 0;
    size_t write_count_ = 0;
    size_t erase_count_ = 0;
    size_t sync_count_ = 0;
};

// ============================================================================
// Basic Tests
// ============================================================================

class FileSystemTest : public ::testing::Test {
protected:
    MockBlockDevice block_device_;
    FileSystem fs_;

    void SetUp() override {
        block_device_.reset();
    }

    Result<void, FSStatus> setupFileSystem() {
        // Configure block device
        auto result = fs_.configure(
            [this](uint32_t block, void* buffer, size_t size) {
                return block_device_.read(block, buffer, size);
            },
            [this](uint32_t block, const void* buffer, size_t size) {
                return block_device_.prog(block, buffer, size);
            },
            [this](uint32_t block) {
                return block_device_.erase(block);
            },
            [this]() {
                return block_device_.sync();
            }
        );

        if (!result.isOk()) return result;

        // Format
        result = fs_.format();
        if (!result.isOk()) return result;

        // Mount
        return fs_.mount();
    }
};

// ============================================================================
// Format and Mount Tests
// ============================================================================

TEST_F(FileSystemTest, FormatAndMount) {
    auto result = setupFileSystem();
    EXPECT_TRUE(result.isOk());
    EXPECT_TRUE(fs_.isMounted());
}

TEST_F(FileSystemTest, DoubleFormat) {
    auto result = setupFileSystem();
    EXPECT_TRUE(result.isOk());

    // Second format should work
    result = fs_.format();
    EXPECT_TRUE(result.isOk());
}

TEST_F(FileSystemTest, UnmountAndRemount) {
    auto result = setupFileSystem();
    EXPECT_TRUE(result.isOk());

    result = fs_.unmount();
    EXPECT_TRUE(result.isOk());
    EXPECT_FALSE(fs_.isMounted());

    result = fs_.mount();
    EXPECT_TRUE(result.isOk());
    EXPECT_TRUE(fs_.isMounted());
}

// ============================================================================
// File Operations Tests
// ============================================================================

// TODO: Enable when LittleFS implementation is ready
TEST_F(FileSystemTest, DISABLED_WriteAndReadFile) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    const char* test_data = "Hello, MKA Embedded!";
    result = fs_.writeFile("/test.txt", test_data, std::strlen(test_data));
    ASSERT_TRUE(result.isOk());

    std::array<char, 64> buffer{};
    auto read_result = fs_.readFile("/test.txt", buffer.data(), buffer.size());
    ASSERT_TRUE(read_result.isOk());

    EXPECT_EQ(std::string(buffer.data()), std::string(test_data));
}

TEST_F(FileSystemTest, DISABLED_ReadNonExistentFile) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    std::array<char, 64> buffer{};
    auto read_result = fs_.readFile("/nonexistent.txt", buffer.data(), buffer.size());
    EXPECT_FALSE(read_result.isOk());
    EXPECT_EQ(read_result.error(), FSStatus::NOT_FOUND);
}

TEST_F(FileSystemTest, DISABLED_OverwriteFile) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    // Write initial data
    const char* data1 = "First version";
    result = fs_.writeFile("/overwrite.txt", data1, std::strlen(data1));
    ASSERT_TRUE(result.isOk());

    // Overwrite with new data
    const char* data2 = "Second version - longer data";
    result = fs_.writeFile("/overwrite.txt", data2, std::strlen(data2));
    ASSERT_TRUE(result.isOk());

    // Read and verify
    std::array<char, 64> buffer{};
    auto read_result = fs_.readFile("/overwrite.txt", buffer.data(), buffer.size());
    ASSERT_TRUE(read_result.isOk());

    EXPECT_EQ(std::string(buffer.data()), std::string(data2));
}

TEST_F(FileSystemTest, DISABLED_FileExists) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    EXPECT_FALSE(fs_.exists("/test.txt"));

    result = fs_.writeFile("/test.txt", "data", 4);
    ASSERT_TRUE(result.isOk());

    EXPECT_TRUE(fs_.exists("/test.txt"));
}

// ============================================================================
// File Handle Tests
// ============================================================================

TEST_F(FileSystemTest, DISABLED_FileHandleReadWrite) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    // Open for writing
    auto open_result = fs_.open("/handle_test.bin", FileMode::WRITE_CREATE);
    ASSERT_TRUE(open_result.isOk());

    FileHandle handle = std::move(open_result.value());
    EXPECT_TRUE(handle.isOpen());

    // Write data
    std::array<uint8_t, 10> write_data = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    auto write_result = handle.write(write_data.data(), write_data.size());
    EXPECT_TRUE(write_result.isOk());
    EXPECT_EQ(write_result.value(), 10);

    // Close
    auto close_result = handle.close();
    EXPECT_TRUE(close_result.isOk());
    EXPECT_FALSE(handle.isOpen());

    // Open for reading
    open_result = fs_.open("/handle_test.bin", FileMode::READ);
    ASSERT_TRUE(open_result.isOk());
    handle = std::move(open_result.value());

    // Read data
    std::array<uint8_t, 10> read_data{};
    auto read_result = handle.read(read_data.data(), read_data.size());
    EXPECT_TRUE(read_result.isOk());
    EXPECT_EQ(read_result.value(), 10);
    EXPECT_EQ(read_data, write_data);

    handle.close();
}

TEST_F(FileSystemTest, DISABLED_FileHandleSeek) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    // Create file with known data
    std::array<uint8_t, 20> data{};
    for (size_t i = 0; i < data.size(); i++) {
        data[i] = static_cast<uint8_t>(i);
    }

    auto write_result = fs_.writeFile("/seek_test.bin", data.data(), data.size());
    ASSERT_TRUE(write_result.isOk());

    // Open and seek
    auto open_result = fs_.open("/seek_test.bin", FileMode::READ);
    ASSERT_TRUE(open_result.isOk());

    FileHandle handle = std::move(open_result.value());

    // Seek to position 5
    auto seek_result = handle.seek(5, 0);  // SEEK_SET
    ASSERT_TRUE(seek_result.isOk());
    EXPECT_EQ(seek_result.value(), 5);
    EXPECT_EQ(handle.tell(), 5);

    // Read from position 5
    std::array<uint8_t, 5> read_data{};
    auto read_result = handle.read(read_data.data(), read_data.size());
    ASSERT_TRUE(read_result.isOk());

    EXPECT_EQ(read_data[0], 5);
    EXPECT_EQ(read_data[1], 6);
    EXPECT_EQ(read_data[2], 7);
    EXPECT_EQ(read_data[3], 8);
    EXPECT_EQ(read_data[4], 9);

    handle.close();
}

TEST_F(FileSystemTest, DISABLED_FileHandleAppend) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    // Write initial data
    const char* data1 = "Hello";
    auto write_result = fs_.writeFile("/append.txt", data1, std::strlen(data1));
    ASSERT_TRUE(write_result.isOk());

    // Open for append
    auto open_result = fs_.open("/append.txt", FileMode::APPEND_CREATE);
    ASSERT_TRUE(open_result.isOk());

    FileHandle handle = std::move(open_result.value());

    // Append data
    const char* data2 = " World";
    auto append_result = handle.write(data2, std::strlen(data2));
    ASSERT_TRUE(append_result.isOk());

    handle.close();

    // Read full content
    std::array<char, 32> buffer{};
    auto read_result = fs_.readFile("/append.txt", buffer.data(), buffer.size());
    ASSERT_TRUE(read_result.isOk());

    EXPECT_EQ(std::string(buffer.data()), "Hello World");
}

// ============================================================================
// Directory Tests
// ============================================================================

TEST_F(FileSystemTest, CreateDirectory) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    result = fs_.mkdir("/data");
    EXPECT_TRUE(result.isOk());

    result = fs_.mkdir("/data/logs");
    EXPECT_TRUE(result.isOk());
}

TEST_F(FileSystemTest, RemoveDirectory) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    result = fs_.mkdir("/test_dir");
    ASSERT_TRUE(result.isOk());

    result = fs_.rmdir("/test_dir");
    EXPECT_TRUE(result.isOk());
}

TEST_F(FileSystemTest, DISABLED_RemoveNonEmptyDirectory) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    result = fs_.mkdir("/nonempty_dir");
    ASSERT_TRUE(result.isOk());

    result = fs_.writeFile("/nonempty_dir/file.txt", "data", 4);
    ASSERT_TRUE(result.isOk());

    result = fs_.rmdir("/nonempty_dir");
    EXPECT_FALSE(result.isOk());
    EXPECT_EQ(result.error(), FSStatus::NOT_EMPTY);
}

TEST_F(FileSystemTest, DISABLED_ListDirectory) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    // Create directory structure
    result = fs_.mkdir("/list_test");
    ASSERT_TRUE(result.isOk());

    result = fs_.writeFile("/list_test/file1.txt", "data1", 5);
    ASSERT_TRUE(result.isOk());

    result = fs_.writeFile("/list_test/file2.txt", "data2", 5);
    ASSERT_TRUE(result.isOk());

    result = fs_.mkdir("/list_test/subdir");
    ASSERT_TRUE(result.isOk());

    // List contents
    std::vector<std::string> entries;
    auto list_result = fs_.listdir("/list_test",
        [&entries](const char* name, bool is_dir) {
            entries.push_back(std::string(name) + (is_dir ? "/" : ""));
        });

    EXPECT_TRUE(list_result.isOk());
    EXPECT_EQ(entries.size(), 3);  // file1.txt, file2.txt, subdir/
}

TEST_F(FileSystemTest, DISABLED_NestedDirectories) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    // Create nested structure
    result = fs_.mkdir("/level1");
    ASSERT_TRUE(result.isOk());

    result = fs_.mkdir("/level1/level2");
    ASSERT_TRUE(result.isOk());

    result = fs_.mkdir("/level1/level2/level3");
    ASSERT_TRUE(result.isOk());

    // Write file in deepest directory
    const char* test_data = "Deep file";
    result = fs_.writeFile("/level1/level2/level3/deep.txt", test_data, std::strlen(test_data));
    ASSERT_TRUE(result.isOk());

    // Read it back
    std::array<char, 32> buffer{};
    auto read_result = fs_.readFile("/level1/level2/level3/deep.txt", buffer.data(), buffer.size());
    ASSERT_TRUE(read_result.isOk());

    EXPECT_EQ(std::string(buffer.data()), std::string(test_data));
}

// ============================================================================
// File Statistics Tests
// ============================================================================

TEST_F(FileSystemTest, DISABLED_FileStat) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    const char* test_data = "Test data for stat";
    result = fs_.writeFile("/stat_test.txt", test_data, std::strlen(test_data));
    ASSERT_TRUE(result.isOk());

    auto stat_result = fs_.stat("/stat_test.txt");
    ASSERT_TRUE(stat_result.isOk());

    EXPECT_EQ(stat_result.value().type, FileType::FILE);
    EXPECT_EQ(stat_result.value().size, std::strlen(test_data));
}

TEST_F(FileSystemTest, DISABLED_DirectoryStat) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    result = fs_.mkdir("/stat_dir");
    ASSERT_TRUE(result.isOk());

    auto stat_result = fs_.stat("/stat_dir");
    ASSERT_TRUE(stat_result.isOk());

    EXPECT_EQ(stat_result.value().type, FileType::DIRECTORY);
}

// ============================================================================
// Rename Tests
// ============================================================================

TEST_F(FileSystemTest, DISABLED_RenameFile) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    const char* test_data = "Rename test";
    result = fs_.writeFile("/old_name.txt", test_data, std::strlen(test_data));
    ASSERT_TRUE(result.isOk());

    result = fs_.rename("/old_name.txt", "/new_name.txt");
    EXPECT_TRUE(result.isOk());

    // Old file should not exist
    EXPECT_FALSE(fs_.exists("/old_name.txt"));

    // New file should exist with same content
    std::array<char, 32> buffer{};
    auto read_result = fs_.readFile("/new_name.txt", buffer.data(), buffer.size());
    ASSERT_TRUE(read_result.isOk());
    EXPECT_EQ(std::string(buffer.data()), std::string(test_data));
}

TEST_F(FileSystemTest, DISABLED_RenameMoveToDirectory) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    result = fs_.mkdir("/destination");
    ASSERT_TRUE(result.isOk());

    result = fs_.writeFile("/source.txt", "data", 4);
    ASSERT_TRUE(result.isOk());

    result = fs_.rename("/source.txt", "/destination/source.txt");
    EXPECT_TRUE(result.isOk());

    EXPECT_FALSE(fs_.exists("/source.txt"));
    EXPECT_TRUE(fs_.exists("/destination/source.txt"));
}

// ============================================================================
// File System Statistics Tests
// ============================================================================

TEST_F(FileSystemTest, DISABLED_FSStats) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    auto stats = fs_.getStats();

    EXPECT_GT(stats.total_blocks, 0);
    EXPECT_EQ(stats.total_blocks, stats.used_blocks + stats.free_blocks);
    EXPECT_GE(stats.getUsagePercent(), 0.0f);
    EXPECT_LE(stats.getUsagePercent(), 100.0f);
}

TEST_F(FileSystemTest, StatsAfterWriting) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    auto stats_before = fs_.getStats();

    // Write some data
    std::array<uint8_t, 4096> data{};  // One block
    std::fill(data.begin(), data.end(), 0xAB);
    result = fs_.writeFile("/large_file.bin", data.data(), data.size());
    ASSERT_TRUE(result.isOk());

    auto stats_after = fs_.getStats();

    // Used blocks should increase
    EXPECT_GE(stats_after.used_blocks, stats_before.used_blocks);
    EXPECT_LE(stats_after.free_blocks, stats_before.free_blocks);
}

// ============================================================================
// Path Normalization Tests
// ============================================================================

TEST_F(FileSystemTest, PathWithDoubleSlashes) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    result = fs_.mkdir("/test");
    ASSERT_TRUE(result.isOk());

    // Double slashes should be handled
    result = fs_.writeFile("/test//file.txt", "data", 4);
    EXPECT_TRUE(result.isOk());
}

TEST_F(FileSystemTest, PathWithoutLeadingSlash) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    // Path without leading slash should work
    result = fs_.writeFile("root_file.txt", "data", 4);
    EXPECT_TRUE(result.isOk());
}

// ============================================================================
// Error Handling Tests
// ============================================================================

TEST_F(FileSystemTest, DISABLED_WriteToNonExistentDirectory) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    // Try to write to non-existent directory
    result = fs_.writeFile("/nonexistent/file.txt", "data", 4);
    EXPECT_FALSE(result.isOk());
}

TEST_F(FileSystemTest, DISABLED_InvalidPath) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    // Empty path should fail
    result = fs_.writeFile("", "data", 4);
    EXPECT_FALSE(result.isOk());
}

TEST_F(FileSystemTest, DISABLED_TooLongPath) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    // Very long path should fail
    std::string long_path(300, 'a');
    result = fs_.writeFile(long_path.c_str(), "data", 4);
    EXPECT_FALSE(result.isOk());
}

// ============================================================================
// Multiple Files Tests
// ============================================================================

TEST_F(FileSystemTest, DISABLED_MultipleFiles) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    // Create multiple files
    for (int i = 0; i < 10; i++) {
        std::string filename = "/data/file" + std::to_string(i) + ".txt";
        std::string content = "Content of file " + std::to_string(i);
        result = fs_.writeFile(filename.c_str(), content.c_str(), content.size());
        ASSERT_TRUE(result.isOk());
    }

    // Verify all files
    for (int i = 0; i < 10; i++) {
        std::string filename = "/data/file" + std::to_string(i) + ".txt";
        std::string expected = "Content of file " + std::to_string(i);

        std::array<char, 64> buffer{};
        auto read_result = fs_.readFile(filename.c_str(), buffer.data(), buffer.size());
        ASSERT_TRUE(read_result.isOk());
        EXPECT_EQ(std::string(buffer.data()), expected);
    }
}

// ============================================================================
// Binary Data Tests
// ============================================================================

TEST_F(FileSystemTest, DISABLED_BinaryData) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    // Create binary data with all byte values
    std::array<uint8_t, 256> binary_data{};
    for (int i = 0; i < 256; i++) {
        binary_data[i] = static_cast<uint8_t>(i);
    }

    result = fs_.writeFile("/binary.bin", binary_data.data(), binary_data.size());
    ASSERT_TRUE(result.isOk());

    // Read back
    std::array<uint8_t, 256> read_data{};
    auto read_result = fs_.readFile("/binary.bin", read_data.data(), read_data.size());
    ASSERT_TRUE(read_result.isOk());

    EXPECT_EQ(read_data, binary_data);
}

TEST_F(FileSystemTest, DISABLED_LargeFile) {
    auto result = setupFileSystem();
    ASSERT_TRUE(result.isOk());

    // Create file larger than one block
    constexpr size_t size = 8192;  // 2 blocks
    std::vector<uint8_t> data(size);
    for (size_t i = 0; i < size; i++) {
        data[i] = static_cast<uint8_t>(i & 0xFF);
    }

    result = fs_.writeFile("/large.bin", data.data(), data.size());
    ASSERT_TRUE(result.isOk());

    // Read back
    std::vector<uint8_t> read_data(size);
    auto read_result = fs_.readFile("/large.bin", read_data.data(), read_data.size());
    ASSERT_TRUE(read_result.isOk());
    EXPECT_EQ(static_cast<size_t>(read_result.value()), size);

    EXPECT_EQ(read_data, data);
}
