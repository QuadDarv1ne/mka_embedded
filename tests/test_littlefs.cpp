/**
 * @file test_littlefs.cpp
 * @brief Unit tests for LittleFS file system
 */

#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>
#include <vector>
#include <string>

#include "systems/file_system.hpp"

using namespace mka::filesystem;

// ============================================================================
// Block Device Tests
// ============================================================================

TEST(BlockDeviceTest, MemoryBlockDeviceInit) {
    MemoryBlockDevice block_dev(4096, 256);
    
    auto result = block_dev.init();
    ASSERT_TRUE(result.isOk());
    EXPECT_TRUE(block_dev.isInitialized());
    
    EXPECT_EQ(block_dev.getBlockSize(), 4096);
    EXPECT_EQ(block_dev.getBlockCount(), 256);
    EXPECT_EQ(block_dev.getTotalSize(), 4096 * 256);
    
    block_dev.deinit();
}

TEST(BlockDeviceTest, MemoryBlockDeviceReadWrite) {
    MemoryBlockDevice block_dev(4096, 256);
    block_dev.init();
    
    uint8_t write_data[256];
    uint8_t read_data[256];
    std::memset(write_data, 0xAB, sizeof(write_data));
    
    // Write to block 0
    auto write_result = block_dev.program(0, 0, write_data, sizeof(write_data));
    ASSERT_TRUE(write_result.isOk());
    EXPECT_EQ(write_result.value(), static_cast<int>(sizeof(write_data)));
    
    // Read from block 0
    auto read_result = block_dev.read(0, 0, read_data, sizeof(read_data));
    ASSERT_TRUE(read_result.isOk());
    EXPECT_EQ(read_result.value(), static_cast<int>(sizeof(read_data)));
    
    // Verify data
    EXPECT_EQ(std::memcmp(write_data, read_data, sizeof(write_data)), 0);
    
    block_dev.deinit();
}

TEST(BlockDeviceTest, MemoryBlockDeviceErase) {
    MemoryBlockDevice block_dev(4096, 256);
    block_dev.init();
    
    // Write some data
    uint8_t write_data[100];
    std::memset(write_data, 0xAB, sizeof(write_data));
    block_dev.program(0, 0, write_data, sizeof(write_data));
    
    // Erase block
    auto erase_result = block_dev.erase(0);
    ASSERT_TRUE(erase_result.isOk());
    
    // Verify erased (0xFF)
    uint8_t read_data[100];
    block_dev.read(0, 0, read_data, sizeof(read_data));
    
    for (size_t i = 0; i < sizeof(read_data); i++) {
        EXPECT_EQ(read_data[i], 0xFF) << "Mismatch at index " << i;
    }
    
    block_dev.deinit();
}

TEST(BlockDeviceTest, MemoryBlockDeviceOutOfBounds) {
    MemoryBlockDevice block_dev(4096, 16);
    block_dev.init();
    
    uint8_t buffer[100];
    
    // Read out of bounds
    auto read_result = block_dev.read(100, 0, buffer, sizeof(buffer));
    EXPECT_FALSE(read_result.isOk());
    
    // Write out of bounds
    auto write_result = block_dev.program(100, 0, buffer, sizeof(buffer));
    EXPECT_FALSE(write_result.isOk());
    
    // Erase out of bounds
    auto erase_result = block_dev.erase(100);
    EXPECT_FALSE(erase_result.isOk());
    
    block_dev.deinit();
}

TEST(BlockDeviceTest, MemoryBlockDeviceEraseCount) {
    MemoryBlockDevice block_dev(4096, 16);
    block_dev.init();
    
    EXPECT_EQ(block_dev.getBlockEraseCount(0), 0);
    EXPECT_EQ(block_dev.getTotalEraseCount(), 0);
    
    block_dev.erase(0);
    EXPECT_EQ(block_dev.getBlockEraseCount(0), 1);
    EXPECT_EQ(block_dev.getTotalEraseCount(), 1);
    
    block_dev.erase(0);
    block_dev.erase(1);
    EXPECT_EQ(block_dev.getBlockEraseCount(0), 2);
    EXPECT_EQ(block_dev.getBlockEraseCount(1), 1);
    EXPECT_EQ(block_dev.getTotalEraseCount(), 3);
    
    block_dev.deinit();
}

// ============================================================================
// File System Tests (Mock Implementation)
// ============================================================================

TEST(FileSystemTest, ConfigureAndMount) {
    FileSystem fs;
    
    // Configure with memory block device
    MemoryBlockDevice block_dev(4096, 64);
    block_dev.init();
    
    auto result = fs.configure(&block_dev);
    ASSERT_TRUE(result.isOk());
    
    // Mount
    auto mount_result = fs.mount();
    ASSERT_TRUE(mount_result.isOk());
    EXPECT_TRUE(fs.isMounted());
    
    // Unmount
    auto unmount_result = fs.unmount();
    ASSERT_TRUE(unmount_result.isOk());
    EXPECT_FALSE(fs.isMounted());
}

TEST(FileSystemTest, CreateAndRemoveDirectory) {
    FileSystem fs;
    
    MemoryBlockDevice block_dev(4096, 64);
    block_dev.init();
    
    fs.configure(&block_dev);
    fs.mount();
    
    // Create directory
    auto mkdir_result = fs.mkdir("/test_dir");
    ASSERT_TRUE(mkdir_result.isOk());
    
    // Remove directory
    auto rmdir_result = fs.rmdir("/test_dir");
    ASSERT_TRUE(rmdir_result.isOk());
    
    fs.unmount();
    block_dev.deinit();
}

TEST(FileSystemTest, WriteAndReadFile) {
    FileSystem fs;
    
    MemoryBlockDevice block_dev(4096, 64);
    block_dev.init();
    
    fs.configure(&block_dev);
    fs.mount();
    
    // Write file
    const char* test_data = "Hello, LittleFS!";
    auto write_result = fs.writeFile("/test.txt", test_data, std::strlen(test_data));
    ASSERT_TRUE(write_result.isOk());
    
    // Check if file exists
    EXPECT_TRUE(fs.exists("/test.txt"));
    
    // Read file
    char buffer[256];
    std::memset(buffer, 0, sizeof(buffer));
    auto read_result = fs.readFile("/test.txt", buffer, sizeof(buffer));
    ASSERT_TRUE(read_result.isOk());
    EXPECT_EQ(read_result.value(), static_cast<int>(std::strlen(test_data)));
    EXPECT_STREQ(buffer, test_data);
    
    fs.unmount();
    block_dev.deinit();
}

TEST(FileSystemTest, FileHandleOperations) {
    FileSystem fs;
    
    MemoryBlockDevice block_dev(4096, 64);
    block_dev.init();
    
    fs.configure(&block_dev);
    fs.mount();
    
    // Open file for writing
    auto open_result = fs.open("/data.bin", FileMode::WRITE_CREATE);
    ASSERT_TRUE(open_result.isOk());
    auto& file = open_result.value();
    
    EXPECT_TRUE(file.isOpen());
    EXPECT_EQ(file.tell(), 0);
    EXPECT_EQ(file.size(), 0);
    
    // Write data
    uint8_t write_data[100];
    for (int i = 0; i < 100; i++) {
        write_data[i] = static_cast<uint8_t>(i);
    }
    
    auto write_result = file.write(write_data, sizeof(write_data));
    ASSERT_TRUE(write_result.isOk());
    EXPECT_EQ(write_result.value(), 100);
    
    // Flush
    auto flush_result = file.flush();
    ASSERT_TRUE(flush_result.isOk());
    
    // Close
    auto close_result = file.close();
    ASSERT_TRUE(close_result.isOk());
    EXPECT_FALSE(file.isOpen());
    
    // Open for reading
    auto open_read = fs.open("/data.bin", FileMode::READ);
    ASSERT_TRUE(open_read.isOk());
    auto& read_file = open_read.value();
    
    EXPECT_TRUE(read_file.isOpen());
    EXPECT_EQ(read_file.tell(), 0);
    
    // Read data
    uint8_t read_data[100];
    auto read_result = read_file.read(read_data, sizeof(read_data));
    ASSERT_TRUE(read_result.isOk());
    EXPECT_EQ(read_result.value(), 100);
    
    // Verify data
    EXPECT_EQ(std::memcmp(write_data, read_data, 100), 0);
    
    // Seek
    auto seek_result = read_file.seek(0);
    ASSERT_TRUE(seek_result.isOk());
    EXPECT_EQ(read_file.tell(), 0);
    
    // Seek to end
    seek_result = read_file.seek(0, 2); // SEEK_END
    ASSERT_TRUE(seek_result.isOk());
    EXPECT_EQ(read_file.tell(), 100);
    
    fs.unmount();
    block_dev.deinit();
}

TEST(FileSystemTest, RenameFile) {
    FileSystem fs;
    
    MemoryBlockDevice block_dev(4096, 64);
    block_dev.init();
    
    fs.configure(&block_dev);
    fs.mount();
    
    // Write file
    const char* test_data = "test data";
    fs.writeFile("/old_name.txt", test_data, std::strlen(test_data));
    
    // Rename
    auto rename_result = fs.rename("/old_name.txt", "/new_name.txt");
    ASSERT_TRUE(rename_result.isOk());
    
    // Verify old doesn't exist
    EXPECT_FALSE(fs.exists("/old_name.txt"));
    
    // Verify new exists
    EXPECT_TRUE(fs.exists("/new_name.txt"));
    
    // Read and verify data
    char buffer[100];
    auto read_result = fs.readFile("/new_name.txt", buffer, sizeof(buffer));
    ASSERT_TRUE(read_result.isOk());
    EXPECT_EQ(read_result.value(), static_cast<int>(std::strlen(test_data)));
    
    fs.unmount();
    block_dev.deinit();
}

TEST(FileSystemTest, RemoveFile) {
    FileSystem fs;
    
    MemoryBlockDevice block_dev(4096, 64);
    block_dev.init();
    
    fs.configure(&block_dev);
    fs.mount();
    
    // Write file
    const char* test_data = "test data";
    fs.writeFile("/to_remove.txt", test_data, std::strlen(test_data));
    EXPECT_TRUE(fs.exists("/to_remove.txt"));
    
    // Remove
    auto remove_result = fs.remove("/to_remove.txt");
    ASSERT_TRUE(remove_result.isOk());
    
    // Verify doesn't exist
    EXPECT_FALSE(fs.exists("/to_remove.txt"));
    
    fs.unmount();
    block_dev.deinit();
}

TEST(FileSystemTest, ListDirectory) {
    FileSystem fs;
    
    MemoryBlockDevice block_dev(4096, 64);
    block_dev.init();
    
    fs.configure(&block_dev);
    fs.mount();
    
    // Create files and directories
    fs.mkdir("/dir1");
    fs.writeFile("/dir1/file1.txt", "data1", 5);
    fs.writeFile("/dir1/file2.txt", "data2", 5);
    
    // List directory
    std::vector<std::string> entries;
    auto list_result = fs.listdir("/dir1",
        [&entries](const char* name, bool is_dir) {
            entries.push_back(std::string(name) + (is_dir ? "/" : ""));
        }
    );
    
    ASSERT_TRUE(list_result.isOk());
    EXPECT_GE(entries.size(), 2); // At least file1.txt and file2.txt
    
    fs.unmount();
    block_dev.deinit();
}

TEST(FileSystemTest, FileStatistics) {
    FileSystem fs;
    
    MemoryBlockDevice block_dev(4096, 64);
    block_dev.init();
    
    fs.configure(&block_dev);
    fs.mount();
    
    // Write file
    const char* test_data = "Hello, World!";
    fs.writeFile("/stats_test.txt", test_data, std::strlen(test_data));
    
    // Get stats
    auto stat_result = fs.stat("/stats_test.txt");
    ASSERT_TRUE(stat_result.isOk());
    
    EXPECT_EQ(stat_result.value().type, FileType::FILE);
    EXPECT_EQ(stat_result.value().size, std::strlen(test_data));
    
    fs.unmount();
    block_dev.deinit();
}

TEST(FileSystemTest, FileSystemStats) {
    FileSystem fs;
    
    MemoryBlockDevice block_dev(4096, 64);
    block_dev.init();
    
    fs.configure(&block_dev);
    fs.mount();
    
    // Write some data
    for (int i = 0; i < 10; i++) {
        std::string path = "/file_" + std::to_string(i) + ".txt";
        std::string data = "data " + std::to_string(i);
        fs.writeFile(path.c_str(), data.c_str(), data.size());
    }
    
    // Get stats
    auto stats = fs.getStats();
    EXPECT_GT(stats.total_blocks, 0);
    EXPECT_GE(stats.used_blocks, 0);
    EXPECT_GT(stats.free_blocks, 0);
    
    // Usage percent should be valid
    float usage = stats.getUsagePercent();
    EXPECT_GE(usage, 0.0f);
    EXPECT_LE(usage, 100.0f);
    
    fs.unmount();
    block_dev.deinit();
}

TEST(FileSystemTest, StatusToString) {
    EXPECT_STREQ(fsStatusToString(FSStatus::OK), "OK");
    EXPECT_STREQ(fsStatusToString(FSStatus::ERROR), "ERROR");
    EXPECT_STREQ(fsStatusToString(FSStatus::NOT_FOUND), "NOT_FOUND");
    EXPECT_STREQ(fsStatusToString(FSStatus::NO_SPACE), "NO_SPACE");
    EXPECT_STREQ(fsStatusToString(FSStatus::CORRUPT), "CORRUPT");
}

TEST(BlockDeviceTest, CallbackBlockDevice) {
    MemoryBlockDevice mem_dev(4096, 16);
    mem_dev.init();
    
    // Wrap with callback device
    CallbackBlockDevice callback_dev(
        [&mem_dev](uint32_t block, void* buffer, size_t size) {
            return mem_dev.read(block, 0, buffer, size);
        },
        [&mem_dev](uint32_t block, const void* buffer, size_t size) {
            return mem_dev.program(block, 0, buffer, size);
        },
        [&mem_dev](uint32_t block) {
            return mem_dev.erase(block);
        },
        [&mem_dev]() {
            return mem_dev.sync();
        },
        4096, 16
    );
    
    // Test read/write through callbacks
    uint8_t write_data[100];
    uint8_t read_data[100];
    std::memset(write_data, 0xCD, sizeof(write_data));
    
    callback_dev.program(0, 0, write_data, sizeof(write_data));
    callback_dev.read(0, 0, read_data, sizeof(read_data));
    
    EXPECT_EQ(std::memcmp(write_data, read_data, 100), 0);
    
    mem_dev.deinit();
}

TEST(FileSystemTest, MultipleFiles) {
    FileSystem fs;
    
    MemoryBlockDevice block_dev(4096, 64);
    block_dev.init();
    
    fs.configure(&block_dev);
    fs.format();
    fs.mount();

    // Create directory and multiple files
    fs.mkdir("/test");
    for (int i = 0; i < 20; i++) {
        std::string path = "/test/file_" + std::to_string(i) + ".txt";
        std::string data = "Content of file " + std::to_string(i);
        fs.writeFile(path.c_str(), data.c_str(), data.size());
    }
    
    // Verify all files exist and have correct content
    for (int i = 0; i < 20; i++) {
        std::string path = "/test/file_" + std::to_string(i) + ".txt";
        std::string expected_data = "Content of file " + std::to_string(i);
        
        EXPECT_TRUE(fs.exists(path.c_str()));
        
        char buffer[256];
        auto read_result = fs.readFile(path.c_str(), buffer, sizeof(buffer));
        ASSERT_TRUE(read_result.isOk());
        buffer[read_result.value()] = '\0';
        EXPECT_STREQ(buffer, expected_data.c_str());
    }
    
    fs.unmount();
    block_dev.deinit();
}

TEST(FileSystemTest, AppendToFile) {
    FileSystem fs;
    
    MemoryBlockDevice block_dev(4096, 64);
    block_dev.init();
    
    fs.configure(&block_dev);
    fs.mount();
    
    // Write initial data
    fs.writeFile("/append_test.txt", "Hello", 5);
    
    // Open in append mode
    auto open_result = fs.open("/append_test.txt", FileMode::APPEND);
    ASSERT_TRUE(open_result.isOk());
    auto& file = open_result.value();
    
    // Append data
    auto write_result = file.write(", World!", 8);
    ASSERT_TRUE(write_result.isOk());
    
    file.close();
    
    // Read and verify
    char buffer[100];
    auto read_result = fs.readFile("/append_test.txt", buffer, sizeof(buffer));
    ASSERT_TRUE(read_result.isOk());
    buffer[read_result.value()] = '\0';
    
    EXPECT_STREQ(buffer, "Hello, World!");
    
    fs.unmount();
    block_dev.deinit();
}

TEST(FileSystemTest, TruncateFile) {
    FileSystem fs;
    
    MemoryBlockDevice block_dev(4096, 64);
    block_dev.init();
    
    fs.configure(&block_dev);
    fs.mount();
    
    // Write file
    const char* test_data = "Hello, World! This is a long string.";
    fs.writeFile("/truncate_test.txt", test_data, std::strlen(test_data));
    
    // Open and truncate
    auto open_result = fs.open("/truncate_test.txt", FileMode::READ_WRITE);
    ASSERT_TRUE(open_result.isOk());
    auto& file = open_result.value();
    
    // Truncate to 5 bytes
    auto truncate_result = file.truncate(5);
    ASSERT_TRUE(truncate_result.isOk());
    
    file.close();
    
    // Read and verify
    char buffer[100];
    auto read_result = fs.readFile("/truncate_test.txt", buffer, sizeof(buffer));
    ASSERT_TRUE(read_result.isOk());
    EXPECT_EQ(read_result.value(), 5);
    buffer[5] = '\0';
    EXPECT_STREQ(buffer, "Hello");
    
    fs.unmount();
    block_dev.deinit();
}

TEST(BlockDeviceTest, MemoryBlockDeviceReset) {
    MemoryBlockDevice block_dev(4096, 16);
    block_dev.init();
    
    // Write some data
    uint8_t write_data[100];
    std::memset(write_data, 0xAB, sizeof(write_data));
    block_dev.program(0, 0, write_data, sizeof(write_data));
    
    // Reset
    block_dev.reset();
    
    // Verify data is erased (0xFF)
    uint8_t read_data[100];
    block_dev.read(0, 0, read_data, sizeof(read_data));
    
    for (size_t i = 0; i < sizeof(read_data); i++) {
        EXPECT_EQ(read_data[i], 0xFF);
    }
}

TEST(BlockDeviceTest, BlockDeviceStatusToString) {
    EXPECT_STREQ(blockDeviceStatusToString(BlockDeviceStatus::OK), "OK");
    EXPECT_STREQ(blockDeviceStatusToString(BlockDeviceStatus::ERROR), "ERROR");
    EXPECT_STREQ(blockDeviceStatusToString(BlockDeviceStatus::NOT_INITIALIZED), "NOT_INITIALIZED");
    EXPECT_STREQ(blockDeviceStatusToString(BlockDeviceStatus::OUT_OF_BOUNDS), "OUT_OF_BOUNDS");
}
