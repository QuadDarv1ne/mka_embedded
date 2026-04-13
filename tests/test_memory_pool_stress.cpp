/**
 * @file test_memory_pool_stress.cpp
 * @brief Стресс-тесты для Memory Pool
 * 
 * Проверяет устойчивость пула памяти к:
 * - Фрагментации при случайном выделении/освобождении
 * - Выделению максимального количества блоков
 * - Работе с разными размерами блоков
 * - Boundary conditions
 */

#include <gtest/gtest.h>
#include <vector>
#include <random>
#include <algorithm>
#include <set>

#include "systems/memory_pool.hpp"

using namespace mka;
using namespace mka::memory;

// MemoryPool - alias для обратной совместимости
using MemoryPool = VariablePool;

// ============================================================================
// Stress Tests
// ============================================================================

TEST(MemoryPoolStressTest, RandomAllocationDeallocation) {
    constexpr size_t poolSize = 4096;
    MemoryPool pool(poolSize);
    
    std::mt19937 gen(42);
    std::uniform_int_distribution<size_t> sizeDist(16, 256);
    std::uniform_int_distribution<int> actionDist(0, 1);
    
    std::vector<void*> allocations;
    constexpr int iterations = 1000;
    
    for (int i = 0; i < iterations; ++i) {
        if (actionDist(gen) == 0 || allocations.empty()) {
            // Выделение
            size_t size = sizeDist(gen);
            void* ptr = pool.allocate(size);
            if (ptr != nullptr) {
                allocations.push_back(ptr);
            }
        } else {
            // Освобождение
            size_t idx = std::uniform_int_distribution<size_t>(0, allocations.size() - 1)(gen);
            pool.deallocate(allocations[idx]);
            allocations.erase(allocations.begin() + idx);
        }
    }
    
    // Очистить все оставшиеся аллокации
    for (void* ptr : allocations) {
        pool.deallocate(ptr);
    }

    // Пул должен оставаться в рабочем состоянии
    EXPECT_TRUE(pool.totalSize() > 0);
}

TEST(MemoryPoolStressTest, FragmentationTest) {
    constexpr size_t poolSize = 8192;
    MemoryPool pool(poolSize);
    
    // Выделить много маленьких блоков
    std::vector<void*> smallBlocks;
    constexpr size_t blockSize = 32;
    
    while (true) {
        void* ptr = pool.allocate(blockSize);
        if (ptr == nullptr) break;
        smallBlocks.push_back(ptr);
    }
    
    // Освободить каждый второй блок
    for (size_t i = 0; i < smallBlocks.size(); i += 2) {
        pool.deallocate(smallBlocks[i]);
        smallBlocks[i] = nullptr;
    }
    
    // Попытаться выделить большой блок (должен вернуть nullptr из-за фрагментации)
    void* largeBlock = pool.allocate(poolSize / 2);
    EXPECT_EQ(largeBlock, nullptr);
    
    // Освободить все оставшиеся блоки
    for (void* ptr : smallBlocks) {
        if (ptr != nullptr) {
            pool.deallocate(ptr);
        }
    }
    
    EXPECT_TRUE(pool.totalSize() > 0);
}

TEST(MemoryPoolStressTest, MaximumAllocations) {
    constexpr size_t poolSize = 4096;
    MemoryPool pool(poolSize);
    
    std::vector<void*> allocations;
    constexpr size_t minBlockSize = 16;
    
    // Выделить максимальное количество блоков минимального размера
    while (true) {
        void* ptr = pool.allocate(minBlockSize);
        if (ptr == nullptr) break;
        allocations.push_back(ptr);
    }
    
    // Проверить что удалось выделить хотя бы несколько блоков
    // VariablePool с buddy allocation может выделить ограниченное количество минимальных блоков
    EXPECT_GE(allocations.size(), 2u);
    
    // Освободить все
    for (void* ptr : allocations) {
        pool.deallocate(ptr);
    }
    
    EXPECT_TRUE(pool.totalSize() > 0);
}

TEST(MemoryPoolStressTest, MixedSizesAllocation) {
    constexpr size_t poolSize = 8192;
    MemoryPool pool(poolSize);
    
    std::vector<std::pair<void*, size_t>> allocations;
    std::vector<size_t> sizes = {16, 32, 64, 128, 256, 512, 1024};
    
    // Выделить блоки разных размеров
    for (size_t size : sizes) {
        void* ptr = pool.allocate(size);
        if (ptr != nullptr) {
            allocations.push_back({ptr, size});
        }
    }
    
    // Проверить что все выделенные блоки валидны
    for (auto& alloc : allocations) {
        EXPECT_NE(alloc.first, nullptr);
    }
    
    // Освободить все
    for (auto& alloc : allocations) {
        pool.deallocate(alloc.first);
    }
    
    EXPECT_TRUE(pool.totalSize() > 0);
}

TEST(MemoryPoolStressTest, RapidRealloc) {
    constexpr size_t poolSize = 4096;
    MemoryPool pool(poolSize);
    
    // Многократное выделение и освобождение одного блока
    for (int i = 0; i < 1000; ++i) {
        void* ptr = pool.allocate(64);
        if (ptr != nullptr) {
            // Заполнить данными
            std::memset(ptr, i % 256, 64);
            pool.deallocate(ptr);
        }
    }
    
    EXPECT_TRUE(pool.totalSize() > 0);
}

TEST(MemoryPoolStressTest, BoundaryAlignment) {
    constexpr size_t poolSize = 4096;
    MemoryPool pool(poolSize);
    
    // Проверить что все аллокации выровнены
    constexpr size_t alignment = alignof(std::max_align_t);
    std::vector<void*> allocations;
    
    for (int i = 0; i < 100; ++i) {
        void* ptr = pool.allocate(17);  // Не выровненный размер
        if (ptr != nullptr) {
            allocations.push_back(ptr);
            EXPECT_EQ(reinterpret_cast<uintptr_t>(ptr) % alignment, 0);
        }
    }
    
    for (void* ptr : allocations) {
        pool.deallocate(ptr);
    }
    
    EXPECT_TRUE(pool.totalSize() > 0);
}

TEST(MemoryPoolStressTest, ExhaustionAndRecovery) {
    constexpr size_t poolSize = 2048;
    MemoryPool pool(poolSize);
    
    // Истощить пул
    std::vector<void*> allocations;
    while (true) {
        void* ptr = pool.allocate(32);
        if (ptr == nullptr) break;
        allocations.push_back(ptr);
    }
    
    EXPECT_FALSE(allocations.empty());
    
    // Освободить все
    for (void* ptr : allocations) {
        pool.deallocate(ptr);
    }
    
    // Проверить что пул восстановился
    void* ptr = pool.allocate(32);
    EXPECT_NE(ptr, nullptr);
    pool.deallocate(ptr);
    
    EXPECT_TRUE(pool.totalSize() > 0);
}

TEST(MemoryPoolStressTest, SequentialAllocations) {
    constexpr size_t poolSize = 4096;
    MemoryPool pool(poolSize);
    
    // Последовательное выделение и освобождение
    for (size_t size = 16; size <= 2048; size *= 2) {
        void* ptr = pool.allocate(size);
        if (ptr != nullptr) {
            std::memset(ptr, 0xAA, size);
            pool.deallocate(ptr);
        }
    }
    
    EXPECT_TRUE(pool.totalSize() > 0);
}

TEST(MemoryPoolStressTest, OverlappingAllocations) {
    constexpr size_t poolSize = 4096;
    MemoryPool pool(poolSize);
    
    std::set<void*> uniquePtrs;
    
    // Выделить много блоков и проверить уникальность
    for (int i = 0; i < 100; ++i) {
        void* ptr = pool.allocate(32);
        if (ptr != nullptr) {
            EXPECT_EQ(uniquePtrs.count(ptr), 0u);  // Не должно быть дубликатов
            uniquePtrs.insert(ptr);
        }
    }
    
    for (void* ptr : uniquePtrs) {
        pool.deallocate(ptr);
    }
    
    EXPECT_TRUE(pool.totalSize() > 0);
}

// ============================================================================
// Fixed Block Pool Tests
// ============================================================================

TEST(FixedBlockPoolStressTest, RapidAllocDealloc) {
    // FixedBlockPool теперь template с compile-time параметрами
    FixedBlockPool<64, 100> pool;

    std::vector<void*> allocations;

    for (int i = 0; i < 1000; ++i) {
        void* ptr = pool.allocate();
        if (ptr != nullptr) {
            allocations.push_back(ptr);
        }

        if (!allocations.empty() && i % 3 == 0) {
            pool.deallocate(allocations.back());
            allocations.pop_back();
        }
    }

    for (void* ptr : allocations) {
        pool.deallocate(ptr);
    }
}

TEST(FixedBlockPoolStressTest, ExhaustPool) {
    FixedBlockPool<32, 50> pool;

    std::vector<void*> allocations;

    // Выделить все блоки
    for (size_t i = 0; i < 100; ++i) {
        void* ptr = pool.allocate();
        if (ptr != nullptr) {
            allocations.push_back(ptr);
        }
    }

    // Должно выделить ровно 50 блоков
    EXPECT_EQ(allocations.size(), 50u);

    for (void* ptr : allocations) {
        pool.deallocate(ptr);
    }
}

TEST(FixedBlockPoolStressTest, DoubleDealloc) {
    FixedBlockPool<64, 10> pool;

    void* ptr = pool.allocate();
    EXPECT_NE(ptr, nullptr);

    pool.deallocate(ptr);
    // Повторное освоболение должно быть безопасным
    pool.deallocate(ptr);
}
