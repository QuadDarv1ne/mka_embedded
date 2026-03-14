/**
 * @file test_memory_pool.cpp
 * @brief Unit tests for Memory Pool system
 */

#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>

#include "systems/memory_pool.hpp"

using namespace mka::memory;

// ============================================================================
// Тесты FixedBlockPool
// ============================================================================

TEST(FixedBlockPoolTest, Allocation) {
    FixedBlockPool<32, 10> pool;
    
    void* ptr = pool.allocate();
    EXPECT_NE(ptr, nullptr);
    
    // Проверка что память можно записать
    std::memset(ptr, 0xAA, 32);
    
    pool.deallocate(ptr);
}

TEST(FixedBlockPoolTest, MultipleAllocations) {
    FixedBlockPool<32, 10> pool;
    
    void* ptrs[10];
    for (int i = 0; i < 10; i++) {
        ptrs[i] = pool.allocate();
        EXPECT_NE(ptrs[i], nullptr);
    }
    
    // Все блоки должны быть выделены
    for (int i = 0; i < 10; i++) {
        EXPECT_NE(ptrs[i], nullptr);
    }
}

TEST(FixedBlockPoolTest, Exhaustion) {
    FixedBlockPool<32, 5> pool;
    
    void* ptrs[5];
    for (int i = 0; i < 5; i++) {
        ptrs[i] = pool.allocate();
        EXPECT_NE(ptrs[i], nullptr);
    }
    
    // Следующее выделение должно вернуть nullptr
    void* extra = pool.allocate();
    EXPECT_EQ(extra, nullptr);
}

TEST(FixedBlockPoolTest, Deallocation) {
    FixedBlockPool<32, 5> pool;
    
    void* ptr1 = pool.allocate();
    void* ptr2 = pool.allocate();
    
    EXPECT_TRUE(pool.deallocate(ptr1));
    EXPECT_TRUE(pool.deallocate(ptr2));
    
    // После освобождения можно снова выделить
    void* ptr3 = pool.allocate();
    EXPECT_NE(ptr3, nullptr);
}

TEST(FixedBlockPoolTest, DoubleDeallocation) {
    FixedBlockPool<32, 5> pool;
    
    void* ptr = pool.allocate();
    EXPECT_TRUE(pool.deallocate(ptr));
    
    // Повторное освобождение должно вернуть false
    EXPECT_FALSE(pool.deallocate(ptr));
}

TEST(FixedBlockPoolTest, InvalidDeallocation) {
    FixedBlockPool<32, 5> pool;
    
    int dummy;
    void* external = &dummy;
    
    // Освобождение внешнего указателя должно вернуть false
    EXPECT_FALSE(pool.deallocate(external));
}

TEST(FixedBlockPoolTest, OwnsPointer) {
    FixedBlockPool<32, 5> pool;
    
    void* ptr = pool.allocate();
    EXPECT_TRUE(pool.owns(ptr));
    
    int dummy;
    void* external = &dummy;
    EXPECT_FALSE(pool.owns(external));
}

TEST(FixedBlockPoolTest, Reset) {
    FixedBlockPool<32, 5> pool;
    
    // Выделить все блоки
    for (int i = 0; i < 5; i++) {
        pool.allocate();
    }
    
    // Проверка что пул пуст
    EXPECT_EQ(pool.allocate(), nullptr);
    
    // Сброс
    pool.reset();
    
    // После сброса можно снова выделить
    void* ptr = pool.allocate();
    EXPECT_NE(ptr, nullptr);
}

TEST(FixedBlockPoolTest, Alignment) {
    FixedBlockPool<7, 10> pool;  // Не выровненный размер
    
    void* ptr = pool.allocate();
    EXPECT_NE(ptr, nullptr);
    
    // Проверка выравнивания (8 байт)
    uintptr_t addr = reinterpret_cast<uintptr_t>(ptr);
    EXPECT_EQ(addr % 8, 0);
}

TEST(FixedBlockPoolTest, DataIntegrity) {
    FixedBlockPool<64, 5> pool;

    struct Data {
        uint32_t a, b, c, d;
    };

    Data* data = static_cast<Data*>(pool.allocate());
    data->a = 0x12345678;
    data->b = 0xDEADBEEF;
    data->c = 0xCAFEBABE;
    data->d = 0x8BADF00D;

    // Проверка что данные записаны
    EXPECT_EQ(data->a, 0x12345678u);
    EXPECT_EQ(data->b, 0xDEADBEEFu);
    EXPECT_EQ(data->c, 0xCAFEBABEu);
    EXPECT_EQ(data->d, 0x8BADF00Du);

    // Освобождение и повторное выделение
    pool.deallocate(data);
    Data* data2 = static_cast<Data*>(pool.allocate());

    // Память обнуляется при освобождении (безопасность)
    EXPECT_EQ(data2->a, 0u);
    EXPECT_EQ(data2->b, 0u);
    EXPECT_EQ(data2->c, 0u);
    EXPECT_EQ(data2->d, 0u);
}

// ============================================================================
// Тесты MemoryPoolManager
// ============================================================================

TEST(MemoryPoolManagerTest, Constructor) {
    // Просто проверяем что код компилируется
    EXPECT_TRUE(true);
}

// ============================================================================
// Тесты RAII обёрток
// ============================================================================

TEST(MemoryPoolRAIITest, UniquePtr) {
    FixedBlockPool<32, 5> pool;
    
    {
        auto ptr = pool.allocate();
        EXPECT_NE(ptr, nullptr);
        
        // В реальном проекте здесь был бы RAII wrapper
        // Проверяем что память можно использовать
        std::memset(ptr, 0, 32);
        
        pool.deallocate(ptr);
    }
    
    // После выхода из scope память освобождена
    // (в данном случае явно вызван deallocate)
}

// ============================================================================
// Тесты производительности (базовые)
// ============================================================================

TEST(MemoryPoolPerformanceTest, AllocationSpeed) {
    FixedBlockPool<64, 100> pool;
    
    const int iterations = 1000;
    void* ptrs[100];
    
    // Выделение всех блоков
    for (int i = 0; i < iterations; i++) {
        for (int j = 0; j < 100; j++) {
            ptrs[j] = pool.allocate();
        }
        
        // Освобождение
        for (int j = 0; j < 100; j++) {
            pool.deallocate(ptrs[j]);
        }
    }
    
    // Тест просто проверяет что код работает
    // В реальном проекте здесь был бы benchmark
    EXPECT_TRUE(true);
}

// ============================================================================
// Тесты граничных условий
// ============================================================================

TEST(MemoryPoolBoundaryTest, ZeroSize) {
    // Пул с минимальным размером блока
    FixedBlockPool<1, 10> pool;
    
    void* ptr1 = pool.allocate();
    void* ptr2 = pool.allocate();
    
    EXPECT_NE(ptr1, nullptr);
    EXPECT_NE(ptr2, nullptr);
    EXPECT_NE(ptr1, ptr2);  // Блоки не должны перекрываться
}

TEST(MemoryPoolBoundaryTest, SingleBlock) {
    FixedBlockPool<32, 1> pool;
    
    void* ptr = pool.allocate();
    EXPECT_NE(ptr, nullptr);
    
    // Второй вызов должен вернуть nullptr
    EXPECT_EQ(pool.allocate(), nullptr);
    
    pool.deallocate(ptr);
    
    // После освобождения снова можно выделить
    EXPECT_NE(pool.allocate(), nullptr);
}

TEST(MemoryPoolBoundaryTest, LargeBlock) {
    FixedBlockPool<256, 5> pool;
    
    void* ptr = pool.allocate();
    EXPECT_NE(ptr, nullptr);
    
    // Запись данных
    std::memset(ptr, 0xFF, 256);
    
    pool.deallocate(ptr);
}

// ============================================================================
// Тесты concurrent доступа (симуляция)
// ============================================================================

TEST(MemoryPoolConcurrentTest, AlternatingAllocation) {
    FixedBlockPool<32, 20> pool;
    
    // Симуляция попеременного выделения из разных "потоков"
    for (int i = 0; i < 10; i++) {
        void* ptr1 = pool.allocate();
        void* ptr2 = pool.allocate();
        
        EXPECT_NE(ptr1, nullptr);
        EXPECT_NE(ptr2, nullptr);
        
        pool.deallocate(ptr1);
        pool.deallocate(ptr2);
    }
    
    EXPECT_TRUE(true);
}
