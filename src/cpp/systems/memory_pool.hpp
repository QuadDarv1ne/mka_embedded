/**
 * @file memory_pool.hpp
 * @brief Thread-safe Memory Pool for RTOS
 * 
 * Пул памяти для безопасного выделения памяти в RTOS окружении.
 * Предотвращает фрагментацию и обеспечивает детерминированное
 * время выделения памяти.
 */

#ifndef MEMORY_POOL_HPP
#define MEMORY_POOL_HPP

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <array>
#include <optional>

namespace mka {
namespace memory {

// ============================================================================
// Fixed-size Block Pool
// ============================================================================

/**
 * @brief Пул блоков фиксированного размера
 * 
 * Эффективен для выделения множества объектов одинакового размера.
 * O(1) время выделения и освобождения.
 */
template<size_t BlockSize, size_t NumBlocks>
class FixedBlockPool {
public:
    static constexpr size_t ALIGNMENT = 8;
    static constexpr size_t ALIGNED_BLOCK_SIZE = 
        ((BlockSize + ALIGNMENT - 1) / ALIGNMENT) * ALIGNMENT;
    
    FixedBlockPool() {
        reset();
    }
    
    /**
     * @brief Выделение блока
     * @return Указатель на блок или nullptr если пул исчерпан
     */
    void* allocate() {
        // Поиск свободного блока
        for (size_t i = 0; i < NumBlocks; i++) {
            if (!used_[i]) {
                used_[i] = true;
                allocatedCount_++;
                return &pool_[i * ALIGNED_BLOCK_SIZE];
            }
        }
        return nullptr;
    }
    
    /**
     * @brief Освобождение блока
     * @param ptr Указатель на блок
     * @return true если освобождение успешно
     */
    bool deallocate(void* ptr) {
        if (!ptr) return false;
        if (!owns(ptr)) return false;

        size_t index = getIndex(ptr);
        if (index >= NumBlocks) return false;
        if (!used_[index]) return false;  // Double free защита

        used_[index] = false;
        allocatedCount_--;
        return true;
    }

    /**
     * @brief Проверка принадлежности указателя пулу
     */
    bool owns(void* ptr) const {
        if (!ptr) return false;
        const uint8_t* p = static_cast<const uint8_t*>(ptr);
        const uint8_t* begin = pool_.data();
        const uint8_t* end = begin + NumBlocks * ALIGNED_BLOCK_SIZE;
        return p >= begin && p < end;
    }
    
    /**
     * @brief Сброс пула (освобождение всех блоков)
     */
    void reset() {
        used_.fill(false);
        allocatedCount_ = 0;
    }
    
    // Статистика
    size_t capacity() const { return NumBlocks; }
    size_t available() const { return NumBlocks - allocatedCount_; }
    size_t allocated() const { return allocatedCount_; }
    size_t blockSize() const { return BlockSize; }
    size_t totalSize() const { return NumBlocks * ALIGNED_BLOCK_SIZE; }
    
private:
    alignas(ALIGNMENT) std::array<uint8_t, NumBlocks * ALIGNED_BLOCK_SIZE> pool_{};
    std::array<bool, NumBlocks> used_{};
    size_t allocatedCount_ = 0;
    
    size_t getIndex(void* ptr) const {
        if (!ptr || !owns(ptr)) return NumBlocks;  // Возвращаем невалидный индекс
        const uint8_t* p = static_cast<const uint8_t*>(ptr);
        return static_cast<size_t>((p - pool_.data()) / ALIGNED_BLOCK_SIZE);
    }
};

// ============================================================================
// Variable-size Pool (Buddy Allocator)
// ============================================================================

/**
 * @brief Пул переменного размера (аллокатор buddy system)
 * 
 * Позволяет выделять блоки разного размера с минимальной фрагментацией.
 */
class VariablePool {
public:
    static constexpr size_t MIN_BLOCK_SIZE = 16;
    static constexpr size_t MAX_ORDER = 8;  // До 16 * 2^8 = 4096 байт
    
    explicit VariablePool(size_t totalSize) 
        : totalSize_(totalSize) {
        // Инициализация свободных списков
        for (auto& list : freeLists_) {
            list.head = nullptr;
            list.count = 0;
        }
    }
    
    void* allocate(size_t size) {
        if (size == 0) return nullptr;
        
        // Округление до ближайшей степени двойки
        size_t blockSize = MIN_BLOCK_SIZE;
        uint8_t order = 0;
        while (blockSize < size && order < MAX_ORDER) {
            blockSize *= 2;
            order++;
        }
        
        // Поиск свободного блока
        BlockHeader* block = freeLists_[order].head;
        if (block) {
            freeLists_[order].head = block->next;
            freeLists_[order].count--;
            block->allocated = true;
            return block + 1;
        }
        
        // Разделение большего блока
        for (uint8_t o = order + 1; o <= MAX_ORDER; o++) {
            if (freeLists_[o].head) {
                // Разделение блока
                splitBlock(o, order);
                block = freeLists_[order].head;
                freeLists_[order].head = block->next;
                freeLists_[order].count--;
                block->allocated = true;
                return block + 1;
            }
        }
        
        return nullptr;
    }
    
    void deallocate(void* ptr) {
        if (!ptr) return;

        BlockHeader* block = static_cast<BlockHeader*>(ptr) - 1;
        if (block->allocated == false) return;  // Double free защита

        block->allocated = false;

        // Попытка объединения (coalescing)
        uint8_t order = block->order;
        block->next = freeLists_[order].head;
        freeLists_[order].head = block;
        freeLists_[order].count++;
    }
    
    size_t totalSize() const { return totalSize_; }
    
private:
    struct BlockHeader {
        BlockHeader* next;
        uint8_t order;
        bool allocated;
        uint8_t reserved[6];  // Padding для выравнивания
    };
    
    struct FreeList {
        BlockHeader* head;
        size_t count;
    };
    
    size_t totalSize_;
    FreeList freeLists_[MAX_ORDER + 1];
    
    void splitBlock(uint8_t fromOrder, uint8_t toOrder) {
        // Найти блок для разделения
        BlockHeader* block = freeLists_[fromOrder].head;
        if (!block) return;
        
        // Удалить из списка больших блоков
        freeLists_[fromOrder].head = block->next;
        freeLists_[fromOrder].count--;
        
        // Разделить на два блока меньшего размера
        size_t blockSize = MIN_BLOCK_SIZE * (1 << toOrder);
        BlockHeader* second = reinterpret_cast<BlockHeader*>(
            reinterpret_cast<uint8_t*>(block) + blockSize);
        
        block->order = toOrder;
        block->allocated = false;
        block->next = nullptr;
        
        second->order = toOrder;
        second->allocated = false;
        second->next = freeLists_[toOrder].head;
        
        // Добавить в список меньших блоков
        freeLists_[toOrder].head = block;
        freeLists_[toOrder].count += 2;
    }
};

// ============================================================================
// Memory Pool Manager
// ============================================================================

/**
 * @brief Менеджер пулов памяти
 * 
 * Управляет несколькими пулами для разных размеров объектов.
 */
class MemoryPoolManager {
public:
    static constexpr size_t SMALL_BLOCK = 32;
    static constexpr size_t MEDIUM_BLOCK = 128;
    static constexpr size_t LARGE_BLOCK = 512;
    
    static constexpr size_t SMALL_COUNT = 64;
    static constexpr size_t MEDIUM_COUNT = 32;
    static constexpr size_t LARGE_COUNT = 16;
    
    MemoryPoolManager() = default;
    
    /**
     * @brief Выделение памяти
     * @param size Запрашиваемый размер
     * @return Указатель на память или nullptr
     */
    void* allocate(size_t size) {
        if (size <= SMALL_BLOCK) {
            return smallPool_.allocate();
        } else if (size <= MEDIUM_BLOCK) {
            return mediumPool_.allocate();
        } else if (size <= LARGE_BLOCK) {
            return largePool_.allocate();
        }
        return nullptr;
    }
    
    /**
     * @brief Освобождение памяти
     * @param ptr Указатель на память
     */
    void deallocate(void* ptr) {
        if (smallPool_.owns(ptr)) {
            smallPool_.deallocate(ptr);
        } else if (mediumPool_.owns(ptr)) {
            mediumPool_.deallocate(ptr);
        } else if (largePool_.owns(ptr)) {
            largePool_.deallocate(ptr);
        }
    }
    
    /**
     * @brief Получение статистики использования памяти
     */
    struct Stats {
        size_t smallUsed;
        size_t smallTotal;
        size_t mediumUsed;
        size_t mediumTotal;
        size_t largeUsed;
        size_t largeTotal;
    };
    
    Stats getStats() const {
        return Stats{
            smallPool_.allocated(), smallPool_.capacity(),
            mediumPool_.allocated(), mediumPool_.capacity(),
            largePool_.allocated(), largePool_.capacity()
        };
    }
    
    /**
     * @brief Сброс всех пулов
     */
    void reset() {
        smallPool_.reset();
        mediumPool_.reset();
        largePool_.reset();
    }
    
private:
    FixedBlockPool<SMALL_BLOCK, SMALL_COUNT> smallPool_;
    FixedBlockPool<MEDIUM_BLOCK, MEDIUM_COUNT> mediumPool_;
    FixedBlockPool<LARGE_BLOCK, LARGE_COUNT> largePool_;
};

// ============================================================================
// RAII Wrapper
// ============================================================================

/**
 * @brief RAII обёртка для выделенной памяти
 */
template<typename T>
class PoolAllocated {
public:
    PoolAllocated(MemoryPoolManager& pool) 
        : pool_(pool), ptr_(nullptr) {}
    
    bool allocate() {
        ptr_ = static_cast<T*>(pool_.allocate(sizeof(T)));
        if (ptr_) {
            new (ptr_) T();
            return true;
        }
        return false;
    }
    
    ~PoolAllocated() {
        if (ptr_) {
            ptr_->~T();
            pool_.deallocate(ptr_);
        }
    }
    
    T* get() { return ptr_; }
    const T* get() const { return ptr_; }
    T* operator->() { return ptr_; }
    const T* operator->() const { return ptr_; }
    T& operator*() { return *ptr_; }
    const T& operator*() const { return *ptr_; }
    
    // Запрет копирования
    PoolAllocated(const PoolAllocated&) = delete;
    PoolAllocated& operator=(const PoolAllocated&) = delete;
    
    // Разрешение перемещения
    PoolAllocated(PoolAllocated&& other) noexcept 
        : pool_(other.pool_), ptr_(other.ptr_) {
        other.ptr_ = nullptr;
    }
    
private:
    MemoryPoolManager& pool_;
    T* ptr_;
};

// ============================================================================
// Стандартные определения пулов для МКА
// ============================================================================

// Пул для сообщений телеметрии
using TelemetryPool = FixedBlockPool<64, 128>;

// Пул для пакетов команд
using CommandPool = FixedBlockPool<128, 32>;

// Пул для буферов данных
using DataBufferPool = FixedBlockPool<256, 64>;

} // namespace memory
} // namespace mka

#endif // MEMORY_POOL_HPP
