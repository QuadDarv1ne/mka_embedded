/**
 * @file span.hpp
 * @brief Span implementation for C++17 compatibility
 *
 * Упрощённая реализация std::span для C++17.
 * Не является полной заменой std::span<C++20>, но покрывает основные нужды.
 */

#ifndef MKA_SPAN_HPP
#define MKA_SPAN_HPP

#include <cstdint>
#include <cstddef>
#include <array>
#include <vector>
#include <type_traits>
#include <stdexcept>

namespace mka {

/**
 * @brief Не владеющий view на непрерывную последовательность объектов
 *
 * Упрощённая альтернатива std::span для C++17.
 * Не хранит размер, только указатель и длину.
 */
template<typename T>
class span {
public:
    using element_type = T;
    using value_type = std::remove_cv_t<T>;
    using size_type = size_t;
    using difference_type = ptrdiff_t;
    using reference = T&;
    using const_reference = const T&;
    using pointer = T*;
    using const_pointer = const T*;
    using iterator = pointer;
    using const_iterator = const T*;

    static constexpr size_type extent = static_cast<size_type>(-1);

    // ========================================================================
    // Конструкторы
    // ========================================================================

    constexpr span() noexcept : data_(nullptr), size_(0) {}

    constexpr span(std::nullptr_t) noexcept : data_(nullptr), size_(0) {}

    constexpr span(pointer ptr, size_type count) noexcept
        : data_(ptr), size_(count) {}

    constexpr span(pointer first, pointer last) noexcept
        : data_(first), size_(static_cast<size_type>(last - first)) {}

    // Конструктор для std::array
    template<size_t N>
    constexpr span(std::array<T, N>& arr) noexcept
        : data_(arr.data()), size_(N) {}

    template<size_t N>
    constexpr span(const std::array<T, N>& arr) noexcept
        : data_(arr.data()), size_(N) {}

    // Конструктор для C-style массива
    template<size_t N>
    constexpr span(T (&arr)[N]) noexcept
        : data_(arr), size_(N) {}

    // Конструктор для C-style массива (const T из non-const массива)
    template<size_t N, typename U = T, std::enable_if_t<std::is_const_v<U>, int> = 0>
    constexpr span(std::remove_const_t<T> (&arr)[N]) noexcept
        : data_(arr), size_(N) {}

    // Конструктор для std::vector (только для non-const T)
    template<typename U = T, std::enable_if_t<!std::is_const_v<U>, int> = 0>
    constexpr span(std::vector<T>& vec) noexcept
        : data_(vec.data()), size_(vec.size()) {}

    constexpr span(const std::vector<std::remove_const_t<T>>& vec) noexcept
        : data_(vec.data()), size_(vec.size()) {}

    // ========================================================================
    // Итераторы
    // ========================================================================

    constexpr iterator begin() const noexcept { return data_; }
    constexpr iterator end() const noexcept { return data_ + size_; }
    constexpr const_iterator cbegin() const noexcept { return data_; }
    constexpr const_iterator cend() const noexcept { return data_ + size_; }

    // Reverse итераторы
    constexpr std::reverse_iterator<iterator> rbegin() const noexcept {
        return std::reverse_iterator<iterator>(end());
    }
    constexpr std::reverse_iterator<iterator> rend() const noexcept {
        return std::reverse_iterator<iterator>(begin());
    }

    // ========================================================================
    // Доступ к элементам
    // ========================================================================

    constexpr reference operator[](size_type idx) const {
#ifdef MKA_SPAN_CHECK_BOUNDS
        if (idx >= size_) throw std::out_of_range("span index out of range");
#endif
        return data_[idx];
    }

    constexpr reference front() const {
#ifdef MKA_SPAN_CHECK_BOUNDS
        if (size_ == 0) throw std::out_of_range("span is empty");
#endif
        return data_[0];
    }
    
    constexpr reference back() const {
#ifdef MKA_SPAN_CHECK_BOUNDS
        if (size_ == 0) throw std::out_of_range("span is empty");
#endif
        return data_[size_ - 1];
    }
    
    constexpr pointer data() const noexcept { return data_; }

    // ========================================================================
    // Размер
    // ========================================================================

    constexpr size_type size() const noexcept { return size_; }
    constexpr size_type size_bytes() const noexcept { return size_ * sizeof(T); }
    constexpr bool empty() const noexcept { return size_ == 0; }

    // ========================================================================
    // Subspan операции (с проверками границ)
    // ========================================================================

    constexpr span first(size_type count) const noexcept {
        return span(data_, count <= size_ ? count : size_);
    }

    constexpr span last(size_type count) const noexcept {
        size_type actualCount = count <= size_ ? count : size_;
        return span(data_ + size_ - actualCount, actualCount);
    }

    constexpr span subspan(size_type offset, size_type count = static_cast<size_type>(-1)) const noexcept {
        if (offset >= size_) {
            return span(data_ + size_, size_type{0});  // Пустой span, избегаем OOB арифметики
        }
        if (count == static_cast<size_type>(-1) || count > size_ - offset) {
            count = size_ - offset;
        }
        return span(data_ + offset, count);
    }

private:
    pointer data_;
    size_type size_;
};

// ============================================================================
// Factory функции
// ============================================================================

template<typename T>
constexpr span<T> make_span(T* ptr, size_t size) noexcept {
    return span<T>(ptr, size);
}

template<typename T, size_t N>
constexpr span<T> make_span(T (&arr)[N]) noexcept {
    return span<T>(arr);
}

template<typename T, size_t N>
constexpr span<T> make_span(std::array<T, N>& arr) noexcept {
    return span<T>(arr);
}

template<typename T, size_t N>
constexpr span<const T> make_span(const std::array<T, N>& arr) noexcept {
    return span<const T>(arr);
}

} // namespace mka

#endif // MKA_SPAN_HPP
