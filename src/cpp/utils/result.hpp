/**
 * @file result.hpp
 * @brief Тип Result<T, E> для обработки ошибок без исключений
 *
 * Inspired by Rust's Result type and C++23 std::expected.
 * Позволяет явно представлять успех или ошибку в возвращаемом значении.
 *
 * Пример использования:
 * @code
 * Result<float, ErrorCode> readSensor() {
 *     if (!sensorPresent) {
 *         return ErrorCode::SENSOR_NOT_FOUND;
 *     }
 *     return sensorValue;  // Автоматическая обёртка в Ok
 * }
 *
 * auto result = readSensor();
 * if (result.isOk()) {
 *     float value = result.value();
 * } else {
 *     ErrorCode error = result.error();
 * }
 * @endcode
 */

#ifndef MKA_RESULT_HPP
#define MKA_RESULT_HPP

#include <cstdint>
#include <new>
#include <type_traits>
#include <utility>
#include <functional>

namespace mka {

/**
 * @brief Моноид для хранения результата операции (успех или ошибка)
 *
 * @tparam T Тип успешного значения
 * @tparam E Тип ошибки (по умолчанию ErrorCode)
 */
template<typename T, typename E = uint8_t>
class Result {
public:
    // ========================================================================
    // Типы
    // ========================================================================

    using ValueType = T;
    using ErrorType = E;

    // ========================================================================
    // Конструкторы
    // ========================================================================

    /// Конструктор по умолчанию (создаёт ошибку)
    constexpr Result() noexcept : ok_(false) {}

    /// Конструктор успеха (из значения)
    constexpr Result(const T& value) noexcept(std::is_nothrow_copy_constructible<T>::value)
        : ok_(true) {
        new (&storage_.value) T(value);
    }

    constexpr Result(T&& value) noexcept(std::is_nothrow_move_constructible<T>::value)
        : ok_(true) {
        new (&storage_.value) T(std::move(value));
    }

    /// Конструктор ошибки (из кода ошибки)
    constexpr Result(const E& error) noexcept(std::is_nothrow_copy_constructible<E>::value)
        : ok_(false) {
        new (&storage_.error) E(error);
    }

    constexpr Result(E&& error) noexcept(std::is_nothrow_move_constructible<E>::value)
        : ok_(false) {
        new (&storage_.error) E(std::move(error));
    }

    /// Копирующий конструктор
    Result(const Result& other) noexcept(
        std::is_nothrow_copy_constructible<T>::value &&
        std::is_nothrow_copy_constructible<E>::value
    ) : ok_(other.ok_) {
        if (ok_) {
            new (&storage_.value) T(other.storage_.value);
        } else {
            new (&storage_.error) E(other.storage_.error);
        }
    }

    /// Перемещающий конструктор
    Result(Result&& other) noexcept(
        std::is_nothrow_move_constructible<T>::value &&
        std::is_nothrow_move_constructible<E>::value
    ) : ok_(other.ok_) {
        if (ok_) {
            new (&storage_.value) T(std::move(other.storage_.value));
        } else {
            new (&storage_.error) E(std::move(other.storage_.error));
        }
    }

    /// Деструктор
    ~Result() {
        if (ok_) {
            storage_.value.~T();
        } else {
            storage_.error.~E();
        }
    }

    // ========================================================================
    // Операторы присваивания
    // ========================================================================

    Result& operator=(const Result& other) {
        if (this != &other) {
            this->~Result();
            new (this) Result(other);
        }
        return *this;
    }

    Result& operator=(Result&& other) noexcept {
        if (this != &other) {
            this->~Result();
            new (this) Result(std::move(other));
        }
        return *this;
    }

    // ========================================================================
    // Методы проверки состояния
    // ========================================================================

    /// Проверка на успех
    constexpr bool isOk() const noexcept { return ok_; }
    constexpr bool isSuccess() const noexcept { return ok_; }

    /// Проверка на ошибку
    constexpr bool isError() const noexcept { return !ok_; }
    constexpr bool isFailure() const noexcept { return !ok_; }

    /// Явное преобразование в bool (true = успех)
    constexpr explicit operator bool() const noexcept { return ok_; }

    // ========================================================================
    // Доступ к значению
    // ========================================================================

    /// Получить значение (с проверкой — UB если ошибка)
    constexpr T& value() & noexcept {
        // В debug mode можно добавить assert(ok_)
        return storage_.value;
    }
    constexpr const T& value() const& noexcept {
        return storage_.value;
    }
    constexpr T&& value() && noexcept {
        return std::move(storage_.value);
    }

    /// Получить значение или значение по умолчанию
    constexpr T valueOr(const T& defaultValue) const& {
        return ok_ ? storage_.value : defaultValue;
    }

    constexpr T valueOr(T&& defaultValue) && {
        return ok_ ? std::move(storage_.value) : std::forward<T>(defaultValue);
    }

    /// Безопасное получение значения (с проверкой)
    constexpr std::optional<T> tryValue() const& {
        return ok_ ? std::optional<T>(storage_.value) : std::nullopt;
    }

    /// Получить ошибку (с проверкой — UB если успех)
    constexpr E& error() & noexcept {
        return storage_.error;
    }
    constexpr const E& error() const& noexcept {
        return storage_.error;
    }
    constexpr E&& error() && noexcept {
        return std::move(storage_.error);
    }

    /// Безопасное получение ошибки (с проверкой)
    constexpr std::optional<E> tryError() const& {
        return ok_ ? std::nullopt : std::optional<E>(storage_.error);
    }

    // ========================================================================
    // Map и transform операции
    // ========================================================================

    /// Применить функцию к успешному значению
    template<typename F>
    constexpr auto map(F&& func) const&
        -> Result<std::invoke_result_t<F, const T&>, E> {
        if (ok_) {
            return std::forward<F>(func)(storage_.value);
        } else {
            return storage_.error;
        }
    }

    template<typename F>
    constexpr auto map(F&& func) &&
        -> Result<std::invoke_result_t<F, T>, E> {
        if (ok_) {
            return std::forward<F>(func)(std::move(storage_.value));
        } else {
            return storage_.error;
        }
    }

    /// Применить функцию к ошибке
    template<typename F>
    constexpr auto mapError(F&& func) const&
        -> Result<T, std::invoke_result_t<F, const E&>> {
        using NewE = std::invoke_result_t<F, const E&>;
        if (!ok_) {
            return NewE(std::forward<F>(func)(storage_.error));
        } else {
            return storage_.value;
        }
    }

    /// ИЛИ операция: вернуть это или другое
    constexpr Result<T, E> orElse(const Result<T, E>& other) const& {
        return ok_ ? *this : other;
    }

    constexpr Result<T, E> orElse(Result<T, E>&& other) && {
        return ok_ ? std::move(*this) : std::move(other);
    }

private:
    // ========================================================================
    // Внутренние данные
    // ========================================================================

    bool ok_ = false;

    union Storage {
        T value;
        E error;

        Storage() {}
        ~Storage() {}
    } storage_;
};

// ============================================================================
// Специализация для void (только успех/ошибка)
// ============================================================================

template<typename E>
class Result<void, E> {
public:
    using ValueType = void;
    using ErrorType = E;

    /// Конструктор по умолчанию (создаёт ошибку)
    constexpr Result() noexcept : ok_(false) {}

    /// Конструктор успеха (для Ok())
    struct success_tag {};
    constexpr Result(success_tag) noexcept : ok_(true) {}

    constexpr Result(const E& error) noexcept : ok_(false) {
        new (&storage_) E(error);
    }

    constexpr Result(E&& error) noexcept : ok_(false) {
        new (&storage_) E(std::move(error));
    }

    constexpr bool isOk() const noexcept { return ok_; }
    constexpr bool isError() const noexcept { return !ok_; }
    constexpr explicit operator bool() const noexcept { return ok_; }

    constexpr const E& error() const& noexcept { return storage_; }
    constexpr E&& error() && noexcept { return std::move(storage_); }

private:
    bool ok_ = false;

    union {
        E storage_;
    };
};

// ============================================================================
// Фабричные функции
// ============================================================================

template<typename T, typename E>
constexpr Result<T, E> Ok(const T& value) {
    return Result<T, E>(value);
}

template<typename T, typename E>
constexpr Result<T, E> Ok(T&& value) {
    return Result<T, E>(std::move(value));
}

template<typename E>
constexpr Result<void, E> Ok() {
    return Result<void, E>(typename Result<void, E>::success_tag{});
}

template<typename T, typename E>
constexpr Result<T, E> Err(const E& error) {
    return Result<T, E>(error);
}

template<typename T, typename E>
constexpr Result<T, E> Err(E&& error) {
    return Result<T, E>(std::move(error));
}

// ============================================================================
// Вспомогательные типы ошибок
// ============================================================================

/**
 * @brief Базовый тип кода ошибки для использования с Result
 */
enum class ErrorCode : uint8_t {
    OK = 0,
    ERROR = 1,
    TIMEOUT = 2,
    BUSY = 3,
    INVALID_PARAM = 4,
    NOT_INITIALIZED = 5,
    BUFFER_OVERFLOW = 6,
    CRC_ERROR = 7,
    NOT_SUPPORTED = 8,
    NOT_FOUND = 9,
    ALREADY_EXISTS = 10,
    OUT_OF_MEMORY = 11,
    DEVICE_ERROR = 12,
    COMMUNICATION_ERROR = 13
};

/**
 * @brief Преобразование ErrorCode в строку
 */
inline const char* errorCodeToString(ErrorCode code) {
    switch (code) {
        case ErrorCode::OK: return "OK";
        case ErrorCode::ERROR: return "ERROR";
        case ErrorCode::TIMEOUT: return "TIMEOUT";
        case ErrorCode::BUSY: return "BUSY";
        case ErrorCode::INVALID_PARAM: return "INVALID_PARAM";
        case ErrorCode::NOT_INITIALIZED: return "NOT_INITIALIZED";
        case ErrorCode::BUFFER_OVERFLOW: return "BUFFER_OVERFLOW";
        case ErrorCode::CRC_ERROR: return "CRC_ERROR";
        case ErrorCode::NOT_SUPPORTED: return "NOT_SUPPORTED";
        case ErrorCode::NOT_FOUND: return "NOT_FOUND";
        case ErrorCode::ALREADY_EXISTS: return "ALREADY_EXISTS";
        case ErrorCode::OUT_OF_MEMORY: return "OUT_OF_MEMORY";
        case ErrorCode::DEVICE_ERROR: return "DEVICE_ERROR";
        case ErrorCode::COMMUNICATION_ERROR: return "COMMUNICATION_ERROR";
        default: return "UNKNOWN";
    }
}

} // namespace mka

#endif // MKA_RESULT_HPP
