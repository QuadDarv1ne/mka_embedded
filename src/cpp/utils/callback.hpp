/**
 * @file callback.hpp
 * @brief Легковесные callback-обёртки без аллокаций в куче
 *
 * Альтернатива std::function для embedded систем.
 */

#ifndef MKA_CALLBACK_HPP
#define MKA_CALLBACK_HPP

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <utility>
#include <functional>

namespace mka {

/**
 * @brief Callback для функций и лямбд без захвата
 */
template<typename Signature>
class Callback;

template<typename Ret, typename... Args>
class Callback<Ret(Args...)> {
public:
    using InvokerFunc = Ret (*)(void*, Args...);

    constexpr Callback() noexcept
        : context_(nullptr)
        , invoker_(nullptr) {}

    constexpr Callback(std::nullptr_t) noexcept
        : context_(nullptr)
        , invoker_(nullptr) {}

    template<typename F>
    constexpr Callback(F* func) noexcept
        : context_(const_cast<void*>(reinterpret_cast<const void*>(func)))
        , invoker_(&invokeFunction<F>) {}

    constexpr bool isEmpty() const noexcept { return invoker_ == nullptr; }
    constexpr bool isValid() const noexcept { return invoker_ != nullptr; }
    constexpr explicit operator bool() const noexcept { return isValid(); }

    Callback& operator=(std::nullptr_t) noexcept {
        context_ = nullptr;
        invoker_ = nullptr;
        return *this;
    }

    friend constexpr bool operator==(const Callback& cb, std::nullptr_t) noexcept {
        return cb.invoker_ == nullptr;
    }

    friend constexpr bool operator==(std::nullptr_t, const Callback& cb) noexcept {
        return cb.invoker_ == nullptr;
    }

    friend constexpr bool operator!=(const Callback& cb, std::nullptr_t) noexcept {
        return cb.invoker_ != nullptr;
    }

    friend constexpr bool operator!=(std::nullptr_t, const Callback& cb) noexcept {
        return cb.invoker_ != nullptr;
    }

    Ret call(Args... args) const {
        if (invoker_) {
            return invoker_(context_, args...);
        }
        return Ret{};
    }

    Ret operator()(Args... args) const {
        return call(args...);
    }

    void reset() {
        context_ = nullptr;
        invoker_ = nullptr;
    }

private:
    void* context_;
    InvokerFunc invoker_;

    template<typename F>
    static Ret invokeFunction(void* context, Args... args) {
        auto* func = reinterpret_cast<F*>(context);
        return func(args...);
    }
};

template<typename... Args>
class Callback<void(Args...)> {
public:
    using InvokerFunc = void (*)(void*, Args...);

    constexpr Callback() noexcept
        : context_(nullptr)
        , invoker_(nullptr) {}

    constexpr Callback(std::nullptr_t) noexcept
        : context_(nullptr)
        , invoker_(nullptr) {}

    template<typename F>
    constexpr Callback(F* func) noexcept
        : context_(const_cast<void*>(reinterpret_cast<const void*>(func)))
        , invoker_(&invokeFunction<F>) {}

    constexpr bool isEmpty() const noexcept { return invoker_ == nullptr; }
    constexpr bool isValid() const noexcept { return invoker_ != nullptr; }
    constexpr explicit operator bool() const noexcept { return isValid(); }

    Callback& operator=(std::nullptr_t) noexcept {
        context_ = nullptr;
        invoker_ = nullptr;
        return *this;
    }

    friend constexpr bool operator==(const Callback& cb, std::nullptr_t) noexcept {
        return cb.invoker_ == nullptr;
    }

    friend constexpr bool operator==(std::nullptr_t, const Callback& cb) noexcept {
        return cb.invoker_ == nullptr;
    }

    friend constexpr bool operator!=(const Callback& cb, std::nullptr_t) noexcept {
        return cb.invoker_ != nullptr;
    }

    friend constexpr bool operator!=(std::nullptr_t, const Callback& cb) noexcept {
        return cb.invoker_ != nullptr;
    }

    void call(Args... args) const {
        if (invoker_) {
            invoker_(context_, args...);
        }
    }

    void operator()(Args... args) const {
        call(args...);
    }

    void reset() {
        context_ = nullptr;
        invoker_ = nullptr;
    }

private:
    void* context_;
    InvokerFunc invoker_;

    template<typename F>
    static void invokeFunction(void* context, Args... args) {
        auto* func = reinterpret_cast<F*>(context);
        func(args...);
    }
};

/**
 * @brief Callback с хранением контекста в статическом буфере (для лямбд с захватом)
 */
template<typename Signature, size_t ContextSize = 64>
class CallbackWithStorage;

template<typename Ret, typename... Args, size_t ContextSize>
class CallbackWithStorage<Ret(Args...), ContextSize> {
public:
    using InvokerFunc = Ret (*)(void*, Args...);
    using DestroyFunc = void (*)(void*);

    constexpr CallbackWithStorage() noexcept
        : invoker_(nullptr)
        , destroy_(nullptr)
        , size_(0) {}

    constexpr CallbackWithStorage(std::nullptr_t) noexcept
        : invoker_(nullptr)
        , destroy_(nullptr)
        , size_(0) {}

    template<typename F>
    CallbackWithStorage(F&& func) {
        using FuncType = std::decay_t<F>;
        static_assert(sizeof(FuncType) <= ContextSize,
                      "Lambda too large for CallbackWithStorage");
        static_assert(std::is_trivially_copyable<FuncType>::value,
                      "CallbackWithStorage requires trivially copyable lambdas "
                      "(no std::string, std::vector, etc. in capture)");
        new (storage_) FuncType(std::forward<F>(func));
        size_ = sizeof(FuncType);
        invoker_ = &invoke<FuncType>;
        destroy_ = &destroy<FuncType>;
    }

    CallbackWithStorage& operator=(std::nullptr_t) noexcept {
        reset();
        return *this;
    }

    CallbackWithStorage(const CallbackWithStorage& other) noexcept
        : invoker_(other.invoker_)
        , destroy_(other.destroy_)
        , size_(other.size_) {
        if (size_ > 0) {
            for (size_t i = 0; i < size_; ++i) {
                storage_[i] = other.storage_[i];
            }
        }
    }

    CallbackWithStorage& operator=(const CallbackWithStorage& other) noexcept {
        if (this != &other) {
            reset();
            invoker_ = other.invoker_;
            destroy_ = other.destroy_;
            size_ = other.size_;
            if (size_ > 0) {
                for (size_t i = 0; i < size_; ++i) {
                    storage_[i] = other.storage_[i];
                }
            }
        }
        return *this;
    }

    CallbackWithStorage(CallbackWithStorage&& other) noexcept
        : invoker_(other.invoker_)
        , destroy_(other.destroy_)
        , size_(other.size_) {
        if (size_ > 0) {
            for (size_t i = 0; i < size_; ++i) {
                storage_[i] = other.storage_[i];
            }
        }
        other.reset();
    }

    CallbackWithStorage& operator=(CallbackWithStorage&& other) noexcept {
        if (this != &other) {
            reset();
            invoker_ = other.invoker_;
            destroy_ = other.destroy_;
            size_ = other.size_;
            if (size_ > 0) {
                for (size_t i = 0; i < size_; ++i) {
                    storage_[i] = other.storage_[i];
                }
            }
            other.reset();
        }
        return *this;
    }

    ~CallbackWithStorage() {
        if (destroy_) {
            destroy_(storage_);
        }
    }

    constexpr bool isEmpty() const noexcept { return invoker_ == nullptr; }
    constexpr bool isValid() const noexcept { return invoker_ != nullptr; }
    constexpr explicit operator bool() const noexcept { return isValid(); }

    Ret call(Args... args) const {
        if (invoker_) {
            return invoker_(const_cast<void*>(reinterpret_cast<const void*>(storage_)), args...);
        }
        return Ret{};
    }

    Ret operator()(Args... args) const {
        return call(args...);
    }

    void reset() {
        if (destroy_) {
            destroy_(storage_);
        }
        size_ = 0;
        invoker_ = nullptr;
        destroy_ = nullptr;
    }

private:
    alignas(alignof(std::max_align_t)) uint8_t storage_[ContextSize];
    InvokerFunc invoker_;
    DestroyFunc destroy_;
    size_t size_;

    template<typename F>
    static Ret invoke(void* context, Args... args) {
        auto* func = reinterpret_cast<F*>(context);
        return (*func)(args...);
    }

    template<typename F>
    static void destroy(void* context) {
        reinterpret_cast<F*>(context)->~F();
    }
};

template<typename... Args, size_t ContextSize>
class CallbackWithStorage<void(Args...), ContextSize> {
public:
    using InvokerFunc = void (*)(void*, Args...);
    using DestroyFunc = void (*)(void*);

    constexpr CallbackWithStorage() noexcept
        : invoker_(nullptr)
        , destroy_(nullptr)
        , size_(0) {}

    constexpr CallbackWithStorage(std::nullptr_t) noexcept
        : invoker_(nullptr)
        , destroy_(nullptr)
        , size_(0) {}

    template<typename F>
    CallbackWithStorage(F&& func) {
        using FuncType = std::decay_t<F>;
        static_assert(sizeof(FuncType) <= ContextSize,
                      "Lambda too large for CallbackWithStorage");
        static_assert(std::is_trivially_copyable<FuncType>::value,
                      "CallbackWithStorage requires trivially copyable lambdas "
                      "(no std::string, std::vector, etc. in capture)");
        new (storage_) FuncType(std::forward<F>(func));
        size_ = sizeof(FuncType);
        invoker_ = &invoke<FuncType>;
        destroy_ = &destroy<FuncType>;
    }

    CallbackWithStorage& operator=(std::nullptr_t) noexcept {
        reset();
        return *this;
    }

    CallbackWithStorage(const CallbackWithStorage& other) noexcept
        : invoker_(other.invoker_)
        , destroy_(other.destroy_)
        , size_(other.size_) {
        if (size_ > 0) {
            for (size_t i = 0; i < size_; ++i) {
                storage_[i] = other.storage_[i];
            }
        }
    }

    CallbackWithStorage& operator=(const CallbackWithStorage& other) noexcept {
        if (this != &other) {
            reset();
            invoker_ = other.invoker_;
            destroy_ = other.destroy_;
            size_ = other.size_;
            if (size_ > 0) {
                for (size_t i = 0; i < size_; ++i) {
                    storage_[i] = other.storage_[i];
                }
            }
        }
        return *this;
    }

    CallbackWithStorage(CallbackWithStorage&& other) noexcept
        : invoker_(other.invoker_)
        , destroy_(other.destroy_)
        , size_(other.size_) {
        if (size_ > 0) {
            for (size_t i = 0; i < size_; ++i) {
                storage_[i] = other.storage_[i];
            }
        }
        other.reset();
    }

    CallbackWithStorage& operator=(CallbackWithStorage&& other) noexcept {
        if (this != &other) {
            reset();
            invoker_ = other.invoker_;
            destroy_ = other.destroy_;
            size_ = other.size_;
            if (size_ > 0) {
                for (size_t i = 0; i < size_; ++i) {
                    storage_[i] = other.storage_[i];
                }
            }
            other.reset();
        }
        return *this;
    }

    ~CallbackWithStorage() {
        if (destroy_) {
            destroy_(storage_);
        }
    }

    constexpr bool isEmpty() const noexcept { return invoker_ == nullptr; }
    constexpr bool isValid() const noexcept { return invoker_ != nullptr; }
    constexpr explicit operator bool() const noexcept { return isValid(); }

    void call(Args... args) const {
        if (invoker_) {
            invoker_(const_cast<void*>(reinterpret_cast<const void*>(storage_)), args...);
        }
    }

    void operator()(Args... args) const {
        call(args...);
    }

    void reset() {
        if (destroy_) {
            destroy_(storage_);
        }
        size_ = 0;
        invoker_ = nullptr;
        destroy_ = nullptr;
    }

private:
    alignas(alignof(std::max_align_t)) uint8_t storage_[ContextSize];
    InvokerFunc invoker_;
    DestroyFunc destroy_;
    size_t size_;

    template<typename F>
    static void invoke(void* context, Args... args) {
        auto* func = reinterpret_cast<F*>(context);
        (*func)(args...);
    }

    template<typename F>
    static void destroy(void* context) {
        reinterpret_cast<F*>(context)->~F();
    }
};

// ============================================================================
// Helper функции
// ============================================================================

template<typename Ret, typename... Args>
constexpr Callback<Ret(Args...)> makeCallback(Ret (*func)(Args...)) {
    return Callback<Ret(Args...)>(func);
}

template<typename F, typename Ret>
auto makeCallbackWithStorage(const F& func) {
    return CallbackWithStorage<Ret>(func);
}

template<typename F>
auto makeCallbackWithStorage(const F& func) {
    using Ret = std::invoke_result_t<const F&>;
    return CallbackWithStorage<Ret>(func);
}

} // namespace mka

#endif // MKA_CALLBACK_HPP
