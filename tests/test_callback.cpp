/**
 * @file test_callback.cpp
 * @brief Полные тесты для Callback utility
 * 
 * Покрывает:
 * - Callback с функциями
 * - Callback с лямбдами (без захвата)
 * - Callback с лямбдами (с захватом)
 * - Callback с методами классов
 * - Void и non-void Callback
 * - Edge cases
 */

#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <memory>

#include "utils/callback.hpp"

using namespace mka;

// ============================================================================
// Free Function Tests
// ============================================================================

static int addNumbers(int a, int b) {
    return a + b;
}

static void incrementCounter(int& counter) {
    counter++;
}

TEST(CallbackFunctionTest, FunctionWithReturn) {
    Callback<int(int, int)> cb(addNumbers);
    EXPECT_TRUE(cb.isValid());
    EXPECT_FALSE(cb.isEmpty());
    EXPECT_EQ(cb(10, 20), 30);
}

TEST(CallbackFunctionTest, VoidFunction) {
    int counter = 0;
    Callback<void(int&)> cb(incrementCounter);
    EXPECT_TRUE(cb.isValid());
    cb(counter);
    EXPECT_EQ(counter, 1);
}

TEST(CallbackFunctionTest, CallReturnsCorrectValue) {
    Callback<int(int, int)> cb(addNumbers);
    EXPECT_EQ(cb.call(5, 7), 12);
}

TEST(CallbackFunctionTest, ResetToNull) {
    Callback<int(int, int)> cb(addNumbers);
    EXPECT_TRUE(cb.isValid());
    
    cb.reset();
    EXPECT_FALSE(cb.isValid());
    EXPECT_TRUE(cb.isEmpty());
}

TEST(CallbackFunctionTest, AssignNull) {
    Callback<int(int, int)> cb(addNumbers);
    cb = nullptr;
    EXPECT_FALSE(cb.isValid());
}

// ============================================================================
// Lambda Tests (No Capture)
// ============================================================================

TEST(CallbackLambdaTest, LambdaWithReturn) {
    auto lambda = [](int a, int b) -> int {
        return a * b;
    };
    
    Callback<int(int, int)> cb(+lambda);
    EXPECT_TRUE(cb.isValid());
    EXPECT_EQ(cb(5, 6), 30);
}

TEST(CallbackLambdaTest, VoidLambda) {
    static int result = 0;
    auto lambda = [](int x) {
        result = x * 2;
    };
    
    Callback<void(int)> cb(+lambda);
    EXPECT_TRUE(cb.isValid());
    cb(21);
    EXPECT_EQ(result, 42);
}

// ============================================================================
// Lambda Tests (With Capture)
// ============================================================================

class CallbackWithCaptureTest : public ::testing::Test {
protected:
    int multiplier_ = 10;
};

TEST_F(CallbackWithCaptureTest, CapturingLambda) {
    auto lambda = [this](int x) -> int {
        return x * multiplier_;
    };
    
    // Для лямбд с захватом нужен CallbackWithStorage
    CallbackWithStorage<int(int), 64> cb(lambda);
    EXPECT_TRUE(cb.isValid());
    EXPECT_EQ(cb(5), 50);
    
    multiplier_ = 20;
    EXPECT_EQ(cb(5), 100);
}

TEST_F(CallbackWithCaptureTest, VoidCapturingLambda) {
    static std::vector<int> results;
    results.clear();
    
    auto lambda = [this](int x) {
        results.push_back(x * multiplier_);
    };
    
    CallbackWithStorage<void(int), 64> cb(lambda);
    EXPECT_TRUE(cb.isValid());
    cb(5);
    cb(10);
    
    EXPECT_EQ(results.size(), 2u);
    EXPECT_EQ(results[0], 50);
    EXPECT_EQ(results[1], 100);
}

// ============================================================================
// Member Function Tests
// ============================================================================

class Counter {
public:
    int count_ = 0;
    
    int increment() {
        return ++count_;
    }
    
    int add(int value) {
        count_ += value;
        return count_;
    }
    
    void reset() {
        count_ = 0;
    }
};

TEST(CallbackMemberTest, MemberFunction) {
    Counter counter;
    
    auto memberFunc = [](Counter* obj) -> int {
        return obj->increment();
    };
    
    Callback<int()> cb([memberFunc, &counter]() -> int {
        return memberFunc(&counter);
    });
    
    // Прямой вызов через лямбду
    EXPECT_EQ(counter.count_, 0);
    counter.increment();
    EXPECT_EQ(counter.count_, 1);
}

TEST(CallbackMemberTest, MemberFunctionWithParameter) {
    Counter counter;
    
    counter.add(10);
    EXPECT_EQ(counter.count_, 10);
    
    counter.add(5);
    EXPECT_EQ(counter.count_, 15);
}

// ============================================================================
// CallbackWithStorage Tests
// ============================================================================

TEST(CallbackWithStorageTest, SmallLambda) {
    CallbackWithStorage<int(), 32> cb([]() -> int {
        return 42;
    });
    
    EXPECT_TRUE(cb.isValid());
    EXPECT_EQ(cb(), 42);
}

TEST(CallbackWithStorageTest, MediumLambda) {
    std::vector<int> data = {1, 2, 3, 4, 5};
    
    CallbackWithStorage<int(), 64> cb([data]() -> int {
        int sum = 0;
        for (int x : data) sum += x;
        return sum;
    });
    
    EXPECT_TRUE(cb.isValid());
    EXPECT_EQ(cb(), 15);
}

TEST(CallbackWithStorageTest, StringCapture) {
    std::string prefix = "Hello, ";
    std::string name = "World";
    
    CallbackWithStorage<std::string(), 128> cb([prefix, name]() -> std::string {
        return prefix + name;
    });
    
    EXPECT_TRUE(cb.isValid());
    EXPECT_EQ(cb(), "Hello, World");
}

TEST(CallbackWithStorageTest, VoidLambdaWithCapture) {
    int result = 0;
    int multiplier = 7;
    
    CallbackWithStorage<void(), 64> cb([multiplier, &result]() {
        result = multiplier * 6;
    });
    
    EXPECT_TRUE(cb.isValid());
    cb();
    EXPECT_EQ(result, 42);
}

// ============================================================================
// Comparison Tests
// ============================================================================

TEST(CallbackComparisonTest, EqualsNull) {
    Callback<int()> empty_cb;
    EXPECT_EQ(empty_cb, nullptr);
    EXPECT_EQ(nullptr, empty_cb);
}

TEST(CallbackComparisonTest, NotEqualsNull) {
    Callback<int()> cb([]() -> int { return 0; });
    EXPECT_NE(cb, nullptr);
    EXPECT_NE(nullptr, cb);
}

// ============================================================================
// Bool Conversion Tests
// ============================================================================

TEST(CallbackBoolTest, EmptyIsFalse) {
    Callback<int()> cb;
    EXPECT_FALSE(cb);
    EXPECT_FALSE(static_cast<bool>(cb));
}

TEST(CallbackBoolTest, ValidIsTrue) {
    Callback<int()> cb([]() -> int { return 0; });
    EXPECT_TRUE(cb);
    EXPECT_TRUE(static_cast<bool>(cb));
}

TEST(CallbackBoolTest, ResetBecomesFalse) {
    Callback<int()> cb([]() -> int { return 0; });
    EXPECT_TRUE(cb);
    
    cb.reset();
    EXPECT_FALSE(cb);
}

// ============================================================================
// Edge Cases Tests
// ============================================================================

TEST(CallbackEdgeCasesTest, CallEmptyCallback) {
    Callback<int()> cb;
    // Вызов пустого callback должен вернуть default значение
    EXPECT_EQ(cb(), 0);
}

TEST(CallbackEdgeCasesTest, CallEmptyVoidCallback) {
    Callback<void()> cb;
    // Вызов пустого void callback должен быть безопасным
    cb();  // Не должно крашиться
}

TEST(CallbackEdgeCasesTest, MultipleCalls) {
    static int callCount = 0;
    Callback<int()> cb([]() -> int {
        return ++callCount;
    });
    
    EXPECT_EQ(cb(), 1);
    EXPECT_EQ(cb(), 2);
    EXPECT_EQ(cb(), 3);
}

TEST(CallbackEdgeCasesTest, ReassignCallback) {
    Callback<int()> cb([]() -> int { return 1; });
    EXPECT_EQ(cb(), 1);
    
    cb = Callback<int()>();
    EXPECT_FALSE(cb.isValid());
}

// ============================================================================
// Performance Tests
// ============================================================================

TEST(CallbackPerformanceTest, ManyCalls) {
    static int counter = 0;
    Callback<int()> cb([]() -> int {
        return ++counter;
    });
    
    for (int i = 0; i < 10000; ++i) {
        cb();
    }
    
    EXPECT_EQ(counter, 10000);
}

TEST(CallbackPerformanceTest, StorageManyCalls) {
    int counter = 0;
    CallbackWithStorage<void(), 64> cb([&counter]() {
        ++counter;
    });
    
    for (int i = 0; i < 10000; ++i) {
        cb();
    }
    
    EXPECT_EQ(counter, 10000);
}

// ============================================================================
// Complex Type Tests
// ============================================================================

TEST(CallbackComplexTest, StringReturn) {
    Callback<std::string()> cb([]() -> std::string {
        return "Hello from callback!";
    });
    
    EXPECT_TRUE(cb.isValid());
    EXPECT_EQ(cb(), "Hello from callback!");
}

TEST(CallbackComplexTest, VectorReturn) {
    Callback<std::vector<int>()> cb([]() -> std::vector<int> {
        return {1, 2, 3, 4, 5};
    });
    
    EXPECT_TRUE(cb.isValid());
    auto result = cb();
    EXPECT_EQ(result.size(), 5u);
    EXPECT_EQ(result[0], 1);
    EXPECT_EQ(result[4], 5);
}

TEST(CallbackComplexTest, UniquePtrReturn) {
    Callback<std::unique_ptr<int>()> cb([]() -> std::unique_ptr<int> {
        return std::make_unique<int>(42);
    });
    
    EXPECT_TRUE(cb.isValid());
    auto ptr = cb();
    EXPECT_NE(ptr, nullptr);
    EXPECT_EQ(*ptr, 42);
}

// ============================================================================
// Static Assertion Tests
// ============================================================================

TEST(CallbackStaticTest, CallbackSize) {
    // Callback должен быть маленьким (2 pointer'а)
    EXPECT_LE(sizeof(Callback<int()>), sizeof(void*) * 2);
}

TEST(CallbackStaticTest, CallbackWithStorageSize) {
    // CallbackWithStorage должен соответствовать заявленному размеру
    EXPECT_EQ(sizeof(CallbackWithStorage<int(), 64>), 64 + sizeof(void*));
}
