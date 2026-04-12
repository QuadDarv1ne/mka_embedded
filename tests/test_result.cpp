/**
 * @file test_result.cpp
 * @brief Полные тесты для Result<T, E> utility
 * 
 * Покрывает:
 * - Создание Ok и Error значений
 * - Доступ к value и error
 * - Методы unwrap, expect, valueOr
 * - Методы map, andThen, orElse
 * - Копирование и перемещение
 * - Edge cases (void, большие типы)
 */

#include <gtest/gtest.h>
#include <string>
#include <memory>
#include <vector>
#include <cmath>

#include "utils/result.hpp"

using namespace mka;

// ============================================================================
// Basic Construction Tests
// ============================================================================

TEST(ResultTest, OkConstruction) {
    Result<int, std::string> result(42);
    EXPECT_TRUE(result.isOk());
    EXPECT_FALSE(result.isError());
    EXPECT_EQ(result.value(), 42);
}

TEST(ResultTest, ErrorConstruction) {
    Result<int, std::string> result(std::string("error"));
    EXPECT_TRUE(result.isError());
    EXPECT_FALSE(result.isOk());
    EXPECT_EQ(result.error(), "error");
}

TEST(ResultTest, DefaultConstruction) {
    Result<int, std::string> result;
    EXPECT_TRUE(result.isError());
}

TEST(ResultTest, BoolConversion) {
    Result<int, std::string> ok_result(42);
    Result<int, std::string> err_result(std::string("error"));
    
    EXPECT_TRUE(ok_result);
    EXPECT_FALSE(err_result);
}

// ============================================================================
// Value Access Tests
// ============================================================================

TEST(ResultTest, GetValue) {
    Result<int, std::string> result(42);
    EXPECT_EQ(result.value(), 42);
}

TEST(ResultTest, GetError) {
    Result<int, std::string> result(std::string("error"));
    EXPECT_EQ(result.error(), "error");
}

TEST(ResultTest, ValueOr) {
    Result<int, std::string> ok_result(42);
    Result<int, std::string> err_result(std::string("error"));
    
    EXPECT_EQ(ok_result.valueOr(100), 42);
    EXPECT_EQ(err_result.valueOr(100), 100);
}

TEST(ResultTest, UnwrapOk) {
    Result<int, std::string> result(42);
    EXPECT_EQ(result.unwrap(), 42);
}

TEST(ResultTest, UnwrapError) {
    Result<int, std::string> result(std::string("test error"));
    EXPECT_DEATH_IF_SUPPORTED(result.unwrap(), "");
}

TEST(ResultTest, ExpectOk) {
    Result<int, std::string> result(42);
    EXPECT_EQ(result.expect("should be ok"), 42);
}

TEST(ResultTest, ExpectError) {
    Result<int, std::string> result(std::string("fatal error"));
    EXPECT_DEATH_IF_SUPPORTED(result.expect("should fail"), "");
}

// ============================================================================
// Copy and Move Tests
// ============================================================================

TEST(ResultTest, CopyOkValue) {
    Result<int, std::string> original(42);
    auto copy = original;
    
    EXPECT_TRUE(copy.isOk());
    EXPECT_EQ(copy.value(), 42);
    EXPECT_EQ(original.value(), 42);
}

TEST(ResultTest, CopyErrorValue) {
    Result<int, std::string> original(std::string("error"));
    auto copy = original;
    
    EXPECT_TRUE(copy.isError());
    EXPECT_EQ(copy.error(), "error");
}

TEST(ResultTest, MoveOkValue) {
    Result<std::string, int> result(std::string("moved"));
    auto moved = std::move(result);
    
    EXPECT_TRUE(moved.isOk());
    EXPECT_EQ(moved.value(), "moved");
}

TEST(ResultTest, MoveErrorValue) {
    Result<int, std::string> result(std::string("moved error"));
    auto moved = std::move(result);
    
    EXPECT_TRUE(moved.isError());
    EXPECT_EQ(moved.error(), "moved error");
}

// ============================================================================
// Assignment Tests
// ============================================================================

TEST(ResultTest, AssignOkToOk) {
    Result<int, std::string> result1(42);
    Result<int, std::string> result2(100);
    
    result1 = result2;
    EXPECT_EQ(result1.value(), 100);
}

TEST(ResultTest, AssignErrorToOk) {
    Result<int, std::string> result1(42);
    Result<int, std::string> result2(std::string("error"));
    
    result1 = result2;
    EXPECT_TRUE(result1.isError());
    EXPECT_EQ(result1.error(), "error");
}

TEST(ResultTest, AssignOkToError) {
    Result<int, std::string> result1(std::string("error"));
    Result<int, std::string> result2(42);
    
    result1 = result2;
    EXPECT_TRUE(result1.isOk());
    EXPECT_EQ(result1.value(), 42);
}

TEST(ResultTest, SelfAssignment) {
    Result<int, std::string> result(42);
    result = result;
    EXPECT_TRUE(result.isOk());
    EXPECT_EQ(result.value(), 42);
}

// ============================================================================
// Map and Then Tests
// ============================================================================

TEST(ResultTest, MapSuccess) {
    Result<int, std::string> result(42);
    auto mapped = result.map([](int v) { return v * 2; });
    
    EXPECT_TRUE(mapped.isOk());
    EXPECT_EQ(mapped.value(), 84);
}

TEST(ResultTest, MapError) {
    Result<int, std::string> result(std::string("error"));
    auto mapped = result.map([](int v) { return v * 2; });
    
    EXPECT_TRUE(mapped.isError());
    EXPECT_EQ(mapped.error(), "error");
}

TEST(ResultTest, AndThenSuccess) {
    Result<int, std::string> result(42);
    auto chained = result.andThen([](int v) -> Result<int, std::string> {
        if (v > 0) return Result<int, std::string>(v * 2);
        return Result<int, std::string>(std::string("negative"));
    });
    
    EXPECT_TRUE(chained.isOk());
    EXPECT_EQ(chained.value(), 84);
}

TEST(ResultTest, AndThenError) {
    Result<int, std::string> result(-42);
    auto chained = result.andThen([](int v) -> Result<int, std::string> {
        if (v > 0) return Result<int, std::string>(v * 2);
        return Result<int, std::string>(std::string("negative"));
    });
    
    EXPECT_TRUE(chained.isError());
    EXPECT_EQ(chained.error(), "negative");
}

TEST(ResultTest, OrElseSuccess) {
    Result<int, std::string> result(42);
    auto fallback = result.orElse([](const std::string& e) -> Result<int, std::string> {
        (void)e;
        return Result<int, std::string>(0);
    });
    
    EXPECT_TRUE(fallback.isOk());
    EXPECT_EQ(fallback.value(), 42);
}

TEST(ResultTest, OrElseError) {
    Result<int, std::string> result(std::string("original error"));
    auto fallback = result.orElse([](const std::string& e) -> Result<int, std::string> {
        return Result<int, std::string>(42);
    });
    
    EXPECT_TRUE(fallback.isOk());
    EXPECT_EQ(fallback.value(), 42);
}

// ============================================================================
// MapError Tests
// ============================================================================

TEST(ResultTest, MapErrorSuccess) {
    Result<int, std::string> result(42);
    auto mapped = result.mapError([](const std::string& e) {
        return e + " (mapped)";
    });
    
    EXPECT_TRUE(mapped.isOk());
    EXPECT_EQ(mapped.value(), 42);
}

TEST(ResultTest, MapErrorError) {
    Result<int, std::string> result(std::string("error"));
    auto mapped = result.mapError([](const std::string& e) {
        return e + " (mapped)";
    });
    
    EXPECT_TRUE(mapped.isError());
    EXPECT_EQ(mapped.error(), "error (mapped)");
}

// ============================================================================
// Void Result Tests
// ============================================================================

TEST(ResultVoidTest, VoidOk) {
    Result<void, std::string> result;
    // void Result по умолчанию Error
    EXPECT_TRUE(result.isError());
}

TEST(ResultVoidTest, VoidOkConstruction) {
    // Для void нужно использовать Ok<void>()
    auto result = Ok<void, std::string>();
    EXPECT_TRUE(result.isOk());
}

TEST(ResultVoidTest, VoidError) {
    Result<void, std::string> result(std::string("error"));
    EXPECT_TRUE(result.isError());
    EXPECT_EQ(result.error(), "error");
}

// ============================================================================
// Complex Type Tests
// ============================================================================

TEST(ResultComplexTest, StringResult) {
    Result<std::string, int> result(std::string("hello"));
    EXPECT_TRUE(result.isOk());
    EXPECT_EQ(result.value(), "hello");
}

TEST(ResultComplexTest, VectorResult) {
    std::vector<int> vec = {1, 2, 3};
    Result<std::vector<int>, int> result(vec);
    EXPECT_TRUE(result.isOk());
    EXPECT_EQ(result.value().size(), 3u);
}

TEST(ResultComplexTest, UniquePtrResult) {
    auto ptr = std::make_unique<int>(42);
    Result<std::unique_ptr<int>, int> result(std::move(ptr));
    EXPECT_TRUE(result.isOk());
    EXPECT_EQ(*result.value(), 42);
}

// ============================================================================
// Numeric Type Tests
// ============================================================================

TEST(ResultNumericTest, FloatResult) {
    Result<float, int> result(3.14f);
    EXPECT_TRUE(result.isOk());
    EXPECT_NEAR(result.value(), 3.14f, 0.001f);
}

TEST(ResultNumericTest, DoubleResult) {
    Result<double, int> result(2.718281828);
    EXPECT_TRUE(result.isOk());
    EXPECT_NEAR(result.value(), 2.718281828, 1e-9);
}

TEST(ResultNumericTest, NaNResult) {
    Result<double, int> result(std::numeric_limits<double>::quiet_NaN());
    EXPECT_TRUE(result.isOk());
    EXPECT_TRUE(std::isnan(result.value()));
}

// ============================================================================
// Chaining Tests
// ============================================================================

TEST(ResultChainingTest, ChainMultipleOperations) {
    Result<int, std::string> result(10);
    
    auto final_result = result
        .map([](int v) { return v + 5; })
        .map([](int v) { return v * 2; })
        .map([](int v) { return v - 10; });
    
    EXPECT_TRUE(final_result.isOk());
    EXPECT_EQ(final_result.value(), 20);  // ((10 + 5) * 2) - 10 = 20
}

TEST(ResultChainingTest, ChainWithError) {
    Result<int, std::string> result(std::string("initial error"));
    
    auto final_result = result
        .map([](int v) { return v + 5; })
        .map([](int v) { return v * 2; })
        .map([](int v) { return v - 10; });
    
    EXPECT_TRUE(final_result.isError());
    EXPECT_EQ(final_result.error(), "initial error");
}

// ============================================================================
// Edge Cases Tests
// ============================================================================

TEST(ResultEdgeCasesTest, ZeroValue) {
    Result<int, int> result(0);
    EXPECT_TRUE(result.isOk());
    EXPECT_EQ(result.value(), 0);
}

TEST(ResultEdgeCasesTest, NegativeError) {
    Result<int, int> result(-1);
    // Это Ok(-1), не ошибка
    EXPECT_TRUE(result.isOk());
    EXPECT_EQ(result.value(), -1);
}

TEST(ResultEdgeCasesTest, LargeValue) {
    Result<int64_t, int> result(9223372036854775807LL);
    EXPECT_TRUE(result.isOk());
    EXPECT_EQ(result.value(), 9223372036854775807LL);
}

TEST(ResultEdgeCasesTest, EmptyStringError) {
    Result<int, std::string> result(std::string(""));
    EXPECT_TRUE(result.isError());
    EXPECT_EQ(result.error(), "");
}

TEST(ResultEdgeCasesTest, BoolResult) {
    Result<bool, int> result(true);
    EXPECT_TRUE(result.isOk());
    EXPECT_EQ(result.value(), true);
}

// ============================================================================
// Ok/Err Helper Functions Tests
// ============================================================================

TEST(ResultHelpersTest, OkHelper) {
    auto result = Ok<int, std::string>(42);
    EXPECT_TRUE(result.isOk());
    EXPECT_EQ(result.value(), 42);
}

TEST(ResultHelpersTest, ErrHelper) {
    auto result = Err<int, std::string>(std::string("error"));
    EXPECT_TRUE(result.isError());
    EXPECT_EQ(result.error(), "error");
}

// ============================================================================
// Try Methods Tests
// ============================================================================

TEST(ResultTryMethodsTest, TryValueOk) {
    Result<int, std::string> result(42);
    auto value = result.tryValue();
    EXPECT_TRUE(value.has_value());
    EXPECT_EQ(value.value(), 42);
}

TEST(ResultTryMethodsTest, TryValueError) {
    Result<int, std::string> result(std::string("error"));
    auto value = result.tryValue();
    EXPECT_FALSE(value.has_value());
}

TEST(ResultTryMethodsTest, TryErrorOk) {
    Result<int, std::string> result(42);
    auto error = result.tryError();
    EXPECT_FALSE(error.has_value());
}

TEST(ResultTryMethodsTest, TryErrorError) {
    Result<int, std::string> result(std::string("error"));
    auto error = result.tryError();
    EXPECT_TRUE(error.has_value());
    EXPECT_EQ(error.value(), "error");
}
