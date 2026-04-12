/**
 * @file test_span.cpp
 * @brief Тесты для span utility - безопасная работа с памятью
 */

#include <gtest/gtest.h>
#include <vector>
#include <array>

#include "utils/span.hpp"

using namespace mka;

// ============================================================================
// Basic Span Tests
// ============================================================================

TEST(SpanTest, DefaultConstruction) {
    Span<int> span;
    EXPECT_EQ(span.size(), 0u);
    EXPECT_TRUE(span.empty());
    EXPECT_EQ(span.data(), nullptr);
}

TEST(SpanTest, ConstructionFromPointerAndSize) {
    int arr[] = {1, 2, 3, 4, 5};
    Span<int> span(arr, 5);
    
    EXPECT_EQ(span.size(), 5u);
    EXPECT_FALSE(span.empty());
    EXPECT_EQ(span.data(), arr);
}

TEST(SpanTest, ConstructionFromArray) {
    int arr[] = {10, 20, 30};
    Span<int> span(arr);
    
    EXPECT_EQ(span.size(), 3u);
    EXPECT_EQ(span[0], 10);
    EXPECT_EQ(span[1], 20);
    EXPECT_EQ(span[2], 30);
}

TEST(SpanTest, ConstructionFromVector) {
    std::vector<int> vec = {100, 200, 300};
    Span<int> span(vec);
    
    EXPECT_EQ(span.size(), 3u);
    EXPECT_EQ(span[0], 100);
    EXPECT_EQ(span[1], 200);
    EXPECT_EQ(span[2], 300);
}

TEST(SpanTest, ConstructionFromStdArray) {
    std::array<int, 4> arr = {1, 2, 3, 4};
    Span<int> span(arr);
    
    EXPECT_EQ(span.size(), 4u);
    EXPECT_EQ(span[0], 1);
    EXPECT_EQ(span[3], 4);
}

// ============================================================================
// Element Access Tests
// ============================================================================

TEST(SpanTest, ElementAccess) {
    int arr[] = {10, 20, 30, 40, 50};
    Span<int> span(arr);
    
    EXPECT_EQ(span[0], 10);
    EXPECT_EQ(span[1], 20);
    EXPECT_EQ(span[2], 30);
    EXPECT_EQ(span[3], 40);
    EXPECT_EQ(span[4], 50);
}

TEST(SpanTest, FrontAndBack) {
    int arr[] = {1, 2, 3, 4, 5};
    Span<int> span(arr);
    
    EXPECT_EQ(span.front(), 1);
    EXPECT_EQ(span.back(), 5);
}

TEST(SpanTest, EmptySpan) {
    Span<int> span;
    EXPECT_TRUE(span.empty());
    EXPECT_EQ(span.size(), 0u);
}

// ============================================================================
// Subspan Tests
// ============================================================================

TEST(SpanTest, SubspanFromBeginning) {
    int arr[] = {1, 2, 3, 4, 5};
    Span<int> span(arr);
    
    auto sub = span.subspan(0, 3);
    EXPECT_EQ(sub.size(), 3u);
    EXPECT_EQ(sub[0], 1);
    EXPECT_EQ(sub[1], 2);
    EXPECT_EQ(sub[2], 3);
}

TEST(SpanTest, SubspanFromMiddle) {
    int arr[] = {1, 2, 3, 4, 5};
    Span<int> span(arr);
    
    auto sub = span.subspan(1, 3);
    EXPECT_EQ(sub.size(), 3u);
    EXPECT_EQ(sub[0], 2);
    EXPECT_EQ(sub[1], 3);
    EXPECT_EQ(sub[2], 4);
}

TEST(SpanTest, SubspanToEnd) {
    int arr[] = {1, 2, 3, 4, 5};
    Span<int> span(arr);
    
    auto sub = span.subspan(2);
    EXPECT_EQ(sub.size(), 3u);
    EXPECT_EQ(sub[0], 3);
    EXPECT_EQ(sub[1], 4);
    EXPECT_EQ(sub[2], 5);
}

TEST(SpanTest, SubspanOutOfBounds) {
    int arr[] = {1, 2, 3};
    Span<int> span(arr);
    
    // Должно вернуть пустой span или обрезать
    auto sub = span.subspan(5, 10);
    EXPECT_TRUE(sub.empty());
}

// ============================================================================
// First and Last Tests
// ============================================================================

TEST(SpanTest, FirstElements) {
    int arr[] = {1, 2, 3, 4, 5};
    Span<int> span(arr);
    
    auto first3 = span.first(3);
    EXPECT_EQ(first3.size(), 3u);
    EXPECT_EQ(first3[0], 1);
    EXPECT_EQ(first3[2], 3);
}

TEST(SpanTest, LastElements) {
    int arr[] = {1, 2, 3, 4, 5};
    Span<int> span(arr);
    
    auto last3 = span.last(3);
    EXPECT_EQ(last3.size(), 3u);
    EXPECT_EQ(last3[0], 3);
    EXPECT_EQ(last3[2], 5);
}

// ============================================================================
// Iterator Tests
// ============================================================================

TEST(SpanTest, Iteration) {
    int arr[] = {10, 20, 30};
    Span<int> span(arr);
    
    int sum = 0;
    for (int val : span) {
        sum += val;
    }
    EXPECT_EQ(sum, 60);
}

TEST(SpanTest, BeginAndEnd) {
    int arr[] = {1, 2, 3};
    Span<int> span(arr);
    
    EXPECT_EQ(*span.begin(), 1);
    EXPECT_EQ(*span.rbegin(), 3);
}

// ============================================================================
// Const Span Tests
// ============================================================================

TEST(SpanTest, ConstSpan) {
    const int arr[] = {100, 200, 300};
    Span<const int> span(arr);
    
    EXPECT_EQ(span.size(), 3u);
    EXPECT_EQ(span[0], 100);
    EXPECT_EQ(span[1], 200);
    EXPECT_EQ(span[2], 300);
}

// ============================================================================
// Mutable Access Tests
// ============================================================================

TEST(SpanTest, MutableAccess) {
    int arr[] = {1, 2, 3};
    Span<int> span(arr);
    
    span[0] = 10;
    span[1] = 20;
    span[2] = 30;
    
    EXPECT_EQ(arr[0], 10);
    EXPECT_EQ(arr[1], 20);
    EXPECT_EQ(arr[2], 30);
}

TEST(SpanTest, DataModificationThroughSpan) {
    std::vector<int> vec = {1, 2, 3, 4, 5};
    Span<int> span(vec);
    
    // Модифицировать через span
    for (size_t i = 0; i < span.size(); ++i) {
        span[i] *= 2;
    }
    
    // Проверить что вектор изменился
    EXPECT_EQ(vec[0], 2);
    EXPECT_EQ(vec[1], 4);
    EXPECT_EQ(vec[2], 6);
    EXPECT_EQ(vec[3], 8);
    EXPECT_EQ(vec[4], 10);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST(SpanTest, SingleElement) {
    int value = 42;
    Span<int> span(&value, 1);
    
    EXPECT_EQ(span.size(), 1u);
    EXPECT_EQ(span[0], 42);
}

TEST(SpanTest, ZeroSizedSpan) {
    int arr[] = {1, 2, 3};
    Span<int> span(arr, 0);
    
    EXPECT_TRUE(span.empty());
    EXPECT_EQ(span.size(), 0u);
}

TEST(SpanTest, SpanOfSpans) {
    int arr1[] = {1, 2};
    int arr2[] = {3, 4};
    
    Span<int> span1(arr1);
    Span<int> span2(arr2);
    
    // Span можно создавать из других span
    EXPECT_EQ(span1.size(), 2u);
    EXPECT_EQ(span2.size(), 2u);
}
