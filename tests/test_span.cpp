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
    span<int> s;
    EXPECT_EQ(s.size(), 0u);
    EXPECT_TRUE(s.empty());
    EXPECT_EQ(s.data(), nullptr);
}

TEST(SpanTest, ConstructionFromPointerAndSize) {
    int arr[] = {1, 2, 3, 4, 5};
    span<int> s(arr, 5);

    EXPECT_EQ(s.size(), 5u);
    EXPECT_FALSE(s.empty());
    EXPECT_EQ(s.data(), arr);
}

TEST(SpanTest, ConstructionFromArray) {
    int arr[] = {10, 20, 30};
    span<int> s(arr);

    EXPECT_EQ(s.size(), 3u);
    EXPECT_EQ(s[0], 10);
    EXPECT_EQ(s[1], 20);
    EXPECT_EQ(s[2], 30);
}

TEST(SpanTest, ConstructionFromVector) {
    std::vector<int> vec = {100, 200, 300};
    span<int> s(vec);

    EXPECT_EQ(s.size(), 3u);
    EXPECT_EQ(s[0], 100);
    EXPECT_EQ(s[1], 200);
    EXPECT_EQ(s[2], 300);
}

TEST(SpanTest, ConstructionFromStdArray) {
    std::array<int, 4> arr = {1, 2, 3, 4};
    span<int> s(arr);

    EXPECT_EQ(s.size(), 4u);
    EXPECT_EQ(s[0], 1);
    EXPECT_EQ(s[3], 4);
}

// ============================================================================
// Element Access Tests
// ============================================================================

TEST(SpanTest, ElementAccess) {
    int arr[] = {10, 20, 30, 40, 50};
    span<int> s(arr);

    EXPECT_EQ(s[0], 10);
    EXPECT_EQ(s[1], 20);
    EXPECT_EQ(s[2], 30);
    EXPECT_EQ(s[3], 40);
    EXPECT_EQ(s[4], 50);
}

TEST(SpanTest, FrontAndBack) {
    int arr[] = {1, 2, 3, 4, 5};
    span<int> s(arr);

    EXPECT_EQ(s.front(), 1);
    EXPECT_EQ(s.back(), 5);
}

TEST(SpanTest, EmptySpan) {
    span<int> s;
    EXPECT_TRUE(s.empty());
    EXPECT_EQ(s.size(), 0u);
}

// ============================================================================
// Subspan Tests
// ============================================================================

TEST(SpanTest, SubspanFromBeginning) {
    int arr[] = {1, 2, 3, 4, 5};
    span<int> s(arr);

    auto sub = s.subspan(0, 3);
    EXPECT_EQ(sub.size(), 3u);
    EXPECT_EQ(sub[0], 1);
    EXPECT_EQ(sub[1], 2);
    EXPECT_EQ(sub[2], 3);
}

TEST(SpanTest, SubspanFromMiddle) {
    int arr[] = {1, 2, 3, 4, 5};
    span<int> s(arr);

    auto sub = s.subspan(1, 3);
    EXPECT_EQ(sub.size(), 3u);
    EXPECT_EQ(sub[0], 2);
    EXPECT_EQ(sub[1], 3);
    EXPECT_EQ(sub[2], 4);
}

TEST(SpanTest, SubspanToEnd) {
    int arr[] = {1, 2, 3, 4, 5};
    span<int> s(arr);

    auto sub = s.subspan(2);
    EXPECT_EQ(sub.size(), 3u);
    EXPECT_EQ(sub[0], 3);
    EXPECT_EQ(sub[1], 4);
    EXPECT_EQ(sub[2], 5);
}

TEST(SpanTest, SubspanOutOfBounds) {
    int arr[] = {1, 2, 3};
    span<int> s(arr);

    // Должно вернуть пустой span или обрезать
    auto sub = s.subspan(5, 10);
    EXPECT_TRUE(sub.empty());
}

// ============================================================================
// First and Last Tests
// ============================================================================

TEST(SpanTest, FirstElements) {
    int arr[] = {1, 2, 3, 4, 5};
    span<int> s(arr);

    auto first3 = s.first(3);
    EXPECT_EQ(first3.size(), 3u);
    EXPECT_EQ(first3[0], 1);
    EXPECT_EQ(first3[2], 3);
}

TEST(SpanTest, LastElements) {
    int arr[] = {1, 2, 3, 4, 5};
    span<int> s(arr);

    auto last3 = s.last(3);
    EXPECT_EQ(last3.size(), 3u);
    EXPECT_EQ(last3[0], 3);
    EXPECT_EQ(last3[2], 5);
}

// ============================================================================
// Iterator Tests
// ============================================================================

TEST(SpanTest, Iteration) {
    int arr[] = {10, 20, 30};
    span<int> s(arr);

    int sum = 0;
    for (int val : s) {
        sum += val;
    }
    EXPECT_EQ(sum, 60);
}

TEST(SpanTest, BeginAndEnd) {
    int arr[] = {1, 2, 3};
    span<int> s(arr);

    EXPECT_EQ(*s.begin(), 1);
    EXPECT_EQ(*s.rbegin(), 3);
}

// ============================================================================
// Const Span Tests
// ============================================================================

TEST(SpanTest, ConstSpan) {
    int arr[] = {100, 200, 300};
    span<const int> s(arr);

    EXPECT_EQ(s.size(), 3u);
    EXPECT_EQ(s[0], 100);
    EXPECT_EQ(s[1], 200);
    EXPECT_EQ(s[2], 300);
}

// ============================================================================
// Mutable Access Tests
// ============================================================================

TEST(SpanTest, MutableAccess) {
    int arr[] = {1, 2, 3};
    span<int> s(arr);

    s[0] = 10;
    s[1] = 20;
    s[2] = 30;

    EXPECT_EQ(arr[0], 10);
    EXPECT_EQ(arr[1], 20);
    EXPECT_EQ(arr[2], 30);
}

TEST(SpanTest, DataModificationThroughSpan) {
    std::vector<int> vec = {1, 2, 3, 4, 5};
    span<int> s(vec);

    // Модифицировать через span
    for (size_t i = 0; i < s.size(); ++i) {
        s[i] *= 2;
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
    span<int> s(&value, 1);

    EXPECT_EQ(s.size(), 1u);
    EXPECT_EQ(s[0], 42);
}

TEST(SpanTest, ZeroSizedSpan) {
    int arr[] = {1, 2, 3};
    span<int> s(arr, static_cast<size_t>(0));

    EXPECT_TRUE(s.empty());
    EXPECT_EQ(s.size(), 0u);
}

TEST(SpanTest, SpanOfSpans) {
    int arr1[] = {1, 2};
    int arr2[] = {3, 4};

    span<int> s1(arr1);
    span<int> s2(arr2);

    // Span можно создавать из других span
    EXPECT_EQ(s1.size(), 2u);
    EXPECT_EQ(s2.size(), 2u);
}
