/**
 * @file test_utils.cpp
 * @brief Unit tests for utility classes (Result, Callback)
 */

#include <gtest/gtest.h>
#include "utils/result.hpp"
#include "utils/callback.hpp"

using namespace mka;

// ============================================================================
// Result Tests
// ============================================================================

TEST(ResultTest, OkConstruction) {
    Result<int, ErrorCode> result = Ok<int, ErrorCode>(42);
    EXPECT_TRUE(result.isOk());
    EXPECT_FALSE(result.isError());
    EXPECT_EQ(result.value(), 42);
}

TEST(ResultTest, ErrConstruction) {
    Result<int, ErrorCode> result = Err<int, ErrorCode>(ErrorCode::TIMEOUT);
    EXPECT_FALSE(result.isOk());
    EXPECT_TRUE(result.isError());
    EXPECT_EQ(result.error(), ErrorCode::TIMEOUT);
}

TEST(ResultTest, BoolConversion) {
    Result<int, ErrorCode> ok_result = Ok<int, ErrorCode>(10);
    Result<int, ErrorCode> err_result = Err<int, ErrorCode>(ErrorCode::ERROR);

    EXPECT_TRUE(ok_result);
    EXPECT_FALSE(err_result);
}

TEST(ResultTest, ValueOr) {
    Result<int, ErrorCode> ok_result = Ok<int, ErrorCode>(42);
    Result<int, ErrorCode> err_result = Err<int, ErrorCode>(ErrorCode::NOT_FOUND);

    EXPECT_EQ(ok_result.valueOr(0), 42);
    EXPECT_EQ(err_result.valueOr(0), 0);
}

TEST(ResultTest, MapSuccess) {
    Result<int, ErrorCode> result = Ok<int, ErrorCode>(5);
    auto mapped = result.map([](int x) { return x * 2; });

    EXPECT_TRUE(mapped.isOk());
    EXPECT_EQ(mapped.value(), 10);
}

TEST(ResultTest, MapError) {
    Result<int, ErrorCode> result = Err<int, ErrorCode>(ErrorCode::TIMEOUT);
    auto mapped = result.map([](int x) { return x * 2; });

    EXPECT_FALSE(mapped.isOk());
    EXPECT_EQ(mapped.error(), ErrorCode::TIMEOUT);
}

TEST(ResultTest, VoidResult) {
    Result<void, ErrorCode> ok_result = Ok<void, ErrorCode>();
    Result<void, ErrorCode> err_result = Err<void, ErrorCode>(ErrorCode::ERROR);

    EXPECT_TRUE(ok_result.isOk());
    EXPECT_FALSE(err_result.isOk());
}

TEST(ResultTest, ErrorCodeToString) {
    EXPECT_STREQ(errorCodeToString(ErrorCode::OK), "OK");
    EXPECT_STREQ(errorCodeToString(ErrorCode::TIMEOUT), "TIMEOUT");
    EXPECT_STREQ(errorCodeToString(ErrorCode::NOT_FOUND), "NOT_FOUND");
}

// ============================================================================
// Callback Tests
// ============================================================================

TEST(CallbackTest, EmptyCallback) {
    Callback<void(int)> cb;
    EXPECT_FALSE(cb.isValid());
    EXPECT_TRUE(cb.isEmpty());
}

TEST(CallbackTest, FunctionCallback) {
    auto lambda = [](int x) { return x * 2; };
    Callback<int(int)> cb(lambda);

    EXPECT_TRUE(cb.isValid());
    EXPECT_EQ(cb.call(5), 10);
    EXPECT_EQ(cb(7), 14);
}

TEST(CallbackTest, VoidCallback) {
    int counter = 0;
    auto lambda = [&counter]() { counter++; };
    Callback<void()> cb(lambda);

    cb.call();
    cb.call();
    EXPECT_EQ(counter, 2);
}

TEST(CallbackTest, CallbackReset) {
    auto lambda = [](int x) { return x; };
    Callback<int(int)> cb(lambda);

    EXPECT_TRUE(cb.isValid());
    cb.reset();
    EXPECT_FALSE(cb.isValid());
}

TEST(CallbackTest, BoolOperator) {
    Callback<int()> empty_cb;
    Callback<int()> valid_cb([]() { return 42; });

    EXPECT_FALSE(empty_cb);
    EXPECT_TRUE(valid_cb);
}

TEST(CallbackTest, WithStorage) {
    int multiplier = 3;
    auto lambda = [multiplier](int x) { return x * multiplier; };

    auto cb = makeCallbackWithStorage(lambda);

    EXPECT_TRUE(cb.isValid());
    EXPECT_EQ(cb(5), 15);
}

TEST(CallbackTest, WithStorageVoid) {
    int value = 0;
    auto lambda = [&value](int x) { value = x; };

    auto cb = makeCallbackWithStorage(lambda);

    cb(42);
    EXPECT_EQ(value, 42);
}

TEST(CallbackTest, MakeCallback) {
    auto func = [](int x, int y) { return x + y; };
    auto cb = makeCallback(func);

    EXPECT_EQ(cb(3, 4), 7);
}
