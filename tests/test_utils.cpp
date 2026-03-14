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

TEST(ResultTest, VoidResult) {
    Result<void, ErrorCode> ok_result = Ok<ErrorCode>();
    Result<void, ErrorCode> err_result = Err<void, ErrorCode>(ErrorCode::ERROR);

    EXPECT_TRUE(ok_result.isOk());
    EXPECT_FALSE(err_result.isOk());
}

TEST(ResultTest, VoidDefaultConstruction) {
    Result<void, ErrorCode> result;
    EXPECT_FALSE(result.isOk());
    EXPECT_TRUE(result.isError());
}

TEST(ResultTest, ErrorCodeToString) {
    EXPECT_STREQ(errorCodeToString(ErrorCode::OK), "OK");
    EXPECT_STREQ(errorCodeToString(ErrorCode::TIMEOUT), "TIMEOUT");
    EXPECT_STREQ(errorCodeToString(ErrorCode::NOT_FOUND), "NOT_FOUND");
}

TEST(ResultTest, MapError) {
    Result<int, ErrorCode> result = Err<int, ErrorCode>(ErrorCode::TIMEOUT);
    auto mapped = result.mapError([](ErrorCode e) {
        return static_cast<ErrorCode>(static_cast<int>(e) + 1);
    });

    EXPECT_FALSE(mapped.isOk());
    EXPECT_EQ(mapped.error(), ErrorCode::BUSY);
}

TEST(ResultTest, OrElse) {
    Result<int, ErrorCode> ok_result = Ok<int, ErrorCode>(42);
    Result<int, ErrorCode> err_result = Err<int, ErrorCode>(ErrorCode::NOT_FOUND);
    Result<int, ErrorCode> fallback = Ok<int, ErrorCode>(0);

    EXPECT_EQ(ok_result.orElse(fallback).value(), 42);
    EXPECT_EQ(err_result.orElse(fallback).value(), 0);
}

TEST(ResultTest, DefaultConstruction) {
    // Result по умолчанию должен быть в состоянии error (ok_ = false)
    Result<int, ErrorCode> result;
    EXPECT_FALSE(result.isOk());
    EXPECT_TRUE(result.isError());
}

TEST(ResultTest, MoveSemantics) {
    Result<int, ErrorCode> result = Ok<int, ErrorCode>(42);
    auto moved = std::move(result);
    
    EXPECT_TRUE(moved.isOk());
    EXPECT_EQ(moved.value(), 42);
}

// ============================================================================
// Callback Tests
// ============================================================================

TEST(CallbackTest, EmptyCallback) {
    Callback<void(int)> cb;
    EXPECT_FALSE(cb.isValid());
    EXPECT_TRUE(cb.isEmpty());
}

TEST(CallbackTest, NullptrComparison) {
    Callback<int()> cb;
    EXPECT_EQ(cb, nullptr);
    EXPECT_EQ(nullptr, cb);
    
    auto lambda = +[]() { return 42; };
    Callback<int()> cb2(lambda);
    EXPECT_NE(cb2, nullptr);
    EXPECT_NE(nullptr, cb2);
}

TEST(CallbackTest, NullptrAssignment) {
    auto lambda = +[]() { return 42; };
    Callback<int()> cb(lambda);
    
    EXPECT_TRUE(cb.isValid());
    cb = nullptr;
    EXPECT_FALSE(cb.isValid());
    EXPECT_EQ(cb, nullptr);
}

TEST(CallbackTest, FunctionCallback) {
    auto lambda = +[](int x) { return x * 2; };
    Callback<int(int)> cb(lambda);

    EXPECT_TRUE(cb.isValid());
    EXPECT_EQ(cb.call(5), 10);
    EXPECT_EQ(cb(7), 14);
}

TEST(CallbackTest, VoidCallback) {
    auto lambda = +[]() { /* no capture - uses global or passed data */ };
    Callback<void()> cb(lambda);

    cb.call();
    cb.call();
    // Note: Callback without capture cannot modify external state
    // For capturing lambdas, use CallbackWithStorage
}

TEST(CallbackTest, CallbackReset) {
    auto lambda = +[](int x) { return x; };
    Callback<int(int)> cb(lambda);

    EXPECT_TRUE(cb.isValid());
    cb.reset();
    EXPECT_FALSE(cb.isValid());
}

TEST(CallbackTest, BoolOperator) {
    Callback<int()> empty_cb;
    auto lambda = +[]() { return 42; };
    Callback<int()> valid_cb(lambda);

    EXPECT_FALSE(empty_cb);
    EXPECT_TRUE(valid_cb);
}

TEST(CallbackTest, WithStorage) {
    int multiplier = 3;
    auto lambda = [multiplier](int x) { return x * multiplier; };

    CallbackWithStorage<int(int)> cb(lambda);

    EXPECT_TRUE(cb.isValid());
    EXPECT_EQ(cb(5), 15);
}

TEST(CallbackTest, WithStorageVoid) {
    int value = 0;
    auto lambda = [&value](int x) { value = x; };

    CallbackWithStorage<void(int)> cb(lambda);

    cb(42);
    EXPECT_EQ(value, 42);
}

TEST(CallbackTest, WithStorageModify) {
    int counter = 0;
    auto lambda = [&counter]() { counter++; };

    CallbackWithStorage<void()> cb(lambda);

    cb();
    cb();
    EXPECT_EQ(counter, 2);
}

TEST(CallbackTest, MakeCallback) {
    auto cb = makeCallback(+[](int x, int y) { return x + y; });

    EXPECT_EQ(cb(3, 4), 7);
}
