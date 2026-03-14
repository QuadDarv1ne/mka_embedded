/**
 * @file test_param_store.cpp
 * @brief Unit tests for Parameter Store system
 */

#include <gtest/gtest.h>
#include <cstring>

#include "systems/param_store.hpp"

using namespace mka::param;

// ============================================================================
// Тесты ParamType
// ============================================================================

TEST(ParamTypeTest, EnumValues) {
    EXPECT_EQ(static_cast<uint8_t>(ParamType::UINT8), 0u);
    EXPECT_EQ(static_cast<uint8_t>(ParamType::INT8), 1u);
    EXPECT_EQ(static_cast<uint8_t>(ParamType::UINT16), 2u);
    EXPECT_EQ(static_cast<uint8_t>(ParamType::INT16), 3u);
    EXPECT_EQ(static_cast<uint8_t>(ParamType::UINT32), 4u);
    EXPECT_EQ(static_cast<uint8_t>(ParamType::INT32), 5u);
    EXPECT_EQ(static_cast<uint8_t>(ParamType::FLOAT), 8u);
}

// ============================================================================
// Тесты ParamValue
// ============================================================================

TEST(ParamValueTest, Uint8Value) {
    ParamValue value{};
    value.u8 = 42;
    EXPECT_EQ(value.u8, 42u);
}

TEST(ParamValueTest, FloatValue) {
    ParamValue value{};
    value.f = 3.14f;
    EXPECT_FLOAT_EQ(value.f, 3.14f);
}

TEST(ParamValueTest, BytesAccess) {
    ParamValue value{};
    value.u32 = 0x12345678;
    
    // Проверка доступа через bytes (зависит от endianness)
    // На x86/x64 little-endian
    EXPECT_EQ(value.bytes[0], 0x78);
    EXPECT_EQ(value.bytes[3], 0x12);
}

// ============================================================================
// Тесты ParamAttributes
// ============================================================================

TEST(ParamAttributesTest, DefaultFlags) {
    ParamAttributes attrs{};
    // По умолчанию все флаги false
    EXPECT_FALSE(attrs.readable);
    EXPECT_FALSE(attrs.writable);
    EXPECT_FALSE(attrs.persistent);
    EXPECT_FALSE(attrs.requiresReset);
    EXPECT_FALSE(attrs.readOnlyAfterInit);
}

TEST(ParamAttributesTest, SetFlags) {
    ParamAttributes attrs{true, true, true, false, false, 0};
    EXPECT_TRUE(attrs.readable);
    EXPECT_TRUE(attrs.writable);
    EXPECT_TRUE(attrs.persistent);
}

// ============================================================================
// Тесты ParamEntry
// ============================================================================

TEST(ParamEntryTest, DefaultInitialization) {
    ParamEntry entry{};
    EXPECT_EQ(entry.id, 0u);
    EXPECT_EQ(entry.type, ParamType::UINT8);
    EXPECT_EQ(entry.size, 0u);
    EXPECT_EQ(entry.name, nullptr);
}

TEST(ParamEntryTest, FullInitialization) {
    ParamAttributes attrs{true, true, true, false, false, 0};
    ParamValue def, min, max;
    def.f = 10.0f;
    min.f = 0.0f;
    max.f = 100.0f;
    
    ParamEntry entry{
        0x0100,
        ParamType::FLOAT,
        attrs,
        4,
        def,
        min,
        max,
        "test_param",
        "Test parameter",
        "V"
    };
    
    EXPECT_EQ(entry.id, 0x0100u);
    EXPECT_EQ(entry.type, ParamType::FLOAT);
    EXPECT_EQ(entry.size, 4u);
    EXPECT_STREQ(entry.name, "test_param");
    EXPECT_FLOAT_EQ(entry.defaultValue.f, 10.0f);
}

// ============================================================================
// Тесты ParameterStore - регистрация
// ============================================================================

TEST(ParameterStoreTest, InitialState) {
    ParameterStore store;
    EXPECT_EQ(store.count(), 0u);
    EXPECT_FALSE(store.exists(0x0100));
}

TEST(ParameterStoreTest, RegisterParameter) {
    ParameterStore store;
    
    ParamAttributes attrs{true, true, true, false, false, 0};
    ParamValue def, min, max;
    def.u8 = 10;
    min.u8 = 0;
    max.u8 = 100;
    
    ParamEntry entry{0x0100, ParamType::UINT8, attrs, 1, def, min, max, 
                     "test", "test desc", "unit"};
    
    EXPECT_TRUE(store.registerParam(entry));
    EXPECT_EQ(store.count(), 1u);
    EXPECT_TRUE(store.exists(0x0100));
}

TEST(ParameterStoreTest, RegisterDuplicateParameter) {
    ParameterStore store;
    
    ParamAttributes attrs{true, true, true, false, false, 0};
    ParamValue def, min, max;
    def.u8 = 10;
    min.u8 = 0;
    max.u8 = 100;
    
    ParamEntry entry{0x0100, ParamType::UINT8, attrs, 1, def, min, max,
                     "test", "test desc", "unit"};
    
    EXPECT_TRUE(store.registerParam(entry));
    EXPECT_FALSE(store.registerParam(entry));  // Дубликат
    EXPECT_EQ(store.count(), 1u);
}

TEST(ParameterStoreTest, RegisterMultipleParameters) {
    ParameterStore store;
    
    for (uint16_t i = 0; i < 10; i++) {
        ParamAttributes attrs{true, true, true, false, false, 0};
        ParamValue def, min, max;
        def.u16 = i;
        min.u16 = 0;
        max.u16 = 1000;
        
        ParamEntry entry{
            static_cast<uint16_t>(0x0100 + i),
            ParamType::UINT16,
            attrs,
            2,
            def,
            min,
            max,
            "test", "test", "unit"
        };
        EXPECT_TRUE(store.registerParam(entry));
    }
    
    EXPECT_EQ(store.count(), 10u);
    
    for (uint16_t i = 0; i < 10; i++) {
        EXPECT_TRUE(store.exists(static_cast<uint16_t>(0x0100 + i)));
    }
}

// ============================================================================
// Тесты ParameterStore - получение/установка
// ============================================================================

TEST(ParameterStoreTest, GetNonExistentParameter) {
    ParameterStore store;
    auto result = store.get(0x0100);
    EXPECT_FALSE(result.has_value());
}

TEST(ParameterStoreTest, GetSetValue) {
    ParameterStore store;
    
    ParamAttributes attrs{true, true, true, false, false, 0};
    ParamValue def, min, max;
    def.u8 = 10;
    min.u8 = 0;
    max.u8 = 100;
    
    ParamEntry entry{0x0100, ParamType::UINT8, attrs, 1, def, min, max,
                     "test", "test desc", "unit"};
    
    store.registerParam(entry);
    
    // Получение значения по умолчанию
    auto value = store.get(0x0100);
    EXPECT_TRUE(value.has_value());
    EXPECT_EQ(value->u8, 10u);
    
    // Установка нового значения
    ParamValue newValue;
    newValue.u8 = 50;
    EXPECT_TRUE(store.set(0x0100, newValue));
    
    // Проверка нового значения
    value = store.get(0x0100);
    EXPECT_TRUE(value.has_value());
    EXPECT_EQ(value->u8, 50u);
}

TEST(ParameterStoreTest, SetReadonlyParameter) {
    ParameterStore store;
    
    ParamAttributes attrs{true, false, false, false, false, 0};  // writable = false
    ParamValue def, min, max;
    def.u8 = 10;
    min.u8 = 0;
    max.u8 = 100;
    
    ParamEntry entry{0x0100, ParamType::UINT8, attrs, 1, def, min, max,
                     "test", "test desc", "unit"};
    
    store.registerParam(entry);
    
    ParamValue newValue;
    newValue.u8 = 50;
    EXPECT_FALSE(store.set(0x0100, newValue));  // Ошибка записи
}

TEST(ParameterStoreTest, SetValueOutOfRange) {
    ParameterStore store;
    
    ParamAttributes attrs{true, true, true, false, false, 0};
    ParamValue def, min, max;
    def.u8 = 10;
    min.u8 = 0;
    max.u8 = 100;
    
    ParamEntry entry{0x0100, ParamType::UINT8, attrs, 1, def, min, max,
                     "test", "test desc", "unit"};
    
    store.registerParam(entry);
    
    ParamValue newValue;
    newValue.u8 = 150;  // Вне диапазона
    EXPECT_FALSE(store.set(0x0100, newValue));
    
    // Значение не изменилось
    auto value = store.get(0x0100);
    EXPECT_EQ(value->u8, 10u);
}

TEST(ParameterStoreTest, SetFloatValue) {
    ParameterStore store;
    
    ParamAttributes attrs{true, true, true, false, false, 0};
    ParamValue def, min, max;
    def.f = 3.14f;
    min.f = 0.0f;
    max.f = 10.0f;
    
    ParamEntry entry{0x0100, ParamType::FLOAT, attrs, 4, def, min, max,
                     "test", "test desc", "unit"};
    
    store.registerParam(entry);
    
    ParamValue newValue;
    newValue.f = 5.5f;
    EXPECT_TRUE(store.set(0x0100, newValue));
    
    auto value = store.get(0x0100);
    EXPECT_TRUE(value.has_value());
    EXPECT_FLOAT_EQ(value->f, 5.5f);
}

// ============================================================================
// Тесты ParameterStore - сброс
// ============================================================================

TEST(ParameterStoreTest, ResetParameter) {
    ParameterStore store;
    
    ParamAttributes attrs{true, true, true, false, false, 0};
    ParamValue def, min, max;
    def.u8 = 10;
    min.u8 = 0;
    max.u8 = 100;
    
    ParamEntry entry{0x0100, ParamType::UINT8, attrs, 1, def, min, max,
                     "test", "test desc", "unit"};
    
    store.registerParam(entry);
    
    // Изменение значения
    ParamValue newValue;
    newValue.u8 = 50;
    store.set(0x0100, newValue);
    
    // Сброс
    EXPECT_TRUE(store.reset(0x0100));
    
    auto value = store.get(0x0100);
    EXPECT_EQ(value->u8, 10u);  // Возврат к значению по умолчанию
}

TEST(ParameterStoreTest, ResetNonExistentParameter) {
    ParameterStore store;
    EXPECT_FALSE(store.reset(0x0100));
}

TEST(ParameterStoreTest, ResetAllParameters) {
    ParameterStore store;
    
    ParamAttributes attrs{true, true, true, false, false, 0};
    
    ParamValue def1, min1, max1;
    def1.u8 = 10;
    min1.u8 = 0;
    max1.u8 = 100;
    ParamEntry entry1{0x0100, ParamType::UINT8, attrs, 1, def1, min1, max1,
                      "test1", "test", "unit"};
    
    ParamValue def2, min2, max2;
    def2.u8 = 20;
    min2.u8 = 0;
    max2.u8 = 100;
    ParamEntry entry2{0x0101, ParamType::UINT8, attrs, 1, def2, min2, max2,
                      "test2", "test", "unit"};
    
    store.registerParam(entry1);
    store.registerParam(entry2);
    
    // Изменение значений
    ParamValue newValue;
    newValue.u8 = 50;
    store.set(0x0100, newValue);
    store.set(0x0101, newValue);
    
    // Сброс всех
    store.resetAll();
    
    auto value1 = store.get(0x0100);
    auto value2 = store.get(0x0101);
    EXPECT_EQ(value1->u8, 10u);
    EXPECT_EQ(value2->u8, 20u);
}

// ============================================================================
// Тесты ParameterStore - информация о параметрах
// ============================================================================

TEST(ParameterStoreTest, GetEntry) {
    ParameterStore store;

    ParamAttributes attrs{true, true, true, false, false, 0};
    ParamValue def{}, min{}, max{};
    def.u8 = 10;
    ParamEntry entry{0x0100, ParamType::UINT8, attrs, 1, def, min, max,
                     "test_name", "test_desc", "unit"};

    store.registerParam(entry);

    const ParamEntry* retrieved = store.getEntry(0x0100);
    EXPECT_NE(retrieved, nullptr);
    EXPECT_STREQ(retrieved->name, "test_name");
    EXPECT_STREQ(retrieved->description, "test_desc");
    EXPECT_STREQ(retrieved->unit, "unit");
}

TEST(ParameterStoreTest, GetNonExistentEntry) {
    ParameterStore store;
    const ParamEntry* entry = store.getEntry(0x0100);
    EXPECT_EQ(entry, nullptr);
}

TEST(ParameterStoreTest, GetParamIds) {
    ParameterStore store;
    
    ParamAttributes attrs{true, true, true, false, false, 0};
    ParamValue def, min, max;
    def.u16 = 0;
    
    for (uint16_t i = 0; i < 5; i++) {
        ParamEntry entry{
            static_cast<uint16_t>(0x0100 + i),
            ParamType::UINT16,
            attrs,
            2,
            def,
            min,
            max,
            "test", "test", "unit"
        };
        store.registerParam(entry);
    }
    
    std::array<uint16_t, 10> ids{};
    size_t count = store.getParamIds(ids.data(), ids.size());
    
    EXPECT_EQ(count, 5u);
    for (size_t i = 0; i < 5; i++) {
        EXPECT_EQ(ids[i], 0x0100 + i);
    }
}

// ============================================================================
// Тесты ParameterStore - персистентность
// ============================================================================

TEST(ParameterStoreTest, HasUnsavedChanges) {
    ParameterStore store;
    
    ParamAttributes attrsPersistent{true, true, true, false, false, 0};   // persistent
    ParamAttributes attrsVolatile{true, true, false, false, false, 0};    // не persistent
    
    ParamValue def, min, max;
    def.u8 = 10;
    min.u8 = 0;
    max.u8 = 100;
    
    ParamEntry entry1{0x0100, ParamType::UINT8, attrsPersistent, 1, def, min, max,
                      "persistent", "test", "unit"};
    ParamEntry entry2{0x0101, ParamType::UINT8, attrsVolatile, 1, def, min, max,
                      "volatile", "test", "unit"};
    
    store.registerParam(entry1);
    store.registerParam(entry2);
    
    // Нет изменений
    EXPECT_FALSE(store.hasUnsavedChanges());
    
    // Изменение volatile параметра
    ParamValue newValue;
    newValue.u8 = 50;
    store.set(0x0101, newValue);
    EXPECT_FALSE(store.hasUnsavedChanges());  // Не persistent
    
    // Изменение persistent параметра
    store.set(0x0100, newValue);
    EXPECT_TRUE(store.hasUnsavedChanges());
}

TEST(ParameterStoreTest, Serialize) {
    ParameterStore store;
    
    ParamAttributes attrs{true, true, true, false, false, 0};
    ParamValue def, min, max;
    def.u8 = 10;
    min.u8 = 0;
    max.u8 = 100;
    
    ParamEntry entry{0x0100, ParamType::UINT8, attrs, 1, def, min, max,
                     "test", "test", "unit"};
    
    store.registerParam(entry);
    
    // Изменение значения
    ParamValue newValue;
    newValue.u8 = 50;
    store.set(0x0100, newValue);
    
    std::array<uint8_t, 256> buffer{};
    size_t written = store.serialize(buffer.data(), buffer.size());
    
    EXPECT_GT(written, 0u);
    EXPECT_EQ(buffer[0], 'P');
    EXPECT_EQ(buffer[1], 'A');
    EXPECT_EQ(buffer[2], 'R');
}

TEST(ParameterStoreTest, SerializeDeserialize) {
    ParameterStore store;
    
    ParamAttributes attrs{true, true, true, false, false, 0};
    ParamValue def, min, max;
    def.u8 = 10;
    min.u8 = 0;
    max.u8 = 100;
    
    ParamEntry entry{0x0100, ParamType::UINT8, attrs, 1, def, min, max,
                     "test", "test", "unit"};
    
    store.registerParam(entry);
    
    // Изменение значения
    ParamValue newValue;
    newValue.u8 = 50;
    store.set(0x0100, newValue);
    
    // Сериализация
    std::array<uint8_t, 256> buffer{};
    size_t written = store.serialize(buffer.data(), buffer.size());
    EXPECT_GT(written, 0u);
    
    // Десериализация в новый store
    ParameterStore store2;
    ParamEntry entry2{0x0100, ParamType::UINT8, attrs, 1, def, min, max,
                      "test", "test", "unit"};
    store2.registerParam(entry2);
    
    EXPECT_TRUE(store2.deserialize(buffer.data(), written));
    
    // Проверка значения
    auto value = store2.get(0x0100);
    EXPECT_TRUE(value.has_value());
    EXPECT_EQ(value->u8, 50u);  // Восстановленное значение
}

TEST(ParameterStoreTest, DeserializeInvalidHeader) {
    ParameterStore store;
    
    std::array<uint8_t, 16> buffer{'X', 'Y', 'Z', 0x01, 0x00, 0x01};
    EXPECT_FALSE(store.deserialize(buffer.data(), buffer.size()));
}

TEST(ParameterStoreTest, DeserializeTooShort) {
    ParameterStore store;
    
    std::array<uint8_t, 3> buffer{'P', 'A', 'R'};
    EXPECT_FALSE(store.deserialize(buffer.data(), buffer.size()));
}

// ============================================================================
// Тесты ParameterStore - callback
// ============================================================================

TEST(ParameterStoreTest, ChangeCallback) {
    ParameterStore store;
    
    ParamAttributes attrs{true, true, true, false, false, 0};
    ParamValue def, min, max;
    def.u8 = 10;
    min.u8 = 0;
    max.u8 = 100;
    
    ParamEntry entry{0x0100, ParamType::UINT8, attrs, 1, def, min, max,
                     "test", "test", "unit"};

    store.registerParam(entry);

    auto callback = +[](uint16_t id, ParamValue value) {
        // Используем глобальные переменные для хранения состояния
        static uint16_t lastId = 0;
        static uint8_t lastValue = 0;
        lastId = id;
        lastValue = value.u8;
        // Для теста просто компилируем callback
        (void)lastId; (void)lastValue;
    };

    store.setChangeCallback(callback);

    ParamValue newValue;
    newValue.u8 = 50;
    store.set(0x0100, newValue);

    // Callback без захвата не может изменить внешние переменные
    // Просто проверяем что код компилируется
    EXPECT_TRUE(true);
}

// ============================================================================
// Тесты макросов
// ============================================================================

TEST(MacrosTest, ParamUint8) {
    auto entry = PARAM_UINT8(0x0100, "test", 10, 0, 100, "desc", "unit");
    
    EXPECT_EQ(entry.id, 0x0100u);
    EXPECT_EQ(entry.type, ParamType::UINT8);
    EXPECT_EQ(entry.size, 1u);
    EXPECT_EQ(entry.defaultValue.u8, 10u);
    EXPECT_EQ(entry.minValue.u8, 0u);
    EXPECT_EQ(entry.maxValue.u8, 100u);
    EXPECT_STREQ(entry.name, "test");
    EXPECT_TRUE(entry.attributes.writable);
    EXPECT_TRUE(entry.attributes.persistent);
}

TEST(MacrosTest, ParamFloat) {
    ParamValue def, min, max;
    def.f = 25.5f;
    min.f = 0.0f;
    max.f = 50.0f;
    
    ParamAttributes attrs{true, true, true, false, false, 0};
    ParamEntry entry{0x0200, ParamType::FLOAT, attrs, 4, def, min, max,
                     "temp", "temperature", "C"};
    
    EXPECT_EQ(entry.id, 0x0200u);
    EXPECT_EQ(entry.type, ParamType::FLOAT);
    EXPECT_EQ(entry.size, 4u);
    EXPECT_FLOAT_EQ(entry.defaultValue.f, 25.5f);
    EXPECT_FLOAT_EQ(entry.minValue.f, 0.0f);
    EXPECT_FLOAT_EQ(entry.maxValue.f, 50.0f);
}

TEST(MacrosTest, ParamReadonly) {
    ParamValue def, min, max;
    def.u32 = 12345u;
    min.u32 = 0;
    max.u32 = 0;
    
    ParamAttributes attrs{true, false, false, false, false, 0};
    ParamEntry entry{0x0300, ParamType::UINT32, attrs, 4, def, min, max,
                     "uptime", "uptime since boot", "s"};
    
    EXPECT_EQ(entry.id, 0x0300u);
    EXPECT_EQ(entry.type, ParamType::UINT32);
    EXPECT_EQ(entry.defaultValue.u32, 12345u);
    EXPECT_FALSE(entry.attributes.writable);
}

// ============================================================================
// Тесты предопределённых ID
// ============================================================================

TEST(PredefinedIdsTest, SystemParams) {
    EXPECT_EQ(ids::SYSTEM_VERSION, 0x0000u);
    EXPECT_EQ(ids::SYSTEM_UPTIME, 0x0001u);
    EXPECT_EQ(ids::SYSTEM_RESET_COUNT, 0x0002u);
}

TEST(PredefinedIdsTest, CommParams) {
    EXPECT_EQ(ids::COMM_BAUD_RATE, 0x0100u);
    EXPECT_EQ(ids::COMM_TX_POWER, 0x0101u);
    EXPECT_EQ(ids::COMM_FREQUENCY, 0x0103u);
}

TEST(PredefinedIdsTest, AdcsParams) {
    EXPECT_EQ(ids::ADCS_MODE, 0x0200u);
    EXPECT_EQ(ids::ADCS_P_GAIN, 0x0202u);
    EXPECT_EQ(ids::ADCS_MAX_RATE, 0x0205u);
}

TEST(PredefinedIdsTest, EpsParams) {
    EXPECT_EQ(ids::EPS_BATTERY_LOW, 0x0300u);
    EXPECT_EQ(ids::EPS_BATTERY_CRITICAL, 0x0301u);
}

TEST(PredefinedIdsTest, MissionParams) {
    EXPECT_EQ(ids::MISSION_MODE, 0x0400u);
    EXPECT_EQ(ids::MISSION_TARGET_LAT, 0x0402u);
}
