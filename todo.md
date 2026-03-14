# MKA Embedded — План разработки

## ✅ Выполнено

### C++17 совместимость
- [x] Добавить `mka::span` для замены `std::span` (C++20)
- [x] Исправить все модули: `fdir.hpp`, `state_machine.hpp`, `command_handler.hpp`, `param_store.hpp`, `watchdog_manager.hpp`, `log_system.hpp`
- [x] Исправить драйверы: `eeprom_driver.hpp`, `radio_driver.hpp`, `sun_sensor.hpp`
- [x] Исправить HAL: `hal_full.hpp`, `freertos_wrapper.hpp`

### Безопасность кода
- [x] Заменить `reinterpret_cast` на `memcpy` в `invSqrt` (strict-aliasing)
- [x] Добавить обнуление памяти в `MemoryPool::allocate/deallocate`
- [x] Добавить защиту от невалидных входов в `invSqrt(x <= 0)`
- [x] Добавить явную инициализацию полей во всех классах

### Улучшения API
- [x] Добавить конструкторы по умолчанию в `Result<T, E>`
- [x] Добавить конструкторы по умолчанию в `FixedBlockPool`, `VariablePool`
- [x] Добавить конструкторы по умолчанию в `PIDController`, `BDotController`, `AttitudeController`
- [x] Добавить `success_tag` для `Result<void, E>::Ok()`
- [x] Добавить явную инициализацию в `Quaternion`, `StateMachineContext`, `TransitionResult`
- [x] Улучшить `CallbackWithStorage` — исправить copy/move операции

### Тесты
- [x] Все 9 тестов проходят: algorithms, fdir, statemachine, utils, commands, param_store, watchdog, memory_pool, log_system
- [x] Добавить тесты для конструкторов по умолчанию
- [x] Добавить тесты валидации контекста
- [x] Добавить тесты `invSqrt(0)` и отрицательных значений

## 🔄 В процессе

## 📋 Запланировано

### Критичные улучшения
- [ ] Добавить статический анализ (cppcheck, clang-tidy) в CI
- [ ] Покрыть тестами все критические модули (>80%)
- [ ] Добавить интеграционные тесты для FreeRTOS

### Документация
- [ ] Обновить README с инструкциями по сборке
- [ ] Добавить примеры использования для основных модулей
- [ ] Документировать API публичных классов

### Производительность
- [ ] Benchmark для memory_pool
- [ ] Оптимизация алгоритмов ADCS (при необходимости)

## 📊 Статус веток

| Ветка | Коммит | Статус |
|-------|--------|--------|
| `main` | `162fa97` | ✅ Стабильная |
| `dev` | `84e8b22` | ✅ Активная разработка |

## 🔧 Сборка

```bash
# Host debug (Ninja)
cmake --build build/host-debug

# Host release
cmake --build build/host-release

# Запуск тестов
ctest --test-dir build/host-debug --preset host-debug
```

Все тесты: **9/9 ✅**
