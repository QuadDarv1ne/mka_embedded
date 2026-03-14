# Заметки о проекте MKA Embedded

## Архитектурные решения

### C++17 совместимость
- Проект использует C++17 стандарт
- Для `std::span` создан аналог `mka::span` в `utils/span.hpp`
- При C++20 автоматически используется `std::span`

### Обработка ошибок
- `Result<T, E>` для функций, которые могут вернуть ошибку
- Конструктор по умолчанию создаёт состояние ошибки
- `Ok()` для void возвращает успех через `success_tag`

### Управление памятью
- `FixedBlockPool` — пул фиксированных блоков (O(1))
- `VariablePool` — buddy allocator для переменных размеров
- Вся память обнуляется при выделении/освобождении

### Callback без аллокаций
- `Callback<F>` — для функций без захвата
- `CallbackWithStorage<F, N>` — для лямбд с захватом (до N байт)

## Критичные точки

### Безопасность
1. `invSqrt` — защита от x <= 0
2. `memcpy` вместо `reinterpret_cast` (strict-aliasing)
3. Double-free защита в `MemoryPool`
4. Валидация контекста в `StateMachineContext::isValid()`

### Производительность
1. `FixedBlockPool::allocate()` — линейный поиск (можно улучшить до O(1) с free list)
2. `VariablePool::splitBlock()` — может быть узким местом

## Тестирование

### Запуск
```bash
cmake --build build/host-debug
ctest --test-dir build/host-debug --preset host-debug
```

### Покрытие
- 9 тестовых наборов
- Все тесты проходят ✅

## Сборка

### Пресеты (Ninja)
- `host-debug` — отладка для хоста
- `host-release` — релиз для хоста
- `stm32f4-debug/release` — STM32F4
- `stm32f7-debug/release` — STM32F7
- `stm32h7-debug/release` — STM32H7

### Переменные окружения
```bash
# Включить cppcheck
-DCMAKE_CXX_CPPCHECK=cppcheck

# Включить coverage
-DCMAKE_BUILD_TYPE=Debug -DENABLE_COVERAGE=ON
```

## Известные ограничения

1. FreeRTOS wrapper требует реального FreeRTOS для линковки
2. `VariablePool` не поддерживает deallocation с coalescing
3. `CallbackWithStorage` имеет ограничение на размер лямбды
