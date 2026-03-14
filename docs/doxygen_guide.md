# Doxygen: Руководство для MKA Embedded

## Быстрый старт

### 1. Установка

**Windows:**
```bash
winget install DimitriVanHeesch.Doxygen --silent --accept-package-agreements
```

**Linux:**
```bash
sudo apt-get install doxygen graphviz -y
```

**macOS:**
```bash
brew install doxygen graphviz
```

**Проверка:** `doxygen --version`

### 2. Генерация

```bash
# Через CMake
cmake -B build -DBUILD_DOCS=ON
cmake --build build --target docs

# Вручную
cd docs && doxygen Doxyfile.in
```

### 3. Просмотр

Откройте `docs/html/index.html`

---

## Синтаксис документирования

### Формат комментариев

```cpp
/**
 * @brief Краткое описание
 *
 * Подробное описание функции или класса.
 *
 * @param param Описание параметра
 * @return Результат выполнения
 *
 * @note Примечание (необязательно)
 * @see relatedFunction()
 */
Status init(const Config& config);
```

### Основные теги

| Тег | Назначение |
|-----|------------|
| `@brief` | Краткое описание |
| `@param` | Параметр функции |
| `@return` | Возвращаемое значение |
| `@retval` | Конкретное значение возврата |
| `@note` | Примечание |
| `@warning` | Предупреждение |
| `@attention` | Важное замечание |
| `@see` | Ссылка на связанный элемент |
| `@todo` | Задача на будущее |
| `@deprecated` | Устаревший элемент |

### Документирование классов

```cpp
/**
 * @brief Контроллер ориентации спутника
 *
 * Реализует PID-регулирование для управления маховиками.
 *
 * @tparam T Тип данных (float/double)
 *
 * @example
 * @code{.cpp}
 * PIDController<float> pid(0.1f, 0.01f, 0.05f);
 * float control = pid.compute(error, dt);
 * @endcode
 */
template<typename T>
class PIDController {
public:
    /**
     * @brief Конструктор контроллера
     * @param kp Пропорциональный коэффициент
     * @param ki Интегральный коэффициент
     * @param kd Дифференциальный коэффициент
     */
    PIDController(T kp, T ki, T kd);

    /**
     * @brief Вычисление управляющего воздействия
     * @param error Ошибка регулирования
     * @param dt Время с последнего вызова (сек)
     * @return Управляющий сигнал
     */
    T compute(T error, T dt);
};
```

### Документирование перечислений

```cpp
/**
 * @brief Режимы работы спутника
 */
enum class SatelliteMode : uint8_t {
    OFF = 0,        ///< Полное отключение
    INIT = 1,       ///< Инициализация
    SAFE = 2,       ///< Безопасный режим
    NOMINAL = 3,    ///< Нормальная работа
    MISSION = 4     ///< Выполнение миссии
};
```

---

## Настройка Doxyfile

### Ключевые параметры

```bash
# Проект
PROJECT_NAME = "MKA Embedded"
PROJECT_NUMBER = "1.0.0"

# Входные файлы
INPUT = ../src/cpp
RECURSIVE = YES
FILE_PATTERNS = *.hpp *.h

# Вывод
GENERATE_HTML = YES
GENERATE_LATEX = NO

# Извлечение документации
EXTRACT_ALL = YES
EXTRACT_STATIC = YES

# Графы (требует Graphviz)
HAVE_DOT = NO
CLASS_GRAPH = YES
INCLUDE_GRAPH = YES

# Предупреждения
WARNINGS = YES
WARN_IF_UNDOCUMENTED = YES
```

---

## Интеграция с CMake

В `CMakeLists.txt`:

```cmake
option(BUILD_DOCS "Build documentation" OFF)

if(BUILD_DOCS)
    find_package(Doxygen QUIET)
    if(DOXYGEN_FOUND)
        set(DOXYGEN_INPUT ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp)
        set(DOXYGEN_OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/docs)
        
        doxygen_add_docs(docs
            ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp
            COMMENT "Generating documentation"
        )
    endif()
endif()
```

---

## Troubleshooting

| Проблема | Решение |
|----------|---------|
| `doxygen: command not found` | Добавьте Doxygen в PATH |
| Нет графов | Установите Graphviz, `HAVE_DOT = YES` |
| Битые ссылки | Используйте полное имя: `mka::fdir::FDIRManager` |
| Кракозябры | Сохраняйте файлы в UTF-8, `INPUT_ENCODING = UTF-8` |

---

## Дополнительные ресурсы

- **Официальная документация:** https://www.doxygen.nl/manual/
- **GitHub:** https://github.com/doxygen/doxygen
- **Шпаргалка:** [doxygen_cheatsheet.md](doxygen_cheatsheet.md)
