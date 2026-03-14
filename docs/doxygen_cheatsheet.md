# Doxygen: Шпаргалка

## Комментарии

```cpp
/** ... */     // Javadoc (основной)
/*! ... */     // Qt-стиль
/// ...         // Для одной функции
//! ...         // Краткий
```

## Теги

### Обязательные
```cpp
@brief       // Краткое описание
@param       // Параметр
@return      // Возвращаемое значение
```

### Дополнительные
```cpp
@note        // Примечание
@warning     // Предупреждение
@attention   // Важно!
@see         // См. также
@todo        // Задача
@deprecated  // Устарело
@pre         // Требование до
@post        // Гарантия после
```

## Примеры

### Функция
```cpp
/**
 * @brief Кратко
 *
 * Подробно...
 *
 * @param x Параметр
 * @return Результат
 * @retval OK Успех
 * @retval ERROR Ошибка
 */
Status foo(int x);
```

### Класс
```cpp
/**
 * @brief Описание класса
 * @tparam T Тип-параметр
 */
template<typename T>
class MyClass {
public:
    /// Конструктор
    MyClass();
    
    int value;  ///< Поле
};
```

### Перечисление
```cpp
/**
 * @brief Статус
 */
enum class Status {
    OK = 0,     ///< Успех
    ERROR = 1   ///< Ошибка
};
```

### Структура
```cpp
/**
 * @brief Конфигурация
 * @member kp Пропорциональный коэффициент
 * @member ki Интегральный коэффициент
 */
struct Config {
    float kp;  ///< Пропорциональный
    float ki;  ///< Интегральный
};
```

### Файл
```cpp
/**
 * @file my_header.hpp
 * @brief Описание файла
 */
```

### Пространство имён
```cpp
/**
 * @namespace mka::drivers
 * @brief Драйверы
 */
```

## Форматирование

```cpp
/**
 * @c inline_code
 * **@b bold**
 * *@e italic*
 * @p monospace
 *
 * @code{.cpp}
 * int x = 42;
 * @endcode
 *
 * @ref MyClass "ссылка"
 * @image html diagram.png
 */
```

## Ссылки

```cpp
@see OtherClass
@link MyClass @endlink
@ref label "текст"
```

## Пользовательские алиасы

В проекте доступны:
```cpp
@threadsafe    // Thread Safety: ...
@hardware      // Hardware: ...
@performance   // Performance: ...
@invariant     // Инвариант
```

## VS Code Snippet

```cpp
/**
 * @brief ${1:описание}
 * 
 * ${2:подробно}
 * 
 * @param ${3:param} ${4:описание}
 * @return ${5:результат}
 */
```

---
*Полная версия: [doxygen_guide.md](doxygen_guide.md)*
