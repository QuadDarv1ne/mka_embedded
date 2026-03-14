# Документация MKA Embedded

## Быстрый старт

### 1. Установка Doxygen

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
# Через CMake (рекомендуется)
cmake -B build -DBUILD_DOCS=ON
cmake --build build --target docs

# Вручную
cd docs && doxygen Doxyfile.in
```

### 3. Просмотр

Откройте `docs/html/index.html` в браузере.

---

## Документы

| Файл | Описание |
|------|----------|
| [**doxygen_guide.md**](doxygen_guide.md) | Руководство: установка, синтаксис, настройка |
| [**doxygen_cheatsheet.md**](doxygen_cheatsheet.md) | Шпаргалка по тегам и синтаксису |
| [Doxyfile.in](Doxyfile.in) | Конфигурация для CMake |

---

## Стиль документирования

Используется **Javadoc-стиль**:

```cpp
/**
 * @brief Краткое описание
 *
 * Подробное описание.
 *
 * @param param Описание параметра
 * @return Результат
 *
 * @note Примечание
 */
Status init(const Config& config);
```

### Основные теги

- `@brief` — краткое описание
- `@param` — параметр
- `@return` — возвращаемое значение
- `@note` — примечание
- `@warning` — предупреждение
- `@see` — ссылка
- `@todo` — задача
