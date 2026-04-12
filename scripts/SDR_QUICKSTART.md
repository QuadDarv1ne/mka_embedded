# SDR Quick Start Guide

## 🚀 Быстрый старт (5 минут)

### 1. Проверка устройства

```bash
# Запустите проверку
sdr_check.bat
```

Должно показать:
```
[OK] rtlsdr.dll найден
[OK] Devices found: 1
[OK] Device opened
```

### 2. Первый приём (FM радио)

```bash
# Простой приём
sdr.bat 100.0

# Или вручную
python scripts\sdr_capture_enhanced.py --freq 100.0 --gain 30 --plot
```

Вы увидите спектр FM сигнала!

### 3. Меню

```bash
# Главное меню со всеми опциями
sdr_menu.bat
```

---

## 📡 Основные команды

### Универсальный приём

```bash
# Приём с анализом
python scripts\sdr_capture_enhanced.py --freq <МГц> --gain <dB> --plot

# Примеры
python scripts\sdr_capture_enhanced.py --freq 145.8 --gain 40 --plot  # ISS
python scripts\sdr_capture_enhanced.py --freq 144.8 --plot             # APRS
python scripts\sdr_capture_enhanced.py --freq 433.92 --plot            # ISM
```

### Приём спутников

```bash
# NOAA 19
python scripts\sdr_noaa.py --sat noaa19 --gain 40

# NOAA 18
python scripts\sdr_noaa.py --sat noaa18 --gain 40
```

### APRS пакеты

```bash
# Мониторинг APRS
python scripts\sdr_ax25.py --decode --monitor
```

### Сканирование

```bash
# Сканирование FM (88-108 МГц)
python scripts\sdr_scanner.py --start 88 --stop 108

# VHF диапазон
python scripts\sdr_scanner.py --start 137 --stop 146
```

### Калькулятор антенн

```bash
# Расчёт для частоты
python scripts\sdr_antenna.py --freq 145.8
```

---

## 🎯 Популярные частоты

| Частота | Назначение | Команда |
|---------|------------|---------|
| **88-108 МГц** | FM радио | `sdr.bat 100.0` |
| **137.100 МГц** | NOAA 19 | `sdr_noaa.bat` |
| **137.9125 МГц** | NOAA 18 | `sdr_noaa.bat noaa18` |
| **144.800 МГц** | APRS Европа | `sdr_aprs.bat` |
| **145.800 МГц** | ISS | `sdr.bat 145.8 40` |
| **145.825 МГц** | ISS APRS | `sdr_aprs.bat` |
| **433.920 МГц** | ISM 70cm | `sdr.bat 433.92 30` |
| **436.700 МГц** | ISS SSTV | `sdr.bat 436.7 40` |

---

## 🔧 Batch-файлы (самый простой способ)

| Файл | Назначение |
|------|------------|
| `sdr_menu.bat` | Главное меню со всеми опциями |
| `sdr_check.bat` | Проверка устройства |
| `sdr.bat <freq> [gain]` | Быстрый приём |
| `sdr_scan.bat [start] [stop]` | Сканирование |
| `sdr_noaa.bat [sat]` | Приём NOAA |
| `sdr_aprs.bat` | Приём APRS |

---

## 📊 Понимание результатов

### Мощность сигнала
- **>-20 dB**: Очень сильный сигнал ✓✓✓
- **-20 до -40 dB**: Сильный сигнал ✓✓
- **-40 до -60 dB**: Средний сигнал ✓
- **<-60 dB**: Слабый сигнал ✗

### SNR (отношение сигнал/шум)
- **>20 dB**: Отличное качество
- **10-20 dB**: Хорошее качество
- **5-10 dB**: Среднее качество
- **<5 dB**: Низкое качество

---

## 🐛 Решение проблем

### Устройство не найдено
```
No RTL-SDR devices!
```

**Решение:**
1. Проверьте USB подключение
2. Установите WinUSB драйвер через Zadig:
   - Скачайте https://zadig.akeo.ie/
   - Выберите RTL-SDR устройство
   - Нажмите "Install Driver"
3. Перезапустите скрипт

### Ошибка DLL
```
ERROR: rtlsdr.dll not found!
```

**Решение:**
1. Установите SDR# из https://airspy.com/download/
2. Или скачайте rtlsdr.dll отдельно
3. Положите в `C:\SDRSharp\tmp\x64\` или `C:\SDRSharp\`

### Слабый сигнал
```
Сигнал: СЛАБЫЙ
```

**Решение:**
1. Увеличьте усиление: `--gain 40`
2. Улучшите антенну (даже провод 50 см поможет!)
3. Подойдите к окну
4. Проверьте, что частота правильная

---

## 📚 Документация

- **Полная документация:** `scripts\SDR_README.md`
- **План развития:** `scripts\SDR_TODO.md`
- **Основной README:** `README.md`

---

## 💡 Советы для начинающих

1. **Начните с FM радио** (100 МГц) — сильные сигналы, легко принять
2. **Используйте антенну** — даже провод 50 см drastically улучшает приём
3. **Сканируйте диапазон** — найдите активные частоты в вашем районе
4. **Мониторьте APRS** — увидите пакеты от местных станций
5. **Следите за ISS** — используйте heavens-above.com для расписания

---

## 🎓 Следующие шаги

1. Прочитайте `scripts\SDR_README.md` для детальной информации
2. Поэкспериментируйте с разными частотами
3. Попробуйте принять NOAA APT изображение
4. Декодируйте APRS пакеты с Dire Wolf
5. Постройте антенну по расчётам из `sdr_antenna.py`

---

**Удачного приёма! 📡**

*Обновлено: 12 апреля 2026 г.*
