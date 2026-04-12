# SDR Scripts — Руководство по приёму сигналов

Набор улучшенных скриптов для приёма и анализа радиосигналов с помощью RTL-SDR V4.

## 📡 Обзор скриптов

| Скрипт | Назначение | Примеры использования |
|--------|------------|----------------------|
| `sdr_capture_enhanced.py` | **Универсальный приём и анализ** | FM, APRS, ISS, произвольные частоты |
| `sdr_noaa.py` | **Приём метеоспутников NOAA** | NOAA 18, 19 (APT изображения) |
| `sdr_ax25.py` | **Приём AX.25 пакетов** | APRS, CubeSat телеметрия |
| `sdr_scanner.py` | **Сканер спектра** | Поиск активных частот |
| `sdr_capture.py` | **Старая версия** | (для обратной совместимости) |

---

## 🚀 Быстрый старт

### Требования

1. **RTL-SDR V4** подключён к USB
2. **Драйвер WinUSB** установлен через Zadig
3. **rtlsdr.dll** доступен в `C:\SDRSharp\tmp\x64\` или `C:\SDRSharp\`

### Проверка устройства

```bash
python scripts\test_rtlsdr.py
```

Должно показать:
```
[OK] Devices found: 1
[OK] Device opened
Tuner: R820T
```

---

## 📖 Скрипты

### 1. sdr_capture_enhanced.py — Универсальный приём

**Возможности:**
- ✅ Улучшенная обработка I/Q (DC correction)
- ✅ Точный FFT с окном Хэмминга
- ✅ Автоматическая детекция пиков
- ✅ Измерение SNR (отношение сигнал/шум)
- ✅ Непрерывный мониторинг
- ✅ Сохранение в WAV (для аудио)
- ✅ Цветной ASCII спектр

**Примеры:**

```bash
# Приём ISS (145.8 МГц) со спектром
python scripts\sdr_capture_enhanced.py --freq 145.8 --plot

# Приём с усилением
python scripts\sdr_capture_enhanced.py --freq 436.7 --gain 40 --plot

# Измерение SNR
python scripts\sdr_capture_enhanced.py --freq 144.8 --snr

# Непрерывный мониторинг FM
python scripts\sdr_capture_enhanced.py --freq 100.0 --monitor --plot

# Сохранить I/Q
python scripts\sdr_capture_enhanced.py --freq 433.92 --output iq.bin

# Сохранить как WAV (для FM)
python scripts\sdr_capture_enhanced.py --freq 100.0 --wav fm.wav

# С коррекцией DC
python scripts\sdr_capture_enhanced.py --freq 145.8 --correct-dc --plot
```

**Параметры:**
```
--freq       Частота МГц (по умолч. 100.0)
--rate       Частота дискретизации MS/s (по умолч. 2.4)
--gain       Усиление dB (0 = авто)
--samples    Количество семплов (по умолч. 16384)
--plot       Показать спектр
--peaks      Детекция пиков
--snr        Измерение SNR
--monitor    Непрерывный мониторинг
--output     Сохранить I/Q в файл
--wav        Сохранить как WAV
--correct-dc Коррекция DC offset
```

---

### 2. sdr_noaa.py — Приём спутников NOAA

**Назначение:** Приём APT изображений с метеоспутников NOAA.

**Частоты спутников:**
- **NOAA 18**: 137.9125 МГц
- **NOAA 19**: 137.1000 МГц
- **NOAA 20**: 137.3500 МГц (не поддерживает APT)

**Примеры:**

```bash
# Приём NOAA 19
python scripts\sdr_noaa.py

# Приём NOAA 18
python scripts\sdr_noaa.py --sat noaa18

# С усилением
python scripts\sdr_noaa.py --sat noaa19 --gain 40

# Сканирование для поиска сигнала
python scripts\sdr_noaa.py --sat noaa19 --scan

# Сохранить I/Q
python scripts\sdr_noaa.py --sat noaa18 --output noaa18.bin
```

**Далее для декодирования:**
1. Установите [WXtoImg](https://wxtoimgrestored.xyz/) или [aptdec](https://github.com/Xerbo/aptdec)
2. Конвертируйте I/Q в WAV: 
   ```bash
   sox -t raw -r 11025 -e unsigned-integer -b 8 -c 1 noaa.bin noaa.wav
   ```
3. Откройте в WXtoImg для получения изображения

---

### 3. sdr_ax25.py — Приём AX.25 пакетов

**Назначение:** Приём APRS пакетов и телеметрии CubeSat.

**Частоты:**
- **APRS Европа**: 144.800 МГц
- **APRS США**: 144.390 МГц
- **ISS APRS**: 145.825 МГц

**Модуляция:** AFSK 1200 бод (Mark: 1200 Гц, Space: 2200 Гц)

**Примеры:**

```bash
# APRS Европа
python scripts\sdr_ax25.py

# ISS APRS
python scripts\sdr_ax25.py --freq 145.825

# С декодированием пакетов
python scripts\sdr_ax25.py --decode

# Непрерывный мониторинг
python scripts\sdr_ax25.py --monitor --decode

# Сохранить I/Q
python scripts\sdr_ax25.py --output aprs.bin
```

**Для полного декодирования APRS:**
Используйте [Dire Wolf](https://github.com/wb2osz/direwolf):
```bash
direwolf -r 11025 -b 8 -c sdr_ax25.conf
```

---

### 4. sdr_scanner.py — Сканер спектра

**Назначение:** Поиск активных сигналов в заданном диапазоне частот.

**Примеры:**

```bash
# Сканирование FM (88-108 МГц)
python scripts\sdr_scanner.py

# VHF диапазон (137-146 МГц)
python scripts\sdr_scanner.py --start 137 --stop 146

# UHF диапазон (430-440 МГц)
python scripts\sdr_scanner.py --start 430 --stop 440

# Быстрое сканирование
python scripts\sdr_scanner.py --step 0.5 --threshold -30

# С гистограммой
python scripts\sdr_scanner.py --start 144 --stop 146 --plot

# Сохранить в CSV
python scripts\sdr_scanner.py --start 88 --stop 108 --output fm_scan.csv
```

**Известные частоты для маркировки:**
- 88-108 МГц: FM радиовещание
- 137.1 МГц: NOAA 19
- 137.9125 МГц: NOAA 18
- 144.8 МГц: APRS (Европа)
- 145.825 МГц: ISS APRS
- 156.8 МГц: Морской канал 16 (distress)
- 433.92 МГц: ISM 70cm
- 436.7 МГц: ISS SSTV

---

## 🎯 Рекомендации по приёму

### FM радиовещание (88-108 МГц)
```bash
python scripts\sdr_capture_enhanced.py --freq 100.0 --gain 30 --plot --wav fm.wav
```
- **Антенна:** Четвертьволновая (75 см) или диполь
- **Усиление:** 30-40 dB
- **Ожидания:** Отличный приём в городе

### APRS (144.800 МГц)
```bash
python scripts\sdr_ax25.py --decode --monitor
```
- **Антенна:** Вертикальная 1/4 волны (50 см)
- **Усиление:** 35-45 dB
- **Ожидания:** Зависит от активности в вашем районе

### ISS (145.800 МГц)
```bash
python scripts\sdr_capture_enhanced.py --freq 145.8 --gain 40 --plot
```
- **Антенна:** Направленная на ISS
- **Усиление:** 40-50 dB
- **Ожидания:** Только во время пролёта ISS
- **Полезные ресурсы:**
  - [ISS Transit Finder](https://transit-finder.com/)
  - [Heavens-Above](https://www.heavens-above.com/)

### NOAA APT (137 МГц)
```bash
python scripts\sdr_noaa.py --sat noaa19 --gain 40 --output noaa.bin
```
- **Антенна:** Квадратурная или диполь (52 см)
- **Усиление:** 40-50 dB
- **Ожидания:** 10-15 минут пролёта
- **Полезные ресурсы:**
  - [NOAA Pass Predictor](https://www.n2yo.com/)

### UHF (430-440 МГц)
```bash
python scripts\sdr_scanner.py --start 430 --stop 440 --plot
```
- **Антенна:** Компактная (17 см для 1/4 волны)
- **Усиление:** 40-50 dB
- **Ожидания:** Локальные репитеры, ISS SSTV

---

## 🔧 Устранение проблем

### Устройство не обнаружено
```
No RTL-SDR devices!
```

**Решение:**
1. Проверьте подключение USB
2. Установите драйвер WinUSB через [Zadig](https://zadig.akeo.ie/)
3. Перезапустите скрипт

### Ошибка чтения
```
ERROR: Read failed (error -1)
```

**Решение:**
1. Уменьшите `--samples` до 4096
2. Увеличьте задержку между чтениями
3. Проверьте кабель USB

### Слабый сигнал
```
Сигнал: СЛАБЫЙ
```

**Решение:**
1. Увеличьте усиление: `--gain 40`
2. Улучшите антенну
3. Переместитесь ближе к окну
4. Проверьте, есть ли передача на частоте

### Нет пиков на спектре
**Решение:**
1. Увеличьте усиление
2. Уменьшите порог: `--threshold -60`
3. Проверьте, что частота правильная
4. Увеличьте количество сэмплов: `--samples 32768`

---

## 📊 Понимание результатов

### Мощность сигнала (dB)
- **>-20 dB:** Очень сильный сигнал
- **-20 до -40 dB:** Сильный сигнал
- **-40 до -60 dB:** Средний сигнал
- **<-60 dB:** Слабый сигнал

### SNR (отношение сигнал/шум)
- **>20 dB:** Отличное качество
- **10-20 dB:** Хорошее качество
- **5-10 dB:** Среднее качество
- **<5 dB:** Низкое качество

### Динамический диапазон
Разница между самым сильным и слабым сигналом в сэмплах. Большой диапазон (>30 dB) указывает на наличие сильных сигналов на фоне шума.

---

## 🎓 Дополнительные ресурсы

### Программы для работы с SDR
- **SDR#** — popular SDR receiver software
- **SDR++** — кроссплатформенный SDR
- **GQRX** — SDR для Linux/Mac
- **HDSDR** — SDR для Windows

### Декодирование
- **Dire Wolf** — AX.25/APRS decoder
- **WXtoImg** — NOAA APT decoder
- **aptdec** — NOAA APT decoder (Python)
- **Multimon-NG** — multi-mode decoder

### Онлайн ресурсы
- [RTL-SDR Blog](https://www.rtl-sdr.com/)
- [RTL-SDR.com Tutorials](https://www.rtl-sdr.com/big-list-rtl-sdr-tutorials/)
- [APRS.fi](https://aprs.fi/) — APRS map
- [Heavens-Above](https://www.heavens-above.com/) — Satellite tracking

---

## 📝 Примеры типичных сценариев

### Сценарий 1: Мониторинг FM радио
```bash
# Настройка на популярную станцию
python scripts\sdr_capture_enhanced.py --freq 100.0 --gain 30 --plot --wav fm.wav

# Прослушайте файл fm.wav в любом плеере
```

### Сценарий 2: Поиск APRS пакетов
```bash
# Мониторинг APRS с декодированием
python scripts\sdr_ax25.py --monitor --decode

# Если нашли пакеты, используйте Dire Wolf для полного декодирования
```

### Сценарий 3: Приём изображения NOAA
```bash
# 1. Проверите расписание пролёта NOAA
# 2. Запустите приём
python scripts\sdr_noaa.py --sat noaa19 --gain 40 --output noaa.bin

# 3. Конвертируйте в WAV
sox -t raw -r 11025 -e unsigned-integer -b 8 -c 1 noaa.bin noaa.wav

# 4. Откройте в WXtoImg
```

### Сценарий 4: Сканирование диапазона
```bash
# Поиск активных сигналов VHF
python scripts\sdr_scanner.py --start 137 --stop 146 --plot --output vhf_scan.csv

# Откройте CSV в Excel для анализа
```

---

## ⚠️ Правовое предупреждение

**Разрешено:**
- ✅ Приём FM радиовещания
- ✅ Приём NOAA APT
- ✅ Приём APRS (публичные пакеты)
- ✅ Сканирование спектра (только приём)

**Запрещено (без лицензии):**
- ❌ Передача на любительских частотах
- ❌ Декодирование зашифрованных сигналов
- ❌ Прослушивание полицейских/авиационных частот (зависит от страны)

**Проверьте местное законодательство!**

---

## 🤝 Вклад в проект

Если у вас есть улучшения или исправления:
1. Создайте issue с описанием
2. Или отправьте pull request
3. Убедитесь, что код работает на Windows с RTL-SDR V4

---

## 📄 Лицензия

Часть проекта MKA Embedded. См. основной LICENSE файл.

---

**Владелец проекта:** Дуплей Максим Игоревич  
**Дата:** 12 апреля 2026 г.
