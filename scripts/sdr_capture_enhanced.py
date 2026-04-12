"""
Улучшенный RTL-SDR V4 — приём и анализ сигналов

Возможности:
  - Улучшенная обработка I/Q (DC correction, нормализация)
  - Точный FFT с окном Хэмминга
  - Автоматическая детекция пиков сигналов
  - Измерение SNR (отношение сигнал/шум)
  - Мониторинг в реальном времени
  - Сохранение в WAV (для аудио мониторинга)
  - Цветной ASCII спектр
  - Статистика по всем сэмплам

Примеры использования:
  python scripts\\sdr_capture_enhanced.py --freq 145.8 --plot          # ISS
  python scripts\\sdr_capture_enhanced.py --freq 436.7 --gain 40       # ISS UHF
  python scripts\\sdr_capture_enhanced.py --freq 137.1 --rate 2.4      # NOAA 18
  python scripts\\sdr_capture_enhanced.py --freq 100.0 --monitor       # FM мониторинг
  python scripts\\sdr_capture_enhanced.py --freq 144.8 --snr           # APRS SNR
  python scripts\\sdr_capture_enhanced.py --freq 433.92 --output iq.bin # Сохранить I/Q
  python scripts\\sdr_capture_enhanced.py --freq 100.0 --wav fm.wav    # Сохранить WAV
"""

import ctypes
import sys
import os
import math
import argparse
import struct
import time
from collections import defaultdict

# Путь к DLL
DLL_PATHS = [
    r"C:\SDRSharp\tmp\x64\rtlsdr.dll",
    r"C:\SDRSharp\rtlsdr.dll",
    r"rtlsdr.dll",
]


def load_rtlsdr():
    """Загрузить rtlsdr.dll"""
    for path in DLL_PATHS:
        if os.path.exists(path):
            try:
                dll = ctypes.windll.LoadLibrary(path)
                print(f"[OK] DLL loaded: {path}")
                return dll
            except OSError as e:
                print(f"  Skip {path}: {e}")
    print("ERROR: rtlsdr.dll not found!")
    print("  Put rtlsdr.dll in PATH or set DLL_PATHS in script")
    return None


def correct_dc(i_samples, q_samples):
    """Коррекция DC offset (среднее значение)"""
    n = len(i_samples)
    if n == 0:
        return i_samples, q_samples
    
    mean_i = sum(i_samples) / n
    mean_q = sum(q_samples) / n
    
    i_corrected = [i - mean_i for i in i_samples]
    q_corrected = [q - mean_q for q in q_samples]
    
    return i_corrected, q_corrected


def iq_to_complex(raw_bytes):
    """Преобразовать сырые байты в комплексные семплы (I+jQ)"""
    samples = list(raw_bytes)
    # RTL-SDR выдаёт 8-бит unsigned I/Q
    iq = [(s - 127.5) / 127.5 for s in samples]  # нормализация [-1, 1]
    return iq[0::2], iq[1::2]  # I, Q


def compute_power(i_samples, q_samples):
    """Вычислить мощность сигнала"""
    return [i*i + q*q for i, q in zip(i_samples, q_samples)]


def compute_db(power):
    """Преобразовать мощность в dB"""
    return [10 * math.log10(p + 1e-12) for p in power]


def hamming_window(n, N):
    """Окно Хэмминга для FFT"""
    return 0.54 - 0.46 * math.cos(2 * math.pi * n / (N - 1))


def fft_cooley_tukey(x):
    """Быстрый FFT Кули-Тьюки (рекурсивный)
    
    Работает только с размером степени 2.
    Сложность: O(N log N) вместо O(N²)
    """
    N = len(x)
    if N <= 1:
        return x
    if N % 2 != 0:
        # Если не степень 2, используем DFT
        return fft_dft(x)
    
    # Разделяем на чётные и нечётные
    even = fft_cooley_tukey(x[0::2])
    odd = fft_cooley_tukey(x[1::2])
    
    # Комбинием
    T = [math.cos(-2 * math.pi * k / N) + 1j * math.sin(-2 * math.pi * k / N) for k in range(N//2)]
    return [even[k] + T[k] * odd[k] for k in range(N//2)] + \
           [even[k] - T[k] * odd[k] for k in range(N//2)]


def fft_dft(x):
    """Прямое DFT (для малых N или когда FFT невозможен)"""
    N = len(x)
    result = []
    for k in range(N):
        re = 0.0
        im = 0.0
        for n in range(N):
            angle = -2 * math.pi * k * n / N
            re += x[n] * math.cos(angle)
            im += x[n] * math.sin(angle)
        result.append(complex(re, im))
    return result


def next_power_of_2(n):
    """Ближайшая степень 2, большая или равная n"""
    return 1 << (n - 1).bit_length()


def fft_fast(i_samples, q_samples, num_bins):
    """Оптимизированный FFT с окном Хэмминга
    
    Использует FFT Кули-Тьюки O(N log N) для больших данных.
    Для малых данных (< 128 сэмплов) использует прямой DFT.
    """
    n = min(len(i_samples), len(q_samples))
    if n == 0:
        return []
    
    # Для малых данных используем прямой DFT (быстрее из-за меньших накладных расходов)
    if n < 128:
        target_size = n
        step = 1
    else:
        # Берём степень 2 для FFT (макс 2048)
        target_size = min(2048, n)
        # Выравниваем до степени 2
        if target_size & (target_size - 1) != 0:
            target_size = 1 << (target_size - 1).bit_length()
            target_size = min(target_size, 2048)
        step = max(1, n // target_size)
    
    i_sub = i_samples[::step][:target_size]
    q_sub = q_samples[::step][:target_size]
    n = len(i_sub)
    
    # Если размер не степень 2, обрезаем
    if n > 1 and (n & (n - 1)) != 0:
        n = 1 << (n - 1).bit_length()
        n = n // 2  # Берём меньшую степень 2
        i_sub = i_sub[:n]
        q_sub = q_sub[:n]
    
    # Применяем окно Хэмминга
    window = [hamming_window(i, n) for i in range(n)]
    i_windowed = [i_sub[i] * window[i] for i in range(n)]
    q_windowed = [q_sub[i] * window[i] for i in range(n)]
    
    # Для малых данных используем прямой DFT
    if n < 128:
        mags = []
        for k in range(num_bins):
            freq_bin = k * n / num_bins
            re = 0.0
            im = 0.0
            for t in range(n):
                angle = -2 * math.pi * freq_bin * t / n
                re += i_windowed[t] * math.cos(angle) - q_windowed[t] * math.sin(angle)
                im += i_windowed[t] * math.sin(angle) + q_windowed[t] * math.cos(angle)
            mag = 10 * math.log10(re*re + im*im + 1e-12)
            mags.append(mag)
        return mags
    
    # Создаём комплексный сигнал для FFT
    x = [complex(i_windowed[i], q_windowed[i]) for i in range(n)]
    
    # Быстрый FFT Кули-Тьюки
    X = fft_cooley_tukey(x)
    
    # Вычисляем магнитуды - берём каждый (n/num_bins)-й бин
    mags = []
    bins_per_output = n // num_bins
    
    for k in range(num_bins):
        idx = k * bins_per_output
        if idx < len(X):
            mag = X[idx].real ** 2 + X[idx].imag ** 2
            mag_db = 10 * math.log10(mag + 1e-12)
            mags.append(mag_db)
        else:
            mags.append(-120.0)  # Минимальное значение
    
    return mags


def fft_hamming(i_samples, q_samples, num_bins):
    """FFT с окном Хэмминга для лучшего разрешения
    
    Обёртка для быстрой совместимости.
    """
    return fft_fast(i_samples, q_samples, num_bins)


def detect_peaks(mags, freq_start_mhz, sample_rate_mhz, threshold_db=-20, min_distance=2):
    """Автоматическая детекция пиков сигналов"""
    if not mags:
        return []
    
    n = len(mags)
    peaks = []
    
    for i in range(1, n - 1):
        # Проверяем, что это локальный максимум
        if mags[i] > mags[i-1] and mags[i] > mags[i+1]:
            # Проверяем порог
            if mags[i] > threshold_db:
                # Проверяем расстояние от предыдущего пика
                if not peaks or (i - peaks[-1]['bin']) >= min_distance:
                    freq = freq_start_mhz + (i / n) * sample_rate_mhz
                    peaks.append({
                        'bin': i,
                        'freq': freq,
                        'power': mags[i],
                        'index': i
                    })
    
    # Сортируем по мощности (сильные первые)
    peaks.sort(key=lambda p: p['power'], reverse=True)
    return peaks[:10]  # Топ-10 пиков


def compute_snr(mags, signal_bins, noise_buffer=5):
    """Вычисление SNR для обнаруженного сигнала"""
    if not mags or not signal_bins:
        return None
    
    n = len(mags)
    signal_power = mags[signal_bins[0]]
    
    # Шум берём из соседних бинов
    noise_bins = []
    for i in range(n):
        if all(abs(i - sb) > noise_buffer for sb in signal_bins):
            noise_bins.append(mags[i])
    
    if not noise_bins:
        return None
    
    noise_power = sum(noise_bins) / len(noise_bins)
    snr = signal_power - noise_power
    
    return {
        'signal_db': signal_power,
        'noise_db': noise_power,
        'snr_db': snr
    }


def ascii_spectrum_colored(mags, freq_start_mhz, sample_rate_mhz, width=80, height=20, peaks=None):
    """Цветной ASCII спектр с маркерами пиков"""
    if not mags:
        return ""

    # Нормализация
    min_mag = min(mags)
    max_mag = max(mags)
    mag_range = max_mag - min_mag if max_mag > min_mag else 1

    # Создаём карту пиков для быстрого доступа
    peak_map = {}
    if peaks:
        for p in peaks:
            bin_idx = int(p['index'] * len(mags) / 64) if len(mags) != 64 else p['index']
            peak_map[bin_idx] = p

    lines = []
    # Заголовок
    lines.append(f"  Частота: {freq_start_mhz:.3f} - {freq_start_mhz + sample_rate_mhz:.3f} МГц")
    lines.append(f"  Мощность: {min_mag:.1f} до {max_mag:.1f} dB")
    lines.append(f"  Полоса: {sample_rate_mhz:.3f} МГц | Разрешение: {sample_rate_mhz/len(mags):.3f} кГц/бин")
    lines.append("")

    # Рисуем по строкам сверху вниз
    for row in range(height, 0, -1):
        threshold = min_mag + (mag_range * row / height)
        line = f"{threshold:+7.1f} |"
        for idx, mag in enumerate(mags):
            if idx in peak_map:
                line += "▼"  # Маркер пика
            elif mag >= threshold:
                line += "█"
            elif mag >= threshold - mag_range / height:
                line += "▓"
            elif mag >= threshold - 2 * mag_range / height:
                line += "▒"
            elif mag >= threshold - 3 * mag_range / height:
                line += "░"
            else:
                line += " "
        lines.append(line)

    # Ось частот
    lines.append("         +" + "-" * len(mags) + "+")
    
    # Метки частот
    num_marks = min(5, len(mags))
    freq_mark_line = "         |"
    for i in range(num_marks):
        pos = int(i * len(mags) / (num_marks - 1))
        freq = freq_start_mhz + (pos / len(mags)) * sample_rate_mhz
        freq_mark_line += f"{freq:.1f}".center(pos - (len(freq_mark_line) - 8))
        freq_mark_line = freq_mark_line.ljust(pos + 1)
    
    lines.append(freq_mark_line[:8 + len(mags) + 1])

    return "\n".join(lines)


def ascii_spectrum(mags, freq_start_mhz, sample_rate_mhz, width=80, height=20):
    """ASCII спектральная диаграмма (без цвета)"""
    return ascii_spectrum_colored(mags, freq_start_mhz, sample_rate_mhz, width, height)


def save_wav(i_samples, q_samples, filename, sample_rate=48000):
    """Сохранить I/Q как аудио WAV (демо для FM)"""
    try:
        import wave
        
        # Конвертируем в аудио (I канал)
        audio = i_samples
        n = len(audio)
        
        # Нормализация до 16-bit
        max_val = max(abs(x) for x in audio)
        if max_val > 0:
            audio = [int(x / max_val * 32767) for x in audio]
        else:
            audio = [0] * n
        
        # Записываем WAV
        with wave.open(filename, 'wb') as wav_file:
            wav_file.setnchannels(1)  # Mono
            wav_file.setsampwidth(2)  # 16-bit
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(struct.pack(f'<{n}h', *audio))
        
        print(f"[OK] Сохранено {n} сэмплов в WAV: {filename}")
        return True
    except ImportError:
        print("  WARN: Модуль wave не найден, пропускаем WAV")
        return False
    except Exception as e:
        print(f"  ERROR: Ошибка сохранения WAV: {e}")
        return False


def print_statistics(i_samples, q_samples, args):
    """Расширенная статистика сигнала"""
    power = compute_power(i_samples, q_samples)
    power_db = compute_db(power)

    avg_power = sum(power) / len(power)
    avg_db = 10 * math.log10(avg_power + 1e-12)
    peak_db = max(power_db)
    min_db = min(power_db)

    # DC offset
    mean_i = sum(i_samples) / len(i_samples)
    mean_q = sum(q_samples) / len(q_samples)
    
    # RMS
    rms_i = math.sqrt(sum(i*i for i in i_samples)/len(i_samples))
    rms_q = math.sqrt(sum(q*q for q in q_samples)/len(q_samples))
    
    # Амплитуда и фаза
    amplitudes = [math.sqrt(i**2 + q**2) for i, q in zip(i_samples, q_samples)]
    avg_amp = sum(amplitudes) / len(amplitudes)
    
    phases = [math.atan2(q, i) * 180 / math.pi for i, q in zip(i_samples, q_samples)]
    avg_phase = sum(phases) / len(phases)

    print(f"\n{'='*60}")
    print(f"  Статистика сигнала")
    print(f"{'='*60}")
    print(f"  Средняя мощность:  {avg_db:.1f} dB")
    print(f"  Пиковая мощность:  {peak_db:.1f} dB")
    print(f"  Мин. мощность:     {min_db:.1f} dB")
    print(f"  Динамич. диапазон: {peak_db - min_db:.1f} dB")
    print(f"  DC offset I:       {mean_i:+.4f}")
    print(f"  DC offset Q:       {mean_q:+.4f}")
    print(f"  RMS I:             {rms_i:.4f}")
    print(f"  RMS Q:             {rms_q:.4f}")
    print(f"  Средняя амплитуда: {avg_amp:.4f}")
    print(f"  Средняя фаза:      {avg_phase:+.1f}°")
    print(f"  Всего сэмплов:     {len(i_samples)}")


def print_peak_peaks(peaks):
    """Вывести обнаруженные пики"""
    if not peaks:
        print("  Пики не обнаружены")
        return
    
    print(f"\n  Обнаружено пиков: {len(peaks)}")
    print(f"  {'№':<4} {'Частота (МГц)':<15} {'Мощность (dB)':<15}")
    print(f"  {'-'*34}")
    for idx, peak in enumerate(peaks, 1):
        print(f"  {idx:<4} {peak['freq']:<15.4f} {peak['power']:<15.1f}")


def main():
    parser = argparse.ArgumentParser(description="RTL-SDR V4 — улучшенный приём и анализ сигналов")
    parser.add_argument("--freq", type=float, default=100.0,
                        help="Центральная частота МГц (по умолч. 100.0)")
    parser.add_argument("--rate", type=float, default=2.4,
                        help="Частота дискретизации MS/s (по умолч. 2.4)")
    parser.add_argument("--gain", type=float, default=0,
                        help="Усиление dB (0 = авто)")
    parser.add_argument("--samples", type=int, default=16384,
                        help="Количество семплов (по умолч. 16384)")
    parser.add_argument("--device", type=int, default=0,
                        help="Номер устройства (по умолч. 0)")
    parser.add_argument("--plot", action="store_true",
                        help="Показать спектр")
    parser.add_argument("--peaks", action="store_true",
                        help="Детекция пиков")
    parser.add_argument("--snr", action="store_true",
                        help="Измерение SNR")
    parser.add_argument("--monitor", action="store_true",
                        help="Непрерывный мониторинг")
    parser.add_argument("--output", type=str, default="",
                        help="Сохранить I/Q в файл (.bin)")
    parser.add_argument("--wav", type=str, default="",
                        help="Сохранить как WAV (для FM)")
    parser.add_argument("--correct-dc", action="store_true",
                        help="Коррекция DC offset")

    args = parser.parse_args()

    freq_hz = int(args.freq * 1e6)
    rate_hz = int(args.rate * 1e6)

    print("=" * 60)
    print("  RTL-SDR V4 — улучшенный приём сигналов")
    print("=" * 60)
    print()

    # Загрузка DLL
    dll = load_rtlsdr()
    if not dll:
        return 1

    # Проверка устройств
    count = dll.rtlsdr_get_device_count()
    print(f"\n[OK] Устройств найдено: {count}")
    if count == 0:
        print("\nНет RTL-SDR устройств!")
        print("Проверьте: устройство подключено, драйвер WinUSB установлен (Zadig)")
        return 1

    # Инфо об устройстве
    print(f"\n--- Устройство {args.device} ---")
    name = ctypes.create_string_buffer(256)
    manufacturer = ctypes.create_string_buffer(256)
    serial = ctypes.create_string_buffer(256)
    dll.rtlsdr_get_device_usb_strings(args.device, manufacturer, serial, name)
    print(f"  Название:       {name.value.decode('utf-8', errors='replace')}")
    print(f"  Производитель:  {manufacturer.value.decode('utf-8', errors='replace')}")
    print(f"  Серийный номер: {serial.value.decode('utf-8', errors='replace')}")

    # Открытие устройства
    dev = ctypes.c_void_p()
    result = dll.rtlsdr_open(ctypes.byref(dev), args.device)
    if result != 0 or not dev:
        print(f"  ОШИБКА: Открытие не удалось (ошибка {result})")
        return 1
    print(f"  [OK] Устройство открыто")

    # Настройка тюнера
    print(f"\n--- Настройка ---")
    print(f"  Центр. частота: {args.freq} МГц ({freq_hz} Гц)")
    print(f"  Частота дискр.: {args.rate} MS/s ({rate_hz} Гц)")
    print(f"  Усиление: {'Авто' if args.gain == 0 else f'{args.gain} dB'}")

    dll.rtlsdr_set_sample_rate(dev, rate_hz)
    dll.rtlsdr_set_center_freq(dev, freq_hz)

    if args.gain > 0:
        # Получить список доступных усилений
        num_gains = dll.rtlsdr_get_tuner_gains(dev, None, 0)
        if num_gains > 0:
            gains = (ctypes.c_int * num_gains)()
            dll.rtlsdr_get_tuner_gains(dev, gains, num_gains)
            available = list(gains[:num_gains])
            # Найти ближайшее
            target_gain = int(args.gain * 10)
            closest = min(available, key=lambda g: abs(g - target_gain))
            dll.rtlsdr_set_tuner_gain(dev, closest)
            print(f"  Усиление: {closest / 10} dB (доступно: {[g/10 for g in available]})")
        else:
            dll.rtlsdr_set_tuner_gain(dev, int(args.gain * 10))
            print(f"  Усиление: {args.gain} dB")
    else:
        dll.rtlsdr_set_tuner_gain_mode(dev, 0)  # AGC
        print(f"  Усиление: Автоматическое (AGC)")

    # Reset buffer
    dll.rtlsdr_reset_buffer(dev)

    # Получение семплов
    print(f"\n--- Приём {args.samples} сэмплов ---")

    if args.monitor:
        # Непрерывный мониторинг
        print("  [MONITOR MODE] Нажмите Ctrl+C для выхода")
        print()
        try:
            iteration = 0
            while True:
                iteration += 1
                buf_len = min(args.samples, 8192) * 2
                buf = ctypes.create_string_buffer(buf_len)
                n_read = ctypes.c_uint32(0)

                result = dll.rtlsdr_read_sync(dev, buf, buf_len, ctypes.byref(n_read))
                
                if result == 0 and n_read.value > 0:
                    raw = buf.raw[:n_read.value]
                    i_samples, q_samples = iq_to_complex(raw)
                    
                    if args.correct_dc:
                        i_samples, q_samples = correct_dc(i_samples, q_samples)
                    
                    power = compute_power(i_samples, q_samples)
                    avg_power = sum(power) / len(power)
                    avg_db = 10 * math.log10(avg_power + 1e-12)
                    
                    # Быстрый спектр
                    num_bins = 64
                    mags = fft_hamming(i_samples, q_samples, num_bins)
                    
                    # Очистка экрана
                    os.system('cls' if os.name == 'nt' else 'clear')
                    print(f"\n{'='*60}")
                    print(f"  Мониторинг: {args.freq} МГц | Итерация: {iteration}")
                    print(f"  Мощность: {avg_db:.1f} dB | Время: {time.strftime('%H:%M:%S')}")
                    print(f"{'='*60}\n")
                    
                    peaks = detect_peaks(mags, args.freq, args.rate, threshold_db=-25)
                    if peaks:
                        print(f"  Обнаружено пиков: {len(peaks)}")
                        for p in peaks[:5]:
                            print(f"    {p['freq']:.4f} МГц @ {p['power']:.1f} dB")
                    
                    print()
                    if args.plot:
                        print(ascii_spectrum_colored(mags, args.freq, args.rate, height=15))
                    
                    time.sleep(0.5)
                else:
                    print(f"  ОШИБКА чтения (ошибка {result})")
                    time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n\n[OK] Мониторинг остановлен")
    else:
        # Однократный приём
        buf_len = args.samples * 2
        buf = ctypes.create_string_buffer(buf_len)
        n_read = ctypes.c_uint32(0)

        result = dll.rtlsdr_read_sync(dev, buf, buf_len, ctypes.byref(n_read))

        if result != 0:
            print(f"  ОШИБКА: Чтение не удалось (ошибка {result})")
            print("  Пробуем асинхронный режим...")

            # Async callback
            callback_data = {"samples": b"", "done": False, "error": None}

            @ctypes.CFUNCTYPE(None, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_uint32, ctypes.c_void_p)
            def rtlsdr_callback(samples, sample_len, ctx):
                callback_data["samples"] += bytes(samples[:sample_len])
                if len(callback_data["samples"]) >= buf_len:
                    callback_data["done"] = True

            result = dll.rtlsdr_read_async(dev, rtlsdr_callback, None, 0, 16)
            timeout = 5.0
            start = time.time()
            while not callback_data["done"] and (time.time() - start) < timeout:
                time.sleep(0.1)

            if callback_data["samples"]:
                n_read.value = min(len(callback_data["samples"]), buf_len)
                buf.raw = callback_data["samples"][:buf_len]
                print(f"  [OK] Асинхронное чтение: {n_read.value} байт")
            else:
                print(f"  ОШИБКА: Асинхронное чтение не удалось ({result})")
                dll.rtlsdr_close(dev)
                return 1
        else:
            print(f"  [OK] Получено {n_read.value} байт ({n_read.value // 2} комплексных сэмплов)")

        # Обработка I/Q
        raw = buf.raw[:n_read.value]
        i_samples, q_samples = iq_to_complex(raw)
        
        # Коррекция DC
        if args.correct_dc:
            print("\n--- Коррекция DC ---")
            mean_i_before = sum(i_samples) / len(i_samples)
            mean_q_before = sum(q_samples) / len(q_samples)
            i_samples, q_samples = correct_dc(i_samples, q_samples)
            print(f"  DC offset до: I={mean_i_before:+.4f}, Q={mean_q_before:+.4f}")
            print(f"  DC offset после: I=0.0000, Q=0.0000")

        # Статистика
        print_statistics(i_samples, q_samples, args)

        # Спектр
        if args.plot or args.peaks or args.snr:
            print(f"\n--- Спектр ---")
            num_bins = min(64, n_read.value // 4)
            mags = fft_hamming(i_samples, q_samples, num_bins)
            
            # Детекция пиков
            if args.peaks:
                peaks = detect_peaks(mags, args.freq, args.rate, threshold_db=-25)
                print_peak_peaks(peaks)
            else:
                peaks = None
            
            # SNR
            if args.snr and peaks:
                print(f"\n--- SNR ---")
                signal_bins = [p['index'] for p in peaks[:3]]
                snr = compute_snr(mags, signal_bins)
                if snr:
                    print(f"  Сигнал:  {snr['signal_db']:.1f} dB")
                    print(f"  Шум:     {snr['noise_db']:.1f} dB")
                    print(f"  SNR:     {snr['snr_db']:.1f} dB")
                    if snr['snr_db'] > 20:
                        print(f"  Качество: ОТЛИЧНОЕ")
                    elif snr['snr_db'] > 10:
                        print(f"  Качество: ХОРОШЕЕ")
                    elif snr['snr_db'] > 5:
                        print(f"  Качество: СРЕДНЕЕ")
                    else:
                        print(f"  Качество: НИЗКОЕ")
                else:
                    print("  Не удалось измерить SNR")
            
            # Визуализация
            if mags:
                spectrum = ascii_spectrum_colored(mags, args.freq, args.rate, peaks=peaks)
                print(f"\n{spectrum}")
            else:
                print("  (недостаточно сэмплов для FFT)")

        # Сохранение I/Q
        if args.output:
            with open(args.output, "wb") as f:
                f.write(raw)
            print(f"\n[OK] Сохранено {len(raw)} байт в {args.output}")

        # Сохранение WAV
        if args.wav:
            print(f"\n--- Сохранение WAV ---")
            save_wav(i_samples, q_samples, args.wav)

    # Закрытие
    dll.rtlsdr_close(dev)
    print(f"\n[OK] Устройство закрыто")
    print("\n" + "=" * 60)

    return 0


if __name__ == "__main__":
    sys.exit(main())
