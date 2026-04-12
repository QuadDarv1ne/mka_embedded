"""
Сканер спектра RTL-SDR V4

Сканирует заданный диапазон частот и обнаруживает активные сигналы.

Использование:
  python scripts\\sdr_scanner.py                          # Сканирование FM (88-108 МГц)
  python scripts\\sdr_scanner.py --start 137 --stop 146   # VHF диапазон
  python scripts\\sdr_scanner.py --start 430 --stop 440   # UHF диапазон
  python scripts\\sdr_scanner.py --step 0.5 --threshold -30  # Быстрое сканирование
  python scripts\\sdr_scanner.py --plot                   # Показать спектр
  python scripts\\sdr_scanner.py --output scan.csv        # Сохранить результаты
"""

import ctypes
import sys
import os
import math
import argparse
import time
import csv

DLL_PATHS = [
    r"C:\SDRSharp\tmp\x64\rtlsdr.dll",
    r"C:\SDRSharp\rtlsdr.dll",
    r"rtlsdr.dll",
]

# Известные частоты для маркировки
KNOWN_FREQUENCIES = {
    88.0: "FM Broadcast начало",
    100.0: "FM Broadcast центр",
    108.0: "FM Broadcast конец",
    118.0: "Авиационная связь",
    121.5: "Аварийная частота (авиация)",
    137.0: "Спутники (начало)",
    137.1: "NOAA 19",
    137.9125: "NOAA 18",
    144.0: "2m Amateur начало",
    144.8: "APRS (Европа)",
    145.825: "ISS APRS",
    146.0: "2m Amateur конец",
    156.0: "Морская связь (начало)",
    156.8: "Морской канал 16 ( distress )",
    160.0: "Морская связь (конец)",
    430.0: "70cm Amateur начало",
    433.05: "LPD (низкая мощность)",
    433.92: "ISM 70cm",
    434.0: "PMR446",
    436.7: "ISS SSTV",
    440.0: "70cm Amateur конец",
}


def load_rtlsdr():
    for path in DLL_PATHS:
        if os.path.exists(path):
            try:
                dll = ctypes.windll.LoadLibrary(path)
                return dll
            except OSError:
                continue
    return None


def iq_to_complex(raw_bytes):
    samples = list(raw_bytes)
    iq = [(s - 127.5) / 127.5 for s in samples]
    return iq[0::2], iq[1::2]


def compute_power_db(i_samples, q_samples):
    power = [i*i + q*q for i, q in zip(i_samples, q_samples)]
    if not power:
        return -100
    avg_power = sum(power) / len(power)
    return 10 * math.log10(avg_power + 1e-12)


def fft_simple(i_samples, q_samples, num_bins):
    """Оптимизированный FFT для быстрого анализа"""
    n = min(len(i_samples), len(q_samples))
    if n == 0:
        return []
    
    # Для больших данных используем FFT
    if n >= 128:
        return fft_fast(i_samples, q_samples, num_bins)
    
    # Для малых данных используем прямой DFT
    target_size = min(1024, n)
    step = max(1, n // target_size)
    i_sub = i_samples[::step][:target_size]
    q_sub = q_samples[::step][:target_size]
    n = len(i_sub)

    mags = []
    for k in range(num_bins):
        freq_bin = k * n / num_bins
        re = 0.0
        im = 0.0
        for t in range(n):
            angle = -2 * math.pi * freq_bin * t / n
            re += i_sub[t] * math.cos(angle) - q_sub[t] * math.sin(angle)
            im += i_sub[t] * math.sin(angle) + q_sub[t] * math.cos(angle)
        mag = 10 * math.log10(re*re + im*im + 1e-12)
        mags.append(mag)

    return mags


def fft_cooley_tukey(x):
    """Быстрый FFT Кули-Тьюки (рекурсивный)"""
    N = len(x)
    if N <= 1:
        return x
    if N % 2 != 0:
        return fft_dft(x)
    
    even = fft_cooley_tukey(x[0::2])
    odd = fft_cooley_tukey(x[1::2])
    
    T = [math.cos(-2 * math.pi * k / N) + 1j * math.sin(-2 * math.pi * k / N) for k in range(N//2)]
    return [even[k] + T[k] * odd[k] for k in range(N//2)] + \
           [even[k] - T[k] * odd[k] for k in range(N//2)]


def fft_dft(x):
    """Прямое DFT"""
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


def fft_fast(i_samples, q_samples, num_bins):
    """Быстрый FFT для больших объёмов данных"""
    n = min(len(i_samples), len(q_samples))
    if n == 0:
        return []
    
    # Размер степени 2 (макс 2048)
    target_size = 1024
    if n > target_size:
        step = max(1, n // target_size)
        i_sub = i_samples[::step][:target_size]
        q_sub = q_samples[::step][:target_size]
    else:
        i_sub = i_samples[:n]
        q_sub = q_samples[:n]
        n = len(i_sub)
    
    # Обрезаем до степени 2
    if n & (n - 1) != 0:
        n = 1 << (n - 1).bit_length()
        i_sub = i_sub[:n]
        q_sub = q_sub[:n]
    
    # Создаём комплексный сигнал
    x = [complex(i_sub[i], q_sub[i]) for i in range(n)]
    
    # FFT
    X = fft_cooley_tukey(x)
    
    # Вычисляем магнитуды
    mags = []
    bins_per_output = len(X) // num_bins
    
    for k in range(num_bins):
        start = k * bins_per_output
        end = start + bins_per_output
        max_mag = 0
        for i in range(start, min(end, len(X))):
            mag = X[i].real ** 2 + X[i].imag ** 2
            if mag > max_mag:
                max_mag = mag
        mag_db = 10 * math.log10(max_mag + 1e-12)
        mags.append(mag_db)
    
    return mags


def scan_frequency_range(dll, dev, start_mhz, stop_mhz, step_mhz, rate_hz, 
                           samples=4096, threshold_db=-50, plot=False):
    """Сканирование диапазона частот"""
    
    print("=" * 60)
    print(f"  Сканер спектра")
    print("=" * 60)
    print(f"\n  Диапазон: {start_mhz:.3f} - {stop_mhz:.3f} МГц")
    print(f"  Шаг: {step_mhz:.3f} МГц")
    print(f"  Порог: {threshold_db} dB")
    print(f"  Сэмплов на частоте: {samples}")
    print()

    # Настройка sample rate
    dll.rtlsdr_set_sample_rate(dev, rate_hz)

    results = []
    freq = start_mhz
    total_steps = int((stop_mhz - start_mhz) / step_mhz) + 1
    current_step = 0

    while freq <= stop_mhz + 0.001:
        current_step += 1
        freq_hz = int(freq * 1e6)
        
        # Установка частоты
        dll.rtlsdr_set_center_freq(dev, freq_hz)
        time.sleep(0.05)  # Ждём стабилизации
        
        # Сброс буфера
        dll.rtlsdr_reset_buffer(dev)
        
        # Чтение сэмплов
        buf_len = samples * 2
        buf = ctypes.create_string_buffer(buf_len)
        n_read = ctypes.c_uint32(0)
        
        result = dll.rtlsdr_read_sync(dev, buf, buf_len, ctypes.byref(n_read))
        
        if result == 0 and n_read.value > 0:
            raw = buf.raw[:n_read.value]
            i_samples, q_samples = iq_to_complex(raw)
            power_db = compute_power_db(i_samples, q_samples)
            
            # Проверка порога
            if power_db > threshold_db:
                # Проверяем известные частоты
                known = None
                for kf, desc in KNOWN_FREQUENCIES.items():
                    if abs(freq - kf) < step_mhz * 0.6:
                        known = desc
                        break
                
                results.append({
                    'freq': freq,
                    'power': power_db,
                    'known': known
                })
                
                # Вывод найденного сигнала
                marker = " ***" if known else ""
                print(f"  [{current_step}/{total_steps}] {freq:.4f} МГц: {power_db:+6.1f} dB{marker}")
                if known:
                    print(f"         -> {known}")
        
        # Прогресс
        progress = current_step / total_steps * 100
        print(f"  Сканирование: {progress:.1f}% ({current_step}/{total_steps})", end='\r')
        
        freq += step_mhz

    print("\n")
    
    # Сортировка по мощности
    results.sort(key=lambda r: r['power'], reverse=True)
    
    return results


def print_results(results):
    """Вывод результатов сканирования"""
    if not results:
        print("  Сигналы не обнаружены")
        return
    
    print("=" * 60)
    print(f"  Результаты сканирования")
    print("=" * 60)
    print(f"\n  Всего сигналов: {len(results)}")
    print(f"\n  {'№':<4} {'Частота (МГц)':<15} {'Мощность (dB)':<15} {'Описание'}")
    print(f"  {'-'*50}")
    
    for idx, res in enumerate(results, 1):
        desc = res['known'] if res['known'] else ""
        print(f"  {idx:<4} {res['freq']:<15.4f} {res['power']:<15.1f} {desc}")
    
    print()


def ascii_spectrum_bar(results, max_items=20):
    """ASCII гистограмма сильнейших сигналов"""
    if not results:
        return
    
    print("=" * 60)
    print(f"  Топ-{min(len(results), max_items)} сигналов")
    print("=" * 60)
    print()
    
    # Топ сигналов
    top = results[:max_items]
    
    # Находим диапазон мощности
    min_power = min(r['power'] for r in top)
    max_power = max(r['power'] for r in top)
    power_range = max_power - min_power if max_power > min_power else 1
    
    # Рисуем гистограмму
    bar_width = 40
    for res in top:
        freq_str = f"{res['freq']:.3f}"
        power_norm = int((res['power'] - min_power) / power_range * bar_width)
        bar = "█" * power_norm
        desc = f" ({res['known']})" if res['known'] else ""
        print(f"  {freq_str:>8} МГц | {bar} {res['power']:+6.1f} dB{desc}")
    
    print()


def save_csv(results, filename):
    """Сохранение результатов в CSV"""
    try:
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['Частота (МГц)', 'Мощность (dB)', 'Описание'])
            for res in results:
                writer.writerow([
                    f"{res['freq']:.4f}",
                    f"{res['power']:.1f}",
                    res['known'] or ""
                ])
        print(f"[OK] Результаты сохранены в {filename}")
    except Exception as e:
        print(f"ОШИБКА сохранения CSV: {e}")


def main():
    parser = argparse.ArgumentParser(description="Сканер спектра RTL-SDR")
    parser.add_argument("--start", type=float, default=88.0,
                        help="Начальная частота МГц (по умолч. 88.0)")
    parser.add_argument("--stop", type=float, default=108.0,
                        help="Конечная частота МГц (по умолч. 108.0)")
    parser.add_argument("--step", type=float, default=0.1,
                        help="Шаг сканирования МГц (по умолч. 0.1)")
    parser.add_argument("--threshold", type=float, default=-50,
                        help="Порог обнаружения dB (по умолч. -50)")
    parser.add_argument("--rate", type=float, default=2.4,
                        help="Частота дискретизации MS/s")
    parser.add_argument("--gain", type=float, default=0,
                        help="Усиление dB (0 = авто)")
    parser.add_argument("--samples", type=int, default=4096,
                        help="Сэмплов на частоте")
    parser.add_argument("--plot", action="store_true",
                        help="Показать гистограмму")
    parser.add_argument("--output", type=str,
                        help="Сохранить в CSV")

    args = parser.parse_args()

    print("=" * 60)
    print("  Сканер спектра RTL-SDR V4")
    print("=" * 60)
    print()

    # Загрузка DLL
    dll = load_rtlsdr()
    if not dll:
        print("ERROR: rtlsdr.dll не найден!")
        return 1

    # Проверка устройств
    count = dll.rtlsdr_get_device_count()
    print(f"[OK] Устройств найдено: {count}")
    if count == 0:
        print("\nНет RTL-SDR устройств!")
        print("Проверьте: устройство подключено, драйвер WinUSB установлен (Zadig)")
        return 1

    # Открытие устройства
    dev = ctypes.c_void_p()
    result = dll.rtlsdr_open(ctypes.byref(dev), 0)
    if result != 0 or not dev:
        print(f"ОШИБКА: Открытие не удалось (ошибка {result})")
        return 1
    print(f"[OK] Устройство открыто")

    # Настройка усиления
    if args.gain > 0:
        dll.rtlsdr_set_tuner_gain(dev, int(args.gain * 10))
        print(f"[OK] Усиление: {args.gain} dB")
    else:
        dll.rtlsdr_set_tuner_gain_mode(dev, 0)
        print(f"[OK] Усиление: Авто (AGC)")

    rate_hz = int(args.rate * 1e6)

    # Сканирование
    start_time = time.time()
    results = scan_frequency_range(
        dll, dev, 
        args.start, args.stop, args.step, rate_hz,
        args.samples, args.threshold
    )
    elapsed = time.time() - start_time

    print(f"  Время сканирования: {elapsed:.1f} сек")
    print()

    # Вывод результатов
    print_results(results)
    
    if args.plot:
        ascii_spectrum_bar(results)
    
    # Сохранение
    if args.output:
        save_csv(results, args.output)

    # Закрытие
    dll.rtlsdr_close(dev)
    
    print(f"{'='*60}")
    print(f"  Сканирование завершено")
    print(f"{'='*60}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
