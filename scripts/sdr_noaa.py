"""
Приём сигналов метеорологических спутников NOAA (APT)

Частоты:
  NOAA 18: 137.9125 МГц
  NOAA 19: 137.1000 МГц
  NOAA 20: 137.3500 МГц (не поддерживает APT, только HRPT)

Модуляция: WFM (широкополосная FM)
Девиация: ~34 кГц
Полоса: ~40 кГц

Использование:
  python scripts\\sdr_noaa.py                  # NOAA 19
  python scripts\\sdr_noaa.py --sat noaa18              # NOAA 18
  python scripts\\sdr_noaa.py --sat noaa19 --gain 40    # NOAA 19 с усилением
  python scripts\\sdr_noaa.py --freq 137.1 --output noaa19.bin  # Произвольная частота
"""

import ctypes
import sys
import os
import math
import argparse
import time

DLL_PATHS = [
    r"C:\SDRSharp\tmp\x64\rtlsdr.dll",
    r"C:\SDRSharp\rtlsdr.dll",
    r"rtlsdr.dll",
]

# Частоты спутников
SATELLITES = {
    'noaa18': {
        'freq': 137.9125,
        'name': 'NOAA 18',
        'orbit': '98.6 мин',
        'altitude': '854 км'
    },
    'noaa19': {
        'freq': 137.1000,
        'name': 'NOAA 19',
        'orbit': '102 мин',
        'altitude': '870 км'
    },
    'noaa20': {
        'freq': 137.3500,
        'name': 'NOAA 20',
        'orbit': '101 мин',
        'altitude': '824 км',
        'note': 'Не поддерживает APT, только HRPT'
    }
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
    avg_power = sum(power) / len(power)
    return 10 * math.log10(avg_power + 1e-12)


def main():
    parser = argparse.ArgumentParser(description="Приём спутников NOAA (APT)")
    parser.add_argument("--sat", type=str, default="noaa19",
                        choices=list(SATELLITES.keys()),
                        help="Спутник (по умолч. noaa19)")
    parser.add_argument("--freq", type=float,
                        help="Ручная установка частоты МГц (переопределяет --sat)")
    parser.add_argument("--rate", type=float, default=2.4,
                        help="Частота дискретизации MS/s")
    parser.add_argument("--gain", type=float, default=0,
                        help="Усиление dB (0 = авто)")
    parser.add_argument("--samples", type=int, default=65536,
                        help="Количество семплов")
    parser.add_argument("--output", type=str,
                        help="Сохранить I/Q в файл")
    parser.add_argument("--scan", action="store_true",
                        help="Сканирование часты для поиска сигнала")

    args = parser.parse_args()

    # Определение частоты
    if args.freq:
        freq_mhz = args.freq
        sat_name = "Ручная частота"
        sat_info = {}
    else:
        sat = SATELLITES[args.sat]
        freq_mhz = sat['freq']
        sat_name = sat['name']
        sat_info = sat

    print("=" * 60)
    print(f"  Приём спутника NOAA — {sat_name}")
    print("=" * 60)
    
    if sat_info:
        print(f"\n  Спутник:    {sat_name}")
        print(f"  Частота:    {freq_mhz:.4f} МГц")
        print(f"  Орбита:     {sat_info.get('orbit', 'н/д')}")
        print(f"  Высота:     {sat_info.get('altitude', 'н/д')}")
        if 'note' in sat_info:
            print(f"  Примечание: {sat_info['note']}")
        print(f"\n  Модуляция:  WFM (широкополосная FM)")
        print(f"  Девиация:   ~34 кГц")
        print(f"  Полоса:     ~40 кГц")
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

    # Настройки
    freq_hz = int(freq_mhz * 1e6)
    rate_hz = int(args.rate * 1e6)
    
    print(f"\n--- Настройка ---")
    print(f"  Частота: {freq_mhz:.4f} МГц")
    print(f"  Дискретизация: {args.rate} MS/s")

    dll.rtlsdr_set_sample_rate(dev, rate_hz)
    dll.rtlsdr_set_center_freq(dev, freq_hz)

    if args.gain > 0:
        dll.rtlsdr_set_tuner_gain(dev, int(args.gain * 10))
        print(f"  Усиление: {args.gain} dB")
    else:
        dll.rtlsdr_set_tuner_gain_mode(dev, 0)
        print(f"  Усиление: Авто (AGC)")

    dll.rtlsdr_reset_buffer(dev)

    if args.scan:
        # Сканирование частот
        print(f"\n--- Сканирование ---")
        scan_range = 0.05  # ±50 кГц
        scan_step = 0.01   # 10 кГц
        scan_freqs = []
        
        f = freq_mhz - scan_range
        while f <= freq_mhz + scan_range:
            scan_freqs.append(f)
            f += scan_step
        
        best_freq = freq_mhz
        best_power = -100
        
        for i, sf in enumerate(scan_freqs):
            print(f"  Сканирование {i+1}/{len(scan_freqs)}: {sf:.4f} МГц", end='\r')
            dll.rtlsdr_set_center_freq(dev, int(sf * 1e6))
            time.sleep(0.1)
            
            buf = ctypes.create_string_buffer(4096)
            n_read = ctypes.c_uint32(0)
            dll.rtlsdr_read_sync(dev, buf, 4096, ctypes.byref(n_read))
            
            if n_read.value > 0:
                raw = buf.raw[:n_read.value]
                i_s, q_s = iq_to_complex(raw)
                power = compute_power_db(i_s, q_s)
                
                if power > best_power:
                    best_power = power
                    best_freq = sf
        
        print(f"\n\n  Лучшая частота: {best_freq:.4f} МГц @ {best_power:.1f} dB")
        freq_hz = int(best_freq * 1e6)
        dll.rtlsdr_set_center_freq(dev, freq_hz)
        dll.rtlsdr_reset_buffer(dev)

    # Приём
    print(f"\n--- Приём ---")
    buf_len = args.samples * 2
    buf = ctypes.create_string_buffer(buf_len)
    n_read = ctypes.c_uint32(0)

    print(f"  Получение {args.samples} сэмплов...")
    result = dll.rtlsdr_read_sync(dev, buf, buf_len, ctypes.byref(n_read))

    if result != 0 or n_read.value == 0:
        print(f"  ОШИБКА: Чтение не удалось ({result})")
        dll.rtlsdr_close(dev)
        return 1

    raw = buf.raw[:n_read.value]
    i_samples, q_samples = iq_to_complex(raw)
    power_db = compute_power_db(i_samples, q_samples)

    print(f"  [OK] Получено {n_read.value // 2} сэмплов")
    print(f"  Мощность: {power_db:.1f} dB")

    # Простая проверка сигнала
    if power_db > -30:
        print(f"  Сигнал: СИЛЬНЫЙ ✓")
    elif power_db > -50:
        print(f"  Сигнал: СРЕДНИЙ")
    else:
        print(f"  Сигнал: СЛАБЫЙ")

    # Сохранение
    if args.output:
        with open(args.output, "wb") as f:
            f.write(raw)
        print(f"\n[OK] Сохранено {len(raw)} байт в {args.output}")

    dll.rtlsdr_close(dev)
    
    print(f"\n{'='*60}")
    print(f"  Далее:")
    print(f"  1. Декодировать APT: sox -t raw -r 11025 -e unsigned-integer -b 8 -c 1 {args.output or 'noaa.wav'} noaa.wav")
    print(f"  2. Обработать в WXtoImg или aptdec")
    print(f"{'='*60}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
