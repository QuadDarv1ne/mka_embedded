"""
RTL-SDR V4 — получение и обработка I/Q семплов

Примеры использования:
  python scripts\\sdr_capture.py                    # I/Q семплы, спектр
  python scripts\\sdr_capture.py --freq 100.0       # FM 100 МГц
  python scripts\\sdr_capture.py --freq 433.92      # LPD диапазон
  python scripts\\sdr_capture.py --freq 144.8       # APRS
  python scripts\\sdr_capture.py --freq 145.8       # ISS
  python scripts\\sdr_capture.py --freq 446.0       # PMR
  python scripts\\sdr_capture.py --samples 8192     # больше семплов
  python scripts\\sdr_capture.py --gain 40          # усилить сигнал
  python scripts\\sdr_capture.py --plot             # ASCII спектр
"""

import ctypes
import sys
import os
import math
import argparse
import struct

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


def fft_size_power2(n):
    """Ближайший размер FFT степени 2"""
    return 1 << (n - 1).bit_length()


def simple_dft_mag(i_samples, q_samples, num_bins):
    """Простой DFT для спектрального анализа (медленно но без numpy)"""
    n = min(len(i_samples), len(q_samples))
    if n == 0:
        return []
    
    # Берём подмножество для скорости
    step = max(1, n // 1024)
    i_sub = i_samples[::step][:1024]
    q_sub = q_samples[::step][:1024]
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


def ascii_spectrum(mags, freq_start_mhz, sample_rate_mhz, width=80, height=20):
    """ASCII спектральная диаграмма"""
    if not mags:
        return ""
    
    # Нормализация
    min_mag = min(mags)
    max_mag = max(mags)
    mag_range = max_mag - min_mag if max_mag > min_mag else 1
    
    lines = []
    # Заголовок
    lines.append(f"  Frequency: {freq_start_mhz:.3f} - {freq_start_mhz + sample_rate_mhz:.3f} MHz")
    lines.append(f"  Power: {min_mag:.1f} to {max_mag:.1f} dB")
    lines.append("")
    
    # Рисуем по строкам сверху вниз
    for row in range(height, 0, -1):
        threshold = min_mag + (mag_range * row / height)
        line = f"{threshold:+7.1f} |"
        for mag in mags:
            if mag >= threshold:
                line += "█"
            elif mag >= threshold - mag_range / height:
                line += "▓"
            elif mag >= threshold - 2 * mag_range / height:
                line += "▒"
            else:
                line += " "
        lines.append(line)
    
    # Ось частот
    lines.append("         +" + "-" * len(mags) + "+")
    
    # Метки частот
    freq_label = f" {freq_start_mhz:.2f}"
    freq_label_end = f" {freq_start_mhz + sample_rate_mhz:.2f} MHz"
    lines.append("         |" + " " * (len(mags) - len(freq_label) - len(freq_label_end)) + freq_label_end)
    lines.append("         " + freq_label)
    
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="RTL-SDR V4 I/Q Capture")
    parser.add_argument("--freq", type=float, default=100.0,
                        help="Центральная частота МГц (по умолч. 100.0)")
    parser.add_argument("--rate", type=float, default=2.4,
                        help="Частота дискретизации MS/s (по умолч. 2.4)")
    parser.add_argument("--gain", type=float, default=0,
                        help="Усиление dB (0 = авто)")
    parser.add_argument("--samples", type=int, default=4096,
                        help="Количество семплов (по умолч. 4096)")
    parser.add_argument("--device", type=int, default=0,
                        help="Номер устройства (по умолч. 0)")
    parser.add_argument("--plot", action="store_true",
                        help="Показать ASCII спектр")
    parser.add_argument("--output", type=str, default="",
                        help="Сохранить I/Q в файл (.bin)")
    
    args = parser.parse_args()
    
    freq_hz = int(args.freq * 1e6)
    rate_hz = int(args.rate * 1e6)
    
    print("=" * 60)
    print("  RTL-SDR V4 — I/Q Capture")
    print("=" * 60)
    print()
    
    # Загрузка DLL
    dll = load_rtlsdr()
    if not dll:
        return 1
    
    # Проверка устройств
    count = dll.rtlsdr_get_device_count()
    print(f"\n[OK] Devices found: {count}")
    if count == 0:
        print("\nNo RTL-SDR devices!")
        print("Check: device connected, WinUSB driver installed (Zadig)")
        return 1
    
    # Инфо об устройстве
    print(f"\n--- Device {args.device} ---")
    name = ctypes.create_string_buffer(256)
    manufacturer = ctypes.create_string_buffer(256)
    serial = ctypes.create_string_buffer(256)
    dll.rtlsdr_get_device_usb_strings(args.device, manufacturer, serial, name)
    print(f"  Name: {name.value.decode('utf-8', errors='replace')}")
    print(f"  Manufacturer: {manufacturer.value.decode('utf-8', errors='replace')}")
    print(f"  Serial: {serial.value.decode('utf-8', errors='replace')}")
    
    # Открытие устройства
    dev = ctypes.c_void_p()
    result = dll.rtlsdr_open(ctypes.byref(dev), args.device)
    if result != 0 or not dev:
        print(f"  ERROR: Open failed (error {result})")
        return 1
    print(f"  [OK] Device opened")
    
    # Настройка тюнера
    print(f"\n--- Configuring ---")
    print(f"  Center freq: {args.freq} MHz ({freq_hz} Hz)")
    print(f"  Sample rate: {args.rate} MS/s ({rate_hz} Hz)")
    print(f"  Gain: {'Auto' if args.gain == 0 else f'{args.gain} dB'}")
    
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
            closest = min(available, key=lambda g: abs(g - int(args.gain * 10)))
            dll.rtlsdr_set_tuner_gain(dev, closest)
            print(f"  Gain set: {closest / 10} dB (from {available})")
        else:
            dll.rtlsdr_set_tuner_gain(dev, int(args.gain * 10))
            print(f"  Gain set: {args.gain} dB")
    else:
        dll.rtlsdr_set_tuner_gain_mode(dev, 0)  # AGC
        print(f"  Gain: Automatic (AGC)")
    
    # Reset buffer
    dll.rtlsdr_reset_buffer(dev)
    
    # Получение семплов
    print(f"\n--- Capturing {args.samples} samples ---")
    
    buf_len = args.samples * 2  # 2 байта на комплексный семпл
    buf = ctypes.create_string_buffer(buf_len)
    n_read = ctypes.c_uint32(0)
    
    result = dll.rtlsdr_read_sync(dev, buf, buf_len, ctypes.byref(n_read))
    
    if result != 0:
        print(f"  ERROR: Read failed (error {result})")
        # Попробовать async режим
        print("  Trying async read...")
        
        # Async callback
        callback_data = {"samples": b"", "done": False, "error": None}
        
        @ctypes.CFUNCTYPE(None, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_uint32, ctypes.c_void_p)
        def rtlsdr_callback(samples, sample_len, ctx):
            callback_data["samples"] += bytes(samples[:sample_len])
            if len(callback_data["samples"]) >= buf_len:
                callback_data["done"] = True
        
        result = dll.rtlsdr_read_async(dev, rtlsdr_callback, None, 0, 16)
        import time
        timeout = 5.0
        start = time.time()
        while not callback_data["done"] and (time.time() - start) < timeout:
            time.sleep(0.1)
        
        if callback_data["samples"]:
            n_read.value = min(len(callback_data["samples"]), buf_len)
            buf.raw = callback_data["samples"][:buf_len]
            print(f"  [OK] Async read: {n_read.value} bytes")
        else:
            print(f"  ERROR: Async read failed ({result})")
            dll.rtlsdr_close(dev)
            return 1
    else:
        print(f"  [OK] Received {n_read.value} bytes ({n_read.value // 2} complex samples)")
    
    # Обработка I/Q
    raw = buf.raw[:n_read.value]
    i_samples, q_samples = iq_to_complex(raw)
    power = compute_power(i_samples, q_samples)
    power_db = compute_db(power)
    
    # Статистика
    avg_power = sum(power) / len(power)
    avg_db = 10 * math.log10(avg_power + 1e-12)
    peak_db = max(power_db)
    min_db = min(power_db)
    
    print(f"\n--- Signal Statistics ---")
    print(f"  Average power: {avg_db:.1f} dB")
    print(f"  Peak power:    {peak_db:.1f} dB")
    print(f"  Min power:     {min_db:.1f} dB")
    print(f"  Dynamic range: {peak_db - min_db:.1f} dB")
    print(f"  DC offset I:   {sum(i_samples)/len(i_samples):.4f}")
    print(f"  DC offset Q:   {sum(q_samples)/len(q_samples):.4f}")
    print(f"  RMS I:         {math.sqrt(sum(i*i for i in i_samples)/len(i_samples)):.4f}")
    print(f"  RMS Q:         {math.sqrt(sum(q*q for q in q_samples)/len(q_samples)):.4f}")
    
    # Первые семплы
    print(f"\n--- First 16 I/Q samples ---")
    for idx in range(min(16, len(i_samples))):
        phase = math.atan2(q_samples[idx], i_samples[idx]) * 180 / math.pi
        amp = math.sqrt(i_samples[idx]**2 + q_samples[idx]**2)
        print(f"  [{idx:3d}] I={i_samples[idx]:+7.4f}  Q={q_samples[idx]:+7.4f}  |A|={amp:.4f}  ∠={phase:+7.1f}°")
    
    # ASCII спектр
    if args.plot:
        print(f"\n--- Spectrum ---")
        num_bins = min(64, n_read.value // 4)
        mags = simple_dft_mag(i_samples, q_samples, num_bins)
        if mags:
            spectrum = ascii_spectrum(mags, args.freq, args.rate)
            print(spectrum)
        else:
            print("  (not enough samples for FFT)")
    
    # Сохранение
    if args.output:
        with open(args.output, "wb") as f:
            f.write(raw)
        print(f"\n[OK] Saved {len(raw)} bytes to {args.output}")
    
    # Закрытие
    dll.rtlsdr_close(dev)
    print(f"\n[OK] Device closed")
    print("\n" + "=" * 60)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
