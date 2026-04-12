"""
Приём AX.25 пакетов (APRS, CubeSat телеметрия)

Частоты:
  APRS VHF: 144.800 МГц (Россия/Европа)
  APRS HF:  144.390 МГц (США)
  ISS APRS: 145.825 МГц

Модуляция: AFSK 1200 бод
  Mark: 1200 Гц (логическая 1)
  Space: 2200 Гц (логический 0)

Использование:
  python scripts\\sdr_ax25.py                     # APRS 144.800
  python scripts\\sdr_ax25.py --freq 145.825      # ISS APRS
  python scripts\\sdr_ax25.py --freq 144.390      # США APRS
  python scripts\\sdr_ax25.py --decode            # Декодирование пакетов
  python scripts\\sdr_ax25.py --output aprs.bin   # Сохранить I/Q
"""

import ctypes
import sys
import os
import math
import argparse
import time
from collections import deque

DLL_PATHS = [
    r"C:\SDRSharp\tmp\x64\rtlsdr.dll",
    r"C:\SDRSharp\rtlsdr.dll",
    r"rtlsdr.dll",
]

# Частоты APRS
APRS_FREQS = {
    'eu': 144.800,    # Европа/Россия
    'us': 144.390,    # США
    'iss': 145.825,   # ISS
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


def goertzel(samples, freq, sample_rate):
    """Алгоритм Гёрцеля для детекции тона"""
    k = int(0.5 + (len(samples) * freq) / sample_rate)
    omega = 2.0 * math.pi * k / len(samples)
    coeff = 2.0 * math.cos(omega)
    
    s_prev2 = 0.0
    s_prev1 = 0.0
    s_curr = 0.0
    
    for sample in samples:
        s_curr = sample + coeff * s_prev1 - s_prev2
        s_prev2 = s_prev1
        s_prev1 = s_curr
    
    power = s_prev1**2 + s_prev2**2 - coeff * s_prev1 * s_prev2
    return power


def detect_afsk(i_samples, q_samples, sample_rate):
    """Детекция AFSK 1200/2200 Гц"""
    # Объединяем I и Q в амплитуду
    magnitudes = [math.sqrt(i**2 + q**2) for i, q in zip(i_samples, q_samples)]
    
    # Параметры для AFSK
    mark_freq = 1200   # Гц (логическая 1)
    space_freq = 2200  # Гц (логический 0)
    
    # Размер окна для Goertzel (~10 мс для 1200 бод)
    window_size = int(sample_rate * 0.01)
    
    bits = []
    
    for i in range(0, len(magnitudes) - window_size, window_size):
        window = magnitudes[i:i+window_size]
        
        # Детекция Mark и Space
        mark_power = goertzel(window, mark_freq, sample_rate)
        space_power = goertzel(window, space_freq, sample_rate)
        
        # Определяем бит
        if mark_power > space_power:
            bits.append(1)
        else:
            bits.append(0)
    
    return bits


def decode_ax25_frame(bits):
    """Попытка декодировать AX.25 фрейм из битов"""
    if len(bits) < 100:
        return None
    
    # AX.25 использует NRZI + бит-стаффинг
    # Ищем флаг 0x7E (01111110)
    flag = [0, 1, 1, 1, 1, 1, 1, 0]
    
    # Простой поиск флага
    flag_positions = []
    for i in range(len(bits) - 7):
        if bits[i:i+8] == flag:
            flag_positions.append(i)
    
    if len(flag_positions) < 2:
        return None
    
    # Между флагами должен быть AX.25 фрейм
    start = flag_positions[0]
    end = flag_positions[1]
    frame_bits = bits[start+8:end]
    
    # Преобразуем биты в байты (LSB first)
    frame_bytes = []
    for i in range(0, len(frame_bits) - 7, 8):
        byte_bits = frame_bits[i:i+8]
        byte_val = 0
        for j, bit in enumerate(byte_bits):
            byte_val |= (bit << j)
        frame_bytes.append(byte_val)
    
    return bytes(frame_bytes)


def parse_callsign(data, offset):
    """Парсинг позывного из AX.25"""
    if offset + 7 > len(data):
        return "", offset
    
    callsign_bytes = data[offset:offset+6]
    callsign = ""
    for b in callsign_bytes:
        c = chr(b >> 1)
        if c != ' ':
            callsign += c
    
    ssid = (data[offset+6] >> 1) & 0x0F
    callsign += f"-{ssid}"
    
    return callsign.strip(), offset + 7


def parse_ax25_header(data):
    """Парсинг AX.25 заголовка"""
    if len(data) < 15:
        return None
    
    offset = 0
    
    # Destination callsign
    dest, offset = parse_callsign(data, offset)
    
    # Source callsign
    source, offset = parse_callsign(data, offset)
    
    # Digipeaters (если есть)
    digipeaters = []
    addr_ext = data[6] & 0x01  # Address extension bit
    i = 0
    while not (data[offset-1] & 0x01) and i < 8:  # Max 8 digipeaters
        if offset + 7 > len(data):
            break
        digi, offset = parse_callsign(data, offset)
        if digi:
            digipeaters.append(digi)
        i += 1
    
    # Control field
    control = data[offset]
    offset += 1
    
    # PID
    pid = data[offset]
    offset += 1
    
    return {
        'destination': dest,
        'source': source,
        'digipeaters': digipeaters,
        'control': control,
        'pid': pid,
        'info_start': offset
    }


def print_packet_info(header, info_data):
    """Вывод информации о пакете"""
    print(f"\n  {'='*50}")
    print(f"  Пакет AX.25")
    print(f"  {'='*50}")
    print(f"  От: {header['source']}")
    print(f"  Кому: {header['destination']}")
    if header['digipeaters']:
        print(f"  Через: {' -> '.join(header['digipeaters'])}")
    print(f"  PID: 0x{header['pid']:02X}")
    
    # Показываем info данные
    try:
        info_str = info_data.decode('ascii', errors='replace')
        # Фильтруем печатаемые символы
        info_clean = ''.join(c for c in info_str if c.isprintable() or c in '\r\n')
        if info_clean:
            print(f"  Данные: {info_clean[:100]}")
    except:
        pass
    
    print(f"  {'='*50}")


def main():
    parser = argparse.ArgumentParser(description="Приём AX.25 пакетов (APRS)")
    parser.add_argument("--freq", type=float,
                        help="Частота МГц (по умолч. 144.800)")
    parser.add_argument("--region", type=str, default="eu",
                        choices=list(APRS_FREQS.keys()),
                        help="Регион (по умолч. eu)")
    parser.add_argument("--rate", type=float, default=2.4,
                        help="Частота дискретизации MS/s")
    parser.add_argument("--gain", type=float, default=0,
                        help="Усиление dB (0 = авто)")
    parser.add_argument("--samples", type=int, default=262144,
                        help="Количество семплов (по умолч. 262144)")
    parser.add_argument("--decode", action="store_true",
                        help="Декодирование пакетов")
    parser.add_argument("--output", type=str,
                        help="Сохранить I/Q в файл")
    parser.add_argument("--monitor", action="store_true",
                        help="Непрерывный мониторинг")

    args = parser.parse_args()

    # Определение частоты
    freq_mhz = args.freq if args.freq else APRS_FREQS[args.region]

    print("=" * 60)
    print(f"  Приём AX.25 пакетов (APRS)")
    print("=" * 60)
    print(f"\n  Частота: {freq_mhz:.3f} МГц")
    print(f"  Модуляция: AFSK 1200 бод")
    print(f"  Mark: 1200 Гц, Space: 2200 Гц")
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
    print(f"  Частота: {freq_mhz:.3f} МГц")
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

    if args.monitor:
        print(f"\n--- Мониторинг APRS ---")
        print(f"  Нажмите Ctrl+C для выхода\n")
        try:
            packet_count = 0
            while True:
                # Читаем блок сэмплов
                buf_len = 32768 * 2
                buf = ctypes.create_string_buffer(buf_len)
                n_read = ctypes.c_uint32(0)
                
                result = dll.rtlsdr_read_sync(dev, buf, buf_len, ctypes.byref(n_read))
                
                if result == 0 and n_read.value > 0:
                    raw = buf.raw[:n_read.value]
                    i_samples, q_samples = iq_to_complex(raw)
                    
                    # Детекция AFSK
                    if args.decode:
                        bits = detect_afsk(i_samples, q_samples, int(rate_hz))
                        
                        # Попытка декодировать AX.25
                        frame = decode_ax25_frame(bits)
                        if frame:
                            header = parse_ax25_header(frame)
                            if header and len(frame) > header['info_start']:
                                packet_count += 1
                                print_packet_info(header, frame[header['info_start']:'])
                                print(f"  [Пакетов обнаружено: {packet_count}]")
                    else:
                        # Просто мониторинг мощности
                        power_db = compute_power_db(i_samples, q_samples)
                        print(f"  [{time.strftime('%H:%M:%S')}] Мощность: {power_db:.1f} dB", end='\r')
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print(f"\n\n[OK] Мониторинг остановлен")
            print(f"  Всего пакетов: {packet_count if 'packet_count' in locals() else 0}")
    else:
        # Однократный приём
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

        # Декодирование
        if args.decode:
            print(f"\n--- Декодирование AX.25 ---")
            bits = detect_afsk(i_samples, q_samples, int(rate_hz))
            print(f"  Бит обнаружено: {len(bits)}")
            
            # Статистика битов
            ones = sum(bits)
            zeros = len(bits) - ones
            print(f"  Единицы: {ones}, Нули: {zeros}")
            print(f"  Соотношение: {ones/len(bits)*100:.1f}% / {zeros/len(bits)*100:.1f}%")
            
            # Попытка декодировать фреймы
            frame = decode_ax25_frame(bits)
            if frame:
                header = parse_ax25_header(frame)
                if header:
                    print_packet_info(header, frame[header['info_start']:])
                else:
                    print(f"  Найдены флаги, но заголовок не распознан")
            else:
                print(f"  Флаги AX.25 не обнаружены")
                print(f"  Советы:")
                print(f"    - Увеличьте усиление (--gain 30-40)")
                print(f"    - Проверьте антенну")
                print(f"    - Убедитесь, что есть передача в эфире")

        # Сохранение
        if args.output:
            with open(args.output, "wb") as f:
                f.write(raw)
            print(f"\n[OK] Сохранено {len(raw)} байт в {args.output}")

    dll.rtlsdr_close(dev)
    
    print(f"\n{'='*60}")
    print(f"  Далее:")
    print(f"  - Используйте Dire Wolf для декодирования APRS")
    print(f"  - Команда: direwolf -r 11025 -b 8 -c sdr_ax25.conf")
    print(f"{'='*60}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
