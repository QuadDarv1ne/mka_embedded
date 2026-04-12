"""
Калькулятор антенн для SDR

Расчёт параметров различных антенн для приёма сигналов.

Использование:
  python scripts\\sdr_antenna.py --freq 145.8             # Четвертьволновая антенна
  python scripts\\sdr_antenna.py --freq 137.1 --type dipole  # Диполь
  python scripts\\sdr_antenna.py --freq 436.7 --type gp   # Ground Plane
"""

import argparse
import math


def calculate_quarter_wave(freq_mhz):
    """Расчёт четвертьволновой антенны"""
    # Длина волны
    wavelength = 300 / freq_mhz  # в метрах
    
    # Четверть волны (с коэффициентом укорочения 0.95)
    length = (wavelength / 4) * 0.95
    
    return {
        'type': 'Четвертьволновая (1/4 λ)',
        'wavelength': wavelength,
        'length': length,
        'impedance': '~36 Ω (без заземления)',
        'gain': '~2.15 dBi'
    }


def calculate_dipole(freq_mhz):
    """Расчёт дипольной антенны"""
    # Длина волны
    wavelength = 300 / freq_mhz
    
    # Полуволновой диполь (с коэффициентом 0.95)
    length = (wavelength / 2) * 0.95
    arm_length = length / 2
    
    return {
        'type': 'Полуволновой диполь (1/2 λ)',
        'wavelength': wavelength,
        'length': length,
        'arm_length': arm_length,
        'impedance': '~73 Ω (сбалансированная)',
        'gain': '~2.15 dBi'
    }


def calculate_ground_plane(freq_mhz):
    """Расчёт Ground Plane антенны"""
    # Вертикальный элемент (1/4 волны)
    vertical = (300 / freq_mhz / 4) * 0.95
    
    # Радиалы (1/4 волны, наклонённые на 45°)
    radials = (300 / freq_mhz / 4) * 0.95 * 1.05  # 5% длиннее
    
    return {
        'type': 'Ground Plane (GP)',
        'wavelength': 300 / freq_mhz,
        'vertical_length': vertical,
        'radial_length': radials,
        'num_radials': '4 (рекомендуется)',
        'impedance': '~50 Ω (с наклонёнными радиалами)',
        'gain': '~2.15 dBi'
    }


def calculate_jpole(freq_mhz):
    """Расчёт J-Pole антенны"""
    wavelength = 300 / freq_mhz
    
    # Элементы (с коэффициентом 0.95)
    half_wave = (wavelength / 2) * 0.95
    quarter_wave = (wavelength / 4) * 0.95
    stub = quarter_wave * 0.15  # 15% от четверти волны
    
    return {
        'type': 'J-Pole',
        'wavelength': wavelength,
        'half_wave': half_wave,
        'quarter_wave': quarter_wave,
        'stub_length': stub,
        'impedance': '~50 Ω',
        'gain': '~2.15 dBi'
    }


def print_antenna_info(info, freq_mhz):
    """Вывод информации об антенне"""
    print("=" * 60)
    print(f"  Расчёт антенны для {freq_mhz:.3f} МГц")
    print("=" * 60)
    print()
    print(f"  Тип:            {info['type']}")
    print(f"  Длина волны:    {info['wavelength']:.3f} м")
    
    if 'length' in info:
        print(f"  Общая длина:  {info['length']:.3f} м ({info['length']*100:.1f} см)")
    
    if 'arm_length' in info:
        print(f"  Плечо диполя: {info['arm_length']:.3f} м ({info['arm_length']*100:.1f} см)")
    
    if 'vertical_length' in info:
        print(f"  Вертикаль:    {info['vertical_length']:.3f} м ({info['vertical_length']*100:.1f} см)")
        print(f"  Длина радил:  {info['radial_length']:.3f} м ({info['radial_length']*100:.1f} см)")
        print(f"  Количество:   {info['num_radials']}")
    
    if 'half_wave' in info:
        print(f"  Полуволна:    {info['half_wave']:.3f} м ({info['half_wave']*100:.1f} см)")
        print(f"  Четвертьволна:{info['quarter_wave']:.3f} м ({info['quarter_wave']*100:.1f} см)")
        print(f"  Stub:         {info['stub_length']:.3f} м ({info['stub_length']*100:.1f} см)")
    
    print(f"  Импеданс:     {info['impedance']}")
    print(f"  Усиление:     {info['gain']}")
    print()


def print_recommendations(freq_mhz):
    """Вывод рекомендаций по частоте"""
    print("=" * 60)
    print(f"  Рекомендации для {freq_mhz:.3f} МГц")
    print("=" * 60)
    print()
    
    if 88 <= freq_mhz <= 108:
        print("  FM радиовещание")
        print("  ✅ Четвертьволновая (75 см)")
        print("  ✅ Диполь (1.5 м)")
        print("  ✅ T-антенна")
        print()
        print("  Советы:")
        print("  - Простая четвертьволновая отлично работает")
        print("  - Разместите вертикально")
        print("  - Подключите к заземлению или(counterpoise)")
        
    elif 137 <= freq_mhz <= 138:
        print("  Метеоспутники NOAA")
        print("  ✅ Квадратурная антенна (QFH)")
        print("  ✅ Диполь с cross-wire")
        print("  ✅ Turnstile")
        print()
        print("  Советы:")
        print("  - QFH даёт круговую поляризацию (лучше для спутников)")
        print("  - Направьте антенну вертикально")
        print("  - Используйте предусилитель если нужно")
        
    elif 144 <= freq_mhz <= 146:
        print("  2m Amateur / APRS / ISS")
        print("  ✅ Четвертьволновая (50 см)")
        print("  ✅ Yagi (направленная на ISS)")
        print("  ✅ Ground Plane")
        print()
        print("  Советы:")
        print("  - Для ISS: направленная антенна (Yagi)")
        print("  - Для APRS: вертикальная всенаправленная")
        print("  - Track ISS через heavens-above.com")
        
    elif 156 <= freq_mhz <= 160:
        print("  Морская связь VHF")
        print("  ✅ Морская VHF антенна")
        print("  ✅ Ground Plane")
        print("  ✅ Whip antenna")
        print()
        print("  Советы:")
        print("  - Чем выше, тем лучше (горизонт)")
        print("  - Используйте морской канал 16 для emergencies")
        
    elif 430 <= freq_mhz <= 440:
        print("  70cm Amateur / UHF")
        print("  ✅ Ground Plane (17 см)")
        print("  ✅ Yagi")
        print("  ✅ Quad")
        print()
        print("  Советы:")
        print("  - Компактные размеры (17 см для 1/4 волны)")
        print("  - Хорошо для локальных QSO и репитеров")
        print("  - ISS SSTV на 436.7 МГц")
        
    else:
        print("  Произвольная частота")
        print("  ✅ Четвертьволновая")
        print("  ✅ Диполь")
        print("  ✅ Ground Plane")
        print()
        print("  Советы:")
        print("  - Проверьтеlegality прослушивания этой частоты")
        print("  - Используйте сканер для поиска сигналов")
    
    print()


def main():
    parser = argparse.ArgumentParser(description="Калькулятор антенн для SDR")
    parser.add_argument("--freq", type=float, required=True,
                        help="Частота МГц")
    parser.add_argument("--type", type=str, default="all",
                        choices=['quarter', 'dipole', 'gp', 'jpole', 'all'],
                        help="Тип антенны (по умолч. all)")

    args = parser.parse_args()

    print()
    print("=" * 60)
    print("  Калькулятор антенн RTL-SDR")
    print("=" * 60)
    print()

    if args.type in ['quarter', 'all']:
        info = calculate_quarter_wave(args.freq)
        print_antenna_info(info, args.freq)

    if args.type in ['dipole', 'all']:
        info = calculate_dipole(args.freq)
        print_antenna_info(info, args.freq)

    if args.type in ['gp', 'all']:
        info = calculate_ground_plane(args.freq)
        print_antenna_info(info, args.freq)

    if args.type in ['jpole', 'all']:
        info = calculate_jpole(args.freq)
        print_antenna_info(info, args.freq)

    # Рекомендации
    print_recommendations(args.freq)

    print("=" * 60)
    print(f"  Формулы:")
    print(f"  Длина волны (λ) = 300 / f(МГц)")
    print(f"  1/4 волны = λ / 4 * 0.95 (коэффициент укорочения)")
    print(f"  1/2 волны = λ / 2 * 0.95")
    print("=" * 60)

    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
