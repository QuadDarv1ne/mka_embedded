/**
 * @file sgp4.cpp
 * @brief Реализация SGP4 орбитального пропагатора
 */

#include "sgp4.hpp"
#include <cstring>
#include <algorithm>

namespace mka {
namespace navigation {

using namespace Constants;

// ============================================================================
// SGP4Propagator реализация
// ============================================================================

bool SGP4Propagator::init(const TLE& tle) {
    if (!parseTLE(tle)) {
        return false;
    }

    initPerturbations();
    initialized_ = true;
    return true;
}

ECIState SGP4Propagator::propagate(double minutesSinceEpoch) const {
    ECIState result;
    result.timeSinceEpoch = minutesSinceEpoch;

    if (!initialized_) {
        return result;
    }

    // Упрощённое кеплерово распространение (без полных возмущений SGP4)
    // Для полной реализации требуется ~1000 строк кода

    // Среднее движение с учётом возмущений
    double n = xnodp;

    // Средняя аномалия на момент времени
    double M = meanAnomaly_ + (n - omgdot) * minutesSinceEpoch;
    M = utils::normalizeAngle(M);

    // Решение уравнения Кеплера (итерации Ньютона)
    double E = M;
    for (int i = 0; i < 10; i++) {
        double dE = (E - eccentricity_ * std::sin(E) - M) /
                    (1.0 - eccentricity_ * std::cos(E));
        E -= dE;
        if (std::abs(dE) < 1e-10) break;
    }

    // Истинная аномалия
    double cosE = std::cos(E);
    double sinE = std::sin(E);
    double cosf = (cosE - eccentricity_) / (1.0 - eccentricity_ * cosE);
    double sinf = std::sqrt(std::max(0.0, 1.0 - eccentricity_ * eccentricity_)) * sinE /
                  (1.0 - eccentricity_ * cosE);
    double f = std::atan2(sinf, cosf);

    // Расстояние от центра Земли
    double r = semiMajorAxis_ * (1.0 - eccentricity_ * cosE);

    // Положение в орбитальной плоскости
    double u = argPerigee_ + f;  // Аргумент широты

    // Преобразование в ECI координаты
    double cosu = std::cos(u);
    double sinu = std::sin(u);
    double cosi = std::cos(inclination_);
    double sini = std::sin(inclination_);
    double cosOmega = std::cos(raan_);
    double sinOmega = std::sin(raan_);

    // Положение
    result.x = r * (cosOmega * cosu - sinOmega * sinu * cosi);
    result.y = r * (sinOmega * cosu + cosOmega * sinu * cosi);
    result.z = r * sinu * sini;

    // Скорость (правильный расчёт для эллиптической орбиты)
    double p = semiMajorAxis_ * (1.0 - eccentricity_ * eccentricity_);  // Параметр орбиты
    double sqrtMuP = std::sqrt(MU_EARTH / p);
    double cosF = std::cos(f);
    double sinF = std::sin(f);

    // Радиальная и трансверсальная компоненты скорости
    double vr = sqrtMuP * eccentricity_ * sinF;
    double vtheta = sqrtMuP * (1.0 + eccentricity_ * cosF);

    // Преобразование в ECI
    result.vx = vr * (cosOmega * cosu - sinOmega * sinu * cosi) -
                vtheta * (cosOmega * sinu + sinOmega * cosu * cosi);
    result.vy = vr * (sinOmega * cosu + cosOmega * sinu * cosi) +
                vtheta * (sinOmega * sinu - cosOmega * cosu * cosi);
    result.vz = vr * sinu * sini + vtheta * cosu * sini;

    return result;
}

ECIState SGP4Propagator::propagateToUTC(int year, double dayOfYear, double secondsOfDay) const {
    // Вычисление времени от эпохи (эпоха хранится как день года)
    double currentDayOfYear = dayOfYear + secondsOfDay / 86400.0;
    double daysSinceEpoch = currentDayOfYear - epochDay;

    // Добавляем полные года
    daysSinceEpoch += (year - epochYear) * 365.25;

    double minutesSinceEpoch = daysSinceEpoch * 1440.0;

    return propagate(minutesSinceEpoch);
}

bool SGP4Propagator::parseTLE(const TLE& tle) {
    try {
        // Парсинг строки 1
        if (tle.line1.length() < 69) return false;

        int catalogNumber = std::stoi(tle.line1.substr(2, 5));
        char classification = tle.line1[7];
        (void)catalogNumber;
        (void)classification;

        int yy = std::stoi(tle.line1.substr(18, 2));
        epochYear = (yy < 57) ? 2000 + yy : 1900 + yy;
        epochDay = std::stod(tle.line1.substr(20, 12));

        double meanMotionDot = std::stod(tle.line1.substr(33, 10));
        (void)meanMotionDot;
        
        // Экспоненциальный формат для второй производной (Cols 45-52)
        // Формат: S.DDDDD-x (без точки в строке TLE)
        // Пример: " 00000-0" -> 0.00000 * 10^0
        std::string mddStr = tle.line1.substr(44, 7); 
        if (mddStr.length() >= 7) {
            std::string numPart = "0." + mddStr.substr(0, 5);
            char expSign = mddStr[5];
            int expVal = std::stoi(mddStr.substr(6, 1));
            double meanMotionDotDot = std::stod(numPart);
            meanMotionDotDot *= std::pow(10.0, (expSign == '-') ? -expVal : expVal);
            (void)meanMotionDotDot;
        }

        // B-star (Cols 54-61)
        // Формат: DDDDD-S (десятичная точка после первой цифры подразумевается)
        // Пример: "10270-3" -> 0.10270 * 10^-3
        std::string bstarStr = tle.line1.substr(53, 7);
        if (bstarStr.length() >= 7) {
            std::string numPart = "0." + bstarStr.substr(0, 5);
            char expSign = bstarStr[5];
            int expVal = std::stoi(bstarStr.substr(6, 1));
            double bstar = std::stod(numPart);
            bstar *= std::pow(10.0, (expSign == '-') ? -expVal : expVal);
            bstar_ = bstar;
        }

        int elementSetNumber = std::stoi(tle.line1.substr(64, 4));
        (void)elementSetNumber;

        // Парсинг строки 2
        if (tle.line2.length() < 69) return false;

        inclination_ = utils::degToRad(std::stod(tle.line2.substr(8, 8)));
        raan_ = utils::degToRad(std::stod(tle.line2.substr(17, 8)));

        // Эксцентриситет (без десятичной точки в TLE)
        std::string eccStr = tle.line2.substr(26, 7);
        eccentricity_ = std::stod("0." + eccStr);

        argPerigee_ = utils::degToRad(std::stod(tle.line2.substr(34, 8)));
        meanAnomaly_ = utils::degToRad(std::stod(tle.line2.substr(43, 8)));
        double meanMotionRevPerDay = std::stod(tle.line2.substr(52, 11));
        int orbitNumber = std::stoi(tle.line2.substr(63, 5));
        (void)orbitNumber;

        // Валидация эксцентриситета (0 <= e < 1 для эллиптических орбит)
        if (eccentricity_ < 0.0 || eccentricity_ >= 1.0) {
            return false;  // Невозможная орбита
        }

        // Вычисление большой полуоси
        meanMotionRevPerDay_ = meanMotionRevPerDay;  // Сохраняем для getOrbitalPeriod
        meanMotion_ = meanMotionRevPerDay * TWOPI / 1440.0;  // rev/day -> rad/min

        // Проверка на положительную большую полуось
        if (meanMotion_ <= 0.0) {
            return false;
        }

        semiMajorAxis_ = std::pow(MU_EARTH / (meanMotion_ * meanMotion_), TWOTHIRD);

        epoch_ = epochDay;

        return true;
    } catch (const std::exception& e) {
        // Ловим все исключения при парсинге (stod, stoi, substr могут выбрасывать)
        return false;
    }
}

void SGP4Propagator::initPerturbations() {
    // Инициализация коэффициентов возмущений
    // Упрощённая версия для околоземных орбит

    double cosI = std::cos(inclination_);
    double sinI = std::sin(inclination_);

    // J2 возмущения
    double j2Factor = CK2 / (semiMajorAxis_ * semiMajorAxis_);

    // Среднее движение с учётом J2
    xnodp = meanMotion_ * (1.0 + 1.5 * j2Factor * (1.5 * sinI * sinI - 1.0) /
                           std::sqrt(1.0 - eccentricity_ * eccentricity_));

    // Скорость изменения RAAN (прецессия узловой линии)
    xnodot = -1.5 * xnodp * j2Factor * cosI / (1.0 - eccentricity_ * eccentricity_);

    // Скорость изменения аргумента перигея
    omgdot = 0.75 * xnodp * j2Factor * (5.0 * cosI * cosI - 1.0) /
             (1.0 - eccentricity_ * eccentricity_);

    // Коэффициенты для возмущений
    t2cof = 1.5 * j2Factor;
    xlcof = 0.125 * j2Factor * (3.0 + 5.0 * cosI) / (1.0 + eccentricity_);
    aycof = 0.25 * j2Factor * (3.0 + 5.0 * cosI);
    x7thm1 = 7.0 * cosI * cosI - 1.0;
    x1mth2 = 1.0 - cosI * cosI;

    t3cof = t2cof * t2cof;
    t4cof = t2cof * t3cof;
    t5cof = t2cof * t4cof;

    xnodcf = 3.5 * j2Factor * xnodot;
}

// ============================================================================
// Утилиты реализация
// ============================================================================

namespace utils {

LLACoords eciToLLA(const ECIState& eci, double jdUT) {
    LLACoords result;

    // Радиус и высота
    double r = eci.radius();
    result.altitude = r - Constants::R_EARTH;

    // Широта
    result.latitude = std::asin(eci.z / r);

    // Долгота (с учётом вращения Земли)
    // Зелёный_sidereal_time на 0 часов UTC
    double gmst = std::fmod(280.46061837 + 360.98564736629 * (jdUT - 2451545.0), 360.0);
    if (gmst < 0) gmst += 360.0;

    // Часовой угол
    double theta = std::atan2(eci.y, eci.x);

    // Долгота
    result.longitude = theta - utils::degToRad(gmst);

    // Нормализация
    result.latitude = utils::radToDeg(result.latitude);
    result.longitude = utils::radToDeg(result.longitude);

    // Нормализация долготы к [-180, 180]
    while (result.longitude > 180.0) result.longitude -= 360.0;
    while (result.longitude < -180.0) result.longitude += 360.0;

    return result;
}

double julianDate(int year, int month, int day, double hour) {
    if (month <= 2) {
        year -= 1;
        month += 12;
    }

    int A = year / 100;
    int B = 2 - A + A / 4;

    return std::floor(365.25 * (year + 4716)) +
           std::floor(30.6001 * (month + 1)) +
           day + B - 1524.5 + hour / 24.0;
}

double normalizeAngle(double angle) {
    angle = std::fmod(angle, TWOPI);
    if (angle < 0) angle += TWOPI;
    return angle;
}

} // namespace utils

} // namespace navigation
} // namespace mka
