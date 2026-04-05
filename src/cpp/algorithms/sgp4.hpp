/**
 * @file sgp4.hpp
 * @brief SGP4 орбитальный пропагатор
 *
 * Упрощённая реализация SGP4 (Simplified General Perturbations 4)
 * для прогноза положения спутника по TLE данным.
 *
 * Основано на:
 * - Spacetrack Report #3
 * - NASA Goddard Space Flight Center
 */

#ifndef SGP4_HPP
#define SGP4_HPP

#include <cstdint>
#include <cmath>
#include <array>
#include <string>

namespace mka {
namespace navigation {

// ============================================================================
// Константы
// ============================================================================

namespace Constants {
    constexpr double PI = 3.14159265358979323846;
    constexpr double TWOPI = 2.0 * PI;
    constexpr double TWOTHIRD = 2.0 / 3.0;

    // Физические константы Земли
    constexpr double R_EARTH = 6378.135;        // Экваториальный радиус [km]
    constexpr double MU_EARTH = 398600.4418;    // Гравитационный параметр [km³/s²]
    constexpr double J2 = 1.08261579e-3;        // Второй зональный коэффициент
    constexpr double J3 = -2.53881e-6;          // Третий зональный коэффициент
    constexpr double J4 = -1.65597e-6;          // Четвёртый зональный коэффициент

    // Производные константы
    constexpr double XKE = std::sqrt(MU_EARTH / (R_EARTH * R_EARTH * R_EARTH));
    constexpr double CK2 = 0.5 * J2 * R_EARTH * R_EARTH;
    constexpr double CK4 = -0.375 * J4 * R_EARTH * R_EARTH * R_EARTH * R_EARTH;
}

// ============================================================================
// Типы данных
// ============================================================================

/// TLE элементы (Two-Line Element set)
struct TLE {
    std::string name;         // Имя спутника
    std::string line1;        // Первая строка TLE
    std::string line2;        // Вторая строка TLE

    // Распарсенные данные
    int catalogNumber = 0;          // Номер по каталогу
    char classification = 'U';      // Классификация (U=unclassified)
    int epochYear = 0;              // Год эпохи
    double epochDay = 0.0;          // День эпохи
    double meanMotionDot = 0.0;     // Первая производная среднего движения
    double meanMotionDotDot = 0.0;  // Вторая производная
    double bstar = 0.0;             // Коэффициент баллистического сопротивления
    int elementSetNumber = 0;       // Номер набора элементов
    double inclination = 0.0;       // Наклонение [deg]
    double raan = 0.0;              // Прямое восхождение восходящего узла [deg]
    double eccentricity = 0.0;      // Эксцентриситет
    double argPerigee = 0.0;        // Аргумент перигея [deg]
    double meanAnomaly = 0.0;       // Средняя аномалия [deg]
    double meanMotion = 0.0;        // Среднее движение [rev/day]
    int orbitNumber = 0;            // Номер витка на эпоху
};

/// Положение и скорость спутника
struct ECIState {
    // Положение в ECI (Earth-Centered Inertial) координатах [km]
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    // Скорость в ECI координатах [km/s]
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;

    // Время вычисления [минуты от эпохи TLE]
    double timeSinceEpoch = 0.0;

    /// Получить радиус-вектор [km]
    double radius() const {
        return std::sqrt(x*x + y*y + z*z);
    }

    /// Получить высоту над поверхностью Земли [km]
    double altitude() const {
        return radius() - Constants::R_EARTH;
    }

    /// Получить скорость [km/s]
    double velocity() const {
        return std::sqrt(vx*vx + vy*vy + vz*vz);
    }
};

/// Географические координаты (LLA)
struct LLACoords {
    double latitude = 0.0;   // Широта [deg]
    double longitude = 0.0;  // Долгота [deg]
    double altitude = 0.0;   // Высота [km]
};

// ============================================================================
// SGP4 Пропагатор
// ============================================================================

/**
 * @brief SGP4 орбитальный пропагатор
 *
 * Реализует упрощённый алгоритм SGP4 для распространения TLE элементов.
 * Подходит для околоземных орбит (период < 225 минут).
 */
class SGP4Propagator {
public:
    SGP4Propagator() = default;

    /**
     * @brief Инициализация по TLE данным
     * @param tle TLE элементы
     * @return true если успешно
     */
    bool init(const TLE& tle);

    /**
     * @brief Вычисление положения на заданное время
     * @param minutesSinceEpoch Минуты от эпохи TLE
     * @return Положение и скорость в ECI
     */
    ECIState propagate(double minutesSinceEpoch) const;

    /**
     * @brief Вычисление положения на заданную дату
     * @param year Год
     * @param dayOfYear День года
     * @param secondsOfDay Секунды от начала дня
     * @return Положение и скорость в ECI
     */
    ECIState propagateToUTC(int year, double dayOfYear, double secondsOfDay) const;

    /**
     * @brief Получить текущую эпоху TLE
     */
    double getEpoch() const { return epoch_; }

    /**
     * @brief Проверка валидности TLE
     */
    bool isValid() const { return initialized_; }

    /**
     * @brief Получить период орбиты [минуты]
     */
    double getOrbitalPeriod() const { return meanMotionRevPerDay_ > 0.0 ? 1440.0 / meanMotionRevPerDay_ : 0.0; }

    /**
     * @brief Получить среднее движение [rev/day]
     */
    double getMeanMotion() const { return meanMotion_; }

    /**
     * @brief Получить эксцентриситет
     */
    double getEccentricity() const { return eccentricity_; }

    /**
     * @brief Получить наклонение [deg]
     */
    double getInclination() const { return inclination_; }

private:
    bool initialized_ = false;

    // Эпоха TLE
    double epoch_ = 0.0;  // Юлианская дата эпохи

    // Орбитальные параметры
    double inclination_ = 0.0;   // [rad]
    double raan_ = 0.0;          // [rad]
    double eccentricity_ = 0.0;
    double argPerigee_ = 0.0;    // [rad]
    double meanAnomaly_ = 0.0;   // [rad]
    double meanMotion_ = 0.0;    // [rad/min]
    double semiMajorAxis_ = 0.0; // [km]

    // Возмущения
    double t2cof = 0.0;
    double t3cof = 0.0;
    double t4cof = 0.0;
    double t5cof = 0.0;
    double x1mth2 = 0.0;
    double x7thm1 = 0.0;
    double aycof = 0.0;
    double xlcof = 0.0;
    double xnodcf = 0.0;
    double xnodot = 0.0;
    double omgdot = 0.0;  // Скорость изменения аргумента перигея
    double xnodp = 0.0;   // Среднее движение с учётом возмущений

    // B-star параметр
    double bstar_ = 0.0;

    // Mean motion в rev/day (для getOrbitalPeriod)
    double meanMotionRevPerDay_ = 0.0;

    // Эпоха TLE (день года)
    double epochDay = 0.0;

    // Год эпохи
    int epochYear = 0;

    /**
     * @brief Парсинг TLE строки
     */
    bool parseTLE(const TLE& tle);

    /**
     * @brief Инициализация возмущений
     */
    void initPerturbations();

    /**
     * @brief Вычисление положения (основной алгоритм)
     */
    void sgp4(double tsince, double& r, double& v) const;
};

// ============================================================================
// Утилиты
// ============================================================================

namespace utils {

/**
 * @brief Преобразование ECI в географические координаты
 * @param eci ECI состояние
 * @param jdUT Юлианская дата UTC
 * @return LLA координаты
 */
LLACoords eciToLLA(const ECIState& eci, double jdUT);

/**
 * @brief Вычисление юлианской даты
 */
double julianDate(int year, int month, int day, double hour = 0.0);

/**
 * @brief Преобразование градусов в радианы
 */
constexpr double degToRad(double deg) {
    return deg * Constants::PI / 180.0;
}

/**
 * @brief Преобразование радиан в градусы
 */
constexpr double radToDeg(double rad) {
    return rad * 180.0 / Constants::PI;
}

/**
 * @brief Нормализация угла к диапазону [0, 2π)
 */
double normalizeAngle(double angle);

} // namespace utils

} // namespace navigation
} // namespace mka

#endif // SGP4_HPP
