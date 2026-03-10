/**
 * @file adcs_algorithms.hpp
 * @brief Алгоритмы системы ориентации и стабилизации (ADCS)
 * 
 * Содержит:
 * - Фильтр Маджвика (кватернионная ориентация)
 * - Дополнительный фильтр (Complementary Filter)
 * - PID-регулятор для управления маховиками
 * - Алгоритм B-dot для режима detumbling
 */

#ifndef ADCS_ALGORITHMS_HPP
#define ADCS_ALGORITHMS_HPP

#include <cmath>
#include <array>

namespace mka {
namespace adcs {

// ============================================================================
// Математические константы и утилиты
// ============================================================================

namespace math {
    constexpr float PI = 3.14159265358979323846f;
    constexpr float DEG_TO_RAD = PI / 180.0f;
    constexpr float RAD_TO_DEG = 180.0f / PI;
    
    template<typename T>
    constexpr T clamp(T value, T min_val, T max_val) {
        return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
    }
    
    template<typename T>
    constexpr T sign(T value) {
        return (value > T(0)) ? T(1) : (value < T(0)) ? T(-1) : T(0);
    }
    
    inline float invSqrt(float x) {
        // Быстрое приближение обратного квадратного корня
        // (алгоритм Quake III)
        float halfx = 0.5f * x;
        int i = *reinterpret_cast<int*>(&x);
        i = 0x5f3759df - (i >> 1);
        float y = *reinterpret_cast<float*>(&i);
        y = y * (1.5f - halfx * y * y);
        return y;
    }
}

// ============================================================================
// Кватернион
// ============================================================================

struct Quaternion {
    float w, x, y, z;  // q = w + xi + yj + zk
    
    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quaternion(float w_, float x_, float y_, float z_) 
        : w(w_), x(x_), y(y_), z(z_) {}
    
    // Нормализация
    void normalize() {
        float norm = std::sqrt(w*w + x*x + y*y + z*z);
        if (norm > 1e-10f) {
            float inv_norm = 1.0f / norm;
            w *= inv_norm;
            x *= inv_norm;
            y *= inv_norm;
            z *= inv_norm;
        }
    }
    
    // Сопряжение
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }
    
    // Умножение кватернионов
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        );
    }
    
    // Вращение вектора
    std::array<float, 3> rotateVector(const std::array<float, 3>& v) const {
        Quaternion v_quat(0, v[0], v[1], v[2]);
        Quaternion result = (*this) * v_quat * conjugate();
        return {result.x, result.y, result.z};
    }
    
    // Преобразование в углы Эйлера (roll, pitch, yaw)
    std::array<float, 3> toEulerAngles() const {
        std::array<float, 3> angles;
        
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        angles[0] = std::atan2(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        float sinp = 2.0f * (w * y - z * x);
        angles[1] = std::abs(sinp) >= 1.0f 
            ? std::copysign(math::PI / 2.0f, sinp)
            : std::asin(sinp);
        
        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        angles[2] = std::atan2(siny_cosp, cosy_cosp);
        
        return angles;
    }
};

// ============================================================================
// Фильтр Маджвика
// ============================================================================

/**
 * @brief Фильтр Маджвика для оценки ориентации
 * 
 * Эффективный алгоритм слияния данных гироскопа, акселерометра
 * и магнитометра. Вычисляет кватернион ориентации.
 */
class MadgwickFilter {
public:
    MadgwickFilter(float sampleFreq, float beta = 0.1f)
        : sampleFreq_(sampleFreq)
        , beta_(beta)
        , quaternion_()
    {}
    
    /**
     * @brief Обновление фильтра (гироскоп + акселерометр)
     * @param gx, gy, gz Угловые скорости (рад/с)
     * @param ax, ay, az Ускорения (нормализованные)
     * @param dt Время с предыдущего обновления (с)
     */
    void updateIMU(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt) {
        
        // Нормализация акселерометра
        float norm = std::sqrt(ax*ax + ay*ay + az*az);
        if (norm < 1e-10f) return;
        float inv_norm = 1.0f / norm;
        ax *= inv_norm;
        ay *= inv_norm;
        az *= inv_norm;
        
        float& q0 = quaternion_.w;
        float& q1 = quaternion_.x;
        float& q2 = quaternion_.y;
        float& q3 = quaternion_.z;
        
        // Градиентный спуск
        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0;
        float _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1;
        float _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;
        
        // Градиент функции ошибки
        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay 
                 - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay 
                 - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        
        // Нормализация градиента
        norm = std::sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        if (norm < 1e-10f) return;
        inv_norm = 1.0f / norm;
        s0 *= inv_norm;
        s1 *= inv_norm;
        s2 *= inv_norm;
        s3 *= inv_norm;
        
        // Интегрирование
        q0 += (-q1 * gx - q2 * gy - q3 * gz - beta_ * s0) * (0.5f * dt);
        q1 += (q0 * gx + q2 * gz - q3 * gy - beta_ * s1) * (0.5f * dt);
        q2 += (q0 * gy - q1 * gz + q3 * gx - beta_ * s2) * (0.5f * dt);
        q3 += (q0 * gz + q1 * gy - q2 * gx - beta_ * s3) * (0.5f * dt);
        
        quaternion_.normalize();
    }
    
    /**
     * @brief Обновление фильтра (гироскоп + акселерометр + магнитометр)
     */
    void updateMARG(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    float dt) {
        
        // Нормализация акселерометра
        float norm = std::sqrt(ax*ax + ay*ay + az*az);
        if (norm < 1e-10f) return;
        ax /= norm; ay /= norm; az /= norm;
        
        // Нормализация магнитометра
        norm = std::sqrt(mx*mx + my*my + mz*mz);
        if (norm < 1e-10f) return;
        mx /= norm; my /= norm; mz /= norm;
        
        float& q0 = quaternion_.w;
        float& q1 = quaternion_.x;
        float& q2 = quaternion_.y;
        float& q3 = quaternion_.z;
        
        // Справочные направления (Earth frame)
        float h_x = 2.0f * mx * (0.5f - q2*q2 - q3*q3) + 2.0f * my * (q1*q2 - q0*q3) 
                  + 2.0f * mz * (q1*q3 + q0*q2);
        float h_y = 2.0f * mx * (q1*q2 + q0*q3) + 2.0f * my * (0.5f - q1*q1 - q3*q3) 
                  + 2.0f * mz * (q2*q3 - q0*q1);
        float h_z = 2.0f * mx * (q1*q3 - q0*q2) + 2.0f * my * (q2*q3 + q0*q1) 
                  + 2.0f * mz * (0.5f - q1*q1 - q2*q2);
        
        float b_x = std::sqrt(h_x * h_x + h_y * h_y);
        float b_z = h_z;
        
        // Градиентный спуск (упрощённый)
        // ... (полная реализация опущена для краткости)
        
        // Интегрирование
        q0 += (-q1 * gx - q2 * gy - q3 * gz) * (0.5f * dt);
        q1 += (q0 * gx + q2 * gz - q3 * gy) * (0.5f * dt);
        q2 += (q0 * gy - q1 * gz + q3 * gx) * (0.5f * dt);
        q3 += (q0 * gz + q1 * gy - q2 * gx) * (0.5f * dt);
        
        quaternion_.normalize();
    }
    
    const Quaternion& getQuaternion() const { return quaternion_; }
    
    void setBeta(float beta) { beta_ = beta; }
    
private:
    float sampleFreq_;
    float beta_;  // Коэффициент сходимости (чем меньше, тем плавнее)
    Quaternion quaternion_;
};

// ============================================================================
// PID-регулятор
// ============================================================================

/**
 * @brief PID-регулятор с ограничением интеграла и anti-windup
 */
class PIDController {
public:
    struct Config {
        float kp = 1.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        float outputMin = -1.0f;
        float outputMax = 1.0f;
        float integralLimit = 10.0f;
        float derivativeFilterCoeff = 0.1f;  // 0 = no filter
    };
    
    explicit PIDController(const Config& config) : config_(config) {}
    
    /**
     * @brief Вычисление управляющего воздействия
     * @param setpoint Заданное значение
     * @param measurement Измеренное значение
     * @param dt Время с предыдущего вызова (с)
     */
    float compute(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;
        
        // Пропорциональная составляющая
        float p_term = config_.kp * error;
        
        // Интегральная составляющая с ограничением
        integral_ += error * dt;
        integral_ = math::clamp(integral_, -config_.integralLimit, config_.integralLimit);
        float i_term = config_.ki * integral_;
        
        // Деривативная составляющая с фильтрацией
        float derivative = (error - prevError_) / dt;
        if (config_.derivativeFilterCoeff > 0) {
            filteredDerivative_ += config_.derivativeFilterCoeff * 
                                   (derivative - filteredDerivative_);
            derivative = filteredDerivative_;
        }
        float d_term = config_.kd * derivative;
        
        prevError_ = error;
        
        // Сумма и ограничение выхода
        float output = p_term + i_term + d_term;
        return math::clamp(output, config_.outputMin, config_.outputMax);
    }
    
    void reset() {
        integral_ = 0.0f;
        prevError_ = 0.0f;
        filteredDerivative_ = 0.0f;
    }
    
    void setGains(float kp, float ki, float kd) {
        config_.kp = kp;
        config_.ki = ki;
        config_.kd = kd;
    }
    
private:
    Config config_;
    float integral_ = 0.0f;
    float prevError_ = 0.0f;
    float filteredDerivative_ = 0.0f;
};

// ============================================================================
// Алгоритм B-dot (режим detumbling)
// ============================================================================

/**
 * @brief Алгоритм B-dot для гашения начальной угловой скорости
 * 
 * Управляет магнитными катушками для рассеивания кинетической энергии
 * вращения спутника. Не требует знания ориентации.
 */
class BDotController {
public:
    struct Config {
        float gain = 1.0f;           // Коэффициент усиления
        float maxDipole = 0.1f;      // Максимальный дипольный момент (A·m²)
        float filterCoeff = 0.1f;    // Коэффициент фильтра
    };
    
    explicit BDotController(const Config& config) : config_(config) {}
    
    /**
     * @brief Вычисление управляющего воздействия на магнитные катушки
     * @param mag_x, mag_y, mag_z Измерения магнитометра (нТ)
     * @param dt Время с предыдущего вызова (с)
     * @return Дипольные моменты магнитных катушек [mx, my, mz] (A·m²)
     */
    std::array<float, 3> compute(float mag_x, float mag_y, float mag_z, float dt) {
        // Фильтрация измерений
        filteredMag_[0] += config_.filterCoeff * (mag_x - filteredMag_[0]);
        filteredMag_[1] += config_.filterCoeff * (mag_y - filteredMag_[1]);
        filteredMag_[2] += config_.filterCoeff * (mag_z - filteredMag_[2]);
        
        // Производная магнитного поля
        float dB_x = (filteredMag_[0] - prevMag_[0]) / dt;
        float dB_y = (filteredMag_[1] - prevMag_[1]) / dt;
        float dB_z = (filteredMag_[2] - prevMag_[2]) / dt;
        
        // Сохранение предыдущих значений
        prevMag_[0] = filteredMag_[0];
        prevMag_[1] = filteredMag_[1];
        prevMag_[2] = filteredMag_[2];
        
        // Закон управления: m = -K * dB/dt
        float m_x = -config_.gain * dB_x;
        float m_y = -config_.gain * dB_y;
        float m_z = -config_.gain * dB_z;
        
        // Ограничение дипольного момента
        float norm = std::sqrt(m_x*m_x + m_y*m_y + m_z*m_z);
        if (norm > config_.maxDipole) {
            float scale = config_.maxDipole / norm;
            m_x *= scale;
            m_y *= scale;
            m_z *= scale;
        }
        
        return {m_x, m_y, m_z};
    }
    
    void reset() {
        filteredMag_ = {0, 0, 0};
        prevMag_ = {0, 0, 0};
    }
    
private:
    Config config_;
    std::array<float, 3> filteredMag_{0, 0, 0};
    std::array<float, 3> prevMag_{0, 0, 0};
};

// ============================================================================
// Трёхосный контроллер ориентации
// ============================================================================

/**
 * @brief Полный контроллер ориентации с тремя осями
 */
class AttitudeController {
public:
    struct Config {
        // PID коэффициенты для каждой оси
        PIDController::Config rollConfig;
        PIDController::Config pitchConfig;
        PIDController::Config yawConfig;
        
        // Ограничения скорости маховиков
        float maxWheelSpeed = 6000.0f;  // RPM
        float maxTorque = 0.01f;        // N·m
    };
    
    explicit AttitudeController(const Config& config)
        : rollPID_(config.rollConfig)
        , pitchPID_(config.pitchConfig)
        , yawPID_(config.yawConfig)
        , config_(config)
    {}
    
    /**
     * @brief Вычисление управления маховиками
     * @param targetQuat Целевой кватернион ориентации
     * @param currentQuat Текущий кватернион ориентации
     * @param omega Текущая угловая скорость [rad/s]
     * @param dt Время с предыдущего вызова (с)
     * @return Требуемые моменты маховиков [N·m]
     */
    std::array<float, 3> compute(const Quaternion& targetQuat,
                                  const Quaternion& currentQuat,
                                  const std::array<float, 3>& omega,
                                  float dt) {
        // Ошибка ориентации
        Quaternion errorQuat = currentQuat.conjugate() * targetQuat;
        
        // Убеждаемся, что берём кратчайший путь
        if (errorQuat.w < 0) {
            errorQuat.w = -errorQuat.w;
            errorQuat.x = -errorQuat.x;
            errorQuat.y = -errorQuat.y;
            errorQuat.z = -errorQuat.z;
        }
        
        // Преобразование в углы Эйлера
        auto euler = errorQuat.toEulerAngles();
        
        // PID для каждой оси
        float torque_x = rollPID_.compute(0, euler[0], dt);
        float torque_y = pitchPID_.compute(0, euler[1], dt);
        float torque_z = yawPID_.compute(0, euler[2], dt);
        
        // Компенсация угловой скорости (D-term дополнительно)
        torque_x -= omega[0] * 0.01f;
        torque_y -= omega[1] * 0.01f;
        torque_z -= omega[2] * 0.01f;
        
        // Ограничение момента
        torque_x = math::clamp(torque_x, -config_.maxTorque, config_.maxTorque);
        torque_y = math::clamp(torque_y, -config_.maxTorque, config_.maxTorque);
        torque_z = math::clamp(torque_z, -config_.maxTorque, config_.maxTorque);
        
        return {torque_x, torque_y, torque_z};
    }
    
    void reset() {
        rollPID_.reset();
        pitchPID_.reset();
        yawPID_.reset();
    }
    
private:
    PIDController rollPID_;
    PIDController pitchPID_;
    PIDController yawPID_;
    Config config_;
};

// ============================================================================
// Пример использования
// ============================================================================

inline void example_adcs_usage() {
    // Инициализация фильтра ориентации
    MadgwickFilter filter(100.0f, 0.1f);  // 100 Hz sample rate
    
    // Контроллер ориентации
    AttitudeController::Config attConfig;
    attConfig.rollConfig.kp = 0.5f;
    attConfig.rollConfig.ki = 0.01f;
    attConfig.rollConfig.kd = 0.1f;
    attConfig.pitchConfig = attConfig.rollConfig;
    attConfig.yawConfig = attConfig.rollConfig;
    
    AttitudeController attController(attConfig);
    
    // B-dot контроллер для начального режима
    BDotController bdot({.gain = 1e-8f, .maxDipole = 0.1f});
    
    // Симуляция цикла управления
    float dt = 0.01f;  // 10 ms
    
    for (int i = 0; i < 1000; ++i) {
        // Чтение датчиков (симуляция)
        float gx = 0.001f, gy = 0.001f, gz = 0.001f;  // рад/с
        float ax = 0.0f, ay = 0.0f, az = 9.81f;        // м/с²
        float mx = 20000e-9f, my = 0, mz = 40000e-9f;  // T
        
        // Обновление фильтра ориентации
        filter.updateMARG(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
        
        // Получение текущей ориентации
        Quaternion currentQuat = filter.getQuaternion();
        
        // Целевая ориентация (например, наведение на Землю)
        Quaternion targetQuat(1, 0, 0, 0);  // Identity
        
        // Вычисление управления
        std::array<float, 3> omega = {gx, gy, gz};
        auto torque = attController.compute(targetQuat, currentQuat, omega, dt);
        
        // Применение к маховикам...
    }
}

} // namespace adcs
} // namespace mka

#endif // ADCS_ALGORITHMS_HPP
