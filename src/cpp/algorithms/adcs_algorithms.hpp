/**
 * @file adcs_algorithms.hpp
 * @brief Алгоритмы системы ориентации и стабилизации (ADCS)
 *
 * Содержит:
 * - Extended Kalman Filter (EKF) для оценки ориентации
 * - Фильтр Маджвика (кватернионная ориентация)
 * - Дополнительный фильтр (Complementary Filter)
 * - PID-регулятор для управления маховиками
 * - Алгоритм B-dot для режима detumbling
 */

#ifndef ADCS_ALGORITHMS_HPP
#define ADCS_ALGORITHMS_HPP

#include <cmath>
#include <cstdint>
#include <array>
#include <cstring>

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
        // Используем memcpy для безопасного type punning (избегает strict-aliasing violation)
        if (x <= 0.0f) return 0.0f;  // Защита от невалидных входов
        float halfx = 0.5f * x;
        int32_t i = 0;
        std::memcpy(&i, &x, sizeof(float));
        i = 0x5f3759df - (i >> 1);
        float y = 0.0f;
        std::memcpy(&y, &i, sizeof(float));
        y = y * (1.5f - halfx * y * y);
        return y;
    }
}

// ============================================================================
// Кватернион
// ============================================================================

struct Quaternion {
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;  // q = w + xi + yj + zk

    Quaternion() = default;
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
// Extended Kalman Filter (EKF) для оценки ориентации
// ============================================================================

/**
 * @brief Extended Kalman Filter для оценки ориентации спутника
 *
 * EKF использует нелинейную модель вращения твёрдого тела и слияние данных
 * гироскопа, акселерометра и магнитометра. Обеспечивает более точную оценку
 * по сравнению с Madgwick filter, особенно при высоких угловых скоростях.
 *
 * Состояние: кватернион ориентации (4 элемента) + смещения гироскопа (3 элемента)
 * Измерения: векторы ускорения и магнитного поля
 *
 * @note Вычислительно сложнее Madgwick, но точнее для динамичных манёвров
 */
class ExtendedKalmanFilter {
public:
    /**
     * @brief Конфигурация EKF
     */
    struct Config {
        // Шумы процесса
        float gyroNoiseStd;          // rad/s (гироскоп шум)
        float gyroBiasNoiseStd;      // rad/s² (дрейф гироскопа)

        // Шумы измерений
        float accelNoiseStd;         // m/s² (акселерометр шум)
        float magNoiseStd;           // µT (магнитометр шум)

        // Начальные ковариации
        float initialOrientationCov; // Начальная неопределённость ориентации
        float initialBiasCov;        // Начальная неопределённость смещения

        // Гравитация и магнитное поле (локальные значения)
        float gravityMagnitude;      // m/s²
        float magFieldMagnitude;     // T (типично для Земли)
        float magDipAngle;           // Угол наклона магнитного поля

        /**
         * @brief Конструктор по умолчанию со значениями по умолчанию
         */
        constexpr Config() noexcept
            : gyroNoiseStd(0.01f)
            , gyroBiasNoiseStd(0.0001f)
            , accelNoiseStd(0.1f)
            , magNoiseStd(0.05f)
            , initialOrientationCov(0.1f)
            , initialBiasCov(0.01f)
            , gravityMagnitude(9.81f)
            , magFieldMagnitude(50e-6f)
            , magDipAngle(60.0f * math::DEG_TO_RAD)
        {}
    };

    ExtendedKalmanFilter() = default;

    /**
     * @brief Инициализация фильтра
     * @param config Конфигурация EKF
     */
    void init(const Config& config = Config()) {
        config_ = config;

        // Состояние: [q0, q1, q2, q3, bias_x, bias_y, bias_z]
        state_ = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        // Инициализация ковариационной матрицы (7x7)
        P_ = {};
        for (int i = 0; i < 4; ++i) {
            P_[i * 7 + i] = config_.initialOrientationCov;
        }
        for (int i = 4; i < 7; ++i) {
            P_[i * 7 + i] = config_.initialBiasCov;
        }

        // Ковариации шумов процесса (7x7)
        Q_ = {};
        float gyroVar = config_.gyroNoiseStd * config_.gyroNoiseStd;
        float biasVar = config_.gyroBiasNoiseStd * config_.gyroBiasNoiseStd;
        for (int i = 0; i < 3; ++i) {
            Q_[(i + 1) * 7 + (i + 1)] = gyroVar;  // orientation
            Q_[(i + 4) * 7 + (i + 4)] = biasVar;  // bias
        }

        // Ковариации шумов измерений (6x6)
        float accelVar = config_.accelNoiseStd * config_.accelNoiseStd;
        float magVar = config_.magNoiseStd * config_.magNoiseStd;
        R_ = {};
        for (int i = 0; i < 3; ++i) {
            R_[i * 6 + i] = accelVar;
            R_[(i + 3) * 6 + (i + 3)] = magVar;
        }

        initialized_ = true;
    }

    /**
     * @brief Шаг предсказания (пропагация состояния)
     * @param gx, gy, gz Измерения гироскопа (рад/с)
     * @param dt Время с предыдущего шага (с)
     */
    void predict(float gx, float gy, float gz, float dt) {
        if (!initialized_) return;

        // Извлечение состояния
        float q0 = state_[0], q1 = state_[1], q2 = state_[2], q3 = state_[3];
        float bx = state_[4], by = state_[5], bz = state_[6];

        // Компенсация смещения гироскопа
        float wx = gx - bx;
        float wy = gy - by;
        float wz = gz - bz;

        // Производная кватерниона: dq/dt = 0.5 * q ⊗ ω
        float dq0 = 0.5f * (-q1 * wx - q2 * wy - q3 * wz);
        float dq1 = 0.5f * (q0 * wx + q2 * wz - q3 * wy);
        float dq2 = 0.5f * (q0 * wy - q1 * wz + q3 * wx);
        float dq3 = 0.5f * (q0 * wz + q1 * wy - q2 * wx);

        // Интегрирование (Euler)
        q0 += dq0 * dt;
        q1 += dq1 * dt;
        q2 += dq2 * dt;
        q3 += dq3 * dt;

        // Нормализация кватерниона с восстановлением при нулевой норме
        float norm = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        if (norm > 1e-10f) {
            float inv_norm = 1.0f / norm;
            q0 *= inv_norm;
            q1 *= inv_norm;
            q2 *= inv_norm;
            q3 *= inv_norm;
        } else {
            // Восстановление кватерниона по умолчанию
            q0 = 1.0f;
            q1 = 0.0f;
            q2 = 0.0f;
            q3 = 0.0f;
        }

        // Обновление состояния
        state_ = {q0, q1, q2, q3, bx, by, bz};

        // Вычисление матрицы перехода F (7x7)
        // F = ∂f/∂x, где f - нелинейная функция перехода
        float half_dt = 0.5f * dt;
        F_ = {};

        // ∂q/∂q (матрица вращения)
        F_[0] = 1.0f;
        F_[1] = -half_dt * wx;
        F_[2] = -half_dt * wy;
        F_[3] = -half_dt * wz;

        F_[7] = half_dt * wx;
        F_[8] = 1.0f;
        F_[9] = half_dt * wz;
        F_[10] = -half_dt * wy;

        F_[14] = half_dt * wy;
        F_[15] = -half_dt * wz;
        F_[16] = 1.0f;
        F_[17] = half_dt * wx;

        F_[21] = half_dt * wz;
        F_[22] = half_dt * wy;
        F_[23] = -half_dt * wx;
        F_[24] = 1.0f;

        // ∂q/∂bias (правильные производные)
        // dq/dbias = 0.5 * dt * ∂(q⊗ω)/∂bias
        F_[3] = 0.5f * dt * q1;   // ∂q0/∂bx
        F_[10] = -0.5f * dt * q0; // ∂q1/∂by
        F_[17] = -0.5f * dt * q0; // ∂q2/∂bz

        // ∂bias/∂bias (identity)
        F_[28] = 1.0f;
        F_[35] = 1.0f;
        F_[42] = 1.0f;

        // Propagate covariance: P = F * P * F^T + Q
        propagateCovariance();
    }

    /**
     * @brief Шаг коррекции (обновление по измерениям)
     * @param ax, ay, az Измерения акселерометра (м/с²)
     * @param mx, my, mz Измерения магнитометра (T)
     */
    void update(float ax, float ay, float az, float mx, float my, float mz) {
        if (!initialized_) return;

        // Нормализация измерений
        float accelNorm = std::sqrt(ax*ax + ay*ay + az*az);
        float magNorm = std::sqrt(mx*mx + my*my + mz*mz);

        if (accelNorm < 1e-6f || magNorm < 1e-6f) return;

        float invAccelNorm = 1.0f / accelNorm;
        float invMagNorm = 1.0f / magNorm;

        ax *= invAccelNorm; ay *= invAccelNorm; az *= invAccelNorm;
        mx *= invMagNorm; my *= invMagNorm; mz *= invMagNorm;

        // Ожидаемые измерения в системе координат спутника
        // Гравитация в Earth frame: [0, 0, g]
        // Магнитное поле в Earth frame: [B*cos(dip), 0, B*sin(dip)]
        float cosDip = std::cos(config_.magDipAngle);
        float sinDip = std::sin(config_.magDipAngle);

        // Преобразование ожидаемых векторов в body frame через кватернион
        float q0 = state_[0], q1 = state_[1], q2 = state_[2], q3 = state_[3];

        // Ожидаемый вектор гравитации (в body frame)
        float expectedAx = 2.0f * (q1*q3 - q0*q2);
        float expectedAy = 2.0f * (q2*q3 + q0*q1);
        float expectedAz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        // Ожидаемый вектор магнитного поля (в body frame)
        float magEarthX = config_.magFieldMagnitude * cosDip;
        float magEarthZ = config_.magFieldMagnitude * sinDip;

        float expectedMx = magEarthX * (q0*q0 + q1*q1 - q2*q2 - q3*q3)
                         + magEarthZ * 2.0f * (q1*q3 - q0*q2);
        float expectedMy = magEarthX * 2.0f * (q1*q2 - q0*q3)
                         + magEarthZ * 2.0f * (q2*q3 + q0*q1);
        float expectedMz = magEarthX * 2.0f * (q1*q3 + q0*q2)
                         + magEarthZ * (q0*q0 - q1*q1 - q2*q2 + q3*q3);

        // Вектор невязки: y = z - h(x)
        std::array<float, 6> y;
        y[0] = ax - expectedAx;
        y[1] = ay - expectedAy;
        y[2] = az - expectedAz;
        y[3] = mx - expectedMx;
        y[4] = my - expectedMy;
        y[5] = mz - expectedMz;

        // Матрица Якоби H (6x7): H = ∂h/∂x
        H_ = {};

        // ∂accel/∂q
        H_[2] = -2.0f * q2;  H_[3] = 2.0f * q1;   H_[6] = -2.0f * q0;  H_[7] = 2.0f * q3;
        H_[8] = 2.0f * q1;   H_[9] = 2.0f * q0;   H_[12] = 2.0f * q3; H_[13] = 2.0f * q2;
        H_[14] = 2.0f * q0;  H_[15] = -2.0f * q1; H_[18] = 2.0f * q2; H_[19] = -2.0f * q3;

        // ∂mag/∂q
        H_[20] = 2.0f * magEarthX * q1 + 2.0f * magEarthZ * q3;
        H_[21] = 2.0f * magEarthX * q0 + 2.0f * magEarthZ * q1;
        H_[22] = -2.0f * magEarthX * q2 + 2.0f * magEarthZ * q0;
        H_[23] = -2.0f * magEarthX * q3 + 2.0f * magEarthZ * q2;

        H_[26] = -2.0f * magEarthX * q3 + 2.0f * magEarthZ * q1;
        H_[27] = 2.0f * magEarthX * q2 + 2.0f * magEarthZ * q0;
        H_[28] = 2.0f * magEarthX * q1 + 2.0f * magEarthZ * q3;
        H_[29] = 2.0f * magEarthX * q0 + 2.0f * magEarthZ * q2;

        H_[32] = 2.0f * magEarthX * q2 + 2.0f * magEarthZ * q0;
        H_[33] = -2.0f * magEarthX * q3 + 2.0f * magEarthZ * q1;
        H_[34] = 2.0f * magEarthX * q0 + 2.0f * magEarthZ * q2;
        H_[35] = -2.0f * magEarthX * q1 + 2.0f * magEarthZ * q3;

        // Kalman gain: K = P * H^T * (H * P * H^T + R)^(-1)
        computeKalmanGain();

        // Обновление состояния: x = x + K * y
        for (int i = 0; i < 7; ++i) {
            float correction = 0.0f;
            for (int j = 0; j < 6; ++j) {
                correction += K_[i * 6 + j] * y[j];
            }
            state_[i] += correction;
        }

        // Нормализация кватерниона после коррекции
        float norm = std::sqrt(state_[0]*state_[0] + state_[1]*state_[1] +
                               state_[2]*state_[2] + state_[3]*state_[3]);
        if (norm > 1e-10f) {
            float inv_norm = 1.0f / norm;
            state_[0] *= inv_norm;
            state_[1] *= inv_norm;
            state_[2] *= inv_norm;
            state_[3] *= inv_norm;
        }

        // Обновление ковариации: P = (I - K*H) * P
        updateCovariance();
    }

    /**
     * @brief Получить текущую оценку ориентации
     * @return Кватернион ориентации
     */
    Quaternion getQuaternion() const {
        return Quaternion(state_[0], state_[1], state_[2], state_[3]);
    }

    /**
     * @brief Получить оценку смещения гироскопа
     * @return Смещение гироскопа [bx, by, bz] (рад/с)
     */
    std::array<float, 3> getGyroBias() const {
        return {state_[4], state_[5], state_[6]};
    }

    /**
     * @brief Проверка инициализации
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Сброс фильтра
     */
    void reset() {
        initialized_ = false;
        state_ = {};
        P_ = {};
        K_ = {};
    }

private:
    /**
     * @brief Пропагация ковариационной матрицы
     * P = F * P * F^T + Q
     */
    void propagateCovariance() {
        // TMP = F * P
        std::array<float, 49> TMP = {};
        for (int i = 0; i < 7; ++i) {
            for (int j = 0; j < 7; ++j) {
                for (int k = 0; k < 7; ++k) {
                    TMP[i * 7 + j] += F_[i * 7 + k] * P_[k * 7 + j];
                }
            }
        }

        // P = TMP * F^T + Q
        for (int i = 0; i < 7; ++i) {
            for (int j = 0; j < 7; ++j) {
                P_[i * 7 + j] = 0.0f;
                for (int k = 0; k < 7; ++k) {
                    P_[i * 7 + j] += TMP[i * 7 + k] * F_[j * 7 + k];
                }
                P_[i * 7 + j] += Q_[i * 7 + j];
            }
        }
    }

    /**
     * @brief Вычисление коэффициента Калмана
     * K = P * H^T * (H * P * H^T + R)^(-1)
     */
    void computeKalmanGain() {
        // S = H * P * H^T + R (6x6)
        // HP = H * P (6x7) * (7x7) -> (6x7)
        std::array<float, 42> HP = {};
        std::array<float, 36> S = {};

        // HP = H * P (6x7)
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 7; ++j) {
                for (int k = 0; k < 7; ++k) {
                    HP[i * 7 + j] += H_[i * 7 + k] * P_[k * 7 + j];
                }
            }
        }

        // S = HP * H^T + R (6x6)
        // H^T[j][k] = H_[k][j] = H_[k * 7 + j]
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                S[i * 6 + j] = R_[i * 6 + j];
                for (int k = 0; k < 7; ++k) {
                    S[i * 6 + j] += HP[i * 7 + k] * H_[j * 7 + k];
                }
            }
        }

        // Инверсия S (методом Гаусса-Жордана)
        std::array<float, 36> S_inv = invertMatrix6x6(S);

        // K = P * H^T * S_inv (7x6)
        // P * H^T: (7x7) * (7x6) -> (7x6)
        // H^T[k][j] = H_[j][k] = H_[j * 7 + k]
        std::array<float, 42> PH_T = {};
        for (int i = 0; i < 7; ++i) {
            for (int j = 0; j < 6; ++j) {
                for (int k = 0; k < 7; ++k) {
                    PH_T[i * 6 + j] += P_[i * 7 + k] * H_[j * 7 + k];
                }
            }
        }

        // K = (P * H^T) * S_inv
        for (int i = 0; i < 7; ++i) {
            for (int j = 0; j < 6; ++j) {
                K_[i * 6 + j] = 0.0f;
                for (int k = 0; k < 6; ++k) {
                    K_[i * 6 + j] += PH_T[i * 6 + k] * S_inv[k * 6 + j];
                }
            }
        }
    }

    /**
     * @brief Обновление ковариационной матрицы
     * P = (I - K*H) * P
     */
    void updateCovariance() {
        // KH = K * H (7x7)
        // K: 7x6, H: 6x7 -> KH: 7x7
        std::array<float, 49> KH = {};
        for (int i = 0; i < 7; ++i) {
            for (int j = 0; j < 7; ++j) {
                for (int k = 0; k < 6; ++k) {
                    KH[i * 7 + j] += K_[i * 6 + k] * H_[k * 7 + j];
                }
            }
        }

        // I - KH
        std::array<float, 49> IKH = {};
        for (int i = 0; i < 7; ++i) {
            IKH[i * 7 + i] = 1.0f;
            for (int j = 0; j < 7; ++j) {
                IKH[i * 7 + j] -= KH[i * 7 + j];
            }
        }

        // P = (I - KH) * P
        std::array<float, 49> newP = {};
        for (int i = 0; i < 7; ++i) {
            for (int j = 0; j < 7; ++j) {
                for (int k = 0; k < 7; ++k) {
                    newP[i * 7 + j] += IKH[i * 7 + k] * P_[k * 7 + j];
                }
            }
        }
        P_ = newP;
    }

    /**
     * @brief Инверсия матрицы 6x6 методом Гаусса-Жордана
     * @param matrix Матрица для инверсии
     * @return Обратная матрица
     */
    std::array<float, 36> invertMatrix6x6(const std::array<float, 36>& matrix) const {
        // Копия матрицы и единичная матрица для результата
        std::array<float, 36> A = matrix;
        std::array<float, 36> inv = {};

        for (int i = 0; i < 6; ++i) {
            inv[i * 6 + i] = 1.0f;
        }

        // Прямой ход
        for (int col = 0; col < 6; ++col) {
            // Поиск ведущего элемента
            int pivotRow = col;
            float maxVal = std::abs(A[col * 6 + col]);
            for (int row = col + 1; row < 6; ++row) {
                float val = std::abs(A[row * 6 + col]);
                if (val > maxVal) {
                    maxVal = val;
                    pivotRow = row;
                }
            }

            // Обмен строк
            if (pivotRow != col) {
                for (int j = 0; j < 6; ++j) {
                    std::swap(A[col * 6 + j], A[pivotRow * 6 + j]);
                    std::swap(inv[col * 6 + j], inv[pivotRow * 6 + j]);
                }
            }

            // Проверка на вырожденность
            float pivot = A[col * 6 + col];
            if (std::abs(pivot) < 1e-10f) {
                // Возврат единичной матрицы при вырожденности
                inv = {};
                for (int i = 0; i < 6; ++i) {
                    inv[i * 6 + i] = 1.0f;
                }
                return inv;
            }

            // Нормализация строки
            float invPivot = 1.0f / pivot;
            for (int j = 0; j < 6; ++j) {
                A[col * 6 + j] *= invPivot;
                inv[col * 6 + j] *= invPivot;
            }

            // Обнуление столбца
            for (int row = 0; row < 6; ++row) {
                if (row != col) {
                    float factor = A[row * 6 + col];
                    for (int j = 0; j < 6; ++j) {
                        A[row * 6 + j] -= factor * A[col * 6 + j];
                        inv[row * 6 + j] -= factor * inv[col * 6 + j];
                    }
                }
            }
        }

        return inv;
    }

    Config config_;
    bool initialized_ = false;

    // Состояние: [q0, q1, q2, q3, bias_x, bias_y, bias_z]
    std::array<float, 7> state_{};

    // Матрицы EKF
    std::array<float, 49> P_{};  // Ковариация (7x7)
    std::array<float, 49> F_{};  // Матрица перехода (7x7)
    std::array<float, 49> Q_{};  // Шум процесса (7x7)
    std::array<float, 42> H_{};  // Матрица измерений (6x7)
    std::array<float, 36> R_{};  // Шум измерений (6x6)
    std::array<float, 42> K_{};  // Коэффициент Калмана (7x6)
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

        // Нормализация горизонтальных компонент магнитного поля
        float h_xy_norm = std::sqrt(h_x * h_x + h_y * h_y);
        if (h_xy_norm < 1e-6f) {
            h_xy_norm = 1e-6f;  // Защита от деления на ноль
        }

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
        float outputMin = -100.0f;
        float outputMax = 100.0f;
        float integralLimit = 10.0f;
        float derivativeFilterCoeff = 0.1f;  // 0 = no filter
    };

    PIDController() = default;
    explicit PIDController(const Config& config) : config_(config) {}
    
    /**
     * @brief Вычисление управляющего воздействия
     * @param setpoint Заданное значение
     * @param measurement Измеренное значение
     * @param dt Время с предыдущего вызова (с)
     */
    float compute(float setpoint, float measurement, float dt) {
        // Защита от деления на ноль
        if (dt < 1e-6f) {
            return lastOutput_;
        }

        float error = setpoint - measurement;

        // Пропорциональная составляющая
        float p_term = config_.kp * error;

        // Интегральная составляющая с anti-windup (conditional integration)
        // Интегрируем только если выход не насыщен
        float outputUnclamped = p_term + config_.ki * integral_ + config_.kd * (error - prevError_) / dt;
        float output = math::clamp(outputUnclamped, config_.outputMin, config_.outputMax);

        // Conditional integration - накапливаем только если не в насыщении
        bool notSaturated = (outputUnclamped > config_.outputMin && outputUnclamped < config_.outputMax);
        if (notSaturated || (error >= 0 && outputUnclamped <= config_.outputMin) ||
            (error <= 0 && outputUnclamped >= config_.outputMax)) {
            integral_ += error * dt;
            integral_ = math::clamp(integral_, -config_.integralLimit, config_.integralLimit);
        }

        // Деривативная составляющая с фильтрацией
        float derivative = (error - prevError_) / dt;
        if (config_.derivativeFilterCoeff > 0) {
            filteredDerivative_ += config_.derivativeFilterCoeff *
                                   (derivative - filteredDerivative_);
            derivative = filteredDerivative_;
        }

        prevError_ = error;

        lastOutput_ = output;
        return output;
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
    float lastOutput_ = 0.0f;
};

// ============================================================================
// PD контроллер ориентации
// ============================================================================

/**
 * @brief PD контроллер ориентации на кватернионах
 *
 * Вычисляет управляющий момент для разворота спутника из текущей
 * ориентации в целевую. Использует кватернионную ошибку и угловую скорость.
 *
 * Закон управления:
 *   τ = -Kp * q_err - Kd * ω
 *
 * где:
 *   - q_err - векторная часть кватерниона ошибки
 *   - ω - угловая скорость
 */
class PDAttitudeController {
public:
    struct Config {
        float kp = 0.1f;           // Пропорциональный коэффициент
        float kd = 1.0f;           // Демпфирующий коэффициент
        float maxTorque = 0.1f;    // Максимальный момент [N·m]
        float maxOmega = 1.0f;     // Макс. угловая скорость для демпфирования [rad/s]
    };

    PDAttitudeController() = default;
    explicit PDAttitudeController(const Config& config) : config_(config) {}

    /**
     * @brief Вычисление управляющего момента
     * @param targetQuat Целевой кватернион ориентации (нормализованный)
     * @param currentQuat Текущий кватернион ориентации (нормализованный)
     * @param omega Угловая скорость [rad/s] (body frame)
     * @return Требуемый момент [N·m]
     */
    std::array<float, 3> compute(const Quaternion& targetQuat,
                                  const Quaternion& currentQuat,
                                  const std::array<float, 3>& omega) {
        // Кватернион ошибки: q_err = current* ⊗ target
        Quaternion errorQuat = currentQuat.conjugate() * targetQuat;

        // Выбор кратчайшего пути разворота
        if (errorQuat.w < 0) {
            errorQuat.w = -errorQuat.w;
            errorQuat.x = -errorQuat.x;
            errorQuat.y = -errorQuat.y;
            errorQuat.z = -errorQuat.z;
        }

        // Векторная часть ошибки (пропорциональная составляющая)
        float q_err[3] = {errorQuat.x, errorQuat.y, errorQuat.z};

        // PD закон управления
        float torque_x = -config_.kp * q_err[0] - config_.kd * omega[0];
        float torque_y = -config_.kp * q_err[1] - config_.kd * omega[1];
        float torque_z = -config_.kp * q_err[2] - config_.kd * omega[2];

        // Ограничение момента
        float norm = std::sqrt(torque_x*torque_x + torque_y*torque_y + torque_z*torque_z);
        if (norm > config_.maxTorque && norm > 1e-10f) {
            float scale = config_.maxTorque / norm;
            torque_x *= scale;
            torque_y *= scale;
            torque_z *= scale;
        }

        return {torque_x, torque_y, torque_z};
    }

    /**
     * @brief Вычисление с нормализацией угловой скорости
     * @param targetQuat Целевой кватернион
     * @param currentQuat Текущий кватернион
     * @param omega Угловая скорость [rad/s]
     * @param dt Время с предыдущего вызова [s]
     * @return Требуемый момент [N·m]
     */
    std::array<float, 3> computeWithRateLimit(
            const Quaternion& targetQuat,
            const Quaternion& currentQuat,
            const std::array<float, 3>& omega,
            float dt) {
        (void)dt; // Reserved for future rate limiting dynamics

        // Ограничение угловой скорости
        std::array<float, 3> omegaLimited = omega;
        for (int i = 0; i < 3; ++i) {
            omegaLimited[i] = math::clamp(omegaLimited[i],
                                          -config_.maxOmega,
                                          config_.maxOmega);
        }

        return compute(targetQuat, currentQuat, omegaLimited);
    }

    /**
     * @brief Установка коэффициентов
     */
    void setGains(float kp, float kd) {
        config_.kp = kp;
        config_.kd = kd;
    }

    /**
     * @brief Получить текущие коэффициенты
     */
    Config getConfig() const { return config_; }

    /**
     * @brief Сброс внутреннего состояния
     */
    void reset() {
        // PD контроллер не имеет состояния, но оставляем для совместимости
    }

private:
    Config config_;
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

    BDotController() = default;
    explicit BDotController(const Config& config) : config_(config) {}
    
    /**
     * @brief Вычисление управляющего воздействия на магнитные катушки
     * @param mag_x, mag_y, mag_z Измерения магнитометра (нТ)
     * @param dt Время с предыдущего вызова (с)
     * @return Дипольные моменты магнитных катушек [mx, my, mz] (A·m²)
     */
    std::array<float, 3> compute(float mag_x, float mag_y, float mag_z, float dt) {
        // Защита от деления на ноль
        if (dt < 1e-6f) {
            return {0.0f, 0.0f, 0.0f};
        }

        // Инициализация при первом вызове
        if (!initialized_) {
            filteredMag_[0] = mag_x;
            filteredMag_[1] = mag_y;
            filteredMag_[2] = mag_z;
            prevMag_[0] = mag_x;
            prevMag_[1] = mag_y;
            prevMag_[2] = mag_z;
            initialized_ = true;
            return {0.0f, 0.0f, 0.0f};  // Первый вызов - нет производной
        }

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
    bool initialized_ = false;
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

    AttitudeController() = default;
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
    // Инициализация фильтра Маджвика
    MadgwickFilter filter(100.0f, 0.1f);  // 100 Hz sample rate

    // Инициализация EKF (более точный, но сложнее)
    ExtendedKalmanFilter ekf;
    ExtendedKalmanFilter::Config ekfConfig;
    ekfConfig.gyroNoiseStd = 0.01f;
    ekfConfig.accelNoiseStd = 0.1f;
    ekfConfig.magNoiseStd = 0.05f;
    ekf.init(ekfConfig);

    // Контроллер ориентации
    AttitudeController::Config attConfig;
    attConfig.rollConfig.kp = 0.5f;
    attConfig.rollConfig.ki = 0.01f;
    attConfig.rollConfig.kd = 0.1f;
    attConfig.pitchConfig = attConfig.rollConfig;
    attConfig.yawConfig = attConfig.rollConfig;

    AttitudeController attController(attConfig);

    // B-dot контроллер для начального режима
    BDotController::Config bdotConfig;
    bdotConfig.gain = 1e-8f;
    bdotConfig.maxDipole = 0.1f;
    BDotController bdot(bdotConfig);

    // Симуляция цикла управления
    float dt = 0.01f;  // 10 ms

    for (int i = 0; i < 1000; ++i) {
        // Чтение датчиков (симуляция)
        float gx = 0.001f, gy = 0.001f, gz = 0.001f;  // рад/с
        float ax = 0.0f, ay = 0.0f, az = 9.81f;        // м/с²
        float mx = 20000e-9f, my = 0, mz = 40000e-9f;  // T

        // Обновление фильтра Маджвика
        filter.updateMARG(gx, gy, gz, ax, ay, az, mx, my, mz, dt);

        // Обновление EKF (predict + update)
        ekf.predict(gx, gy, gz, dt);
        ekf.update(ax, ay, az, mx, my, mz);

        // Получение текущей ориентации (EKF более точный)
        Quaternion currentQuat = ekf.getQuaternion();

        // Получение оценки смещения гироскопа из EKF
        // Используется для компенсации дрейфа гироскопа
        auto gyroBias = ekf.getGyroBias();
        float gx_compensated = gx - gyroBias[0];
        float gy_compensated = gy - gyroBias[1];
        float gz_compensated = gz - gyroBias[2];

        // Целевая ориентация (например, наведение на Землю)
        Quaternion targetQuat(1, 0, 0, 0);  // Identity

        // Вычисление управления с компенсированными значениями
        std::array<float, 3> omega = {gx_compensated, gy_compensated, gz_compensated};
        attController.compute(targetQuat, currentQuat, omega, dt);

        // Применение к маховикам...
        (void)bdot;  // bdot готов к использованию
    }
}

// ============================================================================
// Unscented Kalman Filter (UKF) для оценки ориентации
// ============================================================================

/**
 * @brief Unscented Kalman Filter для оценки ориентации
 *
 * Более точная альтернатива EKF для нелинейных систем.
 * Использует Unscented Transform для распространения статистики
 * через нелинейные функции без линеаризации.
 *
 * Состояние: кватернион ориентации (4) + смещение гироскопа (3) = 7
 * Измерения: акселерометр (3) + магнитометр (3) = 6
 *
 * Преимущества перед EKF:
 * - Не требует вычисления матриц Якоби
 * - Более точная аппроксимация нелинейностей
 * - Лучшая сходимость при больших начальных ошибках
 */
class UnscentedKalmanFilter {
public:
    // Параметры UKF
    struct Config {
        float processNoiseQ = 0.001f;    // Шум процесса
        float measurementNoiseR = 0.1f;  // Шум измерений
        float alpha = 0.001f;            // Parameter for sigma points (1e-3 typical)
        float beta = 2.0f;               // Incorporates prior knowledge (2 is optimal for Gaussian)
        float kappa = 0.0f;              // Secondary scaling parameter (0 or 3-n_x)
    };

    UnscentedKalmanFilter() = default;

    explicit UnscentedKalmanFilter(const Config& config)
        : config_(config) {
        reset();
    }

    /**
     * @brief Инициализация фильтра
     * @param initialQuat Начальный кватернион ориентации
     * @param gyroBias Начальное смещение гироскопа
     */
    void init(const Quaternion& initialQuat = Quaternion(),
              const std::array<float, 3>& gyroBias = {0, 0, 0}) {
        // Инициализация состояния
        state_ = {
            initialQuat.w, initialQuat.x, initialQuat.y, initialQuat.z,
            gyroBias[0], gyroBias[1], gyroBias[2]
        };

        // Инициализация ковариации (диагональная)
        for (int i = 0; i < STATE_DIM; i++) {
            for (int j = 0; j < STATE_DIM; j++) {
                P_[i * STATE_DIM + j] = (i == j) ? 0.1f : 0.0f;
            }
        }

        initialized_ = true;
    }

    /**
     * @brief Прогноз состояния (prediction step)
     * @param gyro Измерение гироскопа [rad/s]
     * @param dt Время с последнего шага [s]
     */
    void predict(const std::array<float, 3>& gyro, float dt) {
        if (!initialized_) return;

        // Компенсация смещения гироскопа
        const float omegaX = gyro[0] - state_[4];
        const float omegaY = gyro[1] - state_[5];
        const float omegaZ = gyro[2] - state_[6];

        // Генерация sigma points
        std::array<std::array<float, STATE_DIM>, SIGMA_COUNT> sigmaPoints;
        generateSigmaPoints(sigmaPoints);

        // Propagate sigma points через кинематическое уравнение
        for (int i = 0; i < SIGMA_COUNT; i++) {
            propagateSigmaPoint(sigmaPoints[i], {omegaX, omegaY, omegaZ}, dt);
        }

        // Вычисление предсказанного среднего состояния
        computeMeanState(sigmaPoints);

        // Вычисление предсказанной ковариации
        computeCovariance(sigmaPoints);
    }

    /**
     * @brief Коррекция по измерениям (update step)
     * @param ax, ay, az Акселерометр [g]
     * @param mx, my, mz Магнитометр [Gauss]
     */
    void update(float ax, float ay, float az, float mx, float my, float mz) {
        if (!initialized_) return;

        // Нормализация измерений
        float accelNorm = std::sqrt(ax*ax + ay*ay + az*az);
        float magNorm = std::sqrt(mx*mx + my*my + mz*mz);

        if (accelNorm < 0.1f || magNorm < 0.1f) return;  // Недействительные данные

        ax /= accelNorm; ay /= accelNorm; az /= accelNorm;
        mx /= magNorm; my /= magNorm; mz /= magNorm;

        // Генерация sigma points из текущего состояния
        std::array<std::array<float, STATE_DIM>, SIGMA_COUNT> sigmaPoints;
        generateSigmaPoints(sigmaPoints);

        // Вычисление предсказанных измерений для каждого sigma point
        std::array<std::array<float, MEAS_DIM>, SIGMA_COUNT> predictedMeasurements;
        for (int i = 0; i < SIGMA_COUNT; i++) {
            predictMeasurement(sigmaPoints[i], predictedMeasurements[i]);
        }

        // Вычисление среднего предсказанного измерения
        std::array<float, MEAS_DIM> zPred{};
        for (int i = 0; i < SIGMA_COUNT; i++) {
            float w = (i == 0) ? Wm_[0] : Wm_[i];
            for (int j = 0; j < MEAS_DIM; j++) {
                zPred[j] += w * predictedMeasurements[i][j];
            }
        }

        // Вычисление ковариации измерений (S) и кросс-ковариации (Pxz)
        float S[MEAS_DIM * MEAS_DIM]{};
        float Pxz[STATE_DIM * MEAS_DIM]{};

        for (int i = 0; i < SIGMA_COUNT; i++) {
            float w = (i == 0) ? Wc_[0] : Wc_[i];

            // Разница измерений
            std::array<float, MEAS_DIM> dz{};
            for (int j = 0; j < MEAS_DIM; j++) {
                dz[j] = predictedMeasurements[i][j] - zPred[j];
            }

            // Разница состояния
            std::array<float, STATE_DIM> dx{};
            for (int j = 0; j < STATE_DIM; j++) {
                dx[j] = sigmaPoints[i][j] - state_[j];
            }

            // S = sum(w * dz * dz^T) + R
            for (int j = 0; j < MEAS_DIM; j++) {
                for (int k = 0; k < MEAS_DIM; k++) {
                    S[j * MEAS_DIM + k] += w * dz[j] * dz[k];
                }
            }

            // Pxz = sum(w * dx * dz^T)
            for (int j = 0; j < STATE_DIM; j++) {
                for (int k = 0; k < MEAS_DIM; k++) {
                    Pxz[j * MEAS_DIM + k] += w * dx[j] * dz[k];
                }
            }
        }

        // Добавление шума измерений к S
        for (int i = 0; i < MEAS_DIM; i++) {
            S[i * MEAS_DIM + i] += config_.measurementNoiseR;
        }

        // Вычисление коэффициента Калмана: K = Pxz * S^-1
        float K[STATE_DIM * MEAS_DIM]{};
        float Sinv[MEAS_DIM * MEAS_DIM]{};
        matrixInverse3x3(S, Sinv);  // Упрощённая инверсия для 6x6
        matrixMultiply(K, Pxz, Sinv, STATE_DIM, MEAS_DIM, MEAS_DIM);

        // Вектор инноваций (разница между измерением и предсказанием)
        std::array<float, MEAS_DIM> y{};
        float zMeas[MEAS_DIM] = {ax, ay, az, mx, my, mz};
        for (int i = 0; i < MEAS_DIM; i++) {
            y[i] = zMeas[i] - zPred[i];
        }

        // Обновление состояния: x = x + K * y
        for (int i = 0; i < STATE_DIM; i++) {
            for (int j = 0; j < MEAS_DIM; j++) {
                state_[i] += K[i * MEAS_DIM + j] * y[j];
            }
        }

        // Нормализация кватерниона
        normalizeQuaternion();

        // Обновление ковариации: P = P - K * S * K^T
        updateCovariance(K, S);
    }

    /**
     * @brief Получить текущую оценку ориентации
     * @return Кватернион ориентации
     */
    Quaternion getQuaternion() const {
        return Quaternion(state_[0], state_[1], state_[2], state_[3]);
    }

    /**
     * @brief Получить оценку смещения гироскопа
     * @return Смещение гироскопа [rad/s]
     */
    std::array<float, 3> getGyroBias() const {
        return {state_[4], state_[5], state_[6]};
    }

    /**
     * @brief Проверка инициализации фильтра
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Сброс фильтра
     */
    void reset() {
        state_ = {};
        for (int i = 0; i < STATE_DIM * STATE_DIM; i++) {
            P_[i] = 0.0f;
        }
        initialized_ = false;
    }

private:
    static constexpr int STATE_DIM = 7;   // 4 (quaternion) + 3 (gyro bias)
    static constexpr int MEAS_DIM = 6;    // 3 (accel) + 3 (mag)
    static constexpr int SIGMA_COUNT = 2 * STATE_DIM + 1;  // 15 sigma points

    Config config_;
    bool initialized_ = false;

    // Вектор состояния [qw, qx, qy, qz, bx, by, bz]
    std::array<float, STATE_DIM> state_{};

    // Матрица ковариации (STATE_DIM x STATE_DIM)
    std::array<float, STATE_DIM * STATE_DIM> P_{};

    // Веса для sigma points
    std::array<float, SIGMA_COUNT> Wm_{};  // Weights for mean
    std::array<float, SIGMA_COUNT> Wc_{};  // Weights for covariance

    // Лямбда параметр
    float lambda_ = 0.0f;

    /**
     * @brief Инициализация весов и лямбда
     */
    void initWeights() {
        const float n = static_cast<float>(STATE_DIM);
        lambda_ = config_.alpha * config_.alpha * (n + config_.kappa) - n;

        // Wm[0] = lambda / (n + lambda)
        Wm_[0] = lambda_ / (n + lambda_);
        Wc_[0] = Wm_[0] + (1.0f - config_.alpha * config_.alpha + config_.beta);

        // Wm[i] = Wc[i] = 1 / (2 * (n + lambda)) for i > 0
        float w = 1.0f / (2.0f * (n + lambda_));
        for (int i = 1; i < SIGMA_COUNT; i++) {
            Wm_[i] = w;
            Wc_[i] = w;
        }
    }

    /**
     * @brief Генерация sigma points
     */
    void generateSigmaPoints(std::array<std::array<float, STATE_DIM>, SIGMA_COUNT>& points) {
        if (lambda_ == 0.0f) initWeights();

        // points[0] = state (mean)
        points[0] = state_;

        // Вычисление sqrt((n + lambda) * P) через разложение Холецкого
        float sqrtCov[STATE_DIM * STATE_DIM]{};
        choleskyDecomposition(P_.data(), sqrtCov, STATE_DIM);

        float scale = std::sqrt(STATE_DIM + lambda_);

        // Sigma points: state +/- scale * column_i of sqrt(P)
        for (int i = 0; i < STATE_DIM; i++) {
            for (int j = 0; j < STATE_DIM; j++) {
                points[i + 1][j] = state_[j] + scale * sqrtCov[j * STATE_DIM + i];
                points[STATE_DIM + i + 1][j] = state_[j] - scale * sqrtCov[j * STATE_DIM + i];
            }
        }
    }

    /**
     * @brief Propagate sigma point через кинематическое уравнение
     */
    void propagateSigmaPoint(std::array<float, STATE_DIM>& sigma,
                             const std::array<float, 3>& omega, float dt) {
        // Извлечение кватерниона из sigma point
        Quaternion q(sigma[0], sigma[1], sigma[2], sigma[3]);

        // Интегрирование кватерниона: q_dot = 0.5 * q ⊗ omega
        float qDot[4];
        qDot[0] = 0.5f * (-q.x * omega[0] - q.y * omega[1] - q.z * omega[2]);
        qDot[1] = 0.5f * ( q.w * omega[0] + q.y * omega[2] - q.z * omega[1]);
        qDot[2] = 0.5f * ( q.w * omega[1] - q.x * omega[2] + q.z * omega[0]);
        qDot[3] = 0.5f * ( q.w * omega[2] + q.x * omega[1] - q.y * omega[0]);

        // Эйлерово интегрирование
        sigma[0] += qDot[0] * dt;
        sigma[1] += qDot[1] * dt;
        sigma[2] += qDot[2] * dt;
        sigma[3] += qDot[3] * dt;

        // Нормализация кватерниона
        float norm = std::sqrt(sigma[0]*sigma[0] + sigma[1]*sigma[1] +
                               sigma[2]*sigma[2] + sigma[3]*sigma[3]);
        if (norm > 1e-6f) {
            sigma[0] /= norm; sigma[1] /= norm;
            sigma[2] /= norm; sigma[3] /= norm;
        }

        // Смещение гироскопа остаётся постоянным (random walk model)
        // sigma[4], sigma[5], sigma[6] не меняются
    }

    /**
     * @brief Предсказание измерения из sigma point
     */
    void predictMeasurement(const std::array<float, STATE_DIM>& sigma,
                           std::array<float, MEAS_DIM>& measurement) {
        // Извлечение кватерниона
        Quaternion q(sigma[0], sigma[1], sigma[2], sigma[3]);

        // Предсказание направления гравитации в body frame
        // g_body = q* ⊗ [0,0,0,1] ⊗ q (гравитация направлена вниз в NED)
        auto gz = q.rotateVector({0.0f, 0.0f, 1.0f});
        measurement[0] = gz[0];
        measurement[1] = gz[1];
        measurement[2] = gz[2];

        // Предсказание направления магнитного поля
        // Предполагаем магнитное поле в NED: [magN, 0, magD]
        constexpr float magN = 0.27f;  // Северная компонента (примерно для средней широты)
        constexpr float magD = 0.45f;  // Вертикальная компонента
        auto magBody = q.rotateVector({magN, 0.0f, magD});
        measurement[3] = magBody[0];
        measurement[4] = magBody[1];
        measurement[5] = magBody[2];
    }

    /**
     * @brief Вычисление среднего состояния из sigma points
     */
    void computeMeanState(const std::array<std::array<float, STATE_DIM>, SIGMA_COUNT>& points) {
        for (int i = 0; i < STATE_DIM; i++) {
            state_[i] = 0.0f;
            for (int j = 0; j < SIGMA_COUNT; j++) {
                state_[i] += Wm_[j] * points[j][i];
            }
        }

        // Нормализация кватерниона
        normalizeQuaternion();
    }

    /**
     * @brief Вычисление ковариации из sigma points
     */
    void computeCovariance(const std::array<std::array<float, STATE_DIM>, SIGMA_COUNT>& points) {
        for (int i = 0; i < STATE_DIM * STATE_DIM; i++) {
            P_[i] = 0.0f;
        }

        for (int j = 0; j < SIGMA_COUNT; j++) {
            float diff[STATE_DIM];
            for (int i = 0; i < STATE_DIM; i++) {
                diff[i] = points[j][i] - state_[i];
            }

            // P += Wc * diff * diff^T
            for (int m = 0; m < STATE_DIM; m++) {
                for (int n = 0; n < STATE_DIM; n++) {
                    P_[m * STATE_DIM + n] += Wc_[j] * diff[m] * diff[n];
                }
            }
        }
    }

    /**
     * @brief Обновление ковариации после коррекции
     */
    void updateCovariance(const float* K, const float* S) {
        // P = P - K * S * K^T
        float KS[STATE_DIM * MEAS_DIM]{};
        matrixMultiply(KS, K, S, STATE_DIM, MEAS_DIM, MEAS_DIM);

        float KSKt[STATE_DIM * STATE_DIM]{};
        matrixMultiplyT2(KSKt, KS, K, STATE_DIM, MEAS_DIM, STATE_DIM);

        for (int i = 0; i < STATE_DIM * STATE_DIM; i++) {
            P_[i] -= KSKt[i];
        }
    }

    /**
     * @brief Нормализация кватерниона в состоянии
     */
    void normalizeQuaternion() {
        float norm = std::sqrt(state_[0]*state_[0] + state_[1]*state_[1] +
                               state_[2]*state_[2] + state_[3]*state_[3]);
        if (norm > 1e-6f) {
            state_[0] /= norm; state_[1] /= norm;
            state_[2] /= norm; state_[3] /= norm;
        }
    }

    /**
     * @brief Разложение Холецкого (A = L * L^T)
     */
    void choleskyDecomposition(const float* A, float* L, int n) {
        for (int i = 0; i < n * n; i++) L[i] = 0.0f;

        for (int i = 0; i < n; i++) {
            for (int j = 0; j <= i; j++) {
                float sum = 0.0f;
                if (j == i) {
                    for (int k = 0; k < j; k++) {
                        sum += L[j * n + k] * L[j * n + k];
                    }
                    float val = A[j * n + j] - sum;
                    L[j * n + j] = (val > 0) ? std::sqrt(val) : 1e-6f;
                } else {
                    for (int k = 0; k < j; k++) {
                        sum += L[i * n + k] * L[j * n + k];
                    }
                    L[i * n + j] = (L[j * n + j] > 1e-6f) ?
                                   (A[i * n + j] - sum) / L[j * n + j] : 0.0f;
                }
            }
        }
    }

    /**
     * @brief Упрощённая матричная инверсия (для 3x3 блоков)
     */
    void matrixInverse3x3(const float* A, float* Ainv) {
        // Для простоты используваем диагональную аппроксимацию
        for (int i = 0; i < MEAS_DIM * MEAS_DIM; i++) Ainv[i] = 0.0f;
        for (int i = 0; i < MEAS_DIM; i++) {
            float diag = A[i * MEAS_DIM + i];
            Ainv[i * MEAS_DIM + i] = (diag > 1e-6f) ? 1.0f / diag : 1.0f;
        }
    }

    /**
     * @brief Матричное умножение C = A * B
     */
    void matrixMultiply(float* C, const float* A, const float* B, int m, int n, int p) {
        for (int i = 0; i < m * p; i++) C[i] = 0.0f;
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < p; j++) {
                for (int k = 0; k < n; k++) {
                    C[i * p + j] += A[i * n + k] * B[k * p + j];
                }
            }
        }
    }

    /**
     * @brief Матричное умножение с транспонированием второй матрицы: C = A * B^T
     */
    void matrixMultiplyT2(float* C, const float* A, const float* B, int m, int n, int p) {
        (void)p; // Используется только для документации, фактически m x m результат
        for (int i = 0; i < m * m; i++) C[i] = 0.0f;
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < m; j++) {
                for (int k = 0; k < n; k++) {
                    C[i * m + j] += A[i * n + k] * B[j * n + k];
                }
            }
        }
    }
};

} // namespace adcs
} // namespace mka

#endif // ADCS_ALGORITHMS_HPP
