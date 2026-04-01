/**
 * @file anomaly_detector.hpp
 * @brief ML детектор аномалий для телеметрии
 *
 * Реализует алгоритм Isolation Forest для обнаружения
 * аномалий в многомерных данных телеметрии.
 *
 * Преимущества:
 * - Работает без размеченных данных (unsupervised)
 * - O(n) сложность предсказания
 * - Хорошо работает на высоких размерностях
 */

#ifndef ANOMALY_DETECTOR_HPP
#define ANOMALY_DETECTOR_HPP

#include <cstdint>
#include <cstddef>
#include <array>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>

namespace mka {
namespace ml {

// ============================================================================
// Конфигурация
// ============================================================================

/**
 * @brief Конфигурация детектора аномалий
 */
struct AnomalyDetectorConfig {
    size_t numTrees = 100;          // Количество деревьев
    size_t sampleSize = 256;        // Размер подвыборки для обучения
    size_t maxDepth = 8;            // Максимальная глубина дерева
    float contamination = 0.1f;     // Ожидаемая доля аномалий (0-0.5)
    uint32_t randomSeed = 42;       // Seed для генератора случайных чисел
};

/**
 * @brief Результат детекции аномалии
 */
struct AnomalyResult {
    float anomalyScore = 0.0f;      // Оценка аномальности (0-1, чем больше тем аномальнее)
    bool isAnomaly = false;         // Флаг аномалии
    float threshold = 0.5f;         // Порог принятия решения
    size_t featureCount = 0;        // Количество признаков
};

// ============================================================================
// Изolation Tree
// ============================================================================

/**
 * @brief Узел дерева изоляции
 */
struct IsolationTreeNode {
    bool isLeaf = true;
    size_t featureIndex = 0;        // Индекс признака для разделения
    float splitValue = 0.0f;        // Значение разделения
    size_t leftChild = 0;           // Индекс левого потомка
    size_t rightChild = 0;          // Индекс правого потомка
    size_t size = 0;                // Количество образцов в узле
};

/**
 * @brief Дерево изоляции
 */
class IsolationTree {
public:
    IsolationTree() = default;

    /**
     * @brief Обучение дерева
     * @param data Обучающая выборка [n_samples x n_features]
     * @param maxDepth Максимальная глубина
     * @param rng Генератор случайных чисел
     */
    void fit(const std::vector<std::vector<float>>& data,
             size_t maxDepth,
             std::mt19937& rng);

    /**
     * @brief Вычисление пути до листа для образца
     * @param sample Образец [n_features]
     * @return Длина пути
     */
    float pathLength(const std::vector<float>& sample) const;

    /**
     * @brief Сброс дерева
     */
    void reset();

    /**
     * @brief Проверка обученности
     */
    bool isTrained() const { return !nodes_.empty(); }

private:
    std::vector<IsolationTreeNode> nodes_;

    /**
     * @brief Рекурсивное построение дерева
     */
    size_t buildTree(const std::vector<std::vector<float>>& data,
                     size_t depth,
                     size_t maxDepth,
                     std::mt19937& rng);

    /**
     * @brief Вычисление длины пути рекурсивно
     */
    float pathLengthRecursive(const std::vector<float>& sample,
                              size_t nodeIndex) const;
};

// ============================================================================
// Isolation Forest
// ============================================================================

/**
 * @brief Ансамбль деревьев изоляции для детекции аномалий
 *
 * Алгоритм:
 * 1. Строим ensemble из numTrees деревьев
 * 2. Для каждого дерева:
 *    - Берём случайную подвыборку sampleSize
 *    - Рекурсивно разделяем данные случайными разрезами
 * 3. Аномалии имеют короткие пути до листа
 *
 * Оценка аномальности:
 *   score(x) = 2^(-E[h(x)] / c(n))
 * где:
 *   - E[h(x)] - средняя длина пути по всем деревьям
 *   - c(n) - средняя длина пути в BST с n узлами
 */
class IsolationForest {
public:
    IsolationForest() = default;

    explicit IsolationForest(const AnomalyDetectorConfig& config)
        : config_(config) {}

    /**
     * @brief Обучение детектора
     * @param data Обучающая выборка [n_samples x n_features]
     * @return true если успешно
     */
    bool fit(const std::vector<std::vector<float>>& data);

    /**
     * @brief Предсказание для одного образца
     * @param sample Образец [n_features]
     * @return Результат детекции
     */
    AnomalyResult predict(const std::vector<float>& sample) const;

    /**
     * @brief Предсказание для множества образцов
     * @param samples Образцы [n_samples x n_features]
     * @return Вектор результатов
     */
    std::vector<AnomalyResult> predictBatch(
        const std::vector<std::vector<float>>& samples) const;

    /**
     * @brief Вычисление оценки аномальности
     * @param sample Образец
     * @return Оценка (0-1)
     */
    float anomalyScore(const std::vector<float>& sample) const;

    /**
     * @brief Проверка обученности
     */
    bool isTrained() const {
        return !trees_.empty() && numFeatures_ > 0;
    }

    /**
     * @brief Получить количество признаков
     */
    size_t getNumFeatures() const { return numFeatures_; }

    /**
     * @brief Получить количество деревьев
     */
    size_t getNumTrees() const { return trees_.size(); }

    /**
     * @brief Сброс детектора
     */
    void reset();

    /**
     * @brief Установить порог детекции
     */
    void setThreshold(float threshold) { threshold_ = threshold; }

    /**
     * @brief Получить текущий порог
     */
    float getThreshold() const { return threshold_; }

private:
    AnomalyDetectorConfig config_;
    std::vector<IsolationTree> trees_;
    size_t numFeatures_ = 0;
    float threshold_ = 0.5f;

    // Статистика обучающей выборки
    std::vector<float> featureMin_;
    std::vector<float> featureMax_;
    std::vector<float> featureMean_;
    std::vector<float> featureStd_;

    /**
     * @brief Вычисление порога по contamination
     */
    void computeThreshold(const std::vector<std::vector<float>>& data);

    /**
     * @brief Нормализация признака
     */
    float normalizeFeature(float value, size_t featureIndex) const;

    /**
     * @brief Средняя длина пути в BST с n узлами
     */
    static float c(size_t n);
};

// ============================================================================
// Детектор аномалий для телеметрии
// ============================================================================

/**
 * @brief Детектор аномалий для бортовой телеметрии
 *
 * Предназначен для обнаружения аномального поведения
 * подсистем спутника по данным телеметрии.
 *
 * Пример использования:
 * @code
 * AnomalyDetector detector;
 * detector.configure({100, 256, 8, 0.1f});
 *
 * // Обучение на нормальных данных
 * std::vector<std::vector<float>> trainingData = ...;
 * detector.fit(trainingData);
 *
 * // Детекция в реальном времени
 * std::vector<float> telemetry = {voltage, current, temp, ...};
 * auto result = detector.predict(telemetry);
 * if (result.isAnomaly) {
 *     // Тревога!
 * }
 * @endcode
 */
class AnomalyDetector {
public:
    AnomalyDetector() = default;

    /**
     * @brief Конфигурация детектора
     */
    void configure(const AnomalyDetectorConfig& config);

    /**
     * @brief Обучение на исторических данных
     */
    bool fit(const std::vector<std::vector<float>>& data);

    /**
     * @brief Добавление одного образца для онлайн обучения
     */
    void partialFit(const std::vector<float>& sample);

    /**
     * @brief Детекция аномалии
     */
    AnomalyResult detect(const std::vector<float>& telemetry) const;

    /**
     * @brief Пакетная детекция
     */
    std::vector<AnomalyResult> detectBatch(
        const std::vector<std::vector<float>>& telemetry) const;

    /**
     * @brief Сброс детектора
     */
    void reset();

    /**
     * @brief Проверка готовности
     */
    bool isReady() const { return forest_.isTrained(); }

    /**
     * @brief Получить статистику
     */
    struct Statistics {
        size_t totalSamples = 0;
        size_t anomalyCount = 0;
        float lastScore = 0.0f;
        size_t featureCount = 0;
    };

    Statistics getStatistics() const { return stats_; }

private:
    IsolationForest forest_;
    Statistics stats_;
    std::vector<std::vector<float>> buffer_;  // Буфер для онлайн обучения
    static constexpr size_t MAX_BUFFER_SIZE = 1000;
};

} // namespace ml
} // namespace mka

#endif // ANOMALY_DETECTOR_HPP
