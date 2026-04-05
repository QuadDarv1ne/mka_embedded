/**
 * @file anomaly_detector.cpp
 * @brief Реализация ML детектора аномалий
 */

#include "anomaly_detector.hpp"
#include <limits>
#include <numeric>

namespace mka {
namespace ml {

// ============================================================================
// Isolation Tree реализация
// ============================================================================

void IsolationTree::fit(const std::vector<std::vector<float>>& data,
                        size_t maxDepth,
                        std::mt19937& rng) {
    nodes_.clear();
    nodes_.reserve(data.size() * 2);

    if (data.empty()) return;

    buildTree(data, 0, maxDepth, rng);
}

float IsolationTree::pathLength(const std::vector<float>& sample) const {
    if (nodes_.empty()) return 0.0f;
    return pathLengthRecursive(sample, 0);
}

void IsolationTree::reset() {
    nodes_.clear();
}

size_t IsolationTree::buildTree(const std::vector<std::vector<float>>& data,
                                 size_t depth,
                                 size_t maxDepth,
                                 std::mt19937& rng) {
    size_t nodeIndex = nodes_.size();
    IsolationTreeNode node;
    node.size = data.size();

    // Условия остановки: достигнута глубина или один образец
    if (depth >= maxDepth || data.size() <= 1) {
        node.isLeaf = true;
        nodes_.push_back(node);
        return nodeIndex;
    }

    // Выбор случайного признака
    std::uniform_int_distribution<size_t> featureDist(0, data[0].size() - 1);
    size_t featureIndex = featureDist(rng);

    // Поиск мин/макс для выбранного признака
    float minVal = std::numeric_limits<float>::max();
    float maxVal = std::numeric_limits<float>::lowest();

    for (const auto& sample : data) {
        float val = sample[featureIndex];
        minVal = std::min(minVal, val);
        maxVal = std::max(maxVal, val);
    }

    // Если все значения одинаковы — лист
    if (minVal == maxVal) {
        node.isLeaf = true;
        nodes_.push_back(node);
        return nodeIndex;
    }

    // Случайное значение разделения
    std::uniform_real_distribution<float> splitDist(minVal, maxVal);
    float splitValue = splitDist(rng);

    // Разделение данных
    std::vector<std::vector<float>> leftData, rightData;
    for (const auto& sample : data) {
        if (sample[featureIndex] < splitValue) {
            leftData.push_back(sample);
        } else {
            rightData.push_back(sample);
        }
    }

    // Если одно из подмножеств пусто — лист
    if (leftData.empty() || rightData.empty()) {
        node.isLeaf = true;
        nodes_.push_back(node);
        return nodeIndex;
    }

    // Сохранение узла
    node.isLeaf = false;
    node.featureIndex = featureIndex;
    node.splitValue = splitValue;

    // Рекурсивное построение поддеревьев
    node.leftChild = buildTree(leftData, depth + 1, maxDepth, rng);
    node.rightChild = buildTree(rightData, depth + 1, maxDepth, rng);

    nodes_[nodeIndex] = node;
    return nodeIndex;
}

float IsolationTree::pathLengthRecursive(const std::vector<float>& sample,
                                          size_t nodeIndex) const {
    const auto& node = nodes_[nodeIndex];

    if (node.isLeaf) {
        // Коррекция на размер листа
        return static_cast<float>(node.size) > 1 ?
               IsolationForest::c(node.size) : 0.0f;
    }

    if (sample[node.featureIndex] < node.splitValue) {
        return 1.0f + pathLengthRecursive(sample, node.leftChild);
    } else {
        return 1.0f + pathLengthRecursive(sample, node.rightChild);
    }
}

// ============================================================================
// Isolation Forest реализация
// ============================================================================

bool IsolationForest::fit(const std::vector<std::vector<float>>& data) {
    if (data.empty() || data[0].empty()) return false;

    reset();
    numFeatures_ = data[0].size();

    // Вычисление статистики признаков
    featureMin_.assign(numFeatures_, std::numeric_limits<float>::max());
    featureMax_.assign(numFeatures_, std::numeric_limits<float>::lowest());
    featureMean_.assign(numFeatures_, 0.0f);

    for (const auto& sample : data) {
        for (size_t i = 0; i < numFeatures_; i++) {
            featureMin_[i] = std::min(featureMin_[i], sample[i]);
            featureMax_[i] = std::max(featureMax_[i], sample[i]);
            featureMean_[i] += sample[i];
        }
    }

    for (size_t i = 0; i < numFeatures_; i++) {
        featureMean_[i] /= data.size();
    }

    // Вычисление стандартного отклонения
    featureStd_.assign(numFeatures_, 1.0f);
    for (const auto& sample : data) {
        for (size_t i = 0; i < numFeatures_; i++) {
            float diff = sample[i] - featureMean_[i];
            featureStd_[i] += diff * diff;
        }
    }
    for (size_t i = 0; i < numFeatures_; i++) {
        featureStd_[i] = std::sqrt(featureStd_[i] / data.size());
        if (featureStd_[i] < 1e-6f) featureStd_[i] = 1.0f;
    }

    // Обучение деревьев
    trees_.resize(config_.numTrees);
    std::mt19937 rng(config_.randomSeed);

    size_t sampleSize = std::min(config_.sampleSize, data.size());

    for (auto& tree : trees_) {
        // Случайная подвыборка
        std::vector<size_t> indices(data.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::shuffle(indices.begin(), indices.end(), rng);

        std::vector<std::vector<float>> subset;
        subset.reserve(sampleSize);
        for (size_t i = 0; i < sampleSize; i++) {
            subset.push_back(data[indices[i]]);
        }

        tree.fit(subset, config_.maxDepth, rng);
    }

    // Вычисление порога
    computeThreshold(data);

    return true;
}

AnomalyResult IsolationForest::predict(const std::vector<float>& sample) const {
    AnomalyResult result;
    result.featureCount = sample.size();
    result.threshold = threshold_;

    if (!isTrained()) {
        return result;
    }

    result.anomalyScore = anomalyScore(sample);
    result.isAnomaly = result.anomalyScore > threshold_;

    return result;
}

std::vector<AnomalyResult> IsolationForest::predictBatch(
    const std::vector<std::vector<float>>& samples) const {
    std::vector<AnomalyResult> results;
    results.reserve(samples.size());

    for (const auto& sample : samples) {
        results.push_back(predict(sample));
    }

    return results;
}

float IsolationForest::anomalyScore(const std::vector<float>& sample) const {
    if (!isTrained() || sample.size() != numFeatures_) {
        return 0.0f;
    }

    // Вычисление средней длины пути
    float avgPathLength = 0.0f;
    for (const auto& tree : trees_) {
        avgPathLength += tree.pathLength(sample);
    }
    avgPathLength /= trees_.size();

    // Оценка аномальности: score = 2^(-E[h(x)] / c(n))
    float c = IsolationForest::c(config_.sampleSize);
    float score = std::pow(2.0f, -avgPathLength / c);

    return score;
}

void IsolationForest::reset() {
    trees_.clear();
    numFeatures_ = 0;
    featureMin_.clear();
    featureMax_.clear();
    featureMean_.clear();
    featureStd_.clear();
}

void IsolationForest::computeThreshold(const std::vector<std::vector<float>>& data) {
    // Вычисление оценок для всех образцов
    std::vector<float> scores;
    scores.reserve(data.size());

    for (const auto& sample : data) {
        scores.push_back(anomalyScore(sample));
    }

    // Сортировка
    std::sort(scores.begin(), scores.end());

    // Порог по процентилю
    size_t thresholdIndex = static_cast<size_t>(
        (1.0f - config_.contamination) * data.size());
    thresholdIndex = std::min(thresholdIndex, scores.size() - 1);
    threshold_ = scores[thresholdIndex];
}

float IsolationForest::normalizeFeature(float value, size_t featureIndex) const {
    if (featureIndex >= numFeatures_) return value;

    // Z-score нормализация
    return (value - featureMean_[featureIndex]) / featureStd_[featureIndex];
}

float IsolationForest::c(size_t n) {
    // Средняя длина пути в BST с n узлами
    if (n <= 1) return 0.0f;
    if (n == 2) return 1.0f;

    float lnN = std::log(static_cast<float>(n));
    float euler = 0.5772156649f;  // Константа Эйлера-Маскерони

    return 2.0f * (lnN + euler) - 2.0f * (n - 1) / n;
}

// ============================================================================
// AnomalyDetector реализация
// ============================================================================

void AnomalyDetector::configure(const AnomalyDetectorConfig& config) {
    config_ = config;
    forest_ = IsolationForest(config);
}

bool AnomalyDetector::fit(const std::vector<std::vector<float>>& data) {
    stats_.totalSamples = data.size();
    stats_.featureCount = data.empty() ? 0 : data[0].size();

    bool success = forest_.fit(data);

    return success;
}

void AnomalyDetector::partialFit(const std::vector<float>& sample) {
    buffer_.push_back(sample);
    stats_.totalSamples++;

    // Переобучение при заполнении буфера
    if (buffer_.size() >= MAX_BUFFER_SIZE) {
        bool success = fit(buffer_);
        // Очищаем буфер независимо от результата, чтобы предотвратить переполнение
        if (success) {
            buffer_.clear();
        } else {
            // При ошибке оставляем последние MAX_BUFFER_SIZE/2 образцов
            if (buffer_.size() > MAX_BUFFER_SIZE / 2) {
                buffer_.erase(buffer_.begin(),
                             buffer_.begin() + (buffer_.size() - MAX_BUFFER_SIZE / 2));
            }
        }
    }
}

AnomalyResult AnomalyDetector::detect(const std::vector<float>& telemetry) const {
    AnomalyResult result = forest_.predict(telemetry);

    stats_.lastScore = result.anomalyScore;
    if (result.isAnomaly) {
        stats_.anomalyCount++;
    }

    return result;
}

std::vector<AnomalyResult> AnomalyDetector::detectBatch(
    const std::vector<std::vector<float>>& telemetry) const {
    return forest_.predictBatch(telemetry);
}

void AnomalyDetector::reset() {
    forest_.reset();
    stats_ = Statistics{};
    buffer_.clear();
}

} // namespace ml
} // namespace mka
