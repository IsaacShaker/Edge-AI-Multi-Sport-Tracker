// Full implementation of Interacting Multiple Model estimator
#include "../include/estimation/imm_estimator.h"
#include "../include/factories/estimator_factory.h"
#include <cmath>
#include <iostream>
#include <algorithm>

namespace tracker {

bool IMMEstimator::initialize(const Config& config) {
    config_ = static_cast<const EstimatorConfig&>(config);
    initialized_ = false;
    
    // Create sub-models
    if (config_.model_types.empty()) {
        // Default: CV + CA models
        EstimatorConfig cv_config = config_;
        cv_config.estimator_type = "kalman_cv";
        auto cv_model = EstimatorFactory::create("kalman_cv", cv_config);
        if (!cv_model) {
            std::cerr << "[IMMEstimator] Failed to create CV model" << std::endl;
            return false;
        }
        models_.push_back(cv_model);
        
        EstimatorConfig ca_config = config_;
        ca_config.estimator_type = "kalman_ca";
        auto ca_model = EstimatorFactory::create("kalman_ca", ca_config);
        if (!ca_model) {
            std::cerr << "[IMMEstimator] Failed to create CA model" << std::endl;
            return false;
        }
        models_.push_back(ca_model);
    } else {
        // Create models from config
        for (const auto& model_type : config_.model_types) {
            EstimatorConfig model_config = config_;
            model_config.estimator_type = model_type;
            auto model = EstimatorFactory::create(model_type, model_config);
            if (!model) {
                std::cerr << "[IMMEstimator] Failed to create model: " << model_type << std::endl;
                return false;
            }
            models_.push_back(model);
        }
    }
    
    // Initialize probabilities
    int N = models_.size();
    if (config_.initial_probabilities.empty()) {
        // Equal probabilities
        model_probs_.resize(N, 1.0f / N);
    } else {
        model_probs_ = config_.initial_probabilities;
        // Normalize
        float sum = 0.0f;
        for (float p : model_probs_) sum += p;
        for (float& p : model_probs_) p /= sum;
    }
    
    // Initialize transition probability matrix
    float stay_prob = config_.transition_probability;
    float switch_prob = (1.0f - stay_prob) / (N - 1);
    
    tpm_.resize(N);
    for (int i = 0; i < N; ++i) {
        tpm_[i].resize(N, switch_prob);
        tpm_[i][i] = stay_prob;
    }
    
    std::cout << "[IMMEstimator] Initialized with " << N << " models" << std::endl;
    for (size_t i = 0; i < models_.size(); ++i) {
        std::cout << "  Model " << i << ": " << models_[i]->getType() 
                  << " (prob=" << model_probs_[i] << ")" << std::endl;
    }
    
    return true;
}

void IMMEstimator::initializeState(const Detection& detection) {
    // Initialize all sub-models with the same detection
    for (auto& model : models_) {
        model->initializeState(detection);
    }
    
    initialized_ = true;
    current_state_ = blendEstimates();
}

EstimatedState IMMEstimator::predict(float dt) {
    if (!initialized_) {
        return EstimatedState();
    }
    
    // Predict with all models
    for (auto& model : models_) {
        model->predict(dt);
    }
    
    // Blend predictions
    current_state_ = blendEstimates();
    return current_state_;
}

EstimatedState IMMEstimator::update(const Detection& detection) {
    if (!initialized_) {
        initializeState(detection);
        return current_state_;
    }
    
    // Update all models with the measurement
    std::vector<float> likelihoods(models_.size());
    
    for (size_t i = 0; i < models_.size(); ++i) {
        models_[i]->update(detection);
        likelihoods[i] = computeLikelihood(models_[i], detection);
    }
    
    // Update model probabilities based on likelihoods
    updateModelProbabilities(likelihoods);
    
    // Blend estimates from all models
    current_state_ = blendEstimates();
    
    return current_state_;
}

EstimatedState IMMEstimator::getState() const {
    return current_state_;
}

void IMMEstimator::reset() {
    initialized_ = false;
    current_state_ = EstimatedState();
    
    // Reset all sub-models
    for (auto& model : models_) {
        model->reset();
    }
    
    // Reset probabilities to uniform
    int N = models_.size();
    std::fill(model_probs_.begin(), model_probs_.end(), 1.0f / N);
}

float IMMEstimator::computeLikelihood(
    const StateEstimatorPtr& model,
    const Detection& detection
) {
    // Compute likelihood based on innovation (measurement residual)
    auto state = model->getState();
    
    // Innovation: difference between measurement and prediction
    float dx = detection.center.x - state.position.x;
    float dy = detection.center.y - state.position.y;
    float dr = detection.radius - 20.0f;  // Approximate expected radius
    
    // Innovation covariance (simplified: use measurement noise)
    float var_pos = config_.measurement_noise_pos;
    float var_r = config_.measurement_noise_radius;
    
    // Mahalanobis distance
    float mahal_dist = (dx * dx + dy * dy) / var_pos + (dr * dr) / var_r;
    
    // Gaussian likelihood
    // L = exp(-0.5 * mahal_dist) / sqrt((2*pi)^k * det(S))
    // For simplicity, ignore normalization constant
    float likelihood = std::exp(-0.5f * mahal_dist);
    
    // Avoid zero likelihood
    return std::max(likelihood, 1e-10f);
}

void IMMEstimator::updateModelProbabilities(const std::vector<float>& likelihoods) {
    int N = models_.size();
    std::vector<float> predicted_probs(N);
    
    // Prediction step: mu_predicted[j] = sum_i(TPM[i][j] * mu[i])
    for (int j = 0; j < N; ++j) {
        predicted_probs[j] = 0.0f;
        for (int i = 0; i < N; ++i) {
            predicted_probs[j] += tpm_[i][j] * model_probs_[i];
        }
    }
    
    // Update step: mu[i] = likelihood[i] * predicted_probs[i]
    std::vector<float> new_probs(N);
    float total = 0.0f;
    
    for (int i = 0; i < N; ++i) {
        new_probs[i] = likelihoods[i] * predicted_probs[i];
        total += new_probs[i];
    }
    
    // Normalize
    if (total > 1e-10f) {
        for (int i = 0; i < N; ++i) {
            model_probs_[i] = new_probs[i] / total;
        }
    }
    
    // Debug output (optional)
    static int update_count = 0;
    if (++update_count % 30 == 0) {  // Every 30 updates
        std::cout << "[IMM] Model probabilities: ";
        for (size_t i = 0; i < model_probs_.size(); ++i) {
            std::cout << models_[i]->getType() << "=" << model_probs_[i] << " ";
        }
        std::cout << std::endl;
    }
}

EstimatedState IMMEstimator::blendEstimates() {
    EstimatedState blended;
    
    // Weighted average of all model estimates
    float total_prob = 0.0f;
    
    for (size_t i = 0; i < models_.size(); ++i) {
        auto state = models_[i]->getState();
        float prob = model_probs_[i];
        
        blended.position.x += prob * state.position.x;
        blended.position.y += prob * state.position.y;
        blended.position.z += prob * state.position.z;
        
        blended.velocity.vx += prob * state.velocity.vx;
        blended.velocity.vy += prob * state.velocity.vy;
        blended.velocity.vz += prob * state.velocity.vz;
        
        blended.acceleration.ax += prob * state.acceleration.ax;
        blended.acceleration.ay += prob * state.acceleration.ay;
        blended.acceleration.az += prob * state.acceleration.az;
        
        blended.confidence += prob * state.confidence;
        
        total_prob += prob;
    }
    
    // Normalize if needed (should already be normalized)
    if (total_prob > 0.0f && std::abs(total_prob - 1.0f) > 1e-6f) {
        blended.position.x /= total_prob;
        blended.position.y /= total_prob;
        blended.position.z /= total_prob;
        blended.velocity.vx /= total_prob;
        blended.velocity.vy /= total_prob;
        blended.velocity.vz /= total_prob;
        blended.acceleration.ax /= total_prob;
        blended.acceleration.ay /= total_prob;
        blended.acceleration.az /= total_prob;
        blended.confidence /= total_prob;
    }
    
    blended.timestamp_ms = 0;  // TODO: Add proper timestamp
    
    return blended;
}

} // namespace tracker
