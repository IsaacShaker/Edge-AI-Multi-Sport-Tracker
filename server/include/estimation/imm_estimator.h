#pragma once

#include "../interfaces/i_state_estimator.h"
#include "../factories/estimator_factory.h"
#include "kalman_filter_cv.h"
#include "kalman_filter_ca.h"
#include <vector>
#include <memory>

namespace tracker {

/**
 * @brief Interacting Multiple Model (IMM) estimator
 * 
 * Runs multiple Kalman filters in parallel and blends their outputs
 * based on likelihood. Adapts to different motion patterns automatically.
 * 
 * Typical configuration: CV model + CA model
 * - CV handles linear trajectories
 * - CA handles ballistic/curved trajectories
 */
class IMMEstimator : public IStateEstimator {
public:
    IMMEstimator() = default;
    ~IMMEstimator() override = default;
    
    bool initialize(const Config& config) override;
    void initializeState(const Detection& detection) override;
    EstimatedState predict(float dt) override;
    EstimatedState update(const Detection& detection) override;
    EstimatedState getState() const override;
    bool isInitialized() const override { return initialized_; }
    void reset() override;
    std::string getType() const override { return "imm"; }
    
    /**
     * @brief Get current model probabilities
     * @return Vector of probabilities for each model
     */
    std::vector<float> getModelProbabilities() const { return model_probs_; }

private:
    EstimatorConfig config_;
    bool initialized_;
    
    // Sub-models (e.g., CV and CA)
    std::vector<StateEstimatorPtr> models_;
    
    // IMM parameters
    std::vector<float> model_probs_;           // Current model probabilities
    std::vector<std::vector<float>> tpm_;      // Transition probability matrix
    
    EstimatedState current_state_;
    
    /**
     * @brief Compute likelihood of a detection given a model's prediction
     */
    float computeLikelihood(
        const StateEstimatorPtr& model,
        const Detection& detection
    );
    
    /**
     * @brief Update model probabilities based on likelihoods
     */
    void updateModelProbabilities(const std::vector<float>& likelihoods);
    
    /**
     * @brief Blend estimates from all models based on probabilities
     */
    EstimatedState blendEstimates();
};

} // namespace tracker
