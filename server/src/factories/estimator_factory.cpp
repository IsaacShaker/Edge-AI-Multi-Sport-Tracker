#include "../include/factories/estimator_factory.h"
#include "../include/estimation/kalman_filter_cv.h"
#include "../include/estimation/kalman_filter_ca.h"
#include "../include/estimation/imm_estimator.h"
#include <iostream>

namespace tracker {

// Static member initialization
std::map<std::string, std::function<StateEstimatorPtr(const EstimatorConfig&)>>
    EstimatorFactory::creators_;

StateEstimatorPtr EstimatorFactory::create(
    const std::string& type,
    const EstimatorConfig& config
) {
    if (creators_.empty()) {
        registerBuiltinCreators();
    }
    
    auto it = creators_.find(type);
    if (it == creators_.end()) {
        std::cerr << "Unknown estimator type: " << type << std::endl;
        return nullptr;
    }
    
    return it->second(config);
}

void EstimatorFactory::registerCreator(
    const std::string& type,
    std::function<StateEstimatorPtr(const EstimatorConfig&)> creator
) {
    creators_[type] = creator;
}

std::vector<std::string> EstimatorFactory::getAvailableTypes() {
    if (creators_.empty()) {
        registerBuiltinCreators();
    }
    
    std::vector<std::string> types;
    types.reserve(creators_.size());
    for (const auto& pair : creators_) {
        types.push_back(pair.first);
    }
    return types;
}

void EstimatorFactory::registerBuiltinCreators() {
    // Register Kalman CV
    registerCreator("kalman_cv", [](const EstimatorConfig& config) -> StateEstimatorPtr {
        auto estimator = std::make_shared<KalmanFilterCV>();
        if (estimator->initialize(config)) {
            return estimator;
        }
        return nullptr;
    });
    
    // Register Kalman CA
    registerCreator("kalman_ca", [](const EstimatorConfig& config) -> StateEstimatorPtr {
        auto estimator = std::make_shared<KalmanFilterCA>();
        if (estimator->initialize(config)) {
            return estimator;
        }
        return nullptr;
    });
    
    // Register IMM
    registerCreator("imm", [](const EstimatorConfig& config) -> StateEstimatorPtr {
        auto estimator = std::make_shared<IMMEstimator>();
        if (estimator->initialize(config)) {
            return estimator;
        }
        return nullptr;
    });
}

} // namespace tracker
