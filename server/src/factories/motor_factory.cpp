#include "../include/factories/motor_factory.h"
#include "../include/motor/simplefoc_controller.h"
#include "../include/motor/mock_controller.h"
#include <iostream>

namespace tracker {

// Static member initialization
std::map<std::string, std::function<MotorControllerPtr(const MotorConfig&)>>
    MotorFactory::creators_;

MotorControllerPtr MotorFactory::create(
    const std::string& type,
    const MotorConfig& config
) {
    if (creators_.empty()) {
        registerBuiltinCreators();
    }
    
    auto it = creators_.find(type);
    if (it == creators_.end()) {
        std::cerr << "Unknown motor controller type: " << type << std::endl;
        return nullptr;
    }
    
    return it->second(config);
}

void MotorFactory::registerCreator(
    const std::string& type,
    std::function<MotorControllerPtr(const MotorConfig&)> creator
) {
    creators_[type] = creator;
}

std::vector<std::string> MotorFactory::getAvailableTypes() {
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

void MotorFactory::registerBuiltinCreators() {
    // Register SimpleFOC controller
    registerCreator("simplefoc", [](const MotorConfig& config) -> MotorControllerPtr {
        auto controller = std::make_shared<SimpleFOCController>();
        if (controller->initialize(config)) {
            return controller;
        }
        return nullptr;
    });
    
    // Register mock controller
    registerCreator("mock", [](const MotorConfig& config) -> MotorControllerPtr {
        auto controller = std::make_shared<MockController>();
        if (controller->initialize(config)) {
            return controller;
        }
        return nullptr;
    });
}

} // namespace tracker
