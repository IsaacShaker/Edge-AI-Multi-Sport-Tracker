#pragma once

#include <memory>
#include <vector>
#include <string>

namespace tracker {

// ============================================================================
// Common Data Structures
// ============================================================================

/**
 * @brief 2D point with floating point coordinates
 */
struct Point2D {
    float x;
    float y;
    
    Point2D() : x(0.0f), y(0.0f) {}
    Point2D(float x_, float y_) : x(x_), y(y_) {}
};

/**
 * @brief 3D point with floating point coordinates
 */
struct Point3D {
    float x;
    float y;
    float z;
    
    Point3D() : x(0.0f), y(0.0f), z(0.0f) {}
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

/**
 * @brief Velocity vector (2D or 3D)
 */
struct Velocity {
    float vx;
    float vy;
    float vz;
    
    Velocity() : vx(0.0f), vy(0.0f), vz(0.0f) {}
    Velocity(float vx_, float vy_, float vz_ = 0.0f) 
        : vx(vx_), vy(vy_), vz(vz_) {}
};

/**
 * @brief Acceleration vector
 */
struct Acceleration {
    float ax;
    float ay;
    float az;
    
    Acceleration() : ax(0.0f), ay(0.0f), az(0.0f) {}
    Acceleration(float ax_, float ay_, float az_ = 0.0f) 
        : ax(ax_), ay(ay_), az(az_) {}
};

/**
 * @brief Bounding box for detected objects
 */
struct BoundingBox {
    float x;      // Center x
    float y;      // Center y
    float width;
    float height;
    
    BoundingBox() : x(0.0f), y(0.0f), width(0.0f), height(0.0f) {}
    BoundingBox(float x_, float y_, float w, float h) 
        : x(x_), y(y_), width(w), height(h) {}
};

/**
 * @brief Detection result from vision system
 */
struct Detection {
    BoundingBox bbox;
    Point2D center;
    float radius;        // For circular objects (ball)
    float confidence;
    std::string label;
    int64_t timestamp_ms;
    bool has_bbox;       // Whether bounding box is valid
    
    Detection() : radius(0.0f), confidence(0.0f), timestamp_ms(0), has_bbox(false) {}
};

/**
 * @brief Estimated state from estimation system
 */
struct EstimatedState {
    Point3D position;        // 3D position (z is distance/depth)
    Velocity velocity;
    Acceleration acceleration;
    float confidence;
    int64_t timestamp_ms;
    
    EstimatedState() : confidence(0.0f), timestamp_ms(0) {}
};

/**
 * @brief Motor/Gimbal angles
 */
struct GimbalAngles {
    float pan;      // Yaw/horizontal angle (radians)
    float tilt;     // Pitch/vertical angle (radians)
    
    GimbalAngles() : pan(0.0f), tilt(0.0f) {}
    GimbalAngles(float p, float t) : pan(p), tilt(t) {}
};

/**
 * @brief Configuration structure base
 */
struct Config {
    virtual ~Config() = default;
};

} // namespace tracker
