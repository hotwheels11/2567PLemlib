#pragma once
#include "Sensor.hpp"
#include "pros/distance.hpp"
#include <cmath>
#include <algorithm>

class DistanceSensor : public Sensor {
private:
    pros::Distance sensor;
    float sensor_x_offset;
    float sensor_y_offset;
    float sensor_angle_offset;
    int sensor_id;  // Unique ID for this sensor
    
    float measured_distance = 0.0f;
    bool valid = false;
    float std_dev = 0.0f;
    float tuning_constant;
    
    static constexpr float FIELD_WIDTH = 144.0f;
    static constexpr float FIELD_HEIGHT = 144.0f;
    
    // Obstacle definition
    struct Obstacle {
        float center_x;
        float center_y;
        float width;
        float height;
        float rotation;  // Rotation in radians (0 = aligned with field axes)
        std::vector<int> visible_to_sensors;
    };
    
    static inline std::vector<Obstacle> obstacles;
    static inline int next_sensor_id = 0;
    
    float expectedDistanceToWall(float x, float y, float angle) {
        float min_dist = 1000.0f;
        
        float cos_a = cos(angle);
        float sin_a = sin(angle);
        
        // Check field walls
        if (cos_a > 0.001f) {
            float d = (FIELD_WIDTH / 2.0f - x) / cos_a;
            if (d > 0) min_dist = std::min(min_dist, d);
        }
        
        if (cos_a < -0.001f) {
            float d = -(FIELD_WIDTH / 2.0f + x) / cos_a;
            if (d > 0) min_dist = std::min(min_dist, d);
        }
        
        if (sin_a > 0.001f) {
            float d = (FIELD_HEIGHT / 2.0f - y) / sin_a;
            if (d > 0) min_dist = std::min(min_dist, d);
        }
        
        if (sin_a < -0.001f) {
            float d = -(FIELD_HEIGHT / 2.0f + y) / sin_a;
            if (d > 0) min_dist = std::min(min_dist, d);
        }
        
        // Check obstacles - only if this sensor can see them
        for (const auto& obs : obstacles) {
            // Skip if this sensor can't see this obstacle
            if (!obs.visible_to_sensors.empty() && 
                std::find(obs.visible_to_sensors.begin(), obs.visible_to_sensors.end(), sensor_id) == obs.visible_to_sensors.end()) {
                continue;
            }
            
            float half_w = obs.width / 2.0f;
            float half_h = obs.height / 2.0f;
            
            // Calculate rotated corners
            float cos_rot = cos(obs.rotation);
            float sin_rot = sin(obs.rotation);
            
            // Four corners in local coordinate system, then rotate and translate
            float local_corners[4][2] = {
                {-half_w, -half_h},  // Bottom-left
                { half_w, -half_h},  // Bottom-right
                { half_w,  half_h},  // Top-right
                {-half_w,  half_h}   // Top-left
            };
            
            float corners[4][2];
            for (int i = 0; i < 4; i++) {
                // Rotate and translate to global position
                corners[i][0] = obs.center_x + local_corners[i][0] * cos_rot - local_corners[i][1] * sin_rot;
                corners[i][1] = obs.center_y + local_corners[i][0] * sin_rot + local_corners[i][1] * cos_rot;
            }
            
            // Four edges of the rotated rectangle
            for (int i = 0; i < 4; i++) {
                int next = (i + 1) % 4;
                float x1 = corners[i][0], y1 = corners[i][1];
                float x2 = corners[next][0], y2 = corners[next][1];
                
                float dx = x2 - x1;
                float dy = y2 - y1;
                float det = cos_a * dy - sin_a * dx;
                
                if (fabs(det) > 1e-6f) {
                    float t = ((x1 - x) * dy - (y1 - y) * dx) / det;
                    float u = ((x1 - x) * sin_a - (y1 - y) * cos_a) / det;
                    
                    if (t > 0 && u >= 0 && u <= 1) {
                        min_dist = std::min(min_dist, t);
                    }
                }
            }
        }
        
        return min_dist;
    }
    
    float cheapNormPdf(float x) {
        const float a = 0.3989422804f;
        const float e = 0.59422804f;
        return a / (1.0f + e * x * x * x * x);
    }
    
public:
    DistanceSensor(int port, float x_offset, float y_offset, float angle_offset, float tuning = 1.0f)
        : sensor(port),
          sensor_x_offset(x_offset),
          sensor_y_offset(y_offset),
          sensor_angle_offset(angle_offset),
          tuning_constant(tuning),
          sensor_id(next_sensor_id++) {}
    
    void update() override {
        int reading_mm = sensor.get();
        valid = (reading_mm < 9999 && sensor.get_confidence() > 30);
        
        if (valid) {
            measured_distance = tuning_constant * reading_mm * 0.0393701f;
            std_dev = 0.2f * measured_distance / sqrt(sensor.get_confidence() / 64.0f);
        }
    }
    
    std::optional<double> p(float x, float y, float theta) override {
        if (!valid) return std::nullopt;
        
        float sensor_global_x = x + sensor_x_offset * cos(theta) - sensor_y_offset * sin(theta);
        float sensor_global_y = y + sensor_x_offset * sin(theta) + sensor_y_offset * cos(theta);
        float sensor_global_angle = theta + sensor_angle_offset;
        
        float expected = expectedDistanceToWall(sensor_global_x, sensor_global_y, sensor_global_angle);
        float diff = (expected - measured_distance) / std_dev;
        return cheapNormPdf(diff);
    }
    
    int getSensorId() const { return sensor_id; }
    float getMeasuredDistance() const { return measured_distance; }
    bool isValid() const { return valid; }
    float getSensorAngleOffset() const { return sensor_angle_offset; }
    
    // Static methods to manage obstacles
    static void addObstacle(float center_x, float center_y, float width, float height, 
                           float rotation_degrees = 0.0f, std::vector<int> visible_to = {}) {
        float rotation_rad = rotation_degrees * M_PI / 180.0f;
        obstacles.push_back({center_x, center_y, width, height, rotation_rad, visible_to});
    }
    
    static void clearObstacles() {
        obstacles.clear();
    }
};
