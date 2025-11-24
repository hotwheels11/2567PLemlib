#pragma once
#include "MCL/DistanceSensor.hpp"
#include "Particle.hpp"
#include "Sensor.hpp"
#include "api.h"
#include <array>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>

template<size_t NUM_PARTICLES>
class ParticleFilter {
private:
    std::array<Particle, NUM_PARTICLES> particles;
    std::array<Particle, NUM_PARTICLES> old_particles;
    std::ranlux24_base rng;
    
    std::vector<Sensor*> sensors;
    
    float est_x = 0.0f;
    float est_y = 0.0f;
    float est_theta = 0.0f;
    
    float distance_since_update = 0.0f;
    uint32_t last_update_time = 0;
    
    static constexpr float UPDATE_DISTANCE_THRESHOLD = 2.0f;
    static constexpr uint32_t UPDATE_TIME_THRESHOLD = 2000;
    static constexpr float ODOM_NOISE = 0.05f;
    static constexpr float ANGLE_NOISE = 0.01f;
    static constexpr float FIELD_WIDTH = 144.0f;
    static constexpr float FIELD_HEIGHT = 144.0f;
    
    bool outOfField(float x, float y) {
        return x > FIELD_WIDTH / 2.0f || x < -FIELD_WIDTH / 2.0f ||
               y > FIELD_HEIGHT / 2.0f || y < -FIELD_HEIGHT / 2.0f;
    }
    
public:
    ParticleFilter() : rng(std::random_device{}()) {
        for (auto& p : particles) {
            p.x = 0.0f;
            p.y = 0.0f;
            p.theta = 0.0f;
            p.weight = 1.0 / NUM_PARTICLES;
        }
    }
    
    void addSensor(Sensor* sensor) {
        sensors.push_back(sensor);
    }
    
    void initNormal(float mean_x, float mean_y, float mean_theta, float std_x, float std_y, float std_theta) {
        std::normal_distribution<float> dist_x(mean_x, std_x);
        std::normal_distribution<float> dist_y(mean_y, std_y);
        std::normal_distribution<float> dist_theta(mean_theta, std_theta);
        
        for (auto& p : particles) {
            p.x = dist_x(rng);
            p.y = dist_y(rng);
            p.theta = dist_theta(rng);
            p.weight = 1.0 / NUM_PARTICLES;
        }
        
        est_x = mean_x;
        est_y = mean_y;
        est_theta = mean_theta;
    }
    
    void initUniform() {
        std::uniform_real_distribution<float> dist_x(-FIELD_WIDTH / 2.0f, FIELD_WIDTH / 2.0f);
        std::uniform_real_distribution<float> dist_y(-FIELD_HEIGHT / 2.0f, FIELD_HEIGHT / 2.0f);
        std::uniform_real_distribution<float> dist_theta(-M_PI, M_PI);
        
        for (auto& p : particles) {
            p.x = dist_x(rng);
            p.y = dist_y(rng);
            p.theta = dist_theta(rng);
            p.weight = 1.0 / NUM_PARTICLES;
        }
    }
    
    bool initialLocalization(float imu_heading) {
        for (auto sensor : sensors) {
            sensor->update();
        }
        
        struct Candidate {
            float x, y, score;
        };
        std::vector<Candidate> candidates;
        
        for (float x = -FIELD_WIDTH / 2.0f + 3.0f; x < FIELD_WIDTH / 2.0f; x += 3.0f) {
            for (float y = -FIELD_HEIGHT / 2.0f + 3.0f; y < FIELD_HEIGHT / 2.0f; y += 3.0f) {
                double total_prob = 1.0;
                int valid_count = 0;
                
                for (auto sensor : sensors) {
                    if (auto prob = sensor->p(x, y, imu_heading)) {
                        total_prob *= prob.value();
                        valid_count++;
                    }
                }
                
                if (valid_count >= 3) {
                    candidates.push_back({x, y, static_cast<float>(total_prob)});
                }
            }
        }
        
        if (candidates.empty()) return false;
        
        auto best = std::max_element(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.score < b.score; });
        
        initNormal(best->x, best->y, imu_heading, 4.0f, 4.0f, 0.1f);
        return true;
    }
    
    // Localize within a specific region (MUCH FASTER for known starting positions!)
    bool initialLocalizationRegion(float imu_heading, float min_x, float max_x, float min_y, float max_y) {
        for (auto sensor : sensors) {
            sensor->update();
        }
        
        struct Candidate {
            float x, y, score;
        };
        std::vector<Candidate> candidates;
        
        // Only search within specified region
        for (float x = min_x; x <= max_x; x += 3.0f) {
            for (float y = min_y; y <= max_y; y += 3.0f) {
                double total_prob = 1.0;
                int valid_count = 0;
                
                for (auto sensor : sensors) {
                    if (auto prob = sensor->p(x, y, imu_heading)) {
                        total_prob *= prob.value();
                        valid_count++;
                    }
                }
                
                if (valid_count >= 2) {  // Only need 2 sensors for regional search
                    candidates.push_back({x, y, static_cast<float>(total_prob)});
                }
            }
        }
        
        if (candidates.empty()) return false;
        
        auto best = std::max_element(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.score < b.score; });
        
        initNormal(best->x, best->y, imu_heading, 3.0f, 3.0f, 0.1f);
        return true;
    }
    
    // Calculate initial pose (X, Y, AND THETA) from distance sensors using GEOMETRY!
    bool initialLocalizationWithHeading(float min_x, float max_x, float min_y, float max_y) {
        // Take multiple sensor readings and average them
        std::vector<std::vector<float>> sensor_readings(sensors.size());
        
        pros::lcd::print(0, "Reading sensors...");
        for (int sample = 0; sample < num_samples; sample++) {
            for (size_t i = 0; i < sensors.size(); i++) {
                sensors[i]->update();
                // Store raw distance reading (we'll need to access sensor internals)
            }
            pros::delay(20);
        }
        
        // Now calculate heading using GEOMETRY from parallel walls
        float calculated_heading = calculateHeadingFromSensors();
        
        if (std::isnan(calculated_heading)) {
            return false;  // Not enough valid sensor readings
        }
        
        pros::lcd::print(0, "Heading: %.1f deg", calculated_heading * 180.0f / M_PI);
        pros::delay(500);
        
        // Now search for X, Y position with the calculated heading
        struct Candidate {
            float x, y, score;
        };
        std::vector<Candidate> candidates;
        
        // Update sensors one more time
        for (auto sensor : sensors) {
            sensor->update();
        }
        
        for (float x = min_x; x <= max_x; x += 3.0f) {
            for (float y = min_y; y <= max_y; y += 3.0f) {
                double total_prob = 1.0;
                int valid_count = 0;
                
                for (auto sensor : sensors) {
                    if (auto prob = sensor->p(x, y, calculated_heading)) {
                        total_prob *= prob.value();
                        valid_count++;
                    }
                }
                
                if (valid_count >= 2) {
                    candidates.push_back({x, y, static_cast<float>(total_prob)});
                }
            }
        }
        
        if (candidates.empty()) return false;
        
        auto best = std::max_element(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.score < b.score; });
        
        initNormal(best->x, best->y, calculated_heading, 2.0f, 2.0f, 0.05f);
        est_theta = calculated_heading;
        return true;
    }
    
private:
    // Calculate heading using geometry from parallel walls
    float calculateHeadingFromSensors() {
        std::vector<float> heading_estimates;
        
        // Take multiple samples and average
        const int NUM_SAMPLES = 10;
        std::vector<std::vector<float>> samples(sensors.size());
        
        pros::lcd::print(1, "Sampling sensors...");
        for (int sample = 0; sample < NUM_SAMPLES; sample++) {
            pros::lcd::print(2, "Sample %d/%d", sample + 1, NUM_SAMPLES);
            for (size_t i = 0; i < sensors.size(); i++) {
                sensors[i]->update();
                
                // Cast to DistanceSensor to access readings
                if (auto* dist_sensor = dynamic_cast<DistanceSensor*>(sensors[i])) {
                    if (dist_sensor->isValid()) {
                        samples[i].push_back(dist_sensor->getMeasuredDistance());
                    }
                }
            }
            pros::delay(20);
        }
        
        pros::lcd::clear_line(2);
        
        // Calculate averaged readings
        std::vector<float> avg_readings;
        std::vector<float> sensor_angles;
        std::vector<size_t> sensor_ids;
        
        for (size_t i = 0; i < sensors.size(); i++) {
            if (!samples[i].empty()) {
                float sum = 0.0f;
                for (float val : samples[i]) {
                    sum += val;
                }
                float avg = sum / samples[i].size();
                avg_readings.push_back(avg);
                
                // Get sensor angle offset
                if (auto* dist_sensor = dynamic_cast<DistanceSensor*>(sensors[i])) {
                    sensor_angles.push_back(dist_sensor->getSensorAngleOffset());
                    sensor_ids.push_back(i);
                }
                
                pros::lcd::print(3 + i, "S%d: %.1f\" (%d samples)", 
                               i, avg, samples[i].size());
            } else {
                pros::lcd::print(3 + i, "S%d: NO DATA", i);
            }
        }
        
        pros::delay(1000);
        
        if (avg_readings.size() < 2) {
            pros::lcd::clear();
            pros::lcd::print(0, "ERROR!");
            pros::lcd::print(1, "Only %d sensors valid", avg_readings.size());
            pros::lcd::print(2, "Need at least 2");
            pros::delay(2000);
            return std::nanf("");
        }
        
        pros::lcd::print(1, "Calculating heading...");
        
        // Method 1: Use opposing sensors (0° and 180°, or 90° and 270°)
        // Find sensor pairs that are roughly opposite (180° apart)
        for (size_t i = 0; i < sensor_angles.size(); i++) {
            for (size_t j = i + 1; j < sensor_angles.size(); j++) {
                float angle_diff = std::abs(sensor_angles[i] - sensor_angles[j]);
                
                // Normalize angle difference
                while (angle_diff > M_PI) angle_diff -= 2.0f * M_PI;
                angle_diff = std::abs(angle_diff);
                
                // Check if sensors are roughly opposite (within 30° of 180°)
                if (angle_diff > 150.0f * M_PI / 180.0f && angle_diff < 210.0f * M_PI / 180.0f) {
                    // These sensors face opposite walls!
                    // Use geometry: tan(heading_error) = (d1 - d2) / baseline
                    // Simplified: heading_error ≈ (d1 - d2) / (d1 + d2) for small angles
                    
                    float d1 = avg_readings[i];
                    float d2 = avg_readings[j];
                    
                    // If both sensors read similar distances, robot is parallel to walls
                    float distance_diff = d1 - d2;
                    float distance_sum = d1 + d2;
                    
                    // Calculate heading relative to the sensor pair's axis
                    // Positive diff means robot is rotated toward sensor i
                    float heading_error = std::atan2(distance_diff, distance_sum);
                    
                    // Calculate absolute heading based on sensor i's angle
                    float base_heading = sensor_angles[i];
                    if (d1 < d2) {
                        // Robot is closer to sensor i's wall, rotated toward it
                        heading_estimates.push_back(base_heading + M_PI - heading_error);
                    } else {
                        // Robot is closer to sensor j's wall, rotated away from i
                        heading_estimates.push_back(base_heading + M_PI + heading_error);
                    }
                    
                    pros::lcd::print(2, "Pair %d-%d: %.1f deg", i, j, 
                                   heading_estimates.back() * 180.0f / M_PI);
                }
            }
        }
        
        if (heading_estimates.empty()) {
            pros::lcd::print(2, "No sensor pairs found");
            return std::nanf("");
        }
        
        // Average all heading estimates
        float sum_sin = 0.0f;
        float sum_cos = 0.0f;
        
        for (float heading : heading_estimates) {
            sum_sin += std::sin(heading);
            sum_cos += std::cos(heading);
        }
        
        float final_heading = std::atan2(sum_sin, sum_cos);
        
        // Normalize to [0, 2π)
        while (final_heading < 0) final_heading += 2.0f * M_PI;
        while (final_heading >= 2.0f * M_PI) final_heading -= 2.0f * M_PI;
        
        pros::lcd::print(2, "Final: %.1f deg (%d pairs)", 
                       final_heading * 180.0f / M_PI, heading_estimates.size());
        
        return final_heading;
    }
    
public:
    
    void update(float dx, float dy, float dtheta, float imu_heading) {
        std::normal_distribution<float> noise_odom(0.0f, ODOM_NOISE);
        std::normal_distribution<float> noise_angle(0.0f, ANGLE_NOISE);
        
        for (auto& p : particles) {
            float noisy_dx = dx + noise_odom(rng) * fabs(dx);
            float noisy_dy = dy + noise_odom(rng) * fabs(dy);
            float noisy_dtheta = dtheta + noise_angle(rng);
            
            p.x += noisy_dx;
            p.y += noisy_dy;
            p.theta += noisy_dtheta;
            
            while (p.theta > M_PI) p.theta -= 2.0f * M_PI;
            while (p.theta < -M_PI) p.theta += 2.0f * M_PI;
        }
        
        distance_since_update += sqrt(dx * dx + dy * dy);
        
        bool should_update = (distance_since_update >= UPDATE_DISTANCE_THRESHOLD) ||
                            (pros::millis() - last_update_time >= UPDATE_TIME_THRESHOLD);
        
        if (!should_update) {
            float sum_x = 0.0f, sum_y = 0.0f;
            for (const auto& p : particles) {
                sum_x += p.x;
                sum_y += p.y;
            }
            est_x = sum_x / NUM_PARTICLES;
            est_y = sum_y / NUM_PARTICLES;
            est_theta = imu_heading;
            return;
        }
        
        for (auto sensor : sensors) {
            sensor->update();
        }
        
        double total_weight = 0.0;
        std::uniform_real_distribution<float> field_dist_x(-FIELD_WIDTH / 2.0f, FIELD_WIDTH / 2.0f);
        std::uniform_real_distribution<float> field_dist_y(-FIELD_HEIGHT / 2.0f, FIELD_HEIGHT / 2.0f);
        
        for (auto& p : particles) {
            if (outOfField(p.x, p.y)) {
                p.x = field_dist_x(rng);
                p.y = field_dist_y(rng);
            }
            
            double weight = 1.0;
            for (auto sensor : sensors) {
                if (auto prob = sensor->p(p.x, p.y, p.theta)) {
                    weight *= prob.value();
                }
            }
            
            p.weight = weight;
            total_weight += weight;
        }
        
        if (total_weight == 0.0) return;
        
        for (auto& p : particles) {
            p.weight /= total_weight;
        }
        
        old_particles = particles;
        
        double avg_weight = 1.0 / NUM_PARTICLES;
        std::uniform_real_distribution<double> rand_dist(0.0, avg_weight);
        double r = rand_dist(rng);
        
        size_t j = 0;
        double cumulative = old_particles[0].weight;
        
        float sum_x = 0.0f, sum_y = 0.0f;
        float sum_cos = 0.0f, sum_sin = 0.0f;
        
        for (size_t i = 0; i < NUM_PARTICLES; i++) {
            double u = r + i * avg_weight;
            
            while (cumulative < u && j < NUM_PARTICLES - 1) {
                j++;
                cumulative += old_particles[j].weight;
            }
            
            particles[i] = old_particles[j];
            particles[i].weight = avg_weight;
            
            sum_x += particles[i].x;
            sum_y += particles[i].y;
            sum_cos += cos(particles[i].theta);
            sum_sin += sin(particles[i].theta);
        }
        
        est_x = sum_x / NUM_PARTICLES;
        est_y = sum_y / NUM_PARTICLES;
        est_theta = atan2(sum_sin, sum_cos);
        
        last_update_time = pros::millis();
        distance_since_update = 0.0f;
    }
    
    float getX() const { return est_x; }
    float getY() const { return est_y; }
    float getTheta() const { return est_theta; }
};