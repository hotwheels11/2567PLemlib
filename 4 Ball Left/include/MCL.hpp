/*#include "main.h"
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include <memory>

// Particle structure
struct Particle {
    double x;      // x position (inches)
    double y;      // y position (inches)
    double theta;  // heading (radians)
    double weight; // particle weight
};

class MCL {
private:
    std::vector<Particle> particles;
    std::mt19937 gen;
    
    // Number of particles
    static constexpr int NUM_PARTICLES = 500;
    
    // Noise parameters
    static constexpr double ODOM_XY_NOISE = 0.5;    // inches
    static constexpr double ODOM_THETA_NOISE = 0.05; // radians
    static constexpr double SENSOR_NOISE = 2.0;      // inches
    
    // Field dimensions - 12x12 feet = 144x144 inches
    static constexpr double FIELD_WIDTH = 144.0;  // inches
    static constexpr double FIELD_HEIGHT = 144.0; // inches
    
    // Obstacle positions (add your obstacles here)
    struct Obstacle {
        double x, y;      // center position
        double width, height; // dimensions
    };
    std::vector<Obstacle> obstacles;
    
    // Previous odometry values
    double prev_vert_pos = 0.0;
    double prev_horiz_pos = 0.0;
    
    // Estimated pose
    double est_x = 0.0;
    double est_y = 0.0;
    double est_theta = 0.0;
    
public:
    MCL() : gen(std::random_device{}()) {
        initializeParticles();
        
        // Add obstacles here (example)
        // obstacles.push_back({72.0, 72.0, 24.0, 24.0}); // center obstacle
    }
    
    void addObstacle(double x, double y, double width, double height) {
        obstacles.push_back({x, y, width, height});
    }
    
    // Initialize particles uniformly across the field
    void initializeParticles() {
        particles.clear();
        particles.reserve(NUM_PARTICLES);
        
        std::uniform_real_distribution<> dis_x(0.0, FIELD_WIDTH);
        std::uniform_real_distribution<> dis_y(0.0, FIELD_HEIGHT);
        std::uniform_real_distribution<> dis_theta(-M_PI, M_PI);
        
        for (int i = 0; i < NUM_PARTICLES; i++) {
            Particle p;
            p.x = dis_x(gen);
            p.y = dis_y(gen);
            p.theta = dis_theta(gen);
            p.weight = 1.0 / NUM_PARTICLES;
            particles.push_back(p);
        }
    }
    
    // Initialize particles around a known starting position
    void initializeAtPose(double x, double y, double theta, double spread = 6.0) {
        particles.clear();
        particles.reserve(NUM_PARTICLES);
        
        std::normal_distribution<> dis_x(x, spread);
        std::normal_distribution<> dis_y(y, spread);
        std::normal_distribution<> dis_theta(theta, 0.2);
        
        for (int i = 0; i < NUM_PARTICLES; i++) {
            Particle p;
            p.x = dis_x(gen);
            p.y = dis_y(gen);
            p.theta = dis_theta(gen);
            p.weight = 1.0 / NUM_PARTICLES;
            particles.push_back(p);
        }
    }
    
    // Motion update using tracking wheels and IMU
    void motionUpdate(double vert_pos, double horiz_pos, double imu_heading) {
        // Calculate odometry deltas
        double d_vert = vert_pos - prev_vert_pos;
        double d_horiz = horiz_pos - prev_horiz_pos;
        double avg_theta = (imu_heading + est_theta) / 2.0;
        
        // Convert tracking wheel movement to global frame
        double dx = d_vert * cos(avg_theta) - d_horiz * sin(avg_theta);
        double dy = d_vert * sin(avg_theta) + d_horiz * cos(avg_theta);
        double dtheta = imu_heading - est_theta;
        
        // Normalize angle
        while (dtheta > M_PI) dtheta -= 2 * M_PI;
        while (dtheta < -M_PI) dtheta += 2 * M_PI;
        
        // Add noise distributions
        std::normal_distribution<> noise_xy(0.0, ODOM_XY_NOISE);
        std::normal_distribution<> noise_theta(0.0, ODOM_THETA_NOISE);
        
        // Update each particle
        for (auto& p : particles) {
            double noisy_dx = dx + noise_xy(gen);
            double noisy_dy = dy + noise_xy(gen);
            double noisy_dtheta = dtheta + noise_theta(gen);
            
            p.x += noisy_dx;
            p.y += noisy_dy;
            p.theta += noisy_dtheta;
            
            // Normalize theta
            while (p.theta > M_PI) p.theta -= 2 * M_PI;
            while (p.theta < -M_PI) p.theta += 2 * M_PI;
        }
        
        prev_vert_pos = vert_pos;
        prev_horiz_pos = horiz_pos;
    }
    
    // Check if ray from (x,y) at angle intersects any obstacle
    bool rayIntersectsObstacle(double x, double y, double angle, double& dist) {
        double cos_a = cos(angle);
        double sin_a = sin(angle);
        
        for (const auto& obs : obstacles) {
            // Simple box intersection
            double half_w = obs.width / 2.0;
            double half_h = obs.height / 2.0;
            
            // Check each edge of the obstacle
            double edges[4][4] = {
                {obs.x - half_w, obs.y - half_h, obs.x + half_w, obs.y - half_h}, // bottom
                {obs.x + half_w, obs.y - half_h, obs.x + half_w, obs.y + half_h}, // right
                {obs.x + half_w, obs.y + half_h, obs.x - half_w, obs.y + half_h}, // top
                {obs.x - half_w, obs.y + half_h, obs.x - half_w, obs.y - half_h}  // left
            };
            
            for (int i = 0; i < 4; i++) {
                double x1 = edges[i][0], y1 = edges[i][1];
                double x2 = edges[i][2], y2 = edges[i][3];
                
                // Ray-line segment intersection
                double dx = x2 - x1;
                double dy = y2 - y1;
                double det = cos_a * dy - sin_a * dx;
                
                if (fabs(det) > 1e-6) {
                    double t = ((x1 - x) * dy - (y1 - y) * dx) / det;
                    double u = ((x1 - x) * sin_a - (y1 - y) * cos_a) / det;
                    
                    if (t > 0 && u >= 0 && u <= 1) {
                        dist = std::min(dist, t);
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    // Calculate expected distance to wall or obstacle
    double expectedDistanceToWall(double x, double y, double angle) {
        // Normalize angle
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        
        double dist = 1000.0; // Large default
        
        // Check intersection with each wall
        double cos_a = cos(angle);
        double sin_a = sin(angle);
        
        // Right wall (x = FIELD_WIDTH)
        if (cos_a > 0.001) {
            double d = (FIELD_WIDTH - x) / cos_a;
            if (d > 0) dist = std::min(dist, d);
        }
        
        // Left wall (x = 0)
        if (cos_a < -0.001) {
            double d = -x / cos_a;
            if (d > 0) dist = std::min(dist, d);
        }
        
        // Top wall (y = FIELD_HEIGHT)
        if (sin_a > 0.001) {
            double d = (FIELD_HEIGHT - y) / sin_a;
            if (d > 0) dist = std::min(dist, d);
        }
        
        // Bottom wall (y = 0)
        if (sin_a < -0.001) {
            double d = -y / sin_a;
            if (d > 0) dist = std::min(dist, d);
        }
        
        // Check obstacles
        rayIntersectsObstacle(x, y, angle, dist);
        
        return dist;
    }
    
    // Sensor measurement model
    double calculateWeight(const Particle& p, const std::vector<double>& sensor_readings,
                          const std::vector<double>& sensor_angles) {
        double weight = 1.0;
        
        for (size_t i = 0; i < sensor_readings.size(); i++) {
            if (sensor_readings[i] < 0) continue; // Invalid reading
            
            // Calculate expected distance from particle to nearest wall/obstacle
            double sensor_global_angle = p.theta + sensor_angles[i];
            double expected_dist = expectedDistanceToWall(p.x, p.y, sensor_global_angle);
            
            // Calculate likelihood using Gaussian
            double diff = sensor_readings[i] - expected_dist;
            double prob = exp(-0.5 * (diff * diff) / (SENSOR_NOISE * SENSOR_NOISE));
            weight *= prob;
        }
        
        return weight;
    }
    
    // Measurement update using distance sensors
    void measurementUpdate(const std::vector<double>& sensor_readings,
                          const std::vector<double>& sensor_angles) {
        // Calculate weights for all particles
        double sum_weights = 0.0;
        for (auto& p : particles) {
            p.weight = calculateWeight(p, sensor_readings, sensor_angles);
            sum_weights += p.weight;
        }
        
        // Normalize weights
        if (sum_weights > 0.0) {
            for (auto& p : particles) {
                p.weight /= sum_weights;
            }
        }
    }
    
    // Resample particles based on weights (low variance resampling)
    void resample() {
        std::vector<Particle> new_particles;
        new_particles.reserve(NUM_PARTICLES);
        
        std::uniform_real_distribution<> dis(0.0, 1.0 / NUM_PARTICLES);
        double r = dis(gen);
        double c = particles[0].weight;
        int i = 0;
        
        for (int m = 0; m < NUM_PARTICLES; m++) {
            double u = r + m * (1.0 / NUM_PARTICLES);
            while (u > c) {
                i++;
                c += particles[i].weight;
            }
            new_particles.push_back(particles[i]);
            new_particles.back().weight = 1.0 / NUM_PARTICLES;
        }
        
        particles = std::move(new_particles);
    }
    
    // Initial localization using distance sensors
    // Call this when you first place the robot on the field
    bool initialLocalization(const std::vector<double>& sensor_readings,
                            const std::vector<double>& sensor_angles,
                            double imu_heading) {
        // Need at least 3 valid sensor readings for triangulation
        std::vector<int> valid_sensors;
        for (size_t i = 0; i < sensor_readings.size(); i++) {
            if (sensor_readings[i] > 0 && sensor_readings[i] < 100.0) {
                valid_sensors.push_back(i);
            }
        }
        
        if (valid_sensors.size() < 3) {
            return false; // Not enough valid readings
        }
        
        // Generate candidate positions by intersecting sensor rays with walls
        struct Candidate {
            double x, y;
            double score;
        };
        std::vector<Candidate> candidates;
        
        // Grid search across the field
        const double step = 3.0; // 3 inch grid
        for (double x = step; x < FIELD_WIDTH - step; x += step) {
            for (double y = step; y < FIELD_HEIGHT - step; y += step) {
                // Calculate error for this position
                double total_error = 0.0;
                int valid_count = 0;
                
                for (int idx : valid_sensors) {
                    double sensor_angle = imu_heading + sensor_angles[idx];
                    double expected = expectedDistanceToWall(x, y, sensor_angle);
                    double measured = sensor_readings[idx];
                    
                    double error = fabs(expected - measured);
                    if (error < 10.0) { // Within 10 inches
                        total_error += error;
                        valid_count++;
                    }
                }
                
                if (valid_count >= 3) {
                    double avg_error = total_error / valid_count;
                    candidates.push_back({x, y, 1.0 / (avg_error + 0.1)});
                }
            }
        }
        
        if (candidates.empty()) {
            return false;
        }
        
        // Find best candidate
        auto best = std::max_element(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.score < b.score; });
        
        // Initialize particles around the best estimate
        initializeAtPose(best->x, best->y, imu_heading, 4.0);
        
        est_x = best->x;
        est_y = best->y;
        est_theta = imu_heading;
        
        return true;
    }
    
    // Estimate pose from particles (weighted average)
    void estimatePose() {
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_cos = 0.0;
        double sum_sin = 0.0;
        
        for (const auto& p : particles) {
            sum_x += p.x * p.weight;
            sum_y += p.y * p.weight;
            sum_cos += cos(p.theta) * p.weight;
            sum_sin += sin(p.theta) * p.weight;
        }
        
        est_x = sum_x;
        est_y = sum_y;
        est_theta = atan2(sum_sin, sum_cos);
    }
    
    // Main update function - call this in your main loop
    void update(double vert_pos, double horiz_pos, double imu_heading,
                const std::vector<double>& sensor_readings,
                const std::vector<double>& sensor_angles) {
        motionUpdate(vert_pos, horiz_pos, imu_heading);
        measurementUpdate(sensor_readings, sensor_angles);
        resample();
        estimatePose();
    }
    
    // Getters for estimated pose
    double getX() const { return est_x; }
    double getY() const { return est_y; }
    double getTheta() const { return est_theta; }
    
    // Get particle cloud for debugging/visualization
    const std::vector<Particle>& getParticles() const { return particles; }
};

// Motion controller with slew rate limiting
class MotionController {
private:
    MCL* mcl;
    
    // Motion parameters
    double min_speed = 20.0;
    double max_speed = 100.0;
    double slew_rate = 5.0; // units per cycle
    
    // Current motion state
    bool is_moving = false;
    double target_x = 0.0;
    double target_y = 0.0;
    double target_heading = 0.0;
    double start_x = 0.0;
    double start_y = 0.0;
    uint32_t motion_timeout = 0;
    uint32_t motion_start_time = 0;
    
    enum MotionType {
        MOVE_TO_POINT,
        TURN_TO_HEADING,
        NONE
    };
    MotionType current_motion = NONE;
    
    // Slew rate limiting
    double current_left_speed = 0.0;
    double current_right_speed = 0.0;
    
    // Distance tracking
    double distance_moved = 0.0;
    double last_x = 0.0;
    double last_y = 0.0;
    
    // Motor group pointers (you'll set these)
    pros::MotorGroup* left_motors = nullptr;
    pros::MotorGroup* right_motors = nullptr;
    
    double applySlew(double current, double target, double rate) {
        double diff = target - current;
        if (fabs(diff) <= rate) {
            return target;
        }
        return current + (diff > 0 ? rate : -rate);
    }
    
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
    
public:
    MotionController(MCL* localization) : mcl(localization) {}
    
    void setMotors(pros::MotorGroup* left, pros::MotorGroup* right) {
        left_motors = left;
        right_motors = right;
    }
    
    void setSpeedLimits(double min, double max) {
        min_speed = min;
        max_speed = max;
    }
    
    void setSlewRate(double rate) {
        slew_rate = rate;
    }
    
    // Move to a point on the field
    void moveToPoint(double x, double y, uint32_t timeout_ms) {
        target_x = x;
        target_y = y;
        motion_timeout = timeout_ms;
        motion_start_time = pros::millis();
        start_x = mcl->getX();
        start_y = mcl->getY();
        last_x = start_x;
        last_y = start_y;
        distance_moved = 0.0;
        is_moving = true;
        current_motion = MOVE_TO_POINT;
    }
    
    // Turn to a specific heading
    void turnToHeading(double heading_deg, uint32_t timeout_ms) {
        target_heading = heading_deg * M_PI / 180.0;
        motion_timeout = timeout_ms;
        motion_start_time = pros::millis();
        distance_moved = 0.0;
        is_moving = true;
        current_motion = TURN_TO_HEADING;
    }
    
    // Cancel current motion
    void cancelMotion() {
        is_moving = false;
        current_motion = NONE;
        if (left_motors && right_motors) {
            left_motors->move(0);
            right_motors->move(0);
        }
        current_left_speed = 0.0;
        current_right_speed = 0.0;
    }
    
    // Check if motion is complete
    bool isMoving() const {
        return is_moving;
    }
    
    // Get distance moved since motion started
    double getDistanceMoved() const {
        return distance_moved;
    }
    
    // Wait until robot has moved a certain distance
    void waitUntil(double inches) {
        while (is_moving && distance_moved < inches) {
            pros::delay(10);
        }
    }
    
    // Update motion controller (call this every loop iteration)
    void update() {
        if (!is_moving || !left_motors || !right_motors) return;
        
        // Check timeout
        if (pros::millis() - motion_start_time > motion_timeout) {
            cancelMotion();
            return;
        }
        
        double current_x = mcl->getX();
        double current_y = mcl->getY();
        double current_theta = mcl->getTheta();
        
        // Update distance tracking
        double dx = current_x - last_x;
        double dy = current_y - last_y;
        distance_moved += sqrt(dx * dx + dy * dy);
        last_x = current_x;
        last_y = current_y;
        
        double target_left = 0.0;
        double target_right = 0.0;
        
        if (current_motion == MOVE_TO_POINT) {
            // Calculate error
            double error_x = target_x - current_x;
            double error_y = target_y - current_y;
            double distance = sqrt(error_x * error_x + error_y * error_y);
            
            // Check if reached target
            if (distance < 2.0) { // 2 inch tolerance
                cancelMotion();
                return;
            }
            
            // Calculate heading to target
            double desired_heading = atan2(error_y, error_x);
            double heading_error = normalizeAngle(desired_heading - current_theta);
            
            // Calculate speeds using a simple controller
            double forward_speed = std::min(max_speed, std::max(min_speed, distance * 10.0));
            double turn_speed = heading_error * 30.0; // proportional turn control
            
            target_left = forward_speed - turn_speed;
            target_right = forward_speed + turn_speed;
            
            // Clamp to limits
            target_left = std::max(-max_speed, std::min(max_speed, target_left));
            target_right = std::max(-max_speed, std::min(max_speed, target_right));
            
        } else if (current_motion == TURN_TO_HEADING) {
            double heading_error = normalizeAngle(target_heading - current_theta);
            
            // Check if reached target heading
            if (fabs(heading_error) < 0.05) { // ~3 degree tolerance
                cancelMotion();
                return;
            }
            
            // Turn in place
            double turn_speed = std::min(max_speed, std::max(min_speed, fabs(heading_error) * 50.0));
            
            if (heading_error > 0) {
                target_left = -turn_speed;
                target_right = turn_speed;
            } else {
                target_left = turn_speed;
                target_right = -turn_speed;
            }
        }
        
        // Apply slew rate limiting
        current_left_speed = applySlew(current_left_speed, target_left, slew_rate);
        current_right_speed = applySlew(current_right_speed, target_right, slew_rate);
        
        // Set motor speeds
        left_motors->move(current_left_speed);
        right_motors->move(current_right_speed);
    }
};

// Global instances (you can also make these class members)
inline std::unique_ptr<MCL> mcl_system;
inline std::unique_ptr<MotionController> motion_controller;

// Example usage
void autonomous() {
    // Initialize MCL
    mcl_system = std::make_unique<MCL>();
    
    // Add obstacles to the field
    mcl_system->addObstacle(72.0, 72.0, 24.0, 24.0); // Center obstacle
    
    // Setup IMU
    pros::IMU imu(10); // Replace with your IMU port
    imu.reset();
    pros::delay(2000); // Wait for IMU to calibrate
    
    // Example sensor angles (relative to robot frame)
    std::vector<double> sensor_angles = {
        0.0, 0.0,           // Front sensors
        M_PI / 2, M_PI / 2, // Right sensors
        M_PI, M_PI,         // Back sensors
        -M_PI / 2, -M_PI / 2 // Left sensors
    };
    
    // Read distance sensors for initial localization
    std::vector<double> initial_sensor_readings(8);
    for (int i = 0; i < 8; i++) {
        initial_sensor_readings[i] = 0.0; // Read from distance sensor i
    }
    
    // Get IMU heading
    double imu_heading = imu.get_heading() * M_PI / 180.0; // Convert to radians
    
    // Perform initial localization
    pros::lcd::print(0, "Calculating position...");
    bool success = mcl_system->initialLocalization(initial_sensor_readings, 
                                                   sensor_angles, 
                                                   imu_heading);
    
    if (success) {
        pros::lcd::print(0, "Position found!");
        pros::lcd::print(1, "X: %.2f Y: %.2f", mcl_system->getX(), mcl_system->getY());
        pros::delay(1000);
    } else {
        pros::lcd::print(0, "Failed! Using default");
        mcl_system->initializeAtPose(12.0, 12.0, imu_heading);
        pros::delay(1000);
    }
    
    // Initialize motion controller
    motion_controller = std::make_unique<MotionController>(mcl_system.get());
    
    // Setup your motor groups
    pros::MotorGroup left_motors({1, 2, 3}); // Replace with your ports
    pros::MotorGroup right_motors({4, 5, 6}); // Replace with your ports
    motion_controller->setMotors(&left_motors, &right_motors);
    
    // Configure motion parameters
    motion_controller->setSpeedLimits(20.0, 100.0); // min 20, max 100
    motion_controller->setSlewRate(5.0); // 5 units per 20ms cycle
    
    // Your intake motor
    pros::Motor intake(7); // Replace with your port
    
    // Main control loop
    while (true) {
        // Read your sensors
        double vert_pos = 0.0;   // Vertical tracking wheel position
        double horiz_pos = 0.0;  // Horizontal tracking wheel position
        imu_heading = imu.get_heading() * M_PI / 180.0; // IMU heading in radians
        
        std::vector<double> sensor_readings(8);
        for (int i = 0; i < 8; i++) {
            sensor_readings[i] = 0.0; // Read from distance sensor i
        }
        
        // Update MCL
        mcl_system->update(vert_pos, horiz_pos, imu_heading, sensor_readings, sensor_angles);
        
        // Update motion controller
        motion_controller->update();
        
        // Display position
        pros::lcd::print(0, "X: %.2f Y: %.2f", mcl_system->getX(), mcl_system->getY());
        pros::lcd::print(1, "Theta: %.2f", mcl_system->getTheta() * 180.0 / M_PI);
        pros::lcd::print(2, "Distance: %.2f", motion_controller->getDistanceMoved());
        
        pros::delay(20);
    }
}

// Example autonomous routine
void exampleRoutine() {
    // Move to first point and start intake after moving 10 inches
    motion_controller->moveToPoint(48.0, 48.0, 3000); // Move to (48", 48") with 3s timeout
    
    pros::Motor intake(7);
    
    // Start intake after moving 10 inches
    while (motion_controller->isMoving()) {
        if (motion_controller->getDistanceMoved() > 10.0) {
            intake.move(127); // Start intake
        }
        pros::delay(10);
    }
    
    intake.move(0); // Stop intake
    
    // Turn to face 90 degrees
    motion_controller->turnToHeading(90.0, 2000); // 2s timeout
    while (motion_controller->isMoving()) {
        pros::delay(10);
    }
    
    // Move to another point
    motion_controller->moveToPoint(96.0, 96.0, 4000);
    
    // Can cancel motion early if needed
    motion_controller->waitUntil(20.0); // Wait until moved 20 inches
    motion_controller->cancelMotion(); // Cancel rest of movement
}*/