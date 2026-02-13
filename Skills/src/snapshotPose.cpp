// ---------------------------------------------------------------------------
// snapshot_pose.cpp  –  LemLib edition
// ---------------------------------------------------------------------------
// All math is identical to the paper (§2-§3).  The only LemLib-specific
// things are (a) the field map uses centre-origin coords and (b) the example
// wires into chassis.getPose / chassis.setPose.  The solver itself never
// touches a LemLib type — it works on doubles and the callback pointers you
// hand it.
// ---------------------------------------------------------------------------

#include "snapshotPose.hpp"
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cstdio>    // printf

// =========================================================================
// A.  Geometry helpers
// =========================================================================

static constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;

// §2.1  (vright, vfwd) in robot frame  →  field frame
Vec2 robot_to_field(double vr, double vf, double theta_deg) {
    double th   = theta_deg * DEG2RAD;
    double sinT = std::sin(th), cosT = std::cos(th);
    // fhat = (sinT, cosT),  rhat = (cosT, -sinT)
    return { vr * cosT  + vf * sinT,
            -vr * sinT  + vf * cosT };
}

// §2.2  Sensor look-direction in field frame
Vec2 sensor_dir_field(double phi_deg, double theta_deg) {
    double phi = phi_deg * DEG2RAD;
    return robot_to_field(std::sin(phi), std::cos(phi), theta_deg);
}

// §2.2  Sensor origin in field frame
Vec2 sensor_origin_field(double x, double y, double theta_deg, const SensorConfig& sc) {
    Vec2 off = robot_to_field(sc.offset_right, sc.offset_fwd, theta_deg);
    return { x + off.x, y + off.y };
}

// =========================================================================
// B.  Ray–segment intersection  (§3.1)
// =========================================================================
// origin + t·dir  =  a + u·(b-a)   →  solve for t ≥ 0, u ∈ [0,1], t ≤ tmax

double ray_segment_intersect(Vec2 origin, Vec2 dir, Vec2 a, Vec2 b, double t_max) {
    Vec2   edge  = b - a;
    double denom = dir.x * (-edge.y) - dir.y * (-edge.x);
    if (std::abs(denom) < 1e-12) return -1.0;

    Vec2   diff = a - origin;
    double t    = ( diff.x * (-edge.y) - diff.y * (-edge.x) ) / denom;
    double u    = ( dir.x  *  diff.y   - dir.y  *  diff.x   ) / denom;

    if (t < 0.0 || t > t_max || u < 0.0 || u > 1.0) return -1.0;
    return t;
}

// =========================================================================
// C.  Noise model  (§3.4)
// =========================================================================
double noise_sigma(double z_in) {
    constexpr double THRESH     = 200.0 / 25.4;   // 200 mm → in
    constexpr double SIGMA_NEAR = 15.0  / 25.4;   // 15 mm  → in
    return (z_in < THRESH) ? SIGMA_NEAR : 0.05 * z_in;
}

// =========================================================================
// D.  Closest point on segment
// =========================================================================
Vec2 closest_point_on_segment(Vec2 p, Vec2 a, Vec2 b) {
    Vec2   ab   = b - a;
    double len2 = ab.dot(ab);
    if (len2 < 1e-24) return a;
    double t = (p - a).dot(ab) / len2;
    t = std::max(0.0, std::min(1.0, t));
    return a + ab * t;
}

// =========================================================================
// E.  Median  (in-place, small arrays only)
// =========================================================================
double snap_median(double* arr, int n) {
    std::nth_element(arr, arr + n / 2, arr + n);
    if (n % 2 == 1) return arr[n / 2];
    double hi = arr[n / 2];
    std::nth_element(arr, arr + (n / 2 - 1), arr + n);
    return (arr[n / 2 - 1] + hi) * 0.5;
}

// =========================================================================
// F.  Quadrant check  (centre-origin bounds)
// =========================================================================
static bool in_quadrant(double x, double y, Quadrant q) {
    if (q == Quadrant::ANY) return true;
    double m    = SNAP_QUADRANT_MARGIN;
    double xlo, xhi, ylo, yhi;
    switch (q) {
        case Quadrant::BL: xlo=-72; xhi= 0; ylo=-72; yhi= 0; break;
        case Quadrant::BR: xlo=  0; xhi=72; ylo=-72; yhi= 0; break;
        case Quadrant::TL: xlo=-72; xhi= 0; ylo=  0; yhi=72; break;
        case Quadrant::TR: xlo=  0; xhi=72; ylo=  0; yhi=72; break;
        default: return true;
    }
    return x >= (xlo - m) && x <= (xhi + m) &&
           y >= (ylo - m) && y <= (yhi + m);
}

// =========================================================================
// G.  Raycast – up to K closest hits for one sensor
// =========================================================================
struct RayHit { double t; int seg_idx; };

static int raycast_sensor(Vec2 origin, Vec2 dir,
                          const MapSegment* map, int map_size,
                          uint32_t eff_mask, int K, RayHit* hits) {
    RayHit all[64];
    int    cnt = 0;
    for (int j = 0; j < map_size && cnt < 64; ++j) {
        if ((eff_mask & map[j].mask) == 0) continue;
        double t = ray_segment_intersect(origin, dir, map[j].a, map[j].b, SNAP_RANGE_MAX_IN);
        if (t < 0.0) continue;
        all[cnt++] = { t, j };
    }
    if (cnt > K)
        std::partial_sort(all, all + K, all + cnt,
                          [](const RayHit& a, const RayHit& b){ return a.t < b.t; });
    int out = std::min(cnt, K);
    std::memcpy(hits, all, out * sizeof(RayHit));
    return out;
}

// =========================================================================
// H.  Sample & gate one sensor  (§3.3)
// =========================================================================
struct GatedMeasurement { bool valid; double z_in; };

static GatedMeasurement sample_and_gate(int port, ReadSensorFn read_sensor, DelayFn delay_fn) {
    double samples[SNAP_SAMPLES];
    int    valid_count = 0;

    printf("  [port %2d] Sampling...\n", port);
    
    for (int i = 0; i < SNAP_SAMPLES; ++i) {
        if (i > 0) delay_fn(SNAP_DELAY_MS);
        SensorReading r = read_sensor(port);
        double d_in = r.distance_mm / 25.4;

        printf("    sample %d: %4dmm (%5.2fin) conf=%2d", i, r.distance_mm, d_in, r.confidence);

        // range gate
        if (d_in < SNAP_RANGE_MIN_IN || d_in > SNAP_RANGE_MAX_IN) {
            printf(" -> REJECTED (range)\n");
            continue;
        }
        
        // confidence gate
        if (d_in > SNAP_CONF_DIST_IN && r.confidence < SNAP_CONF_MIN) {
            printf(" -> REJECTED (confidence)\n");
            continue;
        }

        samples[valid_count++] = d_in;
        printf(" -> OK\n");
    }
    
    if (valid_count < 2) {
        printf("  [port %2d] GATING FAILED: only %d valid samples (need 2)\n", port, valid_count);
        return { false, 0.0 };
    }
    
    double median_val = snap_median(samples, valid_count);
    printf("  [port %2d] PASSED: median = %.2f in (%d valid samples)\n", port, median_val, valid_count);
    return { true, median_val };
}

// =========================================================================
// I.  Locus pairwise closest-point  (§3.4, Figure 4)
// =========================================================================
struct LocusSegment { Vec2 a, b; };

static Vec2 pairwise_midpoint(const LocusSegment& L1, const LocusSegment& L2) {
    Vec2 mid1 = (L1.a + L1.b) * 0.5;
    Vec2 mid2 = (L2.a + L2.b) * 0.5;
    Vec2 c1   = closest_point_on_segment(mid2, L1.a, L1.b);
    Vec2 c2   = closest_point_on_segment(mid1, L2.a, L2.b);
    return (c1 + c2) * 0.5;
}

// =========================================================================
// J.  Main solver
// =========================================================================
SnapshotResult snapshot_setpose(
    const SensorConfig  sensors[NUM_SENSORS],
    const MapSegment*   map,
    int                 map_size,
    uint32_t            global_mask,
    double              guess_x,
    double              guess_y,
    double              heading_deg,
    Quadrant            quadrant,
    ReadSensorFn        read_sensor,
    DelayFn             delay_fn,
    double              field_origin_x,
    double              field_origin_y
) {
    printf("\n========================================\n");
    printf("SNAPSHOT STARTING\n");
    printf("========================================\n");
    
    // Check if guess position is clearly outside field bounds
    bool guess_outside_field = (std::abs(guess_x) > 72.0 || std::abs(guess_y) > 72.0);
    
    // Determine if we need to auto-calculate field position from quadrant
    // Trigger auto-detection if:
    // 1. Explicit robot-centered mode (guess at origin with quadrant specified), OR
    // 2. Position is outside field bounds (odometry drift), OR
    // 3. Manual field_origin not specified and quadrant given
    bool auto_calculate_field_pos = false;
    
    if (guess_outside_field && quadrant != Quadrant::ANY) {
        printf("⚠ WARNING: Guess position (%.2f, %.2f) is OUTSIDE field bounds (±72)\n", guess_x, guess_y);
        printf("  This usually means odometry has drifted significantly\n");
        printf("  Switching to AUTO-DETECTION mode to find true position\n");
        auto_calculate_field_pos = true;
    }
    else if (field_origin_x == 0.0 && field_origin_y == 0.0 && 
             std::abs(guess_x) < 5.0 && std::abs(guess_y) < 5.0 && 
             quadrant != Quadrant::ANY) {
        // Near origin with quadrant specified - robot-centered mode
        printf("ROBOT-CENTERED MODE: Starting from origin (%.2f, %.2f)\n", guess_x, guess_y);
        printf("  Switching to AUTO-DETECTION based on quadrant\n");
        auto_calculate_field_pos = true;
    }
    
    double guess_x_field = guess_x + field_origin_x;
    double guess_y_field = guess_y + field_origin_y;
    
    if (auto_calculate_field_pos) {
        printf("AUTO-CALCULATION MODE: Determining field position from sensors and quadrant\n");
        
        // Take quick sensor readings to estimate distance to walls
        double distances[NUM_SENSORS];
        Vec2 dirs[NUM_SENSORS];
        bool valid[NUM_SENSORS];
        
        for (int i = 0; i < NUM_SENSORS; ++i) {
            SensorReading r = read_sensor(sensors[i].port);
            if (r.distance_mm < 0 || r.distance_mm > 2000) {
                valid[i] = false;
                continue;
            }
            distances[i] = r.distance_mm / 25.4;  // Convert to inches
            dirs[i] = sensor_dir_field(sensors[i].phi_deg, heading_deg);
            valid[i] = true;
            printf("  Sensor %d (port %d): reading=%.2f in, dir=(%.3f, %.3f)\n", 
                   i, sensors[i].port, distances[i], dirs[i].x, dirs[i].y);
        }
        
        // Use quadrant + sensor readings to estimate position
        // Basic heuristic: use back sensors to estimate distance from walls
        double estimated_x = 0.0, estimated_y = 0.0;
        
        switch (quadrant) {
            case Quadrant::TR:  // Top-right: x>0, y>0
                // Estimate based on distance to top and right walls
                // Back sensors (looking backward) see top wall
                // Left sensor (looking left) sees right wall or back wall
                estimated_x = 60.0;  // Default position
                estimated_y = 60.0;
                
                // If back sensors are valid and pointing up (toward top wall)
                for (int i = 0; i < NUM_SENSORS; ++i) {
                    if (!valid[i]) continue;
                    
                    // Back sensors pointing up (y direction)
                    if (sensors[i].phi_deg == 180.0 && dirs[i].y > 0.9) {
                        // Distance to top wall: 72 - distance_reading
                        double dist_to_wall = distances[i];
                        estimated_y = 72.0 - dist_to_wall - sensors[i].offset_fwd;
                        printf("    → Estimated Y from back sensor: %.2f (wall distance: %.2f)\n", 
                               estimated_y, dist_to_wall);
                    }
                    // Left sensor pointing right (x direction) 
                    else if (sensors[i].phi_deg == -90.0 && dirs[i].x > 0.9) {
                        // Distance to right wall: 72 - distance_reading
                        double dist_to_wall = distances[i];
                        estimated_x = 72.0 - dist_to_wall + sensors[i].offset_right;
                        printf("    → Estimated X from left sensor: %.2f (wall distance: %.2f)\n", 
                               estimated_x, dist_to_wall);
                    }
                }
                break;
                
            case Quadrant::TL:  // Top-left: x<0, y>0
                estimated_x = -60.0;
                estimated_y = 60.0;
                
                for (int i = 0; i < NUM_SENSORS; ++i) {
                    if (!valid[i]) continue;
                    
                    if (sensors[i].phi_deg == 180.0 && dirs[i].y > 0.9) {
                        double dist_to_wall = distances[i];
                        estimated_y = 72.0 - dist_to_wall - sensors[i].offset_fwd;
                    }
                    else if (sensors[i].phi_deg == -90.0 && dirs[i].x < -0.9) {
                        double dist_to_wall = distances[i];
                        estimated_x = -72.0 + dist_to_wall + sensors[i].offset_right;
                    }
                }
                break;
                
            case Quadrant::BR:  // Bottom-right: x>0, y<0
                estimated_x = 60.0;
                estimated_y = -60.0;
                
                for (int i = 0; i < NUM_SENSORS; ++i) {
                    if (!valid[i]) continue;
                    
                    if (sensors[i].phi_deg == 180.0 && dirs[i].y < -0.9) {
                        double dist_to_wall = distances[i];
                        estimated_y = -72.0 + dist_to_wall + sensors[i].offset_fwd;
                    }
                    else if (sensors[i].phi_deg == -90.0 && dirs[i].x > 0.9) {
                        double dist_to_wall = distances[i];
                        estimated_x = 72.0 - dist_to_wall + sensors[i].offset_right;
                    }
                }
                break;
                
            case Quadrant::BL:  // Bottom-left: x<0, y<0
                estimated_x = -60.0;
                estimated_y = -60.0;
                
                for (int i = 0; i < NUM_SENSORS; ++i) {
                    if (!valid[i]) continue;
                    
                    if (sensors[i].phi_deg == 180.0 && dirs[i].y < -0.9) {
                        double dist_to_wall = distances[i];
                        estimated_y = -72.0 + dist_to_wall + sensors[i].offset_fwd;
                    }
                    else if (sensors[i].phi_deg == -90.0 && dirs[i].x < -0.9) {
                        double dist_to_wall = distances[i];
                        estimated_x = -72.0 + dist_to_wall + sensors[i].offset_right;
                    }
                }
                break;
                
            default:
                break;
        }
        
        guess_x_field = estimated_x;
        guess_y_field = estimated_y;
        
        printf("  AUTO-CALCULATED field position: (%.2f, %.2f)\n", guess_x_field, guess_y_field);
        printf("  This will be used as the initial guess for raycasting\n");
    }
    else if (field_origin_x != 0.0 || field_origin_y != 0.0) {
        printf("MANUAL COORDINATE MODE: Robot-centered → Field-centered\n");
        printf("  Robot origin maps to field position: (%.2f, %.2f)\n", field_origin_x, field_origin_y);
        printf("  Input pose (robot frame): x=%.2f  y=%.2f  theta=%.2f\n", guess_x, guess_y, heading_deg);
        printf("  Converted to field frame: x=%.2f  y=%.2f  theta=%.2f\n", guess_x_field, guess_y_field, heading_deg);
    } else {
        printf("FIELD-CENTERED MODE\n");
        printf("Guess pose: x=%.2f  y=%.2f  theta=%.2f\n", guess_x_field, guess_y_field, heading_deg);
    }
    printf("Quadrant: %d (0=ANY 1=BL 2=BR 3=TL 4=TR)\n", (int)quadrant);
    printf("Global mask: 0x%08X\n", global_mask);
    printf("\nSensor config:\n");
    for (int i = 0; i < NUM_SENSORS; ++i) {
        printf("  [%d] port=%2d  offset_r=%6.3f  offset_f=%6.3f  phi=%6.1f  mask=0x%08X\n",
               i, sensors[i].port, sensors[i].offset_right, sensors[i].offset_fwd, 
               sensors[i].phi_deg, sensors[i].mask_override);
    }
    
    // ── 1.  Sample every sensor ──────────────────────────────────────
    printf("\n── PHASE 1: Sampling sensors ──\n");
    GatedMeasurement meas[NUM_SENSORS];
    int live_count = 0;
    int live_idx[NUM_SENSORS];

    for (int i = 0; i < NUM_SENSORS; ++i) {
        meas[i] = sample_and_gate(sensors[i].port, read_sensor, delay_fn);
        if (meas[i].valid) live_idx[live_count++] = i;
    }
    
    printf("\nSensors passed gating: %d / %d (need at least %d)\n", 
           live_count, NUM_SENSORS, SNAP_MIN_SENSORS);
    for (int ii = 0; ii < live_count; ++ii) {
        int i = live_idx[ii];
        printf("  Sensor %d (port %d): z = %.2f in\n", i, sensors[i].port, meas[i].z_in);
    }
    
    if (live_count < SNAP_MIN_SENSORS) {
        printf("✗ FAILED: Not enough sensors\n");
        printf("========================================\n\n");
        return { false, 0, 0, 1e18, live_count };
    }

    // ── 2.  Effective masks + raycast from guess ─────────────────────
    printf("\n── PHASE 2: Raycasting from guess pose ──\n");
    uint32_t eff_mask[NUM_SENSORS];
    Vec2     origins[NUM_SENSORS], dirs[NUM_SENSORS];
    RayHit   candidates[NUM_SENSORS][SNAP_K];
    int      n_cand[NUM_SENSORS];

    for (int ii = 0; ii < live_count; ++ii) {
        int i = live_idx[ii];
        eff_mask[i] = sensors[i].mask_override ? sensors[i].mask_override : global_mask;
        origins[i]  = sensor_origin_field(guess_x_field, guess_y_field, heading_deg, sensors[i]);
        dirs[i]     = sensor_dir_field(sensors[i].phi_deg, heading_deg);
        n_cand[i]   = raycast_sensor(origins[i], dirs[i], map, map_size,
                                     eff_mask[i], SNAP_K, candidates[i]);
        printf("  Sensor %d: origin=(%.2f, %.2f) dir=(%.3f, %.3f) hits=%d\n",
               i, origins[i].x, origins[i].y, dirs[i].x, dirs[i].y, n_cand[i]);
        for (int k = 0; k < n_cand[i]; ++k) {
            printf("    candidate %d: t=%.2f in, seg=%d\n", k, candidates[i][k].t, candidates[i][k].seg_idx);
        }
        if (n_cand[i] == 0) {
            printf("    WARNING: No raycast hits! Sensor can't see any walls.\n");
        }
    }

    // ── 3.  Enumerate candidate combos (mixed-radix) ─────────────────
    printf("\n── PHASE 3: Testing candidate combinations ──\n");
    int radix[NUM_SENSORS];
    int total_combos = 1;
    for (int ii = 0; ii < live_count; ++ii) {
        int i     = live_idx[ii];
        radix[ii] = n_cand[i] ? n_cand[i] : 1;
        total_combos *= radix[ii];
    }
    printf("Total combinations to test: %d\n", total_combos);

    SnapshotResult best = { false, 0, 0, 1e18, 0 };
    int combos_tested = 0;
    int combos_passed_chi2 = 0;
    int combos_passed_quadrant = 0;

    for (int combo = 0; combo < total_combos; ++combo) {
        // decode combo → per-sensor candidate index
        int sel[NUM_SENSORS];
        { int tmp = combo;
          for (int ii = 0; ii < live_count; ++ii) {
              sel[ii] = tmp % radix[ii];
              tmp    /= radix[ii];
          }
        }

        // skip if any sensor doesn't actually have that candidate
        bool skip = false;
        for (int ii = 0; ii < live_count; ++ii)
            if (sel[ii] >= n_cand[live_idx[ii]]) { skip = true; break; }
        if (skip) continue;

        combos_tested++;

        // ── 3a.  Build locus segments  (§3.4) ──────────────────────
        LocusSegment locus[NUM_SENSORS];
        for (int ii = 0; ii < live_count; ++ii) {
            int    i     = live_idx[ii];
            int    seg_j = candidates[i][sel[ii]].seg_idx;
            double zi    = meas[i].z_in;

            Vec2 oF_mount = robot_to_field(sensors[i].offset_right,
                                           sensors[i].offset_fwd,
                                           heading_deg);
            Vec2 shift = oF_mount + dirs[i] * zi;
            locus[ii]  = { map[seg_j].a - shift,
                           map[seg_j].b - shift };
        }

        // ── 3b.  Pairwise → median (x,y)  (Figure 4) ──────────────
        double xs[3], ys[3];
        int    n_pts = 0;
        for (int ii = 0; ii < live_count; ++ii)
            for (int jj = ii + 1; jj < live_count; ++jj) {
                Vec2 mid = pairwise_midpoint(locus[ii], locus[jj]);
                xs[n_pts] = mid.x;
                ys[n_pts] = mid.y;
                ++n_pts;
            }
        double est_x = snap_median(xs, n_pts);
        double est_y = snap_median(ys, n_pts);

        if (combos_tested <= 5 || combos_tested == total_combos) {
            printf("  Combo %d: est=(%.2f, %.2f)", combos_tested, est_x, est_y);
        }

        // ── 3c.  χ² score ──────────────────────────────────────────
        double chi2  = 0.0;
        bool   valid = true;
        for (int ii = 0; ii < live_count; ++ii) {
            int    i     = live_idx[ii];
            int    seg_j = candidates[i][sel[ii]].seg_idx;
            Vec2   oF    = sensor_origin_field(est_x, est_y, heading_deg, sensors[i]);
            double t_pred = ray_segment_intersect(oF, dirs[i],
                                                  map[seg_j].a, map[seg_j].b,
                                                  SNAP_RANGE_MAX_IN);
            if (t_pred < 0.0) { valid = false; break; }

            double res   = meas[i].z_in - t_pred;
            double sigma = noise_sigma(meas[i].z_in);
            double chi2_i = (res / sigma) * (res / sigma);
            if (chi2_i > SNAP_CHI2_MAX) { valid = false; break; }
            chi2 += chi2_i;
        }
        if (!valid) continue;

        combos_passed_chi2++;
        if (combos_tested <= 5 || combos_tested == total_combos) {
            printf(" chi2=%.2f PASS", chi2);
        }

        // ── 3d.  Quadrant gate ─────────────────────────────────────
        if (!in_quadrant(est_x, est_y, quadrant)) {
            if (combos_tested <= 5 || combos_tested == total_combos) {
                printf(" REJECT(quadrant)\n");
            }
            continue;
        }
        
        combos_passed_quadrant++;
        if (combos_tested <= 5 || combos_tested == total_combos) {
            printf(" quadrant OK\n");
        }

        // ── 3e.  Keep best ─────────────────────────────────────────
        if (!best.ok || chi2 < best.chi2)
            best = { true, est_x, est_y, chi2, live_count };
    }
    
    printf("\nCombo stats: tested=%d  passed_chi2=%d  passed_quadrant=%d\n",
           combos_tested, combos_passed_chi2, combos_passed_quadrant);

    // ── 4.  Final validation  (§3.6)  re-raycast nearest hit under mask ─
    printf("\n── PHASE 4: Final validation ──\n");
    if (best.ok) {
        printf("Best solution before validation: (%.2f, %.2f) chi2=%.2f\n",
               best.x_in, best.y_in, best.chi2);
        
        for (int ii = 0; ii < live_count; ++ii) {
            int  i    = live_idx[ii];
            Vec2 oF   = sensor_origin_field(best.x_in, best.y_in, heading_deg, sensors[i]);
            RayHit val_hits[1];
            int  nh   = raycast_sensor(oF, dirs[i], map, map_size,
                                       eff_mask[i], 1, val_hits);
            if (nh == 0) {
                printf("  Sensor %d: validation FAILED (no raycast hit)\n", i);
                best.ok = false;
                break;
            }

            double res   = meas[i].z_in - val_hits[0].t;
            double sigma = noise_sigma(meas[i].z_in);
            double chi2_i = (res / sigma) * (res / sigma);
            printf("  Sensor %d: meas=%.2f pred=%.2f residual=%.2f chi2_i=%.2f",
                   i, meas[i].z_in, val_hits[0].t, res, chi2_i);
            if (chi2_i > SNAP_CHI2_MAX) {
                printf(" REJECT\n");
                best.ok = false;
                break;
            }
            printf(" OK\n");
        }
        if (best.ok && !in_quadrant(best.x_in, best.y_in, quadrant)) {
            printf("  Quadrant check FAILED\n");
            best.ok = false;
        }
    } else {
        printf("No solution found in combo search.\n");
    }

    printf("\n========================================\n");
    printf("SNAPSHOT RESULT: %s\n", best.ok ? "SUCCESS" : "FAILED");
    if (best.ok) {
        master.rumble("..");
        printf("  Final pose: (%.2f, %.2f)\n", best.x_in, best.y_in);
        printf("  Chi-square: %.2f\n", best.chi2);
        printf("  Sensors used: %d\n", best.sensors_used);
    }
    printf("========================================\n\n");

    return best;
}