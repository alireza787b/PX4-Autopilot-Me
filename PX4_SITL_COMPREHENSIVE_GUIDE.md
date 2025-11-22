# PX4 SITL Comprehensive Technical Reference
**Complete Guide to Software-In-The-Loop Simulation Architecture**

*Version: 2025 | Based on PX4-Autopilot Main Branch*

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [SITL Architecture Overview](#2-sitl-architecture-overview)
3. [Lockstep Synchronization](#3-lockstep-synchronization)
4. [Sensor Simulation](#4-sensor-simulation)
5. [MAVLink HIL Protocol](#5-mavlink-hil-protocol)
6. [FlightGear Implementation](#6-flightgear-implementation)
7. [Gazebo Implementation](#7-gazebo-implementation)
8. [jMAVSim Implementation](#8-jmavsim-implementation)
9. [EKF2 Configuration](#9-ekf2-configuration)
10. [Building Custom SITL Interface](#10-building-custom-sitl-interface)
11. [Quick Reference Tables](#11-quick-reference-tables)

---

## 1. Executive Summary

PX4's SITL (Software-In-The-Loop) provides a complete simulation environment where PX4 firmware runs on the host computer and interfaces with physics simulators. This document covers three main simulator interfaces:

- **FlightGear:** High-fidelity flight dynamics via UDP + MAVLink bridge
- **Gazebo (gz-sim):** Modern robotics simulator via gz-transport
- **jMAVSim:** Lightweight Java-based quadcopter simulator via MAVLink

### Key Concepts

**Lockstep Synchronization:** PX4 and simulator run in synchronized simulation time (not real-time), enabling deterministic, reproducible simulations at any speed.

**HIL Protocol:** Hardware-In-Loop MAVLink messages (`HIL_SENSOR`, `HIL_GPS`, `HIL_ACTUATOR_CONTROLS`) for sensor data and actuator commands.

**Sensor Fusion:** EKF2 consumes simulated sensor data with configurable noise models and fusion parameters.

---

## 2. SITL Architecture Overview

### 2.1 Core Components

```
┌─────────────────────────────────────────────────────────┐
│  External Simulator (Gazebo/FlightGear/jMAVSim)        │
│  - Physics Engine                                       │
│  - Sensor Simulation                                    │
│  - 3D Visualization                                     │
└────────────────┬────────────────────────────────────────┘
                 │
                 │ Protocol: MAVLink/gz-transport/UDP
                 ↓
┌─────────────────────────────────────────────────────────┐
│  Simulator Interface Module                             │
│  - SimulatorMavlink (FlightGear/jMAVSim)               │
│  - GZBridge (Gazebo)                                    │
└────────────────┬────────────────────────────────────────┘
                 │
                 │ uORB Topics
                 ↓
┌─────────────────────────────────────────────────────────┐
│  PX4 Core Modules                                       │
│  - EKF2 (State Estimation)                             │
│  - Commander (Mode Management)                          │
│  - Navigator (Mission Planning)                         │
│  - Controllers (MC/FW/VTOL)                            │
└────────────────┬────────────────────────────────────────┘
                 │
                 │ uORB: actuator_outputs
                 ↓
┌─────────────────────────────────────────────────────────┐
│  Actuator Output → Simulator                            │
│  - Motor commands                                       │
│  - Servo positions                                      │
└─────────────────────────────────────────────────────────┘
```

### 2.2 File Structure

**Core SITL Code:**
```
src/modules/simulation/
├── simulator_mavlink/       # MAVLink-based simulators (FG, jMAVSim)
│   ├── SimulatorMavlink.cpp
│   └── SimulatorMavlink.hpp
├── gz_bridge/               # Gazebo integration
│   ├── GZBridge.cpp
│   └── GZBridge.hpp
├── sensor_*_sim/            # Individual sensor simulators
│   ├── sensor_gps_sim/
│   ├── sensor_baro_sim/
│   ├── sensor_mag_sim/
│   └── sensor_airspeed_sim/
└── simulator_sih/           # Simulator-In-Hardware (internal)
    └── sih.cpp
```

**Platform-Specific:**
```
platforms/posix/src/px4/common/
├── main.cpp                 # SITL entry point
├── lockstep_scheduler/      # Lockstep implementation
│   ├── lockstep_scheduler.cpp
│   └── lockstep_components.cpp
└── drv_hrt.cpp              # High-resolution timer integration
```

**Configuration:**
```
ROMFS/px4fmu_common/init.d-posix/
├── rcS                      # Main startup script
├── px4-rc.simulator         # Simulator routing
├── px4-rc.mavlinksim        # MAVLink sim config
└── airframes/               # Vehicle configurations
```

---

## 3. Lockstep Synchronization

### 3.1 How Lockstep Works

Lockstep ensures PX4 and the simulator advance in perfect time synchronization:

1. **Simulator sends sensor data** with timestamp (e.g., `HIL_SENSOR`)
2. **PX4 receives data**, sets simulation time via `px4_clock_settime()`
3. **Lockstep scheduler wakes waiting threads** whose deadlines have passed
4. **PX4 modules execute** (EKF2, controllers, etc.)
5. **Barrier synchronization** - All critical components signal completion
6. **PX4 sends actuator commands** to simulator
7. **Simulator receives commands**, advances physics, repeats

### 3.2 Lockstep Scheduler Implementation

**File:** `platforms/posix/src/px4/common/lockstep_scheduler/src/lockstep_scheduler.cpp`

**Key Functions:**

| Function | Purpose | Location |
|----------|---------|----------|
| `set_absolute_time(uint64_t)` | Updates simulation time | Line 50-98 |
| `get_absolute_time()` | Returns current sim time | Inline |
| `cond_timedwait()` | Wait until specific time | Line 100-154 |
| `usleep_until()` | Sleep in sim time | Line 156-173 |

**Time Update Mechanism:**
```cpp
// When simulator sends sensor data
void handle_message_hil_sensor(mavlink_message_t *msg) {
    mavlink_hil_sensor_t sensor;
    mavlink_msg_hil_sensor_decode(msg, &sensor);

    // Set PX4 simulation time
    struct timespec ts;
    ts.tv_sec = sensor.time_usec / 1000000;
    ts.tv_nsec = (sensor.time_usec % 1000000) * 1000;
    px4_clock_settime(CLOCK_MONOTONIC, &ts);

    // Signal lockstep progress
    px4_lockstep_progress();
}
```

### 3.3 Barrier Synchronization (Lockstep Components)

**File:** `platforms/posix/src/px4/common/lockstep_scheduler/src/lockstep_components.cpp`

**Registered Components:**
- SimulatorMavlink receive thread
- WorkQueue threads (hp_default, lp_default, rate_ctrl, etc.)
- Logger (when enabled)

**Flow:**
```cpp
// Each component registers
int comp_id = px4_lockstep_register_component();

// After processing timestep
px4_lockstep_progress(comp_id);

// Sender waits for all components
px4_lockstep_wait_for_components();  // Blocks until all ready
send_actuator_controls();
```

### 3.4 Configuration

**Build Configuration:**
```cmake
# boards/px4/sitl/sitl.cmake
if(REPLAY_FILE)
    set(ENABLE_LOCKSTEP_SCHEDULER no)
else()
    set(ENABLE_LOCKSTEP_SCHEDULER yes)
endif()
```

**Runtime Speed Factor:**
```bash
export PX4_SIM_SPEED_FACTOR=2  # Run at 2x real-time
```

**Board Targets:**
- `px4_sitl_default` - Lockstep enabled
- `px4_sitl_nolockstep` - Real-time mode
- `px4_sitl_replay` - Log replay with lockstep

---

## 4. Sensor Simulation

### 4.1 Sensor Types and Instances

| Sensor | Max Instances | Device Type ID | Update Rate |
|--------|---------------|----------------|-------------|
| IMU (Accel+Gyro) | 3 | 1310988, 1310996, 1311004 | 250-400 Hz |
| GPS/GNSS | 3 | Dynamic | 5-10 Hz |
| Magnetometer | 2 | 197388, 197644 | 50 Hz |
| Barometer | 2 | 6620172, 6620428 | 20 Hz |
| Airspeed | 1 | 1377548 | 8 Hz |
| Distance Sensor | Multiple | Variable | 50 Hz |
| Optical Flow | 1 | Variable | 30-100 Hz |
| Visual Odometry | 1 | - | Variable |

### 4.2 IMU Noise Models

**File:** `src/modules/simulation/simulator_sih/sih.cpp` (Lines 622-640)

**Accelerometer:**
```cpp
// Box-Muller Gaussian noise generation
float wgn = generate_wgn();  // Returns N(0,1)

// Armed (in flight)
float accel_noise_x = 0.5;   // m/s²
float accel_noise_y = 1.7;   // m/s²
float accel_noise_z = 1.4;   // m/s²

// Disarmed (on ground)
float accel_noise_ground = 0.1;  // m/s²

measured_accel = true_accel + wgn * accel_noise;
```

**Gyroscope:**
```cpp
// Armed (in flight)
float gyro_noise_roll = 0.14;   // rad/s
float gyro_noise_pitch = 0.07;  // rad/s
float gyro_noise_yaw = 0.03;    // rad/s

// Disarmed
float gyro_noise_ground = 0.01;  // rad/s

// Add Earth rotation rate
float earth_rate = 7.2921150e-5;  // rad/s
measured_gyro = true_gyro + earth_rate + wgn * gyro_noise;
```

**FIFO Configuration:**
```cpp
// Sensor 0 uses FIFO for realistic data delivery
scale_accel = CONSTANTS_ONE_G / 2048.f;  // ≈ 0.00479 m/s²/LSB
range_accel = 16 * CONSTANTS_ONE_G;       // ±16g

scale_gyro = radians(2000.f / 32768.f);  // ≈ 0.00106 rad/s/LSB
range_gyro = radians(2000.f);             // ±2000 deg/s
```

**Reference:** Bulka & Nahon, IEEE ICRA 2018

### 4.3 GPS Noise Model

**File:** `src/modules/simulation/sensor_gps_sim/SensorGpsSim.cpp` (Lines 117-176)

**Position Noise:**
```cpp
// Gaussian noise
float pos_noise_std = 0.2;  // meters
float alt_noise_std = 0.5;  // meters

float noise_n = generate_wgn() * pos_noise_std;
float noise_e = generate_wgn() * pos_noise_std;
float noise_d = generate_wgn() * alt_noise_std;

// Convert to lat/lon
lat_measured = lat_true + (noise_n / CONSTANTS_RADIUS_OF_EARTH);
lon_measured = lon_true + (noise_e / CONSTANTS_RADIUS_OF_EARTH);
alt_measured = alt_true + noise_d;
```

**Velocity Noise:**
```cpp
float vel_noise_n = 0.06;   // m/s
float vel_noise_e = 0.077;  // m/s
float vel_noise_d = 0.158;  // m/s

vel_measured = vel_true + wgn * vel_noise;
```

**Fix Quality (with 3D fix):**
```cpp
s_variance_m_s = 0.4;     // Speed variance [m/s]
c_variance_rad = 0.1;     // Course variance [rad]
eph = 0.9;                // Horizontal accuracy [m]
epv = 1.78;               // Vertical accuracy [m]
hdop = 0.7;               // Horizontal dilution
vdop = 1.1;               // Vertical dilution
satellites_visible = 10;  // Configurable via SIM_GPS_USED
```

**Update Rate:** 8 Hz (125 ms interval)

### 4.4 Magnetometer Model

**File:** `src/modules/simulation/sensor_mag_sim/SensorMagSim.cpp` (Lines 109-139)

**Physical Model:** World Magnetic Model (WMM)

```cpp
// Get WMM values from GPS position
float declination = get_mag_declination_degrees(lat, lon);
float inclination = get_mag_inclination_degrees(lat, lon);
float field_strength = get_mag_strength_gauss(lat, lon);

// Convert to NED frame
Vector3f mag_earth_frame = Dcm(Euler(0, -inclination, declination))
                           * Vector3f(field_strength, 0, 0);

// Rotate to body frame
Vector3f expected_field = R_to_body.transpose() * mag_earth_frame;

// Add noise
float noise_x = 0.02;  // Gauss
float noise_y = 0.02;  // Gauss
float noise_z = 0.03;  // Gauss

Vector3f noise(wgn() * noise_x, wgn() * noise_y, wgn() * noise_z);
Vector3f measured = expected_field + noise + offset;
```

**Update Rate:** 50 Hz (20 ms interval)

### 4.5 Barometer Model

**File:** `src/modules/simulation/sensor_baro_sim/SensorBaroSim.cpp` (Lines 113-172)

**Physical Model:** ISA (International Standard Atmosphere)

```cpp
// Constants
const float T_MSL = 288.0f;      // K (15°C)
const float P_MSL = 101325.0f;   // Pa
const float L = 0.0065f;         // K/m (lapse rate)
const float g = 9.80665f;        // m/s²
const float M = 0.0289644f;      // kg/mol
const float R = 8.31432f;        // J/(mol·K)

// Temperature at altitude
float temperature = T_MSL - L * altitude;

// Pressure calculation (troposphere)
float pressure_ratio = pow(T_MSL / temperature, (g * M) / (R * L));
float pressure = P_MSL / pressure_ratio;

// Add noise + drift
float noise_std = 1.0;  // Pa
float drift_rate = 0.0;  // Pa/s (configurable)

pressure_measured = pressure + wgn() * noise_std + drift_integrated;
```

**Update Rate:** 20 Hz (50 ms interval)

### 4.6 Airspeed Model

**File:** `src/modules/simulation/sensor_airspeed_sim/SensorAirspeedSim.cpp` (Lines 132-149)

**Physical Model:** Dynamic pressure equation

```cpp
// Air density at altitude
float T_local = 288.15f - 0.0065f * altitude;
float density_ratio = pow(288.15f / T_local, 4.256f);
float air_density = 1.225f / density_ratio;  // kg/m³

// Dynamic pressure
float velocity_x_body = body_velocity.x;  // m/s
float diff_pressure_pa = sign(velocity_x_body)
                         * 0.5f * air_density * velocity_x_body * velocity_x_body;

// Convert to hPa (millibar)
float diff_pressure_hpa = diff_pressure_pa / 100.0f;

// Add noise
float noise_std = 0.01;  // hPa
diff_pressure_measured = diff_pressure_hpa + wgn() * noise_std;
```

**Update Rate:** 8 Hz (125 ms interval)

### 4.7 Noise Parameter Summary Table

| Sensor | Parameter | Value | Unit | File:Line |
|--------|-----------|-------|------|-----------|
| **Accel (armed)** | σ_x, σ_y, σ_z | 0.5, 1.7, 1.4 | m/s² | sih.cpp:623 |
| **Accel (disarmed)** | σ_xyz | 0.1 | m/s² | sih.cpp:628 |
| **Gyro (armed)** | σ_roll, σ_pitch, σ_yaw | 0.14, 0.07, 0.03 | rad/s | sih.cpp:624 |
| **Gyro (disarmed)** | σ_xyz | 0.01 | rad/s | sih.cpp:629 |
| **GPS Position** | σ_horizontal | 0.2 | m | SensorGpsSim.cpp:117 |
| **GPS Position** | σ_vertical | 0.5 | m | SensorGpsSim.cpp:119 |
| **GPS Velocity** | σ_n, σ_e, σ_d | 0.06, 0.077, 0.158 | m/s | SensorGpsSim.cpp:121 |
| **Magnetometer** | σ_x, σ_y, σ_z | 0.02, 0.02, 0.03 | Gauss | SensorMagSim.cpp:133 |
| **Barometer** | σ_pressure | 1.0 | Pa | SensorBaroSim.cpp:154 |
| **Airspeed** | σ_diff_press | 0.01 | hPa | SensorAirspeedSim.cpp:138 |

### 4.8 Box-Muller Transform (Noise Generation)

All sensors use this algorithm for Gaussian white noise:

```cpp
float generate_wgn() {
    static float V1, V2, S;
    static bool phase = true;
    float X;

    if (phase) {
        do {
            float U1 = (float)rand() / RAND_MAX;
            float U2 = (float)rand() / RAND_MAX;
            V1 = 2.0f * U1 - 1.0f;
            V2 = 2.0f * U2 - 1.0f;
            S = V1 * V1 + V2 * V2;
        } while (S >= 1.0f || fabsf(S) < 1e-8f);

        X = V1 * sqrt(-2.0f * log(S) / S);
    } else {
        X = V2 * sqrt(-2.0f * log(S) / S);
    }

    phase = !phase;
    return X;  // Returns N(0,1) sample
}
```

---

## 5. MAVLink HIL Protocol

### 5.1 Messages: Simulator → PX4

#### HIL_SENSOR (ID: 107)

**Purpose:** Primary sensor data (IMU, mag, baro, airspeed)

**Structure:**
```cpp
mavlink_hil_sensor_t {
    uint64_t time_usec;       // Timestamp [µs]
    float xacc, yacc, zacc;   // Acceleration [m/s²]
    float xgyro, ygyro, zgyro; // Angular rate [rad/s]
    float xmag, ymag, zmag;    // Magnetic field [Gauss]
    float abs_pressure;        // Absolute pressure [hPa]
    float diff_pressure;       // Differential pressure [hPa]
    float pressure_alt;        // Pressure altitude [m]
    float temperature;         // Temperature [°C]
    uint32_t fields_updated;   // Bitmask
    uint8_t id;                // Sensor ID (0=primary)
}
```

**Fields Updated Bitmask:**
```
Bit 0-2:   ACCEL (0x007)
Bit 3-5:   GYRO (0x038)
Bit 6-8:   MAG (0x1C0)
Bit 9,11-12: BARO (0x1A00)
Bit 10:    DIFF_PRESS (0x400)
```

**Handler:** `SimulatorMavlink.cpp:503-549`

**Published Topics:**
- `sensor_accel` (PX4Accelerometer)
- `sensor_gyro` (PX4Gyroscope)
- `sensor_mag` (PX4Magnetometer)
- `sensor_baro`
- `differential_pressure`

**Conversions:**
- Pressure: `* 100.0f` (hPa → Pa)

---

#### HIL_GPS (ID: 113)

**Purpose:** GPS/GNSS data

**Structure:**
```cpp
mavlink_hil_gps_t {
    uint64_t time_usec;        // Timestamp [µs]
    int32_t lat, lon;          // Position [degE7]
    int32_t alt;               // Altitude AMSL [mm]
    uint16_t eph, epv;         // Position error [cm]
    uint16_t vel;              // Ground speed [cm/s]
    int16_t vn, ve, vd;        // Velocity NED [cm/s]
    uint16_t cog;              // Course over ground [cdeg]
    uint8_t fix_type;          // 0-1: no fix, 2: 2D, 3: 3D
    uint8_t satellites_visible;
    uint8_t id;                // GPS ID
}
```

**Handler:** `SimulatorMavlink.cpp:2426`

**Conversions:**
- Lat/Lon: `* 1e-7` (degE7 → degrees)
- Altitude: `* 1e-3` (mm → m)
- EPH/EPV: `* 1e-2` (cm → m)
- Velocity: `/ 100.0f` (cm/s → m/s)
- COG: `* 1e-2 * π/180` (cdeg → rad)

---

#### HIL_STATE_QUATERNION (ID: 115)

**Purpose:** Complete vehicle state (alternative to HIL_SENSOR)

**Structure:**
```cpp
mavlink_hil_state_quaternion_t {
    uint64_t time_usec;
    float attitude_quaternion[4];  // [w, x, y, z]
    float rollspeed, pitchspeed, yawspeed;  // [rad/s]
    int32_t lat, lon;              // [degE7]
    int32_t alt;                   // [mm]
    int16_t vx, vy, vz;            // [cm/s]
    uint16_t ind_airspeed;         // [cm/s]
    uint16_t true_airspeed;        // [cm/s]
    int16_t xacc, yacc, zacc;      // [mG]
}
```

**Handler:** `SimulatorMavlink.cpp:2623`

**Published Topics:**
- `airspeed`
- `vehicle_attitude`
- `vehicle_global_position`
- `vehicle_local_position`
- IMU data (derived from attitude + rates)

---

#### HIL_OPTICAL_FLOW (ID: 114)

**Structure:**
```cpp
mavlink_hil_optical_flow_t {
    uint64_t time_usec;
    uint32_t integration_time_us;
    float integrated_x, integrated_y;      // [rad]
    float integrated_xgyro, integrated_ygyro, integrated_zgyro;  // [rad]
    uint32_t time_delta_distance_us;
    float distance;                        // [m]
    int16_t temperature;                   // [cdegC]
    uint8_t sensor_id;
    uint8_t quality;                       // 0-255
}
```

**Handler:** `mavlink_receiver.cpp:884`

---

### 5.2 Messages: PX4 → Simulator

#### HIL_ACTUATOR_CONTROLS (ID: 93)

**Purpose:** Send actuator/motor commands to simulator

**Structure:**
```cpp
mavlink_hil_actuator_controls_t {
    uint64_t time_usec;
    float controls[16];    // Normalized -1 to 1
    uint8_t mode;          // MAV_MODE_FLAG bitmask
    uint64_t flags;        // Bit 0: lockstep enabled
}
```

**Mode Flags:**
```
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  = 0x01
MAV_MODE_FLAG_AUTO_ENABLED         = 0x04
MAV_MODE_FLAG_GUIDED_ENABLED       = 0x08
MAV_MODE_FLAG_STABILIZE_ENABLED    = 0x10
MAV_MODE_FLAG_HIL_ENABLED          = 0x20
MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 0x40
MAV_MODE_FLAG_SAFETY_ARMED         = 0x80
```

**Stream Configuration:**
```cpp
configure_stream("HIL_ACTUATOR_CONTROLS", 200.0f);  // 200 Hz
```

**Source:** `actuator_outputs` or `actuator_outputs_sim` uORB topic

**Handler:** `mavlink/streams/HIL_ACTUATOR_CONTROLS.hpp:67`

---

### 5.3 Message Rates

| Message | Direction | Rate | Notes |
|---------|-----------|------|-------|
| HIL_SENSOR | Sim → PX4 | 250-400 Hz | IMU rate (configurable) |
| HIL_GPS | Sim → PX4 | 5-10 Hz | GPS update rate |
| HIL_STATE_QUATERNION | Sim → PX4 | 200 Hz | Requested by PX4 |
| HIL_OPTICAL_FLOW | Sim → PX4 | 30-100 Hz | Camera-dependent |
| HIL_ACTUATOR_CONTROLS | PX4 → Sim | 200 Hz | Lockstep-synced |
| HEARTBEAT | PX4 → Sim | 1 Hz | Connection keepalive |

### 5.4 HIL Mode Setup

**Enable HIL:**
```cpp
// In mavlink_main.cpp:658
if (hil_enabled && _datarate > 5000) {  // Requires >5KB/s
    _hil_enabled = true;
    configure_stream("HIL_ACTUATOR_CONTROLS", 200.0f);
}
```

**Disable HIL:**
```cpp
_hil_enabled = false;
configure_stream("HIL_ACTUATOR_CONTROLS", 0.0f);
```

**Request Ground Truth:**
```cpp
// PX4 requests HIL_STATE_QUATERNION from simulator
mavlink_command_long_t cmd;
cmd.command = MAV_CMD_SET_MESSAGE_INTERVAL;
cmd.param1 = MAVLINK_MSG_ID_HIL_STATE_QUATERNION;  // 115
cmd.param2 = 5000;  // 5ms = 200 Hz
```

---

## 6. FlightGear Implementation

### 6.1 Architecture

```
┌─────────────────┐         ┌─────────────────┐         ┌─────────────┐
│   FlightGear    │  UDP    │ FlightGear      │  TCP    │  PX4 SITL   │
│   (Physics +    │ <-----> │    Bridge       │ <-----> │  (Autopilot)│
│   Rendering)    │ Binary  │ (C++ Converter) │ MAVLink │             │
└─────────────────┘         └─────────────────┘         └─────────────┘
  Port: 15200+id              Port: 4560+id
```

### 6.2 Communication Protocol

**FlightGear Generic Protocol (UDP Binary):**

**Output (FlightGear → Bridge):** 22 double-precision floats
```
1.  elapsed_sec
2.  latitude_deg
3.  longitude_deg
4.  altitude_ft
5.  roll_deg
6.  pitch_deg
7.  heading_deg
8.  speed_north_fps
9.  speed_east_fps
10. speed_down_fps
11. airspeed_kt
12. accelX_fps²
13. accelY_fps²
14. accelZ_fps²
15. rateRoll_degps
16. ratePitch_degps
17. rateYaw_degps
18. pressure_alt_ft
19. temperature_degc
20. pressure_inhg
21. measured_total_pressure_inhg
22. rpm
```

**Input (Bridge → FlightGear):** 4 double-precision floats
```cpp
struct FGControls {
    double aileron;    // -1 to 1
    double elevator;   // -1 to 1
    double rudder;     // -1 to 1
    double throttle;   // 0 to 1
}
```

### 6.3 Port Configuration

| Connection | Protocol | Port | Description |
|------------|----------|------|-------------|
| FG → Bridge | UDP | 15200 + px4id | Sensor data output |
| Bridge → FG | UDP | 15300 + px4id | Control input |
| Bridge → PX4 | TCP | 4560 + px4id | MAVLink HIL |
| Telnet (optional) | TCP | 15400 + px4id | FlightGear console |

### 6.4 Data Conversions

**Position:**
```cpp
// Feet to meters
altitude_m = altitude_ft * 0.3048;

// Lat/Lon to degE7
lat_degE7 = (int32_t)(latitude_deg * 1e7);
lon_degE7 = (int32_t)(longitude_deg * 1e7);
```

**Velocity:**
```cpp
// Feet per second to cm/s
vel_n_cms = speed_north_fps * 30.48;
vel_e_cms = speed_east_fps * 30.48;
vel_d_cms = speed_down_fps * 30.48;
```

**Acceleration:**
```cpp
// fps² to m/s² with rotation to body frame
float accel_fps2[3] = {accelX, accelY, accelZ};
float accel_ms2[3];
for (int i = 0; i < 3; i++) {
    accel_ms2[i] = accel_fps2[i] * 0.3048;
}
// Apply quaternion rotation to body frame
```

**Angular Rates:**
```cpp
// Degrees/sec to radians/sec
gyro_rad_s[0] = rateRoll_degps * M_PI / 180.0;
gyro_rad_s[1] = ratePitch_degps * M_PI / 180.0;
gyro_rad_s[2] = rateYaw_degps * M_PI / 180.0;
```

**Pressure:**
```cpp
// inHg to Pascals to hPa
pressure_pa = pressure_inhg * 3386.39;
pressure_hpa = pressure_pa / 100.0;
```

### 6.5 Bridge Noise Simulation

**Standard Deviations:**
```cpp
const float ACCEL_NOISE = 0.0001;
const float GYRO_NOISE = 0.001;
const float MAG_NOISE = 0.001;
const float BARO_ALT_NOISE = 0.01;
const float TEMP_NOISE = 0.01;
const float PRESS_NOISE = 0.01;
```

### 6.6 Control Mapping

**Example (Rascal aircraft):**
```json
"Controls": [
    ["5", "/controls/flight/aileron", "-1"],
    ["7", "/controls/flight/elevator", "-1"],
    ["2", "/controls/flight/rudder", "1"],
    ["4", "/controls/engines/engine/throttle", "1"]
]
```

Format: `[PX4_channel, FG_property, scale_factor]`

### 6.7 Startup Commands

**Launch FlightGear:**
```bash
fgfs \
  --fdm=yasim \
  --aircraft=Rascal110-YASim \
  --model-hz=120 \
  --generic=socket,out,100,127.0.0.1,15200,udp,FGtoPX4 \
  --generic=socket,in,100,,15300,udp,PX4toFG \
  --disable-sound \
  --fog-fastest
```

**Launch Bridge:**
```bash
./flightgear_bridge 0 \
  --hostname localhost \
  --tcp_port 4560 \
  --udp_send_port 15300 \
  --udp_recv_port 15200
```

**Launch PX4:**
```bash
make px4_sitl_nolockstep flightgear_rascal
```

### 6.8 Available Aircraft

**File:** `Tools/simulation/flightgear/flightgear_bridge/models/*.json`

- Rascal110 (fixed-wing trainer)
- tf-r1 (rover)
- tf-g1/tf-g2 (autogyro)

**Airframe Files:** `ROMFS/px4fmu_common/init.d-posix/airframes/10XX_flightgear_*`

---

## 7. Gazebo Implementation

### 7.1 Architecture (Modern: Gazebo Garden/Harmonic/Ionic)

```
┌────────────────────────────────────────────────────┐
│  Gazebo (gz-sim)                                   │
│  - Physics: gz-physics (Bullet/DART/TPE)          │
│  - Sensors: gz-sensors                             │
│  - Rendering: gz-rendering (Ogre2)                 │
└────────────────┬───────────────────────────────────┘
                 │
                 │ gz-transport (protobuf over shared mem/UDP)
                 ↓
┌────────────────────────────────────────────────────┐
│  GZBridge Module (PX4)                             │
│  - Subscribes to: /world/*/clock, /world/*/imu,   │
│    /world/*/gps, /world/*/magnetometer, etc.      │
│  - Publishes to: /model/*/command/motor_speed     │
└────────────────┬───────────────────────────────────┘
                 │
                 │ uORB Topics
                 ↓
┌────────────────────────────────────────────────────┐
│  PX4 SITL                                          │
└────────────────────────────────────────────────────┘
```

### 7.2 Communication Topics

**Gazebo → PX4:**
```
/world/{world_name}/clock
/world/{world_name}/model/{model_name}/link/base_link/sensor/imu_sensor/imu
/world/{world_name}/model/{model_name}/link/base_link/sensor/navsat_sensor/navsat
/world/{world_name}/model/{model_name}/link/base_link/sensor/magnetometer_sensor/magnetometer
/world/{world_name}/model/{model_name}/link/base_link/sensor/air_pressure_sensor/air_pressure
/world/{world_name}/model/{model_name}/link/airspeed_link/sensor/air_speed/air_speed
/world/{world_name}/model/{model_name}/link/lidar_sensor_link/sensor/lidar/scan
```

**PX4 → Gazebo:**
```
/model/{model_name}/command/motor_speed     (for ESC)
/model/{model_name}/command/servo_position  (for servos)
/model/{model_name}/command/wheel_speed     (for rovers)
```

### 7.3 Sensor Plugins

**Built-in Gazebo Systems:**
- `gz-sim-imu-system`
- `gz-sim-air-pressure-system`
- `gz-sim-air-speed-system`
- `gz-sim-navsat-system`
- `gz-sim-magnetometer-system`
- `gz-sim-sensors-system` (camera, lidar, depth)

**Custom PX4 Plugins:**

**File:** `src/modules/simulation/gz_plugins/`

| Plugin | Purpose | File |
|--------|---------|------|
| OpticalFlowSystem | Optical flow using OpenCV | optical_flow/ |
| GstCameraSystem | Camera streaming (GStreamer) | gst_camera/ |
| GenericMotorModel | Motor/propeller physics | actuator/ |
| MovingPlatformController | Moving landing platforms | moving_platform/ |
| BuoyancyPlugin | Underwater vehicles | buoyancy/ |
| SpacecraftThruster | Spacecraft simulation | spacecraft_thruster/ |

**Plugin Registration:**
```xml
<!-- server.config -->
<plugin entity_name="OpticalFlowSystem" entity_type="system"
        file="libOpticalFlowSystem.so" name="OpticalFlowSystem">
</plugin>
```

### 7.4 GPS Noise Model (PX4-side)

**File:** `src/modules/simulation/gz_bridge/GZBridge.cpp`

**Based on u-blox F9P characteristics:**

**Markov Process Position Noise:**
```cpp
// Parameters
_pos_noise_amplitude = 0.8f;    // [m]
_pos_random_walk = 0.01f;       // [-]
_pos_markov_time = 0.95f;       // [-]

// Update (per axis: N, E, D)
_pos_noise = _pos_markov_time * _pos_noise +
             _pos_random_walk * generate_wgn();
float pos_measured = pos_true + _pos_noise_amplitude * _pos_noise;
```

**Markov Process Velocity Noise:**
```cpp
// Parameters
_vel_noise_amplitude = 0.05f;   // [m/s]
_vel_noise_density = 0.2f;      // [-]
_vel_markov_time = 0.85f;       // [-]

// Update
_vel_noise = _vel_markov_time * _vel_noise +
             _vel_noise_density * generate_wgn();
float vel_measured = vel_true + _vel_noise_amplitude * _vel_noise;
```

### 7.5 Frame Conversions

**Gazebo uses:**
- Body frame: FLU (Front-Left-Up)
- World frame: ENU (East-North-Up)

**PX4 uses:**
- Body frame: FRD (Front-Right-Down)
- World frame: NED (North-East-Down)

**Conversion:**
```cpp
// FLU to FRD
Vector3f_frd.x =  Vector3f_flu.x;
Vector3f_frd.y = -Vector3f_flu.y;
Vector3f_frd.z = -Vector3f_flu.z;

// ENU to NED
Vector3f_ned.x =  Vector3f_enu.y;
Vector3f_ned.y =  Vector3f_enu.x;
Vector3f_ned.z = -Vector3f_enu.z;
```

### 7.6 Lockstep Synchronization

**Clock Callback:**
```cpp
void GZBridge::clockCallback(const gz::msgs::Clock &msg) {
    struct timespec ts;
    ts.tv_sec = msg.sim().sec();
    ts.tv_nsec = msg.sim().nsec();

    // Initial sync
    px4_clock_settime(CLOCK_REALTIME, &ts);

    // Continuous sync
    px4_clock_settime(CLOCK_MONOTONIC, &ts);
}
```

**Critical:** Clock must be received before sensors to avoid EKF issues.

### 7.7 Actuator Mixing Interfaces

**File:** `src/modules/simulation/gz_bridge/GZMixingInterface*.cpp`

| Interface | Purpose | Command Topic | Type |
|-----------|---------|---------------|------|
| GZMixingInterfaceESC | Motors | /command/motor_speed | double (RPM or throttle) |
| GZMixingInterfaceServo | Control surfaces | /command/servo_position | double (radians) |
| GZMixingInterfaceWheel | Rover wheels | /command/wheel_speed | double (rad/s) |

### 7.8 Update Rates

**Bridge Main Loop:**
```cpp
void GZBridge::Run() {
    // Parameter updates
    parameters_update();

    // Schedule next run
    ScheduleDelayed(10_ms);  // 100 Hz
}
```

**Sensor Callbacks:** Event-driven (triggered by Gazebo)

**Gimbal Update:**
```cpp
ScheduleOnInterval(200_ms);  // 5 Hz
```

### 7.9 Configuration Parameters

**File:** `src/modules/simulation/gz_bridge/parameters.c`

| Parameter | Default | Description |
|-----------|---------|-------------|
| SIM_GZ_EN_LIDAR | 0 | Enable LIDAR |
| SIM_GZ_EN_FLOW | 0 | Enable optical flow |
| SIM_GZ_EN_ASPD | 0 | Enable airspeed |
| SIM_GZ_EN_BARO | 1 | Enable barometer |
| SIM_GZ_EN_ODOM | 0 | Enable odometry |
| SIM_GZ_EN_GPS | 1 | Enable GPS |

### 7.10 Available Models

**File:** `Tools/simulation/gz/models/`

**Multicopters:**
- x500 (basic quadcopter)
- x500_depth (with depth camera)
- x500_vision (with visual odometry)
- x500_lidar_down, x500_lidar_front, x500_lidar_2d

**Fixed-Wing:**
- rc_cessna

**VTOL:**
- standard_vtol
- quadtailsitter
- tiltrotor

**Rovers:**
- rover_differential
- rover_ackermann
- rover_mecanum

**Worlds:**
- default (empty)
- aruco (precision landing)
- baylands (water)
- windy (wind simulation)
- moving_platform

### 7.11 Startup Commands

**Launch Gazebo with PX4:**
```bash
make px4_sitl gz_x500

# Or specify world
PX4_GZ_WORLD=baylands make px4_sitl gz_x500

# Multi-vehicle
PX4_GZ_MODEL_POSE="0,0" make px4_sitl gz_x500
PX4_GZ_MODEL_POSE="2,0" PX4_SYS_AUTOSTART=4001 make px4_sitl gz_x500
```

**Environment Variables:**
```bash
export GZ_SIM_RESOURCE_PATH=$PX4_ROOT/Tools/simulation/gz/models:$PX4_ROOT/Tools/simulation/gz/worlds
export PX4_SIM_SPEED_FACTOR=2  # 2x real-time
```

---

## 8. jMAVSim Implementation

### 8.1 Architecture

```
┌────────────────────────────────────────────────────┐
│  jMAVSim (Java Application)                        │
│  - Simple physics (quadcopter only)                │
│  - 3D visualization (OpenGL via JOGL)              │
│  - Sensor simulation                               │
└────────────────┬───────────────────────────────────┘
                 │
                 │ MAVLink over TCP (default: 4560)
                 ↓
┌────────────────────────────────────────────────────┐
│  SimulatorMavlink Module (PX4)                     │
│  - Receives: HIL_SENSOR, HIL_GPS, HIL_STATE       │
│  - Sends: HIL_ACTUATOR_CONTROLS                   │
└────────────────┬───────────────────────────────────┘
                 │
                 │ uORB Topics
                 ↓
┌────────────────────────────────────────────────────┐
│  PX4 SITL                                          │
└────────────────────────────────────────────────────┘
```

### 8.2 Connection

**Protocol:** MAVLink over TCP

**Default:** `localhost:4560`

**Command-line Options:**
```bash
./jmavsim_run.sh -l           # Enable lockstep
./jmavsim_run.sh -r 250       # IMU rate 250 Hz
./jmavsim_run.sh -tcp 4560    # TCP port
./jmavsim_run.sh -udp 14550   # UDP mode
```

### 8.3 Sensor Simulation

**Simulated Sensors:**
- IMU (accel + gyro) - 3 instances
- GPS - 3 instances
- Magnetometer - 2 instances
- Barometer - 2 instances
- Airspeed - 1 instance

**Note:** Noise modeling occurs in jMAVSim Java code (external repo)

**PX4-side:** Receives pre-noised data via `HIL_SENSOR` and `HIL_GPS`

### 8.4 Update Rates

**Default IMU Rate:** 250 Hz

**Configuration:**
```bash
# ROMFS/px4fmu_common/init.d-posix/px4-rc.jmavsim
param set-default IMU_INTEG_RATE 250
```

**Lockstep:** Enabled with `-l` flag

### 8.5 Startup Commands

**Launch jMAVSim + PX4:**
```bash
make px4_sitl jmavsim

# Headless mode
HEADLESS=1 make px4_sitl jmavsim

# Custom rate
./Tools/simulation/jmavsim/jmavsim_run.sh -l -r 400
```

**Multi-instance:**
```bash
# Instance 0
make px4_sitl jmavsim

# Instance 1
./Tools/simulation/jmavsim/jmavsim_run.sh -l -i 1 -p 4561 &
make px4_sitl_instance_1 jmavsim
```

### 8.6 Limitations

- **Quadcopter only** (no fixed-wing, VTOL, rovers)
- **Basic physics** (simplified dynamics)
- **Limited sensor suite** (no optical flow, range finder)
- **Single vehicle focus** (multi-vehicle requires manual setup)

---

## 9. EKF2 Configuration

### 9.1 Core Parameters

| Parameter | Default | SITL Override | Range | Unit | Description |
|-----------|---------|---------------|-------|------|-------------|
| EKF2_EN | 1 | - | 0-1 | - | Enable EKF2 |
| EKF2_PREDICT_US | 10000 | - | 1000-20000 | µs | Prediction period |
| EKF2_DELAY_MAX | 200 | - | 0-1000 | ms | Max sensor delay |
| EKF2_HGT_REF | 1 (GPS) | - | 0-3 | - | Height reference source |
| EKF2_REQ_GPS_H | 10.0 | 0.5 | - | s | GPS health time required |

### 9.2 IMU Noise Parameters

| Parameter | Default | Range | Unit | Description |
|-----------|---------|-------|------|-------------|
| EKF2_GYR_NOISE | 0.015 | 0.0001-0.1 | rad/s | Gyro noise for covariance |
| EKF2_ACC_NOISE | 0.35 | 0.01-1.0 | m/s² | Accel noise for covariance |
| EKF2_GYR_B_NOISE | 0.001 | 0-0.01 | rad/s² | Gyro bias process noise |
| EKF2_ACC_B_NOISE | 0.003 | 0-0.01 | m/s³ | Accel bias process noise |
| EKF2_GBIAS_INIT | 0.1 | 0-0.2 | rad/s | Gyro bias uncertainty (startup) |
| EKF2_ABIAS_INIT | 0.2 | 0-0.5 | m/s² | Accel bias uncertainty (startup) |
| EKF2_ANGERR_INIT | 0.1 | 0-0.5 | rad | Tilt angle uncertainty (startup) |

### 9.3 GPS Parameters

| Parameter | Default | FW SITL | Range | Unit | Description |
|-----------|---------|---------|-------|------|-------------|
| EKF2_GPS_DELAY | 110 | - | 0-300 | ms | GPS delay relative to IMU |
| EKF2_GPS_P_NOISE | 0.5 | - | 0.01-10 | m | GPS position noise |
| EKF2_GPS_V_NOISE | 0.3 | - | 0.01-5 | m/s | GPS velocity noise |
| EKF2_GPS_P_GATE | 5.0 | - | ≥1.0 | σ | Position innovation gate |
| EKF2_GPS_V_GATE | 5.0 | - | ≥1.0 | σ | Velocity innovation gate |
| EKF2_REQ_EPH | 3.0 | 10.0 | 2-100 | m | Required horizontal accuracy |
| EKF2_REQ_EPV | 5.0 | 10.0 | 2-100 | m | Required vertical accuracy |
| EKF2_REQ_SACC | 0.5 | 1.0 | 0.5-5 | m/s | Required speed accuracy |
| EKF2_REQ_NSATS | 6 | - | 4-12 | - | Required satellite count |
| EKF2_REQ_PDOP | 2.5 | 4.0 | 1.5-5 | - | Maximum PDOP |
| EKF2_REQ_HDRIFT | 0.1 | 0.5 | 0.1-1 | m/s | Max horizontal drift |
| EKF2_REQ_VDRIFT | 0.2 | 1.0 | 0.1-1.5 | m/s | Max vertical drift |

### 9.4 Magnetometer Parameters

| Parameter | Default | Range | Unit | Description |
|-----------|---------|-------|------|-------------|
| EKF2_MAG_TYPE | 0 (auto) | 0-6 | - | Magnetometer fusion mode |
| EKF2_MAG_DELAY | 0 | 0-300 | ms | Mag delay relative to IMU |
| EKF2_MAG_NOISE | 0.05 | 0.001-1 | Gauss | Mag 3-axis fusion noise |
| EKF2_MAG_B_NOISE | 0.0001 | 0-0.1 | Gauss/s | Body mag field process noise |
| EKF2_MAG_E_NOISE | 0.001 | 0-0.1 | Gauss/s | Earth mag field process noise |
| EKF2_MAG_GATE | 3.0 | ≥1.0 | σ | Mag XYZ innovation gate |
| EKF2_HEAD_NOISE | 0.3 | 0.01-1 | rad | Heading fusion noise |
| EKF2_HDG_GATE | 2.6 | ≥1.0 | σ | Heading innovation gate |

### 9.5 Barometer Parameters

| Parameter | Default | Range | Unit | Description |
|-----------|---------|-------|------|-------------|
| EKF2_BARO_CTRL | 1 | 0-1 | - | Enable baro height aiding |
| EKF2_BARO_DELAY | 0 | 0-300 | ms | Baro delay relative to IMU |
| EKF2_BARO_NOISE | 3.5 | 0.01-15 | m | Baro altitude noise |
| EKF2_BARO_GATE | 5.0 | ≥1.0 | σ | Baro/GPS height innovation gate |
| EKF2_GND_EFF_DZ | 4.0 | 0-10 | m | Ground effect deadzone |
| EKF2_GND_MAX_HGT | 0.5 | 0-5 | m | Ground effect max height |

### 9.6 Control Bitmasks

**EKF2_IMU_CTRL (default: 7):**
```
Bit 0: Gyro bias estimation
Bit 1: Accel bias estimation
Bit 2: Gravity vector fusion
```

**EKF2_GPS_CTRL (default: 7):**
```
Bit 0: Longitude/latitude fusion
Bit 1: Altitude fusion
Bit 2: 3D velocity fusion
Bit 3: Dual antenna heading fusion
```

### 9.7 Sensor Data Structures

**File:** `src/modules/ekf2/EKF/common.h`

**IMU Sample:**
```cpp
struct imuSample {
    uint64_t time_us;
    Vector3f delta_ang;        // Integrated gyro [rad]
    Vector3f delta_vel;        // Integrated accel [m/s]
    float delta_ang_dt;        // Integration period [s]
    float delta_vel_dt;        // Integration period [s]
    bool delta_vel_clipping[3];
};
```

**GNSS Sample:**
```cpp
struct gnssSample {
    uint64_t time_us;
    double lat, lon;           // [degrees]
    float alt;                 // MSL [m]
    Vector3f vel;              // NED [m/s]
    float hacc, vacc, sacc;    // Accuracy [m], [m], [m/s]
    uint8_t fix_type, nsats;
    float pdop;
    float yaw, yaw_acc;        // Dual-antenna [rad]
    bool spoofed;
};
```

**Magnetometer Sample:**
```cpp
struct magSample {
    uint64_t time_us;
    Vector3f mag;              // NED body frame [Gauss]
    bool reset;
};
```

**Barometer Sample:**
```cpp
struct baroSample {
    uint64_t time_us;
    float hgt;                 // MSL [m]
    bool reset;
};
```

### 9.8 Sensor Fusion Process

1. **Buffering:** All measurements timestamped and buffered
2. **Delay Compensation:** Aligned using `EKF2_*_DELAY` parameters
3. **Prediction:** State propagated using IMU at `EKF2_PREDICT_US` rate
4. **Innovation:** `z = measurement - prediction`
5. **Gate Test:** `z^T * S^-1 * z < gate_size^2` (S = innovation covariance)
6. **Kalman Update:** If test passes, state corrected: `x = x + K * z`
7. **Covariance Update:** `P = (I - K*H) * P`

### 9.9 SITL-Specific Defaults

**File:** `ROMFS/px4fmu_common/init.d-posix/rcS`
```bash
# Speedup SITL initialization
param set-default EKF2_REQ_GPS_H 0.5  # vs 10.0 hardware
```

**Fixed-Wing SITL:**
```bash
param set-default EKF2_REQ_EPH 10     # Relaxed from 3.0
param set-default EKF2_REQ_EPV 10     # Relaxed from 5.0
param set-default EKF2_REQ_PDOP 4     # Relaxed from 2.5
param set-default EKF2_REQ_SACC 1     # Relaxed from 0.5
```

**SIH (Simulator-In-Hardware):**
```bash
param set-default EKF2_GPS_DELAY 0    # vs 110 ms
```

---

## 10. Building Custom SITL Interface

### 10.1 Design Checklist

**Choose Communication Protocol:**
- [ ] MAVLink HIL (recommended for most cases)
- [ ] Custom transport (gz-transport, ROS, UDP)
- [ ] Hybrid (UDP sensor data + MAVLink commands)

**Implement Lockstep (strongly recommended):**
- [ ] Receive timestamp with sensor data
- [ ] Pause simulator until actuator commands received
- [ ] Advance physics by fixed timestep
- [ ] Ensure deterministic execution

**Sensor Simulation:**
- [ ] IMU (accel + gyro) - mandatory, 250+ Hz
- [ ] GPS - mandatory, 5-10 Hz
- [ ] Magnetometer - mandatory, 50 Hz
- [ ] Barometer - recommended, 20 Hz
- [ ] Airspeed - fixed-wing only, 8 Hz
- [ ] Optical flow - optional
- [ ] Range sensors - optional

**Noise Models:**
- [ ] Implement Box-Muller Gaussian noise
- [ ] Use PX4 reference noise parameters (Section 4.7)
- [ ] Add bias drift for IMU
- [ ] Add Markov process for GPS (optional)

**Frame Conventions:**
- [ ] Body frame: FRD (Front-Right-Down)
- [ ] World frame: NED (North-East-Down)
- [ ] Ensure correct rotations

### 10.2 MAVLink HIL Implementation

**Step 1: Establish Connection**

```cpp
// TCP (recommended)
int sock = socket(AF_INET, SOCK_STREAM, 0);
struct sockaddr_in addr;
addr.sin_family = AF_INET;
addr.sin_addr.s_addr = inet_addr("127.0.0.1");
addr.sin_port = htons(4560);
connect(sock, (struct sockaddr*)&addr, sizeof(addr));

// Set non-blocking
fcntl(sock, F_SETFL, O_NONBLOCK);
```

**Step 2: Main Simulation Loop**

```cpp
uint64_t sim_time_us = 0;
const uint64_t dt_us = 4000;  // 250 Hz

while (running) {
    // 1. Advance physics
    physics_step(dt_us * 1e-6);
    sim_time_us += dt_us;

    // 2. Simulate sensors
    Vector3f accel = get_body_acceleration();
    Vector3f gyro = get_angular_velocity();
    // ... other sensors

    // 3. Send HIL_SENSOR
    send_hil_sensor(sim_time_us, accel, gyro, ...);

    // 4. Send HIL_GPS (every N steps)
    if (sim_time_us % 125000 == 0) {  // 8 Hz
        send_hil_gps(sim_time_us, lat, lon, alt, vel);
    }

    // 5. Wait for HIL_ACTUATOR_CONTROLS (lockstep)
    while (!receive_actuator_controls()) {
        usleep(100);  // Poll every 100µs
    }

    // 6. Apply actuator commands
    apply_motor_commands(actuator_controls);
}
```

**Step 3: Send HIL_SENSOR**

```cpp
void send_hil_sensor(uint64_t time_us, Vector3f accel, Vector3f gyro,
                     Vector3f mag, float abs_press, float diff_press,
                     float temp) {
    mavlink_message_t msg;
    mavlink_hil_sensor_t sensor = {};

    sensor.time_usec = time_us;

    // Acceleration [m/s²]
    sensor.xacc = accel.x;
    sensor.yacc = accel.y;
    sensor.zacc = accel.z;

    // Angular velocity [rad/s]
    sensor.xgyro = gyro.x;
    sensor.ygyro = gyro.y;
    sensor.zgyro = gyro.z;

    // Magnetic field [Gauss]
    sensor.xmag = mag.x;
    sensor.ymag = mag.y;
    sensor.zmag = mag.z;

    // Pressure [hPa/millibar]
    sensor.abs_pressure = abs_press / 100.0f;  // Pa to hPa
    sensor.diff_pressure = diff_press / 100.0f;
    sensor.pressure_alt = calculate_pressure_altitude(abs_press);
    sensor.temperature = temp;

    // Fields updated bitmask
    sensor.fields_updated = 0x1FFF;  // All fields
    sensor.id = 0;  // Primary sensor

    mavlink_msg_hil_sensor_encode(1, 1, &msg, &sensor);
    send_mavlink_message(sock, &msg);
}
```

**Step 4: Send HIL_GPS**

```cpp
void send_hil_gps(uint64_t time_us, double lat, double lon, float alt,
                  Vector3f vel_ned) {
    mavlink_message_t msg;
    mavlink_hil_gps_t gps = {};

    gps.time_usec = time_us;
    gps.lat = (int32_t)(lat * 1e7);     // degE7
    gps.lon = (int32_t)(lon * 1e7);     // degE7
    gps.alt = (int32_t)(alt * 1000.0f); // mm

    gps.vn = (int16_t)(vel_ned.x * 100.0f);  // cm/s
    gps.ve = (int16_t)(vel_ned.y * 100.0f);
    gps.vd = (int16_t)(vel_ned.z * 100.0f);

    float ground_speed = sqrtf(vel_ned.x*vel_ned.x + vel_ned.y*vel_ned.y);
    gps.vel = (uint16_t)(ground_speed * 100.0f);

    gps.cog = (uint16_t)(atan2f(vel_ned.y, vel_ned.x) * 180.0f/M_PI * 100.0f);

    gps.eph = 90;   // 0.9m accuracy (cm)
    gps.epv = 178;  // 1.78m accuracy (cm)
    gps.fix_type = 3;  // 3D fix
    gps.satellites_visible = 10;
    gps.id = 0;

    mavlink_msg_hil_gps_encode(1, 1, &msg, &gps);
    send_mavlink_message(sock, &msg);
}
```

**Step 5: Receive HIL_ACTUATOR_CONTROLS**

```cpp
bool receive_actuator_controls() {
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buf[2048];

    ssize_t len = recv(sock, buf, sizeof(buf), MSG_DONTWAIT);
    if (len <= 0) return false;

    for (ssize_t i = 0; i < len; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
            if (msg.msgid == MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS) {
                mavlink_hil_actuator_controls_t controls;
                mavlink_msg_hil_actuator_controls_decode(&msg, &controls);

                // Extract actuator values (normalized -1 to 1)
                for (int i = 0; i < 16; i++) {
                    actuator_outputs[i] = controls.controls[i];
                }

                // Check armed state
                bool armed = (controls.mode & MAV_MODE_FLAG_SAFETY_ARMED);

                return true;
            }
        }
    }
    return false;
}
```

### 10.3 Noise Implementation

**Box-Muller Transform:**

```cpp
class GaussianNoise {
private:
    bool phase = true;
    float V1, V2, S;

public:
    float generate() {
        float X;
        if (phase) {
            do {
                float U1 = (float)rand() / RAND_MAX;
                float U2 = (float)rand() / RAND_MAX;
                V1 = 2.0f * U1 - 1.0f;
                V2 = 2.0f * U2 - 1.0f;
                S = V1*V1 + V2*V2;
            } while (S >= 1.0f || fabsf(S) < 1e-8f);
            X = V1 * sqrtf(-2.0f * logf(S) / S);
        } else {
            X = V2 * sqrtf(-2.0f * logf(S) / S);
        }
        phase = !phase;
        return X;  // N(0,1)
    }

    float generate(float mean, float std) {
        return mean + std * generate();
    }
};

// Usage
GaussianNoise noise_gen;
float accel_noisy = accel_true + noise_gen.generate(0.0f, 0.5f);
```

**GPS Markov Process (optional):**

```cpp
class MarkovNoise {
private:
    float state;
    float amplitude;
    float random_walk;
    float time_constant;
    GaussianNoise wgn;

public:
    MarkovNoise(float amp, float rw, float tc)
        : state(0), amplitude(amp), random_walk(rw), time_constant(tc) {}

    float update() {
        state = time_constant * state + random_walk * wgn.generate();
        return amplitude * state;
    }
};

// Usage
MarkovNoise gps_pos_noise(0.8f, 0.01f, 0.95f);  // Per axis
float pos_n_noisy = pos_n_true + gps_pos_noise.update();
```

### 10.4 Testing and Validation

**Test 1: Time Synchronization**
```bash
# In PX4 NSH console
listener sensor_accel
# Verify timestamps increase monotonically
```

**Test 2: Sensor Rates**
```bash
listener sensor_accel -n 100
# Check ~250 Hz (4ms intervals)

listener sensor_gps -n 10
# Check ~8 Hz (125ms intervals)
```

**Test 3: EKF Convergence**
```bash
listener vehicle_local_position
# Verify position converges after GPS lock

listener ekf2_innovations
# Check innovation magnitudes < gate sizes
```

**Test 4: Lockstep Determinism**
```bash
# Run same mission twice
# Compare logs - should be identical
```

**Test 5: Actuator Response**
```bash
# Arm and throttle up
# Verify motor commands received in simulator
# Check physics responds correctly
```

### 10.5 Common Pitfalls

**❌ Incorrect Frame Conventions**
- Simulator uses ENU, PX4 expects NED → Convert!
- Simulator uses FLU body frame, PX4 expects FRD → Convert!

**❌ Wrong Units**
- Pressure: Pa vs hPa/mbar
- Lat/Lon: degrees vs degE7
- Altitude: meters vs millimeters
- Velocity: m/s vs cm/s

**❌ Stale Sensor Detection**
- Sensor update rate too slow (< 100 Hz for IMU)
- Irregular timestamps
- Solution: Ensure consistent update rates

**❌ Clock Not First**
- If using custom protocol, set time BEFORE publishing sensors
- Otherwise EKF receives data with future timestamps

**❌ No Lockstep**
- Simulator runs ahead of PX4
- Non-deterministic results
- Solution: Wait for actuator controls before advancing

**❌ Excessive Noise**
- EKF diverges or doesn't converge
- Solution: Use PX4 reference noise values (Section 4.7)

**❌ Quaternion Normalization**
- Send unnormalized quaternions → attitude errors
- Solution: Always normalize before sending

### 10.6 Example Repositories

**FlightGear Bridge:**
- Repo: https://github.com/ThunderFly-aerospace/FlightGear-Ardupilot
- Shows complete UDP + MAVLink bridge architecture

**Gazebo Classic (deprecated but educational):**
- Repo: https://github.com/PX4/PX4-SITL_gazebo-classic
- Plugin-based approach with sensor simulation

**AirSim (alternative SITL):**
- Repo: https://github.com/microsoft/AirSim
- Uses MAVLink HIL protocol
- Good reference for advanced sensor simulation

---

## 11. Quick Reference Tables

### 11.1 Sensor Update Rates

| Sensor | Rate | File | Parameter |
|--------|------|------|-----------|
| IMU | 250-400 Hz | sih.cpp:169 | IMU_INTEG_RATE |
| GPS | 8 Hz | SensorGpsSim.cpp:55 | - |
| Magnetometer | 50 Hz | SensorMagSim.cpp:55 | - |
| Barometer | 20 Hz | SensorBaroSim.cpp:54 | - |
| Airspeed | 8 Hz | SensorAirspeedSim.cpp:55 | - |
| Distance Sensor | 50 Hz | sih.cpp:229 | - |

### 11.2 MAVLink Message IDs

| Message | ID | Direction | Rate |
|---------|-----|-----------|------|
| HIL_SENSOR | 107 | Sim → PX4 | 250-400 Hz |
| HIL_GPS | 113 | Sim → PX4 | 5-10 Hz |
| HIL_STATE_QUATERNION | 115 | Sim → PX4 | 200 Hz |
| HIL_OPTICAL_FLOW | 114 | Sim → PX4 | 30-100 Hz |
| HIL_ACTUATOR_CONTROLS | 93 | PX4 → Sim | 200 Hz |
| HEARTBEAT | 0 | Both | 1 Hz |

### 11.3 Port Assignments

| Simulator | Protocol | Port(s) | Description |
|-----------|----------|---------|-------------|
| jMAVSim | TCP | 4560 + instance | MAVLink |
| FlightGear | UDP | 15200 + instance | FG output |
| FlightGear | UDP | 15300 + instance | FG input |
| FlightGear Bridge | TCP | 4560 + instance | MAVLink to PX4 |
| Gazebo | gz-transport | - | Shared memory/multicast |
| QGroundControl | UDP | 14550 | GCS link |

### 11.4 Key Files Reference

**SITL Core:**
```
platforms/posix/src/px4/common/
├── main.cpp                       # Entry point
├── lockstep_scheduler/            # Lockstep implementation
└── drv_hrt.cpp                    # Time functions

src/modules/simulation/
├── simulator_mavlink/             # MAVLink SITL (FG, jMAVSim)
├── gz_bridge/                     # Gazebo interface
├── sensor_*_sim/                  # Sensor simulators
└── simulator_sih/                 # Internal simulator
```

**Configuration:**
```
ROMFS/px4fmu_common/init.d-posix/
├── rcS                            # Main startup
├── px4-rc.simulator               # Simulator routing
└── airframes/                     # Vehicle configs

boards/px4/sitl/
└── sitl.cmake                     # Build config
```

**EKF2:**
```
src/modules/ekf2/
├── EKF/                           # Kalman filter core
├── EKF2.cpp                       # Main module
└── params.c                       # Parameters
```

### 11.5 Build Targets

**SITL Variants:**
```bash
make px4_sitl_default          # Lockstep enabled
make px4_sitl_nolockstep       # Real-time mode
make px4_sitl_test             # Unit test config
make px4_sitl_replay           # Log replay
```

**Simulators:**
```bash
make px4_sitl gz_x500          # Gazebo quadcopter
make px4_sitl jmavsim          # jMAVSim quadcopter
make px4_sitl_nolockstep flightgear_rascal  # FlightGear plane
```

**Multi-instance:**
```bash
make px4_sitl_instance_0 gz_x500
make px4_sitl_instance_1 gz_x500
```

### 11.6 Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| PX4_SIM_MODEL | - | Simulator model name |
| PX4_SIMULATOR | - | Simulator type (gz, jmavsim, flightgear) |
| PX4_SIM_SPEED_FACTOR | 1 | Simulation speed multiplier |
| PX4_GZ_WORLD | default | Gazebo world name |
| PX4_GZ_MODEL_POSE | 0,0,0,0,0,0 | Model spawn position |
| GZ_SIM_RESOURCE_PATH | - | Gazebo model search paths |
| HEADLESS | 0 | Disable GUI (jMAVSim) |

### 11.7 Noise Parameters (Reference Values)

| Sensor | Parameter | PX4 Value | Units |
|--------|-----------|-----------|-------|
| Accel | σ (armed) | 0.5-1.7 | m/s² |
| Accel | σ (disarmed) | 0.1 | m/s² |
| Gyro | σ (armed) | 0.03-0.14 | rad/s |
| Gyro | σ (disarmed) | 0.01 | rad/s |
| GPS Position | σ_horizontal | 0.2 | m |
| GPS Position | σ_vertical | 0.5 | m |
| GPS Velocity | σ | 0.06-0.16 | m/s |
| Magnetometer | σ | 0.02-0.03 | Gauss |
| Barometer | σ | 1.0 | Pa |
| Airspeed | σ | 0.01 | hPa |

### 11.8 Frame Conventions

**PX4 Standard:**
- Body frame: **FRD** (Front-Right-Down)
- World frame: **NED** (North-East-Down)
- Rotation: Right-hand rule

**Gazebo:**
- Body frame: **FLU** (Front-Left-Up)
- World frame: **ENU** (East-North-Up)

**Conversion FLU → FRD:**
```
x_frd =  x_flu
y_frd = -y_flu
z_frd = -z_flu
```

**Conversion ENU → NED:**
```
x_ned =  y_enu
y_ned =  x_enu
z_ned = -z_enu
```

---

## Conclusion

This guide provides a complete technical reference for PX4 SITL simulation. Key takeaways:

1. **Lockstep is essential** for deterministic, reproducible simulations
2. **MAVLink HIL protocol** is the standard interface (except Gazebo which uses gz-transport)
3. **Sensor noise models** use Box-Muller Gaussian noise with PX4-validated parameters
4. **EKF2 parameters** are more relaxed in SITL for faster initialization
5. **Frame conventions** (FRD/NED) must be strictly followed

For custom simulator integration:
- Use MAVLink HIL protocol (lowest integration effort)
- Implement lockstep synchronization
- Follow PX4 noise parameters
- Test thoroughly with EKF2 convergence metrics

**Further Resources:**
- PX4 Developer Guide: https://docs.px4.io/main/en/simulation/
- MAVLink Documentation: https://mavlink.io/en/
- PX4 Discuss Forum: https://discuss.px4.io/

---

**Document Version:** 1.0
**Last Updated:** 2025-01-22
**PX4 Version:** Main branch (v1.15.0-dev)
**Author:** Generated from PX4-Autopilot codebase analysis
