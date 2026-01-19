# RigidBody6DOF Enhanced Initialization and Multi-Frame Outputs

## Summary

Enhance RigidBody6DOF component to support:

1. **LLA initialization** - Initialize position from Lat/Lon/Alt instead of just ECEF
2. **Euler angle initialization** - Initialize attitude from yaw/pitch/roll relative to local NED
3. **Multi-frame outputs** - Output position/velocity/attitude in multiple coordinate frames

## Background

Currently RigidBody6DOF only supports:

- Position initialization via ECEF (`initial_position: [x, y, z]`)
- Attitude initialization via quaternion (`initial_attitude: [w, x, y, z]`)
- Outputs in ECEF (position) and body frame (velocity)

Users working with real-world scenarios (like the rocket example at 100km altitude) find it unnatural to specify ECEF coordinates. LLA is more intuitive, and Euler angles (yaw/pitch/roll relative to local horizon) are more interpretable than quaternions.

## Vulcan Utilities Available

| Function | Purpose |
|----------|---------|
| `vulcan::lla_to_ecef(lla)` | Convert LLA to ECEF position |
| `vulcan::ecef_to_lla(r_ecef)` | Convert ECEF to LLA (Vermeille 2004) |
| `vulcan::local_ned_at(r_ecef)` | Get NED frame at ECEF position |
| `vulcan::body_from_euler(ned, yaw, pitch, roll)` | Body frame from Euler angles |
| `vulcan::quaternion_from_body(body, ned)` | Extract quaternion from body frame |
| `vulcan::euler_from_body(body, ned)` | Extract Euler angles from body frame |
| `janus::Quaternion::from_euler(roll, pitch, yaw)` | Quaternion from ZYX Euler |

---

## Implementation Plan

### Part 1: LLA Position Initialization

**File:** `components/dynamics/RigidBody6DOF.hpp`

#### 1.1 Add includes

```cpp
#include <vulcan/coordinates/Geodetic.hpp>
#include <vulcan/coordinates/LocalFrames.hpp>
#include <vulcan/coordinates/BodyFrames.hpp>
```

#### 1.2 Update Stage() method

Add LLA initialization before ECEF fallback:

```cpp
void Stage(Backplane<Scalar> &) override {
    const auto &config = this->GetConfig();

    // === Position Initialization ===
    // Priority: initial_lla > initial_position (ECEF)
    if (config.template Has<Vec3<double>>("initial_lla")) {
        // LLA format: [lat_deg, lon_deg, alt_m]
        auto lla_vec = config.template Get<Vec3<double>>("initial_lla", Vec3<double>::Zero());

        // Convert degrees to radians for lat/lon
        constexpr double deg2rad = 3.14159265358979323846 / 180.0;
        vulcan::LLA<double> lla;
        lla.lat = lla_vec(0) * deg2rad;
        lla.lon = lla_vec(1) * deg2rad;
        lla.alt = lla_vec(2);

        // Convert to ECEF
        Vec3<double> r_ecef = vulcan::lla_to_ecef(lla);
        position_ = Vec3<Scalar>{static_cast<Scalar>(r_ecef(0)),
                                 static_cast<Scalar>(r_ecef(1)),
                                 static_cast<Scalar>(r_ecef(2))};
    } else if (config.template Has<Vec3<double>>("initial_position")) {
        // Existing ECEF initialization
        auto pos = config.template Get<Vec3<double>>("initial_position", Vec3<double>::Zero());
        position_ = Vec3<Scalar>{...};
    }
    // ... rest of Stage()
}
```

#### 1.3 Add programmatic setter

```cpp
void SetInitialPositionLLA(double lat_deg, double lon_deg, double alt_m) {
    constexpr double deg2rad = 3.14159265358979323846 / 180.0;
    vulcan::LLA<double> lla{lat_deg * deg2rad, lon_deg * deg2rad, alt_m};
    Vec3<double> r_ecef = vulcan::lla_to_ecef(lla);
    position_ = Vec3<Scalar>{static_cast<Scalar>(r_ecef(0)),
                             static_cast<Scalar>(r_ecef(1)),
                             static_cast<Scalar>(r_ecef(2))};
}
```

---

### Part 2: Euler Angle Attitude Initialization

#### 2.1 Update Stage() for Euler angles

The Euler angles define body orientation relative to local NED frame at the initial position. Order is ZYX (aerospace standard): yaw -> pitch -> roll.

```cpp
// === Attitude Initialization ===
// Priority: initial_euler_zyx > initial_attitude (quaternion)
if (config.template Has<Vec3<double>>("initial_euler_zyx")) {
    // Euler format: [yaw_deg, pitch_deg, roll_deg] (ZYX sequence)
    auto euler = config.template Get<Vec3<double>>("initial_euler_zyx", Vec3<double>::Zero());

    constexpr double deg2rad = 3.14159265358979323846 / 180.0;
    double yaw   = euler(0) * deg2rad;
    double pitch = euler(1) * deg2rad;
    double roll  = euler(2) * deg2rad;

    // Get current position in ECEF (must be set before attitude)
    Vec3<double> r_ecef{static_cast<double>(position_(0)),
                        static_cast<double>(position_(1)),
                        static_cast<double>(position_(2))};

    // Compute NED frame at this position
    auto ned = vulcan::local_ned_at(r_ecef);

    // Build body frame from Euler angles relative to NED
    auto body = vulcan::body_from_euler(ned, yaw, pitch, roll);

    // Extract quaternion (body-to-ECEF, which is what we integrate)
    // The body frame axes in ECEF define the rotation
    auto q_body_to_ecef = janus::Quaternion<double>::from_rotation_matrix(
        (Mat3<double>() << body.x_axis, body.y_axis, body.z_axis).finished());

    attitude_ = Vec4<Scalar>{static_cast<Scalar>(q_body_to_ecef.w),
                             static_cast<Scalar>(q_body_to_ecef.x),
                             static_cast<Scalar>(q_body_to_ecef.y),
                             static_cast<Scalar>(q_body_to_ecef.z)};
} else if (config.template Has<Vec4<double>>("initial_attitude")) {
    // Existing quaternion initialization
    ...
}
```

#### 2.2 Add programmatic setter

```cpp
void SetInitialAttitudeEuler(double yaw_deg, double pitch_deg, double roll_deg) {
    constexpr double deg2rad = 3.14159265358979323846 / 180.0;
    double yaw   = yaw_deg * deg2rad;
    double pitch = pitch_deg * deg2rad;
    double roll  = roll_deg * deg2rad;

    Vec3<double> r_ecef{static_cast<double>(position_(0)),
                        static_cast<double>(position_(1)),
                        static_cast<double>(position_(2))};

    auto ned = vulcan::local_ned_at(r_ecef);
    auto body = vulcan::body_from_euler(ned, yaw, pitch, roll);
    auto q = vulcan::quaternion_from_body(body, ned);

    // Convert to body-to-ECEF quaternion
    // ... (similar logic)
}
```

---

### Part 3: Multi-Frame Output Signals

#### 3.1 Add member variables for derived outputs

```cpp
private:
    // === Derived outputs: Coordinate frames ===
    Vec3<Scalar> position_lla_ = Vec3<Scalar>::Zero();  // [lat, lon, alt] in radians/meters
    Vec3<Scalar> velocity_ned_ = Vec3<Scalar>::Zero();  // [vn, ve, vd] in m/s
    Vec3<Scalar> euler_zyx_ = Vec3<Scalar>::Zero();     // [yaw, pitch, roll] in radians
```

#### 3.2 Register output signals in Provision()

```cpp
void Provision(Backplane<Scalar> &bp) override {
    // ... existing registrations ...

    // === Derived outputs: Alternative coordinate frames ===

    // Position in LLA (geodetic)
    bp.template register_output<Scalar>("position_lla.lat", &position_lla_(0), "rad",
                                        "Geodetic latitude");
    bp.template register_output<Scalar>("position_lla.lon", &position_lla_(1), "rad",
                                        "Longitude");
    bp.template register_output<Scalar>("position_lla.alt", &position_lla_(2), "m",
                                        "Altitude above ellipsoid");

    // Velocity in NED
    bp.template register_output<Scalar>("velocity_ned.n", &velocity_ned_(0), "m/s",
                                        "North velocity");
    bp.template register_output<Scalar>("velocity_ned.e", &velocity_ned_(1), "m/s",
                                        "East velocity");
    bp.template register_output<Scalar>("velocity_ned.d", &velocity_ned_(2), "m/s",
                                        "Down velocity");

    // Attitude as Euler angles (ZYX: yaw-pitch-roll)
    bp.template register_output<Scalar>("euler_zyx.yaw", &euler_zyx_(0), "rad",
                                        "Yaw angle (heading)");
    bp.template register_output<Scalar>("euler_zyx.pitch", &euler_zyx_(1), "rad",
                                        "Pitch angle");
    bp.template register_output<Scalar>("euler_zyx.roll", &euler_zyx_(2), "rad",
                                        "Roll angle");
}
```

#### 3.3 Compute derived outputs in Step()

```cpp
void Step(Scalar t, Scalar dt) override {
    // ... existing derivative computation ...

    // === Compute derived outputs ===

    // Position in LLA
    vulcan::LLA<Scalar> lla = vulcan::ecef_to_lla(position_);
    position_lla_(0) = lla.lat;
    position_lla_(1) = lla.lon;
    position_lla_(2) = lla.alt;

    // Get NED frame at current position
    auto ned = vulcan::local_ned_at(position_);

    // Velocity in NED (transform from ECEF)
    velocity_ned_ = ned.from_ecef(velocity_ref_);

    // Attitude as Euler angles
    // Build body frame from quaternion
    janus::Quaternion<Scalar> quat{attitude_(0), attitude_(1), attitude_(2), attitude_(3)};

    // Body axes in ECEF
    Vec3<Scalar> x_body_ecef = quat.rotate(Vec3<Scalar>::UnitX());
    Vec3<Scalar> y_body_ecef = quat.rotate(Vec3<Scalar>::UnitY());
    Vec3<Scalar> z_body_ecef = quat.rotate(Vec3<Scalar>::UnitZ());

    // Transform body axes to NED
    Vec3<Scalar> x_body_ned = ned.from_ecef(x_body_ecef);
    Vec3<Scalar> y_body_ned = ned.from_ecef(y_body_ecef);
    Vec3<Scalar> z_body_ned = ned.from_ecef(z_body_ecef);

    // Build DCM and extract Euler angles
    Mat3<Scalar> dcm_body_to_ned;
    dcm_body_to_ned.col(0) = x_body_ned;
    dcm_body_to_ned.col(1) = y_body_ned;
    dcm_body_to_ned.col(2) = z_body_ned;

    euler_zyx_ = vulcan::euler_from_dcm(dcm_body_to_ned, vulcan::EulerSequence::ZYX);
}
```

---

### Part 4: YAML Config Examples

#### New rocket_ascent.yaml with LLA and Euler angles

```yaml
- type: RigidBody6DOF
  name: EOM
  entity: Rocket
  vectors:
    # Option A: LLA initialization (more intuitive)
    initial_lla: [0.0, 0.0, 100000.0]  # [lat_deg, lon_deg, alt_m]

    # Option B: ECEF initialization (legacy)
    # initial_position: [6.471e6, 0, 0]

    # Option A: Euler angles relative to local NED (more intuitive)
    initial_euler_zyx: [0.0, 0.0, 0.0]  # [yaw_deg, pitch_deg, roll_deg]

    # Option B: Quaternion (legacy)
    # initial_attitude: [1, 0, 0, 0]

    initial_velocity_body: [0, 0, 0]
    initial_omega_body: [0, 0, 0]
```

#### Recording with new outputs

```yaml
recording:
  signals:
    # Traditional ECEF
    - Rocket.EOM.position.x
    - Rocket.EOM.position.y
    - Rocket.EOM.position.z

    # New: Human-readable coordinates
    - Rocket.EOM.position_lla.lat
    - Rocket.EOM.position_lla.lon
    - Rocket.EOM.position_lla.alt

    # New: NED velocity (easier to interpret)
    - Rocket.EOM.velocity_ned.n
    - Rocket.EOM.velocity_ned.e
    - Rocket.EOM.velocity_ned.d

    # New: Euler angles (radians for consistency with other angular signals)
    - Rocket.EOM.euler_zyx.yaw
    - Rocket.EOM.euler_zyx.pitch
    - Rocket.EOM.euler_zyx.roll
```

---

## ECI Output (Future Enhancement)

ECI requires time context (GMST) which the component doesn't currently have. Options:

1. Add optional `time` input signal for GMST/epoch
2. Add `simulation_time` as a standard backplane signal
3. Accept `epoch` config parameter and compute GMST internally

This should be a separate enhancement after the LLA/NED/Euler work is complete.

---

## Files to Modify

| File | Changes |
|------|---------|
| `components/dynamics/RigidBody6DOF.hpp` | Add includes, member vars, output registration, Stage() updates, Step() updates, new setters |
| `config/examples/rocket_ascent.yaml` | Update to use LLA and Euler angles |
| `tests/python/test_rocket_ascent.py` | Update to verify new signals |

## New Test Cases

1. **LLA initialization roundtrip** - Set initial_lla, verify position_lla outputs match
2. **Euler initialization roundtrip** - Set initial_euler_zyx, verify euler_zyx outputs match
3. **NED velocity consistency** - Verify velocity_ned matches transformed velocity_ref
4. **Existing tests still pass** - ECEF and quaternion initialization still work

---

## Verification Plan

1. Build: `./scripts/build.sh`
2. Run C++ tests: `./scripts/test.sh`
3. Run Python tests: `./scripts/run_example.sh tests/python/test_rocket_ascent.py`
4. Update rocket_ascent.yaml to use LLA/Euler, verify simulation runs
5. Check recorded data has correct coordinate values
