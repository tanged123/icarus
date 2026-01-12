# Component Registration Cleanup

## Problem

In [registration.cpp](file:///home/tanged/sources/icarus/components/registration.cpp), every component is registered **twice** - once for `NumericScalar` and once for `SymbolicScalar`. The code is nearly identical, differing only in the template parameter. This leads to ~180 lines when ~40 would suffice.

```diff
 // Current: 9 components × 2 backends × ~8 lines each ≈ 144 lines of registration
-const bool registered_numeric = []() {
-    factory.Register("PointMass3DOF", [](const ::icarus::ComponentConfig &config) {
-        auto comp = std::make_unique<::icarus::components::PointMass3DOF<janus::NumericScalar>>(
-            config.name, config.entity);
-        comp->SetConfig(config);
-        return comp;
-    });
-    // ... repeat for all 9 components ...
-}();
-
-const bool registered_symbolic = []() {
-    factory.Register("PointMass3DOF", [](const ::icarus::ComponentConfig &config) {
-        auto comp = std::make_unique<::icarus::components::PointMass3DOF<janus::SymbolicScalar>>(
-            config.name, config.entity);
-        comp->SetConfig(config);
-        return comp;
-    });
-    // ... repeat for all 9 components (identical except template param) ...
-}();
```

## Solution

Use the existing `ICARUS_REGISTER_COMPONENT_AS` macro from [ComponentFactory.hpp](file:///home/tanged/sources/icarus/include/icarus/core/ComponentFactory.hpp), which already handles dual-backend registration internally.

## Proposed Changes

### [MODIFY] [registration.cpp](file:///home/tanged/sources/icarus/components/registration.cpp)

Replace the two 70+ line lambda blocks with simple macro invocations:

```cpp
#include <icarus/core/ComponentFactory.hpp>

#include <aggregators/ForceAggregator.hpp>
#include <aggregators/MassAggregator.hpp>
#include <dynamics/PointMass3DOF.hpp>
#include <dynamics/RigidBody6DOF.hpp>
#include <environment/AtmosphericDrag.hpp>
#include <environment/PointMassGravity.hpp>
#include <mass/StaticMass.hpp>
#include <propulsion/FuelTank.hpp>
#include <propulsion/RocketEngine.hpp>

// Dynamics
ICARUS_REGISTER_COMPONENT_AS(icarus::components::PointMass3DOF, "PointMass3DOF")
ICARUS_REGISTER_COMPONENT_AS(icarus::components::RigidBody6DOF, "RigidBody6DOF")

// Environment
ICARUS_REGISTER_COMPONENT_AS(icarus::components::PointMassGravity, "PointMassGravity")
ICARUS_REGISTER_COMPONENT_AS(icarus::components::AtmosphericDrag, "AtmosphericDrag")

// Mass
ICARUS_REGISTER_COMPONENT_AS(icarus::components::StaticMass, "StaticMass")

// Aggregators
ICARUS_REGISTER_COMPONENT_AS(icarus::components::MassAggregator, "MassAggregator")
ICARUS_REGISTER_COMPONENT_AS(icarus::components::ForceAggregator, "ForceAggregator")

// Propulsion
ICARUS_REGISTER_COMPONENT_AS(icarus::components::RocketEngine, "RocketEngine")
ICARUS_REGISTER_COMPONENT_AS(icarus::components::FuelTank, "FuelTank")
```

**Result:** ~30 lines instead of ~180 lines (83% reduction).

## Verification Plan

1. Run `./scripts/build.sh` - must compile successfully
2. Run `./scripts/test.sh` - existing component factory tests must pass
3. Run an example that uses YAML loading (e.g., `rocket_ascent`) to ensure components are instantiated correctly
