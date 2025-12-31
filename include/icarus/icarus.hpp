#pragma once

/**
 * @file icarus.hpp
 * @brief Umbrella header for Icarus 6DOF Simulation Engine
 *
 * Include this header to get access to all Icarus public APIs,
 * including Janus (math/optimization) and Vulcan (physics) dependencies.
 */

// Dependencies - include Janus and Vulcan umbrella headers
#include <janus/janus.hpp>
#include <vulcan/vulcan.hpp>

// Core
#include <icarus/core/Component.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/core/Error.hpp>

// Signal
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Handle.hpp>
#include <icarus/signal/InputHandle.hpp>
#include <icarus/signal/Registry.hpp>
#include <icarus/signal/Signal.hpp>
#include <icarus/signal/VecHandle.hpp>

// Simulation
#include <icarus/sim/Scheduler.hpp>
#include <icarus/sim/integrators/Integrator.hpp>
#include <icarus/sim/integrators/IntegratorFactory.hpp>
#include <icarus/sim/integrators/IntegratorTypes.hpp>
#include <icarus/sim/integrators/RK45Integrator.hpp>
#include <icarus/sim/integrators/RK4Integrator.hpp>
// Note: SimulationBuilder and SimulationRunner disabled pending Phase 4.0.7 update
// #include <icarus/sim/SimulationBuilder.hpp>
#include <icarus/sim/SimulationResults.hpp>
// #include <icarus/sim/SimulationRunner.hpp>
#include <icarus/sim/Simulator.hpp>

// I/O
#include <icarus/io/data/DataDictionary.hpp>
#include <icarus/io/data/Playback.hpp>
#include <icarus/io/data/Recorder.hpp>

// Symbolic (Phase 3) - disabled pending Phase 4.0.7 update
// #include <icarus/staging/SymbolicTracer.hpp>

// Utilities
#include <icarus/io/ScalarFormat.hpp>

namespace icarus {
// Version functions are defined in core/CoreTypes.hpp
} // namespace icarus
