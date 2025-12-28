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
#include <icarus/core/Concepts.hpp>
#include <icarus/core/Error.hpp>
#include <icarus/core/Types.hpp>

// Signal
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Descriptor.hpp>
#include <icarus/signal/Handle.hpp>
#include <icarus/signal/InputHandle.hpp>
#include <icarus/signal/Registry.hpp>
#include <icarus/signal/Signal.hpp>
#include <icarus/signal/VecHandle.hpp>

// Simulation
#include <icarus/sim/Integrator.hpp>
#include <icarus/sim/IntegratorFactory.hpp>
#include <icarus/sim/IntegratorTypes.hpp>
#include <icarus/sim/RK45Integrator.hpp>
#include <icarus/sim/RK4Integrator.hpp>
#include <icarus/sim/Scheduler.hpp>
#include <icarus/sim/SimulationBuilder.hpp>
#include <icarus/sim/SimulationResults.hpp>
#include <icarus/sim/SimulationRunner.hpp>
#include <icarus/sim/Simulator.hpp>

// I/O
#include <icarus/io/Config.hpp>
#include <icarus/io/DataDictionary.hpp>
#include <icarus/io/Playback.hpp>
#include <icarus/io/Recorder.hpp>

// Symbolic (Phase 3)
#include <icarus/symbolic/SymbolicTracer.hpp>

// Utilities
#include <icarus/util/ScalarFormat.hpp>

namespace icarus {
// Version functions are defined in core/Types.hpp
} // namespace icarus
