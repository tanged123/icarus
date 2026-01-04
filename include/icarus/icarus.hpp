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
#include <icarus/core/AggregationTypes.hpp>
#include <icarus/core/Component.hpp>
#include <icarus/core/ComponentConfig.hpp>
#include <icarus/core/ComponentFactory.hpp>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/core/Error.hpp>
#include <icarus/core/ErrorLogging.hpp>

// Signal
#include <icarus/signal/Backplane.hpp>
#include <icarus/signal/Handle.hpp>
#include <icarus/signal/InputHandle.hpp>
#include <icarus/signal/Registry.hpp>
#include <icarus/signal/Signal.hpp>
#include <icarus/signal/SignalRouter.hpp>
#include <icarus/signal/VecHandle.hpp>

// Simulation
#include <icarus/sim/Scheduler.hpp>
#include <icarus/sim/Simulator.hpp>
#include <icarus/sim/SimulatorConfig.hpp>
#include <icarus/sim/integrators/Integrator.hpp>
#include <icarus/sim/integrators/IntegratorFactory.hpp>
#include <icarus/sim/integrators/IntegratorTypes.hpp>
#include <icarus/sim/integrators/RK45Integrator.hpp>
#include <icarus/sim/integrators/RK4Integrator.hpp>

// Staging (Phase 4)
#include <icarus/staging/StagingTypes.hpp>

// I/O
#include <icarus/io/HDF5Recorder.hpp>
#include <icarus/io/MissionLogger.hpp>
#include <icarus/io/RecordingReader.hpp>
#include <icarus/io/SimulationLoader.hpp>
#include <icarus/io/data/DataDictionary.hpp>
#include <icarus/io/data/Playback.hpp>
#include <icarus/io/data/Recorder.hpp>

// Utilities
#include <icarus/io/ScalarFormat.hpp>

namespace icarus {
// Version functions are defined in core/CoreTypes.hpp
} // namespace icarus
