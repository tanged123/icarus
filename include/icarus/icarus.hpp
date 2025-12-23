#pragma once

/**
 * @file icarus.hpp
 * @brief Umbrella header for Icarus 6DOF Simulation Engine
 *
 * Include this header to get access to all Icarus public APIs.
 */

// Core
#include <icarus/core/Component.hpp>
#include <icarus/core/Concepts.hpp>
#include <icarus/core/Error.hpp>
#include <icarus/core/Types.hpp>

// Signal
#include <icarus/signal/Descriptor.hpp>
#include <icarus/signal/Registry.hpp>
#include <icarus/signal/Signal.hpp>

// Simulation
#include <icarus/sim/Integrator.hpp>
#include <icarus/sim/Scheduler.hpp>
#include <icarus/sim/Simulator.hpp>

// I/O
#include <icarus/io/Config.hpp>
#include <icarus/io/Playback.hpp>
#include <icarus/io/Recorder.hpp>

namespace icarus {
// Version functions are defined in core/Types.hpp
} // namespace icarus
