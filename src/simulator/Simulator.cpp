/**
 * @file Simulator.cpp
 * @brief Simulator implementation
 *
 * Currently a placeholder - the Simulator is header-only template.
 * This file provides a compilation unit for future non-template utilities.
 */

#include <icarus/sim/Simulator.hpp>

namespace icarus {

// Explicit template instantiation for common types
template class Simulator<double>;

// Note: Simulator<casadi::MX> instantiation will be added
// when CasADi integration is fully implemented.

} // namespace icarus
