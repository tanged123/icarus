/**
 * @file Backplane.cpp
 * @brief Signal backplane implementation
 *
 * Currently a placeholder - the SignalRegistry is header-only template.
 * This file provides a compilation unit for future non-template utilities.
 */

#include <icarus/signal/Registry.hpp>

namespace icarus {

// Explicit template instantiation for common types
template class SignalRegistry<double>;

// Note: SignalRegistry<casadi::MX> instantiation will be added
// when CasADi integration is fully implemented.

} // namespace icarus
