#pragma once

#include <icarus/core/Error.hpp>
#include <icarus/core/Types.hpp>
#include <string>

namespace icarus {

// Forward declarations
template <typename Scalar> class SignalRegistry;

/**
 * @brief Base class for all simulation components.
 *
 * Components are the fundamental unit of execution in Icarus. They own
 * state and implement the Provision/Stage/Step lifecycle.
 *
 * @tparam Scalar The numeric type (double or casadi::MX)
 */
template <typename Scalar> class Component {
  public:
    virtual ~Component() = default;

    /**
     * @brief Provision phase - called once at application launch.
     *
     * Heavy lifting: allocate memory, register signals, parse config.
     *
     * @param registry Signal registry for registering outputs
     * @param config Component configuration
     */
    virtual void Provision(SignalRegistry<Scalar> &registry, const ComponentConfig &config) = 0;

    /**
     * @brief Stage phase - called at start of each run/episode.
     *
     * Wire inputs, apply initial conditions, prepare for t=0.
     *
     * @param registry Signal registry for resolving inputs
     */
    virtual void Stage(SignalRegistry<Scalar> &registry) = 0;

    /**
     * @brief Step phase - called every time step (hot path!).
     *
     * Read inputs, compute derivatives, write outputs.
     * NO allocation, NO string lookups.
     *
     * @param t Current simulation time
     * @param dt Time step size
     */
    virtual void Step(Scalar t, Scalar dt) = 0;

    // Optional lifecycle hooks
    virtual void PreStep(Scalar /*t*/, Scalar /*dt*/) {}
    virtual void PostStep(Scalar /*t*/, Scalar /*dt*/) {}
    virtual void OnPhaseEnter(Phase /*phase*/) {}
    virtual void OnPhaseExit(Phase /*phase*/) {}
    virtual void OnError(const Error & /*error*/) {}
    virtual void Shutdown() {}

    // Metadata
    [[nodiscard]] virtual std::string Name() const = 0;
    [[nodiscard]] virtual std::string Entity() const { return ""; }
};

} // namespace icarus
