#pragma once

/**
 * @file SignalRouter.hpp
 * @brief Centralized signal routing configuration
 *
 * Part of Phase 4.0: Configuration Infrastructure.
 * Handles all signal wiring separately from component code.
 * Components only declare ports; SignalRouter connects them.
 */

#include <icarus/core/Error.hpp>
#include <icarus/signal/Backplane.hpp>

#include <string>
#include <unordered_set>
#include <vector>

namespace icarus {
namespace signal {

/**
 * @brief A single signal route with optional transformations
 *
 * Routes connect a component's input port to another component's output.
 * Optional gain, offset, and delay allow unit conversion, bias, and transport delay.
 */
struct SignalRoute {
    std::string input_path;  ///< Full path: Entity.Component.signal (destination)
    std::string output_path; ///< Full path: Entity.Component.signal (source)

    // Optional transformations
    double gain = 1.0;   ///< Scale factor applied on read
    double offset = 0.0; ///< Bias added after gain
    double delay = 0.0;  ///< Transport delay in seconds

    SignalRoute() = default;
    SignalRoute(std::string input, std::string output, double g = 1.0, double o = 0.0,
                double d = 0.0)
        : input_path(std::move(input)), output_path(std::move(output)), gain(g), offset(o),
          delay(d) {}

    /// Validate route paths
    [[nodiscard]] std::vector<std::string> Validate() const {
        std::vector<std::string> errors;
        if (input_path.empty()) {
            errors.push_back("SignalRoute: input_path is empty");
        }
        if (output_path.empty()) {
            errors.push_back("SignalRoute: output_path is empty");
        }
        if (delay < 0.0) {
            errors.push_back("SignalRoute: delay cannot be negative");
        }
        return errors;
    }
};

/**
 * @brief Centralized signal routing configuration
 *
 * Handles all signal wiring separately from component code.
 * Components only declare ports; SignalRouter connects them.
 *
 * Workflow:
 * 1. Components declare inputs/outputs during Provision
 * 2. Route file specifies all connections
 * 3. SignalRouter validates and applies routes after Provision
 *
 * Example routes YAML:
 * @code
 * routes:
 *   - input: Rocket.EOM.total_force
 *     output: Rocket.Forces.total_force
 *     gain: 1.0
 *   - input: Rocket.Gravity.position
 *     output: Rocket.EOM.position
 * @endcode
 */
template <typename Scalar> class SignalRouter {
  public:
    /**
     * @brief Add a route
     */
    void AddRoute(const SignalRoute &route) { routes_.push_back(route); }

    /**
     * @brief Add a route with parameters
     */
    void AddRoute(const std::string &input_path, const std::string &output_path,
                  double gain = 1.0) {
        routes_.emplace_back(input_path, output_path, gain);
    }

    /**
     * @brief Apply all routes to the backplane
     *
     * Called by Simulator after all components have declared their ports.
     * Validates that all inputs and outputs exist before wiring.
     *
     * @param bp Backplane to wire
     * @throws RoutingError if any signal is not found
     */
    void ApplyRoutes(Backplane<Scalar> &bp) {
        // First pass: validate all signals exist
        auto errors = ValidateRoutes(bp);
        if (!errors.empty()) {
            std::string msg = "Signal routing validation failed:\n";
            for (const auto &err : errors) {
                msg += "  - " + err + "\n";
            }
            throw RoutingError(msg);
        }

        // Second pass: wire with gains
        for (const auto &route : routes_) {
            bp.WireWithGain(route.input_path, route.output_path, route.gain);
        }
    }

    /**
     * @brief Validate routes without applying
     *
     * @param bp Backplane to check against
     * @return List of errors (empty if valid)
     */
    [[nodiscard]] std::vector<std::string> ValidateRoutes(const Backplane<Scalar> &bp) const {
        std::vector<std::string> errors;

        for (const auto &route : routes_) {
            if (!bp.HasOutput(route.output_path)) {
                errors.push_back("Output not found: '" + route.output_path + "'");
            }
            if (!bp.HasInput(route.input_path)) {
                errors.push_back("Input not found: '" + route.input_path + "'");
            }
        }

        return errors;
    }

    /**
     * @brief Get unwired inputs after routing
     *
     * Useful for finding configuration issues.
     *
     * @param bp Backplane to check
     * @return List of input paths that have no route
     */
    [[nodiscard]] std::vector<std::string> GetUnwiredInputs(const Backplane<Scalar> &bp) const {
        // Get all declared inputs from backplane
        auto all_inputs = bp.GetDeclaredInputs();

        // Build set of wired inputs
        std::unordered_set<std::string> wired;
        for (const auto &route : routes_) {
            wired.insert(route.input_path);
        }

        // Find unwired
        std::vector<std::string> unwired;
        for (const auto &input : all_inputs) {
            if (wired.find(input) == wired.end()) {
                unwired.push_back(input);
            }
        }

        return unwired;
    }

    /**
     * @brief Get all routes for inspection
     */
    [[nodiscard]] const std::vector<SignalRoute> &GetRoutes() const { return routes_; }

    /**
     * @brief Clear all routes
     */
    void Clear() { routes_.clear(); }

    /**
     * @brief Get number of routes
     */
    [[nodiscard]] std::size_t Size() const { return routes_.size(); }

  private:
    std::vector<SignalRoute> routes_;
};

} // namespace signal
} // namespace icarus
