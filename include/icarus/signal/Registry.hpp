#pragma once

#include <icarus/core/Error.hpp>
#include <icarus/signal/Signal.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace icarus {

/**
 * @brief Central registry for all simulation signals.
 *
 * The SignalRegistry manages the signal backplane, providing:
 * - Signal registration during Provision
 * - Signal resolution during Stage
 * - Fast indexed access during Step
 *
 * @tparam Scalar The numeric type (double or casadi::MX)
 */
template <typename Scalar> class SignalRegistry {
  public:
    using SignalIndex = std::size_t;
    static constexpr SignalIndex InvalidIndex = static_cast<SignalIndex>(-1);

    /**
     * @brief Register a new signal output.
     *
     * Called during Provision phase to declare a signal.
     *
     * @param descriptor Signal metadata
     * @return Index for fast access during Step
     */
    SignalIndex RegisterSignal(const SignalDescriptor &descriptor) {
        if (name_to_index_.contains(descriptor.name)) {
            throw SignalError("Signal already registered: " + descriptor.name);
        }

        SignalIndex index = signals_.size();
        signals_.push_back(descriptor);
        values_.push_back(Scalar{});
        name_to_index_[descriptor.name] = index;
        return index;
    }

    /**
     * @brief Resolve a signal by name.
     *
     * Called during Stage phase to get index for later access.
     *
     * @param name Full signal path
     * @return Index for fast access during Step
     */
    [[nodiscard]] SignalIndex Resolve(const std::string &name) const {
        auto it = name_to_index_.find(name);
        if (it == name_to_index_.end()) {
            throw SignalError("Signal not found: " + name);
        }
        return it->second;
    }

    /**
     * @brief Check if a signal exists.
     */
    [[nodiscard]] bool HasSignal(const std::string &name) const {
        return name_to_index_.contains(name);
    }

    /**
     * @brief Get signal value by index (hot path).
     */
    [[nodiscard]] const Scalar &Get(SignalIndex index) const { return values_[index]; }

    /**
     * @brief Set signal value by index (hot path).
     */
    void Set(SignalIndex index, const Scalar &value) { values_[index] = value; }

    /**
     * @brief Get signal value by name (slow path, for debugging).
     */
    [[nodiscard]] const Scalar &GetByName(const std::string &name) const {
        return values_[Resolve(name)];
    }

    /**
     * @brief Set signal value by name (slow path, for initialization).
     */
    void SetByName(const std::string &name, const Scalar &value) { values_[Resolve(name)] = value; }

    /**
     * @brief Get all signal descriptors.
     */
    [[nodiscard]] const std::vector<SignalDescriptor> &GetDescriptors() const { return signals_; }

    /**
     * @brief Get number of registered signals.
     */
    [[nodiscard]] std::size_t Size() const { return signals_.size(); }

  private:
    std::vector<SignalDescriptor> signals_;
    std::vector<Scalar> values_;
    std::unordered_map<std::string, SignalIndex> name_to_index_;
};

} // namespace icarus
