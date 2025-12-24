#pragma once

/**
 * @file Registry.hpp
 * @brief Signal Registry (Backplane) for Icarus
 *
 * Part of Phase 1.3: Signal Backplane.
 * Provides signal registration, resolution, and access patterns.
 */

#include <deque>
#include <icarus/core/Config.hpp>
#include <icarus/core/Error.hpp>
#include <icarus/core/Types.hpp>
#include <icarus/signal/Handle.hpp>
#include <icarus/signal/Signal.hpp>
#include <icarus/signal/VecHandle.hpp>
#include <regex>
#include <string>
#include <unordered_map>
#include <vector>

namespace icarus {

/**
 * @brief Central registry for all simulation signals
 *
 * The SignalRegistry manages the signal backplane, providing:
 * - Signal registration during Provision (with pointer binding)
 * - Type-safe signal resolution during Stage (returning SignalHandle)
 * - Fast indexed access during Step (hot path)
 * - Vec3/Mat3 expansion for vector signals
 * - Owner tracking for collision detection
 *
 * @tparam Scalar The numeric type (double or casadi::MX)
 */
template <typename Scalar> class SignalRegistry {
  public:
    using SignalIndex = std::size_t;
    static constexpr SignalIndex InvalidIndex = static_cast<SignalIndex>(-1);

    // =========================================================================
    // Context Management
    // =========================================================================

    /**
     * @brief Set the current component context for registration
     *
     * All signals registered after this call will be attributed to this component.
     *
     * @param name Component name
     */
    void set_current_component(const std::string &name) { current_component_ = name; }

    /**
     * @brief Clear the current component context
     */
    void clear_current_component() { current_component_.clear(); }

    /**
     * @brief Get the current component name
     */
    [[nodiscard]] const std::string &current_component() const { return current_component_; }

    // =========================================================================
    // Pointer-Based Registration (Phase 1.3)
    // =========================================================================

    /**
     * @brief Register an output signal with pointer binding
     *
     * @tparam T The signal value type (must have TypeTraits specialization)
     * @param name Signal path
     * @param data_ptr Pointer to component-owned storage
     * @param unit Physical unit (optional)
     * @param description Human-readable description (optional)
     */
    template <typename T>
    void register_output(const std::string &name, T *data_ptr, const std::string &unit = "",
                         const std::string &description = "") {
        register_signal_impl<T>(name, data_ptr, unit, description, SignalLifecycle::Dynamic);
    }

    /**
     * @brief Register a static (immutable) signal with pointer binding
     *
     * Static signals are set during Provision/Stage and remain constant.
     *
     * @tparam T The signal value type (must have TypeTraits specialization)
     * @param name Signal path
     * @param data_ptr Pointer to component-owned storage (const for static)
     * @param unit Physical unit (optional)
     * @param description Human-readable description (optional)
     */
    template <typename T>
    void register_static(const std::string &name, const T *data_ptr, const std::string &unit = "",
                         const std::string &description = "") {
        // Cast away const for storage (we track lifecycle separately)
        register_signal_impl<T>(name, const_cast<T *>(data_ptr), unit, description,
                                SignalLifecycle::Static);
    }

    /**
     * @brief Register a Vec3 signal as three scalar components
     *
     * Creates three signals with `.x`, `.y`, `.z` suffixes.
     *
     * @tparam S The scalar type (typically same as Scalar template)
     * @param name Base signal path (e.g., "position")
     * @param data_ptr Pointer to Vec3 storage
     * @param unit Physical unit (optional)
     * @param description Human-readable description (optional)
     */
    template <typename S>
    void register_vec3(const std::string &name, Vec3<S> *data_ptr, const std::string &unit = "",
                       const std::string &description = "") {
        if (data_ptr == nullptr) {
            throw SignalError("Null data_ptr for Vec3 signal '" + name + "'");
        }
        // Register three scalar signals pointing to Vec3 elements
        register_signal_impl<S>(name + ".x", &((*data_ptr)(0)), unit, description + " (x)",
                                SignalLifecycle::Dynamic);
        register_signal_impl<S>(name + ".y", &((*data_ptr)(1)), unit, description + " (y)",
                                SignalLifecycle::Dynamic);
        register_signal_impl<S>(name + ".z", &((*data_ptr)(2)), unit, description + " (z)",
                                SignalLifecycle::Dynamic);
    }

    /**
     * @brief Register a Vec4/Quaternion signal as four scalar components
     *
     * Creates four signals with `.w`, `.x`, `.y`, `.z` suffixes.
     */
    template <typename S>
    void register_quat(const std::string &name, Vec4<S> *data_ptr, const std::string &unit = "",
                       const std::string &description = "") {
        if (data_ptr == nullptr) {
            throw SignalError("Null data_ptr for Quat signal '" + name + "'");
        }
        register_signal_impl<S>(name + ".w", &((*data_ptr)(0)), unit, description + " (w)",
                                SignalLifecycle::Dynamic);
        register_signal_impl<S>(name + ".x", &((*data_ptr)(1)), unit, description + " (x)",
                                SignalLifecycle::Dynamic);
        register_signal_impl<S>(name + ".y", &((*data_ptr)(2)), unit, description + " (y)",
                                SignalLifecycle::Dynamic);
        register_signal_impl<S>(name + ".z", &((*data_ptr)(3)), unit, description + " (z)",
                                SignalLifecycle::Dynamic);
    }

    // =========================================================================
    // Type-Safe Resolution (Phase 1.3)
    // =========================================================================

    /**
     * @brief Resolve a signal by name and return a type-safe handle
     *
     * @tparam T The expected signal value type
     * @param name Signal path
     * @return SignalHandle<T> for zero-overhead access
     * @throws SignalNotFoundError if signal doesn't exist
     * @throws TypeMismatchError if type doesn't match registered type
     */
    template <typename T> [[nodiscard]] SignalHandle<T> resolve(const std::string &name) {
        auto it = name_to_index_.find(name);
        if (it == name_to_index_.end()) {
            throw SignalNotFoundError(name);
        }

        SignalIndex index = it->second;
        const SignalDescriptor &desc = signals_[index];

        // Type check
        constexpr SignalType expected_type = TypeTraits<T>::type_id;
        if (desc.type != expected_type) {
            throw TypeMismatchError(name, TypeTraits<T>::name, signal_type_name(desc.type));
        }

        return SignalHandle<T>(static_cast<T *>(desc.data_ptr), &desc);
    }

    /**
     * @brief Resolve a signal as const (for read-only access)
     */
    template <typename T>
    [[nodiscard]] SignalHandle<const T> resolve_const(const std::string &name) const {
        auto it = name_to_index_.find(name);
        if (it == name_to_index_.end()) {
            throw SignalNotFoundError(name);
        }

        SignalIndex index = it->second;
        const SignalDescriptor &desc = signals_[index];

        // Type check
        constexpr SignalType expected_type = TypeTraits<T>::type_id;
        if (desc.type != expected_type) {
            throw TypeMismatchError(name, TypeTraits<T>::name, signal_type_name(desc.type));
        }

        return SignalHandle<const T>(static_cast<const T *>(desc.data_ptr), &desc);
    }

    /**
     * @brief Resolve a Vec3 signal as a Vec3Handle
     *
     * @tparam S The scalar type
     * @param name Base signal path (without .x/.y/.z suffix)
     * @return Vec3Handle<S> with handles for each component
     */
    template <typename S> [[nodiscard]] Vec3Handle<S> resolve_vec3(const std::string &name) {
        return Vec3Handle<S>{resolve<S>(name + ".x"), resolve<S>(name + ".y"),
                             resolve<S>(name + ".z")};
    }

    /**
     * @brief Resolve a quaternion signal as a QuatHandle
     */
    template <typename S> [[nodiscard]] QuatHandle<S> resolve_quat(const std::string &name) {
        return QuatHandle<S>{resolve<S>(name + ".w"), resolve<S>(name + ".x"),
                             resolve<S>(name + ".y"), resolve<S>(name + ".z")};
    }

    // =========================================================================
    // Legacy API (Backward Compatibility)
    // =========================================================================

    /**
     * @brief Register a new signal output (legacy API)
     *
     * Called during Provision phase to declare a signal.
     * Creates internal storage managed by the registry.
     *
     * @param descriptor Signal metadata
     * @return Index for fast access during Step
     */
    SignalIndex RegisterSignal(const SignalDescriptor &descriptor) {
        if (name_to_index_.contains(descriptor.name)) {
            const auto &existing = signals_[name_to_index_[descriptor.name]];
            throw DuplicateSignalError(descriptor.name, existing.owner_component,
                                       current_component_.empty() ? "(unknown)"
                                                                  : current_component_);
        }

        SignalIndex index = signals_.size();
        signals_.push_back(descriptor);
        signals_.back().owner_component = current_component_;
        values_.push_back(Scalar{});

        // For legacy API, point data_ptr at our internal storage
        signals_.back().data_ptr = &values_.back();

        name_to_index_[descriptor.name] = index;
        return index;
    }

    /**
     * @brief Resolve a signal by name (legacy API)
     *
     * @param name Full signal path
     * @return Index for fast access during Step
     */
    [[nodiscard]] SignalIndex Resolve(const std::string &name) const {
        auto it = name_to_index_.find(name);
        if (it == name_to_index_.end()) {
            throw SignalNotFoundError(name);
        }
        return it->second;
    }

    /**
     * @brief Check if a signal exists
     */
    [[nodiscard]] bool HasSignal(const std::string &name) const {
        return name_to_index_.contains(name);
    }

    /**
     * @brief Get signal value by index (hot path, legacy)
     */
    [[nodiscard]] const Scalar &Get(SignalIndex index) const { return values_[index]; }

    /**
     * @brief Set signal value by index (hot path, legacy)
     */
    void Set(SignalIndex index, const Scalar &value) { values_[index] = value; }

    /**
     * @brief Get signal value by name (slow path, for debugging)
     */
    [[nodiscard]] const Scalar &GetByName(const std::string &name) const {
        const SignalDescriptor &desc = signals_[Resolve(name)];
        if (desc.data_ptr == nullptr) {
            throw SignalError("Null data_ptr for signal '" + name + "'");
        }
        return *static_cast<const Scalar *>(desc.data_ptr);
    }

    /**
     * @brief Set signal value by name (slow path, for initialization)
     */
    void SetByName(const std::string &name, const Scalar &value) {
        const SignalDescriptor &desc = signals_[Resolve(name)];
        if (desc.data_ptr == nullptr) {
            throw SignalError("Null data_ptr for signal '" + name + "'");
        }
        *static_cast<Scalar *>(desc.data_ptr) = value;
    }

    // =========================================================================
    // Query / Introspection (Cold Path)
    // =========================================================================

    /**
     * @brief Get all signal descriptors
     */
    [[nodiscard]] const std::deque<SignalDescriptor> &GetDescriptors() const { return signals_; }

    /**
     * @brief Get number of registered signals
     */
    [[nodiscard]] std::size_t Size() const { return signals_.size(); }

    /**
     * @brief Query signals matching a pattern
     *
     * @param pattern Regex pattern to match signal names
     * @return Vector of pointers to matching descriptors
     * @throws std::regex_error if pattern is invalid
     */
    [[nodiscard]] std::vector<const SignalDescriptor *> query(const std::string &pattern) const {
        std::vector<const SignalDescriptor *> results;
        std::regex re(pattern);
        for (const auto &desc : signals_) {
            if (std::regex_search(desc.name, re)) {
                results.push_back(&desc);
            }
        }
        return results;
    }

    /**
     * @brief Get descriptor by name
     */
    [[nodiscard]] const SignalDescriptor *get_descriptor(const std::string &name) const {
        auto it = name_to_index_.find(name);
        if (it == name_to_index_.end()) {
            return nullptr;
        }
        return &signals_[it->second];
    }

  private:
    // =========================================================================
    // Implementation Helpers
    // =========================================================================

    template <typename T>
    void register_signal_impl(const std::string &name, T *data_ptr, const std::string &unit,
                              const std::string &description, SignalLifecycle lifecycle) {
        if (data_ptr == nullptr) {
            throw SignalError("Null data_ptr for signal '" + name + "'");
        }

        if (name_to_index_.contains(name)) {
            const auto &existing = signals_[name_to_index_[name]];
            throw DuplicateSignalError(name, existing.owner_component,
                                       current_component_.empty() ? "(unknown)"
                                                                  : current_component_);
        }

        SignalDescriptor desc;
        desc.name = name;
        desc.unit = unit;
        desc.type = TypeTraits<T>::type_id;
        desc.lifecycle = lifecycle;
        desc.description = description;
        desc.owner_component = current_component_;
        desc.data_ptr = data_ptr;

        SignalIndex index = signals_.size();
        signals_.push_back(std::move(desc));
        name_to_index_[name] = index;
    }

    [[nodiscard]] static const char *signal_type_name(SignalType type) {
        switch (type) {
        case SignalType::Double:
            return "Double";
        case SignalType::Int32:
            return "Int32";
        case SignalType::Int64:
            return "Int64";
        default:
            return "Unknown";
        }
    }

    // =========================================================================
    // Data Members
    // =========================================================================

    std::deque<SignalDescriptor> signals_; // Deque for stable descriptor refs
    std::deque<Scalar> values_;            // Deque for stable value refs (legacy API)
    std::unordered_map<std::string, SignalIndex> name_to_index_;
    std::string current_component_;
};

} // namespace icarus
