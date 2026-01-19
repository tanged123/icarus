#pragma once

/**
 * @file Registry.hpp
 * @brief Signal Registry (Backplane) for Icarus
 *
 * Part of Phase 1.3: Signal Backplane.
 * Provides signal registration, resolution, and access patterns.
 */

#include <deque>
#include <icarus/core/CoreTypes.hpp>
#include <icarus/core/Error.hpp>
#include <icarus/signal/Handle.hpp>
#include <icarus/signal/InputHandle.hpp>
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
     * @brief Register a Vec3 output signal as three scalar components
     *
     * Creates three output signals with `.x`, `.y`, `.z` suffixes.
     *
     * @tparam S The scalar type (typically same as Scalar template)
     * @param name Base signal path (e.g., "position")
     * @param data_ptr Pointer to Vec3 storage
     * @param unit Physical unit (optional)
     * @param description Human-readable description (optional)
     */
    template <typename S>
    void register_output_vec3(const std::string &name, Vec3<S> *data_ptr,
                              const std::string &unit = "", const std::string &description = "") {
        if (data_ptr == nullptr) {
            throw SignalError::NullPointer(name, "Vec3 output");
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
     * @brief Register a Vec4/Quaternion output signal as four scalar components
     *
     * Creates four output signals with `.w`, `.x`, `.y`, `.z` suffixes.
     */
    template <typename S>
    void register_output_quat(const std::string &name, Vec4<S> *data_ptr,
                              const std::string &unit = "", const std::string &description = "") {
        if (data_ptr == nullptr) {
            throw SignalError::NullPointer(name, "Quat output");
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
    // Phase 6: State Registration (Unified Signal Model)
    // =========================================================================

    /**
     * @brief Register a scalar state with its derivative
     *
     * Creates two signals: the state value (integrable) and its derivative.
     * StateManager will discover these and build the integration vector.
     *
     * @tparam T The scalar type
     * @param name State name (e.g., "altitude")
     * @param value Pointer to state value storage
     * @param derivative Pointer to derivative storage
     * @param unit Physical unit
     * @param description Human-readable description
     */
    template <typename T>
    void register_state(const std::string &name, T *value, T *derivative,
                        const std::string &unit = "", const std::string &description = "") {
        if (value == nullptr) {
            throw SignalError::NullPointer(name, "state value");
        }
        if (derivative == nullptr) {
            throw SignalError::NullPointer(name + "_dot", "state derivative");
        }

        std::string value_name = name;
        std::string deriv_name = name + "_dot";

        // Register value signal (integrable)
        register_signal_impl<T>(value_name, value, unit, description, SignalLifecycle::Dynamic);
        auto &value_desc = signals_[name_to_index_[value_name]];
        value_desc.is_integrable = true;
        value_desc.derivative_signal = deriv_name;

        // Register derivative signal
        std::string deriv_unit = unit.empty() ? "" : unit + "/s";
        register_signal_impl<T>(deriv_name, derivative, deriv_unit, description + " derivative",
                                SignalLifecycle::Dynamic);
        auto &deriv_desc = signals_[name_to_index_[deriv_name]];
        deriv_desc.integrated_signal = value_name;

        // Track the state pair
        state_pairs_.push_back({value_name, deriv_name, value, derivative});
    }

    /**
     * @brief Register a Vec3 state with its derivative
     *
     * Creates 6 signals: 3 for value (.x/.y/.z) and 3 for derivative (_dot.x/.y/.z).
     */
    template <typename S>
    void register_state_vec3(const std::string &name, Vec3<S> *value, Vec3<S> *derivative,
                             const std::string &unit = "", const std::string &description = "") {
        if (value == nullptr) {
            throw SignalError::NullPointer(name, "Vec3 state value");
        }
        if (derivative == nullptr) {
            throw SignalError::NullPointer(name + "_dot", "Vec3 state derivative");
        }

        std::string deriv_unit = unit.empty() ? "" : unit + "/s";

        // Register each component as a state pair
        const char *suffixes[] = {"x", "y", "z"};
        for (int i = 0; i < 3; ++i) {
            std::string value_name = name + "." + suffixes[i];
            std::string deriv_name = name + "_dot." + suffixes[i];

            // Register value (integrable)
            register_signal_impl<S>(value_name, &((*value)(i)), unit,
                                    description + " (" + suffixes[i] + ")",
                                    SignalLifecycle::Dynamic);
            auto &value_desc = signals_[name_to_index_[value_name]];
            value_desc.is_integrable = true;
            value_desc.derivative_signal = deriv_name;

            // Register derivative
            register_signal_impl<S>(deriv_name, &((*derivative)(i)), deriv_unit,
                                    description + " derivative (" + suffixes[i] + ")",
                                    SignalLifecycle::Dynamic);
            auto &deriv_desc = signals_[name_to_index_[deriv_name]];
            deriv_desc.integrated_signal = value_name;

            // Track the pair
            state_pairs_.push_back({value_name, deriv_name, &((*value)(i)), &((*derivative)(i))});
        }
    }

    /**
     * @brief Register a quaternion state with its derivative
     *
     * Creates 8 signals: 4 for value (.w/.x/.y/.z) and 4 for derivative (_dot.w/.x/.y/.z).
     */
    template <typename S>
    void register_state_quat(const std::string &name, Vec4<S> *value, Vec4<S> *derivative,
                             const std::string &unit = "", const std::string &description = "") {
        if (value == nullptr) {
            throw SignalError::NullPointer(name, "Quat state value");
        }
        if (derivative == nullptr) {
            throw SignalError::NullPointer(name + "_dot", "Quat state derivative");
        }

        // Quaternion components: w, x, y, z (indices 0, 1, 2, 3)
        const char *suffixes[] = {"w", "x", "y", "z"};
        for (int i = 0; i < 4; ++i) {
            std::string value_name = name + "." + suffixes[i];
            std::string deriv_name = name + "_dot." + suffixes[i];

            // Register value (integrable)
            register_signal_impl<S>(value_name, &((*value)(i)), unit,
                                    description + " (" + suffixes[i] + ")",
                                    SignalLifecycle::Dynamic);
            auto &value_desc = signals_[name_to_index_[value_name]];
            value_desc.is_integrable = true;
            value_desc.derivative_signal = deriv_name;

            // Register derivative
            register_signal_impl<S>(deriv_name, &((*derivative)(i)), "",
                                    description + " derivative (" + suffixes[i] + ")",
                                    SignalLifecycle::Dynamic);
            auto &deriv_desc = signals_[name_to_index_[deriv_name]];
            deriv_desc.integrated_signal = value_name;

            // Track the pair
            state_pairs_.push_back({value_name, deriv_name, &((*value)(i)), &((*derivative)(i))});
        }
    }

    /**
     * @brief Get all integrable state pairs
     *
     * Returns pairs of (value_ptr, derivative_ptr) for building integration vectors.
     * Used by StateManager::DiscoverStates().
     */
    [[nodiscard]] const auto &get_state_pairs() const { return state_pairs_; }

    /**
     * @brief Get all integrable signal descriptors
     *
     * Returns descriptors for signals where is_integrable == true.
     */
    [[nodiscard]] std::vector<const SignalDescriptor *> get_integrable_signals() const {
        std::vector<const SignalDescriptor *> result;
        for (const auto &desc : signals_) {
            if (desc.is_integrable) {
                result.push_back(&desc);
            }
        }
        return result;
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
    // @deprecated Use register_output<T>() and resolve<T>() instead.
    // These methods are retained only for test compatibility.
    // The legacy API uses registry-managed storage (values_ deque) rather than
    // component-owned pointers, which is less efficient and type-unsafe.
    // =========================================================================

    /**
     * @brief Register a new signal output (legacy API)
     * @deprecated Use register_output<T>() instead for pointer-based registration
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
     * @deprecated Use resolve<T>() for type-safe handle-based access
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
     * @deprecated Use SignalHandle<T> for direct pointer access
     */
    [[nodiscard]] const Scalar &Get(SignalIndex index) const { return values_[index]; }

    /**
     * @brief Set signal value by index (hot path, legacy)
     * @deprecated Use SignalHandle<T> for direct pointer access
     */
    void Set(SignalIndex index, const Scalar &value) { values_[index] = value; }

    /**
     * @brief Get signal value by name (slow path, for debugging)
     */
    [[nodiscard]] const Scalar &GetByName(const std::string &name) const {
        const SignalDescriptor &desc = signals_[Resolve(name)];
        if (desc.data_ptr == nullptr) {
            throw SignalError::NullPointer(name, "output");
        }
        return *static_cast<const Scalar *>(desc.data_ptr);
    }

    /**
     * @brief Set signal value by name (slow path, for initialization)
     */
    void SetByName(const std::string &name, const Scalar &value) {
        const SignalDescriptor &desc = signals_[Resolve(name)];
        if (desc.data_ptr == nullptr) {
            throw SignalError::NullPointer(name, "output");
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
        std::regex re;
        try {
            re = std::regex(pattern);
        } catch (const std::regex_error &e) {
            throw SignalError("Invalid regex pattern '" + pattern + "': " + e.what());
        }
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
            throw SignalError::NullPointer(name, "output");
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

    // Phase 6: State pair tracking for integration
    struct StatePair {
        std::string value_name;
        std::string derivative_name;
        void *value_ptr = nullptr;
        void *derivative_ptr = nullptr;
    };
    std::vector<StatePair> state_pairs_;

    // =========================================================================
    // Phase 2.4: Input, Parameter, Config Storage
    // =========================================================================

    /// Input port entry (stores handle pointer and metadata)
    struct InputEntry {
        void *handle_ptr = nullptr; // Type-erased InputHandle<T>*
        SignalDescriptor info;
    };

    /// Parameter entry (Scalar-typed, optimizable)
    struct ParamEntry {
        Scalar *storage = nullptr;
        Scalar initial_value{};
        SignalDescriptor info;
    };

    /// Config entry (int/bool/enum, discrete, not optimizable)
    struct ConfigEntry {
        void *storage = nullptr; // Type-erased pointer
        SignalDescriptor info;
    };

    std::unordered_map<std::string, InputEntry> inputs_;
    std::unordered_map<std::string, ParamEntry> params_;
    std::unordered_map<std::string, ConfigEntry> config_;

  public:
    // =========================================================================
    // Phase 2.4: Input Registration
    // =========================================================================

    /**
     * @brief Register an input port
     *
     * Inputs are declared at Provision and wired at Stage.
     *
     * @tparam T The value type (Scalar, Vec3<Scalar>, etc.)
     * @param name Local signal name
     * @param handle Pointer to component-owned InputHandle
     * @param units Physical units
     * @param description Human-readable description
     */
    template <typename T>
    void register_input(const std::string &name, InputHandle<T> *handle,
                        const std::string &units = "", const std::string &description = "") {
        if (handle == nullptr) {
            throw SignalError::NullPointer(name, "input handle");
        }

        if (inputs_.contains(name)) {
            throw DuplicateSignalError(name, inputs_[name].info.name, current_component_);
        }

        // Set up handle metadata
        handle->set_name(name);
        handle->set_full_name(name); // Will be prefixed by Backplane
        handle->set_units(units);
        handle->set_description(description);

        InputEntry entry;
        entry.handle_ptr = handle;
        entry.info.name = name;
        entry.info.unit = units;
        entry.info.description = description;
        entry.info.kind = SignalKind::Input;
        entry.info.semantic = typeid(T).name();
        entry.info.owner_component = current_component_;

        inputs_[name] = std::move(entry);

        // Also register input in signal lookup for poke support
        // The data_ptr points to the input's default value buffer
        SignalDescriptor desc;
        desc.name = name;
        desc.unit = units;
        desc.description = description;
        desc.kind = SignalKind::Input;
        desc.type = TypeTraits<T>::type_id;
        desc.semantic = typeid(T).name();
        desc.owner_component = current_component_;
        desc.data_ptr = static_cast<void *>(handle->data_ptr());

        SignalIndex index = signals_.size();
        signals_.push_back(std::move(desc));
        name_to_index_[name] = index;
    }

    // =========================================================================
    // Phase 2.4: Parameter Registration (Scalar-typed, optimizable)
    // =========================================================================

    /**
     * @brief Register a parameter
     *
     * Parameters are Scalar-typed and can be optimization variables.
     *
     * @param name Local parameter name
     * @param storage Pointer to component-owned Scalar storage
     * @param initial_value Initial value
     * @param units Physical units
     * @param description Human-readable description
     */
    void register_param(const std::string &name, Scalar *storage, Scalar initial_value,
                        const std::string &units = "", const std::string &description = "") {
        if (storage == nullptr) {
            throw SignalError::NullPointer(name, "param");
        }

        if (params_.contains(name)) {
            throw DuplicateSignalError(name, params_[name].info.name, current_component_);
        }

        *storage = initial_value;

        ParamEntry entry;
        entry.storage = storage;
        entry.initial_value = initial_value;
        entry.info.name = name;
        entry.info.unit = units;
        entry.info.description = description;
        entry.info.kind = SignalKind::Parameter;
        entry.info.semantic = "Scalar";
        entry.info.is_optimizable = true;
        entry.info.owner_component = current_component_;

        params_[name] = std::move(entry);
    }

    // =========================================================================
    // Phase 2.4: Config Registration (discrete, not optimizable)
    // =========================================================================

    /**
     * @brief Register an int config value
     */
    void register_config(const std::string &name, int *storage, int initial_value,
                         const std::string &description = "") {
        register_config_impl(name, storage, initial_value, "int", description);
    }

    /**
     * @brief Register a bool config value
     */
    void register_config(const std::string &name, bool *storage, bool initial_value,
                         const std::string &description = "") {
        register_config_impl(name, storage, initial_value, "bool", description);
    }

    // =========================================================================
    // Phase 2.4: Input Wiring
    // =========================================================================

    /**
     * @brief Wire an input to a source signal
     *
     * @param input_name Full name of the input port
     * @param source_name Full name of the source signal
     * @throws SignalNotFoundError if source doesn't exist
     * @throws WiringError if input not found
     */
    template <typename T>
    void wire_input(const std::string &input_name, const std::string &source_name) {
        auto it = inputs_.find(input_name);
        if (it == inputs_.end()) {
            throw WiringError("Input not found: '" + input_name + "'");
        }

        // Verify type matches what was registered (prevents undefined behavior)
        const std::string expected_type = typeid(T).name();
        if (it->second.info.semantic != expected_type) {
            throw SignalError::TypeMismatch(input_name, it->second.info.semantic, expected_type);
        }

        // Resolve the source signal
        auto source_handle = resolve<T>(source_name);

        // Wire the input - safe cast after type verification
        auto *handle = static_cast<InputHandle<T> *>(it->second.handle_ptr);
        handle->wire(source_handle.ptr(), source_name);
        it->second.info.wired_to = source_name;
    }

    // =========================================================================
    // Phase 2.4: Validation
    // =========================================================================

    /**
     * @brief Get list of unwired input names
     */
    [[nodiscard]] std::vector<std::string> get_unwired_inputs() const {
        std::vector<std::string> unwired;
        for (const auto &[name, entry] : inputs_) {
            if (entry.info.wired_to.empty()) {
                unwired.push_back(entry.info.name);
            }
        }
        return unwired;
    }

    /**
     * @brief Validate all inputs are wired
     * @throws WiringError with list of unwired inputs
     */
    void validate_wiring() const {
        auto unwired = get_unwired_inputs();
        if (!unwired.empty()) {
            std::string msg = "Unwired inputs: ";
            for (size_t i = 0; i < unwired.size(); ++i) {
                if (i > 0)
                    msg += ", ";
                msg += unwired[i];
            }
            throw WiringError(msg);
        }
    }

    // =========================================================================
    // Phase 2.4: Introspection
    // =========================================================================

    /**
     * @brief Get all output signal descriptors
     */
    [[nodiscard]] std::vector<SignalDescriptor> get_outputs() const {
        std::vector<SignalDescriptor> result;
        result.reserve(signals_.size());
        for (const auto &desc : signals_) {
            result.push_back(desc);
        }
        return result;
    }

    /**
     * @brief Get output signals for a specific component
     */
    [[nodiscard]] std::vector<SignalDescriptor>
    get_outputs_for_component(const std::string &component_name) const {
        std::vector<SignalDescriptor> result;
        for (const auto &desc : signals_) {
            if (desc.owner_component == component_name) {
                result.push_back(desc);
            }
        }
        return result;
    }

    /**
     * @brief Get inputs for a specific component
     */
    [[nodiscard]] std::vector<SignalDescriptor>
    get_inputs_for_component(const std::string &component_name) const {
        std::vector<SignalDescriptor> result;
        for (const auto &[name, entry] : inputs_) {
            if (entry.info.owner_component == component_name) {
                result.push_back(entry.info);
            }
        }
        return result;
    }

    /**
     * @brief Get parameters for a specific component
     */
    [[nodiscard]] std::vector<SignalDescriptor>
    get_params_for_component(const std::string &component_name) const {
        std::vector<SignalDescriptor> result;
        for (const auto &[name, entry] : params_) {
            if (entry.info.owner_component == component_name) {
                result.push_back(entry.info);
            }
        }
        return result;
    }

    /**
     * @brief Get config for a specific component
     */
    [[nodiscard]] std::vector<SignalDescriptor>
    get_config_for_component(const std::string &component_name) const {
        std::vector<SignalDescriptor> result;
        for (const auto &[name, entry] : config_) {
            if (entry.info.owner_component == component_name) {
                result.push_back(entry.info);
            }
        }
        return result;
    }

    /**
     * @brief Get all input info
     */
    [[nodiscard]] std::vector<SignalDescriptor> get_inputs() const {
        std::vector<SignalDescriptor> result;
        result.reserve(inputs_.size());
        for (const auto &[name, entry] : inputs_) {
            result.push_back(entry.info);
        }
        return result;
    }

    /**
     * @brief Get all parameter info
     */
    [[nodiscard]] std::vector<SignalDescriptor> get_params() const {
        std::vector<SignalDescriptor> result;
        result.reserve(params_.size());
        for (const auto &[name, entry] : params_) {
            result.push_back(entry.info);
        }
        return result;
    }

    /**
     * @brief Get all config info
     */
    [[nodiscard]] std::vector<SignalDescriptor> get_config() const {
        std::vector<SignalDescriptor> result;
        result.reserve(config_.size());
        for (const auto &[name, entry] : config_) {
            result.push_back(entry.info);
        }
        return result;
    }

    /**
     * @brief Check if an input exists
     */
    [[nodiscard]] bool has_input(const std::string &name) const { return inputs_.contains(name); }

    /**
     * @brief Check if a parameter exists
     */
    [[nodiscard]] bool has_param(const std::string &name) const { return params_.contains(name); }

    /**
     * @brief Check if a config exists
     */
    [[nodiscard]] bool has_config(const std::string &name) const { return config_.contains(name); }

    // =========================================================================
    // Phase 3: Signal Discovery (for external bindings)
    // =========================================================================

    /**
     * @brief Get all signal names (outputs)
     */
    [[nodiscard]] std::vector<std::string> get_all_signal_names() const {
        std::vector<std::string> names;
        names.reserve(signals_.size());
        for (const auto &desc : signals_) {
            names.push_back(desc.name);
        }
        return names;
    }

    /**
     * @brief Get all input signal names
     */
    [[nodiscard]] std::vector<std::string> get_all_input_names() const {
        std::vector<std::string> names;
        names.reserve(inputs_.size());
        for (const auto &[name, entry] : inputs_) {
            names.push_back(entry.info.name);
        }
        return names;
    }

    /**
     * @brief Get all output signal names
     */
    [[nodiscard]] std::vector<std::string> get_all_output_names() const {
        return get_all_signal_names(); // Outputs = signals
    }

    // =========================================================================
    // Phase 4.0: API Additions for SignalRouter and Backplane
    // =========================================================================

    /**
     * @brief Check if an output signal exists (PascalCase alias)
     */
    [[nodiscard]] bool HasOutput(const std::string &name) const { return HasSignal(name); }

    /**
     * @brief Check if an input port exists (PascalCase alias)
     */
    [[nodiscard]] bool HasInput(const std::string &name) const { return has_input(name); }

    /**
     * @brief Get all declared input paths (for SignalRouter validation)
     */
    [[nodiscard]] std::vector<std::string> GetAllInputPaths() const {
        return get_all_input_names();
    }

    /**
     * @brief Get all declared output paths (for SignalRouter validation)
     */
    [[nodiscard]] std::vector<std::string> GetAllOutputPaths() const {
        return get_all_signal_names();
    }

    /**
     * @brief Wire an input to a source signal with gain factor
     *
     * The gain is applied when reading the input value.
     * gain = 1.0 means no scaling (pass-through).
     *
     * @param input_name Full name of the input port
     * @param source_name Full name of the source signal
     * @param gain Scale factor (default 1.0)
     */
    template <typename T>
    void wire_input_with_gain(const std::string &input_name, const std::string &source_name,
                              double gain = 1.0) {
        auto it = inputs_.find(input_name);
        if (it == inputs_.end()) {
            throw WiringError("Input not found: '" + input_name + "'");
        }

        // Verify type matches what was registered (prevents undefined behavior)
        const std::string expected_type = typeid(T).name();
        if (it->second.info.semantic != expected_type) {
            throw SignalError::TypeMismatch(input_name, it->second.info.semantic, expected_type);
        }

        // Resolve the source signal
        auto source_handle = resolve<T>(source_name);

        // Wire the input with gain
        auto *handle = static_cast<InputHandle<T> *>(it->second.handle_ptr);
        handle->wire_with_gain(source_handle.ptr(), source_name, gain);
        it->second.info.wired_to = source_name;
    }

    /**
     * @brief Wire an input with gain (type-erased version)
     *
     * Uses the input's registered type information to perform wiring.
     * Only supports Scalar type - gain wiring is not applicable to vector types.
     *
     * @throws WiringError if input type is not Scalar
     */
    void wire_input_with_gain(const std::string &input_name, const std::string &source_name,
                              double gain = 1.0) {
        auto it = inputs_.find(input_name);
        if (it == inputs_.end()) {
            throw WiringError("Input not found: '" + input_name + "'");
        }

        // Verify input is Scalar type - gain wiring only applies to scalars
        const std::string &semantic_type = it->second.info.semantic;
        if (semantic_type != typeid(Scalar).name()) {
            throw WiringError(
                "wire_input_with_gain only supports Scalar type, got: " + semantic_type +
                " (input: '" + input_name + "'). Use wire_input for vector types.");
        }

        wire_input_with_gain<Scalar>(input_name, source_name, gain);
    }

  private:
    template <typename T>
    void register_config_impl(const std::string &name, T *storage, T initial_value,
                              const std::string &type_name, const std::string &description) {
        if (storage == nullptr) {
            throw SignalError::NullPointer(name, "config");
        }

        if (config_.contains(name)) {
            throw DuplicateSignalError(name, config_[name].info.name, current_component_);
        }

        *storage = initial_value;

        ConfigEntry entry;
        entry.storage = storage;
        entry.info.name = name;
        entry.info.description = description;
        entry.info.kind = SignalKind::Config;
        entry.info.semantic = type_name;
        entry.info.is_optimizable = false;
        entry.info.owner_component = current_component_;

        config_[name] = std::move(entry);
    }
};

} // namespace icarus
