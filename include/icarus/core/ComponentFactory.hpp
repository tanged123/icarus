#pragma once

/**
 * @file ComponentFactory.hpp
 * @brief Factory for creating components from configuration
 *
 * Part of Phase 4.0.7: Configuration Infrastructure.
 * Provides a singleton factory with component registration.
 *
 * Updated to pass full ComponentConfig to creators, enabling components
 * to read their scalars, vectors, and other configuration values.
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/ComponentConfig.hpp>
#include <icarus/core/Error.hpp>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace icarus {

/**
 * @brief Factory for creating components from configuration
 *
 * Components register themselves using the ICARUS_REGISTER_COMPONENT macro.
 * The factory creates components by type name from ComponentConfig.
 *
 * Example usage:
 * @code
 * // In component file - register with factory
 * ICARUS_REGISTER_COMPONENT(PointMass3DOF)
 *
 * // In Simulator - create from config
 * auto& factory = ComponentFactory<double>::Instance();
 * auto component = factory.Create(config);  // config has type, name, scalars, etc.
 * @endcode
 */
template <typename Scalar> class ComponentFactory {
  public:
    /// Creator function type: takes full ComponentConfig, returns component
    using Creator = std::function<std::unique_ptr<Component<Scalar>>(const ComponentConfig &)>;

    /**
     * @brief Register a component type with custom creator
     *
     * @param type_name Type name used in config files
     * @param creator Function that creates the component from config
     */
    void Register(const std::string &type_name, Creator creator) {
        creators_[type_name] = std::move(creator);
    }

    /**
     * @brief Create a component from config
     *
     * The creator receives the full ComponentConfig including:
     * - name: Component instance name
     * - entity: Entity namespace (if any)
     * - scalars: Scalar parameter values
     * - vectors: Vector parameter values
     * - strings: String parameter values
     *
     * @param config Component configuration
     * @return Unique pointer to created component
     * @throws ConfigError if type is not registered
     */
    [[nodiscard]] std::unique_ptr<Component<Scalar>> Create(const ComponentConfig &config) {
        auto it = creators_.find(config.type);
        if (it == creators_.end()) {
            throw ConfigError("Unknown component type: '" + config.type +
                              "'. Registered types: " + ListTypesString());
        }
        return it->second(config);
    }

    /**
     * @brief Check if a type is registered
     */
    [[nodiscard]] bool HasType(const std::string &type_name) const {
        return creators_.count(type_name) > 0;
    }

    /**
     * @brief Get list of registered type names
     */
    [[nodiscard]] std::vector<std::string> GetRegisteredTypes() const {
        std::vector<std::string> types;
        types.reserve(creators_.size());
        for (const auto &pair : creators_) {
            types.push_back(pair.first);
        }
        return types;
    }

    /**
     * @brief Get number of registered types
     */
    [[nodiscard]] std::size_t NumRegistered() const { return creators_.size(); }

    /**
     * @brief Get singleton instance
     */
    static ComponentFactory &Instance() {
        static ComponentFactory instance;
        return instance;
    }

    /**
     * @brief Clear all registrations (for testing)
     */
    void Clear() { creators_.clear(); }

  private:
    ComponentFactory() = default;

    [[nodiscard]] std::string ListTypesString() const {
        std::string result;
        for (const auto &pair : creators_) {
            if (!result.empty())
                result += ", ";
            result += pair.first;
        }
        return result.empty() ? "(none)" : result;
    }

    std::unordered_map<std::string, Creator> creators_;
};

// =============================================================================
// Registration Macros
// =============================================================================

/**
 * @brief Register a component type with the factory
 *
 * The component class must have a constructor that accepts (name, entity):
 *   ComponentType(std::string name, std::string entity)
 *
 * The factory will call SetConfig() after construction. Components read
 * their configuration in Stage() via GetConfig().
 *
 * Usage in component header or cpp file (namespace scope):
 * @code
 * ICARUS_REGISTER_COMPONENT(PointMass3DOF)
 * @endcode
 */
#define ICARUS_REGISTER_COMPONENT(ComponentType)                                                   \
    namespace {                                                                                    \
    static bool _reg_double_##ComponentType = []() {                                               \
        ::icarus::ComponentFactory<double>::Instance().Register(                                   \
            #ComponentType, [](const ::icarus::ComponentConfig &config) {                          \
                auto comp = std::make_unique<ComponentType<double>>(config.name, config.entity);   \
                comp->SetConfig(config);                                                           \
                return comp;                                                                       \
            });                                                                                    \
        return true;                                                                               \
    }();                                                                                           \
    }

/**
 * @brief Register component with custom type name
 *
 * Usage (at namespace scope, inside the component's namespace):
 * @code
 * ICARUS_REGISTER_COMPONENT_AS(PointMass3DOF, "PointMass3DOF")
 * @endcode
 */
#define ICARUS_REGISTER_COMPONENT_IMPL2(ComponentType, TypeName, Counter)                          \
    namespace {                                                                                    \
    static const bool _icarus_reg_##Counter = []() {                                               \
        ::icarus::ComponentFactory<double>::Instance().Register(                                   \
            TypeName, [](const ::icarus::ComponentConfig &config) {                                \
                auto comp = std::make_unique<ComponentType<double>>(config.name, config.entity);   \
                comp->SetConfig(config);                                                           \
                return comp;                                                                       \
            });                                                                                    \
        return true;                                                                               \
    }();                                                                                           \
    }

#define ICARUS_REGISTER_COMPONENT_IMPL(ComponentType, TypeName, Counter)                           \
    ICARUS_REGISTER_COMPONENT_IMPL2(ComponentType, TypeName, Counter)

#define ICARUS_REGISTER_COMPONENT_AS(ComponentType, TypeName)                                      \
    ICARUS_REGISTER_COMPONENT_IMPL(ComponentType, TypeName, __COUNTER__)

/**
 * @brief Register component with custom creator function
 *
 * Use when component needs special construction logic.
 * The creator lambda is responsible for calling SetConfig() on the component.
 *
 * Usage:
 * @code
 * ICARUS_REGISTER_COMPONENT_WITH_CREATOR("CustomType",
 *     [](const ComponentConfig& cfg) {
 *         auto comp = std::make_unique<CustomComponent<double>>(cfg.name, cfg.entity);
 *         comp->SetConfig(cfg);
 *         return comp;
 *     })
 * @endcode
 */
#define ICARUS_REGISTER_COMPONENT_WITH_CREATOR(TypeName, CreatorLambda)                            \
    namespace {                                                                                    \
    static bool _reg_creator_##__LINE__ = []() {                                                   \
        ::icarus::ComponentFactory<double>::Instance().Register(TypeName, CreatorLambda);          \
        return true;                                                                               \
    }();                                                                                           \
    }

} // namespace icarus
