#pragma once

/**
 * @file ComponentFactory.hpp
 * @brief Factory for creating components from configuration
 *
 * Part of Phase 4.0: Configuration Infrastructure.
 * Provides a singleton factory with component registration.
 */

#include <icarus/core/Component.hpp>
#include <icarus/core/ComponentConfig.hpp>
#include <icarus/core/Error.hpp>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

namespace icarus {

/**
 * @brief Factory for creating components from configuration
 *
 * Components register themselves using the ICARUS_REGISTER_COMPONENT macro.
 * The factory creates components by type name from ComponentConfig.
 *
 * Example usage:
 * @code
 * auto& factory = ComponentFactory<double>::Instance();
 * auto component = factory.Create(config);
 * @endcode
 */
template <typename Scalar> class ComponentFactory {
  public:
    /// Creator function type: takes name and entity, returns component
    using Creator = std::function<std::unique_ptr<Component<Scalar>>(const std::string &name,
                                                                     const std::string &entity)>;

    /**
     * @brief Register a component type
     *
     * @param type_name Type name used in config files
     * @param creator Function that creates the component
     */
    void Register(const std::string &type_name, Creator creator) {
        creators_[type_name] = std::move(creator);
    }

    /**
     * @brief Create a component from config
     *
     * @param config Component configuration
     * @return Unique pointer to created component
     * @throws ConfigError if type is not registered
     */
    std::unique_ptr<Component<Scalar>> Create(const ComponentConfig &config) {
        auto it = creators_.find(config.type);
        if (it == creators_.end()) {
            throw ConfigError("Unknown component type: '" + config.type + "'");
        }
        return it->second(config.name, config.entity);
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
     * @brief Get singleton instance
     */
    static ComponentFactory &Instance() {
        static ComponentFactory instance;
        return instance;
    }

  private:
    ComponentFactory() = default;
    std::unordered_map<std::string, Creator> creators_;
};

/**
 * @brief Helper macro for component registration
 *
 * Usage in component header or cpp file:
 * @code
 * ICARUS_REGISTER_COMPONENT(MassAggregator)
 * @endcode
 *
 * This registers the component for both double and symbolic backends.
 */
#define ICARUS_REGISTER_COMPONENT(ComponentType)                                                   \
    namespace {                                                                                    \
    static bool _reg_double_##ComponentType = []() {                                               \
        ::icarus::ComponentFactory<double>::Instance().Register(                                   \
            #ComponentType, [](const std::string &name, const std::string &entity) {               \
                return std::make_unique<ComponentType<double>>(name, entity);                      \
            });                                                                                    \
        return true;                                                                               \
    }();                                                                                           \
    }

/**
 * @brief Register component with custom type name
 *
 * Usage:
 * @code
 * ICARUS_REGISTER_COMPONENT_AS(MyCustomComponent, "CustomName")
 * @endcode
 */
#define ICARUS_REGISTER_COMPONENT_AS(ComponentType, TypeName)                                      \
    namespace {                                                                                    \
    static bool _reg_double_##ComponentType = []() {                                               \
        ::icarus::ComponentFactory<double>::Instance().Register(                                   \
            TypeName, [](const std::string &name, const std::string &entity) {                     \
                return std::make_unique<ComponentType<double>>(name, entity);                      \
            });                                                                                    \
        return true;                                                                               \
    }();                                                                                           \
    }

} // namespace icarus
