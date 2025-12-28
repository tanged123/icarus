#pragma once

/**
 * @file ComponentConfig.hpp
 * @brief Configuration container for components with typed accessors
 *
 * Part of Phase 4.0: Configuration Infrastructure.
 * Components request config values using typed accessors in Provision().
 */

#include <icarus/core/Error.hpp>
#include <icarus/core/Types.hpp>

#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace icarus {

/**
 * @brief Configuration container for components
 *
 * Passed to Provision() and Stage(). Components request values using
 * typed accessors. This is the ONLY way to configure a component
 * beyond the constructor (which only takes name/entity).
 *
 * Example usage in Provision():
 * @code
 * void Provision(Backplane<Scalar>& bp, const ComponentConfig& cfg) override {
 *     mass_ = cfg.Require<double>("mass");  // Throws if missing
 *     cd_ = cfg.Get<double>("drag_coefficient", 0.5);  // With default
 *     position_ = cfg.Get<Vec3<double>>("initial_position", Vec3<double>::Zero());
 * }
 * @endcode
 */
struct ComponentConfig {
    // =========================================================================
    // Identity (set by loader/factory)
    // =========================================================================

    std::string name;   ///< Component instance name
    std::string entity; ///< Entity namespace (optional)
    std::string type;   ///< Component type name

    // =========================================================================
    // Raw Storage (populated by loader)
    // =========================================================================

    std::unordered_map<std::string, double> scalars;
    std::unordered_map<std::string, std::vector<double>> vectors; ///< Vec3 stored as [x,y,z]
    std::unordered_map<std::string, std::vector<double>> arrays;
    std::unordered_map<std::string, std::string> strings;
    std::unordered_map<std::string, int64_t> integers;
    std::unordered_map<std::string, bool> booleans;

    /// List-based config (for aggregators with variable source counts, etc.)
    std::vector<std::string> sources;

    // =========================================================================
    // Typed Accessors
    // =========================================================================

    /**
     * @brief Get a required config value (throws if missing)
     * @tparam T Value type
     * @param key Configuration key
     * @return Value of type T
     * @throws ConfigError if key is missing
     */
    template <typename T> T Require(const std::string &key) const;

    /**
     * @brief Get an optional config value with default
     * @tparam T Value type
     * @param key Configuration key
     * @param default_value Value to return if key is missing
     * @return Value of type T, or default_value if missing
     */
    template <typename T> T Get(const std::string &key, const T &default_value) const;

    /**
     * @brief Check if a key exists
     * @tparam T Value type (determines which storage to check)
     * @param key Configuration key
     * @return True if key exists in appropriate storage
     */
    template <typename T> [[nodiscard]] bool Has(const std::string &key) const;

    /**
     * @brief Get the full component path: entity.name (or just name if no entity)
     */
    [[nodiscard]] std::string FullPath() const {
        if (entity.empty())
            return name;
        return entity + "." + name;
    }
};

// =============================================================================
// Template Specializations - double
// =============================================================================

template <>
inline double ComponentConfig::Get<double>(const std::string &key, const double &def) const {
    auto it = scalars.find(key);
    return (it != scalars.end()) ? it->second : def;
}

template <> inline double ComponentConfig::Require<double>(const std::string &key) const {
    auto it = scalars.find(key);
    if (it == scalars.end()) {
        throw ConfigError("Component '" + FullPath() + "' missing required scalar: " + key);
    }
    return it->second;
}

template <> inline bool ComponentConfig::Has<double>(const std::string &key) const {
    return scalars.count(key) > 0;
}

// =============================================================================
// Template Specializations - int / int64_t
// =============================================================================

template <> inline int ComponentConfig::Get<int>(const std::string &key, const int &def) const {
    auto it = integers.find(key);
    return (it != integers.end()) ? static_cast<int>(it->second) : def;
}

template <> inline int ComponentConfig::Require<int>(const std::string &key) const {
    auto it = integers.find(key);
    if (it == integers.end()) {
        throw ConfigError("Component '" + FullPath() + "' missing required integer: " + key);
    }
    return static_cast<int>(it->second);
}

template <> inline bool ComponentConfig::Has<int>(const std::string &key) const {
    return integers.count(key) > 0;
}

template <>
inline int64_t ComponentConfig::Get<int64_t>(const std::string &key, const int64_t &def) const {
    auto it = integers.find(key);
    return (it != integers.end()) ? it->second : def;
}

template <> inline int64_t ComponentConfig::Require<int64_t>(const std::string &key) const {
    auto it = integers.find(key);
    if (it == integers.end()) {
        throw ConfigError("Component '" + FullPath() + "' missing required integer: " + key);
    }
    return it->second;
}

template <> inline bool ComponentConfig::Has<int64_t>(const std::string &key) const {
    return integers.count(key) > 0;
}

// =============================================================================
// Template Specializations - bool
// =============================================================================

template <> inline bool ComponentConfig::Get<bool>(const std::string &key, const bool &def) const {
    auto it = booleans.find(key);
    return (it != booleans.end()) ? it->second : def;
}

template <>
inline bool ComponentConfig::Require<bool>(
    const std::string &key) const { // NOLINT(readability-identifier-naming)
    auto it = booleans.find(key);
    if (it == booleans.end()) {
        throw ConfigError("Component '" + FullPath() + "' missing required boolean: " + key);
    }
    return it->second;
}

// Note: Has<bool> specialization would conflict with Get<bool>'s return type
// Use Has<int> pattern for bool existence check via integers map or direct booleans.count()

// =============================================================================
// Template Specializations - std::string
// =============================================================================

template <>
inline std::string ComponentConfig::Get<std::string>(const std::string &key,
                                                     const std::string &def) const {
    auto it = strings.find(key);
    return (it != strings.end()) ? it->second : def;
}

template <> inline std::string ComponentConfig::Require<std::string>(const std::string &key) const {
    auto it = strings.find(key);
    if (it == strings.end()) {
        throw ConfigError("Component '" + FullPath() + "' missing required string: " + key);
    }
    return it->second;
}

template <> inline bool ComponentConfig::Has<std::string>(const std::string &key) const {
    return strings.count(key) > 0;
}

// =============================================================================
// Template Specializations - Vec3<double>
// =============================================================================

template <>
inline Vec3<double> ComponentConfig::Get<Vec3<double>>(const std::string &key,
                                                       const Vec3<double> &def) const {
    auto it = vectors.find(key);
    if (it == vectors.end() || it->second.size() != 3) {
        return def;
    }
    return Vec3<double>{it->second[0], it->second[1], it->second[2]};
}

template <>
inline Vec3<double> ComponentConfig::Require<Vec3<double>>(const std::string &key) const {
    auto it = vectors.find(key);
    if (it == vectors.end()) {
        throw ConfigError("Component '" + FullPath() + "' missing required vector: " + key);
    }
    if (it->second.size() != 3) {
        throw ConfigError("Component '" + FullPath() + "' vector '" + key +
                          "' must have exactly 3 elements, got " +
                          std::to_string(it->second.size()));
    }
    return Vec3<double>{it->second[0], it->second[1], it->second[2]};
}

template <> inline bool ComponentConfig::Has<Vec3<double>>(const std::string &key) const {
    auto it = vectors.find(key);
    return it != vectors.end() && it->second.size() == 3;
}

// =============================================================================
// Template Specializations - Vec4<double> (Quaternion stored as [w,x,y,z])
// =============================================================================

template <>
inline Vec4<double> ComponentConfig::Get<Vec4<double>>(const std::string &key,
                                                       const Vec4<double> &def) const {
    auto it = vectors.find(key);
    if (it == vectors.end() || it->second.size() != 4) {
        return def;
    }
    return Vec4<double>{it->second[0], it->second[1], it->second[2], it->second[3]};
}

template <>
inline Vec4<double> ComponentConfig::Require<Vec4<double>>(const std::string &key) const {
    auto it = vectors.find(key);
    if (it == vectors.end()) {
        throw ConfigError("Component '" + FullPath() +
                          "' missing required quaternion/vec4: " + key);
    }
    if (it->second.size() != 4) {
        throw ConfigError("Component '" + FullPath() + "' quaternion/vec4 '" + key +
                          "' must have exactly 4 elements, got " +
                          std::to_string(it->second.size()));
    }
    return Vec4<double>{it->second[0], it->second[1], it->second[2], it->second[3]};
}

template <> inline bool ComponentConfig::Has<Vec4<double>>(const std::string &key) const {
    auto it = vectors.find(key);
    return it != vectors.end() && it->second.size() == 4;
}

// =============================================================================
// Template Specializations - std::vector<double>
// =============================================================================

template <>
inline std::vector<double>
ComponentConfig::Get<std::vector<double>>(const std::string &key,
                                          const std::vector<double> &def) const {
    auto it = arrays.find(key);
    if (it != arrays.end()) {
        return it->second;
    }
    // Also check vectors (arrays and vectors use same underlying storage for flexibility)
    auto vit = vectors.find(key);
    return (vit != vectors.end()) ? vit->second : def;
}

template <>
inline std::vector<double>
ComponentConfig::Require<std::vector<double>>(const std::string &key) const {
    auto it = arrays.find(key);
    if (it != arrays.end()) {
        return it->second;
    }
    auto vit = vectors.find(key);
    if (vit != vectors.end()) {
        return vit->second;
    }
    throw ConfigError("Component '" + FullPath() + "' missing required array: " + key);
}

template <> inline bool ComponentConfig::Has<std::vector<double>>(const std::string &key) const {
    return arrays.count(key) > 0 || vectors.count(key) > 0;
}

// =============================================================================
// Template Specializations - std::vector<std::string>
// =============================================================================

template <>
inline std::vector<std::string>
ComponentConfig::Get<std::vector<std::string>>(const std::string &key,
                                               const std::vector<std::string> &def) const {
    // Special case: "sources" returns the sources member
    if (key == "sources") {
        return sources.empty() ? def : sources;
    }
    return def;
}

template <>
inline std::vector<std::string>
ComponentConfig::Require<std::vector<std::string>>(const std::string &key) const {
    if (key == "sources") {
        if (sources.empty()) {
            throw ConfigError("Component '" + FullPath() + "' missing required sources list");
        }
        return sources;
    }
    throw ConfigError("Component '" + FullPath() + "' missing required string list: " + key);
}

template <>
inline bool ComponentConfig::Has<std::vector<std::string>>(const std::string &key) const {
    if (key == "sources") {
        return !sources.empty();
    }
    return false;
}

} // namespace icarus
