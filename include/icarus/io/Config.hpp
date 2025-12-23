#pragma once

#include <string>

namespace icarus {

/**
 * @brief Configuration loader interface.
 *
 * Loads simulation configuration from YAML files.
 */
class ConfigLoader {
  public:
    virtual ~ConfigLoader() = default;

    /**
     * @brief Load configuration from file.
     *
     * @param path Path to configuration file
     */
    virtual void Load(const std::string &path) = 0;
};

} // namespace icarus
