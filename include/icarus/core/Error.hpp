#pragma once

#include <stdexcept>
#include <string>

namespace icarus {

/**
 * @brief Base class for all Icarus errors
 */
class Error : public std::runtime_error {
  public:
    explicit Error(const std::string &msg) : std::runtime_error(msg) {}
};

/**
 * @brief Error during component Provision phase
 */
class ProvisionError : public Error {
  public:
    explicit ProvisionError(const std::string &msg) : Error("Provision error: " + msg) {}
};

/**
 * @brief Error during component Stage phase
 */
class StageError : public Error {
  public:
    explicit StageError(const std::string &msg) : Error("Stage error: " + msg) {}
};

/**
 * @brief Error during component Step phase
 */
class StepError : public Error {
  public:
    explicit StepError(const std::string &msg) : Error("Step error: " + msg) {}
};

/**
 * @brief Signal-related error
 */
class SignalError : public Error {
  public:
    explicit SignalError(const std::string &msg) : Error("Signal error: " + msg) {}
};

/**
 * @brief Configuration error
 */
class ConfigError : public Error {
  public:
    explicit ConfigError(const std::string &msg) : Error("Config error: " + msg) {}
};

} // namespace icarus
