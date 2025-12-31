#pragma once

#include <string>

namespace icarus {

/**
 * @brief Simulation data recorder interface.
 *
 * Records signal values to persistent storage (HDF5).
 */
class Recorder {
  public:
    virtual ~Recorder() = default;

    /**
     * @brief Open recording file.
     */
    virtual void Open(const std::string &path) = 0;

    /**
     * @brief Close recording file.
     */
    virtual void Close() = 0;

    /**
     * @brief Record current signal values.
     */
    virtual void Record(double time) = 0;
};

} // namespace icarus
