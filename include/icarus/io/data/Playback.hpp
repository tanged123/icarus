#pragma once

#include <string>

namespace icarus {

/**
 * @brief Simulation data playback interface.
 *
 * Loads recorded data for analysis or warmstart.
 */
class Playback {
  public:
    virtual ~Playback() = default;

    /**
     * @brief Open recording file for playback.
     */
    virtual void Open(const std::string &path) = 0;

    /**
     * @brief Close playback file.
     */
    virtual void Close() = 0;
};

} // namespace icarus
