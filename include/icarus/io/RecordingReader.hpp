#pragma once

/**
 * @file RecordingReader.hpp
 * @brief Re-exports Vulcan's HDF5Reader for Icarus recordings
 *
 * This is a thin re-export of Vulcan's telemetry reader.
 * No Icarus-specific code - just namespace convenience.
 */

#include <vulcan/io/HDF5Reader.hpp>

namespace icarus {

/**
 * @brief HDF5 recording reader (re-export of Vulcan's HDF5Reader)
 *
 * Reads recordings created by Recorder. See vulcan::io::HDF5Reader
 * for full API documentation.
 *
 * Example:
 * @code
 * RecordingReader reader("sim.h5");
 *
 * auto times = reader.times();
 * auto positions = reader.read_vec3("Satellite.position");
 *
 * std::cout << "Frames: " << reader.frame_count() << "\n";
 * @endcode
 */
using RecordingReader = vulcan::io::HDF5Reader;

} // namespace icarus
