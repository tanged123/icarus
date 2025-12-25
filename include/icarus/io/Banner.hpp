#pragma once

/**
 * @file Banner.hpp
 * @brief ASCII art banners and headers
 *
 * Part of Phase 2.5: ASCII-Rich Logging.
 */

#include <string>

namespace icarus {

/**
 * @brief ASCII art banners and headers
 */
class Banner {
  public:
    /// Get the main Icarus splash screen (ASCII art logo)
    [[nodiscard]] static std::string GetSplashScreen(const std::string &build_type = "RELEASE",
                                                     const std::string &version = "0.1.0") {
        // clang-format off
        return R"(
################################################################################
#                                                                              #
#    ██╗ ██████╗ █████╗ ██████╗ ██╗   ██╗███████╗                              #
#    ██║██╔════╝██╔══██╗██╔══██╗██║   ██║██╔════╝                              #
#    ██║██║     ███████║██████╔╝██║   ██║███████╗                              #
#    ██║██║     ██╔══██║██╔══██╗██║   ██║╚════██║                              #
#    ██║╚██████╗██║  ██║██║  ██║╚██████╔╝███████║                              #
#    ╚═╝ ╚═════╝╚═╝  ╚═╝╚═╝  ╚═╝ ╚═════╝ ╚══════╝                              #
#                                                                              #
#    6DOF FLIGHT DYNAMICS ENGINE | BUILD: )" + PadRight(build_type, 7) + " | VER: " + PadRight(version, 7) + R"(               #
################################################################################
)";
        // clang-format on
    }

    /// Get a section header
    [[nodiscard]] static std::string GetPhaseHeader(const std::string &phase_name) {
        std::string header = "─── [ PHASE: " + phase_name + " ] ";
        std::size_t remaining = 80 - header.size();
        for (std::size_t i = 0; i < remaining; ++i) {
            header += "─";
        }
        return header;
    }

    /// Get the mission debrief header
    [[nodiscard]] static std::string GetDebriefHeader() {
        return R"(
================================================================================
  MISSION DEBRIEF
================================================================================)";
    }

    /// Get horizontal rule
    [[nodiscard]] static std::string GetRule(int width = 80, char c = '=') {
        return std::string(static_cast<std::size_t>(width), c);
    }

    /// Get double-line rule
    [[nodiscard]] static std::string GetDoubleRule(int width = 80) {
        std::string rule;
        for (int i = 0; i < width; ++i) {
            rule += "═";
        }
        return rule;
    }

    /// Get section header (for data dictionary, etc.)
    [[nodiscard]] static std::string GetSectionHeader(const std::string &title) {
        std::string header = "─── [ " + title + " ] ";
        std::size_t remaining = 80 - header.size();
        for (std::size_t i = 0; i < remaining; ++i) {
            header += "─";
        }
        return header;
    }

  private:
    [[nodiscard]] static std::string PadRight(const std::string &s, std::size_t width) {
        if (s.size() >= width) {
            return s;
        }
        return s + std::string(width - s.size(), ' ');
    }
};

} // namespace icarus
